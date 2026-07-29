#include "uart_console.h"

#include <stdio.h>
#include <string.h>
#include "laser_range.h"
#include "laser_range2.h"
#include "rs485_abs_encoder.h"
#include "usart.h"

#define UART_CONSOLE_TX_STALL_TIMEOUT_MS 500U

/*
 * USART3 控制台/ESP32 通信模块：
 * 1. 支持普通 ASCII 文本命令，以换行或空闲中断作为一条命令结束；
 * 2. 支持二进制帧，帧格式为 A5 CMD LEN DATA CRC 5A；
 * 3. 支持异步发送队列，避免业务代码长时间阻塞在串口发送上。
 */
static uint8_t g_rx_dma_buf[UART_CONSOLE_RX_DMA_SIZE];
/* 文本命令接收缓存：work_line 正在接收，ready_line 等待主循环取走。 */
static volatile uint16_t g_line_len = 0U;
static volatile uint8_t g_line_ready = 0U;
static char g_work_line[UART_CONSOLE_LINE_MAX];
static char g_ready_line[UART_CONSOLE_LINE_MAX];
/* 二进制帧接收缓存：frame_work 正在拼帧，ready_frame 等待主循环取走。 */
static uint8_t g_frame_work[UART_CONSOLE_FRAME_MAX];
static volatile uint16_t g_frame_len = 0U;
static volatile uint8_t g_frame_ready = 0U;
static UartConsoleFrame_t g_ready_frame;

/* 发送队列采用环形缓冲区，队列元素保存一段待发送字节。 */
static uint8_t g_tx_queue[UART_CONSOLE_TX_QUEUE_DEPTH][UART_CONSOLE_TX_MSG_MAX];
static uint16_t g_tx_len[UART_CONSOLE_TX_QUEUE_DEPTH];
static volatile uint16_t g_tx_head = 0U;
static volatile uint16_t g_tx_tail = 0U;
static volatile uint16_t g_tx_count = 0U;
static volatile uint8_t g_tx_busy = 0U;

static uint8_t g_tx_active[UART_CONSOLE_TX_MSG_MAX];
static uint16_t g_tx_active_len = 0U;
static volatile uint32_t g_tx_started_ms = 0U;
/* 文本输出开关：关闭时普通文本队列接口直接返回成功，二进制字节发送不受影响。 */
static volatile uint8_t g_text_output_enabled = 0U;

static uint16_t bounded_strlen_u16(const char *text, uint16_t max_len)
{
    uint16_t len = 0U;

    /* 限长 strlen，避免 ready_line 异常缺少 '\0' 时越界扫描。 */
    if (text == 0) {
        return 0U;
    }

    while ((len < max_len) && (text[len] != '\0')) {
        len++;
    }

    return len;
}

static bool uart_console_send_blocking(const char *text, uint16_t len)
{
    /* 阻塞发送保留给兼容或调试场景，主要发送路径使用异步队列。 */
    if ((text == 0) || (len == 0U)) {
        return false;
    }

    if (HAL_UART_Transmit(&huart3, (uint8_t *)text, len, 100U) != HAL_OK) {
        return false;
    }

    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    return true;
}

static void uart_console_restart_dma(void)
{
    /* 重新启动 USART3 接收到空闲中断，配合 HAL_UARTEx_RxEventCallback() 收包。 */
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart3, g_rx_dma_buf, UART_CONSOLE_RX_DMA_SIZE);
}

static void uart_console_kick_tx_locked(void)
{
    HAL_StatusTypeDef rc;

    /*
     * 在关中断保护区内启动下一段发送。
     * g_tx_active 保存正在发送的数据，防止发送期间队列槽被覆盖。
     */
    if ((g_tx_busy != 0U) || (g_tx_count == 0U)) {
        return;
    }

    g_tx_active_len = g_tx_len[g_tx_head];
    memcpy(g_tx_active, g_tx_queue[g_tx_head], g_tx_active_len);
    g_tx_head = (uint16_t)((g_tx_head + 1U) % UART_CONSOLE_TX_QUEUE_DEPTH);
    g_tx_count--;

    rc = HAL_UART_Transmit_IT(&huart3, g_tx_active, g_tx_active_len);
    if (rc == HAL_OK) {
        g_tx_busy = 1U;
        g_tx_started_ms = HAL_GetTick();
        return;
    }

    g_tx_busy = 0U;
    g_tx_active_len = 0U;
    g_tx_started_ms = 0U;
}

static bool uart_console_queue_locked(const uint8_t *data,
                                      uint16_t len,
                                      bool drop_oldest,
                                      bool *dropped_oldest)
{
    /*
     * 把数据压入发送队列。
     * 调用方必须已经关中断，确保 head/tail/count 不会被发送完成中断同时修改。
     */
    if (dropped_oldest != 0) {
        *dropped_oldest = false;
    }

    /* 队列满时，如果允许 drop_oldest，就丢弃最旧的一条消息给新消息腾位置。 */
    if (g_tx_count >= UART_CONSOLE_TX_QUEUE_DEPTH) {
        if (!drop_oldest) {
            return false;
        }

        g_tx_head = (uint16_t)((g_tx_head + 1U) % UART_CONSOLE_TX_QUEUE_DEPTH);
        g_tx_count--;

        if (dropped_oldest != 0) {
            *dropped_oldest = true;
        }
    }

    memcpy(g_tx_queue[g_tx_tail], data, len);
    if (len < UART_CONSOLE_TX_MSG_MAX) {
        g_tx_queue[g_tx_tail][len] = 0U;
    }
    g_tx_len[g_tx_tail] = len;
    g_tx_tail = (uint16_t)((g_tx_tail + 1U) % UART_CONSOLE_TX_QUEUE_DEPTH);
    g_tx_count++;

    uart_console_kick_tx_locked();
    return true;
}

static bool uart_console_queue_async(const uint8_t *data,
                                     uint16_t len,
                                     bool drop_oldest,
                                     bool *dropped_oldest)
{
    uint32_t primask;
    bool ret;

    /* 外层异步入队接口，统一负责参数校验和临界区保护。 */
    if (dropped_oldest != 0) {
        *dropped_oldest = false;
    }
    if ((data == 0) || (len == 0U) || (len >= UART_CONSOLE_TX_MSG_MAX)) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    ret = uart_console_queue_locked(data, len, drop_oldest, dropped_oldest);
    if (primask == 0U) {
        __enable_irq();
    }

    return ret;
}

void UartConsole_Init(void)
{
    /* 清空接收/发送状态，并启动 USART3 接收到空闲中断。 */
    memset(g_rx_dma_buf, 0, sizeof(g_rx_dma_buf));
    memset(g_work_line, 0, sizeof(g_work_line));
    memset(g_ready_line, 0, sizeof(g_ready_line));
    memset(g_frame_work, 0, sizeof(g_frame_work));
    memset(&g_ready_frame, 0, sizeof(g_ready_frame));
    memset(g_tx_queue, 0, sizeof(g_tx_queue));
    memset(g_tx_len, 0, sizeof(g_tx_len));
    memset(g_tx_active, 0, sizeof(g_tx_active));

    g_line_len = 0U;
    g_line_ready = 0U;
    g_frame_len = 0U;
    g_frame_ready = 0U;

    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_count = 0U;
    g_tx_busy = 0U;
    g_tx_active_len = 0U;
    g_tx_started_ms = 0U;
    g_text_output_enabled = 0U;

    uart_console_restart_dma();
}

void UartConsole_SetTextOutputEnabled(bool enable)
{
    /* 控制文本调试输出是否真正进入发送队列。 */
    g_text_output_enabled = enable ? 1U : 0U;
}

bool UartConsole_GetTextOutputEnabled(void)
{
    return (g_text_output_enabled != 0U);
}

bool UartConsole_IsTxIdle(void)
{
    uint32_t primask;
    bool idle;

    primask = __get_PRIMASK();
    __disable_irq();
    idle = ((g_tx_busy == 0U) && (g_tx_count == 0U));
    if (primask == 0U) {
        __enable_irq();
    }

    return idle;
}

void UartConsole_Tick(void)
{
    uint32_t primask;
    uint8_t tx_stalled = 0U;

    /*
     * 发送保护：
     * 如果中断发送长时间没有完成，主动 Abort 并尝试发送队列中的下一条，
     * 防止一次异常发送导致整个输出队列永久卡住。
     */
    if ((g_tx_busy != 0U) &&
        (g_tx_started_ms != 0U) &&
        ((HAL_GetTick() - g_tx_started_ms) > UART_CONSOLE_TX_STALL_TIMEOUT_MS)) {
        (void)HAL_UART_AbortTransmit(&huart3);
        tx_stalled = 1U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (tx_stalled != 0U) {
        g_tx_busy = 0U;
        g_tx_active_len = 0U;
        g_tx_started_ms = 0U;
    }
    uart_console_kick_tx_locked();
    if (primask == 0U) {
        __enable_irq();
    }
}

bool UartConsole_PollLine(char *line_out, uint16_t max_len)
{
    uint32_t primask;
    uint16_t copy_len;

    /* 主循环轮询一条已经接收完成的文本命令。 */
    if ((line_out == 0) || (max_len == 0U)) {
        return false;
    }

    if (g_line_ready == 0U) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    copy_len = bounded_strlen_u16(g_ready_line, UART_CONSOLE_LINE_MAX);
    if (copy_len >= max_len) {
        copy_len = max_len - 1U;
    }
    memcpy(line_out, g_ready_line, copy_len);
    line_out[copy_len] = '\0';
    g_ready_line[0] = '\0';
    g_line_ready = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    return true;
}

bool UartConsole_PollFrame(UartConsoleFrame_t *frame_out)
{
    uint32_t primask;

    /* 主循环轮询一帧已经校验通过的二进制协议帧。 */
    if (frame_out == 0) {
        return false;
    }

    if (g_frame_ready == 0U) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(frame_out, &g_ready_frame, sizeof(*frame_out));
    memset(&g_ready_frame, 0, sizeof(g_ready_frame));
    g_frame_ready = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    return true;
}

bool UartConsole_Queue(const char *text)
{
    uint16_t len;

    /* 普通文本入队；文本输出关闭时直接忽略但返回成功。 */
    if (g_text_output_enabled == 0U) {
        return true;
    }

    if (text == 0) {
        return false;
    }

    len = (uint16_t)strlen(text);
    if (len == 0U) {
        return true;
    }
    if (len >= UART_CONSOLE_TX_MSG_MAX) {
        return false;
    }

    return uart_console_queue_async((const uint8_t *)text, len, false, 0);
}

bool UartConsole_QueueDropOldest(const char *text, bool *dropped_oldest)
{
    uint16_t len;

    /* 文本入队，队列满时允许丢弃最旧消息，适合心跳和曲线遥测。 */
    if (dropped_oldest != 0) {
        *dropped_oldest = false;
    }
    if (g_text_output_enabled == 0U) {
        return true;
    }

    if (text == 0) {
        return false;
    }

    len = (uint16_t)strlen(text);
    if (len == 0U) {
        return true;
    }
    if (len >= UART_CONSOLE_TX_MSG_MAX) {
        return false;
    }

    return uart_console_queue_async((const uint8_t *)text, len, true, dropped_oldest);
}

bool UartConsole_QueueLine(const char *text)
{
    uint16_t len;
    uint32_t primask;
    bool ret;

    /* 入队一行文本，正文和 CRLF 分两段入队，因此需要至少两个队列空位。 */
    if (g_text_output_enabled == 0U) {
        return true;
    }

    if (text == 0) {
        return false;
    }

    len = (uint16_t)strlen(text);
    if (len >= UART_CONSOLE_TX_MSG_MAX) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((uint16_t)(UART_CONSOLE_TX_QUEUE_DEPTH - g_tx_count) < 2U) {
        ret = false;
    } else {
        ret = true;
        if (len > 0U) {
            ret = uart_console_queue_locked((const uint8_t *)text, len, false, 0);
        }
        if (ret) {
            ret = uart_console_queue_locked((const uint8_t *)"\r\n", 2U, false, 0);
        }
    }
    if (primask == 0U) {
        __enable_irq();
    }

    return ret;
}

bool UartConsole_QueueLineDropOldest(const char *text, bool *dropped_oldest)
{
    uint16_t len;
    uint32_t primask;
    bool ret = true;
    bool dropped_body = false;
    bool dropped_crlf = false;

    /* 入队一行文本，队列满时允许分别为正文和 CRLF 丢弃旧消息。 */
    if (dropped_oldest != 0) {
        *dropped_oldest = false;
    }
    if (g_text_output_enabled == 0U) {
        return true;
    }

    if (text == 0) {
        return false;
    }

    len = (uint16_t)strlen(text);
    if (len >= UART_CONSOLE_TX_MSG_MAX) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (len > 0U) {
        ret = uart_console_queue_locked((const uint8_t *)text, len, true, &dropped_body);
    }
    if (ret) {
        ret = uart_console_queue_locked((const uint8_t *)"\r\n", 2U, true, &dropped_crlf);
    }
    if (primask == 0U) {
        __enable_irq();
    }

    if (dropped_oldest != 0) {
        *dropped_oldest = (dropped_body || dropped_crlf);
    }

    return ret;
}

bool UartConsole_QueueBytes(const uint8_t *data, uint16_t len)
{
    /*
     * 原始字节发送接口，用于 STM32 给 ESP32 返回二进制帧。
     * 这个接口不受 g_text_output_enabled 控制。
     */
    if ((data == 0) || (len == 0U) || (len >= UART_CONSOLE_TX_MSG_MAX)) {
        return false;
    }

    return uart_console_queue_async(data, len, true, 0);
}

void UartConsole_Send(const char *text)
{
    (void)UartConsole_Queue(text);
}

void UartConsole_SendLine(const char *text)
{
    (void)UartConsole_QueueLine(text);
}

static uint8_t uart_console_frame_crc(uint8_t cmd, uint8_t len, const uint8_t *data)
{
    /*
     * 自定义二进制帧 CRC：
     * CRC = CMD ^ LEN ^ DATA[0] ^ DATA[1] ...
     * 和 ESP32 工程中的 UART 帧解析保持一致。
     */
    uint8_t crc = (uint8_t)(cmd ^ len);
    uint8_t i;

    for (i = 0U; i < len; i++) {
        crc ^= data[i];
    }

    return crc;
}

static void uart_console_frame_reset(void)
{
    /* 丢弃当前正在拼接的二进制帧。 */
    g_frame_len = 0U;
    memset(g_frame_work, 0, sizeof(g_frame_work));
}

static uint8_t uart_console_process_frame_byte(uint8_t byte)
{
    uint16_t expected_len;
    uint8_t payload_len;
    uint8_t crc;

    /*
     * 逐字节解析自定义二进制帧。
     * 返回 1 表示该字节已经被二进制帧解析器消费；
     * 返回 0 表示该字节不是帧内容，可以继续作为 ASCII 文本命令处理。
     */
    if (g_frame_len == 0U) {
        if (byte != UART_CONSOLE_FRAME_SOF) {
            return 0U;
        }
        g_frame_work[g_frame_len++] = byte;
        return 1U;
    }

    if (g_frame_len >= UART_CONSOLE_FRAME_MAX) {
        uart_console_frame_reset();
        if (byte == UART_CONSOLE_FRAME_SOF) {
            g_frame_work[g_frame_len++] = byte;
        }
        return 1U;
    }

    g_frame_work[g_frame_len++] = byte;

    /* 第 3 个字节是 LEN，超过最大载荷长度时立即丢弃当前帧。 */
    if ((g_frame_len == 3U) && (g_frame_work[2] > UART_CONSOLE_FRAME_DATA_MAX)) {
        uart_console_frame_reset();
        return 1U;
    }

    if (g_frame_len < 3U) {
        return 1U;
    }

    payload_len = g_frame_work[2];
    expected_len = (uint16_t)payload_len + 5U;
    if (g_frame_len < expected_len) {
        return 1U;
    }

    /* 帧尾必须是 0x5A，否则认为当前帧无效并重新寻找帧头。 */
    if (g_frame_work[expected_len - 1U] != UART_CONSOLE_FRAME_EOF) {
        uart_console_frame_reset();
        if (byte == UART_CONSOLE_FRAME_SOF) {
            g_frame_work[g_frame_len++] = byte;
        }
        return 1U;
    }

    crc = uart_console_frame_crc(g_frame_work[1], payload_len, &g_frame_work[3]);
    /* CRC 正确且上一帧已被主循环取走时，保存为 ready_frame。 */
    if (crc == g_frame_work[expected_len - 2U]) {
        if (g_frame_ready == 0U) {
            g_ready_frame.cmd = g_frame_work[1];
            g_ready_frame.len = payload_len;
            if (payload_len > 0U) {
                memcpy(g_ready_frame.data, &g_frame_work[3], payload_len);
            }
            g_frame_ready = 1U;
        }
    }

    uart_console_frame_reset();
    return 1U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t i;
    uint8_t got_line_ending = 0U;

    /*
     * USART3 接收到空闲事件后的统一入口。
     * 收到的字节先尝试按二进制帧解析；如果不是帧内容，再按文本命令累积。
     */
    if (huart->Instance == USART1) {
        LaserRange_OnUartRxEvent(size);
        return;
    }
    if (huart->Instance == USART2) {
        Rs485AbsEncoder_OnUartRxEvent(size);
        return;
    }
    if (huart->Instance == UART4) {
        LaserRange2_OnUartRxEvent(size);
        return;
    }
    if (huart->Instance != USART3) {
        return;
    }

    if (size > 0U) {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }

    for (i = 0U; i < size; i++) {
        uint8_t byte = g_rx_dma_buf[i];
        char ch;

        /* 二进制帧字节不参与文本命令拼接。 */
        if (uart_console_process_frame_byte(byte) != 0U) {
            continue;
        }

        ch = (char)byte;

        /* CR/LF 表示一条 ASCII 文本命令结束。 */
        if ((ch == '\r') || (ch == '\n')) {
            if ((g_line_len > 0U) && (g_line_ready == 0U)) {
                g_work_line[g_line_len] = '\0';
                strncpy(g_ready_line, g_work_line, UART_CONSOLE_LINE_MAX - 1U);
                g_ready_line[UART_CONSOLE_LINE_MAX - 1U] = '\0';
                g_line_ready = 1U;
                got_line_ending = 1U;
            }
            g_line_len = 0U;
            continue;
        }

        if (ch == '\0') {
            continue;
        }

        /* 上一条命令还没被主循环取走时，暂时丢弃后续文本，避免覆盖 ready_line。 */
        if (g_line_ready != 0U) {
            continue;
        }

        if (g_line_len < (UART_CONSOLE_LINE_MAX - 1U)) {
            g_work_line[g_line_len++] = ch;
        } else {
            g_line_len = 0U;
        }
    }

    /*
     * ReceiveToIdle 可能在没有换行符时因为总线空闲触发。
     * 如果当前已经积累了文本内容，也把它作为一条命令交给主循环。
     */
    if ((size > 0U) &&
        (size < UART_CONSOLE_RX_DMA_SIZE) &&
        (got_line_ending == 0U) &&
        (g_line_ready == 0U) &&
        (g_line_len > 0U)) {
        g_work_line[g_line_len] = '\0';
        strncpy(g_ready_line, g_work_line, UART_CONSOLE_LINE_MAX - 1U);
        g_ready_line[UART_CONSOLE_LINE_MAX - 1U] = '\0';
        g_line_ready = 1U;
        g_line_len = 0U;
    }

    uart_console_restart_dma();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if (huart->Instance == USART1) {
        LaserRange_OnUartTxComplete();
        return;
    }
    if (huart->Instance == UART4) {
        LaserRange2_OnUartTxComplete();
        return;
    }

    /* USART3 异步发送完成后，清 busy 标志并启动队列中的下一段数据。 */
    if (huart->Instance != USART3) {
        return;
    }

    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

    primask = __get_PRIMASK();
    __disable_irq();

    g_tx_busy = 0U;
    g_tx_active_len = 0U;
    g_tx_started_ms = 0U;
    uart_console_kick_tx_locked();

    if (primask == 0U) {
        __enable_irq();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if (huart->Instance == USART1) {
        LaserRange_OnUartError();
        return;
    }
    if (huart->Instance == USART2) {
        Rs485AbsEncoder_OnUartError();
        return;
    }
    if (huart->Instance == UART4) {
        LaserRange2_OnUartError();
        return;
    }

    /* USART3 出错时重启接收，并恢复发送队列，避免串口错误后通信停止。 */
    if (huart->Instance != USART3) {
        return;
    }

    uart_console_restart_dma();

    primask = __get_PRIMASK();
    __disable_irq();

    g_tx_busy = 0U;
    g_tx_active_len = 0U;
    g_tx_started_ms = 0U;
    uart_console_kick_tx_locked();

    if (primask == 0U) {
        __enable_irq();
    }
}
