#include "rs485_abs_encoder.h"

#include "main.h"
#include "usart.h"

/*
 * RS485 绝对值编码器驱动：
 * 通过 USART2 使用 Modbus RTU 协议读取/设置绝对值编码器。
 * 当前主要用于 M2 旋转轴，上层通过 Rs485AbsEncoder_ReadM2() 读取当前绝对角度。
 */
#define MODBUS_FUNC_READ_HOLDING 0x03U
#define MODBUS_FUNC_WRITE_SINGLE 0x06U
#define MODBUS_EXCEPTION_MASK 0x80U
#define RS485_ABS_POSITION_REG 0x0000U
#define RS485_ABS_ZERO_REG 0x0008U
#define RS485_ABS_UART_TIMEOUT_MS 80U
#define RS485_ABS_WRITE_TIMEOUT_MS 300U
#define RS485_ABS_ZERO_VERIFY_TOL 4U
#define RS485_ABS_READ_RETRY_COUNT 3U
#define RS485_ABS_READ_RETRY_DELAY_MS 20U

enum {
    RS485_ABS_ERR_ARG = -1,
    RS485_ABS_ERR_TX = -2,
    RS485_ABS_ERR_RX = -3,
    RS485_ABS_ERR_FRAME = -4,
    RS485_ABS_ERR_CRC = -5,
    RS485_ABS_ERR_EXCEPTION = -6,
    RS485_ABS_ERR_BUSY = -7
};

typedef enum {
    RS485_ABS_ASYNC_IDLE = 0,
    RS485_ABS_ASYNC_RX,
    RS485_ABS_ASYNC_RESULT
} Rs485AbsAsyncState_t;

static volatile Rs485AbsAsyncState_t g_async_state;
static volatile uint32_t g_async_started_ms;
static volatile int g_async_result;
static volatile uint16_t g_async_rx_len;
static uint8_t g_async_tx[8];
static uint8_t g_async_rx[7];

static uint16_t modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    /* Modbus RTU CRC16，低字节在前，高字节在后。 */
    for (i = 0U; i < len; i++) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc = (uint16_t)(crc >> 1U);
            }
        }
    }

    return crc;
}

static void append_crc(uint8_t *frame, uint16_t len_without_crc)
{
    uint16_t crc = modbus_crc16(frame, len_without_crc);

    /* Modbus 帧尾 CRC 发送顺序为低字节、高字节。 */
    frame[len_without_crc] = (uint8_t)(crc & 0xFFU);
    frame[len_without_crc + 1U] = (uint8_t)((crc >> 8U) & 0xFFU);
}

static void uart2_drain_rx(void)
{
    uint8_t ch;
    uint8_t guard = 0U;

    /* 发送新请求前清掉串口中可能残留的旧响应字节，最多清 32 字节防止卡死。 */
    while ((guard < 32U) && (HAL_UART_Receive(&huart2, &ch, 1U, 1U) == HAL_OK)) {
        guard++;
    }
}

void Rs485AbsEncoder_Init(void)
{
    /* 初始化阶段先清空 USART2 接收缓存。 */
    uart2_drain_rx();
    g_async_state = RS485_ABS_ASYNC_IDLE;
    g_async_started_ms = 0U;
    g_async_result = 0;
    g_async_rx_len = 0U;
}

int Rs485AbsEncoder_RequestM2Read(void)
{
    if (g_async_state != RS485_ABS_ASYNC_IDLE) {
        return RS485_ABS_ERR_BUSY;
    }
    if (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {
        return RS485_ABS_ERR_BUSY;
    }

    g_async_tx[0] = RS485_ABS_ENCODER_M2_ADDR;
    g_async_tx[1] = MODBUS_FUNC_READ_HOLDING;
    g_async_tx[2] = (uint8_t)((RS485_ABS_POSITION_REG >> 8U) & 0xFFU);
    g_async_tx[3] = (uint8_t)(RS485_ABS_POSITION_REG & 0xFFU);
    g_async_tx[4] = 0x00U;
    g_async_tx[5] = 0x01U;
    append_crc(g_async_tx, 6U);

    uart2_drain_rx();
    if (HAL_UART_Transmit(&huart2,
                          g_async_tx,
                          sizeof(g_async_tx),
                          RS485_ABS_UART_TIMEOUT_MS) != HAL_OK) {
        return RS485_ABS_ERR_TX;
    }

    g_async_rx_len = 0U;
    g_async_result = 0;
    g_async_started_ms = HAL_GetTick();
    g_async_state = RS485_ABS_ASYNC_RX;
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, g_async_rx, sizeof(g_async_rx)) != HAL_OK) {
        g_async_state = RS485_ABS_ASYNC_IDLE;
        return RS485_ABS_ERR_RX;
    }
    return 0;
}

void Rs485AbsEncoder_OnUartRxEvent(uint16_t size)
{
    if (g_async_state == RS485_ABS_ASYNC_RX) {
        g_async_rx_len = size;
        g_async_result = 0;
        g_async_state = RS485_ABS_ASYNC_RESULT;
    }
}

void Rs485AbsEncoder_OnUartError(void)
{
    if (g_async_state == RS485_ABS_ASYNC_RX) {
        g_async_rx_len = 0U;
        g_async_result = RS485_ABS_ERR_RX;
        g_async_state = RS485_ABS_ASYNC_RESULT;
    }
}

void Rs485AbsEncoder_Service(void)
{
    if ((g_async_state == RS485_ABS_ASYNC_RX) &&
        ((uint32_t)(HAL_GetTick() - g_async_started_ms) >= RS485_ABS_UART_TIMEOUT_MS)) {
        (void)HAL_UART_AbortReceive(&huart2);
        g_async_rx_len = 0U;
        g_async_result = RS485_ABS_ERR_RX;
        g_async_state = RS485_ABS_ASYNC_RESULT;
    }
}

void Rs485AbsEncoder_CancelM2Read(void)
{
    if (g_async_state == RS485_ABS_ASYNC_RX) {
        (void)HAL_UART_AbortReceive(&huart2);
    }
    g_async_rx_len = 0U;
    g_async_result = 0;
    g_async_state = RS485_ABS_ASYNC_IDLE;
}

int Rs485AbsEncoder_PollM2Read(Rs485AbsEncoderStatus_t *out)
{
    uint16_t crc_rx;
    uint16_t crc_calc;
    uint16_t raw;
    int result;

    if (out == 0) {
        return RS485_ABS_ERR_ARG;
    }
    if (g_async_state != RS485_ABS_ASYNC_RESULT) {
        return 0;
    }

    result = g_async_result;
    if (result == 0) {
        if ((g_async_rx_len == 5U) &&
            (g_async_rx[0] == RS485_ABS_ENCODER_M2_ADDR) &&
            ((g_async_rx[1] & MODBUS_EXCEPTION_MASK) != 0U)) {
            crc_rx = (uint16_t)g_async_rx[3] | ((uint16_t)g_async_rx[4] << 8U);
            crc_calc = modbus_crc16(g_async_rx, 3U);
            result = (crc_rx == crc_calc) ? RS485_ABS_ERR_EXCEPTION : RS485_ABS_ERR_CRC;
        } else if ((g_async_rx_len != 7U) ||
                   (g_async_rx[0] != RS485_ABS_ENCODER_M2_ADDR) ||
                   (g_async_rx[1] != MODBUS_FUNC_READ_HOLDING) ||
                   (g_async_rx[2] != 0x02U)) {
            result = RS485_ABS_ERR_FRAME;
        } else {
            crc_rx = (uint16_t)g_async_rx[5] | ((uint16_t)g_async_rx[6] << 8U);
            crc_calc = modbus_crc16(g_async_rx, 5U);
            if (crc_rx != crc_calc) {
                result = RS485_ABS_ERR_CRC;
            } else {
                raw = (uint16_t)(((uint16_t)g_async_rx[3] << 8U) | g_async_rx[4]);
                raw = (uint16_t)(raw % RS485_ABS_ENCODER_CPR);
                out->addr = RS485_ABS_ENCODER_M2_ADDR;
                out->raw = raw;
                out->deg = ((float)raw * 360.0f) / (float)RS485_ABS_ENCODER_CPR;
                out->tick_ms = HAL_GetTick();
            }
        }
    }

    g_async_rx_len = 0U;
    g_async_state = RS485_ABS_ASYNC_IDLE;
    return (result == 0) ? 1 : result;
}

static int rs485_abs_encoder_read_once(uint8_t addr, Rs485AbsEncoderStatus_t *out)
{
    uint8_t tx[8];
    uint8_t rx[7];
    uint8_t extra_len;
    uint16_t crc_rx;
    uint16_t crc_calc;
    uint16_t raw;

    /*
     * 读一次绝对值位置寄存器：
     * 请求：addr 0x03 reg_hi reg_lo 0x00 0x01 CRC_L CRC_H
     * 正常响应：addr 0x03 0x02 data_hi data_lo CRC_L CRC_H
     */
    if ((addr == 0U) || (out == 0)) {
        return RS485_ABS_ERR_ARG;
    }

    tx[0] = addr;
    tx[1] = MODBUS_FUNC_READ_HOLDING;
    tx[2] = (uint8_t)((RS485_ABS_POSITION_REG >> 8U) & 0xFFU);
    tx[3] = (uint8_t)(RS485_ABS_POSITION_REG & 0xFFU);
    tx[4] = 0x00U;
    tx[5] = 0x01U;
    append_crc(tx, 6U);

    uart2_drain_rx();

    if (HAL_UART_Transmit(&huart2, tx, sizeof(tx), RS485_ABS_UART_TIMEOUT_MS) != HAL_OK) {
        return RS485_ABS_ERR_TX;
    }

    if (HAL_UART_Receive(&huart2, rx, 3U, RS485_ABS_UART_TIMEOUT_MS) != HAL_OK) {
        return RS485_ABS_ERR_RX;
    }

    if (rx[0] != addr) {
        return RS485_ABS_ERR_FRAME;
    }
    /* 功能码最高位为 1 表示 Modbus 异常响应。 */
    if ((rx[1] & MODBUS_EXCEPTION_MASK) != 0U) {
        if (HAL_UART_Receive(&huart2, &rx[3], 2U, RS485_ABS_UART_TIMEOUT_MS) != HAL_OK) {
            return RS485_ABS_ERR_RX;
        }
        crc_rx = (uint16_t)rx[3] | ((uint16_t)rx[4] << 8U);
        crc_calc = modbus_crc16(rx, 3U);
        if (crc_rx != crc_calc) {
            return RS485_ABS_ERR_CRC;
        }
        return RS485_ABS_ERR_EXCEPTION;
    }
    if ((rx[1] != MODBUS_FUNC_READ_HOLDING) || (rx[2] != 0x02U)) {
        return RS485_ABS_ERR_FRAME;
    }

    extra_len = (uint8_t)(rx[2] + 2U);
    if (extra_len != 4U) {
        return RS485_ABS_ERR_FRAME;
    }
    if (HAL_UART_Receive(&huart2, &rx[3], extra_len, RS485_ABS_UART_TIMEOUT_MS) != HAL_OK) {
        return RS485_ABS_ERR_RX;
    }

    crc_rx = (uint16_t)rx[5] | ((uint16_t)rx[6] << 8U);
    crc_calc = modbus_crc16(rx, 5U);
    if (crc_rx != crc_calc) {
        return RS485_ABS_ERR_CRC;
    }

    /* 原始位置按编码器一圈计数取模，再换算成 0~360 度。 */
    raw = (uint16_t)(((uint16_t)rx[3] << 8U) | rx[4]);
    raw = (uint16_t)(raw % RS485_ABS_ENCODER_CPR);

    out->addr = addr;
    out->raw = raw;
    out->deg = ((float)raw * 360.0f) / (float)RS485_ABS_ENCODER_CPR;
    out->tick_ms = HAL_GetTick();
    return 0;
}

int Rs485AbsEncoder_Read(uint8_t addr, Rs485AbsEncoderStatus_t *out)
{
    int rc = RS485_ABS_ERR_RX;
    uint8_t attempt;

    /* 读位置带重试：只对接收超时和帧格式异常重试，CRC/异常响应直接返回。 */
    if ((addr == 0U) || (out == 0)) {
        return RS485_ABS_ERR_ARG;
    }
    if (g_async_state != RS485_ABS_ASYNC_IDLE) {
        return RS485_ABS_ERR_BUSY;
    }

    for (attempt = 0U; attempt < RS485_ABS_READ_RETRY_COUNT; attempt++) {
        rc = rs485_abs_encoder_read_once(addr, out);
        if (rc == 0) {
            return 0;
        }
        if ((rc != RS485_ABS_ERR_RX) && (rc != RS485_ABS_ERR_FRAME)) {
            return rc;
        }
        HAL_Delay(RS485_ABS_READ_RETRY_DELAY_MS);
    }

    return rc;
}

int Rs485AbsEncoder_ReadM2(Rs485AbsEncoderStatus_t *out)
{
    /* 读取当前项目固定的 M2 绝对值编码器地址。 */
    return Rs485AbsEncoder_Read(RS485_ABS_ENCODER_M2_ADDR, out);
}

int Rs485AbsEncoder_SetCurrentZero(uint8_t addr)
{
    uint8_t tx[8];
    uint8_t rx[8];
    Rs485AbsEncoderStatus_t verify;
    uint16_t crc_rx;
    uint16_t crc_calc;
    uint16_t i;
    int rc;

    /*
     * 设置绝对值编码器当前点为零点。
     * 使用写单寄存器 0x06，写入 RS485_ABS_ZERO_REG。
     */
    if (addr == 0U) {
        return RS485_ABS_ERR_ARG;
    }
    if (g_async_state != RS485_ABS_ASYNC_IDLE) {
        return RS485_ABS_ERR_BUSY;
    }

    tx[0] = addr;
    tx[1] = MODBUS_FUNC_WRITE_SINGLE;
    tx[2] = (uint8_t)((RS485_ABS_ZERO_REG >> 8U) & 0xFFU);
    tx[3] = (uint8_t)(RS485_ABS_ZERO_REG & 0xFFU);
    tx[4] = 0x00U;
    tx[5] = 0x01U;
    append_crc(tx, 6U);

    uart2_drain_rx();

    if (HAL_UART_Transmit(&huart2, tx, sizeof(tx), RS485_ABS_UART_TIMEOUT_MS) != HAL_OK) {
        return RS485_ABS_ERR_TX;
    }

    /*
     * 有些编码器写零点后可能重启或响应慢。
     * 如果写响应超时，则延迟后读回位置，只要接近 0 点也认为写零成功。
     */
    if (HAL_UART_Receive(&huart2, rx, sizeof(rx), RS485_ABS_WRITE_TIMEOUT_MS) != HAL_OK) {
        HAL_Delay(80U);
        rc = Rs485AbsEncoder_Read(addr, &verify);
        if ((rc == 0) &&
            ((verify.raw <= RS485_ABS_ZERO_VERIFY_TOL) ||
             (verify.raw >= (RS485_ABS_ENCODER_CPR - RS485_ABS_ZERO_VERIFY_TOL)))) {
            return 0;
        }
        return RS485_ABS_ERR_RX;
    }

    crc_rx = (uint16_t)rx[6] | ((uint16_t)rx[7] << 8U);
    crc_calc = modbus_crc16(rx, 6U);
    if (crc_rx != crc_calc) {
        return RS485_ABS_ERR_CRC;
    }

    if (rx[0] != addr) {
        return RS485_ABS_ERR_FRAME;
    }
    if ((rx[1] & MODBUS_EXCEPTION_MASK) != 0U) {
        return RS485_ABS_ERR_EXCEPTION;
    }
    if (rx[1] != MODBUS_FUNC_WRITE_SINGLE) {
        return RS485_ABS_ERR_FRAME;
    }
    for (i = 2U; i < 6U; i++) {
        if (rx[i] != tx[i]) {
            return RS485_ABS_ERR_FRAME;
        }
    }

    return 0;
}

int Rs485AbsEncoder_SetM2CurrentZero(void)
{
    /* 将 M2 绝对值编码器当前位置设置为零点。 */
    return Rs485AbsEncoder_SetCurrentZero(RS485_ABS_ENCODER_M2_ADDR);
}

const char *Rs485AbsEncoder_ErrorText(int err)
{
    /* 错误码转文本，方便串口命令返回可读诊断信息。 */
    switch (err) {
        case 0: return "OK";
        case RS485_ABS_ERR_ARG: return "ARG";
        case RS485_ABS_ERR_TX: return "TX";
        case RS485_ABS_ERR_RX: return "RX_TIMEOUT";
        case RS485_ABS_ERR_FRAME: return "FRAME";
        case RS485_ABS_ERR_CRC: return "CRC";
        case RS485_ABS_ERR_EXCEPTION: return "EXCEPTION";
        case RS485_ABS_ERR_BUSY: return "BUSY";
        default: return "UNKNOWN";
    }
}
