#ifndef UART_CONSOLE_H
#define UART_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* USART3 接收/发送缓存配置。 */
#define UART_CONSOLE_RX_DMA_SIZE 256U
#define UART_CONSOLE_LINE_MAX 128U
#define UART_CONSOLE_TX_QUEUE_DEPTH 24U
#define UART_CONSOLE_TX_MSG_MAX 768U
/* STM32 与 ESP32 的二进制帧格式：A5 CMD LEN DATA CRC 5A。 */
#define UART_CONSOLE_FRAME_SOF 0xA5U
#define UART_CONSOLE_FRAME_EOF 0x5AU
#define UART_CONSOLE_FRAME_DATA_MAX 128U
#define UART_CONSOLE_FRAME_MAX (UART_CONSOLE_FRAME_DATA_MAX + 5U)

typedef struct {
    /* 二进制帧命令字。 */
    uint8_t cmd;
    /* data 有效长度。 */
    uint8_t len;
    /* 帧载荷。 */
    uint8_t data[UART_CONSOLE_FRAME_DATA_MAX];
} UartConsoleFrame_t;

/* 初始化 USART3 接收状态和异步发送队列。 */
void UartConsole_Init(void);
/* 周期调度发送超时保护。 */
void UartConsole_Tick(void);
/* 控制普通文本调试输出是否启用。 */
void UartConsole_SetTextOutputEnabled(bool enable);
bool UartConsole_GetTextOutputEnabled(void);
/* 发送中断和待发送队列都为空时返回 true。 */
bool UartConsole_IsTxIdle(void);

/* 轮询一条 ASCII 文本命令或一帧二进制协议帧。 */
bool UartConsole_PollLine(char *line_out, uint16_t max_len);
bool UartConsole_PollFrame(UartConsoleFrame_t *frame_out);

/* 文本发送队列接口。DropOldest 版本适合心跳/曲线这类可丢旧数据的输出。 */
bool UartConsole_Queue(const char *text);
bool UartConsole_QueueLine(const char *text);
bool UartConsole_QueueDropOldest(const char *text, bool *dropped_oldest);
bool UartConsole_QueueLineDropOldest(const char *text, bool *dropped_oldest);
/* 原始字节发送接口，用于给 ESP32 返回二进制帧。 */
bool UartConsole_QueueBytes(const uint8_t *data, uint16_t len);

void UartConsole_Send(const char *text);
void UartConsole_SendLine(const char *text);

#ifdef __cplusplus
}
#endif

#endif
