/**
 * @file uart_config.h
 * @brief UART component configuration header
 * 
 * This file contains compile-time configuration macros for the UART component.
 * These can be overridden by menuconfig or sdkconfig settings.
 *
 * 中文说明：
 * 本文件定义 ESP32 UART 组件默认参数。
 * 当前智能衣柜项目使用 UART1 连接 STM32：
 * ESP32 GPIO17(TX) -> STM32 RX
 * ESP32 GPIO18(RX) -> STM32 TX
 * GND 必须共地，波特率默认 115200，数据格式默认 8N1。
 */

#ifndef UART_CONFIG_H
#define UART_CONFIG_H

#include "driver/uart.h"

/** 最多支持的逻辑 UART 端口数量。当前项目只初始化 1 个端口。 */
#define UART_MAX_PORTS           3

/** 默认波特率，必须与 STM32 USART3/协议串口设置一致。 */
#ifndef CONFIG_UART_DEFAULT_BAUD_RATE
#define CONFIG_UART_DEFAULT_BAUD_RATE    115200
#endif

/** 默认数据位：8 位数据。 */
#ifndef CONFIG_UART_DEFAULT_DATA_BITS
#define CONFIG_UART_DEFAULT_DATA_BITS    UART_DATA_8_BITS
#endif

/** 默认不使用奇偶校验。 */
#ifndef CONFIG_UART_DEFAULT_PARITY
#define CONFIG_UART_DEFAULT_PARITY       UART_PARITY_DISABLE
#endif

/** 默认 1 个停止位。 */
#ifndef CONFIG_UART_DEFAULT_STOP_BITS
#define CONFIG_UART_DEFAULT_STOP_BITS    UART_STOP_BITS_1
#endif

/** 默认关闭硬件流控，只使用 TX/RX/GND 三线。 */
#ifndef CONFIG_UART_DEFAULT_FLOW_CTRL
#define CONFIG_UART_DEFAULT_FLOW_CTRL    UART_HW_FLOWCTRL_DISABLE
#endif

/** Default RX buffer size */
#ifndef CONFIG_UART_DEFAULT_RX_BUF_SIZE
#define CONFIG_UART_DEFAULT_RX_BUF_SIZE  1024
#endif

/** Default TX buffer size */
#ifndef CONFIG_UART_DEFAULT_TX_BUF_SIZE
#define CONFIG_UART_DEFAULT_TX_BUF_SIZE  1024
#endif

/** Default timeout in milliseconds */
#ifndef CONFIG_UART_DEFAULT_TIMEOUT_MS
#define CONFIG_UART_DEFAULT_TIMEOUT_MS   1000
#endif

/** Default interrupt flags */
#ifndef CONFIG_UART_DEFAULT_INTR_FLAGS
#define CONFIG_UART_DEFAULT_INTR_FLAGS   0
#endif

/** UART1 TX 引脚：ESP32 发给 STM32 的数据从这里输出。 */
#ifndef CONFIG_UART1_TX_PIN
#define CONFIG_UART1_TX_PIN    17
#endif

/** UART1 RX 引脚：STM32 发回 ESP32 的数据从这里输入。 */
#ifndef CONFIG_UART1_RX_PIN
#define CONFIG_UART1_RX_PIN    18
#endif

/** UART1 RTS pin */
#ifndef CONFIG_UART1_RTS_PIN
#define CONFIG_UART1_RTS_PIN   19
#endif

/** UART1 CTS pin */
#ifndef CONFIG_UART1_CTS_PIN
#define CONFIG_UART1_CTS_PIN   20
#endif

/** Enable UART acknowledgement on data reception */
#ifndef CONFIG_UART_ENABLE_ACK
#define CONFIG_UART_ENABLE_ACK 0
#endif

/** UART acknowledgement message */
#ifndef CONFIG_UART_ACK_MESSAGE
#define CONFIG_UART_ACK_MESSAGE "OK\n"
#endif

#endif
