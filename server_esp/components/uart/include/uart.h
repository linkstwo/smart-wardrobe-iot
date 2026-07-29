/**
 * @file uart.h
 * @brief UART component header file
 * 
 * This file defines the public API for the UART component, supporting
 * multiple UART ports with event-driven callbacks for data reception
 * and error handling.
 *
 * 中文说明：
 * 本头文件定义 ESP32 UART 组件对外接口。
 * 当前智能衣柜项目使用它与 STM32 通信：
 * - MQTT 下行命令通过 uart_write() 发给 STM32。
 * - STM32 上行二进制帧由 uart.c 内部解析并上报 MQTT。
 */

#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "uart_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UART port identifier type
 */
typedef uint8_t uart_port_id_t;

/**
 * @brief UART port configuration structure
 */
typedef struct {
    uart_port_t port;           /**< ESP32 硬件 UART 编号，例如 UART_NUM_1 */
    int tx_pin;                 /**< TX GPIO，连接 STM32 RX */
    int rx_pin;                 /**< RX GPIO，连接 STM32 TX */
    int rts_pin;                /**< RTS GPIO，当前默认不用硬件流控 */
    int cts_pin;                /**< CTS GPIO，当前默认不用硬件流控 */
    int baud_rate;              /**< 波特率，必须与 STM32 一致 */
    uart_word_length_t data_bits;   /**< 数据位，当前默认 8 位 */
    uart_parity_t parity;       /**< 校验位，当前默认无校验 */
    uart_stop_bits_t stop_bits; /**< 停止位，当前默认 1 位 */
    uart_hw_flowcontrol_t flow_ctrl;  /**< 硬件流控模式 */
    int rx_buf_size;            /**< RX 驱动缓冲区大小 */
    int tx_buf_size;            /**< TX 驱动缓冲区大小 */
    uint32_t timeout_ms;        /**< 读取超时时间，单位毫秒 */
    int intr_flags;             /**< UART 中断标志 */
    bool enable_rx_intr;        /**< 是否启用接收中断 */
    bool enable_tx_intr;        /**< 是否启用发送中断 */
    bool enable_err_intr;       /**< 是否启用错误中断 */
} uart_port_cfg_t;

/**
 * @brief UART event types
 */
typedef enum {
    UART_EVT_INIT_COMPLETE,     /**< UART initialization complete */
    UART_EVT_DEINIT_COMPLETE,   /**< UART deinitialization complete */
    UART_EVT_PORT_ADDED,        /**< UART port added */
    UART_EVT_PORT_REMOVED,      /**< UART port removed */
    UART_EVT_RX_DATA,           /**< Data received */
    UART_EVT_RX_FIFO_OVF,       /**< RX FIFO overflow */
    UART_EVT_RX_FIFO_FULL,      /**< RX buffer full */
    UART_EVT_RX_TIMEOUT,        /**< RX timeout */
    UART_EVT_TX_DONE,           /**< Transmission complete */
    UART_EVT_TX_FIFO_EMPTY,     /**< TX FIFO empty */
    UART_EVT_BRK_DET,           /**< Break detected */
    UART_EVT_PARITY_ERR,        /**< Parity error */
    UART_EVT_FRAME_ERR,         /**< Frame error */
    UART_EVT_PATTERN_DET,       /**< Pattern detected */
} uart_evt_type_t;

/**
 * @brief UART event data structure
 */
typedef struct {
    uart_evt_type_t type;       /**< UART 事件类型 */
    uart_port_id_t port_id;     /**< 逻辑端口 ID */
    uint8_t *data;              /**< 接收事件携带的数据指针 */
    size_t data_len;            /**< 数据长度 */
    int status;                 /**< 事件状态码 */
} uart_evt_data_t;

/**
 * @brief UART event callback function type
 */
typedef void (*uart_evt_cb_t)(uart_evt_data_t *event_data);

/**
 * @brief UART configuration structure
 */
typedef struct {
    uart_port_cfg_t *ports;     /**< 要初始化的端口配置数组 */
    uint8_t port_count;         /**< 配置数组中的端口数量 */
    uart_evt_cb_t event_callback;  /**< 全局 UART 事件回调 */
} uart_cfg_t;

/**
 * @brief Initialize UART component
 * 
 * @param config Pointer to UART configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_init(uart_cfg_t *config);

/**
 * @brief Deinitialize UART component
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_deinit(void);

/**
 * @brief Add a UART port
 * 
 * @param config Port configuration
 * @param port_id Pointer to store assigned port ID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_add_port(uart_port_cfg_t *config, uart_port_id_t *port_id);

/**
 * @brief Remove a UART port
 * 
 * @param port_id Port identifier
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_remove_port(uart_port_id_t port_id);

/**
 * @brief Update port configuration
 * 
 * @param port_id Port identifier
 * @param config New port configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_update_port_config(uart_port_id_t port_id, uart_port_cfg_t *config);

/**
 * @brief Register global event callback
 * 
 * @param callback Callback function
 * @return ESP_OK on success
 */
esp_err_t uart_register_callback(uart_evt_cb_t callback);

/**
 * @brief Register port-specific event callback
 * 
 * @param port_id Port identifier
 * @param callback Callback function
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_port_register_callback(uart_port_id_t port_id, uart_evt_cb_t callback);

/**
 * @brief Write data to UART port
 * 
 * @param port_id Port identifier
 * @param data Data buffer
 * @param len Data length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_write(uart_port_id_t port_id, const uint8_t *data, size_t len);

/**
 * @brief Remember the last wardrobe command sent to a UART port.
 *
 * The UART protocol response frame only carries the position on success, so
 * the MQTT response uses this pending command to fill the original action.
 *
 * 中文说明：
 * STM32 的成功响应帧只告诉 ESP32“哪个位置完成了”，不告诉“这次是存还是取”。
 * 因此 MQTT 下行命令发出后，需要在这里暂存 action 和 position，
 * 等 STM32 回 0x90 成功帧时再补到云端 JSON 响应里。
 *
 * @param port_id Port identifier
 * @param action Last command action, 0x10 for store or 0x11 for take
 * @param position Position sent with the command
 */
void uart_protocol_set_pending_command(uart_port_id_t port_id, uint8_t action,
                                       uint8_t position);

/**
 * @brief Read data from UART port
 * 
 * @param port_id Port identifier
 * @param data Data buffer
 * @param len Buffer length
 * @param actual_len Pointer to store actual read length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_read(uart_port_id_t port_id, uint8_t *data, size_t len, size_t *actual_len);

/**
 * @brief Write data asynchronously
 * 
 * @param port_id Port identifier
 * @param data Data buffer
 * @param len Data length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_write_bytes_async(uart_port_id_t port_id, const uint8_t *data, size_t len);

/**
 * @brief Get number of bytes available in RX buffer
 * 
 * @param port_id Port identifier
 * @return Number of bytes available, -1 on error
 */
int uart_available(uart_port_id_t port_id);

/**
 * @brief Flush TX and RX buffers
 * 
 * @param port_id Port identifier
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_port_flush(uart_port_id_t port_id);

/**
 * @brief Flush RX buffer only
 * 
 * @param port_id Port identifier
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_port_flush_input(uart_port_id_t port_id);

/**
 * @brief Set baud rate
 * 
 * @param port_id Port identifier
 * @param baud_rate New baud rate
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_port_set_baudrate(uart_port_id_t port_id, int baud_rate);

/**
 * @brief Get current baud rate
 * 
 * @param port_id Port identifier
 * @return Baud rate, -1 on error
 */
int uart_port_get_baudrate(uart_port_id_t port_id);

/**
 * @brief Enable pattern detection
 * 
 * @param port_id Port identifier
 * @param pattern_char Pattern character to detect
 * @param num_chars Number of consecutive characters
 * @param idle_timeout Idle timeout between characters
 * @param pre_idle_chars Pre-idle character count
 * @param post_idle_chars Post-idle character count
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_enable_pattern_det(uart_port_id_t port_id, char pattern_char, uint8_t num_chars, uint32_t idle_timeout, uint8_t pre_idle_chars, uint8_t post_idle_chars);

/**
 * @brief Disable pattern detection
 * 
 * @param port_id Port identifier
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t uart_disable_pattern_det(uart_port_id_t port_id);

/**
 * @brief Check if port is valid
 * 
 * @param port_id Port identifier
 * @return true if valid, false otherwise
 */
bool uart_port_is_valid(uart_port_id_t port_id);

/**
 * @brief Get hardware port number
 * 
 * @param port_id Port identifier
 * @return Hardware port number, UART_NUM_MAX on error
 */
uart_port_t uart_get_hw_port(uart_port_id_t port_id);

#ifdef __cplusplus
}
#endif

#endif
