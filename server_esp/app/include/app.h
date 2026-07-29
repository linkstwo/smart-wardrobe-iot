/**
 * @file app.h
 * @brief Main application header file
 * 
 * This file defines the public API for the main application layer,
 * including configuration structures and lifecycle management functions.
 *
 * 中文说明：
 * 应用层只暴露“初始化、运行、反初始化”三个生命周期接口。
 * 具体 UART/WiFi/MQTT 参数由 app_init.c 和各组件配置头文件负责。
 */

#ifndef APP_H
#define APP_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Application configuration structure
 * 
 * This structure contains flags to enable or disable various
 * hardware components and features of the application.
 */
typedef struct {
    bool uart_enabled;          /**< 是否启用 UART 组件，用于连接 STM32 */
    bool wifi_enabled;          /**< 是否启用 WiFi 组件，用于连接路由器 */
    bool mqtt_enabled;          /**< 是否启用 MQTT 组件，用于连接阿里云 IoT */
} app_config_t;

/**
 * @brief Initialize the application
 * 
 * Initializes all enabled components according to the configuration.
 * 
 * @param config Pointer to application configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_init(app_config_t *config);

/**
 * @brief Deinitialize the application
 * 
 * Stops and cleans up all initialized components in reverse order.
 */
void app_deinit(void);

/**
 * @brief Run the application main loop
 * 
 * Starts the scheduler (if enabled) and enters the main loop.
 * This function does not return.
 */
void app_run(void);

#ifdef __cplusplus
}
#endif

#endif
