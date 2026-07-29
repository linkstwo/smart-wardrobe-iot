/**
 * @file app_init.h
 * @brief Application initialization header file
 * 
 * This file defines the API for initializing and deinitializing
 * individual application components.
 * and their default event callback functions.
 *
 * 中文说明：
 * 本头文件声明单个组件的初始化/反初始化函数。
 * app.c 通过这些接口按顺序启动 UART、WiFi、MQTT。
 */

#ifndef APP_INIT_H
#define APP_INIT_H

#include "esp_err.h"
#include "uart.h"
#include "wifi.h"
#include "mqtt.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize UART component
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_init_uart(void);

/**
 * @brief Initialize WiFi component
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_init_wifi(void);

/**
 * @brief Deinitialize UART component
 */
void app_deinit_uart(void);

/**
 * @brief Deinitialize WiFi component
 */
void app_deinit_wifi(void);

/**
 * @brief Initialize MQTT component
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_init_mqtt(void);

/**
 * @brief Deinitialize MQTT component
 */
void app_deinit_mqtt(void);

/**
 * @brief Default MQTT event callback
 *
 * @param event_data Pointer to MQTT event data
 */
void mqtt_default_callback(mqtt_evt_data_t *event_data);

/**
 * @brief Default UART event callback
 * 
 * @param event_data Pointer to UART event data
 */
void uart_default_callback(uart_evt_data_t *event_data);

/**
 * @brief Default WiFi event callback
 * 
 * @param event_data Pointer to WiFi event data
 */
void wifi_default_callback(wifi_evt_data_t *event_data);

#ifdef __cplusplus
}
#endif

#endif
