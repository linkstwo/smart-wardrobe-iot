/**
 * @file app.c
 * @brief Main application layer implementation
 *
 * 中文说明：
 * 本文件是 ESP32 应用层的生命周期管理中心。
 * 它不关心每个组件内部怎么连接 WiFi、怎么收 MQTT、怎么读 UART，
 * 只根据 app_config_t 决定哪些组件需要初始化、哪些组件需要反初始化。
 *
 * 当前工程的完整工作链路：
 * app_init() -> 初始化 UART -> 初始化 WiFi -> 初始化 MQTT 并订阅衣柜 Topic
 * app_run()  -> 保持 FreeRTOS 任务运行，实际事件由各组件任务处理
 * app_deinit()-> 按 MQTT、WiFi、UART 的反向顺序释放资源
 */

#include "app.h"
#include "app_init.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "APP";

/** 保存当前应用启用的组件配置，供反初始化时按需关闭组件。 */
static app_config_t s_app_config = {0};

/** 应用层是否已经初始化，防止重复初始化导致资源重复创建。 */
static bool s_initialized = false;

/**
 * @brief Initialize application layer
 * 
 * Initializes all enabled components based on the provided configuration.
 * If no configuration is provided, default values are used.
 * 
 * @param config Pointer to application configuration structure, or NULL for defaults
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_init(app_config_t *config)
{
    /* 已初始化时直接返回成功，保证 app_init() 可以被安全重复调用。 */
    if (s_initialized) {
        ESP_LOGW(TAG, "App already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing application...");
    
    /*
     * 如果调用者提供了配置，就完全按调用者配置执行；
     * 如果没有提供，则默认启用 UART、WiFi、MQTT，满足当前智能衣柜完整链路。
     */
    if (config) {
        memcpy(&s_app_config, config, sizeof(app_config_t));
    } else {
        s_app_config.uart_enabled = true;
        s_app_config.wifi_enabled = true;
        s_app_config.mqtt_enabled = true;
    }
    
    esp_err_t ret;
    
    /*
     * UART 是 ESP32 和 STM32 的本地链路。
     * MQTT 收到云端命令后最终要通过 UART 发给 STM32，
     * 所以正常情况下 UART 应先于 MQTT 初始化。
     */
    if (s_app_config.uart_enabled) {
        ret = app_init_uart();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "UART init failed");
        }
    }
    
    /*
     * WiFi 是 MQTT 的网络基础。
     * 当前配置为 STA 模式，ESP32 作为客户端连接路由器。
     */
    if (s_app_config.wifi_enabled) {
        ret = app_init_wifi();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "WiFi init failed");
        }
    }
    
    /*
     * MQTT 依赖网络，放在 WiFi 初始化之后。
     * MQTT 初始化完成后会连接阿里云并订阅自定义衣柜 Topic。
     */
    if (s_app_config.mqtt_enabled) {
        ret = app_init_mqtt();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "MQTT init failed");
        }
    }
    
    s_initialized = true;
    
    ESP_LOGI(TAG, "Application initialized");
    ESP_LOGI(TAG, "=========================");
    
    return ESP_OK;
}

/**
 * @brief Deinitialize application layer
 * 
 * Deinitializes all components in reverse order of initialization.
 * Safe to call even if not initialized.
 */
void app_deinit(void)
{
    if (!s_initialized) {
        return;
    }
    
    ESP_LOGI(TAG, "Deinitializing application...");
    
    /*
     * 反初始化顺序和初始化顺序相反：
     * 先断开 MQTT，避免网络或 UART 已关闭后仍收到命令；
     * 再关闭 WiFi；最后关闭 UART。
     */
    if (s_app_config.mqtt_enabled) {
        app_deinit_mqtt();
    }
    
    if (s_app_config.wifi_enabled) {
        app_deinit_wifi();
    }
    
    if (s_app_config.uart_enabled) {
        app_deinit_uart();
    }
    
    s_initialized = false;
    ESP_LOGI(TAG, "Application deinitialized");
}

/**
 * @brief Run application main loop
 * 
 * Starts the scheduler if enabled and enters an infinite loop.
 * This function does not return.
 */
void app_run(void)
{
    ESP_LOGI(TAG, "Running application main loop");
    

    
    /*
     * ESP-IDF 的各组件事件在各自的 FreeRTOS 任务/回调中处理。
     * 这里保持主任务不退出即可，避免 app_main() 返回后应用生命周期结束。
     */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
