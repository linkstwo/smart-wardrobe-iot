/**
 * @file main.c
 * @brief Main entry point for ESP32 Server
 *
 * 中文说明：
 * 本文件是 ESP-IDF 工程的入口文件。ESP32 上电后会从 app_main() 进入，
 * 这里不直接处理业务协议，而是只负责打开应用层需要的三个基础能力：
 * 1. UART：和 STM32 主控通信，用于转发取衣/放衣/回零/状态查询帧。
 * 2. WiFi：连接路由器，让 ESP32 能访问阿里云 IoT MQTT Broker。
 * 3. MQTT：订阅阿里云自定义 Topic，接收云端命令并上报 STM32 响应。
 *
 * 这样分层后，入口文件保持很薄，具体初始化细节都放在 app/app_init.c，
 * 具体运行生命周期管理放在 app/app.c。
 */

#include <stdio.h>
#include "esp_log.h"
#include "app.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 Server Starting...");
    ESP_LOGI(TAG, "=======================");
    
    /*
     * 应用功能开关：
     * 当前智能衣柜链路需要 UART、WiFi、MQTT 三者同时启用。
     * 如果后续只想单独调试某一层，可以临时关闭其它开关，
     * 但正常联调阿里云到 STM32 的完整链路时必须全部为 true。
     */
    app_config_t config = {
        .uart_enabled = true,
        .wifi_enabled = true,
        .mqtt_enabled = true,
    };
    
    /* 初始化应用层。失败时直接返回，避免在组件未就绪时进入主循环。 */
    esp_err_t ret = app_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Application init failed");
        return;
    }
    
    /*
     * 进入应用主循环。app_run() 内部是常驻循环，正常情况下不会返回。
     * 只有未来扩展为可退出应用时，才会继续执行 app_deinit()。
     */
    app_run();
    
    app_deinit();
}
