/**
 * @file app_init.c
 * @brief Application initialization and default callbacks
 *
 * 中文说明：
 * 本文件负责把底层组件按项目默认参数配置好。
 * 它是“配置装配层”，主要做三件事：
 * 1. 配置 UART1：连接 STM32，发送/接收智能衣柜二进制协议帧。
 * 2. 配置 WiFi STA：连接路由器，为 MQTT 提供网络。
 * 3. 配置 MQTT：连接阿里云 IoT，并订阅自定义 Topic。
 *
 * 注意：
 * 这里的默认回调主要用于打印日志和观察状态，不直接写业务逻辑。
 * 真正的衣柜命令解析在 mqtt_wardrobe_command_handler() 中完成。
 */

#include "app_init.h"
#include "esp_log.h"
#include "uart_config.h"
#include "wifi_config.h"
#include "mqtt_config.h"
#include <string.h>

static const char *TAG = "APP_INIT";

/**
 * @brief Default UART event callback handler
 * 
 * Handles UART events and logs them for debugging purposes.
 * 
 * @param event_data Pointer to UART event data structure
 */
void uart_default_callback(uart_evt_data_t *event_data)
{
    /*
     * UART 事件日志用于联调：
     * - RX_DATA 表示 ESP32 从 STM32 收到了字节；
     * - TX_DONE 表示 ESP32 已把命令帧写入 UART 驱动；
     * - FIFO_OVF/TIMEOUT 可用于判断串口是否有溢出或异常等待。
     */
    switch (event_data->type) {
        case UART_EVT_INIT_COMPLETE:
            ESP_LOGI(TAG, "UART init complete");
            break;
        case UART_EVT_DEINIT_COMPLETE:
            ESP_LOGI(TAG, "UART deinit complete");
            break;
        case UART_EVT_PORT_ADDED:
            ESP_LOGI(TAG, "UART port added, port_id=%d", event_data->port_id);
            break;
        case UART_EVT_PORT_REMOVED:
            ESP_LOGI(TAG, "UART port removed, port_id=%d", event_data->port_id);
            break;
        case UART_EVT_RX_DATA:
            ESP_LOGI(TAG, "UART RX data, port_id=%d, len=%d", event_data->port_id, event_data->data_len);
            break;
        case UART_EVT_TX_DONE:
            ESP_LOGI(TAG, "UART TX done, port_id=%d", event_data->port_id);
            break;
        case UART_EVT_RX_FIFO_OVF:
            ESP_LOGW(TAG, "UART RX FIFO overflow, port_id=%d", event_data->port_id);
            break;
        case UART_EVT_RX_TIMEOUT:
            ESP_LOGI(TAG, "UART RX timeout, port_id=%d", event_data->port_id);
            break;
        default:
            break;
    }
}

/**
 * @brief Default WiFi event callback handler
 * 
 * Handles WiFi events (connection, disconnection, IP acquisition, etc.)
 * and logs them for debugging purposes.
 * 
 * @param event_data Pointer to WiFi event data structure
 */
void wifi_default_callback(wifi_evt_data_t *event_data)
{
    /*
     * WiFi 事件日志用于确认 MQTT 的前置条件是否满足。
     * 只有 STA_CONNECTED 并 GOT_IP 之后，MQTT 才有稳定连接阿里云的基础。
     */
    switch (event_data->type) {
        case WIFI_EVT_INIT_COMPLETE:
            ESP_LOGI(TAG, "WiFi init complete");
            break;
        case WIFI_EVT_DEINIT_COMPLETE:
            ESP_LOGI(TAG, "WiFi deinit complete");
            break;
        case WIFI_EVT_STA_START:
            ESP_LOGI(TAG, "WiFi STA started");
            break;
        case WIFI_EVT_STA_STOP:
            ESP_LOGI(TAG, "WiFi STA stopped");
            break;
        case WIFI_EVT_STA_CONNECTED:
            ESP_LOGI(TAG, "WiFi STA connected to SSID: %s", event_data->data.sta_info.ssid);
            break;
        case WIFI_EVT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "WiFi STA disconnected");
            break;
        case WIFI_EVT_STA_GOT_IP:
            ESP_LOGI(TAG, "WiFi STA got IP: %lu.%lu.%lu.%lu",
                     (unsigned long)(event_data->data.ip_info.ip & 0xFF),
                     (unsigned long)((event_data->data.ip_info.ip >> 8) & 0xFF),
                     (unsigned long)((event_data->data.ip_info.ip >> 16) & 0xFF),
                     (unsigned long)((event_data->data.ip_info.ip >> 24) & 0xFF));
            break;
        case WIFI_EVT_STA_LOST_IP:
            ESP_LOGW(TAG, "WiFi STA lost IP");
            break;
        case WIFI_EVT_AP_START:
            ESP_LOGI(TAG, "WiFi AP started");
            break;
        case WIFI_EVT_AP_STOP:
            ESP_LOGI(TAG, "WiFi AP stopped");
            break;
        case WIFI_EVT_AP_STACONNECTED:
            ESP_LOGI(TAG, "WiFi AP: station connected, MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                     event_data->data.ap_sta_info.mac[0], event_data->data.ap_sta_info.mac[1],
                     event_data->data.ap_sta_info.mac[2], event_data->data.ap_sta_info.mac[3],
                     event_data->data.ap_sta_info.mac[4], event_data->data.ap_sta_info.mac[5]);
            break;
        case WIFI_EVT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "WiFi AP: station disconnected, MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                     event_data->data.ap_sta_info.mac[0], event_data->data.ap_sta_info.mac[1],
                     event_data->data.ap_sta_info.mac[2], event_data->data.ap_sta_info.mac[3],
                     event_data->data.ap_sta_info.mac[4], event_data->data.ap_sta_info.mac[5]);
            break;
        case WIFI_EVT_SCAN_DONE:
            ESP_LOGI(TAG, "WiFi scan done");
            break;
        case WIFI_EVT_CONNECTION_TIMEOUT:
            ESP_LOGW(TAG, "WiFi connection timeout");
            break;
        case WIFI_EVT_CONNECTION_FAILED:
            ESP_LOGE(TAG, "WiFi connection failed, status=%d", event_data->status);
            break;
        default:
            break;
    }
}

/**
 * @brief Initialize UART component with default configuration
 * 
 * Configures UART1 with default pins and baud rate from menuconfig.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_init_uart(void)
{
    /*
     * UART1 默认作为 ESP32 -> STM32 的控制链路：
     * TX 引脚接 STM32 RX，RX 引脚接 STM32 TX，双方必须共地。
     * 参数来自 menuconfig/Kconfig，当前项目默认 115200-8N1。
     */
    uart_port_cfg_t port1_config = {
        .port = UART_NUM_1,
        .tx_pin = CONFIG_UART1_TX_PIN,
        .rx_pin = CONFIG_UART1_RX_PIN,
        .rts_pin = CONFIG_UART1_RTS_PIN,
        .cts_pin = CONFIG_UART1_CTS_PIN,
        .baud_rate = CONFIG_UART_DEFAULT_BAUD_RATE,
        .data_bits = CONFIG_UART_DEFAULT_DATA_BITS,
        .parity = CONFIG_UART_DEFAULT_PARITY,
        .stop_bits = CONFIG_UART_DEFAULT_STOP_BITS,
        .flow_ctrl = CONFIG_UART_DEFAULT_FLOW_CTRL,
        .rx_buf_size = CONFIG_UART_DEFAULT_RX_BUF_SIZE,
        .tx_buf_size = CONFIG_UART_DEFAULT_TX_BUF_SIZE,
        .timeout_ms = CONFIG_UART_DEFAULT_TIMEOUT_MS,
        .intr_flags = CONFIG_UART_DEFAULT_INTR_FLAGS,
        .enable_rx_intr = true,
        .enable_tx_intr = false,
        .enable_err_intr = true,
    };

    uart_cfg_t uart_config = {
        .ports = &port1_config,
        .port_count = 1,
        .event_callback = uart_default_callback,
    };

    /* 创建 UART 组件、安装驱动并启动接收任务。 */
    esp_err_t ret = uart_init(&uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART initialized: Port count=%d, UART1: TX=%d, RX=%d, Baud=%d",
             uart_config.port_count, port1_config.tx_pin, port1_config.rx_pin, port1_config.baud_rate);

    return ESP_OK;
}

/**
 * @brief Initialize WiFi component with default configuration
 * 
 * Configures WiFi in station mode and attempts to connect to configured AP.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_init_wifi(void)
{
    /*
     * 当前项目使用 STA 模式接入路由器。
     * AP 配置保留在结构体里，是为了后续扩展配网/热点模式；
     * 当前完整云端链路主要依赖 STA。
     */
    wifi_cfg_t wifi_config = {
         .mode = WIFI_MODE_SELECT_STA,
        .sta = {
            .ssid = CONFIG_WIFI_STA_SSID,
            .password = CONFIG_WIFI_STA_PASSWORD,
            .channel = 0,
            .power_save = WIFI_PS_NONE,
            .auto_reconnect = true,
            .max_retry = CONFIG_WIFI_MAX_RETRY_COUNT,
            .scan_method = CONFIG_WIFI_STA_SCAN_METHOD,
            .sort_method = CONFIG_WIFI_STA_SORT_METHOD,
            .rssi_threshold = CONFIG_WIFI_STA_RSSI_THRESHOLD,
        },
        .ap = {
            .ssid = CONFIG_WIFI_AP_SSID,
            .password = CONFIG_WIFI_AP_PASSWORD,
            .channel = CONFIG_WIFI_AP_CHANNEL,
            .max_connection = CONFIG_WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .beacon_interval = CONFIG_WIFI_AP_BEACON_INTERVAL,
            .ssid_hidden = false,
        },
        .event_callback = wifi_default_callback,
    };

    ESP_LOGI(TAG, "WiFi STA target SSID: %s", wifi_config.sta.ssid);

    /* 初始化 WiFi 驱动并按 mode 自动启动 STA 连接流程。 */
    esp_err_t ret = wifi_init(&wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WiFi initialized: Mode=%s",
             wifi_config.mode == WIFI_MODE_SELECT_STA ? "STA" : 
             (wifi_config.mode == WIFI_MODE_SELECT_AP ? "AP" : "APSTA"));

    return ESP_OK;
}


/**
 * @brief Deinitialize UART component
 */
void app_deinit_uart(void)
{
    uart_deinit();
}

/**
 * @brief Deinitialize WiFi component
 */
void app_deinit_wifi(void)
{
    wifi_wrapper_deinit();
}

/**
 * @brief Default MQTT event callback handler
 *
 * Handles MQTT events and logs them for debugging purposes.
 *
 * @param event_data Pointer to MQTT event data structure
 */
void mqtt_default_callback(mqtt_evt_data_t *event_data)
{
    /*
     * MQTT 事件日志用于确认阿里云链路：
     * CONNECTED 表示已连上 Broker；
     * SUBSCRIBED 表示已订阅自定义 Topic；
     * DATA_RECEIVED 表示云端 Pub 的消息已经到达 ESP32。
     */
    switch (event_data->type) {
        case MQTT_EVT_INIT_COMPLETE:
            ESP_LOGI(TAG, "MQTT init complete");
            break;
        case MQTT_EVT_DEINIT_COMPLETE:
            ESP_LOGI(TAG, "MQTT deinit complete");
            break;
        case MQTT_EVT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected to broker");
            break;
        case MQTT_EVT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
        case MQTT_EVT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT subscribed to topic: %s",
                     event_data->data.data_rx.topic);
            break;
        case MQTT_EVT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT unsubscribed from topic");
            break;
        case MQTT_EVT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT published to topic: %s",
                     event_data->data.publish.topic);
            break;
        case MQTT_EVT_DATA_RECEIVED:
            ESP_LOGI(TAG, "MQTT data received on topic: %s, len=%d",
                     event_data->data.data_rx.topic,
                     (int)event_data->data.data_rx.data_len);
            break;
        case MQTT_EVT_ERROR:
            ESP_LOGE(TAG, "MQTT error: %d", event_data->status);
            break;
        default:
            break;
    }
}

/**
 * @brief Initialize MQTT component with default configuration
 *
 * Configures MQTT client, connects to broker, and subscribes
 * to topics for api, health, data, and move commands.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_init_mqtt(void)
{
    /*
     * MQTT 连接参数来自 mqtt_config.h / menuconfig。
     * 当前使用阿里云 IoT 的设备三元组计算出来的 client_id、username、password。
     * 注意：自定义 Topic 收发都使用 WARDROBE_MQTT_TOPIC。
     */
    mqtt_cfg_t mqtt_config = {
        .broker_host = CONFIG_MQTT_BROKER_HOST,
        .broker_port = CONFIG_MQTT_BROKER_PORT,
        .client_id = CONFIG_MQTT_CLIENT_ID,
        .username = CONFIG_MQTT_USERNAME,
        .password = CONFIG_MQTT_PASSWORD,
        .keepalive_sec = CONFIG_MQTT_KEEPALIVE_SEC,
        .reconnect_interval_ms = CONFIG_MQTT_RECONNECT_INTERVAL_MS,
#ifdef CONFIG_MQTT_ENABLE_SSL
        .use_ssl = true,
        .cert_pem = CONFIG_MQTT_BROKER_CERT,
#else
        .use_ssl = false,
        .cert_pem = NULL,
#endif
        .task_stack_size = CONFIG_ESP_MQTT_EVT_TASK_STACK_SIZE,
        .task_priority = CONFIG_ESP_MQTT_EVT_TASK_PRIORITY,
        .event_callback = mqtt_default_callback,
    };

    esp_err_t ret = mqtt_init(&mqtt_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MQTT init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mqtt_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MQTT connect failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * 订阅智能衣柜自定义 Topic。
     * 阿里云 OpenAPI Pub 向该 Topic 发布 Base64(JSON) 后，
     * Broker 会把解码后的 MQTT payload 交给 mqtt_wardrobe_command_handler()。
     */
    mqtt_subscribe(WARDROBE_MQTT_TOPIC, mqtt_wardrobe_command_handler);

    ESP_LOGI(TAG, "MQTT initialized: broker=%s:%d, client_id=%s",
             mqtt_config.broker_host, mqtt_config.broker_port,
             mqtt_config.client_id);

    return ESP_OK;
}

/**
 * @brief Deinitialize MQTT component
 */
void app_deinit_mqtt(void)
{
    mqtt_disconnect();
    mqtt_deinit();
}
