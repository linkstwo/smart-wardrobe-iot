/**
 * @file wifi.c
 * @brief WiFi component implementation
 * 
 * This file implements the WiFi component functionality, supporting
 * STA (Station), AP (Access Point), and APSTA modes with automatic
 * reconnection and event handling.
 *
 * 中文说明：
 * 本文件封装 ESP-IDF WiFi 初始化和事件处理。
 * 在当前智能衣柜项目中，ESP32 主要使用 STA 模式连接路由器，
 * 让 MQTT 组件可以访问阿里云 IoT Broker。
 *
 * AP/APSTA 相关代码保留用于后续扩展，例如本地配网或调试热点；
 * 当前云端联调的关键路径是：
 * wifi_init() -> STA_START -> esp_wifi_connect() -> STA_GOT_IP -> MQTT 连接阿里云。
 */

#include "wifi.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "WIFI";

/** Internal state variables */
static bool s_wifi_initialized = false;         /**< WiFi 是否已经初始化 */
static wifi_mode_select_t s_wifi_mode = WIFI_MODE_SELECT_NONE;  /**< 当前选择的 WiFi 模式 */
static wifi_evt_cb_t s_event_callback = NULL;   /**< 应用层 WiFi 事件回调 */
static EventGroupHandle_t s_wifi_event_group = NULL;  /**< 用于等待连接成功/失败的事件组 */
static bool s_sta_connected = false;            /**< STA 是否已连接到路由器 */
static bool s_ap_active = false;                /**< AP 热点是否已启动 */
static int s_retry_count = 0;                   /**< STA 自动重连计数 */
static wifi_sta_cfg_t s_sta_config = {0};       /**< STA 模式配置缓存 */
static wifi_ap_cfg_t s_ap_config = {0};         /**< AP 模式配置缓存 */

/** Event group bits */
/* STA 获取 IP 后置位，表示 MQTT 可以尝试连接云端。 */
#define WIFI_CONNECTED_BIT BIT0
/* STA 重连耗尽或连接失败后置位。 */
#define WIFI_FAIL_BIT      BIT1

/**
 * @brief WiFi event handler
 * 
 * Handles WiFi and IP events from the ESP-IDF event system.
 * Dispatches events to the registered callback function.
 * 
 * @param arg User argument (unused)
 * @param event_base Event base type
 * @param event_id Event identifier
 * @param event_data Event data pointer
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    wifi_evt_data_t evt_data = {0};

    if (event_base == WIFI_EVENT) {
        /*
         * WIFI_EVENT 主要描述 WiFi 链路层状态：
         * STA_START/STOP、连接/断开、AP 启停、扫描完成等。
         */
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi STA started");
                evt_data.type = WIFI_EVT_STA_START;
                if (s_event_callback) s_event_callback(&evt_data);
                /* STA 启动后立即尝试连接配置的路由器。 */
                esp_wifi_connect();
                break;

            case WIFI_EVENT_STA_STOP:
                ESP_LOGI(TAG, "WiFi STA stopped");
                evt_data.type = WIFI_EVT_STA_STOP;
                if (s_event_callback) s_event_callback(&evt_data);
                break;

            case WIFI_EVENT_STA_CONNECTED: {
                wifi_event_sta_connected_t* event = (wifi_event_sta_connected_t*) event_data;
                ESP_LOGI(TAG, "WiFi STA connected to SSID: %s", event->ssid);
                s_sta_connected = true;
                s_retry_count = 0;
                evt_data.type = WIFI_EVT_STA_CONNECTED;
                memcpy(evt_data.data.sta_info.ssid, event->ssid, sizeof(event->ssid));
                memcpy(evt_data.data.sta_info.bssid, event->bssid, sizeof(event->bssid));
                evt_data.data.sta_info.channel = event->channel;
                evt_data.data.sta_info.authmode = event->authmode;
                if (s_event_callback) s_event_callback(&evt_data);
                break;
            }

            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
                ESP_LOGW(TAG, "WiFi STA disconnected, reason: %d", event->reason);
                s_sta_connected = false;
                evt_data.type = WIFI_EVT_STA_DISCONNECTED;
                if (s_event_callback) s_event_callback(&evt_data);
                
                /*
                 * 自动重连：
                 * 短暂 WiFi 抖动不应该让整个设备永久离线，因此在 max_retry 范围内继续重连。
                 * 超过次数后设置失败位，并通过回调通知应用层。
                 */
                if (s_sta_config.auto_reconnect && s_retry_count < s_sta_config.max_retry) {
                    s_retry_count++;
                    ESP_LOGI(TAG, "Retrying connection (%d/%d)", s_retry_count, s_sta_config.max_retry);
                    esp_wifi_connect();
                } else {
                    if (s_wifi_event_group) {
                        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                    }
                    evt_data.type = WIFI_EVT_CONNECTION_FAILED;
                    evt_data.status = event->reason;
                    if (s_event_callback) s_event_callback(&evt_data);
                }
                break;
            }

            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "WiFi AP started");
                s_ap_active = true;
                evt_data.type = WIFI_EVT_AP_START;
                if (s_event_callback) s_event_callback(&evt_data);
                break;

            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(TAG, "WiFi AP stopped");
                s_ap_active = false;
                evt_data.type = WIFI_EVT_AP_STOP;
                if (s_event_callback) s_event_callback(&evt_data);
                break;

            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
                ESP_LOGI(TAG, "Station connected to AP, MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                         event->mac[0], event->mac[1], event->mac[2],
                         event->mac[3], event->mac[4], event->mac[5]);
                evt_data.type = WIFI_EVT_AP_STACONNECTED;
                memcpy(evt_data.data.ap_sta_info.mac, event->mac, 6);
                evt_data.data.ap_sta_info.aid = event->aid;
                if (s_event_callback) s_event_callback(&evt_data);
                break;
            }

            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
                ESP_LOGI(TAG, "Station disconnected from AP, MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                         event->mac[0], event->mac[1], event->mac[2],
                         event->mac[3], event->mac[4], event->mac[5]);
                evt_data.type = WIFI_EVT_AP_STADISCONNECTED;
                memcpy(evt_data.data.ap_sta_info.mac, event->mac, 6);
                evt_data.data.ap_sta_info.aid = event->aid;
                if (s_event_callback) s_event_callback(&evt_data);
                break;
            }

            case WIFI_EVENT_SCAN_DONE:
                ESP_LOGI(TAG, "WiFi scan done");
                evt_data.type = WIFI_EVT_SCAN_DONE;
                if (s_event_callback) s_event_callback(&evt_data);
                break;

            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        /*
         * IP_EVENT 表示网络层地址变化。
         * 对 MQTT 来说，STA_GOT_IP 是可以连接阿里云的关键事件。
         */
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP: {
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI(TAG, "Got IP: %lu.%lu.%lu.%lu",
                         (unsigned long)(event->ip_info.ip.addr & 0xFF),
                         (unsigned long)((event->ip_info.ip.addr >> 8) & 0xFF),
                         (unsigned long)((event->ip_info.ip.addr >> 16) & 0xFF),
                         (unsigned long)((event->ip_info.ip.addr >> 24) & 0xFF));
                if (s_wifi_event_group) {
                    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                }
                evt_data.type = WIFI_EVT_STA_GOT_IP;
                evt_data.data.ip_info.ip = event->ip_info.ip.addr;
                evt_data.data.ip_info.netmask = event->ip_info.netmask.addr;
                evt_data.data.ip_info.gw = event->ip_info.gw.addr;
                if (s_event_callback) s_event_callback(&evt_data);
                break;
            }

            case IP_EVENT_STA_LOST_IP:
                ESP_LOGW(TAG, "Lost IP");
                evt_data.type = WIFI_EVT_STA_LOST_IP;
                if (s_event_callback) s_event_callback(&evt_data);
                break;

            default:
                break;
        }
    }
}

/**
 * @brief Initialize WiFi component
 * 
 * Initializes the NVS, netif, event loop, and WiFi driver.
 * Configures WiFi in the specified mode (STA/AP/APSTA).
 * 
 * @param config Pointer to WiFi configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_init(wifi_cfg_t *config)
{
    if (s_wifi_initialized) {
        ESP_LOGW(TAG, "WiFi already initialized");
        return ESP_OK;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * 初始化 NVS。
     * ESP-IDF WiFi 驱动会使用 NVS 保存校准数据和部分 WiFi 状态，
     * 因此 WiFi 初始化前必须先保证 NVS 可用。
     */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 初始化 TCP/IP 协议栈，为 DHCP、IP 地址、网关等网络能力做准备。 */
    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize netif: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 创建事件组，用于等待连接成功或连接失败。 */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    /* 创建默认事件循环，WiFi/IP 事件都会通过该事件循环进入 wifi_event_handler()。 */
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
        return ret;
    }

    /*
     * 根据配置创建网络接口：
     * STA 模式创建 station netif；
     * AP 模式创建 access point netif；
     * APSTA 同时创建两者。
     */
    wifi_mode_t mode = WIFI_MODE_NULL;
    if (config->mode == WIFI_MODE_SELECT_STA) {
        mode = WIFI_MODE_STA;
        esp_netif_create_default_wifi_sta();
    } else if (config->mode == WIFI_MODE_SELECT_AP) {
        mode = WIFI_MODE_AP;
        esp_netif_create_default_wifi_ap();
    } else if (config->mode == WIFI_MODE_SELECT_APSTA) {
        mode = WIFI_MODE_APSTA;
        esp_netif_create_default_wifi_sta();
        esp_netif_create_default_wifi_ap();
    }

    /* 使用 ESP-IDF 默认参数初始化 WiFi 驱动。 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 注册 WiFi 事件和 IP 事件处理函数。 */
    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP lost event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set WiFi mode */
    ret = esp_wifi_set_mode(mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure station mode if selected */
    if (config->mode == WIFI_MODE_SELECT_STA || config->mode == WIFI_MODE_SELECT_APSTA) {
        wifi_config_t sta_config = {0};
        strncpy((char*)sta_config.sta.ssid, config->sta.ssid, sizeof(sta_config.sta.ssid) - 1);
        strncpy((char*)sta_config.sta.password, config->sta.password, sizeof(sta_config.sta.password) - 1);
        sta_config.sta.channel = config->sta.channel;
        sta_config.sta.scan_method = config->sta.scan_method;
        sta_config.sta.sort_method = config->sta.sort_method;
        sta_config.sta.threshold.rssi = config->sta.rssi_threshold;
        sta_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
        sta_config.sta.pmf_cfg.capable = true;
        sta_config.sta.pmf_cfg.required = false;

        ret = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set STA config: %s", esp_err_to_name(ret));
            return ret;
        }

        memcpy(&s_sta_config, &config->sta, sizeof(wifi_sta_cfg_t));
    }

    /* Configure AP mode if selected */
    if (config->mode == WIFI_MODE_SELECT_AP || config->mode == WIFI_MODE_SELECT_APSTA) {
        wifi_config_t ap_config = {0};
        strncpy((char*)ap_config.ap.ssid, config->ap.ssid, sizeof(ap_config.ap.ssid) - 1);
        strncpy((char*)ap_config.ap.password, config->ap.password, sizeof(ap_config.ap.password) - 1);
        ap_config.ap.channel = config->ap.channel;
        ap_config.ap.max_connection = config->ap.max_connection;
        ap_config.ap.authmode = config->ap.authmode;
        ap_config.ap.beacon_interval = config->ap.beacon_interval;
        ap_config.ap.ssid_hidden = config->ap.ssid_hidden;

        ret = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set AP config: %s", esp_err_to_name(ret));
            return ret;
        }

        memcpy(&s_ap_config, &config->ap, sizeof(wifi_ap_cfg_t));
    }

    /* Start WiFi */
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Disable power save to improve stability */
    ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to disable power save: %s", esp_err_to_name(ret));
    }

    s_wifi_mode = config->mode;
    s_event_callback = config->event_callback;
    s_wifi_initialized = true;

    /* Notify initialization complete */
    wifi_evt_data_t evt_data = {.type = WIFI_EVT_INIT_COMPLETE};
    if (s_event_callback) s_event_callback(&evt_data);

    ESP_LOGI(TAG, "WiFi initialized successfully, mode: %d", s_wifi_mode);
    return ESP_OK;
}

/**
 * @brief Deinitialize WiFi component
 * 
 * Stops and deinitializes the WiFi driver and cleans up resources.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_wrapper_deinit(void)
{
    if (!s_wifi_initialized) {
        return ESP_OK;
    }

    esp_err_t ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinitialize WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    s_wifi_initialized = false;
    s_wifi_mode = WIFI_MODE_SELECT_NONE;
    s_sta_connected = false;
    s_ap_active = false;
    s_event_callback = NULL;

    wifi_evt_data_t evt_data = {.type = WIFI_EVT_DEINIT_COMPLETE};
    if (s_event_callback) s_event_callback(&evt_data);

    ESP_LOGI(TAG, "WiFi deinitialized");
    return ESP_OK;
}

/**
 * @brief Connect to configured AP
 */
esp_err_t wifi_sta_connect(void)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    s_retry_count = 0;
    return esp_wifi_connect();
}

/**
 * @brief Disconnect from AP
 */
esp_err_t wifi_sta_wrapper_disconnect(void)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return esp_wifi_disconnect();
}

/**
 * @brief Connect to specific AP with SSID and password
 */
esp_err_t wifi_sta_connect_to(const char *ssid, const char *password)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "SSID or password is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t sta_config = {0};
    strncpy((char*)sta_config.sta.ssid, ssid, sizeof(sta_config.sta.ssid) - 1);
    strncpy((char*)sta_config.sta.password, password, sizeof(sta_config.sta.password) - 1);

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STA config: %s", esp_err_to_name(ret));
        return ret;
    }

    strncpy(s_sta_config.ssid, ssid, sizeof(s_sta_config.ssid) - 1);
    strncpy(s_sta_config.password, password, sizeof(s_sta_config.password) - 1);

    s_retry_count = 0;
    return esp_wifi_connect();
}

/**
 * @brief Set station configuration
 */
esp_err_t wifi_sta_set_config(const char *ssid, const char *password)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "SSID or password is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t sta_config = {0};
    strncpy((char*)sta_config.sta.ssid, ssid, sizeof(sta_config.sta.ssid) - 1);
    strncpy((char*)sta_config.sta.password, password, sizeof(sta_config.sta.password) - 1);

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STA config: %s", esp_err_to_name(ret));
        return ret;
    }

    strncpy(s_sta_config.ssid, ssid, sizeof(s_sta_config.ssid) - 1);
    strncpy(s_sta_config.password, password, sizeof(s_sta_config.password) - 1);

    return ESP_OK;
}

/**
 * @brief Set AP configuration
 */
esp_err_t wifi_ap_set_config(const char *ssid, const char *password, uint8_t channel)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "SSID or password is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t ap_config = {0};
    strncpy((char*)ap_config.ap.ssid, ssid, sizeof(ap_config.ap.ssid) - 1);
    strncpy((char*)ap_config.ap.password, password, sizeof(ap_config.ap.password) - 1);
    ap_config.ap.channel = channel;
    ap_config.ap.max_connection = s_ap_config.max_connection;
    ap_config.ap.authmode = s_ap_config.authmode;

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set AP config: %s", esp_err_to_name(ret));
        return ret;
    }

    strncpy(s_ap_config.ssid, ssid, sizeof(s_ap_config.ssid) - 1);
    strncpy(s_ap_config.password, password, sizeof(s_ap_config.password) - 1);
    s_ap_config.channel = channel;

    return ESP_OK;
}

/**
 * @brief Get list of connected stations in AP mode
 */
esp_err_t wifi_ap_get_sta_list(wifi_sta_list_t *sta_list)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sta_list == NULL) {
        ESP_LOGE(TAG, "sta_list is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    return esp_wifi_ap_get_sta_list(sta_list);
}

/**
 * @brief Start WiFi scan
 */
esp_err_t wifi_scan_start(void)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return esp_wifi_scan_start(NULL, false);
}

/**
 * @brief Stop WiFi scan
 */
esp_err_t wifi_scan_stop(void)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return esp_wifi_scan_stop();
}

/**
 * @brief Get scan results
 */
esp_err_t wifi_scan_get_ap_list(wifi_ap_record_t **ap_list, uint16_t *ap_count)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (ap_list == NULL || ap_count == NULL) {
        ESP_LOGE(TAG, "ap_list or ap_count is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    return esp_wifi_scan_get_ap_records(ap_count, *ap_list);
}

/**
 * @brief Register event callback
 */
esp_err_t wifi_register_callback(wifi_evt_cb_t callback)
{
    s_event_callback = callback;
    return ESP_OK;
}

/**
 * @brief Set WiFi mode
 */
esp_err_t wifi_set_mode(wifi_mode_select_t mode)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    wifi_mode_t hw_mode;
    if (mode == WIFI_MODE_SELECT_STA) {
        hw_mode = WIFI_MODE_STA;
    } else if (mode == WIFI_MODE_SELECT_AP) {
        hw_mode = WIFI_MODE_AP;
    } else if (mode == WIFI_MODE_SELECT_APSTA) {
        hw_mode = WIFI_MODE_APSTA;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_wifi_set_mode(hw_mode);
    if (ret != ESP_OK) {
        return ret;
    }

    s_wifi_mode = mode;
    return ESP_OK;
}

/**
 * @brief Get current WiFi mode
 */
wifi_mode_select_t wifi_get_mode(void)
{
    return s_wifi_mode;
}

/**
 * @brief Check if station is connected
 */
bool wifi_sta_is_connected(void)
{
    return s_sta_connected;
}

/**
 * @brief Check if AP is active
 */
bool wifi_ap_is_active(void)
{
    return s_ap_active;
}

/**
 * @brief Get IP information
 */
esp_err_t wifi_get_ip_info(uint32_t *ip, uint32_t *netmask, uint32_t *gw)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    esp_netif_ip_info_t ip_info;
    esp_err_t ret = esp_netif_get_ip_info(netif, &ip_info);
    if (ret != ESP_OK) {
        return ret;
    }

    if (ip) *ip = ip_info.ip.addr;
    if (netmask) *netmask = ip_info.netmask.addr;
    if (gw) *gw = ip_info.gw.addr;

    return ESP_OK;
}

/**
 * @brief Get station information
 */
esp_err_t wifi_get_sta_info(wifi_ap_record_t *ap_info)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (ap_info == NULL) {
        ESP_LOGE(TAG, "ap_info is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    return esp_wifi_sta_get_ap_info(ap_info);
}

/**
 * @brief Get RSSI of connected AP
 */
int8_t wifi_get_rssi(void)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    }
    return -127;
}

/**
 * @brief Get current channel
 */
uint8_t wifi_get_channel(void)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.primary;
    }
    return 0;
}

/**
 * @brief Set power save mode
 */
esp_err_t wifi_set_power_save(wifi_ps_type_t power_save)
{
    if (!s_wifi_initialized) {
        ESP_LOGE(TAG, "WiFi not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return esp_wifi_set_ps(power_save);
}
