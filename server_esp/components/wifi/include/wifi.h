/**
 * @file wifi.h
 * @brief WiFi component header file
 * 
 * This file defines the public API for the WiFi component, supporting
 * STA (Station), AP (Access Point), and APSTA modes with event-driven
 * callbacks for various WiFi events.
 *
 * 中文说明：
 * 本头文件定义 WiFi 组件的配置结构、事件类型和对外 API。
 * 当前智能衣柜项目主要使用 STA 模式连接路由器，从而让 MQTT 连接阿里云。
 */

#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_wifi.h"
#include "wifi_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi operation mode selection
 */
typedef enum {
    WIFI_MODE_SELECT_NONE = 0,      /**< 未选择模式 */
    WIFI_MODE_SELECT_STA,           /**< STA 模式：连接外部路由器，当前项目主要使用 */
    WIFI_MODE_SELECT_AP,            /**< AP 模式：ESP32 自己开热点 */
    WIFI_MODE_SELECT_APSTA,         /**< AP+STA 模式：同时开热点并连接路由器 */
} wifi_mode_select_t;

/**
 * @brief WiFi event types
 */
typedef enum {
    WIFI_EVT_INIT_COMPLETE,         /**< WiFi initialization complete */
    WIFI_EVT_DEINIT_COMPLETE,       /**< WiFi deinitialization complete */
    WIFI_EVT_STA_START,             /**< Station started */
    WIFI_EVT_STA_STOP,              /**< Station stopped */
    WIFI_EVT_STA_CONNECTED,         /**< Station connected to AP */
    WIFI_EVT_STA_DISCONNECTED,      /**< Station disconnected from AP */
    WIFI_EVT_STA_AUTH_MODE_CHANGE,  /**< Authentication mode changed */
    WIFI_EVT_STA_GOT_IP,            /**< Station obtained IP address */
    WIFI_EVT_STA_LOST_IP,           /**< Station lost IP address */
    WIFI_EVT_STA_WPS_SUCCESS,       /**< WPS connection successful */
    WIFI_EVT_STA_WPS_FAILED,        /**< WPS connection failed */
    WIFI_EVT_STA_WPS_TIMEOUT,       /**< WPS connection timeout */
    WIFI_EVT_AP_START,              /**< Access Point started */
    WIFI_EVT_AP_STOP,               /**< Access Point stopped */
    WIFI_EVT_AP_STACONNECTED,       /**< Station connected to AP */
    WIFI_EVT_AP_STADISCONNECTED,    /**< Station disconnected from AP */
    WIFI_EVT_AP_PROBEREQRECVED,     /**< Probe request received */
    WIFI_EVT_SCAN_DONE,             /**< WiFi scan complete */
    WIFI_EVT_CONNECTION_TIMEOUT,    /**< Connection timeout */
    WIFI_EVT_CONNECTION_FAILED,     /**< Connection failed */
} wifi_evt_type_t;

/**
 * @brief WiFi event data union
 */
typedef struct {
    wifi_evt_type_t type;           /**< Event type */
    int status;                     /**< Event status code */
    union {
        struct {
            uint8_t ssid[32];       /**< SSID of connected AP */
            uint8_t bssid[6];       /**< BSSID of connected AP */
            uint8_t channel;        /**< Channel number */
            uint8_t authmode;       /**< Authentication mode */
            int8_t rssi;            /**< Signal strength */
        } sta_info;                 /**< Station information */
        struct {
            uint8_t mac[6];         /**< MAC address of connected station */
            uint8_t aid;            /**< Association ID */
        } ap_sta_info;              /**< AP station information */
        struct {
            uint8_t mac[6];         /**< MAC address of probing device */
            int8_t rssi;            /**< Signal strength */
        } probe_info;               /**< Probe request information */
        struct {
            uint32_t ip;            /**< IP address */
            uint32_t netmask;       /**< Subnet mask */
            uint32_t gw;            /**< Gateway address */
        } ip_info;                  /**< IP information */
        struct {
            uint8_t num;            /**< Number of scan results */
            uint8_t ssid[32];       /**< SSID */
            uint8_t bssid[6];       /**< BSSID */
            uint8_t channel;        /**< Channel number */
            int8_t rssi;            /**< Signal strength */
            uint8_t authmode;       /**< Authentication mode */
        } scan_result;              /**< Scan result information */
    } data;
} wifi_evt_data_t;

/**
 * @brief WiFi event callback function type
 */
typedef void (*wifi_evt_cb_t)(wifi_evt_data_t *event_data);

/**
 * @brief Station mode configuration
 */
typedef struct {
    char ssid[WIFI_SSID_MAX_LEN];           /**< 要连接的路由器 SSID */
    char password[WIFI_PASSWORD_MAX_LEN];   /**< 路由器密码 */
    uint8_t channel;                        /**< 指定信道，0 表示自动扫描 */
    wifi_ps_type_t power_save;              /**< 省电模式，云端实时控制建议关闭省电 */
    bool auto_reconnect;                    /**< 断线后是否自动重连 */
    uint8_t max_retry;                      /**< 最大重连次数 */
    wifi_scan_method_t scan_method;         /**< 扫描方式 */
    wifi_sort_method_t sort_method;         /**< 多个 AP 时的选择排序方式 */
    int8_t rssi_threshold;                  /**< 连接信号强度门限 */
} wifi_sta_cfg_t;

/**
 * @brief Access Point mode configuration
 */
typedef struct {
    char ssid[WIFI_SSID_MAX_LEN];           /**< ESP32 热点名称 */
    char password[WIFI_PASSWORD_MAX_LEN];   /**< ESP32 热点密码 */
    uint8_t channel;                        /**< AP 信道 */
    uint8_t max_connection;                 /**< AP 最大连接设备数 */
    wifi_auth_mode_t authmode;              /**< AP 认证方式 */
    uint16_t beacon_interval;               /**< Beacon 间隔 */
    bool ssid_hidden;                       /**< 是否隐藏热点名称 */
} wifi_ap_cfg_t;

/**
 * @brief WiFi configuration structure
 */
typedef struct {
    wifi_mode_select_t mode;        /**< WiFi 模式选择 */
    wifi_sta_cfg_t sta;             /**< STA 连接路由器配置 */
    wifi_ap_cfg_t ap;               /**< AP 热点配置 */
    wifi_evt_cb_t event_callback;   /**< WiFi 事件回调 */
} wifi_cfg_t;

/**
 * @brief Initialize WiFi component
 * 
 * @param config Pointer to WiFi configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_init(wifi_cfg_t *config);

/**
 * @brief Deinitialize WiFi component
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_wrapper_deinit(void);

/**
 * @brief Connect to configured AP in station mode
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_sta_connect(void);

/**
 * @brief Disconnect from AP in station mode
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_sta_wrapper_disconnect(void);

/**
 * @brief Connect to specific AP
 * 
 * @param ssid SSID of the AP
 * @param password Password for the AP
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_sta_connect_to(const char *ssid, const char *password);

/**
 * @brief Set station configuration
 * 
 * @param ssid SSID to configure
 * @param password Password to configure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_sta_set_config(const char *ssid, const char *password);

/**
 * @brief Set Access Point configuration
 * 
 * @param ssid AP SSID
 * @param password AP password
 * @param channel AP channel
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_ap_set_config(const char *ssid, const char *password, uint8_t channel);

/**
 * @brief Get list of connected stations in AP mode
 * 
 * @param sta_list Pointer to store station list
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_ap_get_sta_list(wifi_sta_list_t *sta_list);

/**
 * @brief Start WiFi scan
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_scan_start(void);

/**
 * @brief Stop WiFi scan
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_scan_stop(void);

/**
 * @brief Get scan results
 * 
 * @param ap_list Pointer to store AP list
 * @param ap_count Pointer to store AP count
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_scan_get_ap_list(wifi_ap_record_t **ap_list, uint16_t *ap_count);

/**
 * @brief Register event callback
 * 
 * @param callback Callback function pointer
 * @return ESP_OK on success
 */
esp_err_t wifi_register_callback(wifi_evt_cb_t callback);

/**
 * @brief Set WiFi mode
 * 
 * @param mode WiFi mode to set
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_set_mode(wifi_mode_select_t mode);

/**
 * @brief Get current WiFi mode
 * 
 * @return Current WiFi mode
 */
wifi_mode_select_t wifi_get_mode(void);

/**
 * @brief Check if station is connected
 * 
 * @return true if connected, false otherwise
 */
bool wifi_sta_is_connected(void);

/**
 * @brief Check if AP is active
 * 
 * @return true if active, false otherwise
 */
bool wifi_ap_is_active(void);

/**
 * @brief Get IP information
 * 
 * @param ip Pointer to store IP address
 * @param netmask Pointer to store netmask
 * @param gw Pointer to store gateway
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_get_ip_info(uint32_t *ip, uint32_t *netmask, uint32_t *gw);

/**
 * @brief Get station information
 * 
 * @param ap_info Pointer to store AP information
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_get_sta_info(wifi_ap_record_t *ap_info);

/**
 * @brief Get RSSI of connected AP
 * 
 * @return RSSI value, -127 on error
 */
int8_t wifi_get_rssi(void);

/**
 * @brief Get current channel
 * 
 * @return Channel number, 0 on error
 */
uint8_t wifi_get_channel(void);

/**
 * @brief Set power save mode
 * 
 * @param power_save Power save type
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_set_power_save(wifi_ps_type_t power_save);

#ifdef __cplusplus
}
#endif

#endif
