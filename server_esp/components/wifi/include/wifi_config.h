/**
 * @file wifi_config.h
 * @brief WiFi component configuration header
 * 
 * This file contains compile-time configuration macros for the WiFi component.
 * These can be overridden by menuconfig or sdkconfig settings.
 *
 * 中文说明：
 * 本文件定义 WiFi 默认连接参数。
 * 当前智能衣柜项目主要使用 STA 模式连接路由器，使 ESP32 能访问阿里云 MQTT。
 * 如果现场 WiFi 名称或密码改变，应优先修改 CONFIG_WIFI_STA_SSID/PASSWORD
 * 或通过 menuconfig/sdkconfig 覆盖这些默认值。
 */

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

/** Maximum length of SSID string */
#define WIFI_SSID_MAX_LEN          32

/** Maximum length of password string */
#define WIFI_PASSWORD_MAX_LEN      64

/** STA 连接失败后的最大重试次数。 */
#ifndef CONFIG_WIFI_MAX_RETRY_COUNT
#define CONFIG_WIFI_MAX_RETRY_COUNT    5
#endif

/** STA 模式默认连接的路由器 SSID。 */
#ifndef CONFIG_WIFI_STA_SSID
#define CONFIG_WIFI_STA_SSID           "TPH"
#endif

/** STA 模式默认连接的路由器密码。 */
#ifndef CONFIG_WIFI_STA_PASSWORD
#define CONFIG_WIFI_STA_PASSWORD       "666888tph"
#endif

/** AP 模式默认热点名称，当前完整云端链路暂不依赖 AP 模式。 */
#ifndef CONFIG_WIFI_AP_SSID
#define CONFIG_WIFI_AP_SSID            "ESP32_AP"
#endif

/** AP 模式默认热点密码。 */
#ifndef CONFIG_WIFI_AP_PASSWORD
#define CONFIG_WIFI_AP_PASSWORD        "12345678"
#endif

/** Default channel for AP mode */
#ifndef CONFIG_WIFI_AP_CHANNEL
#define CONFIG_WIFI_AP_CHANNEL         1
#endif

/** Maximum connections for AP mode */
#ifndef CONFIG_WIFI_AP_MAX_CONN
#define CONFIG_WIFI_AP_MAX_CONN        4
#endif

/** Scan method for station mode */
#ifndef CONFIG_WIFI_STA_SCAN_METHOD
#define CONFIG_WIFI_STA_SCAN_METHOD    WIFI_ALL_CHANNEL_SCAN
#endif

/** Sort method for AP selection */
#ifndef CONFIG_WIFI_STA_SORT_METHOD
#define CONFIG_WIFI_STA_SORT_METHOD    WIFI_CONNECT_AP_BY_SIGNAL
#endif

/** RSSI threshold for connection */
#ifndef CONFIG_WIFI_STA_RSSI_THRESHOLD
#define CONFIG_WIFI_STA_RSSI_THRESHOLD -127
#endif

/** Beacon interval for AP mode (in ms) */
#ifndef CONFIG_WIFI_AP_BEACON_INTERVAL
#define CONFIG_WIFI_AP_BEACON_INTERVAL 100
#endif

#endif
