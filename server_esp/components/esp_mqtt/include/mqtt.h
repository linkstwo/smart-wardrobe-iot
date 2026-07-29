/**
 * @file mqtt.h
 * @brief MQTT component header file
 *
 * This file defines the public API for the MQTT component, providing
 * publish/subscribe messaging for ESP32 communication with a remote server.
 *
 * 中文说明：
 * 本头文件定义 ESP32 MQTT 组件对外接口。
 * 当前项目中，MQTT 组件用于连接阿里云 IoT：
 * - 订阅智能衣柜自定义 Topic，接收云端下发的 JSON 命令。
 * - 发布 STM32 返回的成功/故障/状态 JSON 到云端。
 */

#ifndef MQTT_H
#define MQTT_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "mqtt_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MQTT_EVT_INIT_COMPLETE,      /**< MQTT 组件初始化完成 */
    MQTT_EVT_DEINIT_COMPLETE,    /**< MQTT 组件反初始化完成 */
    MQTT_EVT_CONNECTED,          /**< 已连接到 Broker */
    MQTT_EVT_DISCONNECTED,       /**< 与 Broker 断开 */
    MQTT_EVT_SUBSCRIBED,         /**< Topic 订阅成功 */
    MQTT_EVT_UNSUBSCRIBED,       /**< Topic 取消订阅 */
    MQTT_EVT_PUBLISHED,          /**< 消息发布完成 */
    MQTT_EVT_DATA_RECEIVED,      /**< 收到 MQTT 下行数据 */
    MQTT_EVT_ERROR,              /**< MQTT 错误事件 */
} mqtt_evt_type_t;

typedef struct {
    char topic[MQTT_MAX_TOPIC_LEN];       /**< 收到消息的 Topic */
    char data[MQTT_MAX_PAYLOAD_LEN];      /**< 收到的 payload 数据缓存 */
    size_t data_len;                      /**< payload 实际长度 */
} mqtt_rx_data_t;

typedef struct {
    mqtt_evt_type_t type;                 /**< MQTT 事件类型 */
    int status;                           /**< 事件状态码 */
    union {
        struct {
            char topic[MQTT_MAX_TOPIC_LEN];   /**< 收到数据的 Topic */
            size_t data_len;                  /**< 收到数据长度 */
        } data_rx;
        struct {
            char topic[MQTT_MAX_TOPIC_LEN];   /**< 已发布消息的 Topic */
        } publish;
    } data;
} mqtt_evt_data_t;

/* MQTT 事件回调类型，应用层用它观察连接、订阅、收包、发布等状态。 */
typedef void (*mqtt_evt_cb_t)(mqtt_evt_data_t *event_data);

/* Topic 处理函数类型：收到指定 Topic 的 payload 后由组件调用。 */
typedef esp_err_t (*mqtt_topic_handler_t)(const char *topic,
                                           const uint8_t *payload,
                                           size_t payload_len);

typedef struct {
    char topic[MQTT_MAX_TOPIC_LEN];       /**< 要订阅并路由的 Topic */
    mqtt_topic_handler_t handler;         /**< 该 Topic 对应的处理函数 */
} mqtt_topic_handler_cfg_t;

typedef struct {
    const char *broker_host;              /**< Broker 地址，例如阿里云 iothub 域名 */
    uint16_t broker_port;                 /**< Broker 端口，当前默认 1883 */
    const char *client_id;                /**< MQTT client_id，阿里云设备认证需要 */
    const char *username;                 /**< MQTT 用户名，通常为 DeviceName&ProductKey */
    const char *password;                 /**< MQTT 密码，通常为 DeviceSecret 签名结果 */
    uint32_t keepalive_sec;               /**< MQTT 心跳保活时间 */
    uint32_t reconnect_interval_ms;       /**< 断线重连间隔 */
    bool use_ssl;                         /**< 是否使用 SSL/TLS */
    const char *cert_pem;                 /**< SSL 根证书 PEM，未启用 SSL 时为 NULL */
    uint32_t task_stack_size;             /**< MQTT 事件任务栈大小 */
    int task_priority;                    /**< MQTT 事件任务优先级 */
    mqtt_evt_cb_t event_callback;         /**< 事件回调函数 */
} mqtt_cfg_t;

esp_err_t mqtt_init(mqtt_cfg_t *config);
esp_err_t mqtt_deinit(void);

esp_err_t mqtt_connect(void);
esp_err_t mqtt_disconnect(void);
bool mqtt_is_connected(void);

esp_err_t mqtt_subscribe(const char *topic, mqtt_topic_handler_t handler);
esp_err_t mqtt_unsubscribe(const char *topic);

esp_err_t mqtt_publish(const char *topic, const uint8_t *payload,
                        size_t payload_len, int qos);
esp_err_t mqtt_publish_str(const char *topic, const char *payload, int qos);

esp_err_t mqtt_register_callback(mqtt_evt_cb_t callback);

/**
 * @brief MQTT wardrobe command topic handler
 *
 * Handles wardrobe commands on the Aliyun custom wardrobe topic. Parses JSON command
 * and sends protocol frames via UART.
 *
 * 中文说明：
 * 这是智能衣柜下行命令处理函数。
 * 云端通过 Pub 发送 JSON，例如 {"cmd":17,"position":1}，
 * 该函数解析后会生成 STM32 UART 帧，例如 A5 11 01 01 11 5A。
 *
 * @param topic Topic string
 * @param payload Message payload
 * @param payload_len Payload length
 * @return ESP_OK on success
 */
esp_err_t mqtt_wardrobe_command_handler(const char *topic,
                                         const uint8_t *payload,
                                         size_t payload_len);

#ifdef __cplusplus
}
#endif

#endif
