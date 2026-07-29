/**
 * @file mqtt.c
 * @brief MQTT component implementation
 *
 * This file implements the MQTT component, providing publish/subscribe
 * messaging to replace HTTP-based communication with the remote server.
 *
 * 中文说明：
 * 本文件封装 ESP-IDF 的 MQTT 客户端，并额外实现智能衣柜云端命令处理。
 * 在当前项目中，阿里云 OpenAPI Pub 发送 JSON 命令到自定义 Topic，
 * ESP32 收到后由 mqtt_wardrobe_command_handler() 解析 cmd/position，
 * 再组装为 STM32 能识别的 UART 二进制帧。
 *
 * 云端到 STM32 的核心转换关系：
 * {"cmd":16,"position":1} -> A5 10 01 01 10 5A  存衣服
 * {"cmd":17,"position":1} -> A5 11 01 01 11 5A  取衣服
 * {"cmd":18}              -> A5 12 00 12 5A     复位回零
 * {"cmd":32}              -> A5 20 00 20 5A     读取状态
 *
 * 注意：
 * 当前自定义 Topic 同时用于下行命令和上行响应，因此收到带 status 字段
 * 的消息时会认为它是 ESP32/STM32 自己上报的响应并忽略，避免自触发循环。
 */

#include "mqtt.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "../uart/include/uart.h"
#include "cJSON.h"

static const char *TAG = "MQTT";

/* MQTT 组件初始化状态，防止重复创建队列、任务和客户端。 */
static bool s_initialized = false;
/* Broker 连接状态，发布/订阅前用它判断链路是否可用。 */
static bool s_connected = false;

/* 当前 MQTT 运行配置，来自 app_init_mqtt() 传入的阿里云连接参数。 */
static mqtt_cfg_t s_config = {0};
/* 应用层事件回调，用于把连接、订阅、收包等状态通知给 app_init.c。 */
static mqtt_evt_cb_t s_event_callback = NULL;

/* ESP-IDF mqtt_client 组件的句柄，真正执行 connect/publish/subscribe。 */
static esp_mqtt_client_handle_t s_mqtt_client = NULL;

/* Topic 到处理函数的注册表；收到 MQTT 数据后按 Topic 路由到对应 handler。 */
static mqtt_topic_handler_cfg_t s_topic_handlers[MQTT_MAX_SUBSCRIBE_COUNT];
static uint8_t s_topic_handler_count = 0;

/* MQTT 事件队列和事件任务，用于把 ESP-IDF 回调中的事件转到普通任务上下文处理。 */
static QueueHandle_t s_event_queue = NULL;
static TaskHandle_t s_event_task = NULL;

/* 智能衣柜使用的阿里云自定义 Topic，实际字符串在 mqtt_config.h 中配置。 */
#define WARDROBE_TOPIC                WARDROBE_MQTT_TOPIC
/* 当前工程只添加了 UART 端口 0，因此 MQTT 下行命令固定写入这个逻辑端口。 */
#define WARDROBE_UART_PORT_ID         0
/* STM32 二进制帧固定帧头/帧尾。 */
#define WARDROBE_FRAME_SOF            0xA5
#define WARDROBE_FRAME_EOF            0x5A
/* STM32 智能衣柜协议命令码。 */
#define WARDROBE_CMD_STORE            0x10
#define WARDROBE_CMD_TAKE             0x11
#define WARDROBE_CMD_RESET_HOME       0x12
#define WARDROBE_CMD_READ_STATUS      0x20
/* 通用未知故障码，用于 JSON 格式错误、命令非法、位置不支持等情况。 */
#define WARDROBE_FAULT_UNKNOWN        0xFF
/* STORE/TAKE both use the requested laser marker number, range 1..255. */

typedef struct {
    /* 转发给应用层的 MQTT 事件类型。 */
    mqtt_evt_type_t type;
    /* 事件状态码，错误时可携带 ESP_FAIL 等状态。 */
    int status;
    /* 事件相关 Topic，例如收到消息或发布消息的 Topic。 */
    char topic[MQTT_MAX_TOPIC_LEN];
    /* 事件相关数据长度，例如收到的 payload 长度。 */
    size_t data_len;
} mqtt_event_queue_item_t;

/* 计算 STM32 智能衣柜帧校验：CRC = CMD ^ LEN ^ DATA... */
static uint8_t wardrobe_calc_crc(uint8_t cmd, uint8_t data_len,
                                 const uint8_t *data)
{
    uint8_t crc = cmd ^ data_len;

    for (uint8_t i = 0; i < data_len; i++) {
        crc ^= data[i];
    }

    return crc;
}

/* 生成上报 JSON 使用的时间戳；未同步时间时返回 0。 */
static int64_t wardrobe_timestamp(void)
{
    time_t now = time(NULL);
    return now > 0 ? (int64_t)now : 0;
}

/*
 * 发布智能衣柜故障 JSON 到阿里云自定义 Topic。
 * 该函数用于 MQTT payload 非法、命令不支持、UART 写失败等 ESP32 本地发现的错误。
 */
static esp_err_t wardrobe_publish_fault(uint8_t code, const char *msg,
                                        int position)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(root, "status", 0x91);
    cJSON_AddNumberToObject(root, "code", code);
    cJSON_AddStringToObject(root, "msg", msg ? msg : "unknown");
    cJSON_AddNumberToObject(root, "position", position);
    cJSON_AddNumberToObject(root, "timestamp", wardrobe_timestamp());

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json_str) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = mqtt_publish_str(WARDROBE_TOPIC, json_str, 1);
    free(json_str);
    return ret;
}

/**
 * @brief Notify application of MQTT event
 *
 * @param type Event type
 * @param status Event status code
 * @param topic Topic string (optional)
 * @param data_len Data length (optional)
 */
static void mqtt_notify_event(mqtt_evt_type_t type, int status,
                               const char *topic, size_t data_len)
{
    /*
     * ESP-IDF MQTT 回调可能发生在组件内部任务中。
     * 这里先把事件放入本组件队列，再由 mqtt_event_task() 调用用户回调，
     * 可减少回调里做较重操作时对 ESP-IDF MQTT 事件处理的影响。
     */
    if (s_event_queue) {
        mqtt_event_queue_item_t item = {0};
        item.type = type;
        item.status = status;
        if (topic) {
            strncpy(item.topic, topic, MQTT_MAX_TOPIC_LEN - 1);
        }
        item.data_len = data_len;

        if (xQueueSend(s_event_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "Failed to send event to queue, type=%d", type);
        }
    } else {
        if (s_event_callback) {
            mqtt_evt_data_t evt_data = {0};
            evt_data.type = type;
            evt_data.status = status;
            if (topic) {
                strncpy(evt_data.data.data_rx.topic, topic,
                        sizeof(evt_data.data.data_rx.topic) - 1);
            }
            evt_data.data.data_rx.data_len = data_len;
            s_event_callback(&evt_data);
        }
    }
}

/**
 * @brief MQTT event handler callback from ESP-MQTT library
 *
 * Called when any MQTT event occurs (connected, disconnected,
 * data received, subscribed, published, error).
 *
 * @param handler_args User context (unused)
 * @param base Event base
 * @param event_id Event ID
 * @param event_data Event data
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t evt = (esp_mqtt_event_handle_t)event_data;

    (void)handler_args;
    (void)base;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected to broker");
        s_connected = true;
        mqtt_notify_event(MQTT_EVT_CONNECTED, 0, NULL, 0);

        /*
         * 断线重连后 ESP-IDF 客户端会重新连接 Broker，
         * 这里把之前注册过的 Topic 全部重新订阅，保证云端命令还能下发。
         */
        for (int i = 0; i < s_topic_handler_count; i++) {
            int msg_id = esp_mqtt_client_subscribe(s_mqtt_client,
                                                    s_topic_handlers[i].topic, 1);
            ESP_LOGI(TAG, "Subscribed to topic: %s, msg_id=%d",
                     s_topic_handlers[i].topic, msg_id);
            mqtt_notify_event(MQTT_EVT_SUBSCRIBED, 0,
                              s_topic_handlers[i].topic, 0);
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected from broker");
        s_connected = false;
        mqtt_notify_event(MQTT_EVT_DISCONNECTED, 0, NULL, 0);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT subscribed, msg_id=%d", evt->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT unsubscribed, msg_id=%d", evt->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "MQTT published, msg_id=%d", evt->msg_id);
        mqtt_notify_event(MQTT_EVT_PUBLISHED, 0, evt->topic, evt->data_len);
        break;

    case MQTT_EVENT_DATA: {
        ESP_LOGI(TAG, "MQTT data received: topic=%.*s, data_len=%d",
                 evt->topic_len, evt->topic, evt->data_len);

        mqtt_notify_event(MQTT_EVT_DATA_RECEIVED, 0, evt->topic, evt->data_len);

        /*
         * 按 Topic 精确匹配处理函数。
         * 当前智能衣柜 Topic 会匹配到 mqtt_wardrobe_command_handler()，
         * 由该函数继续解析 JSON 并转 UART 帧。
         */
        for (int i = 0; i < s_topic_handler_count; i++) {
            size_t top_len = strlen(s_topic_handlers[i].topic);
            if (evt->topic_len == top_len &&
                strncmp(evt->topic, s_topic_handlers[i].topic, top_len) == 0) {
                if (s_topic_handlers[i].handler) {
                    s_topic_handlers[i].handler(evt->topic,
                                                 (const uint8_t *)evt->data,
                                                 evt->data_len);
                }
                break;
            }
        }
        break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        mqtt_notify_event(MQTT_EVT_ERROR, ESP_FAIL, NULL, 0);
        break;

    default:
        break;
    }
}

/**
 * @brief MQTT event task
 *
 * Processes events from the event queue and calls the registered callback.
 *
 * @param pvParameters Task parameters (unused)
 */
static void mqtt_event_task(void *pvParameters)
{
    mqtt_event_queue_item_t item;

    (void)pvParameters;

    while (s_initialized && s_event_queue) {
        if (xQueueReceive(s_event_queue, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (s_event_callback) {
                mqtt_evt_data_t evt_data = {0};
                evt_data.type = item.type;
                evt_data.status = item.status;

                switch (item.type) {
                case MQTT_EVT_DATA_RECEIVED:
                    strncpy(evt_data.data.data_rx.topic, item.topic,
                            sizeof(evt_data.data.data_rx.topic) - 1);
                    evt_data.data.data_rx.data_len = item.data_len;
                    break;
                case MQTT_EVT_PUBLISHED:
                    strncpy(evt_data.data.publish.topic, item.topic,
                            sizeof(evt_data.data.publish.topic) - 1);
                    break;
                default:
                    break;
                }

                s_event_callback(&evt_data);
            }
        }
    }

    vTaskDelete(NULL);
}

esp_err_t mqtt_init(mqtt_cfg_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "MQTT already initialized");
        return ESP_OK;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&s_config, config, sizeof(mqtt_cfg_t));
    s_event_callback = config->event_callback;
    s_topic_handler_count = 0;
    memset(s_topic_handlers, 0, sizeof(s_topic_handlers));

    /* 创建 MQTT 事件队列，供 mqtt_event_handler() 和 mqtt_event_task() 解耦。 */
    s_event_queue = xQueueCreate(MQTT_EVENT_QUEUE_SIZE,
                                  sizeof(mqtt_event_queue_item_t));
    if (s_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return ESP_ERR_NO_MEM;
    }

    /* 创建事件分发任务，把队列事件转成应用层 mqtt_evt_data_t 回调。 */
    uint32_t stack_size = config->task_stack_size > 0 ?
                          config->task_stack_size : MQTT_EVT_TASK_STACK_SIZE;
    int priority = config->task_priority > 0 ?
                   config->task_priority : MQTT_EVT_TASK_PRIORITY;

    s_initialized = true;

    BaseType_t task_ret = xTaskCreate(mqtt_event_task, "mqtt_evt",
                                       stack_size, NULL, priority,
                                       &s_event_task);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create event task");
        s_initialized = false;
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return ESP_FAIL;
    }

    mqtt_notify_event(MQTT_EVT_INIT_COMPLETE, 0, NULL, 0);

    ESP_LOGI(TAG, "MQTT initialized, broker: %s:%d, client_id: %s",
             config->broker_host, config->broker_port, config->client_id);

    return ESP_OK;
}

esp_err_t mqtt_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    if (s_mqtt_client) {
        mqtt_disconnect();
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
    }

    s_initialized = false;
    s_connected = false;
    s_event_callback = NULL;
    s_topic_handler_count = 0;

    if (s_event_task) {
        vTaskDelay(pdMS_TO_TICKS(150));
        vTaskDelete(s_event_task);
        s_event_task = NULL;
    }

    if (s_event_queue) {
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
    }

    mqtt_notify_event(MQTT_EVT_DEINIT_COMPLETE, 0, NULL, 0);
    ESP_LOGI(TAG, "MQTT deinitialized");

    return ESP_OK;
}

esp_err_t mqtt_connect(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "MQTT not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_mqtt_client) {
        ESP_LOGW(TAG, "MQTT already connected");
        return ESP_OK;
    }

    char broker_uri[MQTT_MAX_TOPIC_LEN];
    /* 根据是否启用 SSL 选择 mqtt:// 或 mqtts://。当前默认端口 1883 通常为非 SSL。 */
    const char *scheme = s_config.use_ssl ? "mqtts" : "mqtt";
    snprintf(broker_uri, sizeof(broker_uri), "%s://%s:%d",
             scheme, s_config.broker_host, s_config.broker_port);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
        .credentials.client_id = s_config.client_id,
        .session.keepalive = s_config.keepalive_sec > 0 ?
                              s_config.keepalive_sec : MQTT_DEFAULT_KEEPALIVE_SEC,
        .network.disable_auto_reconnect = false,
        .network.reconnect_timeout_ms = s_config.reconnect_interval_ms > 0 ?
                           s_config.reconnect_interval_ms : MQTT_DEFAULT_RECONNECT_MS,
    };

    /* 阿里云 IoT 需要 username/password 时在这里填入三元组签名生成的参数。 */
    if (s_config.username && strlen(s_config.username) > 0) {
        mqtt_cfg.credentials.username = s_config.username;
    }
    if (s_config.password && strlen(s_config.password) > 0) {
        mqtt_cfg.credentials.authentication.password = s_config.password;
    }

    if (s_config.use_ssl && s_config.cert_pem && strlen(s_config.cert_pem) > 0) {
        mqtt_cfg.broker.verification.certificate = s_config.cert_pem;
    }

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        return ESP_FAIL;
    }

    esp_err_t ret = esp_mqtt_client_register_event(s_mqtt_client,
                                                     ESP_EVENT_ANY_ID,
                                                     mqtt_event_handler,
                                                     NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT event handler");
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
        return ret;
    }

    ret = esp_mqtt_client_start(s_mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client");
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "MQTT client connecting to %s", broker_uri);

    return ESP_OK;
}

esp_err_t mqtt_disconnect(void)
{
    if (!s_mqtt_client) {
        return ESP_OK;
    }

    esp_err_t ret = esp_mqtt_client_stop(s_mqtt_client);
    esp_mqtt_client_destroy(s_mqtt_client);
    s_mqtt_client = NULL;
    s_connected = false;

    mqtt_notify_event(MQTT_EVT_DISCONNECTED, 0, NULL, 0);
    ESP_LOGI(TAG, "MQTT client disconnected");

    return ret;
}

bool mqtt_is_connected(void)
{
    return s_connected && s_mqtt_client != NULL;
}

esp_err_t mqtt_subscribe(const char *topic, mqtt_topic_handler_t handler)
{
    if (topic == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_topic_handler_count >= MQTT_MAX_SUBSCRIBE_COUNT) {
        ESP_LOGE(TAG, "Max topic handlers reached");
        return ESP_ERR_NO_MEM;
    }

    /*
     * 先登记到本地 handler 表。
     * 即使当前尚未连接 Broker，连接成功事件里也会自动补订阅。
     */
    strncpy(s_topic_handlers[s_topic_handler_count].topic, topic,
            MQTT_MAX_TOPIC_LEN - 1);
    s_topic_handlers[s_topic_handler_count].topic[MQTT_MAX_TOPIC_LEN - 1] = '\0';
    s_topic_handlers[s_topic_handler_count].handler = handler;
    s_topic_handler_count++;

    if (s_mqtt_client && s_connected) {
        /* 如果已经在线，则立即向 Broker 订阅；否则等待 CONNECTED 事件统一订阅。 */
        int msg_id = esp_mqtt_client_subscribe(s_mqtt_client, topic, 1);
        ESP_LOGI(TAG, "Subscribed to topic: %s, msg_id=%d", topic, msg_id);
        mqtt_notify_event(MQTT_EVT_SUBSCRIBED, 0, topic, 0);
    }

    ESP_LOGI(TAG, "Topic handler registered: %s", topic);

    return ESP_OK;
}

esp_err_t mqtt_unsubscribe(const char *topic)
{
    if (topic == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < s_topic_handler_count; i++) {
        if (strcmp(s_topic_handlers[i].topic, topic) == 0) {
            if (s_mqtt_client && s_connected) {
                esp_mqtt_client_unsubscribe(s_mqtt_client, topic);
            }
            for (int j = i; j < s_topic_handler_count - 1; j++) {
                memcpy(&s_topic_handlers[j], &s_topic_handlers[j + 1],
                       sizeof(mqtt_topic_handler_cfg_t));
            }
            s_topic_handler_count--;
            ESP_LOGI(TAG, "Topic handler unregistered: %s", topic);
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t mqtt_publish(const char *topic, const uint8_t *payload,
                        size_t payload_len, int qos)
{
    if (topic == NULL || payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 所有上报都要求 MQTT 已连接，否则直接返回错误，避免误以为云端已收到。 */
    if (!s_mqtt_client || !s_connected) {
        ESP_LOGE(TAG, "MQTT not connected, cannot publish");
        return ESP_ERR_INVALID_STATE;
    }

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic,
                                          (const char *)payload,
                                          payload_len, qos, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish to topic: %s", topic);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Published to %s, msg_id=%d, len=%d",
             topic, msg_id, (int)payload_len);
    mqtt_notify_event(MQTT_EVT_PUBLISHED, 0, topic, payload_len);

    return ESP_OK;
}

esp_err_t mqtt_publish_str(const char *topic, const char *payload, int qos)
{
    if (payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return mqtt_publish(topic, (const uint8_t *)payload, strlen(payload), qos);
}

esp_err_t mqtt_register_callback(mqtt_evt_cb_t callback)
{
    s_event_callback = callback;
    return ESP_OK;
}

/**
 * @brief MQTT wardrobe command topic handler
 *
 * Handles wardrobe commands on the Aliyun custom wardrobe topic. Parses JSON command
 * and sends the protocol frame via UART.
 */
esp_err_t mqtt_wardrobe_command_handler(const char *topic, const uint8_t *payload,
                                        size_t payload_len)
{
    (void)topic;

    /* MQTT payload 必须是非空 JSON，且长度不能超过组件缓冲上限。 */
    if (!payload || payload_len == 0 || payload_len > MQTT_MAX_PAYLOAD_LEN) {
        ESP_LOGW(TAG, "Invalid payload: payload=%p, len=%d",
                 (void *)payload, (int)payload_len);
        return wardrobe_publish_fault(WARDROBE_FAULT_UNKNOWN,
                                      "payload_required", -1);
    }

    /* 阿里云自定义 Topic 下发到设备时，ESP-IDF 收到的是原始 JSON 字符串。 */
    ESP_LOGI(TAG, "Wardrobe command received: %.*s", (int)payload_len,
             (const char *)payload);

    /* 使用 cJSON 按长度解析，避免 payload 不是 NUL 结尾字符串时越界读取。 */
    cJSON *root = cJSON_ParseWithLength((const char *)payload, payload_len);
    if (!root) {
        ESP_LOGW(TAG, "Invalid JSON in wardrobe command");
        return wardrobe_publish_fault(WARDROBE_FAULT_UNKNOWN,
                                      "invalid_json", -1);
    }

    /*
     * 当前上下行共用同一个 Topic。
     * ESP32 上报的成功/故障/状态 JSON 都带 status 字段；
     * 如果不忽略这些响应消息，设备可能把自己发布的响应再次当成命令处理。
     */
    if (cJSON_GetObjectItem(root, "status")) {
        cJSON_Delete(root);
        ESP_LOGI(TAG, "Ignoring wardrobe response message on shared topic");
        return ESP_OK;
    }

    /* 下行命令必须包含数字 cmd 字段。 */
    cJSON *cmd_item = cJSON_GetObjectItem(root, "cmd");
    if (!cmd_item || !cJSON_IsNumber(cmd_item)) {
        cJSON_Delete(root);
        return wardrobe_publish_fault(WARDROBE_FAULT_UNKNOWN,
                                      "missing_or_invalid_cmd", -1);
    }

    int cmd_value = cmd_item->valueint;
    /* 仅允许协议中定义的四类命令，未知命令直接上报 unsupported_cmd。 */
    if (cmd_value != WARDROBE_CMD_STORE &&
        cmd_value != WARDROBE_CMD_TAKE &&
        cmd_value != WARDROBE_CMD_RESET_HOME &&
        cmd_value != WARDROBE_CMD_READ_STATUS) {
        cJSON_Delete(root);
        return wardrobe_publish_fault(WARDROBE_FAULT_UNKNOWN,
                                      "unsupported_cmd", -1);
    }

    uint8_t data[1] = {0};
    uint8_t data_len = 0;
    int position = -1;

    /*
     * 存衣服和取衣服必须携带 position。
     * 回零和读状态命令没有 DATA 字段，因此 data_len 保持 0。
     */
    if (cmd_value == WARDROBE_CMD_STORE || cmd_value == WARDROBE_CMD_TAKE) {
        cJSON *position_item = cJSON_GetObjectItem(root, "position");
        if (!position_item || !cJSON_IsNumber(position_item) ||
            position_item->valueint < 1 || position_item->valueint > 255) {
            cJSON_Delete(root);
            return wardrobe_publish_fault(WARDROBE_FAULT_UNKNOWN,
                                          "missing_or_invalid_position", -1);
        }

        position = position_item->valueint;
        data[0] = (uint8_t)position;
        data_len = 1;
    }

    cJSON_Delete(root);

    /*
     * 组装 STM32 二进制协议帧：
     * A5 CMD LEN [DATA...] CRC 5A
     * 其中 CRC 为 CMD、LEN 和所有 DATA 字节异或。
     */
    uint8_t frame[6] = {0};
    frame[0] = WARDROBE_FRAME_SOF;
    frame[1] = (uint8_t)cmd_value;
    frame[2] = data_len;
    if (data_len > 0) {
        frame[3] = data[0];
    }
    frame[3 + data_len] = wardrobe_calc_crc((uint8_t)cmd_value, data_len, data);
    frame[4 + data_len] = WARDROBE_FRAME_EOF;
    size_t frame_len = 5 + data_len;

    uart_port_id_t port_id = WARDROBE_UART_PORT_ID;
    ESP_LOGI(TAG, "Writing wardrobe frame to UART port %d: cmd=0x%02X, len=%d",
             port_id, cmd_value, data_len);
    ESP_LOG_BUFFER_HEXDUMP(TAG, frame, frame_len, ESP_LOG_INFO);

    esp_err_t uart_ret = uart_write(port_id, frame, frame_len);
    if (uart_ret != ESP_OK) {
        ESP_LOGE(TAG, "UART write failed: %s", esp_err_to_name(uart_ret));
        return wardrobe_publish_fault(0x05, "uart_write_failed", position);
    }

    if (data_len > 0) {
        /*
         * STM32 成功响应只带 position，不带原始 action。
         * 这里记录最近一次下发的 action，后续 UART 收到 0x90 成功帧时补充到上报 JSON。
         */
        uart_protocol_set_pending_command(port_id, (uint8_t)cmd_value,
                                          (uint8_t)position);
    }

    ESP_LOGI(TAG, "UART wardrobe frame write success");
    return ESP_OK;
}
