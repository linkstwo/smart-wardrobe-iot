/**
 * @file uart.c
 * @brief UART component implementation
 * 
 * This file implements the UART component functionality, supporting
 * multiple UART ports with event-driven callbacks for data handling.
 *
 * 中文说明：
 * 本文件封装 ESP32 的 UART 驱动，并在接收路径中解析 STM32 发回的
 * 智能衣柜二进制协议帧。它承担两类职责：
 * 1. 通用 UART 管理：添加端口、安装驱动、读取字节、写入字节、上报事件。
 * 2. 衣柜协议上行处理：把 STM32 的 A5...5A 帧解析成 JSON，再通过 MQTT 上报。
 *
 * 与 mqtt.c 的关系：
 * - mqtt.c 负责“云端 JSON -> STM32 UART 帧”。
 * - uart.c 负责“STM32 UART 帧 -> 云端 JSON”。
 */

#include "uart.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "mqtt.h"
#include "cJSON.h"
#include "esp_timer.h"

static const char *TAG = "UART";

/* 阿里云自定义 Topic 同时用于命令下发和状态响应，status 字段用于避免响应回环。 */
#define WARDROBE_TOPIC_RESPONSE       WARDROBE_MQTT_TOPIC
/* STM32 智能衣柜二进制帧格式：A5 CMD LEN DATA CRC 5A。 */
#define WARDROBE_FRAME_SOF            0xA5
#define WARDROBE_FRAME_EOF            0x5A
/* DATA 最大长度和完整帧长度边界，用于防止异常字节流撑爆缓存。 */
#define WARDROBE_MAX_DATA_LEN         128
#define WARDROBE_MIN_FRAME_LEN        5
#define WARDROBE_MAX_FRAME_LEN        (WARDROBE_MAX_DATA_LEN + 5)
/* 记录下行存/取命令，用于后续成功响应里补充 action 字段。 */
#define WARDROBE_CMD_STORE            0x10
#define WARDROBE_CMD_TAKE             0x11
/* STM32 上行响应命令码。 */
#define WARDROBE_RESP_SUCCESS         0x90
#define WARDROBE_RESP_FAULT           0x91
#define WARDROBE_RESP_STATUS          0xA0

typedef struct {
    /* 正在拼接的二进制帧缓存。 */
    uint8_t data[WARDROBE_MAX_FRAME_LEN];
    /* 当前已经收到的帧字节数。 */
    size_t len;
} wardrobe_frame_buffer_t;

typedef struct {
    /* 是否存在一条正在等待 STM32 响应的存/取命令。 */
    bool active;
    /* 最近一次下发给 STM32 的动作：0x10 存衣服，0x11 取衣服。 */
    uint8_t action;
    /* 最近一次下发的衣格位置。 */
    uint8_t position;
} wardrobe_pending_command_t;

static void uart_process_wardrobe_frame(uart_port_id_t port_id,
                                        const uint8_t *frame,
                                        size_t frame_len);
static void uart_process_received_data(uart_port_id_t port_id, uint8_t *data, size_t len);

/**
 * @brief UART port context structure
 */
typedef struct {
    bool used;                      /**< 该逻辑端口是否已被占用 */
    uart_port_cfg_t config;         /**< 该端口对应的硬件 UART、引脚、波特率等配置 */
    QueueHandle_t event_queue;      /**< ESP-IDF UART 驱动事件队列 */
    TaskHandle_t event_task;        /**< 本端口的 UART 事件处理任务 */
    uart_evt_cb_t port_callback;    /**< 端口级回调，只通知该端口的事件 */
} uart_port_ctx_t;

/** 所有逻辑 UART 端口的上下文数组。 */
static uart_port_ctx_t s_ports[UART_MAX_PORTS];

/* 每个 UART 端口各自保存一份智能衣柜帧拼接状态，避免多端口互相污染。 */
static wardrobe_frame_buffer_t s_frame_buffers[UART_MAX_PORTS];
/* 每个 UART 端口记录最近一次下发的存/取命令。 */
static wardrobe_pending_command_t s_pending_commands[UART_MAX_PORTS];

/** 全局 UART 事件回调，应用层可用它统一观察所有端口事件。 */
static uart_evt_cb_t s_global_callback = NULL;

/** UART 组件是否已经初始化。 */
static bool s_initialized = false;

/** 当前已经添加的活动端口数量。 */
static uint8_t s_port_count = 0;

/**
 * @brief Notify application of UART event
 * 
 * @param type Event type
 * @param port_id Port identifier
 * @param data Data buffer
 * @param data_len Data length
 * @param status Event status
 */
static void uart_notify_event(uart_evt_type_t type, uart_port_id_t port_id, uint8_t *data, size_t data_len, int status)
{
    /*
     * 事件先通知全局回调，再通知端口专属回调。
     * 当前 app_init.c 使用全局回调打印日志，业务协议解析则在本文件内部完成。
     */
    uart_evt_data_t event_data = {
        .type = type,
        .port_id = port_id,
        .data = data,
        .data_len = data_len,
        .status = status,
    };

    if (s_global_callback) {
        s_global_callback(&event_data);
    }

    if (port_id < UART_MAX_PORTS && s_ports[port_id].used && s_ports[port_id].port_callback) {
        s_ports[port_id].port_callback(&event_data);
    }
}

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

/*
 * 生成 MQTT 上报时间戳。
 * 如果 SNTP/系统时间可用，使用 Unix 时间；否则退化为 ESP32 上电秒数。
 */
static int64_t wardrobe_timestamp(void)
{
    time_t now = time(NULL);

    if (now > 0) {
        return (int64_t)now;
    }

    return (int64_t)(esp_timer_get_time() / 1000000);
}

/* 将 STM32 故障码转换为云端 JSON 中更易读的英文故障字符串。 */
static const char *wardrobe_fault_msg(uint8_t code)
{
    switch (code) {
    case 0x01:
        return "motor_overcurrent";
    case 0x02:
        return "motor_stall";
    case 0x03:
        return "limit_switch_error";
    case 0x04:
        return "sensor_fault";
    case 0x05:
        return "communication_timeout";
    case 0x06:
        return "eeprom_error";
    case 0x07:
        return "slot_occupied";
    case 0x08:
        return "slot_empty";
    case 0xFF:
    default:
        return "unknown_fault";
    }
}

/*
 * 把构造好的 cJSON 对象发布到 MQTT。
 * 注意：本函数只负责序列化和发布，不释放 root；调用方负责 cJSON_Delete(root)。
 */
static esp_err_t wardrobe_publish_json(cJSON *root)
{
    if (!root) {
        return ESP_ERR_INVALID_ARG;
    }

    char *json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t mqtt_ret = mqtt_publish_str(WARDROBE_TOPIC_RESPONSE, json_str, 1);
    if (mqtt_ret != ESP_OK) {
        ESP_LOGW(TAG, "MQTT publish failed: %s", esp_err_to_name(mqtt_ret));
    }

    free(json_str);
    return mqtt_ret;
}

static void uart_process_wardrobe_frame(uart_port_id_t port_id,
                                        const uint8_t *frame,
                                        size_t frame_len)
{
    if (!frame || frame_len < WARDROBE_MIN_FRAME_LEN) {
        return;
    }

    uint8_t cmd = frame[1];
    uint8_t data_len = frame[2];
    const uint8_t *payload = &frame[3];

    /*
     * 这里把 STM32 的二进制响应转换为 MQTT JSON：
     * 0x90 成功 -> {"status":144,"action":...,"position":...}
     * 0x91 故障 -> {"status":145,"code":...,"msg":...,"position":...}
     * 0xA0 状态 -> {"status":160,"clothing_count":...,"uptime_...":...}
     */
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return;
    }

    if (cmd == WARDROBE_RESP_SUCCESS && data_len == 1) {
        uint8_t position = payload[0];
        uint8_t action = 0;

        /*
         * STM32 成功帧只带 position，不带这次是“存”还是“取”。
         * 因此使用 mqtt.c 下发命令时记录的 pending action 补全云端 JSON。
         */
        if (port_id < UART_MAX_PORTS && s_pending_commands[port_id].active) {
            action = s_pending_commands[port_id].action;
            s_pending_commands[port_id].active = false;
        }

        cJSON_AddNumberToObject(root, "status", WARDROBE_RESP_SUCCESS);
        cJSON_AddNumberToObject(root, "action", action);
        cJSON_AddNumberToObject(root, "position", position);
        cJSON_AddNumberToObject(root, "timestamp", wardrobe_timestamp());
    } else if (cmd == WARDROBE_RESP_FAULT && data_len == 2) {
        /* 故障帧 DATA[0] 为故障码，DATA[1] 为位置；0xFF 表示无特定位置。 */
        uint8_t code = payload[0];
        int position = payload[1] == 0xFF ? -1 : payload[1];

        if (port_id < UART_MAX_PORTS && s_pending_commands[port_id].active) {
            s_pending_commands[port_id].active = false;
        }

        cJSON_AddNumberToObject(root, "status", WARDROBE_RESP_FAULT);
        cJSON_AddNumberToObject(root, "code", code);
        cJSON_AddStringToObject(root, "msg", wardrobe_fault_msg(code));
        cJSON_AddNumberToObject(root, "position", position);
        cJSON_AddNumberToObject(root, "timestamp", wardrobe_timestamp());
    } else if (cmd == WARDROBE_RESP_STATUS && data_len == 6) {
        /*
         * 状态帧使用大端序：
         * DATA[0:1] 衣物数量，DATA[2:3] 运行天数，DATA[4] 小时，DATA[5] 分钟。
         */
        uint16_t clothing_count = ((uint16_t)payload[0] << 8) | payload[1];
        uint16_t uptime_days = ((uint16_t)payload[2] << 8) | payload[3];

        cJSON_AddNumberToObject(root, "status", WARDROBE_RESP_STATUS);
        cJSON_AddNumberToObject(root, "clothing_count", clothing_count);
        cJSON_AddNumberToObject(root, "uptime_days", uptime_days);
        cJSON_AddNumberToObject(root, "uptime_hours", payload[4]);
        cJSON_AddNumberToObject(root, "uptime_minutes", payload[5]);
        cJSON_AddNumberToObject(root, "timestamp", wardrobe_timestamp());
    } else {
        ESP_LOGW(TAG, "Unsupported wardrobe frame: cmd=0x%02X, len=%d",
                 cmd, data_len);
        cJSON_Delete(root);
        return;
    }

    wardrobe_publish_json(root);
    cJSON_Delete(root);
}

static void uart_process_received_data(uart_port_id_t port_id, uint8_t *data, size_t len)
{
    if (port_id >= UART_MAX_PORTS || !data || len == 0) {
        return;
    }

    ESP_LOGI(TAG, "UART%d received %d bytes", port_id, len);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, len > 32 ? 32 : len, ESP_LOG_INFO);

    wardrobe_frame_buffer_t *buffer = &s_frame_buffers[port_id];

    /*
     * UART 是字节流，不保证一次回调就是一整帧。
     * 因此这里逐字节扫描：
     * 1. 等待帧头 A5；
     * 2. 收到 CMD 和 LEN 后计算期望总长度；
     * 3. 收齐后检查帧尾 5A；
     * 4. 校验 CRC；
     * 5. 交给 uart_process_wardrobe_frame() 解析业务含义。
     */
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        if (buffer->len == 0) {
            /* 未进入帧时，丢弃所有非帧头字节，避免串口噪声影响解析。 */
            if (byte != WARDROBE_FRAME_SOF) {
                continue;
            }
            buffer->data[buffer->len++] = byte;
            continue;
        }

        if (buffer->len >= WARDROBE_MAX_FRAME_LEN) {
            /* 帧过长说明数据损坏，清空缓存并尝试把当前字节作为新帧头重新同步。 */
            ESP_LOGW(TAG, "Wardrobe frame buffer overflow");
            buffer->len = 0;
            if (byte == WARDROBE_FRAME_SOF) {
                buffer->data[buffer->len++] = byte;
            }
            continue;
        }

        buffer->data[buffer->len++] = byte;

        if (buffer->len == 3 && buffer->data[2] > WARDROBE_MAX_DATA_LEN) {
            /* LEN 字段超出协议上限，说明当前帧非法，直接丢弃。 */
            ESP_LOGW(TAG, "Invalid wardrobe frame data len: %d",
                     buffer->data[2]);
            buffer->len = 0;
            continue;
        }

        if (buffer->len >= 3) {
            size_t expected_len = (size_t)buffer->data[2] + 5;
            if (buffer->len < expected_len) {
                continue;
            }

            /* 收到期望长度后先校验帧尾，帧尾错误通常是丢字节或噪声导致。 */
            if (buffer->data[expected_len - 1] != WARDROBE_FRAME_EOF) {
                ESP_LOGW(TAG, "Invalid wardrobe frame EOF");
                buffer->len = 0;
                if (byte == WARDROBE_FRAME_SOF) {
                    buffer->data[buffer->len++] = byte;
                }
                continue;
            }

            /* 帧尾正确后再校验 CRC，防止错误数据被当成有效响应上报。 */
            uint8_t expected_crc = wardrobe_calc_crc(buffer->data[1],
                                                    buffer->data[2],
                                                    &buffer->data[3]);
            uint8_t actual_crc = buffer->data[expected_len - 2];
            if (expected_crc != actual_crc) {
                ESP_LOGW(TAG, "Invalid wardrobe frame CRC: expected=0x%02X, actual=0x%02X",
                         expected_crc, actual_crc);
                buffer->len = 0;
                continue;
            }

            uart_process_wardrobe_frame(port_id, buffer->data, expected_len);
            buffer->len = 0;
        }
    }
}

void uart_protocol_set_pending_command(uart_port_id_t port_id, uint8_t action,
                                       uint8_t position)
{
    if (port_id >= UART_MAX_PORTS) {
        return;
    }

    if (action != WARDROBE_CMD_STORE && action != WARDROBE_CMD_TAKE) {
        return;
    }

    /* 记录最近一次下发的存/取命令，等待 STM32 成功帧回来时补充 action。 */
    s_pending_commands[port_id].active = true;
    s_pending_commands[port_id].action = action;
    s_pending_commands[port_id].position = position;
}

/**
 * @brief UART event handling task
 * 
 * @param pvParameters Port ID as parameter
 */
static void uart_event_task(void *pvParameters)
{
    uart_port_id_t port_id = (uart_port_id_t)(uint32_t)pvParameters;
    
    uart_port_ctx_t *ctx = &s_ports[port_id];
    
    uart_event_t event;
    uint8_t *rx_buffer = malloc(ctx->config.rx_buf_size);

    if (!rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer for port %d", port_id);
        vTaskDelete(NULL);
        return;
    }

    while (ctx->used && ctx->event_queue) {
        if (xQueueReceive(ctx->event_queue, &event, pdMS_TO_TICKS(100))) {
            switch (event.type) {
                case UART_DATA:
                    /* 收到 UART 数据后先通知应用层，再进入智能衣柜帧解析器。 */
                    if (event.size > 0) {
                        int len = uart_read_bytes(ctx->config.port, rx_buffer,
                            event.size < ctx->config.rx_buf_size ? event.size : ctx->config.rx_buf_size,
                            pdMS_TO_TICKS(ctx->config.timeout_ms));
                        if (len > 0) {
                            uart_notify_event(UART_EVT_RX_DATA, port_id, rx_buffer, len, 0);
                            uart_process_received_data(port_id, rx_buffer, len);
                        }
                    }
                    break;

                case UART_FIFO_OVF:
                    /* 硬件 FIFO 溢出，说明处理速度跟不上或数据异常，清空输入缓存重新同步。 */
                    uart_notify_event(UART_EVT_RX_FIFO_OVF, port_id, NULL, 0, 0);
                    ESP_LOGW(TAG, "UART%d FIFO overflow", ctx->config.port);
                    uart_flush_input(ctx->config.port);
                    break;

                case UART_BUFFER_FULL:
                    /* 驱动环形缓冲区满，清空输入缓存避免旧数据继续污染后续帧。 */
                    uart_notify_event(UART_EVT_RX_FIFO_FULL, port_id, NULL, 0, 0);
                    ESP_LOGW(TAG, "UART%d buffer full", ctx->config.port);
                    uart_flush_input(ctx->config.port);
                    break;

                case UART_BREAK:
                    /* Break detected */
                    uart_notify_event(UART_EVT_BRK_DET, port_id, NULL, 0, 0);
                    break;

                case UART_PARITY_ERR:
                    /* Parity error */
                    uart_notify_event(UART_EVT_PARITY_ERR, port_id, NULL, 0, 0);
                    ESP_LOGW(TAG, "UART%d parity error", ctx->config.port);
                    break;

                case UART_FRAME_ERR:
                    /* Frame error */
                    uart_notify_event(UART_EVT_FRAME_ERR, port_id, NULL, 0, 0);
                    ESP_LOGW(TAG, "UART%d frame error", ctx->config.port);
                    break;

                case UART_PATTERN_DET:
                    /* Pattern detected */
                    uart_notify_event(UART_EVT_PATTERN_DET, port_id, NULL, 0, 0);
                    break;

                case UART_EVENT_MAX:
                default:
                    break;
            }
        }
    }

    free(rx_buffer);
    vTaskDelete(NULL);
}

/**
 * @brief Find free port slot
 * 
 * @return Slot index, -1 if none available
 */
static int find_free_port_slot(void)
{
    for (int i = 0; i < UART_MAX_PORTS; i++) {
        if (!s_ports[i].used) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Setup UART port with configuration
 * 
 * @param port_id Port identifier
 * @param config Port configuration
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t uart_setup_port(uart_port_id_t port_id, uart_port_cfg_t *config)
{
    uart_port_ctx_t *ctx = &s_ports[port_id];
    esp_err_t ret;

    /* 配置 UART 基本参数：波特率、数据位、校验位、停止位、流控。 */
    uart_config_t uart_cfg = {
        .baud_rate = config->baud_rate,
        .data_bits = config->data_bits,
        .parity = config->parity,
        .stop_bits = config->stop_bits,
        .flow_ctrl = config->flow_ctrl,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ret = uart_param_config(config->port, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART%d param config failed: %s", config->port, esp_err_to_name(ret));
        return ret;
    }

    /* 绑定 UART TX/RX/RTS/CTS 引脚；当前智能衣柜主要使用 TX/RX。 */
    ret = uart_set_pin(config->port, config->tx_pin, config->rx_pin, config->rts_pin, config->cts_pin);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART%d set pin failed: %s", config->port, esp_err_to_name(ret));
        return ret;
    }

    /* Install UART driver */
    ret = uart_driver_install(config->port, config->rx_buf_size, config->tx_buf_size,
        20, &ctx->event_queue, config->intr_flags);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART%d driver install failed: %s", config->port, esp_err_to_name(ret));
        return ret;
    }

    /* Enable pattern detection if interrupts enabled */
    if (config->enable_rx_intr || config->enable_err_intr) {
        uart_enable_pattern_det_baud_intr(config->port, '\n', 1, 9, 0, 0);
        uart_pattern_queue_reset(config->port, 20);
    }

    memcpy(&ctx->config, config, sizeof(uart_port_cfg_t));
    ctx->used = true;

    /* Create event handling task */
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "uart_evt_%d", port_id);
    BaseType_t task_ret = xTaskCreate(uart_event_task, task_name, 16384,
        (void *)(uint32_t)port_id, 12, &ctx->event_task);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create event task for port %d", port_id);
        uart_driver_delete(config->port);
        ctx->used = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Initialize UART component
 */
esp_err_t uart_init(uart_cfg_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "UART already initialized");
        return ESP_OK;
    }

    memset(s_ports, 0, sizeof(s_ports));
    ESP_LOGI(TAG, "UART ports array initialized (size=%zu, count=%d)", sizeof(s_ports), UART_MAX_PORTS);
    s_initialized = true;

    if (config && config->event_callback) {
        s_global_callback = config->event_callback;
    }

    /* Add configured ports */
    if (config && config->ports && config->port_count > 0) {
        for (uint8_t i = 0; i < config->port_count && i < UART_MAX_PORTS; i++) {
            uart_port_id_t port_id;
            esp_err_t ret = uart_add_port(&config->ports[i], &port_id);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to add port %d: %s", i, esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG, "Port %d added with id %d", i, port_id);
            }
        }
    }

    s_initialized = true;
    ESP_LOGI(TAG, "UART initialized");

    uart_notify_event(UART_EVT_INIT_COMPLETE, 0, NULL, 0, 0);

    return ESP_OK;
}

/**
 * @brief Deinitialize UART component
 */
esp_err_t uart_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    /* Remove all ports */
    for (int i = 0; i < UART_MAX_PORTS; i++) {
        if (s_ports[i].used) {
            uart_remove_port(i);
        }
    }

    s_initialized = false;
    s_global_callback = NULL;
    s_port_count = 0;

    ESP_LOGI(TAG, "UART deinitialized");

    uart_notify_event(UART_EVT_DEINIT_COMPLETE, 0, NULL, 0, 0);

    return ESP_OK;
}

/**
 * @brief Add UART port
 */
esp_err_t uart_add_port(uart_port_cfg_t *config, uart_port_id_t *port_id)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "UART not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    int slot = find_free_port_slot();
    if (slot < 0) {
        ESP_LOGE(TAG, "No free port slots available");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = uart_setup_port(slot, config);
    if (ret != ESP_OK) {
        return ret;
    }

    *port_id = slot;
    s_port_count++;

    ESP_LOGI(TAG, "UART port added: id=%d, hw_port=%d, baud=%d", slot, config->port, config->baud_rate);

    uart_notify_event(UART_EVT_PORT_ADDED, slot, NULL, 0, 0);

    return ESP_OK;
}

/**
 * @brief Remove UART port
 */
esp_err_t uart_remove_port(uart_port_id_t port_id)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_port_ctx_t *ctx = &s_ports[port_id];

    ctx->used = false;

    /* Wait for event task to finish */
    if (ctx->event_queue) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* Delete event task */
    if (ctx->event_task) {
        vTaskDelete(ctx->event_task);
        ctx->event_task = NULL;
    }

    /* Delete UART driver */
    if (ctx->event_queue) {
        uart_driver_delete(ctx->config.port);
        ctx->event_queue = NULL;
    }

    memset(&ctx->config, 0, sizeof(uart_port_cfg_t));
    ctx->port_callback = NULL;
    s_port_count--;

    ESP_LOGI(TAG, "UART port removed: id=%d", port_id);

    uart_notify_event(UART_EVT_PORT_REMOVED, port_id, NULL, 0, 0);

    return ESP_OK;
}

/**
 * @brief Update port configuration
 */
esp_err_t uart_update_port_config(uart_port_id_t port_id, uart_port_cfg_t *config)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_port_ctx_t *ctx = &s_ports[port_id];
    uart_port_cfg_t old_config;
    memcpy(&old_config, &ctx->config, sizeof(uart_port_cfg_t));

    /* Remove and re-setup port */
    esp_err_t ret = uart_remove_port(port_id);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = uart_setup_port(port_id, config);
    if (ret != ESP_OK) {
        uart_setup_port(port_id, &old_config);
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Register global callback
 */
esp_err_t uart_register_callback(uart_evt_cb_t callback)
{
    s_global_callback = callback;
    ESP_LOGI(TAG, "Global event callback registered");
    return ESP_OK;
}

/**
 * @brief Register port-specific callback
 */
esp_err_t uart_port_register_callback(uart_port_id_t port_id, uart_evt_cb_t callback)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return ESP_ERR_INVALID_ARG;
    }

    s_ports[port_id].port_callback = callback;
    ESP_LOGI(TAG, "Port %d event callback registered", port_id);
    return ESP_OK;
}

/**
 * @brief Write data to UART port
 */
esp_err_t uart_write(uart_port_id_t port_id, const uint8_t *data, size_t len)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_port_ctx_t *ctx = &s_ports[port_id];
    int bytes_written = uart_write_bytes(ctx->config.port, data, len);

    if (bytes_written < 0) {
        return ESP_FAIL;
    }

    if ((size_t)bytes_written != len) {
        return ESP_ERR_TIMEOUT;
    }

    uart_notify_event(UART_EVT_TX_DONE, port_id, (uint8_t *)data, len, 0);

    return ESP_OK;
}

/**
 * @brief Read data from UART port
 */
esp_err_t uart_read(uart_port_id_t port_id, uint8_t *data, size_t len, size_t *actual_len)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_port_ctx_t *ctx = &s_ports[port_id];
    int bytes_read = uart_read_bytes(ctx->config.port, data, len, pdMS_TO_TICKS(ctx->config.timeout_ms));

    if (bytes_read < 0) {
        return ESP_FAIL;
    }

    *actual_len = bytes_read;
    return ESP_OK;
}

/**
 * @brief Write bytes asynchronously
 */
esp_err_t uart_write_bytes_async(uart_port_id_t port_id, const uint8_t *data, size_t len)
{
    return uart_write(port_id, data, len);
}

/**
 * @brief Get available bytes in RX buffer
 */
int uart_available(uart_port_id_t port_id)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return -1;
    }

    size_t buffered_size;
    uart_get_buffered_data_len(s_ports[port_id].config.port, &buffered_size);
    return (int)buffered_size;
}

/**
 * @brief Flush TX and RX buffers
 */
esp_err_t uart_port_flush(uart_port_id_t port_id)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return ESP_ERR_INVALID_ARG;
    }

    return uart_flush_input(s_ports[port_id].config.port);
}

/**
 * @brief Flush RX buffer only
 */
esp_err_t uart_port_flush_input(uart_port_id_t port_id)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return ESP_ERR_INVALID_ARG;
    }

    return uart_flush_input(s_ports[port_id].config.port);
}

/**
 * @brief Set baud rate
 */
esp_err_t uart_port_set_baudrate(uart_port_id_t port_id, int baud_rate)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_port_ctx_t *ctx = &s_ports[port_id];
    esp_err_t ret = uart_set_baudrate(ctx->config.port, baud_rate);

    if (ret == ESP_OK) {
        ctx->config.baud_rate = baud_rate;
        ESP_LOGI(TAG, "Port %d baudrate changed to %d", port_id, baud_rate);
    }

    return ret;
}

/**
 * @brief Get baud rate
 */
int uart_port_get_baudrate(uart_port_id_t port_id)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return -1;
    }

    return s_ports[port_id].config.baud_rate;
}

/**
 * @brief Enable pattern detection
 */
esp_err_t uart_enable_pattern_det(uart_port_id_t port_id, char pattern_char, uint8_t num_chars, uint32_t idle_timeout, uint8_t pre_idle_chars, uint8_t post_idle_chars)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_port_t hw_port = s_ports[port_id].config.port;

    esp_err_t ret = uart_enable_pattern_det_baud_intr(hw_port, (uint8_t)pattern_char, num_chars, idle_timeout, pre_idle_chars, post_idle_chars);
    if (ret != ESP_OK) {
        return ret;
    }

    return uart_pattern_queue_reset(hw_port, 20);
}

/**
 * @brief Disable pattern detection
 */
esp_err_t uart_disable_pattern_det(uart_port_id_t port_id)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return ESP_ERR_INVALID_ARG;
    }

    return uart_disable_pattern_det_intr(s_ports[port_id].config.port);
}

/**
 * @brief Check if port is valid
 */
bool uart_port_is_valid(uart_port_id_t port_id)
{
    return port_id < UART_MAX_PORTS && s_ports[port_id].used;
}

/**
 * @brief Get hardware port number
 */
uart_port_t uart_get_hw_port(uart_port_id_t port_id)
{
    if (port_id >= UART_MAX_PORTS || !s_ports[port_id].used) {
        return UART_NUM_MAX;
    }
    return s_ports[port_id].config.port;
}
