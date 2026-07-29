/**
 * @file mqtt_config.h
 * @brief MQTT component configuration header
 *
 * This file contains compile-time configuration macros for the MQTT component.
 *
 * 中文说明：
 * 本文件集中定义 MQTT 组件的编译期配置。
 * 当前参数用于连接阿里云 IoT MQTT Broker，并订阅/发布智能衣柜自定义 Topic。
 * 如果更换设备、ProductKey、DeviceName 或重新生成三元组密码，应优先检查这里。
 */

#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#include "sdkconfig.h"

/* MQTT Topic 字符串缓冲区上限。阿里云自定义 Topic 必须小于该长度。 */
#define MQTT_MAX_TOPIC_LEN             128
/* MQTT payload 缓冲区上限。当前下行 JSON 很短，该值主要留给状态/调试消息。 */
#define MQTT_MAX_PAYLOAD_LEN           4096
/* 最多可注册的订阅 Topic 数量。当前项目只使用智能衣柜自定义 Topic。 */
#define MQTT_MAX_SUBSCRIBE_COUNT       16
/* MQTT keepalive 和断线重连默认值。 */
#define MQTT_DEFAULT_KEEPALIVE_SEC     60
#define MQTT_DEFAULT_RECONNECT_MS      5000
/* MQTT 事件分发任务的默认栈大小、优先级和队列深度。 */
#define MQTT_EVT_TASK_STACK_SIZE        8192
#define MQTT_EVT_TASK_PRIORITY          5
#define MQTT_EVENT_QUEUE_SIZE          20

/* 阿里云自定义 Topic：云端 Pub 下发命令，ESP32 也通过该 Topic 上报响应。 */
#define WARDROBE_MQTT_TOPIC           "/k1vtgAzED8C/7Rzu9Cy7Ze6Yhe1YcMjQ/user/wardrobe"

/* 阿里云 MQTT Broker 地址。 */
#ifndef CONFIG_MQTT_BROKER_HOST
#define CONFIG_MQTT_BROKER_HOST        "iot-06z00dgi0u8jci8.mqtt.iothub.aliyuncs.com"
#endif

/* 1883 为普通 MQTT 端口；如果启用 SSL，通常需要换成对应安全端口并配置证书。 */
#ifndef CONFIG_MQTT_BROKER_PORT
#define CONFIG_MQTT_BROKER_PORT        1883
#endif

/* 阿里云设备 client_id，通常由 ProductKey.DeviceName 和 securemode/signmethod/timestamp 组成。 */
#ifndef CONFIG_MQTT_CLIENT_ID
#define CONFIG_MQTT_CLIENT_ID          "k1vtgAzED8C.7Rzu9Cy7Ze6Yhe1YcMjQ|securemode=2,signmethod=hmacsha256,timestamp=1781099957494|"
#endif

/* 阿里云 MQTT 用户名格式通常为 DeviceName&ProductKey。 */
#ifndef CONFIG_MQTT_USERNAME
#define CONFIG_MQTT_USERNAME           "7Rzu9Cy7Ze6Yhe1YcMjQ&k1vtgAzED8C"
#endif

/* 阿里云 MQTT 密码，通常由 DeviceSecret 按签名算法生成。 */
#ifndef CONFIG_MQTT_PASSWORD
#define CONFIG_MQTT_PASSWORD           "9d8a5497902d72cc11704e2df41983c445d4e25e8438fcb645f2725b330e0627"
#endif

#ifndef CONFIG_MQTT_KEEPALIVE_SEC
#define CONFIG_MQTT_KEEPALIVE_SEC      60
#endif

#ifndef CONFIG_MQTT_RECONNECT_INTERVAL_MS
#define CONFIG_MQTT_RECONNECT_INTERVAL_MS  5000
#endif

#ifndef CONFIG_MQTT_BROKER_CERT
#define CONFIG_MQTT_BROKER_CERT        ""
#endif

#endif
