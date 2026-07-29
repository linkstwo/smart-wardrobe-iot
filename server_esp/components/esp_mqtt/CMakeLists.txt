# esp_mqtt 组件封装 MQTT 连接、阿里云 Topic 收发和衣柜命令 JSON 解析。
idf_component_register(SRCS "mqtt.c"
                        INCLUDE_DIRS "include"
                        REQUIRES mqtt freertos json uart)
