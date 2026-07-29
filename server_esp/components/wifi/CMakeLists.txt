# wifi 组件负责 ESP32 STA/AP 网络初始化，供 MQTT 组件联网使用。
idf_component_register(SRCS "wifi.c"
                        INCLUDE_DIRS "include"
                        REQUIRES esp_wifi esp_event esp_netif nvs_flash freertos lwip)
