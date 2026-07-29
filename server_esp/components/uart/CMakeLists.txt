# uart 组件负责 ESP32 与 STM32 的串口通信。
# UART 收到 STM32 二进制响应后，会转交 esp_mqtt 组件上报到阿里云。
idf_component_register(SRCS "uart.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart log esp_timer)

# 关键：手动添加所有需要的头文件路径，绕过循环依赖检查。
# 这里让 uart.c 能引用 esp_mqtt 的接口声明和 cJSON 头文件，但不把组件依赖改成互相循环。
target_include_directories(${COMPONENT_LIB} PRIVATE
    ${CMAKE_SOURCE_DIR}/components/esp_mqtt/include
    ${IDF_PATH}/components/json/cJSON
)
