# app 组件负责总应用逻辑，依赖 uart、wifi 和 esp_mqtt 三个自定义/系统组件。
idf_component_register(SRCS "app.c" "app_init.c"
                    INCLUDE_DIRS "include"
                    REQUIRES uart wifi esp_mqtt)
