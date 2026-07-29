# main 组件只包含 main.c，并依赖 app 组件完成应用初始化。
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES app)
