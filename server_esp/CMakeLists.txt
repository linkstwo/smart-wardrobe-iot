# ESP-IDF 顶层工程 CMake 配置。
# 下面这几行是 ESP-IDF 要求的固定模板，顺序不能随意调整，否则 CMake 可能无法正确加载工程。
cmake_minimum_required(VERSION 3.16)

# 这里只编译 main 组件；app 目录通过 EXTRA_COMPONENT_DIRS 作为额外组件目录加入工程。
set(COMPONENTS main)
set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/app")

# 加载 ESP-IDF 自带的工程构建脚本，并声明当前工程名。
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(main)
