# ESP32 Server

ESP32 作为中间人，桥接单片机与远程服务器，实现数据上报与远程控制。

## 系统架构

```
┌─────────┐    UART     ┌─────────┐     MQTT      ┌──────────┐
│  单片机  │ ◄─────────► │  ESP32  │ ◄────────────► │  服务器   │
└─────────┘             └─────────┘               └──────────┘
```

- **单片机 ↔ ESP32**: UART 串口二进制帧协议 (GPIO17 TX / GPIO18 RX, 115200-8N1)
- **ESP32 ↔ 服务器**: WiFi STA + MQTT

## 通信业务

### 上行：单片机 → ESP32 → 服务器

ESP32 从 UART 接收协议帧并通过 MQTT 上报到阿里云自定义 Topic
`/k1vtgAzED8C/7Rzu9Cy7Ze6Yhe1YcMjQ/user/wardrobe`：

| UART 响应 | 含义 | MQTT JSON |
|-----------|------|-----------|
| `0x90` | 操作成功 | `{"status":144,"action":16,"position":1,"timestamp":...}` |
| `0x91` | 故障异常 | `{"status":145,"code":255,"msg":"unknown_fault","position":1,"timestamp":...}` |
| `0xA0` | 状态上报 | `{"status":160,"clothing_count":12,"uptime_days":3,"uptime_hours":8,"uptime_minutes":30,"timestamp":...}` |

UART 帧格式：`A5 CMD LEN DATA CRC 5A`，CRC 为 `CMD ^ LEN ^ DATA...`。

### 下行：服务器 → ESP32 → 单片机

| 通道 | 主题 | 请求 | ESP32 串口转发 |
|------|------|------|----------------|
| MQTT | `/k1vtgAzED8C/7Rzu9Cy7Ze6Yhe1YcMjQ/user/wardrobe` | `{"cmd":16,"position":1}` | `A5 10 01 01 10 5A` |
| MQTT | `/k1vtgAzED8C/7Rzu9Cy7Ze6Yhe1YcMjQ/user/wardrobe` | `{"cmd":17,"position":1}` | `A5 11 01 01 11 5A` |
| MQTT | `/k1vtgAzED8C/7Rzu9Cy7Ze6Yhe1YcMjQ/user/wardrobe` | `{"cmd":18}` | `A5 12 00 12 5A` |
| MQTT | `/k1vtgAzED8C/7Rzu9Cy7Ze6Yhe1YcMjQ/user/wardrobe` | `{"cmd":32}` | `A5 20 00 20 5A` |

支持命令: `0x10` 存衣服、`0x11` 取衣服、`0x12` 复位回零、`0x20` 读状态。
存衣命令当前只支持 `position=1`；取衣命令支持 `position=1..255`，数值直接表示 STM32 从 X 原点扫描到的第 N 块三级定位板，不再提供 `position=5` 到 1 号位的别名映射。

当前阿里云自定义 Topic 同时具备发布和订阅权限，ESP32 使用同一个 Topic 收发消息。收到带 `status` 字段的响应消息时，ESP32 会忽略，避免处理自己发布的上行响应。

## 配置

运行 `idf.py menuconfig` 进入各组件配置菜单：

| 组件 | 关键配置项 |
|------|-----------|
| WiFi | SSID、密码、重试次数 |
| MQTT | Broker 地址/端口、客户端 ID、用户名/密码、SSL |
| UART | 串口波特率、TX/RX 引脚 |

## 编译与烧录

```bash
idf.py build          # 编译
idf.py -p PORT flash  # 烧录
idf.py monitor        # 查看串口日志
```

## 项目结构

```
├── main/              # 入口 (app_main)
├── app/               # 应用层 (各组件初始化与绑定)
├── components/
│   ├── wifi/          # WiFi STA/AP 组件
│   ├── esp_mqtt/      # MQTT 客户端组件
│   └── uart/          # UART 串口协议组件
├── sdkconfig          # 项目配置
└── partitions.csv     # 分区表
```

## 依赖

- ESP-IDF v5.5
- 目标芯片: ESP32-S3
- 外部依赖: cJSON、esp_mqtt
