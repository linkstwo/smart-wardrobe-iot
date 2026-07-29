# 智能衣柜串口协议解析工具

这个工具不修改 ESP32 和 STM32 固件，只在电脑端解析串口二进制帧。

## 用途

当前 ESP32 和 STM32 之间使用 `A5 CMD LEN DATA CRC 5A` 二进制协议。普通串口助手按文本显示时会像乱码，本工具会把二进制帧翻译成人能看的内容。

## 安装依赖

```powershell
python -m pip install pyserial
```

## 监听串口

```powershell
python "wardrobe_uart_parser.py" COM5
```

默认参数是 `115200 8N1`。

如果你的串口号不是 `COM5`，替换成设备管理器里看到的串口号。

## 直接解析一帧 HEX

```powershell
python "wardrobe_uart_parser.py" --hex "A5 10 01 01 10 5A"
```

输出示例：

```text
原始帧: A5 10 01 01 10 5A
命令: ESP->STM 存衣服
长度: 1
DATA: 01
CRC: 0x10 OK
位置 position: 1
```

## 常见帧

```text
A5 10 01 01 10 5A  存 1 号位
A5 11 01 01 11 5A  取 1 号位
A5 12 00 12 5A     回零
A5 20 00 20 5A     读状态
```

## 接线提醒

- ESP32 和 STM32 联调时，ESP32 UART1 默认是 `GPIO17 TX / GPIO18 RX`。
- STM32 的 ESP 通信口是 `USART3`，默认 `115200 8N1`。
- 如果 USB-TTL 只是监听，建议只接 GND 和被监听设备的 TX 到 USB-TTL RX。
- 不建议让 ESP32 TX 和 USB-TTL TX 同时接到 STM32 RX，否则两个发送端会抢线，容易产生乱码或丢帧。
