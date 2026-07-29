#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能衣柜 UART 二进制协议解析器。

用途：
- 不修改 ESP32 和 STM32 固件；
- 在电脑端通过 USB-TTL 监听串口数据；
- 把 A5 CMD LEN DATA CRC 5A 二进制帧解析成人能看的文本。
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Iterable

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


SOF = 0xA5
EOF = 0x5A
MAX_DATA_LEN = 128


CMD_NAMES = {
    0x10: "ESP->STM 存衣服",
    0x11: "ESP->STM 取衣服",
    0x12: "ESP->STM 回零",
    0x20: "ESP->STM 读状态",
    0x90: "STM->ESP 操作成功",
    0x91: "STM->ESP 故障",
    0xA0: "STM->ESP 状态上报",
}


FAULT_NAMES = {
    0x01: "motor_overcurrent",
    0x02: "motor_stall",
    0x03: "limit_switch_error",
    0x04: "sensor_fault",
    0x05: "communication_timeout",
    0x06: "eeprom_error",
    0x07: "slot_occupied",
    0x08: "slot_empty",
    0xFF: "unknown_fault",
}


@dataclass(frozen=True)
class WardrobeFrame:
    raw: bytes
    cmd: int
    data: bytes
    crc: int
    crc_ok: bool


def calc_crc(cmd: int, data: bytes) -> int:
    """计算项目协议中的 CRC：CMD ^ LEN ^ DATA..."""
    crc = cmd ^ len(data)
    for byte in data:
        crc ^= byte
    return crc & 0xFF


def hex_bytes(data: Iterable[int]) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def parse_hex_text(text: str) -> bytes:
    cleaned = text.replace(",", " ").replace("0x", " ").replace("0X", " ")
    parts = [part for part in cleaned.split() if part]
    return bytes(int(part, 16) for part in parts)


def decode_frame(raw: bytes) -> WardrobeFrame:
    if len(raw) < 5:
        raise ValueError("帧长度不足，至少需要 5 字节")
    if raw[0] != SOF:
        raise ValueError(f"帧头错误，应为 A5，实际为 {raw[0]:02X}")
    if raw[-1] != EOF:
        raise ValueError(f"帧尾错误，应为 5A，实际为 {raw[-1]:02X}")

    cmd = raw[1]
    data_len = raw[2]
    expected_len = data_len + 5
    if data_len > MAX_DATA_LEN:
        raise ValueError(f"DATA 长度过大：{data_len}")
    if len(raw) != expected_len:
        raise ValueError(f"帧长度不匹配，LEN={data_len} 时完整帧应为 {expected_len} 字节，实际 {len(raw)} 字节")

    data = raw[3 : 3 + data_len]
    crc = raw[3 + data_len]
    return WardrobeFrame(
        raw=raw,
        cmd=cmd,
        data=data,
        crc=crc,
        crc_ok=(calc_crc(cmd, data) == crc),
    )


def explain_frame(frame: WardrobeFrame) -> str:
    lines = [
        f"原始帧: {hex_bytes(frame.raw)}",
        f"命令: {CMD_NAMES.get(frame.cmd, f'未知命令 0x{frame.cmd:02X}')}",
        f"长度: {len(frame.data)}",
        f"DATA: {hex_bytes(frame.data) if frame.data else '(空)'}",
        f"CRC: 0x{frame.crc:02X} {'OK' if frame.crc_ok else '错误'}",
    ]

    if frame.cmd in (0x10, 0x11) and len(frame.data) == 1:
        lines.append(f"位置 position: {frame.data[0]}")
    elif frame.cmd == 0x12 and len(frame.data) == 0:
        lines.append("动作: 回零")
    elif frame.cmd == 0x20 and len(frame.data) == 0:
        lines.append("动作: 读取状态")
    elif frame.cmd == 0x90 and len(frame.data) == 1:
        lines.append(f"结果: 成功，position={frame.data[0]}")
    elif frame.cmd == 0x91 and len(frame.data) == 2:
        code = frame.data[0]
        position = frame.data[1]
        position_text = "无指定位置" if position == 0xFF else str(position)
        lines.append(
            f"结果: 故障，code=0x{code:02X}, "
            f"msg={FAULT_NAMES.get(code, 'unknown_fault')}, position={position_text}"
        )
    elif frame.cmd == 0xA0 and len(frame.data) == 6:
        clothing_count = (frame.data[0] << 8) | frame.data[1]
        uptime_days = (frame.data[2] << 8) | frame.data[3]
        lines.append(
            f"状态: 衣物数量={clothing_count}, "
            f"运行={uptime_days}天 {frame.data[4]}小时 {frame.data[5]}分钟"
        )

    return "\n".join(lines)


class StreamParser:
    def __init__(self, show_noise: bool = False) -> None:
        self._buf = bytearray()
        self._show_noise = show_noise

    def feed(self, chunk: bytes) -> list[WardrobeFrame]:
        frames: list[WardrobeFrame] = []
        for byte in chunk:
            frames.extend(self._feed_byte(byte))
        return frames

    def _feed_byte(self, byte: int) -> list[WardrobeFrame]:
        if not self._buf:
            if byte == SOF:
                self._buf.append(byte)
            elif self._show_noise:
                print(f"丢弃非帧头字节: {byte:02X}")
            return []

        self._buf.append(byte)

        if len(self._buf) == 3 and self._buf[2] > MAX_DATA_LEN:
            print(f"非法 LEN={self._buf[2]}，丢弃: {hex_bytes(self._buf)}")
            self._buf.clear()
            return []

        if len(self._buf) >= 3:
            expected_len = self._buf[2] + 5
            if len(self._buf) == expected_len:
                raw = bytes(self._buf)
                self._buf.clear()
                try:
                    return [decode_frame(raw)]
                except ValueError as exc:
                    print(f"无效帧: {exc}; raw={hex_bytes(raw)}")

        return []


def run_serial(port: str, baud: int, show_noise: bool) -> None:
    if serial is None:
        raise RuntimeError("缺少 pyserial，请先运行：python -m pip install pyserial")

    parser = StreamParser(show_noise=show_noise)
    with serial.Serial(port, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=0.1) as ser:
        print(f"正在监听 {port}，{baud} 8N1。按 Ctrl+C 退出。")
        while True:
            chunk = ser.read(128)
            if not chunk:
                continue
            for frame in parser.feed(chunk):
                print()
                print(explain_frame(frame))


def run_hex_once(hex_text: str) -> None:
    raw = parse_hex_text(hex_text)
    frame = decode_frame(raw)
    print(explain_frame(frame))


def main(argv: list[str]) -> int:
    arg_parser = argparse.ArgumentParser(description="智能衣柜 UART 二进制协议解析器")
    arg_parser.add_argument("port", nargs="?", help="串口号，例如 COM5")
    arg_parser.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    arg_parser.add_argument("--hex", dest="hex_text", help="直接解析一帧 HEX 文本，不打开串口")
    arg_parser.add_argument("--show-noise", action="store_true", help="显示被丢弃的非帧头字节")
    args = arg_parser.parse_args(argv)

    try:
        if args.hex_text:
            run_hex_once(args.hex_text)
            return 0
        if not args.port:
            arg_parser.error("请提供串口号，例如 COM5，或使用 --hex 解析单帧")
        run_serial(args.port, args.baud, args.show_noise)
    except KeyboardInterrupt:
        print("\n已退出。")
        return 0
    except Exception as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
