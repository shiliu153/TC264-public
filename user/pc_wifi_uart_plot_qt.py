from __future__ import annotations

import argparse
from collections import deque
import ctypes
import math
import numpy as np
import queue
import re
import socket
import threading
import time
import json
import winsound
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


_NUMBER_RE = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")


_FRAME_MAGIC = b"\xA5\x5A"
_FRAME_TYPE_FAST = 0x01
_FRAME_TYPE_ROUTE = 0x02
_FRAME_TYPE_POWER = 0x03
_FRAME_TYPE_GRAY = 0x04
_FRAME_TYPE_GRAY_RAW = 0x05
_FRAME_TYPE_PERF = 0x06
_FRAME_TYPE_CONTROL_CMD = 0x10
_FRAME_TYPE_CONTROL_ACK = 0x11
_ANGLE_SCALE = 100.0
_GYRO_SCALE = 10.0
_SPEED_SCALE = 10.0
_VQ_SCALE = 1000.0
_VOLT_SCALE = 100.0
_NP_PERCENT_SCALE = 10.0
_NP_PULSE_US_SCALE = 1.0
_DEFAULT_SENSOR_COLS = 16
_PARAM_DECIMALS = 5
_FOC_TEST_RUN_KEY_DEFAULT = "TST_RUN"

_BTN_ACTION_GREEN = (
    "QPushButton { background:#059669; color:#ffffff; border:1px solid #047857; border-radius:6px; padding:6px 16px; font:700 12px; }"
    "QPushButton:hover { background:#07b87a; border-color:#059669; }"
    "QPushButton:pressed { background:#047857; border-color:#065f46; }"
    "QPushButton:disabled { background:#d1d5db; color:#9ca3af; border-color:#d1d5db; }"
)
_BTN_ACTION_RED = (
    "QPushButton { background:#dc2626; color:#ffffff; border:1px solid #b91c1c; border-radius:6px; padding:6px 16px; font:700 12px; }"
    "QPushButton:hover { background:#ef4444; border-color:#dc2626; }"
    "QPushButton:pressed { background:#b91c1c; border-color:#991b1b; }"
    "QPushButton:disabled { background:#d1d5db; color:#9ca3af; border-color:#d1d5db; }"
)
_FOC_TEST_LEFT_KEY_DEFAULT = "LTST_RPM"
_FOC_TEST_RIGHT_KEY_DEFAULT = "RTST_RPM"
_BOARD_PAGE_MAIN = "MAIN"
_BOARD_PAGE_RUN = "RUN"
_BOARD_PAGE_FOC = "FOC"
_BOARD_PAGE_LINE = "LINE"
_BOARD_PAGE_JUNCTION = "JUNCTION"
_BOARD_PAGE_GRAY = "GRAY"
_BOARD_PAGE_IMU = "IMU"
_BOARD_PAGE_NEG = "NEG"
_BOARD_PAGE_OPENLOOP = "OPENLOOP"
_BOARD_PAGE_WIFI = "WIFI"
_CTRL_OPCODE_START = 0x01
_CTRL_OPCODE_PAUSE = 0x02
_CTRL_OPCODE_ODOM_START = 0x03
_CTRL_OPCODE_ODOM_CLEAR = 0x04
_CTRL_OPCODE_NPON = 0x05
_CTRL_OPCODE_NPOFF = 0x06
_CTRL_STATUS_OK = 0x00
_CTRL_STATUS_BUSY = 0x01
_CTRL_STATUS_INVALID_ARG = 0x02
_CTRL_STATUS_CALIB_NOT_READY = 0x03
_CTRL_STATUS_FOC_TEST_RUNNING = 0x04
_CTRL_STATUS_UNKNOWN_CMD = 0x05
_RUN_CTRL_ACK_TIMEOUT_S = 0.150
_RUN_CTRL_MAX_RETRIES = 3
_FAST_FRAME_PAYLOAD_LEN = 22
_ROUTE_FRAME_PAYLOAD_LEN = 29
_POWER_FRAME_PAYLOAD_LEN = 10

_CTRL_OPCODE_TO_NAME = {
    _CTRL_OPCODE_START: "START",
    _CTRL_OPCODE_PAUSE: "PAUSE",
    _CTRL_OPCODE_ODOM_START: "ODOM_START",
    _CTRL_OPCODE_ODOM_CLEAR: "ODOM_CLEAR",
    _CTRL_OPCODE_NPON: "NPON",
    _CTRL_OPCODE_NPOFF: "NPOFF",
}
_CTRL_NAME_TO_OPCODE = {name: opcode for opcode, name in _CTRL_OPCODE_TO_NAME.items()}
_CTRL_STATUS_TO_TEXT = {
    _CTRL_STATUS_OK: "OK",
    _CTRL_STATUS_BUSY: "BUSY",
    _CTRL_STATUS_INVALID_ARG: "INVALID_ARG",
    _CTRL_STATUS_CALIB_NOT_READY: "CALIB_NOT_READY",
    _CTRL_STATUS_FOC_TEST_RUNNING: "FOC_TEST_RUNNING",
    _CTRL_STATUS_UNKNOWN_CMD: "UNKNOWN_CMD",
}


@dataclass(frozen=True)
class RxEvent:
    t: float
    raw_line: str
    values: Optional[Tuple[float, ...]]
    param: Optional[Tuple[str, str]] = None
    frame_type: Optional[int] = None
    control_ack: Optional[Tuple[int, int, int]] = None


@dataclass
class ConnectionState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    conn: Optional[socket.socket] = None
    addr: Optional[Tuple[str, int]] = None
    rx_thread: Optional[threading.Thread] = None
    connected_at: float = 0.0
    last_rx_time: float = 0.0
    raw_rx_count: int = 0
    raw_text_count: int = 0


@dataclass
class RateSnapshot:
    raw_rx_hz: float = 0.0
    ui_rx_hz: float = 0.0
    raw_text_hz: float = 0.0
    ui_text_hz: float = 0.0


class WindowsTimerResolution:
    def __init__(self) -> None:
        self.enabled = False
        self._winmm = None

    def enable(self) -> None:
        if self.enabled:
            return
        try:
            self._winmm = ctypes.WinDLL("winmm")
            if self._winmm.timeBeginPeriod(1) == 0:
                self.enabled = True
        except Exception:
            self._winmm = None

    def disable(self) -> None:
        if not self.enabled or self._winmm is None:
            return
        try:
            self._winmm.timeEndPeriod(1)
        finally:
            self.enabled = False


def configure_client_socket(conn: socket.socket) -> None:
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    if hasattr(socket, "SIO_KEEPALIVE_VALS"):
        conn.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 2000, 1000))


def queue_put_drop_oldest(out_q: queue.Queue, item: object) -> None:
    try:
        out_q.put_nowait(item)
        return
    except queue.Full:
        pass

    try:
        out_q.get_nowait()
    except queue.Empty:
        pass

    try:
        out_q.put_nowait(item)
    except queue.Full:
        pass


def push_log(log_q: "queue.Queue[str]", text: str) -> None:
    timestamp = time.strftime("%H:%M:%S")
    queue_put_drop_oldest(log_q, f"[{timestamp}] {text}")


def parse_numbers_from_line(line: str) -> Optional[Tuple[float, ...]]:
    nums = _NUMBER_RE.findall(line)
    if not nums:
        return None
    try:
        return tuple(float(x) for x in nums)
    except ValueError:
        return None


def checksum_frame(data: bytes) -> int:
    return sum(data) & 0xFF


def unpack_s16_le(data: bytes, offset: int) -> int:
    value = data[offset] | (data[offset + 1] << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def decode_fast_frame(payload: bytes) -> Optional[Tuple[float, ...]]:
    if len(payload) != 22:
        return None
    return (
        float(payload[0]),
        unpack_s16_le(payload, 1) / _ANGLE_SCALE,
        unpack_s16_le(payload, 3) / _ANGLE_SCALE,
        unpack_s16_le(payload, 5) / _ANGLE_SCALE,
        unpack_s16_le(payload, 7) / _GYRO_SCALE,
        unpack_s16_le(payload, 9) / _SPEED_SCALE,
        unpack_s16_le(payload, 11) / _SPEED_SCALE,
        unpack_s16_le(payload, 13) / _SPEED_SCALE,
        unpack_s16_le(payload, 15) / _SPEED_SCALE,
        unpack_s16_le(payload, 17) / _VQ_SCALE,
        unpack_s16_le(payload, 19) / _VQ_SCALE,
        float(payload[21]),
    )


def decode_route_frame(payload: bytes) -> Optional[Tuple[float, ...]]:
    if len(payload) != 29:
        return None
    return (
        float(payload[0]),
        unpack_s16_le(payload, 1) / _ANGLE_SCALE,
        unpack_s16_le(payload, 3) / _SPEED_SCALE,
        float(payload[5] | (payload[6] << 8)),
        float(payload[7]),
        float(payload[8]),
        float(payload[9]),
        float(payload[10]),
        float(payload[11]),
        float(payload[12]),
        float(payload[13]),
        float(payload[14]),
        float(payload[15]),
        float(payload[16]),
        float(payload[17]),
        float(payload[18]),
        unpack_s16_le(payload, 19) / _ANGLE_SCALE,
        unpack_s16_le(payload, 21) / _ANGLE_SCALE,
        unpack_s16_le(payload, 23) / _ANGLE_SCALE,
        float(unpack_s16_le(payload, 25)),
        float(unpack_s16_le(payload, 27)),
    )


def decode_power_frame(payload: bytes) -> Optional[Tuple[float, ...]]:
    if len(payload) != 10:
        return None
    return (
        float(payload[0]),
        unpack_s16_le(payload, 1) / _VOLT_SCALE,
        float(payload[3]),
        float(payload[4]),
        unpack_s16_le(payload, 5) / _NP_PERCENT_SCALE,
        unpack_s16_le(payload, 7) / _NP_PULSE_US_SCALE,
        float(payload[9]),
    )


def decode_gray_frame(payload: bytes, sensor_cols: int) -> Optional[Tuple[float, ...]]:
    sensor_cols = max(0, int(sensor_cols))
    if len(payload) != (1 + sensor_cols):
        return None
    values: List[float] = [float(payload[0])]
    for index in range(sensor_cols):
        values.append(float(payload[1 + index]))
    return tuple(values)


def decode_gray_raw_frame(payload: bytes, sensor_cols: int) -> Optional[Tuple[float, ...]]:
    sensor_cols = max(0, int(sensor_cols))
    if len(payload) != (1 + sensor_cols * 2):
        return None
    values: List[float] = [float(payload[0])]
    for index in range(sensor_cols):
        lo = payload[1 + index * 2]
        hi = payload[1 + index * 2 + 1]
        values.append(float(lo | (hi << 8)))
    return tuple(values)


def decode_perf_frame(payload: bytes) -> Optional[Tuple[float, ...]]:
    if len(payload) < 17:
        return None
    cpu0_load = float((payload[2] | (payload[3] << 8))) / 100.0
    cpu1_load = float((payload[4] | (payload[5] << 8))) / 100.0
    foc_us = float((payload[6] | (payload[7] << 8)))
    isr1k_us = float((payload[8] | (payload[9] << 8)))
    foc_pct = float(payload[10])
    window_ms = float((payload[11] | (payload[12] << 8)))
    return (cpu0_load, cpu1_load, foc_us, isr1k_us, foc_pct, window_ms)


def try_parse_binary_frame(buf: bytearray, log_q: "queue.Queue[str]", sensor_cols: int) -> Tuple[Optional[RxEvent], bool]:
    if len(buf) < 5:
        return None, False
    if buf[0:2] != _FRAME_MAGIC:
        return None, False

    frame_type = buf[2]
    flags = buf[3]
    payload_len = buf[4]
    frame_len = 6 + payload_len
    if len(buf) < frame_len:
        return None, False

    payload = bytes(buf[5 : 5 + payload_len])
    checksum = buf[5 + payload_len]
    expect_checksum = checksum_frame(bytes(buf[2 : 5 + payload_len]))
    if checksum != expect_checksum:
        del buf[0]
        push_log(log_q, "检测到损坏的二进制帧，已尝试重同步")
        return None, True

    del buf[:frame_len]

    if frame_type == _FRAME_TYPE_FAST:
        values = decode_fast_frame(payload)
    elif frame_type == _FRAME_TYPE_ROUTE:
        values = decode_route_frame(payload)
    elif frame_type == _FRAME_TYPE_POWER:
        values = decode_power_frame(payload)
    elif frame_type == _FRAME_TYPE_GRAY:
        values = decode_gray_frame(payload, sensor_cols)
    elif frame_type == _FRAME_TYPE_GRAY_RAW:
        values = decode_gray_raw_frame(payload, sensor_cols)
    elif frame_type == _FRAME_TYPE_PERF:
        values = decode_perf_frame(payload)
    elif frame_type == _FRAME_TYPE_CONTROL_ACK:
        if len(payload) != 3:
            push_log(log_q, f"收到长度错误的控制ACK帧: len={payload_len}")
            return None, True
        return RxEvent(
            t=time.perf_counter(),
            raw_line=f"CTRL_ACK seq={payload[0]} op=0x{payload[1]:02X} st=0x{payload[2]:02X}",
            values=None,
            frame_type=frame_type,
            control_ack=(int(payload[0]), int(payload[1]), int(payload[2])),
        ), True
    else:
        push_log(log_q, f"收到未知二进制帧类型: {frame_type}")
        return None, True

    if values is None:
        push_log(log_q, f"收到长度不匹配的二进制帧: type=0x{frame_type:02X} len={payload_len} flags=0x{flags:02X}")
        return None, True

    return RxEvent(
        t=time.perf_counter(),
        raw_line=f"BIN type=0x{frame_type:02X} len={payload_len}",
        values=values,
        frame_type=frame_type,
    ), True


def try_parse_text_line(buf: bytearray) -> Tuple[Optional[RxEvent], bool]:
    nl = buf.find(b"\n")
    if nl < 0:
        return None, False

    line_bytes = bytes(buf[:nl])
    del buf[: nl + 1]
    line = line_bytes.decode("utf-8", errors="ignore").strip("\r")
    if not line:
        return None, True

    param_kv = None
    if line.startswith("PARAM:"):
        parts = line[6:].split("=", 1)
        if len(parts) == 2:
            param_kv = (parts[0].strip(), parts[1].strip())
            return RxEvent(t=time.perf_counter(), raw_line=line, values=None, param=param_kv), True
    elif line.startswith("PAGEPARAM_END:"):
        return RxEvent(t=time.perf_counter(), raw_line=line, values=None, param=("PAGEPARAM_END", line[14:].strip())), True
    elif line.startswith("PAGEPARAM:"):
        return RxEvent(t=time.perf_counter(), raw_line=line, values=None, param=("PAGEPARAM", line[10:].strip())), True
    elif line.startswith("PAGE:"):
        return RxEvent(t=time.perf_counter(), raw_line=line, values=None, param=("PAGE", line[5:].strip())), True
    elif line.startswith("ACK:"):
        return RxEvent(t=time.perf_counter(), raw_line=line, values=None, param=("ACK", line[4:].strip())), True
    elif line.startswith("NACK:"):
        # Treat NACK as an error for unified UI logging.
        return RxEvent(t=time.perf_counter(), raw_line=line, values=None, param=("ERR", line[5:].strip())), True
    elif line.startswith("ERR:"):
        return RxEvent(t=time.perf_counter(), raw_line=line, values=None, param=("ERR", line[4:].strip())), True

    return RxEvent(t=time.perf_counter(), raw_line=line, values=parse_numbers_from_line(line)), True


def try_extract_rx_event(buf: bytearray, log_q: "queue.Queue[str]", sensor_cols: int) -> Tuple[Optional[RxEvent], bool]:
    if not buf:
        return None, False

    if len(buf) >= 2 and buf[0:2] == _FRAME_MAGIC:
        return try_parse_binary_frame(buf, log_q, sensor_cols)

    text_event, progressed = try_parse_text_line(buf)
    if progressed:
        return text_event, True

    magic_index = buf.find(_FRAME_MAGIC)
    if magic_index > 0:
        del buf[:magic_index]
        return None, True

    if len(buf) > 4096:
        del buf[:-2]
        push_log(log_q, "接收缓存异常增长，已丢弃部分无效数据")
        return None, True

    return None, False


def control_opcode_to_name(opcode: int) -> str:
    return _CTRL_OPCODE_TO_NAME.get(int(opcode), f"0x{int(opcode) & 0xFF:02X}")


def control_status_to_text(status: int) -> str:
    return _CTRL_STATUS_TO_TEXT.get(int(status), f"0x{int(status) & 0xFF:02X}")


def lf_state_to_text(state: Optional[int]) -> str:
    mapping = {
        0: "循迹",
        1: "预转向",
        2: "左转",
        3: "右转",
    }
    if state is None:
        return "--"
    return mapping.get(int(state), f"S{int(state)}")


def bool_text(value: Optional[int]) -> str:
    if value is None:
        return "--"
    return "开" if int(value) else "关"


def np_state_text(enable: Optional[int], arming: Optional[int], running: Optional[int]) -> str:
    if arming is None and running is None and enable is None:
        return "--"
    if arming:
        return "ARM"
    if running:
        return "RUN"
    return "IDLE"


def turn_to_text(turn: Optional[int]) -> str:
    if turn is None:
        return "未知"
    mapping = {
        0: "直行",
        1: "左转",
        2: "右转",
        3: "按键",
        4: "灯泡",
        5: "电池_1型",
        6: "电池_2型",
        7: "电感_1型",
        8: "电感_2型",
        9: "电流源",
        10: "电容",
        11: "电压源",
        12: "电源_2型",
        13: "电阻_1型",
        14: "电阻_2型",
        15: "定距巡线",
        16: "二极管_分叉_左",
        17: "二级管_分叉_右",
        18: "发光二极管",
        19: "非门",
        20: "三极管_1型",
        21: "三极管_2型",
        22: "分叉左_转弯",
        23: "分叉右_转弯",
    }
    turn_i = int(turn)
    return mapping.get(turn_i, f"未知({turn_i})")


def stop_reason_to_text(reason: Optional[int]) -> str:
    if reason is None:
        return "--"
    mapping = {
        0: "无",
        1: "丢线",
        2: "路线完成",
        3: "低电压",
        4: "系统错误",
        5: "姿态倾覆",
    }
    reason_i = int(reason)
    return mapping.get(reason_i, f"未知({reason_i})")


def fmt_float(value: Optional[float], width: int = 7) -> str:
    return "--" if value is None else f"{value:{width}.2f}"


def cleanup_connection(state: ConnectionState, target_conn: socket.socket) -> None:
    with state.lock:
        if state.conn is target_conn:
            state.conn = None
            state.addr = None
            state.rx_thread = None
            state.connected_at = 0.0
            state.last_rx_time = 0.0
            state.raw_rx_count = 0
            state.raw_text_count = 0
    try:
        target_conn.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    try:
        target_conn.close()
    except OSError:
        pass


def receiver_loop(
    conn: socket.socket,
    state: ConnectionState,
    text_q: "queue.Queue[RxEvent]",
    data_q: "queue.Queue[RxEvent]",
    log_q: "queue.Queue[str]",
    stop_evt: threading.Event,
    sensor_cols: int,
) -> None:
    conn.settimeout(0.2)
    buf = bytearray()
    try:
        while not stop_evt.is_set():
            try:
                chunk = conn.recv(4096)
                if not chunk:
                    push_log(log_q, "连接已被远端关闭")
                    break

                with state.lock:
                    if state.conn is conn:
                        state.last_rx_time = time.time()

                buf.extend(chunk)
                while True:
                    event, progressed = try_extract_rx_event(buf, log_q, sensor_cols)
                    if not progressed:
                        break
                    if event is None:
                        continue

                    if event.values is None:
                        with state.lock:
                            if state.conn is conn:
                                state.raw_text_count += 1
                        queue_put_drop_oldest(text_q, event)
                        push_log(log_q, f"RX {event.raw_line}")
                        continue

                    with state.lock:
                        if state.conn is conn:
                            state.raw_rx_count += 1
                    queue_put_drop_oldest(data_q, event)

            except socket.timeout:
                continue
            except OSError as exc:
                if not stop_evt.is_set():
                    with state.lock:
                        still_active = state.conn is conn
                    # Socket may have been closed by timeout/reconnect path in another thread.
                    if still_active and (getattr(exc, "winerror", None) != 10038):
                        push_log(log_q, f"接收异常: {exc}")
                break
    finally:
        cleanup_connection(state, conn)


def accept_loop(
    server: socket.socket,
    state: ConnectionState,
    text_q: "queue.Queue[RxEvent]",
    data_q: "queue.Queue[RxEvent]",
    log_q: "queue.Queue[str]",
    stop_evt: threading.Event,
    sensor_cols: int,
) -> None:
    server.settimeout(0.5)
    while not stop_evt.is_set():
        with state.lock:
            active_conn = state.conn
            active_rx_thread = state.rx_thread

        if active_conn is not None:
            if active_rx_thread is not None and not active_rx_thread.is_alive():
                cleanup_connection(state, active_conn)
            stop_evt.wait(0.05)
            continue

        try:
            conn, addr = server.accept()
        except socket.timeout:
            continue
        except OSError:
            break

        configure_client_socket(conn)
        rx_thread = threading.Thread(
            target=receiver_loop,
            args=(conn, state, text_q, data_q, log_q, stop_evt, sensor_cols),
            daemon=True,
        )

        with state.lock:
            state.conn = conn
            state.addr = (addr[0], addr[1])
            state.rx_thread = rx_thread
            state.connected_at = time.time()
            state.last_rx_time = state.connected_at
            state.raw_rx_count = 0
            state.raw_text_count = 0

        push_log(log_q, f"客户端已连接 {addr[0]}:{addr[1]}")
        rx_thread.start()


def send_command(state: ConnectionState, log_q: "queue.Queue[str]", command: str) -> bool:
    cmd = command.strip()
    if not cmd:
        push_log(log_q, "发送失败: 指令为空")
        return False

    with state.lock:
        conn = state.conn

    if conn is None:
        push_log(log_q, f"发送失败: 未连接, 指令={cmd}")
        return False

    try:
        conn.sendall((cmd + "\n").encode("utf-8"))
        push_log(log_q, f"TX {cmd}")
        return True
    except OSError as exc:
        push_log(log_q, f"发送失败: {exc}")
        cleanup_connection(state, conn)
        return False


def send_raw_frame(state: ConnectionState, log_q: "queue.Queue[str]", frame: bytes, log_text: str) -> bool:
    if not frame:
        push_log(log_q, "发送失败: 二进制帧为空")
        return False

    with state.lock:
        conn = state.conn

    if conn is None:
        push_log(log_q, f"发送失败: 未连接, 帧={log_text}")
        return False

    try:
        conn.sendall(frame)
        push_log(log_q, f"TX {log_text}")
        return True
    except OSError as exc:
        push_log(log_q, f"发送失败: {exc}")
        cleanup_connection(state, conn)
        return False


def build_control_frame(seq: int, opcode: int, args: bytes = b"", flags: int = 0) -> bytes:
    payload = bytes([seq & 0xFF, opcode & 0xFF]) + bytes(args)
    header = bytes([_FRAME_TYPE_CONTROL_CMD, flags & 0xFF, len(payload) & 0xFF])
    checksum = checksum_frame(header + payload)
    return _FRAME_MAGIC + header + payload + bytes([checksum])


def reconnect_client(state: ConnectionState, log_q: "queue.Queue[str]") -> None:
    with state.lock:
        conn = state.conn
        addr = state.addr

    if conn is None:
        push_log(log_q, "重新连接: 当前没有活动连接，继续监听中")
        return
    if addr is None:
        push_log(log_q, "重新连接: 正在断开当前连接并重新等待")
    else:
        push_log(log_q, f"重新连接: 断开 {addr[0]}:{addr[1]}，重新等待设备接入")
    cleanup_connection(state, conn)


def check_connection_idle_timeout(state: ConnectionState, log_q: "queue.Queue[str]", idle_timeout_s: float) -> None:
    if idle_timeout_s <= 0.0:
        return

    with state.lock:
        conn = state.conn
        last_rx_time = state.last_rx_time
        addr = state.addr

    if conn is None or addr is None or last_rx_time <= 0.0:
        return

    idle_s = time.time() - last_rx_time
    if idle_s < idle_timeout_s:
        return

    push_log(log_q, f"连接超时: {addr[0]}:{addr[1]} 在 {idle_s:.1f}s 内无数据，已判定断开")
    cleanup_connection(state, conn)


class TelemetryModel:
    def __init__(self, cols: int) -> None:
        self.cols = cols
        self.reset()

    def reset(self) -> None:
        self.roll: Optional[float] = None
        self.pitch: Optional[float] = None
        self.yaw: Optional[float] = None
        self.gx: Optional[float] = None
        self.gy: Optional[float] = None
        self.gz: Optional[float] = None
        self.speed_l_target: Optional[float] = None
        self.speed_r_target: Optional[float] = None
        self.speed_l_actual: Optional[float] = None
        self.speed_r_actual: Optional[float] = None
        self.left_vq: Optional[float] = None
        self.right_vq: Optional[float] = None
        self.elec_l_deg: Optional[float] = None
        self.elec_r_deg: Optional[float] = None
        self.battery_v: Optional[float] = None
        self.battery_percent: Optional[int] = None
        self.battery_low: Optional[int] = None
        self.battery_critical: Optional[int] = None
        self.np_enable: Optional[int] = None
        self.np_active: Optional[int] = None
        self.np_arming: Optional[int] = None
        self.np_running: Optional[int] = None
        self.np_throttle: Optional[float] = None
        self.np_pulse_us: Optional[float] = None
        self.line_pos: Optional[float] = None
        self.delta_rpm: Optional[float] = None
        self.line_bits: Optional[int] = None
        self.run_enable: Optional[int] = None
        self.pos_valid: Optional[int] = None
        self.line_ready: Optional[int] = None
        self.lf_state: Optional[int] = None
        self.pending_turn: Optional[int] = None
        self.route_index: Optional[int] = None
        self.lost_line_cnt: Optional[int] = None
        self.stop_reason: Optional[int] = None
        self.junction_lock_cnt: Optional[int] = None
        self.element_enter_cnt: Optional[int] = None
        self.junction_enter_cnt: Optional[int] = None
        self.lf_actual_hz: Optional[int] = None
        self.calib_active: Optional[int] = None
        self.calib_samples_hi: Optional[int] = None
        self.calib_valid: Optional[int] = None
        self.turn_yaw_deg: Optional[float] = None
        self.odom_left_mm: Optional[float] = None
        self.odom_right_mm: Optional[float] = None
        self.odom_count_enable: Optional[int] = None
        self.sensors: List[float] = [0.0] * self.cols
        self.last_text_rx = "--"
        self.cpu0_load: float = 0.0
        self.cpu1_load: float = 0.0
        self.foc_us: float = 0.0
        self.isr1k_us: float = 0.0
        self.foc_pct: float = 0.0
        self.rx_total = 0
        self.text_total = 0
        self._calib_text_lines: List[str] = []

    def apply_text(self, item: RxEvent) -> None:
        self.last_text_rx = item.raw_line
        self.text_total += 1
        if item.raw_line.startswith("MIN:") or item.raw_line.startswith("MAX:"):
            self._calib_text_lines.append(item.raw_line)

    def apply_data(self, item: RxEvent) -> None:
        if item.values is None:
            return

        frame_type = item.frame_type
        values = item.values

        if frame_type == _FRAME_TYPE_FAST:
            (
                _seq,
                self.roll,
                self.pitch,
                self.yaw,
                self.gz,
                self.speed_l_target,
                self.speed_r_target,
                self.speed_l_actual,
                self.speed_r_actual,
                self.left_vq,
                self.right_vq,
                status_flags_raw,
            ) = values
            status_flags = int(status_flags_raw)
            self.gx = None
            self.gy = None
            self.run_enable = 1 if (status_flags & 0x01) else 0
            self.pos_valid = 1 if (status_flags & 0x02) else 0
            self.line_ready = 1 if (status_flags & 0x04) else 0
            self.odom_count_enable = 1 if (status_flags & 0x08) else 0
            if self.pos_valid == 0:
                self.line_pos = None
        elif frame_type == _FRAME_TYPE_ROUTE:
            (
                _seq,
                line_pos,
                self.delta_rpm,
                line_bits,
                lf_state,
                pending_turn,
                route_index,
                lost_line_cnt,
                stop_reason,
                junction_lock_cnt,
                element_enter_cnt,
                junction_enter_cnt,
                lf_actual_hz,
                calib_active,
                calib_samples_hi,
                calib_valid,
                self.turn_yaw_deg,
                self.elec_l_deg,
                self.elec_r_deg,
                self.odom_left_mm,
                self.odom_right_mm,
            ) = values
            self.line_bits = int(line_bits)
            self.lf_state = int(lf_state)
            self.pending_turn = int(pending_turn)
            self.route_index = int(route_index)
            self.lost_line_cnt = int(lost_line_cnt)
            self.stop_reason = int(stop_reason)
            self.junction_lock_cnt = int(junction_lock_cnt)
            self.element_enter_cnt = int(element_enter_cnt)
            self.junction_enter_cnt = int(junction_enter_cnt)
            self.lf_actual_hz = int(lf_actual_hz)
            self.calib_active = int(calib_active)
            self.calib_samples_hi = int(calib_samples_hi)
            self.calib_valid = int(calib_valid)
            self.line_pos = None if self.pos_valid == 0 else line_pos
        elif frame_type == _FRAME_TYPE_POWER:
            (
                _seq,
                self.battery_v,
                battery_flags_raw,
                battery_percent_raw,
                self.np_throttle,
                self.np_pulse_us,
                np_flags_raw,
            ) = values
            battery_flags = int(battery_flags_raw)
            np_flags = int(np_flags_raw)
            self.battery_percent = int(battery_percent_raw)
            self.battery_low = 1 if (battery_flags & 0x01) else 0
            self.battery_critical = 1 if (battery_flags & 0x02) else 0
            self.np_enable = 1 if (np_flags & 0x01) else 0
            self.np_active = 1 if (np_flags & 0x02) else 0
            self.np_arming = 1 if (np_flags & 0x04) else 0
            self.np_running = 1 if (np_flags & 0x08) else 0
        elif frame_type == _FRAME_TYPE_PERF:
            (
                self.cpu0_load,
                self.cpu1_load,
                self.foc_us,
                self.isr1k_us,
                self.foc_pct,
                _window_ms,
            ) = values
        elif frame_type == _FRAME_TYPE_GRAY:
            gray_values = list(values[1:])
            if len(gray_values) == self.cols:
                self.sensors = gray_values
        elif frame_type == _FRAME_TYPE_GRAY_RAW:
            gray_values = list(values[1:])
            if len(gray_values) == self.cols:
                self.sensors = gray_values

        self.rx_total += 1


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="TC264 WiFi-UART Qt 高频遥测上位机")
    parser.add_argument("--host", default="0.0.0.0", help="监听地址（默认 0.0.0.0）")
    parser.add_argument("--port", type=int, default=9000, help="监听端口（默认 9000）")
    parser.add_argument("--fps", type=float, default=120.0, help="主循环目标 FPS（默认 120）")
    parser.add_argument("--cols", type=int, default=_DEFAULT_SENSOR_COLS, help=f"点阵列数/光电管路数（默认 {_DEFAULT_SENSOR_COLS}）")
    parser.add_argument("--idle-timeout", type=float, default=3.0, help="连接静默超时秒数（默认 3.0）")
    return parser


class QtPlotApp:
    def __init__(self, args: argparse.Namespace) -> None:
        from PySide6 import QtCore, QtGui, QtWidgets
        import pyqtgraph as pg
        
        self.QtCore = QtCore
        self.QtGui = QtGui
        self.QtWidgets = QtWidgets
        self.pg = pg
        pg.setConfigOptions(antialias=True)

        class CpuGaugeWidget(QtWidgets.QWidget):
            def __init__(self, title: str, color: str, parent=None):
                super().__init__(parent)
                self._title = title
                self._color = color
                self._value = 0.0
                self.setMinimumSize(160, 180)
                self.setMaximumSize(220, 220)

            def set_value(self, v: float):
                self._value = max(0.0, min(100.0, v))
                self.update()

            def paintEvent(self, event):
                p = QtGui.QPainter(self)
                p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
                w = self.width()
                h = self.height()
                cx = w / 2.0
                cy = h / 2.0 + 4
                r = min(w, h) / 2.0 - 8

                pen_bg = QtGui.QPen(QtGui.QColor("#e5e7eb"), 18.0, QtCore.Qt.PenStyle.SolidLine)
                pen_bg.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
                p.setPen(pen_bg)
                p.drawArc(QtCore.QRectF(cx - r, cy - r, r * 2, r * 2), 135 * 16, 270 * 16)

                pen_val = QtGui.QPen(QtGui.QColor(self._color), 18.0, QtCore.Qt.PenStyle.SolidLine)
                pen_val.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
                p.setPen(pen_val)
                span = int(270 * self._value / 100.0)
                p.drawArc(QtCore.QRectF(cx - r, cy - r, r * 2, r * 2), 135 * 16, -span * 16)

                p.setPen(QtGui.QColor("#1f2937"))
                font_val = QtGui.QFont("Segoe UI", 26, QtGui.QFont.Weight.Bold)
                p.setFont(font_val)
                p.drawText(QtCore.QRectF(0, cy - r + 30, w, 50), QtCore.Qt.AlignmentFlag.AlignCenter,
                           f"{self._value:.1f}%")

                font_title = QtGui.QFont("Segoe UI", 11)
                p.setPen(QtGui.QColor("#6b7280"))
                p.setFont(font_title)
                p.drawText(QtCore.QRectF(0, cy + r - 8, w, 22), QtCore.Qt.AlignmentFlag.AlignCenter, self._title)

                font_sub = QtGui.QFont("Segoe UI", 9)
                p.setFont(font_sub)
                p.drawText(QtCore.QRectF(0, cy + r + 10, w, 20), QtCore.Qt.AlignmentFlag.AlignCenter, "CPU 占用率")

                p.end()

        self.CpuGaugeWidget = CpuGaugeWidget

        self._init_theme()
        self.args = args
        self.ui_fps = max(1.0, float(args.fps))
        self.panel_interval_s = 1.0 / min(self.ui_fps, 60.0)
        self.log_flush_interval_s = 0.10
        self.speed_plot_window_s = 8.0
        self.speed_curve_smooth_alpha = 0.28
        self.speed_curve_smooth_min_points = 6
        self.model = TelemetryModel(int(args.cols))
        self.rate_snapshot = RateSnapshot()
        self.last_rate_sample_t = time.perf_counter()
        self.last_panel_refresh_t = 0.0
        self.last_log_flush_t = 0.0
        self.last_raw_rx_count = 0
        self.last_raw_text_count = 0
        self.last_ui_rx_count = 0
        self.last_ui_text_count = 0
        self.pending_logs: List[str] = []
        self.was_connected = False
        self.current_board_page = _BOARD_PAGE_MAIN
        self.suppress_tab_sync_event = False
        self.suppress_param_page_sync_event = False
        self.tabs = None
        self.tab_main_index = 0
        self.tab_foc_index = 1
        self.tab_param_indices = {}
        self.last_run_enable_state: Optional[int] = None
        self.run_timer_active = False
        self.run_timer_start_t = 0.0
        self.run_timer_elapsed_s = 0.0
        self.run_timer_last_s: Optional[float] = None
        self.run_timer_start_route_idx: Optional[int] = None
        self.run_timer_end_route_idx: Optional[int] = None
        self.last_route_index_seen: Optional[int] = None
        self.last_turn_hint: Optional[int] = None
        self.last_logged_stop_reason: int = 0
        self.last_junction_lock_cnt: int = 0
        self.last_element_enter_cnt: int = 0
        self.last_junction_enter_cnt: int = 0
        self.run_ctrl_pending_cmd: Optional[str] = None
        self.run_ctrl_pending_opcode: Optional[int] = None
        self.run_ctrl_pending_seq: Optional[int] = None
        self.run_ctrl_pending_frame: Optional[bytes] = None
        self.run_ctrl_pending_expect_run_enable: Optional[int] = None
        self.run_ctrl_pending_since_t = 0.0
        self.run_ctrl_pending_last_tx_t = 0.0
        self.run_ctrl_pending_retries = 0
        self.run_ctrl_next_seq = 1
        self.gray_stream_enabled = False
        self.gray_raw_enabled = False
        self.att_zero_roll_deg: Optional[float] = None
        self.att_zero_pitch_deg: Optional[float] = None
        self.att_zero_yaw_deg: Optional[float] = None
        self.speed_time_hist: deque[float] = deque()
        self.speed_lt_hist: deque[float] = deque()
        self.speed_rt_hist: deque[float] = deque()
        self.speed_la_hist: deque[float] = deque()
        self.speed_ra_hist: deque[float] = deque()
        self.speed_lvq_hist: deque[float] = deque()
        self.speed_rvq_hist: deque[float] = deque()
        self.error_hist: deque[float] = deque()
        self.elec_time_hist: deque[float] = deque()
        self.elec_l_hist: deque[float] = deque()
        self.elec_r_hist: deque[float] = deque()
        self.cols = self.model.cols
        self.waterfall_hist_len = max(1, int(round(self.speed_plot_window_s * self.ui_fps)))
        self.line_pos_hist: deque[float] = deque()
        self.waterfall_data = np.zeros((self.cols, self.waterfall_hist_len), dtype=np.uint8)
        self.waterfall_lut = self._build_waterfall_lut()

        self.stop_evt = threading.Event()
        self.state = ConnectionState()
        self.text_q: "queue.Queue[RxEvent]" = queue.Queue(maxsize=512)
        self.data_q: "queue.Queue[RxEvent]" = queue.Queue(maxsize=4096)
        self.log_q: "queue.Queue[str]" = queue.Queue(maxsize=512)
        self.timer_resolution = WindowsTimerResolution()
        self.timer_resolution.enable()

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.server.bind((args.host, args.port))
        except OSError as exc:
            winerror = getattr(exc, "winerror", None)
            if winerror == 10013:
                msg = (
                    f"无法绑定 {args.host}:{args.port}（WinError 10013）。"
                    "该端口可能被 Windows 系统保留，请改用 --port 9000（或其它非保留端口），"
                    "并确保下位机目标端口保持一致。"
                )
            elif winerror == 10048:
                msg = (
                    f"无法绑定 {args.host}:{args.port}（WinError 10048，端口已被占用）。"
                    "请关闭占用进程或改用 --port 9000。"
                )
            else:
                msg = f"无法绑定 {args.host}:{args.port}: {exc}"
            try:
                self.server.close()
            except OSError:
                pass
            raise RuntimeError(msg) from exc
        self.server.listen(1)

        self.accept_thread = threading.Thread(
            target=accept_loop,
            args=(self.server, self.state, self.text_q, self.data_q, self.log_q, self.stop_evt, int(args.cols)),
            daemon=True,
        )
        self.accept_thread.start()

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.app.setApplicationName("TC264 智能车调试上位机")

        self.window = QtWidgets.QMainWindow()
        self.window.setWindowTitle("TC264 智能车调试上位机")
        self.window.resize(1120, 760)
        self.window.closeEvent = self._on_close_event
        self._build_ui()
        self._center_window()

        self.loop_timer = QtCore.QTimer()
        self.loop_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.loop_timer.timeout.connect(self.loop)
        self.loop_timer.start(max(1, int(1000.0 / self.ui_fps)))

        push_log(self.log_q, f"服务器已启动 {args.host}:{args.port}")
        push_log(
            self.log_q,
            (
                "二进制帧已切换为分帧模式: "
                f"FAST={_FAST_FRAME_PAYLOAD_LEN}B ROUTE={_ROUTE_FRAME_PAYLOAD_LEN}B "
                f"POWER={_POWER_FRAME_PAYLOAD_LEN}B GRAY=1+{self.cols}B, CTRL ACK超时={int(_RUN_CTRL_ACK_TIMEOUT_S * 1000)}ms"
            ),
        )

    def _center_window(self) -> None:
        screen = self.app.primaryScreen()
        if screen is None:
            return
        area = screen.availableGeometry()
        frame = self.window.frameGeometry()
        frame.moveCenter(area.center())
        self.window.move(frame.topLeft())

    def _init_theme(self) -> None:
        self.theme = {
            "bg_canvas": "#f2f5f9",
            "bg_panel": "#ffffff",
            "bg_panel_soft": "#e9eef5",
            "text_primary": "#1b2532",
            "text_muted": "#5b6676",
            "border": "#d5dde8",
            "accent": "#0f766e",
            "accent_soft": "#dff5f1",
            "plot_bg": "#ffffff",
            "plot_grid": "#a4b1c2",
            "plot_axis": "#5f6f83",
            "plot_border": "#cfd8e4",
            "plot_legend_bg": "#f8fbff",
            "plot_legend_border": "#d3dceb",
            "plot_zero": "#92a2b8",
            "plot_dark_bg": "#ffffff",
            "plot_dark_grid": "#cfd8e4",
            "plot_dark_axis": "#5f6f83",
            "waterfall_bg": "#ffffff",
            "waterfall_axis": "#5f6f83",
            "speed_lt": "#f59e0b",
            "speed_la": "#0891b2",
            "speed_rt": "#ef4444",
            "speed_ra": "#2563eb",
            "error_line": "#fbbf24",
            "waterfall_error": "#1f2a37",
            "font_main": "'Microsoft YaHei UI'",
            "font_ui": "'Microsoft YaHei UI'",
            "font_data": "'Consolas'",
        }

    @staticmethod
    def _build_waterfall_lut() -> np.ndarray:
        levels = np.array([0.0, 48.0, 104.0, 168.0, 224.0, 255.0], dtype=float)
        colors = np.array(
            [
                [247.0, 250.0, 255.0],
                [219.0, 233.0, 247.0],
                [173.0, 213.0, 235.0],
                [125.0, 197.0, 205.0],
                [250.0, 214.0, 140.0],
                [223.0, 124.0, 77.0],
            ],
            dtype=float,
        )
        x = np.arange(256, dtype=float)
        lut = np.zeros((256, 3), dtype=np.uint8)
        for channel in range(3):
            lut[:, channel] = np.interp(x, levels, colors[:, channel]).astype(np.uint8)
        return lut

    @staticmethod
    def _wrap_deg_180(deg: float) -> float:
        while deg > 180.0:
            deg -= 360.0
        while deg < -180.0:
            deg += 360.0
        return deg

    def get_display_attitude(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        roll = self.model.roll
        pitch = self.model.pitch
        yaw = self.model.yaw

        if (roll is None) or (pitch is None) or (yaw is None):
            return roll, pitch, yaw

        if self.att_zero_roll_deg is not None:
            roll = self._wrap_deg_180(roll - self.att_zero_roll_deg)
        if self.att_zero_pitch_deg is not None:
            pitch = self._wrap_deg_180(pitch - self.att_zero_pitch_deg)
        if self.att_zero_yaw_deg is not None:
            yaw = self._wrap_deg_180(yaw - self.att_zero_yaw_deg)

        return roll, pitch, yaw

    def reset_attitude_zero(self) -> None:
        roll = self.model.roll
        pitch = self.model.pitch
        yaw = self.model.yaw

        if (roll is None) or (pitch is None) or (yaw is None):
            push_log(self.log_q, "姿态重置失败: 当前无有效姿态数据")
            return

        self.att_zero_roll_deg = float(roll)
        self.att_zero_pitch_deg = float(pitch)
        self.att_zero_yaw_deg = float(yaw)
        push_log(self.log_q, f"姿态显示零点已重置: roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f}")

    @staticmethod
    def normalize_param_key(key: str) -> str:
        return key.strip().replace(" ", "_").upper()

    @staticmethod
    def normalize_page_name(page_name: str) -> str:
        text = (page_name or "").strip().replace(" ", "_").upper()
        if not text:
            return _BOARD_PAGE_MAIN
        alias = {
            "FOC_TEST": _BOARD_PAGE_FOC,
            "FOC_SPEED_DEBUG": _BOARD_PAGE_FOC,
            "LINE_FOLLOW": _BOARD_PAGE_LINE,
            "LINE_FOLLOW_DEBUG": _BOARD_PAGE_LINE,
            "NEG_PRESSURE": _BOARD_PAGE_NEG,
            "NEG_PRESSURE_DEBUG": _BOARD_PAGE_NEG,
            "BLDC_OPEN_LOOP": _BOARD_PAGE_OPENLOOP,
            "OPEN_LOOP": _BOARD_PAGE_OPENLOOP,
        }
        return alias.get(text, text)

    def next_control_seq(self) -> int:
        seq = self.run_ctrl_next_seq & 0xFF
        self.run_ctrl_next_seq = (seq + 1) & 0xFF
        return seq

    def set_gray_stream_enabled(self, enable: bool, force: bool = False) -> bool:
        desired = bool(enable)
        if (not force) and (self.gray_stream_enabled == desired):
            return True

        ok = send_command(self.state, self.log_q, f"GRAY:{1 if desired else 0}")
        if ok:
            self.gray_stream_enabled = desired
        return ok

    def set_gray_raw_enabled(self, enable: bool, force: bool = False) -> bool:
        desired = bool(enable)
        if (not force) and (self.gray_raw_enabled == desired):
            return True

        ok = send_command(self.state, self.log_q, f"GRAY_RAW:{1 if desired else 0}")
        if ok:
            self.gray_raw_enabled = desired
        return ok

    def get_target_page_for_tab(self, tab_index: int) -> str:
        if tab_index == getattr(self, "tab_main_index", -1):
            return _BOARD_PAGE_MAIN
        if tab_index == getattr(self, "tab_foc_index", -1):
            return _BOARD_PAGE_FOC
        if hasattr(self, "tab_param_indices"):
            for page_id, idx in self.tab_param_indices.items():
                if idx == tab_index:
                    return page_id
        return _BOARD_PAGE_MAIN

    def query_page_params(self, page_name: Optional[str] = None) -> None:
        page_norm = self.normalize_page_name(page_name or self.current_board_page)
        if not send_command(self.state, self.log_q, f"PAGEPARAM:{page_norm}"):
            if page_norm == _BOARD_PAGE_FOC:
                # Fallback for older firmware that has not implemented PAGEPARAM yet.
                run_key, left_key, right_key = self.get_foc_test_param_keys()
                self.query_named_param(run_key)
                self.query_named_param(left_key)
                self.query_named_param(right_key)

    def request_page_sync(self, page_name: str, query_params: bool = True) -> None:
        page_norm = self.normalize_page_name(page_name)
        with self.state.lock:
            connected = self.state.conn is not None
        self.current_board_page = page_norm
        if not connected:
            return

        send_command(self.state, self.log_q, f"PAGE:{page_norm}")
        self.set_gray_stream_enabled(page_norm in (_BOARD_PAGE_GRAY, _BOARD_PAGE_LINE))
        self.set_gray_raw_enabled(page_norm == _BOARD_PAGE_GRAY)
        if query_params:
            self.query_page_params(page_norm)

    def get_current_ui_target_page(self) -> str:
        if self.tabs is None:
            return self.current_board_page

        tab_index = self.tabs.currentIndex()
        if tab_index < 0:
            return self.current_board_page

        return self.get_target_page_for_tab(tab_index)

    def sync_current_tab_to_board(self, query_params: bool = True) -> None:
        page_norm = self.get_current_ui_target_page()
        self.request_page_sync(page_norm, query_params=(query_params and page_norm != _BOARD_PAGE_MAIN))

    def apply_board_page_to_ui(self, page_name: str) -> None:
        if self.tabs is None:
            return

        page_norm = self.normalize_page_name(page_name)
        self.current_board_page = page_norm
        self.set_gray_stream_enabled(page_norm in (_BOARD_PAGE_GRAY, _BOARD_PAGE_LINE))
        self.set_gray_raw_enabled(page_norm == _BOARD_PAGE_GRAY)

        target_tab = None
        current_index = self.tabs.currentIndex()
        if hasattr(self, "tab_param_indices") and page_norm in self.tab_param_indices:
            if current_index == self.tab_param_indices[page_norm]:
                target_tab = current_index
            else:
                target_tab = self.tab_param_indices[page_norm]

        if target_tab is None:
            if page_norm == _BOARD_PAGE_MAIN:
                target_tab = getattr(self, "tab_main_index", current_index)
            elif page_norm == _BOARD_PAGE_FOC:
                target_tab = getattr(self, "tab_foc_index", current_index)
            else:
                target_tab = current_index

        self.suppress_tab_sync_event = True
        try:
            if self.tabs.currentIndex() != target_tab:
                self.tabs.setCurrentIndex(target_tab)
        finally:
            self.suppress_tab_sync_event = False

    def on_tab_changed(self, tab_index: int) -> None:
        if self.suppress_tab_sync_event:
            return
        page_norm = self.get_target_page_for_tab(tab_index)
        self.request_page_sync(page_norm, query_params=(page_norm != _BOARD_PAGE_MAIN))

    def on_tab_clicked(self, tab_index: int) -> None:
        if self.suppress_tab_sync_event:
            return
        page_norm = self.get_target_page_for_tab(tab_index)
        self.request_page_sync(page_norm, query_params=(page_norm != _BOARD_PAGE_MAIN))

    def navigate_to_page(self, page_name: str) -> None:
        if self.tabs is None:
            return

        page_norm = self.normalize_page_name(page_name)
        target_tab = None
        if page_norm == _BOARD_PAGE_MAIN:
            target_tab = getattr(self, "tab_main_index", -1)
        elif page_norm == _BOARD_PAGE_FOC:
            target_tab = getattr(self, "tab_foc_index", -1)
        elif hasattr(self, "tab_param_indices"):
            target_tab = self.tab_param_indices.get(page_norm)

        if (target_tab is None) or (target_tab < 0):
            return

        if self.tabs.currentIndex() != target_tab:
            self.tabs.setCurrentIndex(target_tab)
        else:
            self.request_page_sync(page_norm, query_params=(page_norm != _BOARD_PAGE_MAIN))

    def get_foc_test_param_keys(self) -> Tuple[str, str, str]:
        run_key = _FOC_TEST_RUN_KEY_DEFAULT
        left_key = _FOC_TEST_LEFT_KEY_DEFAULT
        right_key = _FOC_TEST_RIGHT_KEY_DEFAULT

        if hasattr(self, "foc_test_run_key_edit"):
            run_key = self.normalize_param_key(self.foc_test_run_key_edit.text()) or _FOC_TEST_RUN_KEY_DEFAULT
        if hasattr(self, "foc_test_left_key_edit"):
            left_key = self.normalize_param_key(self.foc_test_left_key_edit.text()) or _FOC_TEST_LEFT_KEY_DEFAULT
        if hasattr(self, "foc_test_right_key_edit"):
            right_key = self.normalize_param_key(self.foc_test_right_key_edit.text()) or _FOC_TEST_RIGHT_KEY_DEFAULT

        return run_key, left_key, right_key

    def send_named_param(self, key: str, value: float, decimals: int = _PARAM_DECIMALS) -> bool:
        norm_key = self.normalize_param_key(key)
        if not norm_key:
            self.log_q.put("[参数] 发送失败: 参数键名为空")
            return False

        if decimals <= 0:
            cmd = f"{norm_key}:{int(round(value))}"
        else:
            cmd = f"{norm_key}:{value:.{decimals}f}"

        ok = send_command(self.state, self.log_q, cmd)
        if ok:
            if not hasattr(self, "param_last_known"):
                self.param_last_known = {}
            self.param_last_known[norm_key] = float(value)
        return ok

    def query_named_param(self, key: str) -> bool:
        norm_key = self.normalize_param_key(key)
        if not norm_key:
            self.log_q.put("[参数] 查询失败: 参数键名为空")
            return False
        return send_command(self.state, self.log_q, f"{norm_key}?")

    def send_param_debounced(self, key: str, sb) -> None:
        now = time.perf_counter()
        if now - getattr(self, "last_param_set_t", 0.0) < 0.5:
            self.log_q.put("[防刷] 操作过于频繁，请等待...")
            return
        self.last_param_set_t = now

        norm_key = self.normalize_param_key(key)
        new_val = sb.value()
        if not hasattr(self, "param_last_known"):
            self.param_last_known = {}
        old_val = self.param_last_known.get(norm_key)
        name = getattr(self, "param_name_map", {}).get(norm_key, norm_key)

        if old_val is not None:
            if abs(old_val - new_val) > 1e-6:
                self.log_q.put(f"[参数] 申请将 {name}({norm_key}) 从 {old_val:.{_PARAM_DECIMALS}f} 修改为 {new_val:.{_PARAM_DECIMALS}f}")
            else:
                self.log_q.put(f"[参数] 申请写入 {name}({norm_key}) 为 {new_val:.{_PARAM_DECIMALS}f} (未改变)")
        else:
            self.log_q.put(f"[参数] 申请将 {name}({norm_key}) 从 未知 修改为 {new_val:.{_PARAM_DECIMALS}f}")

        self.param_last_known[norm_key] = new_val
        send_command(self.state, self.log_q, f"{norm_key}:{new_val:.{_PARAM_DECIMALS}f}")
        send_command(self.state, self.log_q, "SAVE")

    def send_save_debounced(self) -> None:
        now = time.perf_counter()
        if now - getattr(self, "last_param_set_t", 0.0) < 0.5:
            self.log_q.put("[防刷] 操作过于频繁，请等待...")
            return
        self.last_param_set_t = now
        send_command(self.state, self.log_q, "SAVE")

    def reset_run_control_state(self) -> None:
        self.run_ctrl_pending_cmd = None
        self.run_ctrl_pending_opcode = None
        self.run_ctrl_pending_seq = None
        self.run_ctrl_pending_frame = None
        self.run_ctrl_pending_expect_run_enable = None
        self.run_ctrl_pending_since_t = 0.0
        self.run_ctrl_pending_last_tx_t = 0.0
        self.run_ctrl_pending_retries = 0
        self.update_run_control_buttons()

    def request_runtime_resync(self) -> None:
        send_command(self.state, self.log_q, "PAGE?")

    def finish_run_control_pending(self, ok: bool, message: str) -> None:
        self.reset_run_control_state()
        if message:
            push_log(self.log_q, message)

    def begin_run_control_transaction(self, command: str, expect_run_enable: Optional[int]) -> bool:
        command_norm = self.normalize_param_key(command)
        opcode = _CTRL_NAME_TO_OPCODE.get(command_norm)
        if opcode is None:
            push_log(self.log_q, f"[运行] 未知控制命令: {command_norm}")
            return False
        if self.run_ctrl_pending_cmd is not None:
            push_log(self.log_q, f"[运行] 命令已挂起: 等待 {self.run_ctrl_pending_cmd} 确认")
            return False

        seq = self.next_control_seq()
        frame = build_control_frame(seq, opcode)
        if not send_raw_frame(self.state, self.log_q, frame, f"CTRL {command_norm} seq={seq}"):
            return False

        self.run_ctrl_pending_cmd = command_norm
        self.run_ctrl_pending_opcode = opcode
        self.run_ctrl_pending_seq = seq
        self.run_ctrl_pending_frame = frame
        self.run_ctrl_pending_expect_run_enable = None if expect_run_enable is None else int(expect_run_enable)
        now = time.perf_counter()
        self.run_ctrl_pending_since_t = now
        self.run_ctrl_pending_last_tx_t = now
        self.run_ctrl_pending_retries = 0
        self.update_run_control_buttons()
        return True

    def handle_control_ack(self, seq: int, opcode: int, status: int) -> bool:
        opcode_name = control_opcode_to_name(opcode)
        status_text = control_status_to_text(status)

        if (self.run_ctrl_pending_seq == seq) and (self.run_ctrl_pending_opcode == opcode):
            if status == _CTRL_STATUS_OK:
                self.finish_run_control_pending(True, f"[运行] {opcode_name} ACK seq={seq} 状态={status_text}")
            else:
                self.finish_run_control_pending(False, f"[运行] {opcode_name} 失败 seq={seq} 状态={status_text}")
                self.request_runtime_resync()
            return True

        if status != _CTRL_STATUS_OK:
            push_log(self.log_q, f"[运行] 收到非当前控制ACK: {opcode_name} seq={seq} 状态={status_text}")
        return False

    def update_run_control_from_telemetry(self) -> None:
        if self.run_ctrl_pending_cmd is None:
            return
        if self.run_ctrl_pending_expect_run_enable is None:
            return

        run_now = self.model.run_enable
        if run_now is None:
            return

        if int(run_now) == int(self.run_ctrl_pending_expect_run_enable):
            self.finish_run_control_pending(True, f"[运行] {self.run_ctrl_pending_cmd} 已由遥测二次确认")

    def check_run_control_timeout(self, now: float) -> None:
        if self.run_ctrl_pending_cmd is None:
            return

        if (now - self.run_ctrl_pending_last_tx_t) < _RUN_CTRL_ACK_TIMEOUT_S:
            return

        pending_cmd = self.run_ctrl_pending_cmd
        pending_seq = self.run_ctrl_pending_seq
        pending_frame = self.run_ctrl_pending_frame
        if (self.run_ctrl_pending_retries < _RUN_CTRL_MAX_RETRIES) and (pending_frame is not None) and (pending_seq is not None):
            retry_index = self.run_ctrl_pending_retries + 1
            if send_raw_frame(
                self.state,
                self.log_q,
                pending_frame,
                f"CTRL {pending_cmd} seq={pending_seq} RETRY {retry_index}/{_RUN_CTRL_MAX_RETRIES}",
            ):
                self.run_ctrl_pending_retries = retry_index
                self.run_ctrl_pending_last_tx_t = now
                return

            self.reset_run_control_state()
            return

        self.reset_run_control_state()
        push_log(self.log_q, f"[运行] {pending_cmd} ACK超时，已触发状态重同步")
        self.request_runtime_resync()

    def update_run_control_buttons(self) -> None:
        pending = self.run_ctrl_pending_cmd is not None
        main_running = (self.model.run_enable == 1)
        foc_running = False

        if hasattr(self, "foc_test_run_sb"):
            foc_running = int(round(self.foc_test_run_sb.value())) != 0

        if hasattr(self, "btn_foc_start"):
            self.btn_foc_start.setEnabled((not pending) and (not foc_running) and (not main_running))
        if hasattr(self, "btn_foc_pause"):
            self.btn_foc_pause.setEnabled((not pending) and (main_running or foc_running))
        if hasattr(self, "btn_run_start"):
            self.btn_run_start.setEnabled((not pending) and (not foc_running) and (not main_running))
        if hasattr(self, "btn_run_pause"):
            self.btn_run_pause.setEnabled((not pending) and (main_running or foc_running))
        if hasattr(self, "btn_odom_start"):
            self.btn_odom_start.setEnabled(not pending)
        if hasattr(self, "btn_odom_clear"):
            self.btn_odom_clear.setEnabled(not pending)
        if hasattr(self, "btn_np_on"):
            self.btn_np_on.setEnabled((not pending) and (self.model.np_enable != 1))
        if hasattr(self, "btn_np_off"):
            self.btn_np_off.setEnabled((not pending) and (self.model.np_enable != 0))

    def send_foc_test_run(self, run_value: int) -> None:
        run_key, _, _ = self.get_foc_test_param_keys()
        run_value_i = 1 if int(run_value) != 0 else 0
        self.send_named_param(run_key, float(run_value_i), decimals=0)
        if hasattr(self, "foc_test_run_sb"):
            self.foc_test_run_sb.setValue(float(run_value_i))

    def set_local_param_value(self, key: str, value: float) -> None:
        norm_key = self.normalize_param_key(key)
        if not norm_key:
            return

        if hasattr(self, "param_widgets") and norm_key in self.param_widgets:
            self.param_widgets[norm_key].setValue(float(value))

        if not hasattr(self, "param_last_known"):
            self.param_last_known = {}
        self.param_last_known[norm_key] = float(value)

    def force_local_pause_state(self) -> None:
        run_key, _, _ = self.get_foc_test_param_keys()

        self.set_local_param_value(run_key, 0.0)
        self.set_local_param_value("NP_EN", 0.0)
        self.set_local_param_value("OL_RUN", 0.0)
        self.reset_run_control_state()

        if hasattr(self, "foc_test_run_sb"):
            self.foc_test_run_sb.setValue(0.0)
        self.update_run_control_buttons()

        if self.run_timer_active:
            self.run_timer_last_s = self.run_timer_elapsed_s
            self.run_timer_end_route_idx = self.model.route_index
        self.run_timer_active = False

        self.model.run_enable = 0
        self.model.np_enable = 0
        self.model.np_active = 0
        self.model.np_arming = 0
        self.model.np_running = 0
        self.model.speed_l_target = 0.0
        self.model.speed_r_target = 0.0
        self.model.stop_reason = 0

        self.last_route_index_seen = None
        self.last_turn_hint = None
        self.last_logged_stop_reason = 0
        self.last_junction_lock_cnt = 0
        self.last_element_enter_cnt = 0
        self.last_junction_enter_cnt = 0
    def send_pause_all(self) -> None:
        if self.begin_run_control_transaction("PAUSE", 0):
            self.refresh_panels()

    def send_foc_pause_all(self) -> None:
        self.send_pause_all()

    def send_main_start_safe(self) -> None:
        foc_test_run = 0
        if hasattr(self, "foc_test_run_sb"):
            foc_test_run = int(round(self.foc_test_run_sb.value()))

        if foc_test_run != 0:
            self.log_q.put("[安全] FOC测试运行中，已禁止启动主流程；请先停机(TST_RUN:0/PAUSE)。")
            return

        if self.begin_run_control_transaction("START", 1):
            self.refresh_panels()

    def send_odom_start(self) -> None:
        self.begin_run_control_transaction("ODOM_START", None)

    def send_odom_clear(self) -> None:
        self.begin_run_control_transaction("ODOM_CLEAR", None)

    def send_np_on(self) -> None:
        self.begin_run_control_transaction("NPON", None)

    def send_np_off(self) -> None:
        self.begin_run_control_transaction("NPOFF", None)

    def send_foc_test_rpm_targets(self) -> None:
        _, left_key, right_key = self.get_foc_test_param_keys()
        left_val = float(self.foc_test_left_rpm_sb.value()) if hasattr(self, "foc_test_left_rpm_sb") else 0.0
        right_val = float(self.foc_test_right_rpm_sb.value()) if hasattr(self, "foc_test_right_rpm_sb") else 0.0
        self.send_named_param(left_key, left_val, decimals=0)
        self.send_named_param(right_key, right_val, decimals=0)

    def send_foc_test_all(self) -> None:
        run_value = int(round(self.foc_test_run_sb.value())) if hasattr(self, "foc_test_run_sb") else 0
        self.send_foc_test_run(run_value)
        self.send_foc_test_rpm_targets()

    def query_foc_test_params(self) -> None:
        self.query_page_params(_BOARD_PAGE_FOC)

    def set_foc_test_rpm_preset(self, left_rpm: float, right_rpm: float) -> None:
        if hasattr(self, "foc_test_left_rpm_sb"):
            self.foc_test_left_rpm_sb.setValue(float(left_rpm))
        if hasattr(self, "foc_test_right_rpm_sb"):
            self.foc_test_right_rpm_sb.setValue(float(right_rpm))

    def format_observed_diff_wheel_text(self) -> str:
        base_speed = None
        if hasattr(self, "param_widgets") and ("BAS_SPD" in self.param_widgets):
            try:
                base_speed = float(self.param_widgets["BAS_SPD"].value())
            except Exception:
                base_speed = None
        if (base_speed is None) and hasattr(self, "param_last_known"):
            base_speed = self.param_last_known.get("BAS_SPD")

        delta_rpm = self.model.delta_rpm
        if (base_speed is None) or (delta_rpm is None):
            return "观测轮速(基于BAS_SPD+差速)  左=--   右=--"

        diff_left_rpm = base_speed - delta_rpm
        diff_right_rpm = base_speed + delta_rpm
        return f"观测轮速(基于BAS_SPD+差速)  左={fmt_float(diff_left_rpm)}   右={fmt_float(diff_right_rpm)}"

    def refresh_foc_test_panel(self) -> None:
        if not hasattr(self, "foc_test_state_label"):
            return

        main_running = (self.model.run_enable == 1)
        main_text = "运行中" if main_running else "已停止"

        foc_test_run = None
        if hasattr(self, "foc_test_run_sb"):
            foc_test_run = int(round(self.foc_test_run_sb.value()))
        foc_running = (foc_test_run == 1)

        run_color = "#166534" if (main_running or foc_running) else "#9a3412"
        pending_text = ""
        if self.run_ctrl_pending_cmd is not None:
            pending_text = f" | 控制命令=等待{self.run_ctrl_pending_cmd}确认"
        self.foc_test_state_label.setStyleSheet(
            f"background:#f8fafc;color:{run_color};border:1px solid #dbe3ee;border-radius:6px;padding:6px;font:700 12px 'Consolas';"
        )
        self.foc_test_state_label.setText(
            f"主流程: {main_text} | FOC测试: {('运行中' if foc_running else '已停止')} | 线控就绪={self.model.line_ready if self.model.line_ready is not None else '--'} | 停机={stop_reason_to_text(self.model.stop_reason)}{pending_text}"
        )
        self.update_run_control_buttons()

        self.foc_test_target_label.setText(
            f"目标RPM  左={fmt_float(self.model.speed_l_target)}   右={fmt_float(self.model.speed_r_target)}"
        )
        self.foc_test_actual_label.setText(
            f"实际RPM  左={fmt_float(self.model.speed_l_actual)}   右={fmt_float(self.model.speed_r_actual)}"
        )
        self.foc_test_elec_label.setText(
            f"电角度   左={fmt_float(self.model.elec_l_deg)}   右={fmt_float(self.model.elec_r_deg)}"
        )

    def save_local_profile(self) -> None:
        file_path, _ = self.QtWidgets.QFileDialog.getSaveFileName(
            self.window, "保存本地配置", "", "JSON Files (*.json)"
        )
        if file_path:
            try:
                data = {key: w.value() for key, w in self.param_widgets.items()}
                with open(file_path, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=4, ensure_ascii=False)
                self.log_q.put("[UI] 本地配置已保存至: " + file_path)
            except Exception as e:
                self.log_q.put(f"[UI] 保存本地配置失败: {e}")

    def load_local_profile(self) -> None:
        file_path, _ = self.QtWidgets.QFileDialog.getOpenFileName(
            self.window, "加载本地配置", "", "JSON Files (*.json)"
        )
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                # To prevent blocking the UI thread and flooding the network, 
                # we can use a QTimer or separate thread. Since it's a small delay,
                # creating a background thread is safer for time.sleep
                def _apply():
                    count = 0
                    for key, val in data.items():
                        if key in self.param_widgets:
                            # Update UI safely? No, better updating UI in main thread.
                            # So update UI immediately, but send command in loop is fine if it's main thread?
                            # Wait, send_command is thread-safe (just queueing/socket), updating UI is main thread.
                            # So we shouldn't sleep in main thread.
                            pass

                count = 0
                for key, val in data.items():
                    if key in self.param_widgets:
                        self.param_widgets[key].setValue(float(val))
                        # Use QTimer to delay send_command to avoid flooding
                        # Or rely on TCP coalescing which is fine, send_command just writes to socket
                        send_command(self.state, self.log_q, f"{key}:{val:.{_PARAM_DECIMALS}f}")
                        count += 1
                self.log_q.put(f"[UI] 本地配置已加载，共发出 {count} 个参数更新指令")
            except Exception as e:
                self.log_q.put(f"[UI] 加载本地配置失败: {e}")

    def _build_ui(self) -> None:
        QtCore = self.QtCore
        QtGui = self.QtGui
        QtWidgets = self.QtWidgets
        pg = self.pg
        theme = self.theme

        self.window.setStyleSheet(
            """
            QMainWindow {{
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:1,
                    stop:0 {bg_canvas},
                    stop:1 {bg_panel_soft}
                );
            }}
            QWidget {{
                color: {text_primary};
                background: {bg_panel_soft};
                font-family: {font_main};
            }}
            QSplitter {{
                background: {bg_panel_soft};
            }}
            QSplitter::handle {{
                background: {border};
            }}
            QLabel {{
                background: transparent;
            }}
            QPushButton {{
                background: {bg_panel};
                color: {text_primary};
                border: 1px solid {border};
                border-radius: 6px;
                padding: 6px 12px;
                font: 600 12px {font_ui};
            }}
            QPushButton:hover {{
                background: {accent_soft};
                border-color: {accent};
                color: {text_primary};
            }}
            QPushButton:pressed {{
                background: #c5e2ff;
                border-color: #0a6ca4;
            }}
            QPushButton:disabled {{
                background: #e8ecf0;
                color: #94a3b8;
                border-color: #d0d6de;
            }}
            QLineEdit, QDoubleSpinBox {{
                background: {bg_panel};
                color: {text_primary};
                border: 1px solid {border};
                border-radius: 5px;
                padding: 5px 8px;
                selection-background-color: {accent_soft};
            }}
            QDoubleSpinBox:hover {{
                border-color: {accent};
            }}
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
                background: transparent;
                border-left: 1px solid {border};
                width: 18px;
            }}
            QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {{
                background: {accent_soft};
            }}
            QGroupBox {{
                background: {bg_panel};
                border: 1px solid {border};
                border-radius: 8px;
                margin-top: 14px;
                padding-top: 14px;
                font: 600 12px {font_ui};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 6px;
                color: {accent};
                background: transparent;
            }}
            QPlainTextEdit {{
                background: {bg_panel};
                color: {text_primary};
                border: 1px solid {border};
                border-radius: 8px;
                selection-background-color: {accent_soft};
                padding: 6px;
            }}
            QTabWidget::pane {{
                border: 1px solid {border};
                border-radius: 6px;
                background: {bg_panel};
            }}
            QTabBar::tab {{
                background: #eaf1f9;
                color: {text_muted};
                padding: 9px 16px;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
                margin-right: 4px;
                font: 600 12px {font_ui};
                border: 1px solid transparent;
            }}
            QTabBar::tab:selected {{
                background: {bg_panel};
                color: {accent};
                border: 1px solid {border};
                border-bottom: 2px solid {accent};
            }}
            QTabBar::tab:hover:!selected {{
                background: {accent_soft};
                color: {text_primary};
            }}
            QScrollBar:vertical {{
                background: transparent;
                width: 10px;
                margin: 0px;
            }}
            QScrollBar::handle:vertical {{
                background: {border};
                min-height: 20px;
                border-radius: 5px;
            }}
            QScrollBar::handle:vertical:hover {{
                background: #9eb2c8;
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
                height: 0px;
            }}
            """.format_map(theme)
        )

        central = QtWidgets.QWidget()
        self.window.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        left_column = QtWidgets.QWidget()
        left_column.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        left_column_layout = QtWidgets.QVBoxLayout(left_column)
        left_column_layout.setContentsMargins(0, 0, 0, 0)
        left_column_layout.setSpacing(8)

        right_column = QtWidgets.QWidget()
        right_column.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        right_column_layout = QtWidgets.QVBoxLayout(right_column)
        right_column_layout.setContentsMargins(0, 0, 0, 0)
        right_column_layout.setSpacing(8)

        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        main_splitter.setChildrenCollapsible(True)
        main_splitter.addWidget(left_column)
        main_splitter.addWidget(right_column)
        main_splitter.setStretchFactor(0, 2)
        main_splitter.setStretchFactor(1, 8)
        main_splitter.setSizes([300, 1020])
        root.addWidget(main_splitter)

        left_panel = QtWidgets.QWidget()
        left_panel.setStyleSheet(f"background:{theme['bg_panel']};border:1px solid {theme['border']};border-radius:12px;")
        left_panel.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        left_layout.setContentsMargins(10, 10, 10, 10)
        left_layout.setSpacing(8)
        left_column_layout.addWidget(left_panel, 0)

        self.alarm_label = QtWidgets.QLabel("系统正常")
        self.alarm_label.setStyleSheet(f"background:#e8f6ef;color:#156f4b;padding:8px;font:600 13px {theme['font_ui']};border-radius:6px;")
        left_layout.addWidget(self.alarm_label)

        self.status_label = QtWidgets.QLabel(f"监听中 {self.args.host}:{self.args.port}，等待设备连接")
        self.status_label.setStyleSheet(f"color:{theme['text_muted']};font:11px {theme['font_ui']};")
        self.status_label.setWordWrap(True)
        left_layout.addWidget(self.status_label)

        battery_row = QtWidgets.QHBoxLayout()
        battery_row.setContentsMargins(2, 0, 2, 0)
        battery_row.setSpacing(10)

        battery_caption = QtWidgets.QLabel("电量")
        battery_caption.setStyleSheet(f"color:{theme['text_muted']};font:11px {theme['font_ui']};")
        self.battery_icon_label = QtWidgets.QLabel()
        self.battery_icon_label.setFixedSize(64, 30)
        self.battery_text_label = QtWidgets.QLabel("--%  --V")
        self.battery_text_label.setStyleSheet(f"color:{theme['text_primary']};font:700 14px {theme['font_data']};")

        battery_row.addWidget(battery_caption)
        battery_row.addWidget(self.battery_icon_label)
        battery_row.addWidget(self.battery_text_label)
        battery_row.addStretch(1)
        left_layout.addLayout(battery_row)

        self.info_grid = QtWidgets.QGridLayout()
        self.info_grid.setHorizontalSpacing(8)
        self.info_grid.setVerticalSpacing(8)
        left_layout.addLayout(self.info_grid)

        self.value_labels = {}
        rows = [
            ("angle", "姿态角", "横滚=-- 俯仰=-- 航向=--"),
            ("gyro", "陀螺仪", "gx=-- gy=-- gz=--"),
            ("speed", "速度", "左目标=-- 右目标=-- 左实际=-- 右实际=--"),
            ("elec", "电角度", "左电角=-- 右电角=--"),
            ("np", "负压控制", "使能=-- 状态=-- 工作=-- 油门=-- 脉宽=--"),
            ("line", "循迹控制", "位置=-- 差速=-- bits=--\n观测轮速 左=-- 右=-- 运行=-- 就绪=--"),
            ("text", "最近文本", "--"),
        ]
        for row_index, (key, title, value) in enumerate(rows):
            card = QtWidgets.QFrame()
            card.setStyleSheet(f"background:{theme['bg_panel']};border:1px solid {theme['border']};border-radius:8px;")
            card.setMinimumHeight(70)
            card_layout = QtWidgets.QVBoxLayout(card)
            card_layout.setContentsMargins(8, 6, 8, 6)
            card_layout.setSpacing(2)

            title_label = QtWidgets.QLabel(title)
            title_label.setStyleSheet(f"color:{theme['text_muted']};font:11px {theme['font_ui']};")
            value_label = QtWidgets.QLabel(value)
            value_label.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_data']};")
            value_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            value_label.setWordWrap(True)
            card_layout.addWidget(title_label)
            card_layout.addWidget(value_label)

            self.info_grid.addWidget(card, row_index // 2, row_index % 2)
            self.value_labels[key] = value_label

        button_row = QtWidgets.QHBoxLayout()
        button_row.setSpacing(6)
        left_layout.addLayout(button_row)

        left_button_palette = {
            "PAUSE": {
                "bg": "#fff6e8",
                "border": "#d98f00",
                "text": "#7a4b00",
                "hover_bg": "#ffefcf",
                "hover_border": "#b37400",
                "press_bg": "#ffe3a7",
                "press_border": "#8f5c00",
            },
            None: {
                "bg": "#eaf3ff",
                "border": "#3d84d9",
                "text": "#144f8a",
                "hover_bg": "#d9eaff",
                "hover_border": "#2e73c5",
                "press_bg": "#c8e0ff",
                "press_border": "#1f5ea8",
            },
            "CLEAR_LOGS": {
                "bg": "#eef2f7",
                "border": "#8fa0b5",
                "text": "#334155",
                "hover_bg": "#e4ebf3",
                "hover_border": "#7d8ea3",
                "press_bg": "#dbe4ee",
                "press_border": "#66788f",
            },
            "EXIT_APP": {
                "bg": "#fff1f2",
                "border": "#d73a49",
                "text": "#9f1d2a",
                "hover_bg": "#ffe4e6",
                "hover_border": "#be2432",
                "press_bg": "#ffd0d5",
                "press_border": "#9f1d2a",
            },
        }

        for text, cmd in (("暂停", "PAUSE"), ("重新连接", None), ("清空日志", "CLEAR_LOGS"), ("退出", "EXIT_APP")):
            button = QtWidgets.QPushButton(text)
            button.setMinimumHeight(30)
            palette = left_button_palette.get(cmd, left_button_palette[None])
            button.setStyleSheet(
                f"QPushButton {{"
                f"background:{palette['bg']};"
                f"color:{palette['text']};"
                f"border:1px solid {palette['border']};"
                f"border-radius:6px;"
                f"padding:6px 12px;"
                f"font:600 12px {theme['font_ui']};"
                f"}}"
                f"QPushButton:hover {{"
                f"background:{palette['hover_bg']};"
                f"border-color:{palette['hover_border']};"
                f"color:{palette['text']};"
                f"}}"
                f"QPushButton:pressed {{"
                f"background:{palette['press_bg']};"
                f"border-color:{palette['press_border']};"
                f"color:{palette['text']};"
                f"}}"
            )
            if cmd == "CLEAR_LOGS":
                button.clicked.connect(lambda _checked=False: self.clear_log_panel())
            elif cmd == "EXIT_APP":
                button.clicked.connect(lambda _checked=False: self.window.close())
            elif cmd is None:
                button.clicked.connect(lambda _checked=False: reconnect_client(self.state, self.log_q))
            elif cmd == "PAUSE":
                button.clicked.connect(lambda _checked=False: self.send_pause_all())
            else:
                button.clicked.connect(lambda _checked=False, command=cmd: send_command(self.state, self.log_q, command))
            button_row.addWidget(button)

        timer_box = QtWidgets.QFrame()
        timer_box.setStyleSheet(f"background:{theme['bg_panel_soft']};border:1px solid {theme['border']};border-radius:6px;")
        timer_box_layout = QtWidgets.QHBoxLayout(timer_box)
        timer_box_layout.setContentsMargins(8, 4, 8, 4)
        timer_box_layout.setSpacing(8)

        timer_title = QtWidgets.QLabel("计时")
        timer_title.setStyleSheet(f"color:{theme['text_muted']};font:11px {theme['font_ui']};")
        self.timer_current_label = QtWidgets.QLabel("00:00.000")
        self.timer_current_label.setStyleSheet(f"color:#c46a12;font:700 14px {theme['font_data']};")
        self.timer_last_label = QtWidgets.QLabel("上次 --")
        self.timer_last_label.setStyleSheet(f"color:{theme['text_muted']};font:11px {theme['font_ui']};")

        timer_box_layout.addWidget(timer_title)
        timer_box_layout.addWidget(self.timer_current_label)
        timer_box_layout.addWidget(self.timer_last_label)
        button_row.addWidget(timer_box)
        button_row.addStretch(1)

        bits_box = QtWidgets.QGroupBox("循迹 Bits")
        bits_layout = QtWidgets.QHBoxLayout(bits_box)
        bits_layout.setContentsMargins(8, 10, 8, 8)
        bits_layout.setSpacing(6)
        self.bit_labels = []
        for index in range(int(self.args.cols)):
            label = QtWidgets.QLabel(str(index))
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setFixedSize(28, 24)
            label.setStyleSheet(f"background:{theme['bg_panel_soft']};color:{theme['text_muted']};border:1px solid {theme['border']};border-radius:4px;font:11px {theme['font_data']};")
            bits_layout.addWidget(label)
            self.bit_labels.append(label)
        left_layout.addWidget(bits_box)

        def style_plot_widget(plot_widget: object, y_label: str, x_label: str = "时间 (s)") -> None:
            plot_widget.setBackground(theme["plot_bg"])
            plot_widget.showGrid(x=True, y=True, alpha=0.18)

            plot_item = plot_widget.getPlotItem()
            plot_item.setLabel("left", y_label, color=theme["plot_axis"])
            plot_item.setLabel("bottom", x_label, color=theme["plot_axis"])
            plot_item.getAxis("left").setTextPen(pg.mkColor(theme["plot_axis"]))
            plot_item.getAxis("bottom").setTextPen(pg.mkColor(theme["plot_axis"]))
            plot_item.getAxis("left").setPen(pg.mkPen(theme["plot_border"]))
            plot_item.getAxis("bottom").setPen(pg.mkPen(theme["plot_border"]))
            plot_item.getAxis("left").setGrid(90)
            plot_item.getAxis("bottom").setGrid(90)
            plot_item.vb.setBorder(pg.mkPen(theme["plot_border"]))

            legend = plot_widget.addLegend(offset=(12, 8))
            legend.setBrush(pg.mkBrush(theme["plot_legend_bg"]))
            legend.setPen(pg.mkPen(theme["plot_legend_border"]))

            plot_widget.setClipToView(True)
            plot_widget.setDownsampling(auto=True, mode="mean")
            plot_widget.setMouseEnabled(x=True, y=False)

        def style_error_plot_widget(plot_widget: object) -> None:
            plot_widget.setBackground(theme["plot_bg"])
            plot_widget.showGrid(x=True, y=True, alpha=0.18)

            plot_item = plot_widget.getPlotItem()
            plot_item.setLabel("left", "时间倒推 (s)", color=theme["plot_axis"])
            plot_item.setLabel("bottom", "位置误差 (0为中心)", color=theme["plot_axis"])
            plot_item.getAxis("left").setTextPen(pg.mkColor(theme["plot_axis"]))
            plot_item.getAxis("bottom").setTextPen(pg.mkColor(theme["plot_axis"]))
            plot_item.getAxis("left").setPen(pg.mkPen(theme["plot_border"]))
            plot_item.getAxis("bottom").setPen(pg.mkPen(theme["plot_border"]))
            plot_item.getAxis("left").setGrid(100)
            plot_item.getAxis("bottom").setGrid(100)
            plot_item.vb.setBorder(pg.mkPen(theme["plot_border"]))
            plot_widget.setClipToView(True)
            plot_widget.setDownsampling(auto=True, mode="mean")
            plot_widget.setMouseEnabled(x=True, y=False)
            plot_widget.addItem(
                pg.InfiniteLine(
                    pos=0.0,
                    angle=90,
                    pen=pg.mkPen(theme["plot_zero"], width=1.0, style=QtCore.Qt.DashLine),
                )
            )

        def build_speed_plot_card(title: str) -> tuple[object, object]:
            card = QtWidgets.QFrame()
            card.setStyleSheet(f"background:{theme['bg_panel']};border:1px solid {theme['border']};border-radius:10px;")
            card.setMinimumHeight(230)
            layout = QtWidgets.QVBoxLayout(card)
            layout.setContentsMargins(8, 8, 8, 8)
            layout.setSpacing(6)

            plot_title = QtWidgets.QLabel(title)
            plot_title.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_ui']};")
            layout.addWidget(plot_title)

            plot_widget = pg.PlotWidget()
            style_plot_widget(plot_widget, "转速", "时间 (s)")
            plot_widget.addItem(
                pg.InfiniteLine(
                    pos=0.0,
                    angle=0,
                    pen=pg.mkPen(theme["plot_zero"], width=1.0, style=QtCore.Qt.DashLine),
                )
            )
            layout.addWidget(plot_widget)
            return card, plot_widget

        def build_error_plot_card(title: str) -> tuple[object, object]:
            card = QtWidgets.QFrame()
            card.setStyleSheet(f"background:{theme['bg_panel']};border:1px solid {theme['border']};border-radius:10px;")
            card.setMinimumHeight(230)
            layout = QtWidgets.QVBoxLayout(card)
            layout.setContentsMargins(8, 8, 8, 8)
            layout.setSpacing(6)

            plot_title = QtWidgets.QLabel(title)
            plot_title.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_ui']};")
            layout.addWidget(plot_title)

            plot_widget = pg.PlotWidget()
            style_error_plot_widget(plot_widget)
            layout.addWidget(plot_widget)
            return card, plot_widget

        def build_speed_curves(plot_widget: object, target_color: str, actual_color: str) -> tuple[object, object, object]:
            target_pen = pg.mkPen(target_color, width=1.8, style=QtCore.Qt.DashLine)
            target_pen.setCapStyle(QtCore.Qt.RoundCap)
            target_pen.setJoinStyle(QtCore.Qt.RoundJoin)
            actual_pen = pg.mkPen(actual_color, width=2.2)
            actual_pen.setCapStyle(QtCore.Qt.RoundCap)
            actual_pen.setJoinStyle(QtCore.Qt.RoundJoin)
            vq_pen = pg.mkPen("#FF6600", width=1.5)
            vq_pen.setCapStyle(QtCore.Qt.RoundCap)
            vq_pen.setJoinStyle(QtCore.Qt.RoundJoin)

            target_curve = plot_widget.plot(
                name="目标",
                pen=target_pen,
            )
            actual_curve = plot_widget.plot(name="实际", pen=actual_pen)
            vq_curve = plot_widget.plot(name="Vq×1000", pen=vq_pen)
            return target_curve, actual_curve, vq_curve

        def build_elec_wheel_card(title: str) -> tuple[object, object, object]:
            card = QtWidgets.QFrame()
            card.setStyleSheet(f"background:{theme['bg_panel']};border:1px solid {theme['border']};border-radius:10px;")
            card.setMinimumHeight(220)
            layout = QtWidgets.QVBoxLayout(card)
            layout.setContentsMargins(8, 8, 8, 8)
            layout.setSpacing(6)

            plot_title = QtWidgets.QLabel(title)
            plot_title.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_ui']};")
            layout.addWidget(plot_title)

            wheel_label = QtWidgets.QLabel()
            wheel_label.setFixedSize(420, 170)
            wheel_label.setAlignment(QtCore.Qt.AlignCenter)
            wheel_stats = QtWidgets.QLabel("左轮=-- 右轮=--")
            wheel_stats.setStyleSheet(f"color:{theme['text_muted']};font:11px {theme['font_data']};")
            wheel_stats.setAlignment(QtCore.Qt.AlignCenter)
            layout.addWidget(wheel_label, 0, QtCore.Qt.AlignCenter)
            layout.addWidget(wheel_stats)
            return card, wheel_label, wheel_stats

        def build_gyro3d_card(title: str, show_reset: bool = False) -> tuple[object, object, object]:
            card = QtWidgets.QFrame()
            card.setStyleSheet(f"background:{theme['bg_panel']};border:1px solid {theme['border']};border-radius:10px;")
            layout = QtWidgets.QVBoxLayout(card)
            layout.setContentsMargins(8, 8, 8, 8)
            layout.setSpacing(6)

            header = QtWidgets.QHBoxLayout()
            header.setContentsMargins(0, 0, 0, 0)
            header.setSpacing(8)

            title_label = QtWidgets.QLabel(title)
            title_label.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_ui']};")
            header.addWidget(title_label)
            header.addStretch(1)

            if show_reset:
                reset_btn = QtWidgets.QPushButton("重置姿态")
                reset_btn.setMinimumHeight(26)
                reset_btn.setStyleSheet(f"background:{theme['accent_soft']};color:{theme['accent']};border:1px solid #93d7cd;border-radius:5px;padding:4px 10px;font:600 11px {theme['font_ui']};")
                reset_btn.clicked.connect(self.reset_attitude_zero)
                header.addWidget(reset_btn)

            gyro_label = QtWidgets.QLabel()
            gyro_label.setFixedSize(300, 190)
            gyro_stats_label = QtWidgets.QLabel("roll=-- pitch=-- yaw=--")
            gyro_stats_label.setStyleSheet(f"color:{theme['text_muted']};font:11px {theme['font_data']};")
            gyro_stats_label.setAlignment(QtCore.Qt.AlignCenter)

            layout.addLayout(header)
            layout.addWidget(gyro_label, 0, QtCore.Qt.AlignCenter)
            layout.addWidget(gyro_stats_label)
            return card, gyro_label, gyro_stats_label

        class NoWheelTabBar(QtWidgets.QTabBar):
            def wheelEvent(self, event: object) -> None:
                event.accept()

        tabs = QtWidgets.QTabWidget()
        self.tabs = tabs
        tabs.setTabBar(NoWheelTabBar())
        tabs.setUsesScrollButtons(True)
        tabs.setMovable(True)
        tabs.tabBar().setExpanding(False)
        tabs.tabBar().setElideMode(QtCore.Qt.ElideRight)
        right_column_layout.addWidget(tabs)

        main_tab = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout(main_tab)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        main_card = QtWidgets.QFrame()
        main_card.setStyleSheet(f"background:{theme['bg_panel']};border:1px solid {theme['border']};border-radius:12px;")
        main_card_layout = QtWidgets.QVBoxLayout(main_card)
        main_card_layout.setContentsMargins(12, 12, 12, 12)
        main_card_layout.setSpacing(10)

        main_title = QtWidgets.QLabel("主菜单")
        main_title.setStyleSheet(f"color:{theme['text_primary']};font:700 16px {theme['font_ui']};")
        main_desc = QtWidgets.QLabel("仅用于页面跳转，不提供参数设置。")
        main_desc.setStyleSheet(f"color:{theme['text_muted']};font:12px {theme['font_ui']};")
        main_card_layout.addWidget(main_title)
        main_card_layout.addWidget(main_desc)

        nav_grid = QtWidgets.QGridLayout()
        nav_grid.setContentsMargins(0, 6, 0, 0)
        nav_grid.setHorizontalSpacing(10)
        nav_grid.setVerticalSpacing(10)

        nav_items = [
            ("运行调试", _BOARD_PAGE_RUN),
            ("FOC 控制", _BOARD_PAGE_FOC),
            ("循迹调试", _BOARD_PAGE_LINE),
            ("路口调试", _BOARD_PAGE_JUNCTION),
            ("灰度调试", _BOARD_PAGE_GRAY),
            ("IMU 调试", _BOARD_PAGE_IMU),
            ("负压调试", _BOARD_PAGE_NEG),
            ("开环测试", _BOARD_PAGE_OPENLOOP),
            ("WiFi 页面", _BOARD_PAGE_WIFI),
        ]

        for idx, (text, page_id) in enumerate(nav_items):
            btn = QtWidgets.QPushButton(text)
            btn.setMinimumHeight(34)
            btn.setStyleSheet("background:#f8fafc;color:#334155;border:1px solid #dbe3ee;border-radius:8px;font:600 12px 'Consolas';")
            btn.clicked.connect(lambda _checked=False, p=page_id: self.navigate_to_page(p))
            nav_grid.addWidget(btn, idx // 3, idx % 3)

        main_card_layout.addLayout(nav_grid)
        main_layout.addWidget(main_card)
        main_layout.addStretch(1)

        self.tab_main_index = tabs.addTab(main_tab, "主菜单")

        foc_tab = QtWidgets.QWidget()
        foc_layout = QtWidgets.QVBoxLayout(foc_tab)
        foc_layout.setContentsMargins(8, 8, 8, 8)
        foc_layout.setSpacing(8)
        self.tab_foc_index = tabs.addTab(foc_tab, "FOC控制")

        foc_scroll_area = QtWidgets.QScrollArea()
        foc_scroll_area.setWidgetResizable(True)
        foc_scroll_area.setStyleSheet("QScrollArea { border: none; }")
        foc_scroll_content = QtWidgets.QWidget()
        foc_scroll_layout = QtWidgets.QVBoxLayout(foc_scroll_content)
        foc_scroll_layout.setContentsMargins(0, 0, 0, 0)
        foc_scroll_layout.setSpacing(8)

        foc_cmd_row = QtWidgets.QHBoxLayout()
        foc_cmd_row.setContentsMargins(0, 0, 0, 0)
        foc_cmd_row.setSpacing(6)
        btn_foc_start = QtWidgets.QPushButton("START 主流程")
        btn_foc_pause = QtWidgets.QPushButton("停机(主流程+FOC测试)")
        btn_foc_query = QtWidgets.QPushButton("读取FOC测试参数")
        self.btn_foc_start = btn_foc_start
        self.btn_foc_pause = btn_foc_pause
        btn_foc_start.setStyleSheet(_BTN_ACTION_GREEN)
        btn_foc_pause.setStyleSheet(_BTN_ACTION_RED)
        btn_foc_start.clicked.connect(lambda _checked=False: self.send_main_start_safe())
        btn_foc_pause.clicked.connect(lambda _checked=False: self.send_foc_pause_all())
        btn_foc_query.clicked.connect(self.query_foc_test_params)
        self.chk_pause_plot = QtWidgets.QCheckBox("暂停波形刷新")
        self.chk_pause_plot.setChecked(False)
        foc_cmd_row.addWidget(btn_foc_start)
        foc_cmd_row.addWidget(btn_foc_pause)
        foc_cmd_row.addWidget(btn_foc_query)
        foc_cmd_row.addWidget(self.chk_pause_plot)
        foc_cmd_row.addStretch(1)
        foc_scroll_layout.addLayout(foc_cmd_row)

        gyro3d_card, self.gyro_3d_label, self.gyro_3d_stats_label = build_gyro3d_card("车身姿态 3D", show_reset=True)
        foc_scroll_layout.addWidget(gyro3d_card, 0)

        foc_speed_plots_row = QtWidgets.QWidget()
        foc_speed_plots_row_layout = QtWidgets.QHBoxLayout(foc_speed_plots_row)
        foc_speed_plots_row_layout.setContentsMargins(0, 0, 0, 0)
        foc_speed_plots_row_layout.setSpacing(8)

        foc_left_speed_card, self.foc_left_speed_plot = build_speed_plot_card("FOC左轮速度")
        self.foc_speed_curve_lt, self.foc_speed_curve_la, self.foc_speed_curve_lvq = build_speed_curves(
            self.foc_left_speed_plot,
            theme["speed_lt"],
            theme["speed_la"],
        )
        self.foc_left_speed_stats_label = QtWidgets.QLabel("目标=--   实际=--")
        self.foc_left_speed_stats_label.setStyleSheet(f"color:{theme['text_muted']};font:600 11px {theme['font_data']};")
        foc_left_speed_card.layout().addWidget(self.foc_left_speed_stats_label)
        foc_speed_plots_row_layout.addWidget(foc_left_speed_card, 1)

        foc_right_speed_card, self.foc_right_speed_plot = build_speed_plot_card("FOC右轮速度")
        self.foc_speed_curve_rt, self.foc_speed_curve_ra, self.foc_speed_curve_rvq = build_speed_curves(
            self.foc_right_speed_plot,
            theme["speed_rt"],
            theme["speed_ra"],
        )
        self.foc_right_speed_stats_label = QtWidgets.QLabel("目标=--   实际=--")
        self.foc_right_speed_stats_label.setStyleSheet(f"color:{theme['text_muted']};font:600 11px {theme['font_data']};")
        foc_right_speed_card.layout().addWidget(self.foc_right_speed_stats_label)
        foc_speed_plots_row_layout.addWidget(foc_right_speed_card, 1)

        foc_scroll_layout.addWidget(foc_speed_plots_row)

        foc_set_group = QtWidgets.QGroupBox("FOC测试目标设定")
        foc_set_grid = QtWidgets.QGridLayout(foc_set_group)

        self.foc_test_run_sb = QtWidgets.QDoubleSpinBox()
        self.foc_test_run_sb.wheelEvent = lambda e: e.ignore()
        self.foc_test_run_sb.setRange(0.0, 1.0)
        self.foc_test_run_sb.setSingleStep(1.0)
        self.foc_test_run_sb.setDecimals(0)

        self.foc_test_left_rpm_sb = QtWidgets.QDoubleSpinBox()
        self.foc_test_left_rpm_sb.wheelEvent = lambda e: e.ignore()
        self.foc_test_left_rpm_sb.setRange(-1500.0, 1500.0)
        self.foc_test_left_rpm_sb.setSingleStep(20.0)
        self.foc_test_left_rpm_sb.setDecimals(0)

        self.foc_test_right_rpm_sb = QtWidgets.QDoubleSpinBox()
        self.foc_test_right_rpm_sb.wheelEvent = lambda e: e.ignore()
        self.foc_test_right_rpm_sb.setRange(-1500.0, 1500.0)
        self.foc_test_right_rpm_sb.setSingleStep(20.0)
        self.foc_test_right_rpm_sb.setDecimals(0)

        btn_run_0 = QtWidgets.QPushButton("Run=0")
        btn_run_1 = QtWidgets.QPushButton("Run=1")
        btn_run_0.clicked.connect(lambda _checked=False: self.send_foc_test_run(0))
        btn_run_1.clicked.connect(lambda _checked=False: self.send_foc_test_run(1))

        btn_send_rpm = QtWidgets.QPushButton("写入双轮RPM")
        btn_send_all = QtWidgets.QPushButton("一键写入Run+RPM")
        btn_send_all.setStyleSheet(_BTN_ACTION_GREEN)
        btn_run_1.setStyleSheet(_BTN_ACTION_GREEN)
        btn_send_rpm.clicked.connect(self.send_foc_test_rpm_targets)
        btn_send_all.clicked.connect(self.send_foc_test_all)

        foc_set_grid.addWidget(QtWidgets.QLabel("Test Run"), 0, 0)
        foc_set_grid.addWidget(self.foc_test_run_sb, 0, 1)
        foc_set_grid.addWidget(btn_run_0, 0, 2)
        foc_set_grid.addWidget(btn_run_1, 0, 3)

        foc_set_grid.addWidget(QtWidgets.QLabel("LTest RPM"), 1, 0)
        foc_set_grid.addWidget(self.foc_test_left_rpm_sb, 1, 1)
        foc_set_grid.addWidget(QtWidgets.QLabel("RTest RPM"), 1, 2)
        foc_set_grid.addWidget(self.foc_test_right_rpm_sb, 1, 3)

        foc_set_grid.addWidget(btn_send_rpm, 2, 2)
        foc_set_grid.addWidget(btn_send_all, 2, 3)
        foc_scroll_layout.addWidget(foc_set_group)

        foc_preset_row = QtWidgets.QHBoxLayout()
        foc_preset_row.setContentsMargins(0, 0, 0, 0)
        foc_preset_row.setSpacing(6)
        for text, left_rpm, right_rpm in (
            ("停转 0/0", 0.0, 0.0),
            ("同向 200/200", 200.0, 200.0),
            ("同向 -200/-200", -200.0, -200.0),
            ("反向 300/-300", 300.0, -300.0),
        ):
            btn = QtWidgets.QPushButton(text)
            btn.clicked.connect(lambda _checked=False, l=left_rpm, r=right_rpm: self.set_foc_test_rpm_preset(l, r))
            foc_preset_row.addWidget(btn)
        foc_preset_row.addStretch(1)
        foc_scroll_layout.addLayout(foc_preset_row)

        foc_live_group = QtWidgets.QGroupBox("FOC实时反馈")
        foc_live_layout = QtWidgets.QVBoxLayout(foc_live_group)
        foc_live_layout.setContentsMargins(8, 10, 8, 8)
        foc_live_layout.setSpacing(6)
        self.foc_test_state_label = QtWidgets.QLabel("主流程状态: --")
        self.foc_test_target_label = QtWidgets.QLabel("目标RPM  左=--   右=--")
        self.foc_test_actual_label = QtWidgets.QLabel("实际RPM  左=--   右=--")
        self.foc_test_elec_label = QtWidgets.QLabel("电角度   左=--   右=--")
        self.foc_test_target_label.setStyleSheet("color:#334155;font:12px 'Consolas';")
        self.foc_test_actual_label.setStyleSheet("color:#334155;font:12px 'Consolas';")
        self.foc_test_elec_label.setStyleSheet("color:#334155;font:12px 'Consolas';")
        foc_live_layout.addWidget(self.foc_test_state_label)
        foc_live_layout.addWidget(self.foc_test_target_label)
        foc_live_layout.addWidget(self.foc_test_actual_label)
        foc_live_layout.addWidget(self.foc_test_elec_label)
        foc_scroll_layout.addWidget(foc_live_group)

        foc_tip = QtWidgets.QLabel("提示: 若固件键名不一致，请修改上方键名后再写入。")
        foc_tip.setStyleSheet("color:#64748b;font:11px 'Consolas';")
        foc_scroll_layout.addWidget(foc_tip)
        
        self.param_widgets = {}
        self.param_name_map = {}
        self.tab_param_indices = {}
        self.tab_param_indices[_BOARD_PAGE_FOC] = self.tab_foc_index
        channel_limit = float(max(1, self.model.cols))
        PARAM_GROUPS = [
            (_BOARD_PAGE_RUN, "运行 (RUN)", [
                ("BAS_SPD", "基础速度", 0.0, 2000.0, 10.0),
                ("BAT_CEL", "电池串数", 2.0, 6.0, 1.0),
            ]),
            (_BOARD_PAGE_FOC, "FOC速度控制 (FOC)", [
                ("BAS_SPD", "基础速度", 0.0, 2000.0, 10.0),
                ("KP_L", "左轮 Kp", 0.0, 100.0, 0.005),
                ("KI_L", "左轮 Ki", 0.0, 100.0, 0.005),
                ("VQL_L", "左轮 Vq", 0.0, 1.0, 0.0005),
                ("KP_R", "右轮 Kp", 0.0, 100.0, 0.005),
                ("KI_R", "右轮 Ki", 0.0, 100.0, 0.005),
                ("VQL_R", "右轮 Vq", 0.0, 1.0, 0.0005),
                ("A5_LP_A", "编码器低通Alpha", 0.001, 1.0, 0.001),
                ("A5_AVG_N", "编码器速度均值窗口", 1.0, 100.0, 1.0),
                ("A5_ZDB_R", "编码器零速死区RPM", 0.0, 20.0, 0.1),
            ]),
            (_BOARD_PAGE_LINE, "循迹调试 (LINE)", [
                ("STR_KP", "转向 Kp", 0.0, 100.0, 0.01),
                ("STR_KD", "转向 Kd", 0.0, 100.0, 0.01),
                ("LF_KP2", "循迹备用 Kp2", 0.0, 100.0, 0.01),
                ("LF_GKD", "循迹全局 KD", 0.0, 100.0, 0.01),
                ("TRN_SCL", "转弯速度缩放", 0.0, 0.01, 0.001),
            ]),
            (_BOARD_PAGE_JUNCTION, "路口调试 (JUNCTION)", [
                ("PR_TURN", "预转弯距离(mm)", 0.0, 500.0, 1.0),
                ("IS_PMAX", "路口最大偏移", 0.0, 6.0, 0.1),
                ("IS_LCKS", "路口锁定时间(s)", 0.0, 2.0, 0.1),
                ("IS_HITM", "路口电感最小触发", 0.0, channel_limit, 1.0),
                ("IS_CFRM", "路口确认帧数", 0.0, 10.0, 1.0),
                ("IS_HIME", "边沿电感最小触发", 0.0, channel_limit, 1.0),
                ("DRPM_LM", "转向差速限幅", 0.0, 1000.0, 10.0),
                ("T_MINYW", "最小偏航角(deg)", 0.0, 180.0, 1.0),
                ("T_MINTS", "最小转向时间(s)", 0.0, 5.0, 0.1),
                ("T_DNYAW", "完成偏航角(deg)", 0.0, 180.0, 1.0),
                ("T_TIMES", "转向超时时间(s)", 0.0, 10.0, 0.1),
                ("T_YSGN", "偏航符号", -1.0, 1.0, 2.0),
                ("T_OSGN", "转向输出符号", -1.0, 1.0, 2.0),
                ("FIN_CST", "冲线滑行距离(mm)", 0.0, 1000.0, 10.0),
            ]),
            (_BOARD_PAGE_GRAY, "灰度调试 (GRAY)", [
                ("IS_HITM", "路口电感最小触发", 0.0, channel_limit, 1.0),
                ("IS_CFRM", "路口确认帧数", 0.0, 10.0, 1.0),
                ("IS_HIME", "边沿电感最小触发", 0.0, channel_limit, 1.0),
            ]),
            (_BOARD_PAGE_IMU, "IMU调试 (IMU)", [
                ("GY_LP_A", "陀螺仪低通Alpha", 0.001, 1.0, 0.001),
            ]),
            (_BOARD_PAGE_NEG, "负压调试 (NEG)", [
                ("NP_EN", "负压使能", 0.0, 1.0, 1.0),
                ("NP_THR", "负压油门(%)", 0.0, 100.0, 1.0),
            ]),
            (_BOARD_PAGE_OPENLOOP, "开环测试 (OPENLOOP)", [
                ("OL_RUN", "开环运行", 0.0, 1.0, 1.0),
                ("OL_HZ", "开环强拖Hz", 0.0, 100.0, 1.0),
                ("OL_VQ", "开环强拖Vq", 0.0, 1.0, 0.0005),
            ]),
            (_BOARD_PAGE_WIFI, "WiFi连接 (WIFI)", [
                ("BAT_CEL", "电池串数", 2.0, 6.0, 1.0),
            ])
        ]

        # Inject FOC parameter group UI here
        for _p_id, _p_name, _p_params in PARAM_GROUPS:
            if _p_id == _BOARD_PAGE_FOC:
                foc_param_group_box = QtWidgets.QGroupBox(_p_name)
                foc_param_group_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid #dbe3ee; border-radius: 5px; margin-top: 15px; padding-top: 10px; background:#ffffff;} QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; color: #2563eb; }")
                foc_param_form = QtWidgets.QFormLayout(foc_param_group_box)
                for key, name, min_v, max_v, step in _p_params:
                    self.param_name_map[key] = name
                    sb = QtWidgets.QDoubleSpinBox()
                    sb.wheelEvent = lambda e: e.ignore()
                    sb.setRange(min_v, max_v)
                    sb.setSingleStep(step)
                    sb.setDecimals(_PARAM_DECIMALS)
                    btn = QtWidgets.QPushButton("写入")
                    btn.clicked.connect(lambda _, k=key, s=sb: self.send_param_debounced(k, s))
                    row = QtWidgets.QHBoxLayout()
                    row.addWidget(sb)
                    row.addWidget(btn)
                    foc_param_form.addRow(name, row)
                    self.param_widgets[key] = sb
                foc_scroll_layout.addWidget(foc_param_group_box)
                break
        
        foc_scroll_content.setLayout(foc_scroll_layout)
        foc_scroll_area.setWidget(foc_scroll_content)
        foc_layout.addWidget(foc_scroll_area)
        
        for page_id, group_name, group_params in PARAM_GROUPS:
            if page_id == _BOARD_PAGE_FOC:
                continue

            page_tab = QtWidgets.QWidget()
            page_layout = QtWidgets.QVBoxLayout(page_tab)
            page_layout.setContentsMargins(8, 8, 8, 8)
            page_layout.setSpacing(10)
            
            top_tools = QtWidgets.QHBoxLayout()
            btn_sync_page = QtWidgets.QPushButton("同步本页")
            btn_query_page = QtWidgets.QPushButton("读取参数")
            btn_load_local = QtWidgets.QPushButton("加载配置")
            btn_save_local = QtWidgets.QPushButton("保存配置")
            btn_save_flash = QtWidgets.QPushButton("固化到Flash")
            btn_save_flash.setStyleSheet("background-color:#2563eb;color:#ffffff;font-weight:bold;border:1px solid #1d4ed8;border-radius:5px;padding:4px 10px;")
            
            btn_sync_page.clicked.connect(lambda _checked=False, pid=page_id: self.request_page_sync(pid, query_params=True))
            btn_query_page.clicked.connect(lambda _checked=False, pid=page_id: self.query_page_params(pid))
            btn_load_local.clicked.connect(self.load_local_profile)
            btn_save_local.clicked.connect(self.save_local_profile)
            btn_save_flash.clicked.connect(self.send_save_debounced)
            
            top_tools.addWidget(btn_sync_page)
            top_tools.addWidget(btn_query_page)
            top_tools.addWidget(btn_load_local)
            top_tools.addWidget(btn_save_local)
            top_tools.addWidget(btn_save_flash)
            top_tools.addStretch(1)
            page_layout.addLayout(top_tools)
            
            scroll_area = QtWidgets.QScrollArea()
            scroll_area.setWidgetResizable(True)
            scroll_area.setStyleSheet("QScrollArea { border: none; }")
            scroll_content = QtWidgets.QWidget()
            scroll_layout = QtWidgets.QVBoxLayout(scroll_content)
            scroll_layout.setContentsMargins(0, 0, 0, 0)
            
            group_box = QtWidgets.QGroupBox(group_name)
            group_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid #dbe3ee; border-radius: 5px; margin-top: 15px; padding-top: 10px; background:#ffffff;} QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; color: #2563eb; }")
            form_layout = QtWidgets.QFormLayout(group_box)
            for key, name, min_v, max_v, step in group_params:
                self.param_name_map[key] = name
                sb = QtWidgets.QDoubleSpinBox()
                sb.wheelEvent = lambda e: e.ignore()
                sb.setRange(min_v, max_v)
                sb.setSingleStep(step)
                sb.setDecimals(_PARAM_DECIMALS)
                btn = QtWidgets.QPushButton("写入")
                btn.clicked.connect(lambda _, k=key, s=sb: self.send_param_debounced(k, s))
                row = QtWidgets.QHBoxLayout()
                row.addWidget(sb)
                row.addWidget(btn)
                form_layout.addRow(name, row)
                self.param_widgets[key] = sb
            scroll_layout.addWidget(group_box)

            if page_id == _BOARD_PAGE_RUN:
                run_overview_card = QtWidgets.QFrame()
                run_overview_card.setStyleSheet("background:#ffffff;border:1px solid #dbe3ee;border-radius:10px;")
                run_overview_layout = QtWidgets.QVBoxLayout(run_overview_card)
                run_overview_layout.setContentsMargins(8, 8, 8, 8)
                run_overview_layout.setSpacing(6)

                run_top_row = QtWidgets.QHBoxLayout()
                run_top_row.setContentsMargins(0, 0, 0, 0)
                run_top_row.setSpacing(8)
                run_title = QtWidgets.QLabel("运行实时状态")
                run_title.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_ui']};")
                run_top_row.addWidget(run_title)
                run_top_row.addStretch(1)
                run_overview_layout.addLayout(run_top_row)

                run_cmd_row = QtWidgets.QHBoxLayout()
                run_cmd_row.setContentsMargins(0, 0, 0, 0)
                run_cmd_row.setSpacing(6)
                btn_run_start = QtWidgets.QPushButton("START 主流程")
                btn_run_pause = QtWidgets.QPushButton("停机(主流程+FOC测试)")
                self.btn_run_start = btn_run_start
                self.btn_run_pause = btn_run_pause
                btn_run_start.setStyleSheet(_BTN_ACTION_GREEN)
                btn_run_pause.setStyleSheet(_BTN_ACTION_RED)
                btn_run_start.clicked.connect(lambda _checked=False: self.send_main_start_safe())
                btn_run_pause.clicked.connect(lambda _checked=False: self.send_foc_pause_all())
                run_cmd_row.addWidget(btn_run_start)
                run_cmd_row.addWidget(btn_run_pause)
                run_cmd_row.addStretch(1)
                run_overview_layout.addLayout(run_cmd_row)

                self.run_wheel_speed_label = QtWidgets.QLabel("左右轮速度: 左目标=-- 右目标=-- 左实际=-- 右实际=--")
                self.run_wheel_speed_label.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_data']};")
                self.run_path_progress_label = QtWidgets.QLabel("路径进度: idx=-- 待执行=-- 剩余路径=--")
                self.run_path_progress_label.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_data']};")
                run_odom_btn_row = QtWidgets.QHBoxLayout()
                run_odom_btn_row.setContentsMargins(0, 0, 0, 0)
                run_odom_btn_row.setSpacing(8)
                btn_odom_start = QtWidgets.QPushButton("开始计数")
                btn_odom_clear = QtWidgets.QPushButton("里程清零")
                btn_odom_start.clicked.connect(self.send_odom_start)
                btn_odom_clear.clicked.connect(self.send_odom_clear)
                run_odom_btn_row.addWidget(btn_odom_start)
                run_odom_btn_row.addWidget(btn_odom_clear)
                run_odom_btn_row.addStretch(1)
                self.run_odom_state_label = QtWidgets.QLabel("里程计状态: --")
                self.run_odom_state_label.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_data']};")
                self.run_odom_label = QtWidgets.QLabel("轮里程: 左=-- mm 右=-- mm")
                self.run_odom_label.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_data']};")
                run_overview_layout.addWidget(self.run_wheel_speed_label)
                run_overview_layout.addWidget(self.run_path_progress_label)
                run_overview_layout.addLayout(run_odom_btn_row)
                run_overview_layout.addWidget(self.run_odom_state_label)
                run_overview_layout.addWidget(self.run_odom_label)

                scroll_layout.addWidget(run_overview_card)

                # ===== 双核 CPU 占用率圆圈 =====
                cpu_gauge_row = QtWidgets.QWidget()
                cpu_gauge_layout = QtWidgets.QHBoxLayout(cpu_gauge_row)
                cpu_gauge_layout.setContentsMargins(0, 0, 0, 0)
                cpu_gauge_layout.setSpacing(16)
                cpu_gauge_layout.addStretch(1)
                self.cpu0_gauge = self.CpuGaugeWidget("CPU0", "#2563eb")
                self.cpu1_gauge = self.CpuGaugeWidget("CPU1", "#059669")
                cpu_gauge_layout.addWidget(self.cpu0_gauge)
                cpu_gauge_layout.addWidget(self.cpu1_gauge)
                cpu_gauge_layout.addStretch(1)
                scroll_layout.addWidget(cpu_gauge_row)
                # ===================================

                run_speed_plots_row = QtWidgets.QWidget()
                run_speed_plots_row_layout = QtWidgets.QHBoxLayout(run_speed_plots_row)
                run_speed_plots_row_layout.setContentsMargins(0, 0, 0, 0)
                run_speed_plots_row_layout.setSpacing(8)

                run_left_speed_card, self.run_left_speed_plot = build_speed_plot_card("FOC左轮速度")
                self.run_speed_curve_lt, self.run_speed_curve_la, self.run_speed_curve_lvq = build_speed_curves(
                    self.run_left_speed_plot,
                    theme["speed_lt"],
                    theme["speed_la"],
                )
                run_speed_plots_row_layout.addWidget(run_left_speed_card, 1)

                run_right_speed_card, self.run_right_speed_plot = build_speed_plot_card("FOC右轮速度")
                self.run_speed_curve_rt, self.run_speed_curve_ra, self.run_speed_curve_rvq = build_speed_curves(
                    self.run_right_speed_plot,
                    theme["speed_rt"],
                    theme["speed_ra"],
                )
                run_speed_plots_row_layout.addWidget(run_right_speed_card, 1)

                scroll_layout.addWidget(run_speed_plots_row)

                run_gyro3d_card, self.run_gyro_3d_label, self.run_gyro_3d_stats_label = build_gyro3d_card("车身姿态 3D", show_reset=True)
                scroll_layout.addWidget(run_gyro3d_card)

                run_error_card, self.run_error_plot = build_error_plot_card("位置误差曲线")
                self.run_error_curve = self.run_error_plot.plot(name="白线位置", pen=pg.mkPen(theme["error_line"], width=2.2))
                scroll_layout.addWidget(run_error_card)

            if page_id == _BOARD_PAGE_NEG:
                neg_runtime_card = QtWidgets.QFrame()
                neg_runtime_card.setStyleSheet("background:#ffffff;border:1px solid #dbe3ee;border-radius:10px;")
                neg_runtime_layout = QtWidgets.QVBoxLayout(neg_runtime_card)
                neg_runtime_layout.setContentsMargins(8, 8, 8, 8)
                neg_runtime_layout.setSpacing(6)

                neg_runtime_title = QtWidgets.QLabel("运行时负压 ESC")
                neg_runtime_title.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_ui']};")
                neg_runtime_layout.addWidget(neg_runtime_title)

                neg_runtime_btn_row = QtWidgets.QHBoxLayout()
                neg_runtime_btn_row.setContentsMargins(0, 0, 0, 0)
                neg_runtime_btn_row.setSpacing(8)
                self.btn_np_on = QtWidgets.QPushButton("负压开启")
                self.btn_np_off = QtWidgets.QPushButton("负压关闭")
                self.btn_np_on.clicked.connect(self.send_np_on)
                self.btn_np_off.clicked.connect(self.send_np_off)
                neg_runtime_btn_row.addWidget(self.btn_np_on)
                neg_runtime_btn_row.addWidget(self.btn_np_off)
                neg_runtime_btn_row.addStretch(1)
                neg_runtime_layout.addLayout(neg_runtime_btn_row)

                self.neg_runtime_status_label = QtWidgets.QLabel("状态: 使能=-- 状态=-- 工作=-- 油门=-- 脉宽=--")
                self.neg_runtime_status_label.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_data']};")
                neg_runtime_layout.addWidget(self.neg_runtime_status_label)
                scroll_layout.addWidget(neg_runtime_card)

            if page_id == _BOARD_PAGE_GRAY:
                calib_panel = QtWidgets.QFrame()
                calib_panel.setStyleSheet(f"background:{theme['bg_panel']};border:1px solid {theme['border']};border-radius:10px;")
                calib_layout = QtWidgets.QVBoxLayout(calib_panel)
                calib_layout.setContentsMargins(8, 8, 8, 8)
                calib_layout.setSpacing(6)

                calib_btn_row = QtWidgets.QHBoxLayout()
                self.btn_calib_start = QtWidgets.QPushButton("开始校准")
                self.btn_calib_stop = QtWidgets.QPushButton("停止校准")
                self.btn_calib_clear = QtWidgets.QPushButton("清空校准")
                self.btn_calib_start.clicked.connect(lambda: send_command(self.state, self.log_q, "CALIB:START"))
                self.btn_calib_stop.clicked.connect(lambda: send_command(self.state, self.log_q, "CALIB:STOP"))
                self.btn_calib_clear.clicked.connect(lambda: send_command(self.state, self.log_q, "CALIB:CLEAR"))
                calib_btn_row.addWidget(self.btn_calib_start)
                calib_btn_row.addWidget(self.btn_calib_stop)
                calib_btn_row.addWidget(self.btn_calib_clear)
                calib_btn_row.addStretch(1)
                calib_layout.addLayout(calib_btn_row)

                self.calib_status_label = QtWidgets.QLabel("状态: --")
                self.calib_status_label.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_data']};")
                calib_layout.addWidget(self.calib_status_label)

                self.calib_data_label = QtWidgets.QLabel("min/max: --")
                self.calib_data_label.setStyleSheet(f"color:{theme['text_muted']};font:11px {theme['font_data']};")
                self.calib_data_label.setWordWrap(True)
                calib_layout.addWidget(self.calib_data_label)

                self.calib_raw_label = QtWidgets.QLabel("ADC raw: --")
                self.calib_raw_label.setStyleSheet(f"color:{theme['text_primary']};font:11px {theme['font_data']};")
                self.calib_raw_label.setWordWrap(True)
                calib_layout.addWidget(self.calib_raw_label)
                scroll_layout.addWidget(calib_panel)

            if page_id == _BOARD_PAGE_LINE:
                sensor_panel = QtWidgets.QFrame()
                sensor_panel.setStyleSheet(f"background:{theme['bg_panel']};border:1px solid {theme['border']};border-radius:10px;")
                sensor_panel_layout = QtWidgets.QVBoxLayout(sensor_panel)
                sensor_panel_layout.setContentsMargins(8, 8, 8, 8)
                sensor_panel_layout.setSpacing(8)

                self.line_debug_obs_speed_label = QtWidgets.QLabel("观测轮速(基于BAS_SPD+差速)  左=--   右=--")
                self.line_debug_obs_speed_label.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_data']};")
                sensor_panel_layout.addWidget(self.line_debug_obs_speed_label)

                self.sensors_plot_widget = pg.PlotWidget()
                self.sensors_plot_widget.setBackground(theme["plot_bg"])
                self.sensors_plot_widget.showGrid(x=True, y=True, alpha=0.20)
                self.sensors_plot_widget.getPlotItem().hideAxis('bottom')
                self.sensors_plot_widget.getPlotItem().getAxis("left").setTextPen(pg.mkColor(theme["plot_axis"]))
                self.sensors_plot_widget.getPlotItem().getAxis("left").setPen(pg.mkPen(theme["plot_border"]))
                self.sensors_plot_widget.getPlotItem().setLabel("left", "时间倒推行数 (0 为最新)", color=theme["plot_axis"])
                self.sensors_plot_widget.getPlotItem().vb.setBorder(pg.mkPen(theme["plot_border"]))

                self.waterfall_image = pg.ImageItem()
                self.waterfall_image.setLookupTable(self.waterfall_lut)
                self.sensors_plot_widget.addItem(self.waterfall_image)
                self.waterfall_error_curve = self.sensors_plot_widget.plot(pen=pg.mkPen(theme["waterfall_error"], width=1.8))
                sensor_panel_layout.addWidget(self.sensors_plot_widget, 2)

                error_card, self.error_plot = build_error_plot_card("位置误差曲线")
                self.error_curve = self.error_plot.plot(name="白线位置", pen=pg.mkPen(theme["error_line"], width=2.2))
                sensor_panel_layout.addWidget(error_card, 1)

                scroll_layout.addWidget(sensor_panel, 1)

            if page_id == _BOARD_PAGE_IMU:
                imu_gyro3d_card, self.imu_gyro_3d_label, self.imu_gyro_3d_stats_label = build_gyro3d_card("车身姿态 3D", show_reset=True)
                scroll_layout.addWidget(imu_gyro3d_card)

            scroll_layout.addStretch()
            scroll_area.setWidget(scroll_content)
            page_layout.addWidget(scroll_area)
            
            tab_title = group_name.split()[0]
            idx = tabs.addTab(page_tab, tab_title)
            self.tab_param_indices[page_id] = idx

        self.left_speed_plot = self.foc_left_speed_plot
        self.right_speed_plot = self.foc_right_speed_plot
        self.speed_curve_lt = self.foc_speed_curve_lt
        self.speed_curve_rt = self.foc_speed_curve_rt
        self.speed_curve_la = self.foc_speed_curve_la
        self.speed_curve_ra = self.foc_speed_curve_ra

        elec_angle_card, self.elec_wheel_label, self.elec_wheel_stats_label = build_elec_wheel_card("电角度双轮可视化")
        foc_scroll_layout.addWidget(elec_angle_card, 0)
        foc_scroll_layout.addStretch(1)

        log_card = QtWidgets.QFrame()
        log_card.setStyleSheet(f"background:{theme['bg_panel']};border:1px solid {theme['border']};border-radius:12px;")
        log_layout = QtWidgets.QVBoxLayout(log_card)
        log_layout.setContentsMargins(10, 10, 10, 10)
        log_layout.setSpacing(8)
        log_title = QtWidgets.QLabel("运行日志")
        log_title.setStyleSheet(f"color:{theme['text_primary']};font:600 12px {theme['font_ui']};")

        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumBlockCount(400)
        log_layout.addWidget(log_title)
        log_layout.addWidget(self.log_text, 1)
        left_column_layout.addWidget(log_card, 1)

        tabs.currentChanged.connect(self.on_tab_changed)
        tabs.tabBarClicked.connect(self.on_tab_clicked)
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Space), self.window, activated=self.send_pause_all)

    def _battery_indicator_color(self, percent: Optional[int], low: Optional[int], critical: Optional[int]) -> str:
        if critical:
            return "#e76f51"
        if low:
            return "#f4a261"
        if percent is None:
            return "#6c757d"
        if percent <= 20:
            return "#e9c46a"
        return "#2a9d8f"

    def _build_battery_icon(self, percent: Optional[int], low: Optional[int], critical: Optional[int]):
        QtCore = self.QtCore
        QtGui = self.QtGui

        width, height = 64, 30
        body = QtCore.QRectF(2.0, 5.0, 52.0, 20.0)
        cap = QtCore.QRectF(55.0, 10.0, 7.0, 10.0)
        inner_margin = 3.0

        if percent is None:
            pct = 0
        else:
            pct = max(0, min(100, int(percent)))

        fill_color = self._battery_indicator_color(percent, low, critical)
        pix = QtGui.QPixmap(width, height)
        pix.fill(QtCore.Qt.transparent)

        painter = QtGui.QPainter(pix)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        outline_pen = QtGui.QPen(QtGui.QColor("#64748b"), 1.5)
        painter.setPen(outline_pen)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawRoundedRect(body, 3.0, 3.0)
        painter.drawRoundedRect(cap, 1.5, 1.5)

        inner = QtCore.QRectF(
            body.x() + inner_margin,
            body.y() + inner_margin,
            body.width() - inner_margin * 2.0,
            body.height() - inner_margin * 2.0,
        )
        fill_w = int(inner.width() * pct / 100.0)
        if pct > 0 and fill_w < 2:
            fill_w = 2

        if fill_w > 0:
            fill_rect = QtCore.QRectF(inner.x(), inner.y(), float(fill_w), inner.height())
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor(fill_color))
            painter.drawRoundedRect(fill_rect, 2.0, 2.0)

        painter.end()
        return pix

    def _project_gyro_point(self, x: float, y: float, z: float, center_x: float, center_y: float, scale: float):
        QtCore = self.QtCore

        # 轻量级3D->2D投影（固定观察角）
        yaw = -0.75
        pitch = 0.48
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)

        x1 = cos_yaw * x - sin_yaw * y
        y1 = sin_yaw * x + cos_yaw * y
        z2 = cos_pitch * z - sin_pitch * y1
        y2 = sin_pitch * z + cos_pitch * y1

        screen_x = center_x + x1 * scale
        screen_y = center_y - y2 * scale
        _ = z2
        return QtCore.QPointF(screen_x, screen_y)

    def _build_gyro_3d_icon(self, roll: Optional[float], pitch: Optional[float], yaw_deg: Optional[float]):
        QtCore = self.QtCore
        QtGui = self.QtGui

        width, height = 300, 190
        center_x, center_y = 140.0, 100.0
        scale = 68.0
        axis_len = 0.95

        pix = QtGui.QPixmap(width, height)
        pix.fill(QtGui.QColor(self.theme["plot_bg"]))

        painter = QtGui.QPainter(pix)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        origin = self._project_gyro_point(0.0, 0.0, 0.0, center_x, center_y, scale)

        # 先画世界坐标轴，作为姿态参考
        axis_specs = [
            ((axis_len, 0.0, 0.0), "#fca5a5", "X"),
            ((0.0, axis_len, 0.0), "#86efac", "Y"),
            ((0.0, 0.0, axis_len), "#93c5fd", "Z"),
        ]
        for (ax, ay, az), color, name in axis_specs:
            end = self._project_gyro_point(ax, ay, az, center_x, center_y, scale)
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 1.4))
            painter.drawLine(origin, end)
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 1.0))
            painter.drawText(end.x() + 4.0, end.y() - 4.0, name)

        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor("#64748b"))
        painter.drawEllipse(origin, 3.0, 3.0)

        has_att = (roll is not None) and (pitch is not None) and (yaw_deg is not None)
        if has_att:
            rr = math.radians(float(roll))
            pr = math.radians(float(pitch))
            yr = math.radians(float(yaw_deg))

            cr, sr = math.cos(rr), math.sin(rr)
            cp, sp = math.cos(pr), math.sin(pr)
            cy, sy = math.cos(yr), math.sin(yr)

            # R = Rz(yaw) * Ry(pitch) * Rx(roll)
            r00 = cy * cp
            r01 = cy * sp * sr - sy * cr
            r02 = cy * sp * cr + sy * sr
            r10 = sy * cp
            r11 = sy * sp * sr + cy * cr
            r12 = sy * sp * cr - cy * sr
            r20 = -sp
            r21 = cp * sr
            r22 = cp * cr

            def rot(x: float, y: float, z: float):
                return (
                    r00 * x + r01 * y + r02 * z,
                    r10 * x + r11 * y + r12 * z,
                    r20 * x + r21 * y + r22 * z,
                )

            # 简化车体框架（前后长、左右宽、上下高）
            L, W, H = 0.95, 0.42, 0.22
            body_vertices = [
                ( L,  W,  H),
                ( L, -W,  H),
                ( L, -W, -H),
                ( L,  W, -H),
                (-L,  W,  H),
                (-L, -W,  H),
                (-L, -W, -H),
                (-L,  W, -H),
            ]
            rotated = [rot(x, y, z) for (x, y, z) in body_vertices]
            pts2d = [self._project_gyro_point(x, y, z, center_x, center_y, scale) for (x, y, z) in rotated]

            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),
                (4, 5), (5, 6), (6, 7), (7, 4),
                (0, 4), (1, 5), (2, 6), (3, 7),
            ]

            painter.setPen(QtGui.QPen(QtGui.QColor("#334155"), 2.0))
            for a, b in edges:
                painter.drawLine(pts2d[a], pts2d[b])

            # 前向箭头，强调车头方向
            nose = rot(L + 0.35, 0.0, 0.0)
            nose_pt = self._project_gyro_point(nose[0], nose[1], nose[2], center_x, center_y, scale)
            center_pt = self._project_gyro_point(0.0, 0.0, 0.0, center_x, center_y, scale)
            painter.setPen(QtGui.QPen(QtGui.QColor("#2563eb"), 2.6))
            painter.drawLine(center_pt, nose_pt)
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor("#2563eb"))
            painter.drawEllipse(nose_pt, 4.2, 4.2)

            # 车体局部轴
            lx = rot(0.45, 0.0, 0.0)
            ly = rot(0.0, 0.45, 0.0)
            lz = rot(0.0, 0.0, 0.45)
            lx_pt = self._project_gyro_point(lx[0], lx[1], lx[2], center_x, center_y, scale)
            ly_pt = self._project_gyro_point(ly[0], ly[1], ly[2], center_x, center_y, scale)
            lz_pt = self._project_gyro_point(lz[0], lz[1], lz[2], center_x, center_y, scale)
            painter.setPen(QtGui.QPen(QtGui.QColor("#ef4444"), 1.8))
            painter.drawLine(center_pt, lx_pt)
            painter.setPen(QtGui.QPen(QtGui.QColor("#16a34a"), 1.8))
            painter.drawLine(center_pt, ly_pt)
            painter.setPen(QtGui.QPen(QtGui.QColor("#0284c7"), 1.8))
            painter.drawLine(center_pt, lz_pt)
        else:
            # 无姿态数据时给占位框
            painter.setPen(QtGui.QPen(QtGui.QColor("#94a3b8"), 1.5, QtCore.Qt.DashLine))
            painter.drawRoundedRect(QtCore.QRectF(64.0, 55.0, 110.0, 70.0), 6.0, 6.0)
            painter.setPen(QtGui.QPen(QtGui.QColor("#64748b"), 1.0))
            painter.drawText(84.0, 92.0, "ATTITUDE --")

        painter.end()
        return pix

    def _build_elec_wheels_icon(self, left_deg: Optional[float], right_deg: Optional[float]):
        QtCore = self.QtCore
        QtGui = self.QtGui

        width, height = 420, 170
        pix = QtGui.QPixmap(width, height)
        pix.fill(QtGui.QColor(self.theme["plot_bg"]))

        painter = QtGui.QPainter(pix)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        wheel_radius = 54.0
        hubs_radius = 14.0
        pointer_len = wheel_radius - 8.0
        centers = [(125.0, 86.0), (295.0, 86.0)]
        wheel_data = [
            ("左轮", left_deg, QtGui.QColor("#2563eb")),
            ("右轮", right_deg, QtGui.QColor("#db2777")),
        ]

        for (cx, cy), (name, degree, pointer_color) in zip(centers, wheel_data):
            center = QtCore.QPointF(cx, cy)

            painter.setPen(QtGui.QPen(QtGui.QColor("#334155"), 3.0))
            painter.setBrush(QtGui.QColor("#f8fafc"))
            painter.drawEllipse(center, wheel_radius, wheel_radius)

            painter.setPen(QtGui.QPen(QtGui.QColor("#cbd5e1"), 1.0, QtCore.Qt.DashLine))
            painter.drawLine(
                QtCore.QPointF(cx - wheel_radius, cy),
                QtCore.QPointF(cx + wheel_radius, cy),
            )
            painter.drawLine(
                QtCore.QPointF(cx, cy - wheel_radius),
                QtCore.QPointF(cx, cy + wheel_radius),
            )

            painter.setPen(QtGui.QPen(QtGui.QColor("#64748b"), 1.2))
            painter.setBrush(QtGui.QColor("#ffffff"))
            painter.drawEllipse(center, hubs_radius, hubs_radius)

            if (degree is not None) and math.isfinite(float(degree)):
                angle_deg = float(degree) % 360.0
                # 0deg 设为正上方，顺时针旋转。
                angle_rad = math.radians(angle_deg - 90.0)
                px = cx + math.cos(angle_rad) * pointer_len
                py = cy + math.sin(angle_rad) * pointer_len
                pointer_end = QtCore.QPointF(px, py)

                painter.setPen(QtGui.QPen(pointer_color, 3.0))
                painter.drawLine(center, pointer_end)
                painter.setBrush(pointer_color)
                painter.setPen(QtCore.Qt.NoPen)
                painter.drawEllipse(pointer_end, 4.0, 4.0)
                text = f"{name}  {angle_deg:6.1f} deg"
            else:
                painter.setPen(QtGui.QPen(QtGui.QColor("#94a3b8"), 2.0, QtCore.Qt.DotLine))
                painter.drawLine(center, QtCore.QPointF(cx, cy - pointer_len))
                text = f"{name}  --"

            painter.setPen(QtGui.QPen(QtGui.QColor("#475569"), 1.0))
            painter.drawText(QtCore.QRectF(cx - 74.0, cy + 62.0, 148.0, 22.0), QtCore.Qt.AlignCenter, text)

        painter.end()
        return pix

    def _on_close_event(self, event) -> None:
        self.shutdown()
        event.accept()

    def clear_log_panel(self) -> None:
        while True:
            try:
                self.log_q.get_nowait()
            except queue.Empty:
                break
        self.pending_logs.clear()
        self.log_text.clear()

    def drain_queues(self) -> None:
        while True:
            try:
                item = self.data_q.get_nowait()
            except queue.Empty:
                break
            self.model.apply_data(item)
            if item.frame_type == _FRAME_TYPE_FAST:
                self.update_run_control_from_telemetry()
                self.update_run_timer(item.t)
                self.record_speed_sample(item.t)
            elif item.frame_type == _FRAME_TYPE_ROUTE:
                self.update_route_progress_log(item.t)
            elif item.frame_type in (_FRAME_TYPE_GRAY, _FRAME_TYPE_GRAY_RAW):
                self.record_gray_sample()
            self.update_stop_reason_log_once()

        while True:
            try:
                item = self.text_q.get_nowait()
            except queue.Empty:
                break

            if item.control_ack is not None:
                ack_seq, ack_opcode, ack_status = item.control_ack
                self.handle_control_ack(ack_seq, ack_opcode, ack_status)
                continue

            if item.param:
                key, val_str = item.param
                key_norm = self.normalize_param_key(key)

                if key_norm == "PAGE":
                    page_norm = self.normalize_page_name(val_str)
                    prev_page = self.current_board_page
                    self.apply_board_page_to_ui(page_norm)
                    self.pending_logs.append(f"[页面] 板端页面 -> {page_norm}")
                    if page_norm != prev_page:
                        self.query_page_params(page_norm)
                    continue

                if key_norm == "PAGEPARAM":
                    page_norm = self.normalize_page_name(val_str)
                    self.pending_logs.append(f"[页面] 参数分页开始: {page_norm}")
                    continue

                if key_norm == "PAGEPARAM_END":
                    page_norm = self.normalize_page_name(val_str)
                    self.pending_logs.append(f"[页面] 参数分页结束: {page_norm}")
                    continue

                if key_norm in self.param_widgets:
                    try:
                        val = float(val_str)
                        self.param_widgets[key_norm].setValue(val)
                        if not hasattr(self, "param_last_known"):
                            self.param_last_known = {}

                        self.param_last_known[key_norm] = val

                        run_key, left_key, right_key = self.get_foc_test_param_keys()
                        if key_norm == run_key and hasattr(self, "foc_test_run_sb"):
                            self.foc_test_run_sb.setValue(float(round(val)))
                        elif key_norm == left_key and hasattr(self, "foc_test_left_rpm_sb"):
                            self.foc_test_left_rpm_sb.setValue(val)
                        elif key_norm == right_key and hasattr(self, "foc_test_right_rpm_sb"):
                            self.foc_test_right_rpm_sb.setValue(val)
                    except ValueError:
                        pass
                elif key_norm == "ACK" or key_norm == "ERR":
                    ack_key = self.normalize_param_key(val_str)
                    name = getattr(self, "param_name_map", {}).get(ack_key, val_str)
                    prefix = "[参数] 成功:" if key_norm == "ACK" else "[错误] 失败:"
                    self.pending_logs.append(f"{prefix} {name}")
                continue

            self.model.apply_text(item)

        while True:
            try:
                line = self.log_q.get_nowait()
            except queue.Empty:
                break
            self.pending_logs.append(line)

    @staticmethod
    def format_elapsed(seconds: Optional[float]) -> str:
        if seconds is None:
            return "--"
        if seconds < 0.0:
            seconds = 0.0
        total_ms = int(seconds * 1000.0)
        minutes = total_ms // 60000
        sec = (total_ms % 60000) // 1000
        ms = total_ms % 1000
        return f"{minutes:02d}:{sec:02d}.{ms:03d}"

    def reset_run_timer_state(self) -> None:
        self.last_run_enable_state = None
        self.run_timer_active = False
        self.run_timer_start_t = 0.0
        self.run_timer_elapsed_s = 0.0
        self.run_timer_last_s = None
        self.run_timer_start_route_idx = None
        self.run_timer_end_route_idx = None
        self.last_route_index_seen = None
        self.last_turn_hint = None
        self.last_logged_stop_reason = 0
        self.last_junction_lock_cnt = 0
        self.last_element_enter_cnt = 0
        self.last_junction_enter_cnt = 0
    def reset_foc_test_page_state(self) -> None:
        if hasattr(self, "foc_test_run_sb"):
            self.foc_test_run_sb.setValue(0.0)
        if hasattr(self, "foc_test_left_rpm_sb"):
            self.foc_test_left_rpm_sb.setValue(0.0)
        if hasattr(self, "foc_test_right_rpm_sb"):
            self.foc_test_right_rpm_sb.setValue(0.0)
        self.update_run_control_buttons()

    def reset_connection_page_state(self) -> None:
        self.model.reset()
        self.clear_speed_history()
        self.reset_run_timer_state()
        self.reset_run_control_state()
        self.reset_foc_test_page_state()
        self.gray_stream_enabled = False
        self.gray_raw_enabled = False

    def update_stop_reason_log_once(self) -> None:
        run_now = self.model.run_enable
        stop_reason = self.model.stop_reason
        if run_now is None or stop_reason is None:
            return

        current_reason = int(stop_reason) if int(run_now) == 0 else 0
        if (current_reason != self.last_logged_stop_reason) and (current_reason == 1):
            self.pending_logs.append("[事件] 停机原因: 丢线")
        self.last_logged_stop_reason = current_reason

    def update_route_progress_log(self, sample_t: float) -> None:
        _ = sample_t
        run_now = self.model.run_enable
        route_index = self.model.route_index
        pending_turn = self.model.pending_turn

        if (pending_turn is not None) and (1 <= int(pending_turn) <= 21):
            self.last_turn_hint = int(pending_turn)

        if run_now != 1 or route_index is None:
            if run_now != 1:
                self.last_route_index_seen = None
                self.last_turn_hint = None
                self.last_junction_lock_cnt = 0
                self.last_element_enter_cnt = 0
                self.last_junction_enter_cnt = 0
            return

        if self.last_route_index_seen is None:
            self.last_route_index_seen = route_index
            self.last_junction_lock_cnt = self.model.junction_lock_cnt or 0
            self.last_element_enter_cnt = self.model.element_enter_cnt or 0
            self.last_junction_enter_cnt = self.model.junction_enter_cnt or 0
            return

        jlc = self.model.junction_lock_cnt
        if (jlc is not None) and (jlc != self.last_junction_lock_cnt):
            self.pending_logs.append(f"[轨迹] 元素锁定完成")
            self.last_junction_lock_cnt = jlc

        eec = self.model.element_enter_cnt
        if (eec is not None) and (eec != self.last_element_enter_cnt):
            self.pending_logs.append(f"[轨迹] 进入元素")
            self.last_element_enter_cnt = eec

        jec = self.model.junction_enter_cnt
        if (jec is not None) and (jec != self.last_junction_enter_cnt):
            self.pending_logs.append(f"[轨迹] 进入路口")
            self.last_junction_enter_cnt = jec

        if route_index > self.last_route_index_seen:
            if self.last_turn_hint is not None:
                completed_turn_text = turn_to_text(self.last_turn_hint)
            else:
                completed_turn_text = turn_to_text(pending_turn)
            for completed_idx in range(self.last_route_index_seen, route_index):
                self.pending_logs.append(f"[轨迹] 第{completed_idx + 1}条已完成（{completed_turn_text}）")
            winsound.Beep(1000, 200)
            self.last_turn_hint = None
        elif route_index < self.last_route_index_seen:
            self.pending_logs.append(
                f"[轨迹] 路线索引回退 {self.last_route_index_seen}->{route_index}，已重新同步"
            )
            self.last_turn_hint = None

        self.last_route_index_seen = route_index

    def update_run_timer(self, sample_t: float) -> None:
        run_now = self.model.run_enable
        if run_now is None:
            return

        if (self.last_run_enable_state != 1) and (run_now == 1):
            self.run_timer_active = True
            self.run_timer_start_t = sample_t
            self.run_timer_elapsed_s = 0.0
            self.run_timer_start_route_idx = self.model.route_index
            self.run_timer_end_route_idx = None

        if self.run_timer_active and (run_now == 1):
            self.run_timer_elapsed_s = max(0.0, sample_t - self.run_timer_start_t)

        if (self.last_run_enable_state == 1) and (run_now == 0) and self.run_timer_active:
            self.run_timer_elapsed_s = max(0.0, sample_t - self.run_timer_start_t)
            self.run_timer_last_s = self.run_timer_elapsed_s
            self.run_timer_end_route_idx = self.model.route_index
            self.run_timer_active = False

        self.last_run_enable_state = run_now

    def clear_speed_history(self) -> None:
        self.speed_time_hist.clear()
        self.speed_lt_hist.clear()
        self.speed_rt_hist.clear()
        self.speed_la_hist.clear()
        self.speed_ra_hist.clear()
        if hasattr(self, 'speed_lvq_hist'):
            self.speed_lvq_hist.clear()
            self.speed_rvq_hist.clear()
        if hasattr(self, 'error_hist'):
            self.error_hist.clear()
        
        self.waterfall_data.fill(0)
        self.line_pos_hist.clear()
        if hasattr(self, 'waterfall_image'):
            self.waterfall_image.setImage(self.waterfall_data)
            self.waterfall_error_curve.setData([], [])

        self.speed_curve_lt.setData([], [])
        self.speed_curve_rt.setData([], [])
        self.speed_curve_la.setData([], [])
        self.speed_curve_ra.setData([], [])
        if hasattr(self, 'foc_speed_curve_lt'):
            self.foc_speed_curve_lt.setData([], [])
            self.foc_speed_curve_rt.setData([], [])
            self.foc_speed_curve_la.setData([], [])
            self.foc_speed_curve_ra.setData([], [])
        if hasattr(self, 'run_speed_curve_lt'):
            self.run_speed_curve_lt.setData([], [])
            self.run_speed_curve_rt.setData([], [])
            self.run_speed_curve_la.setData([], [])
            self.run_speed_curve_ra.setData([], [])
        if hasattr(self, 'run_error_curve'):
            self.run_error_curve.setData([], [])
        if hasattr(self, 'error_curve'):
            self.error_curve.setData([], [])

        self.left_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
        self.right_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
        if hasattr(self, 'foc_left_speed_plot'):
            self.foc_left_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            self.foc_right_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
        if hasattr(self, 'run_left_speed_plot'):
            self.run_left_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            self.run_right_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
        if hasattr(self, 'run_error_plot'):
            self.run_error_plot.setYRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            self.run_error_plot.setXRange(-50.0, 50.0, padding=0.0)
        if hasattr(self, 'error_plot'):
            self.error_plot.setYRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            self.error_plot.setXRange(-50.0, 50.0, padding=0.0)

        self.left_speed_plot.setYRange(-100.0, 100.0, padding=0.0)
        self.right_speed_plot.setYRange(-100.0, 100.0, padding=0.0)
        if hasattr(self, 'foc_left_speed_plot'):
            self.foc_left_speed_plot.setYRange(-100.0, 100.0, padding=0.0)
            self.foc_right_speed_plot.setYRange(-100.0, 100.0, padding=0.0)
        if hasattr(self, 'run_left_speed_plot'):
            self.run_left_speed_plot.setYRange(-100.0, 100.0, padding=0.0)
            self.run_right_speed_plot.setYRange(-100.0, 100.0, padding=0.0)
        if hasattr(self, 'run_error_plot'):
            half_width = (self.cols - 1) / 2.0
            self.run_error_plot.setXRange(-half_width - 1.0, half_width + 1.0, padding=0.0)
        self.refresh_elec_plot()

    def record_speed_sample(self, sample_t: float) -> None:
        if all(
            value is None
            for value in (
                self.model.speed_l_target,
                self.model.speed_r_target,
                self.model.speed_l_actual,
                self.model.speed_r_actual,
                self.model.left_vq,
                self.model.right_vq,
            )
        ):
            return

        self.speed_time_hist.append(sample_t)
        self.speed_lt_hist.append(float("nan") if self.model.speed_l_target is None else self.model.speed_l_target)
        self.speed_rt_hist.append(float("nan") if self.model.speed_r_target is None else self.model.speed_r_target)
        self.speed_la_hist.append(float("nan") if self.model.speed_l_actual is None else self.model.speed_l_actual)
        self.speed_ra_hist.append(float("nan") if self.model.speed_r_actual is None else self.model.speed_r_actual)
        self.speed_lvq_hist.append(float("nan") if self.model.left_vq is None else self.model.left_vq)
        self.speed_rvq_hist.append(float("nan") if self.model.right_vq is None else self.model.right_vq)

        self.error_hist.append(float("nan") if self.model.line_pos is None else self.model.line_pos)

        keep_after_t = sample_t - self.speed_plot_window_s
        while self.speed_time_hist and self.speed_time_hist[0] < keep_after_t:
            self.speed_time_hist.popleft()
            self.speed_lt_hist.popleft()
            self.speed_rt_hist.popleft()
            self.speed_la_hist.popleft()
            self.speed_ra_hist.popleft()
            self.speed_lvq_hist.popleft()
            self.speed_rvq_hist.popleft()
            self.error_hist.popleft()

    def record_gray_sample(self) -> None:
        self.waterfall_data = np.roll(self.waterfall_data, -1, axis=1)
        for i in range(self.cols):
            v = int(self.model.sensors[i])
            if v > 255:
                v = v >> 4
            self.waterfall_data[i, -1] = min(max(v, 0), 255)

        self.line_pos_hist.append(float("nan") if self.model.line_pos is None else self.model.line_pos)
        if len(self.line_pos_hist) > self.waterfall_hist_len:
            self.line_pos_hist.popleft()

    def refresh_speed_plot(self) -> None:
        if getattr(self, "chk_pause_plot", None) and self.chk_pause_plot.isChecked():
            return
            
        if not self.speed_time_hist:
            self.left_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            self.right_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            if hasattr(self, 'foc_left_speed_plot'):
                self.foc_left_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
                self.foc_right_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            if hasattr(self, 'run_left_speed_plot'):
                self.run_left_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
                self.run_right_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            if hasattr(self, 'run_error_plot'):
                self.run_error_plot.setYRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            self.error_plot.setYRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            return

        latest_t = self.speed_time_hist[-1]
        x_values = np.asarray([timestamp - latest_t for timestamp in self.speed_time_hist], dtype=float)
        speed_lt_display = self.smooth_speed_series(self.speed_lt_hist)
        speed_rt_display = self.smooth_speed_series(self.speed_rt_hist)
        speed_la_display = self.smooth_speed_series(self.speed_la_hist)
        speed_ra_display = self.smooth_speed_series(self.speed_ra_hist)
        speed_lvq_scaled = np.asarray(self.speed_lvq_hist, dtype=float) * 1000.0
        speed_rvq_scaled = np.asarray(self.speed_rvq_hist, dtype=float) * 1000.0

        self.speed_curve_lt.setData(x_values, speed_lt_display, connect="finite")
        self.speed_curve_rt.setData(x_values, speed_rt_display, connect="finite")
        self.speed_curve_la.setData(x_values, speed_la_display, connect="finite")
        self.speed_curve_ra.setData(x_values, speed_ra_display, connect="finite")
        if hasattr(self, 'foc_speed_curve_lt'):
            self.foc_speed_curve_lt.setData(x_values, speed_lt_display, connect="finite")
            self.foc_speed_curve_rt.setData(x_values, speed_rt_display, connect="finite")
            self.foc_speed_curve_la.setData(x_values, speed_la_display, connect="finite")
            self.foc_speed_curve_ra.setData(x_values, speed_ra_display, connect="finite")
            self.foc_speed_curve_lvq.setData(x_values, speed_lvq_scaled, connect="finite")
            self.foc_speed_curve_rvq.setData(x_values, speed_rvq_scaled, connect="finite")
        if hasattr(self, 'run_speed_curve_lt'):
            self.run_speed_curve_lt.setData(x_values, speed_lt_display, connect="finite")
            self.run_speed_curve_rt.setData(x_values, speed_rt_display, connect="finite")
            self.run_speed_curve_la.setData(x_values, speed_la_display, connect="finite")
            self.run_speed_curve_ra.setData(x_values, speed_ra_display, connect="finite")
            self.run_speed_curve_lvq.setData(x_values, speed_lvq_scaled, connect="finite")
            self.run_speed_curve_rvq.setData(x_values, speed_rvq_scaled, connect="finite")
        if hasattr(self, 'run_error_curve'):
            self.run_error_curve.setData(list(self.error_hist), x_values)
        self.error_curve.setData(list(self.error_hist), x_values)
        
        self.left_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
        self.right_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
        if hasattr(self, 'foc_left_speed_plot'):
            self.foc_left_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            self.foc_right_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
        if hasattr(self, 'run_left_speed_plot'):
            self.run_left_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
            self.run_right_speed_plot.setXRange(-self.speed_plot_window_s, 0.0, padding=0.02)
        if hasattr(self, 'run_error_plot'):
            self.run_error_plot.setYRange(-self.speed_plot_window_s, 0.0, padding=0.02)
        
        self.error_plot.setYRange(-self.speed_plot_window_s, 0.0, padding=0.02)
        
        self.update_speed_plot_y_range(self.left_speed_plot, speed_lt_display, speed_la_display)
        self.update_speed_plot_y_range(self.right_speed_plot, speed_rt_display, speed_ra_display)
        if hasattr(self, 'foc_left_speed_plot'):
            self.update_speed_plot_y_range(self.foc_left_speed_plot, speed_lt_display, speed_la_display)
            self.update_speed_plot_y_range(self.foc_right_speed_plot, speed_rt_display, speed_ra_display)
        if hasattr(self, 'run_left_speed_plot'):
            self.update_speed_plot_y_range(self.run_left_speed_plot, speed_lt_display, speed_la_display)
            self.update_speed_plot_y_range(self.run_right_speed_plot, speed_rt_display, speed_ra_display)
        
        # 误差图由于已经居中，我们将X轴固定为对称范围
        half_width = (self.cols - 1) / 2.0
        if hasattr(self.error_plot, 'setXRange'):
            self.error_plot.setXRange(-half_width - 1.0, half_width + 1.0, padding=0.0)
        if hasattr(self, 'run_error_plot'):
            self.run_error_plot.setXRange(-half_width - 1.0, half_width + 1.0, padding=0.0)

        if hasattr(self, 'waterfall_image') and not getattr(self, "chk_pause_plot", None).isChecked():
            self.waterfall_image.setImage(self.waterfall_data, autoLevels=False, levels=(0, 255))
            y_pts = np.arange(len(self.line_pos_hist))
            x_pts = [(val + (self.cols / 2.0)) if not math.isnan(val) else float("nan") for val in self.line_pos_hist]
            self.waterfall_error_curve.setData(x_pts, y_pts)
            self.sensors_plot_widget.setXRange(0, self.cols, padding=0.0)
            self.sensors_plot_widget.setYRange(max(0, len(self.line_pos_hist) - self.waterfall_hist_len), self.waterfall_hist_len, padding=0.0)

    def refresh_elec_plot(self) -> None:
        if getattr(self, "chk_pause_plot", None) and self.chk_pause_plot.isChecked():
            return

        self.elec_wheel_label.setPixmap(self._build_elec_wheels_icon(self.model.elec_l_deg, self.model.elec_r_deg))
        self.elec_wheel_stats_label.setText(
            f"左轮={fmt_float(self.model.elec_l_deg)} deg    右轮={fmt_float(self.model.elec_r_deg)} deg"
        )

    @staticmethod
    def _ema_filter_with_nan(values: np.ndarray, alpha: float) -> np.ndarray:
        filtered = np.full(values.shape, np.nan, dtype=float)
        last = float("nan")
        for idx, value in enumerate(values):
            if not np.isfinite(value):
                last = float("nan")
                continue

            if np.isfinite(last):
                last = last + alpha * (value - last)
            else:
                last = value
            filtered[idx] = last
        return filtered

    def smooth_speed_series(self, values: deque[float]) -> np.ndarray:
        series = np.asarray(values, dtype=float)
        if series.size < self.speed_curve_smooth_min_points:
            return series

        alpha = min(0.95, max(0.05, float(self.speed_curve_smooth_alpha)))
        forward = self._ema_filter_with_nan(series, alpha)
        # Forward + backward EMA keeps smoothness and avoids obvious phase lag.
        return self._ema_filter_with_nan(forward[::-1], alpha)[::-1]

    def update_speed_plot_y_range(
        self,
        plot_widget: object,
        *series: deque[float],
        min_span: float = 100.0,
        min_padding: float = 20.0,
        fallback_half_range: float = 100.0,
    ) -> None:
        finite_values = [value for values in series for value in values if math.isfinite(value)]
        if not finite_values:
            plot_widget.setYRange(-fallback_half_range, fallback_half_range, padding=0.0)
            return

        min_value = min(finite_values)
        max_value = max(finite_values)
        span = max(max_value - min_value, min_span)
        padding = max(span * 0.18, min_padding)
        center = (min_value + max_value) * 0.5
        half_range = span * 0.5 + padding
        plot_widget.setYRange(center - half_range, center + half_range, padding=0.0)

    def update_rates(self, now: float) -> None:
        dt = now - self.last_rate_sample_t
        if dt < 0.20:
            return

        with self.state.lock:
            raw_rx_count = self.state.raw_rx_count
            raw_text_count = self.state.raw_text_count

        self.rate_snapshot = RateSnapshot(
            raw_rx_hz=max(0.0, (raw_rx_count - self.last_raw_rx_count) / dt),
            ui_rx_hz=max(0.0, (self.model.rx_total - self.last_ui_rx_count) / dt),
            raw_text_hz=max(0.0, (raw_text_count - self.last_raw_text_count) / dt),
            ui_text_hz=max(0.0, (self.model.text_total - self.last_ui_text_count) / dt),
        )
        self.last_rate_sample_t = now
        self.last_raw_rx_count = raw_rx_count
        self.last_raw_text_count = raw_text_count
        self.last_ui_rx_count = self.model.rx_total
        self.last_ui_text_count = self.model.text_total

    def refresh_panels(self) -> None:
        with self.state.lock:
            addr = self.state.addr
            raw_rx_count = self.state.raw_rx_count
            raw_text_count = self.state.raw_text_count

        is_connected = addr is not None
        if self.was_connected and not is_connected:
            self.reset_connection_page_state()
        elif not self.was_connected and is_connected:
            push_log(self.log_q, "客户端已连接，延迟500ms后自动同步参数...")
            def delayed_sync():
                if self.state.conn is not None:
                    self.sync_current_tab_to_board(query_params=True)
            self.QtCore.QTimer.singleShot(500, delayed_sync)
        
        self.was_connected = is_connected

        if not is_connected:
            self.status_label.setText(
                f"监听中 {self.args.host}:{self.args.port}，等待设备连接   rx_ui={self.model.rx_total}({self.rate_snapshot.ui_rx_hz:5.1f}/s) text_ui={self.model.text_total}({self.rate_snapshot.ui_text_hz:4.1f}/s)"
            )
        else:
            self.status_label.setText(
                f"已连接 {addr[0]}:{addr[1]}   监听 {self.args.host}:{self.args.port}   rx_raw={raw_rx_count}({self.rate_snapshot.raw_rx_hz:5.1f}/s) rx_ui={self.model.rx_total}({self.rate_snapshot.ui_rx_hz:5.1f}/s)   text_raw={raw_text_count}({self.rate_snapshot.raw_text_hz:4.1f}/s) text_ui={self.model.text_total}({self.rate_snapshot.ui_text_hz:4.1f}/s)"
            )

        alarms: List[str] = []
        if not is_connected:
            alarms.append("TCP 已断开")
        if self.run_ctrl_pending_cmd is not None:
            alarms.append(f"等待{self.run_ctrl_pending_cmd}确认")
        if self.model.battery_critical:
            alarms.append("电池严重低压")
        elif self.model.battery_low:
            alarms.append("电池低压")
        if self.model.line_ready == 0:
            alarms.append("循迹未就绪")
        if self.model.pos_valid == 0 and self.model.run_enable:
            alarms.append("线位置丢失")
        if self.model.lost_line_cnt is not None and self.model.lost_line_cnt > 4:
            alarms.append(f"丢线 x{self.model.lost_line_cnt}")
        if (self.model.run_enable == 0) and (self.model.stop_reason is not None) and (self.model.stop_reason != 0):
            alarms.append(f"停机: {stop_reason_to_text(self.model.stop_reason)}")

        if alarms:
            self.alarm_label.setText(" | ".join(alarms))
            self.alarm_label.setStyleSheet(f"background:#fff1f2;color:#b42334;padding:8px;font:600 13px {self.theme['font_ui']};border-radius:6px;")
        else:
            self.alarm_label.setText("系统正常")
            self.alarm_label.setStyleSheet(f"background:#e8f6ef;color:#156f4b;padding:8px;font:600 13px {self.theme['font_ui']};border-radius:6px;")

        display_roll, display_pitch, display_yaw = self.get_display_attitude()

        self.value_labels["angle"].setText(f"横滚={fmt_float(display_roll)} 俯仰={fmt_float(display_pitch)} 航向={fmt_float(display_yaw)}")
        self.value_labels["gyro"].setText(f"gx={fmt_float(self.model.gx)} gy={fmt_float(self.model.gy)} gz={fmt_float(self.model.gz)}")

        gyro_pix = self._build_gyro_3d_icon(display_roll, display_pitch, display_yaw)
        if hasattr(self, "gyro_3d_label"):
            self.gyro_3d_label.setPixmap(gyro_pix)
        if hasattr(self, "imu_gyro_3d_label"):
            self.imu_gyro_3d_label.setPixmap(gyro_pix)
        if hasattr(self, "run_gyro_3d_label"):
            self.run_gyro_3d_label.setPixmap(gyro_pix)

        if (display_roll is None) or (display_pitch is None) or (display_yaw is None):
            gyro_stats_text = "roll=-- pitch=-- yaw=--"
        else:
            gyro_stats_text = f"roll={display_roll:6.1f} pitch={display_pitch:6.1f} yaw={display_yaw:6.1f}"

        if hasattr(self, "gyro_3d_stats_label"):
            self.gyro_3d_stats_label.setText(gyro_stats_text)
        if hasattr(self, "imu_gyro_3d_stats_label"):
            self.imu_gyro_3d_stats_label.setText(gyro_stats_text)
        if hasattr(self, "run_gyro_3d_stats_label"):
            self.run_gyro_3d_stats_label.setText(gyro_stats_text)

        if hasattr(self, "cpu0_gauge"):
            self.cpu0_gauge.set_value(self.model.cpu0_load)
        if hasattr(self, "cpu1_gauge"):
            self.cpu1_gauge.set_value(self.model.cpu1_load)

        speed_text = (
            f"左目标={fmt_float(self.model.speed_l_target)} 右目标={fmt_float(self.model.speed_r_target)} "
            f"左实际={fmt_float(self.model.speed_l_actual)} 右实际={fmt_float(self.model.speed_r_actual)}"
        )
        self.value_labels["speed"].setText(speed_text)
        if hasattr(self, 'foc_left_speed_stats_label'):
            self.foc_left_speed_stats_label.setText(
                f"目标={fmt_float(self.model.speed_l_target)}   实际={fmt_float(self.model.speed_l_actual)}"
            )
        if hasattr(self, 'foc_right_speed_stats_label'):
            self.foc_right_speed_stats_label.setText(
                f"目标={fmt_float(self.model.speed_r_target)}   实际={fmt_float(self.model.speed_r_actual)}"
            )
        if hasattr(self, 'run_wheel_speed_label'):
            self.run_wheel_speed_label.setText(f"左右轮速度: {speed_text}")
        if hasattr(self, 'run_odom_state_label'):
            odom_state_text = "计数中" if self.model.odom_count_enable == 1 else ("未计数" if self.model.odom_count_enable == 0 else "--")
            self.run_odom_state_label.setText(f"里程计状态: {odom_state_text}")
        if hasattr(self, 'run_odom_label'):
            self.run_odom_label.setText(
                f"轮里程: 左={fmt_float(self.model.odom_left_mm)} mm 右={fmt_float(self.model.odom_right_mm)} mm"
            )
        if hasattr(self, 'run_path_progress_label'):
            if self.model.route_index is None:
                route_text = f"路径进度: idx=-- 待执行={turn_to_text(self.model.pending_turn)} 转角={fmt_float(self.model.turn_yaw_deg)} 剩余路径=--"
            else:
                route_idx = max(0, int(self.model.route_index))
                total_hint = self.run_timer_end_route_idx
                if (self.model.stop_reason is not None) and (int(self.model.stop_reason) == 2):
                    remaining_text = "0"
                elif total_hint is None:
                    remaining_text = "--"
                else:
                    remaining_text = str(max(0, int(total_hint) - route_idx))
                route_text = f"路径进度: idx={route_idx} 待执行={turn_to_text(self.model.pending_turn)} 转角={fmt_float(self.model.turn_yaw_deg)} 剩余路径={remaining_text}"
            self.run_path_progress_label.setText(route_text)
        self.value_labels["elec"].setText(f"左电角={fmt_float(self.model.elec_l_deg)} 右电角={fmt_float(self.model.elec_r_deg)}")
        current_timer_s = self.run_timer_elapsed_s if self.run_timer_active else None
        self.timer_current_label.setText(self.format_elapsed(current_timer_s))
        self.timer_last_label.setText(f"上次 {self.format_elapsed(self.run_timer_last_s)}")

        battery_color = self._battery_indicator_color(self.model.battery_percent, self.model.battery_low, self.model.battery_critical)
        self.battery_icon_label.setPixmap(self._build_battery_icon(self.model.battery_percent, self.model.battery_low, self.model.battery_critical))
        battery_state = "CRIT" if self.model.battery_critical else ("LOW" if self.model.battery_low else "OK")
        if self.model.battery_percent is None:
            battery_text = f"--%  {fmt_float(self.model.battery_v)}V"
        else:
            battery_text = f"{int(self.model.battery_percent):3d}%  {fmt_float(self.model.battery_v)}V"
        self.battery_text_label.setStyleSheet(f"color:{battery_color};font:700 14px {self.theme['font_data']};")
        self.battery_text_label.setText(f"{battery_text}  {battery_state}")

        np_state = np_state_text(self.model.np_enable, self.model.np_arming, self.model.np_running)
        self.value_labels["np"].setText(
            f"使能={bool_text(self.model.np_enable)} 状态={np_state} 工作={bool_text(self.model.np_active)} "
            f"油门={fmt_float(self.model.np_throttle)} 脉宽={fmt_float(self.model.np_pulse_us)}"
        )
        if hasattr(self, "neg_runtime_status_label"):
            self.neg_runtime_status_label.setText(
                f"状态: 使能={bool_text(self.model.np_enable)} 状态={np_state} 工作={bool_text(self.model.np_active)} "
                f"油门={fmt_float(self.model.np_throttle)} 脉宽={fmt_float(self.model.np_pulse_us)}us"
            )
        observed_speed_text = self.format_observed_diff_wheel_text()
        observed_speed_summary_text = observed_speed_text.replace("观测轮速(基于BAS_SPD+差速)", "观测轮速")
        self.value_labels["line"].setText(
            f"位置={fmt_float(self.model.line_pos)} 差速={fmt_float(self.model.delta_rpm)} bits={('--' if self.model.line_bits is None else format(int(self.model.line_bits), '03X'))}\n"
            f"{observed_speed_summary_text} 运行={self.model.run_enable if self.model.run_enable is not None else '--'} 就绪={self.model.line_ready if self.model.line_ready is not None else '--'} "
            f"状态={lf_state_to_text(self.model.lf_state)} 待执行={turn_to_text(self.model.pending_turn)} 转角={fmt_float(self.model.turn_yaw_deg)} "
            f"路线idx={self.model.route_index if self.model.route_index is not None else '--'} 停机={stop_reason_to_text(self.model.stop_reason)} "
            f"光电管={self.model.lf_actual_hz if self.model.lf_actual_hz is not None else '--'}Hz"
        )
        if hasattr(self, "line_debug_obs_speed_label"):
            self.line_debug_obs_speed_label.setText(observed_speed_text)
        self.value_labels["text"].setText(self.model.last_text_rx)

        for index, label in enumerate(self.bit_labels):
            active = (self.model.line_bits is not None) and (((int(self.model.line_bits) >> index) & 0x1) != 0)
            if active:
                label.setStyleSheet(f"background:#fef3c7;color:#8b4f08;border:1px solid #f2b84b;border-radius:4px;font:600 11px {self.theme['font_data']};")
            else:
                label.setStyleSheet(f"background:{self.theme['bg_panel_soft']};color:{self.theme['text_muted']};border:1px solid {self.theme['border']};border-radius:4px;font:11px {self.theme['font_data']};")

        self._refresh_calib_status()
        self._refresh_calib_raw()
        self.refresh_foc_test_panel()
        self.refresh_speed_plot()
        self.refresh_elec_plot()

    def _refresh_calib_status(self) -> None:
        if not hasattr(self, 'calib_status_label'):
            return
        for line in self.model._calib_text_lines:
            self._handle_calib_response(line)
        self.model._calib_text_lines.clear()
        cal = self.model.calib_active
        valid = self.model.calib_valid
        samples_hi = self.model.calib_samples_hi or 0
        if cal:
            self.calib_status_label.setText(f"状态: 采样中  samples≈{samples_hi * 256}+")
        elif valid:
            self.calib_status_label.setText("状态: 校准有效")
        else:
            self.calib_status_label.setText("状态: 未校准/无效")
        self._calib_auto_query()

    def _refresh_calib_raw(self) -> None:
        if not hasattr(self, 'calib_raw_label'):
            return
        sensors = self.model.sensors
        if (sensors is None) or (len(sensors) < self.cols):
            self.calib_raw_label.setText("ADC raw: --")
            return
        lines = []
        for i in range(self.cols):
            v = int(sensors[i])
            lines.append(f"ch{i:02d}:{v:4d}")
        self.calib_raw_label.setText("ADC raw:  " + "  ".join(lines))

    _calib_last_active: int = 0
    def _calib_auto_query(self) -> None:
        cal = self.model.calib_active or 0
        if self._calib_last_active == 1 and cal == 0:
            send_command(self.state, self.log_q, "CALIB?")
        self._calib_last_active = cal

    def _handle_calib_response(self, line: str) -> None:
        if not hasattr(self, 'calib_data_label'):
            return
        if line.startswith("MIN:"):
            try:
                vals = [int(x) for x in line[4:].strip().split(",")]
                self._calib_min = vals
                self._update_calib_display()
            except ValueError:
                pass
        elif line.startswith("MAX:"):
            try:
                vals = [int(x) for x in line[4:].strip().split(",")]
                self._calib_max = vals
                self._update_calib_display()
            except ValueError:
                pass

    _calib_min: List[int] = []
    _calib_max: List[int] = []
    def _update_calib_display(self) -> None:
        if not self._calib_min or not self._calib_max:
            return
        lines = []
        for i in range(len(self._calib_min)):
            mn = self._calib_min[i]
            mx = self._calib_max[i]
            sp = mx - mn
            th = int(mn + sp * 0.35)
            lines.append(f"ch{i:02d}:[{mn:4d}-{mx:4d}] span={sp:4d} thr={th}")
        self.calib_data_label.setText("\n".join(lines))

    def flush_logs(self, now: float) -> None:
        if not self.pending_logs or (now - self.last_log_flush_t) < self.log_flush_interval_s:
            return
        self.log_text.appendPlainText("\n".join(self.pending_logs))
        self.pending_logs.clear()
        self.last_log_flush_t = now

    def loop(self) -> None:
        check_connection_idle_timeout(self.state, self.log_q, float(self.args.idle_timeout))
        self.drain_queues()
        now = time.perf_counter()
        self.update_rates(now)
        self.check_run_control_timeout(now)

        if now - self.last_panel_refresh_t >= self.panel_interval_s:
            self.refresh_panels()
            self.last_panel_refresh_t = now

        self.flush_logs(now)

    def shutdown(self) -> None:
        if self.stop_evt.is_set():
            return
        self.stop_evt.set()
        self.loop_timer.stop()
        with self.state.lock:
            conn = self.state.conn
        if conn is not None:
            cleanup_connection(self.state, conn)
        try:
            self.server.close()
        except OSError:
            pass
        self.timer_resolution.disable()

    def run(self) -> int:
        self.window.show()
        self.window.raise_()
        self.window.activateWindow()
        print("[PC-Qt] 窗口已显示，进入事件循环")
        code = self.app.exec()
        self.shutdown()
        return int(code)


def main() -> int:
    try:
        import PySide6  # noqa: F401
        import pyqtgraph  # noqa: F401
    except Exception as exc:
        print("缺少 Qt 绘图依赖，请先安装: pip install PySide6 pyqtgraph")
        print(f"导入失败: {exc}")
        return 2

    args = build_arg_parser().parse_args()
    try:
        app = QtPlotApp(args)
    except RuntimeError as exc:
        print(f"启动失败: {exc}")
        return 3
    return app.run()


if __name__ == "__main__":
    raise SystemExit(main())
