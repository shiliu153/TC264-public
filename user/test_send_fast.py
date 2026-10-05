"""模拟板端发送 FAST 帧，60Hz，用于测试 PC 客户端渲染性能。

用法: 先启动 PC 客户端，再运行本脚本。
      python test_send_fast.py [--rate 60] [--port 9000]
"""

import argparse
import math
import socket
import struct
import time

MAGIC = b"\xA5\x5A"
FRAME_FAST = 0x01
FRAME_PERF = 0x06
PAYLOAD_FAST_LEN = 22
PAYLOAD_PERF_LEN = 17


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def pack_s16_le(v: float, scale: float) -> bytes:
    ival = int(max(-32768, min(32767, round(v * scale))))
    return struct.pack("<h", ival)


def build_fast_frame(seq: int, t: float) -> bytes:
    """构建 FAST 遥测帧 (0x01)，含模拟正弦波数据。"""
    roll = math.sin(t * 1.3) * 15.0
    pitch = math.sin(t * 0.9) * 8.0
    yaw = math.sin(t * 0.3) * 180.0
    gz = math.sin(t * 2.1) * 50.0
    spd_l_tgt = abs(math.sin(t * 0.7)) * 300.0
    spd_r_tgt = abs(math.sin(t * 0.7 + 0.1)) * 300.0
    spd_l_act = spd_l_tgt + math.sin(t * 5.0) * 10.0
    spd_r_act = spd_r_tgt + math.sin(t * 5.0 + 0.1) * 10.0
    lvq = math.sin(t * 1.1) * 2.0
    rvq = math.sin(t * 1.1 + 0.05) * 2.0
    status = 0x07  # run_enable | pos_valid | line_ready

    payload = bytearray(22)
    payload[0] = seq & 0xFF
    payload[1:3] = pack_s16_le(roll, 100.0)
    payload[3:5] = pack_s16_le(pitch, 100.0)
    payload[5:7] = pack_s16_le(yaw, 100.0)
    payload[7:9] = pack_s16_le(gz, 10.0)
    payload[9:11] = pack_s16_le(spd_l_tgt, 10.0)
    payload[11:13] = pack_s16_le(spd_r_tgt, 10.0)
    payload[13:15] = pack_s16_le(spd_l_act, 10.0)
    payload[15:17] = pack_s16_le(spd_r_act, 10.0)
    payload[17:19] = pack_s16_le(lvq, 1000.0)
    payload[19:21] = pack_s16_le(rvq, 1000.0)
    payload[21] = status

    header = MAGIC + bytes([FRAME_FAST, 0x00, PAYLOAD_FAST_LEN])
    ck = checksum(bytes([FRAME_FAST, 0x00, PAYLOAD_FAST_LEN]) + payload)
    return header + payload + bytes([ck])


def build_perf_frame(seq: int) -> bytes:
    """构建 PERF 遥测帧 (0x06)，模拟 CPU 占用数据。"""
    import random
    cpu0 = int(8000 + random.randint(-200, 200))  # ~80% × 100
    cpu1 = int(3500 + random.randint(-100, 100))  # ~35% × 100
    foc_us = int(45 + random.randint(-5, 5))
    isr1k_us = int(12 + random.randint(-3, 3))
    foc_pct = min(99, int(foc_us * 2))
    window_ms = 200

    payload = bytearray(17)
    payload[0:2] = struct.pack("<H", seq & 0xFFFF)
    payload[2:4] = struct.pack("<H", cpu0)
    payload[4:6] = struct.pack("<H", cpu1)
    payload[6:8] = struct.pack("<H", foc_us)
    payload[8:10] = struct.pack("<H", isr1k_us)
    payload[10] = foc_pct
    payload[11:13] = struct.pack("<H", window_ms)
    payload[13:15] = struct.pack("<H", 0)
    payload[15:17] = struct.pack("<H", 0)

    header = MAGIC + bytes([FRAME_PERF, 0x00, PAYLOAD_PERF_LEN])
    ck = checksum(bytes([FRAME_PERF, 0x00, PAYLOAD_PERF_LEN]) + payload)
    return header + payload + bytes([ck])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=int, default=60, help="发送频率 Hz")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--host", type=str, default="127.0.0.1")
    args = parser.parse_args()

    print(f"[TEST] 连接到 PC 客户端 {args.host}:{args.port} ...")
    print(f"[TEST] 目标发送频率: {args.rate} Hz")

    conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    retries = 0
    while retries < 30:
        try:
            conn.connect((args.host, args.port))
            break
        except (ConnectionRefusedError, OSError):
            retries += 1
            time.sleep(0.5)
    else:
        print("[TEST] 30 次重试后仍无法连接，请确认 PC 客户端已启动")
        return

    print(f"[TEST] 已连接到 PC 客户端")
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    period = 1.0 / args.rate
    seq = 0
    perf_seq = 0
    t0 = time.perf_counter()
    frame_count = 0
    last_report_t = t0
    last_perf_t = t0

    try:
        conn.settimeout(0.1)
        while True:
            loop_start = time.perf_counter()
            t = loop_start - t0

            # 发送 FAST 帧
            frame = build_fast_frame(seq, t)
            try:
                conn.sendall(frame)
            except (BrokenPipeError, ConnectionResetError):
                print("[TEST] 连接断开")
                break

            seq = (seq + 1) & 0xFF
            frame_count += 1

            # 每 200ms 发送一次 PERF 帧
            if loop_start - last_perf_t >= 0.2:
                perf_frame = build_perf_frame(perf_seq)
                try:
                    conn.sendall(perf_frame)
                except (BrokenPipeError, ConnectionResetError):
                    break
                perf_seq = (perf_seq + 1) & 0xFF
                last_perf_t = loop_start

            # 速率报告
            if loop_start - last_report_t >= 1.0:
                elapsed = loop_start - last_report_t
                actual_rate = frame_count / elapsed
                print(f"[TEST] 已发送 {frame_count} 帧, 实际速率 {actual_rate:.1f}/s")
                frame_count = 0
                last_report_t = loop_start

            # 忙等调速，避免 time.sleep 在 Windows 上的 ~15ms 精度限制
            while time.perf_counter() - loop_start < period:
                pass

    except KeyboardInterrupt:
        print("\n[TEST] 中断")
    finally:
        conn.close()
        print("[TEST] 退出")


if __name__ == "__main__":
    main()
