# 遥测协议

> 本文基于分支 `feat/20260711-encoder-turn-exit` 的快照。
> 事实来源：`user/wifi_service.c`、`user/pc_wifi_uart_plot_qt.py`。

## 1. 概述

- **链路**：TC264 的 `UART2` ↔ WiFi 模块 ↔ PC。上位机 `user/pc_wifi_uart_plot_qt.py` 监听 TCP。
- **方向**：上行 = 遥测帧（下文）；下行 = 控制命令（见 §5）。
- **采集方式**：**快照式**。ISR 只把数据写进快照结构体（`wifi_service_capture_snapshot_isr()`），
  CPU0 周期性取最新快照并打包发送——避免在中断里做串口发送。
- **配置**：SSID / 口令 / 上位机 IP 端口在 `user/wifi_cfg.h`（**占位符，使用前必须填写**）。

> **WiFi 仅供调试**：任何运行路径都不得依赖 WiFi 在线。掉线不影响车辆行驶。

## 2. 帧格式（实测自 `wifi_service_send_frame()`，`wifi_service.c:473`）

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | `MAGIC0` | 固定 `0xA5` |
| 1 | 1 | `MAGIC1` | 固定 `0x5A` |
| 2 | 1 | `type` | 帧类型（§3） |
| 3 | 1 | `flags` | 标志位；当前遥测帧均为 `0x00` |
| 4 | 1 | `len` | 载荷长度（0–255） |
| 5 | len | `payload` | 载荷（§4） |
| 5+len | 1 | `checksum` | 见下 |

**校验和**：从 `type` 字节（偏移 2）开始、累计 `len + 2` 个字节的**算术和取低 8 位**
（`wifi_service_frame_checksum()`，`wifi_service.c:669`）：

```c
sum = Σ buf[start .. start+len-1];   // start = 2, len = 2 + payload_len
checksum = sum & 0xFF;
```

即校验覆盖区间是 `type`、`flags`、`len` 与全部 `payload`，**不覆盖两个 magic 字节**。

**帧长上限**：`WIFI_UART_TELEMETRY_LINE_MAX = 96` 字节（含 6 字节头部开销）。

**magic 常量**：`WIFI_UART_TELEMETRY_MAGIC0 = 0xA5`、`MAGIC1 = 0x5A`。

## 3. 帧类型

| 值 | 宏 | 方向 | 内容 |
|---:|---|---|---|
| `0x01` | `WIFI_UART_TELEMETRY_TYPE_FAST` | 上行 | 高频姿态/速度/Vq（§4.1） |
| `0x02` | `WIFI_UART_TELEMETRY_TYPE_ROUTE` | 上行 | 巡线状态、路线进度、里程（§4.2） |
| `0x03` | `WIFI_UART_TELEMETRY_TYPE_POWER` | 上行 | 电池与负压电机（§4.3） |
| `0x04` | `WIFI_UART_TELEMETRY_TYPE_GRAY` | 上行 | 16 路灰度**压缩**值（§4.4） |
| `0x05` | `WIFI_UART_TELEMETRY_TYPE_GRAY_RAW` | 上行 | 16 路灰度**原始**值（§4.5） |
| `0x06` | `WIFI_UART_TELEMETRY_TYPE_PERF` | 上行 | 双核性能监视器（§4.6） |
| `0x10` | `WIFI_UART_CONTROL_TYPE_CMD` | 下行 | 控制命令（§5） |
| `0x11` | `WIFI_UART_CONTROL_TYPE_ACK` | 上行 | 命令应答（§5.2） |

## 4. 载荷布局

所有多字节整数均为**小端**。带 `_scaled` 的字段是「浮点 × 缩放系数 → 有符号 16 位」，
上位机除以同一系数还原。

| 缩放宏 | 系数 | 用于 |
|---|---:|---|
| `WIFI_UART_TELEMETRY_ANGLE_SCALE` | 100 | 角度（度） |
| `WIFI_UART_TELEMETRY_GYRO_SCALE` | 10 | 角速度（°/s） |
| `WIFI_UART_TELEMETRY_SPEED_SCALE` | 10 | 转速（RPM） |
| `WIFI_UART_TELEMETRY_VQ_SCALE` | 1000 | Vq 电压指令 |
| `WIFI_UART_TELEMETRY_VOLT_SCALE` | 100 | 电池电压（V） |
| `WIFI_UART_TELEMETRY_NP_PERCENT_SCALE` | 10 | 负压电机百分比 |
| `WIFI_UART_TELEMETRY_NP_PULSE_US_SCALE` | 1 | 负压电机脉宽（µs） |
| `WIFI_UART_TELEMETRY_ODOM_SCALE` | 1 | 里程（mm） |

### 4.1 `0x01` FAST 帧（载荷 22 字节）
| 偏移 | 类型 | 字段 | 缩放 |
|---:|---|---|---:|
| 0 | u8 | `seq` | — |
| 1 | s16 | `roll`（已减去上电零点） | 100 |
| 3 | s16 | `pitch`（已减去上电零点） | 100 |
| 5 | s16 | `yaw`（已减去上电零点，环绕 ±180°） | 100 |
| 7 | s16 | `gyro_z` | 10 |
| 9 | s16 | `left_target_rpm` | 10 |
| 11 | s16 | `right_target_rpm` | 10 |
| 13 | s16 | `left_actual_rpm` | 10 |
| 15 | s16 | `right_actual_rpm` | 10 |
| 17 | s16 | `left_vq_out` | 1000 |
| 19 | s16 | `right_vq_out` | 1000 |
| 21 | u8 | `status_flags` | — |

`status_flags` 位定义：

| 位 | 含义 |
|---:|---|
| 0 | `run_enable` 运行中 |
| 1 | `line_pos_valid` 线位置有效 |
| 2 | `line_follow_ready` 巡线就绪 |
| 3 | `odom_count_active` 里程计数激活 |

> 载荷自检：1（`seq`）+ 9×2（9 个 s16）+ 1（`status_flags`）= **22 字节**，
> 与实现中 `uint8_t payload[22]` 的声明一致。

### 4.2 `0x02` ROUTE 帧

| 偏移 | 类型 | 字段 | 缩放 |
|---:|---|---|---:|
| 0 | u8 | `seq` | — |
| 1 | s16 | `line_pos` | 100 |
| 3 | s16 | `turn_delta_rpm` | 10 |
| 5 | u16 | `line_bits`（16 路灰度位图） | — |
| 7 | u8 | `lf_state` 巡线状态机 | — |
| 8 | u8 | `pending_turn` 待执行转向 | — |
| 9 | u8 | `route_index` 路线游标 | — |
| 10 | u8 | `lost_line_cnt` 丢线计数 | — |
| 11 | u8 | `stop_reason` 停止原因（见 `isr.h` 的 `system_stop_reason_t`） | — |
| 12 | u8 | `junction_lock_cnt` | — |
| 13 | u8 | `element_enter_cnt` | — |
| 14 | u8 | `junction_enter_cnt` | — |
| 15 | u16 | `lf_actual_hz` 巡线实际帧率 | — |
| 17 | u8 | `calib_active` | — |
| 18 | u8 | `calib_samples_hi` | — |
| 19 | u8 | `calib_valid` | — |
| 20 | s16 | `turn_yaw_deg` | 100 |
| 22 | s16 | `left_elec_angle_deg` | 100 |
| 24 | s16 | `right_elec_angle_deg` | 100 |
| 26 | s16 | `odom_left_mm` | 1 |
| 28 | s16 | `odom_right_mm` | 1 |

### 4.3 `0x03` POWER 帧

| 偏移 | 类型 | 字段 | 缩放 |
|---:|---|---|---:|
| 0 | u8 | `seq` | — |
| 1 | s16 | `battery_voltage_v` | 100 |
| 3 | u8 | `battery_flags` | — |
| 4 | u8 | `battery_percent` | — |
| 5 | s16 | `np_target_percent` | 10 |
| 7 | s16 | `np_output_pulse_us` | 1 |
| 9 | u8 | `np_flags` | — |

### 4.4 `0x04` GRAY 帧（压缩）

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u8 | `seq` |
| 1–16 | u8 ×16 | 16 路灰度值，每路取原始值右移 `WIFI_UART_TELEMETRY_GRAY_SHIFT` 位后压成 1 字节 |

### 4.5 `0x05` GRAY_RAW 帧（原始）

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u8 | `seq` |
| 1–32 | u16LE ×16 | 16 路灰度原始值，逐路小端两字节 |

**选择哪一帧**由 `g_wifi_gray_raw_enable` 决定（`wifi_service.c:604`）：
非 0 发 `0x05`，否则发 `0x04`。原始帧信息全但带宽翻倍。

> 若快照的 `gray_valid` 为 0，灰度帧**直接不发送**（`wifi_service.c:599`）。

### 4.6 `0x06` PERF 帧

双核 CPU 占用率监视（`user/perf_monitor.c`，100ms 统计窗口）。
载荷由 `wifi_service_send_perf_frame()` 打包，含 CPU0/CPU1 占用率与 FOC ISR 计时统计。

> **该帧在分支 `feat/20260718-adjustable-power-supply` 上更完整**（那条线含独立的
> `perf_monitor.c/.h` 与配套上位机仪表盘）。本分支的载荷布局请直接以
> `wifi_service_send_perf_frame()` 的实现为准——此处不复制具体偏移，避免与实现脱节。

## 5. 下行控制命令

### 5.1 命令帧（`type = 0x10`）

载荷首字节为 `seq`（序号），其后为 `opcode` 与参数。
已定义的 opcode（`wifi_service.c:126` 起）：

| 值 | 宏 | 语义 |
|---:|---|---|
| `0x01` | `WIFI_UART_CONTROL_OPCODE_START` | 开始运行 |
| `0x02` | `WIFI_UART_CONTROL_OPCODE_PAUSE` | 暂停 |
| `0x03` | `WIFI_UART_CONTROL_OPCODE_ODOM_START` | 里程计数开始 |
| `0x04` | `WIFI_UART_CONTROL_OPCODE_ODOM_CLEAR` | 里程清零 |
| `0x05` | `WIFI_UART_CONTROL_OPCODE_NPON` | 负压电机开 |
| `0x06` | `WIFI_UART_CONTROL_OPCODE_NPOFF` | 负压电机关 |

除二进制命令外，`wifi_service` 还支持**关键字文本命令**（`wifi_service_cmd_equals()` /
`wifi_service_cmd_starts_with()`），其中一部分用于**参数读写**——
参数系统会把已注册的参数名加入可识别关键字（见 [tuning.md](tuning.md)）。

### 5.2 应答帧（`type = 0x11`）

载荷固定 3 字节：

| 偏移 | 字段 |
|---:|---|
| 0 | `seq`（回显命令序号） |
| 1 | `opcode` |
| 2 | `status`（0 = 成功） |

固件侧记录 `g_wifi_last_ok_cmd_seq`，用于**命令防丢**：重复序号可被识别。

## 6. 上位机

`user/pc_wifi_uart_plot_qt.py`（**3,496 行**，PyQt5）职责：

- 连接车辆（TCP 客户端），解析上述全部帧类型；
- 实时绘图：姿态、双轮目标/实际转速、Vq、线位置；
- 灰度可视化（压缩/原始两种）；
- 参数面板：下发参数读写命令；
- 页面切换与调试页面联动（`0x10` 命令）。

**依赖**：PyQt5（绘图部分依赖 `pyqtgraph`，若缺失请按脚本顶部 import 安装）。

**运行**：

```bash
python user/pc_wifi_uart_plot_qt.py
```

运行前确认 `user/wifi_cfg.h` 的 `WIFI_UART_TARGET_IP` / `WIFI_UART_TARGET_PORT`
与 PC 实际地址一致（默认 `192.168.137.1:9000`，即 Windows 网络共享的默认网关地址）。

## 7. 改协议时的注意点

- **向后兼容是硬要求**：上位机与固件是两套独立分发的代码。新增字段请**追加到载荷末尾**，
  不要插在中间；新增帧类型用未占用的 `type` 值。
- 改动帧结构必须同步更新：`wifi_service_send_*_frame()`、上位机解析、**以及本文档**。
- 校验和算法、magic 字节、缩放系数属于跨端契约，任一处改动都会导致全部帧解析失败。
