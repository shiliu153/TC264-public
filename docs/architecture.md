# 架构说明

> 本文基于分支 `feat/20260711-encoder-turn-exit` 的快照。

## 1. 六层软件栈

自底向上，上层依赖下层，依赖方向单向：

| 层 | 路径 | 职责 | 归属 |
|---|---|---|---|
| 0 | `libraries/infineon_libraries/` | 英飞凌 iLLD 1.0.1.11.0——外设硬件抽象（MCAL） | 第三方（Boost 1.0） |
| 1 | `libraries/zf_common/` | 逐飞通用层：类型定义、时钟、FIFO、字体、中断封装 | 第三方（GPL-3.0） |
| 2 | `libraries/zf_driver/` | 逐飞外设驱动：ADC、PWM、SPI、UART、GPIO、PIT、DMA、Flash、编码器、软 IIC/SPI | 第三方（GPL-3.0） |
| 3 | `libraries/zf_device/` | 逐飞器件驱动：AS5047P、IMU660RB、IPS200、灰度 4067、WiFi-UART、按键 | 第三方（GPL-3.0） |
| 4 | `libraries/zf_components/` | 逐飞高层组件：printf 重定向、上位机协议 | 第三方（GPL-3.0） |
| 5 | `code/` | **自研控制算法**：FOC/SVPWM、Mahony AHRS、PID、DRV8300 驱动 | 自研 |
| 6 | `user/` | **自研应用层**：双核入口、ISR、菜单、巡线、路线、里程、遥测、参数 | 自研 |

**伞形头**：`libraries/zf_common/zf_common_headfile.h` 把层 0–5 的头文件全部包含进来，
因此自研代码通常只需 `#include "zf_common_headfile.h"`。

## 2. 双核分工

TC264D 是双核 TriCore。两核在 `main()` 之后各自进入自己的入口：

| | CPU0 | CPU1 |
|---|---|---|
| 入口 | `user/cpu0_main.c` → `core0_main()` | `user/cpu1_main.c` → `core1_main()` |
| 初始化 | **全部**系统初始化：时钟、外设、传感器、FOC 校准 | 仅关闭看门狗、开全局中断，然后等待 CPU0 就绪 |
| 主循环 | 10 状态菜单主循环：显示、按键、遥测上报、参数 Flash 读写 | `while(TRUE)` 空转（不承载业务） |
| 实时任务 | 200Hz / 1kHz 低频 ISR、UART ISR、DMA ISR | **10kHz FOC 周期中断** |

**就绪握手**：CPU1 在 `core1_main()` 中调用 `cpu_wait_event_ready()` 阻塞等待；
CPU0 完成全部初始化后发事件放行。这保证 FOC 中断不会在传感器/驱动未初始化时触发。

**为什么 CPU1 主循环是空的**：CPU1 的职责只有 FOC 中断。业务逻辑全部在 CPU0，
避免两核争用同一批外设与共享数据。

## 3. FOC 控制环（全系统最关键路径）

### 3.1 中断归属：注意实现位置

FOC 周期中断的**中断向量注册在 `user/cpu1_main.c`**，不在 `user/isr.c`：

```c
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)   // cpu1_main.c:89
{
    foc_ctrl_periodic_isr();                                  // 控制主体在 isr.c:356
}
```

即：**中断入口在 `cpu1_main.c`，控制主体 `foc_ctrl_periodic_isr()` 实现在 `user/isr.c`**。
两者分处不同文件是为把 ISR 定义放进 CPU1 的 RAM 段（见 3.3）。

`IFX_INTERRUPT` 宏来自 iLLD 的 `IfxCpu_Irq.h`。本工程在非 Tasking 编译器下
把它重定义为普通函数声明（`isr.c:18`、`cpu1_main.c:54`），以便静态检查工具解析。

### 3.2 定时器分配（实测自 `user/isr.c` 与 `user/cpu0_main.c`）

| PIT | 周期 | 频率 | 归属核 | 中断优先级 | 用途 |
|---|---|---|---|---|---|
| `CCU61_CH0` | 100 µs | **10 kHz** | cpu1 | 200 | FOC 主控制环 |
| `CCU61_CH1` | 1000 µs | 1 kHz | cpu0 | 33 | 次级周期任务、遥测快照 |
| `CCU60_CH1` | 1000 µs | 1 kHz | cpu0 | 31 | 低频任务 |
| `CCU60_CH0` | 5000 µs | 200 Hz | cpu0 | 30 | 低频任务 |

初始化序列（`cpu0_main.c:356-359`）：

```c
pit_us_init(FOC_CTRL_PIT_INDEX, FOC_CTRL_PERIOD_US);   // CCU61_CH0, 100us
pit_us_init(CCU61_CH1, 1000u);
pit_us_init(CCU60_CH1, 1000);
pit_us_init(CCU60_CH0, 5000);
```

其中 `FOC_CTRL_PIT_INDEX = CCU61_CH0`、`FOC_CTRL_PERIOD_US = 100u`（`user/isr.h:23,27`）。

> **注意**：中文注释与既有文档里出现的「CCU60/CCU61」是同一组 CCU6 模块的两个单元；
> 代码中的宏名为 `CCU60_CH0/CH1`、`CCU61_CH0/CH1`。以本节表格为准。

### 3.3 代码与数据放入 CPU1 本地 RAM

`user/cpu1_main.c` 整体被段指令包住：

```c
#pragma section all "cpu1_dsram"     // cpu1_main.c:57
...
#pragma section all restore          // cpu1_main.c:94
```

落在此段内的全局数据与 ISR 代码被链接到 CPU1 的 **DSPR/PSPR（scratchpad RAM）**——
本地 RAM 无总线仲裁延迟，访问确定性远高于共享内存，对 10kHz 硬实时的最坏执行时间至关重要。

链接脚本 `Lcf_Tasking_Tricore_Tc.lsl` 的默认数据宿主为 CPU1（多数全局变量位于 CPU1）。

### 3.4 一个周期内做什么

`foc_ctrl_periodic_isr()`（`isr.c:356` 起，441 行）按顺序完成：

1. 清 `CCU61_CH0` 中断标志、记录性能计时起点；
2. 读双路 AS5047P 磁编码器 → 机械角/电角度；
3. 运行 IMU660RB + Mahony AHRS → 姿态角、陀螺零偏；
4. 按 `FOC_SPEED_ESTIMATE_HZ`（1kHz）降频做速度估计与 PI 速度环；
5. 施加转向 PID，得到左右目标转速；
6. SVPWM 输出到 ATOM0 六路 PWM；
7. 发布遥测快照与运行时命令。

关键常量（`code/foc_svpwm.h`）：

| 常量 | 值 | 含义 |
|---|---|---|
| `FOC_DEFAULT_POLE_PAIRS` | 7 | 电机极对数 |
| `FOC_MAX_ACCEL_RPM_PS` | 10000 | 加速上限（转/分/秒） |
| `FOC_MAX_DECEL_RPM_PS` | 120000 | 减速上限（转/分/秒） |
| `FOC_SVPWM_LUT_SIZE` | 256 | SVPWM 查表长度 |
| `FOC_SPEED_EST_GLITCH_MIN_RPM` / `MAX_RPM` | 800 / 6000 | 速度估计毛刺窗口 |

速度斜坡统一由 PI 层的加减速上限控制（`isr.h:103-104` 注释说明 1kHz ISR 与 FOC ISR
不再做命令级爬坡）。

### 3.5 电压模式 FOC

本工程**没有电流传感器**，属**电压模式 FOC**：

- 电角度来自 AS5047P 磁编码器（非无感观测器）；
- 速度环输出 Vq 指令，经 SVPWM 查表转成三相占空比；
- 无电流内环，因此过流保护依赖硬件（DRV8300 与电源侧）。

## 4. 中断一览

除 4 个 PIT 外，`user/isr.c` 还注册了：

| 外设 | 中断 | 服务核 |
|---|---|---|
| ERU 外部中断 | `exti_ch0_ch4`、`exti_ch3_ch7` | cpu0 |
| ERU 外部中断 | `exti_ch1_ch5`、`exti_ch2_ch6` | **dma** |
| DMA | `dma_ch6`、`dma_ch7` | cpu0 |
| UART0–3 | tx / rx / error 各一路 | cpu0 |

优先级定义集中在 `user/isr_config.h`，每个中断同时指定**服务提供者**（`IfxSrc_Tos_*`）
与**优先级**。改动中断向量、优先级或服务核属于 **R3 风险**，见 `CONTRIBUTING.md`。

## 5. 数据流概览

```
        ┌─────────────── CPU0 ───────────────┐        ┌──────── CPU1 ────────┐
按键 ──►│ menu 状态机（10 状态）              │        │                      │
显示 ◄──│  ├ 显示渲染                         │        │  cc61_pit_ch0_isr    │
        │  ├ 参数读写 → params_flash → Flash  │        │   └ foc_ctrl_periodic│
        │  └ 遥测上报 → wifi_service ─────────┼──UART2─┼──► WiFi 模块 → 上位机 │
        │                                     │        │  ├ 编码器 AS5047P ×2 │
        │  200Hz/1kHz ISR                     │        │  ├ IMU660RB + AHRS   │
        │   └ 巡线判定 → line_follow          │        │  ├ 速度环 + 转向 PID │
        │       └ 路线推进 → route_plan       │        │  └ SVPWM → ATOM0 →电机│
        └─────────────────────────────────────┘        └──────────────────────┘
```

共享数据通过 `user/isr.h` 中声明的 `volatile` 全局量在两核间传递。

## 6. 相关文档

- 元素识别原理：[element-recognition.md](element-recognition.md)
- 引脚分配与冲突：[pinout.md](pinout.md)
- 遥测帧格式：[telemetry-protocol.md](telemetry-protocol.md)
- 参数与调参：[tuning.md](tuning.md)
