# TC264 智能车 · 2026 疯狂电路赛道

基于**英飞凌 TC264D（AURIX 双核）**的无刷双电机智能车，参加 **2026 年疯狂电路赛道**。
车辆需识别赛道上随机出现的**电路元素**并据此完成循迹决策——工程的核心因此是两条线：
**灰度阵列的元素判定**与**双无刷电机的闭环控制**。

| 亮点 | 说明 |
|---|---|
| **30 种电路元素识别** | 电阻/电容/电感/二极管/三极管/电源/电流源/非门等，含分叉与转弯变体 |
| **双核 10kHz FOC** | CPU1 托管 10kHz 电压模式 FOC 中断，中断代码与数据置于 scratchpad RAM |
| **看门狗 + 双启动模式** | 10kHz 与 1kHz 中断内喂狗；支持有屏/无屏双启动，无屏时超时直接进比赛模式 |
| **双核性能监视器** | 100ms 窗口统计 CPU0/CPU1 占用率，经 `0x06` 遥测帧上报 |
| **可调电源驱动** | `libraries/zf_device/zf_device_adjustable_power.c/.h`（本分支独有） |
| **无线遥测与在线调参** | UART-WiFi 上报快照式遥测帧，可在线改参并写入 Flash |
| **上位机实时可视化** | PyQt5 上位机（3,496 行）实时绘图与命令下发 |

---

## 1. 硬件清单

| 类别 | 型号 / 规格 | 总线与引脚 |
|---|---|---|
| 主控 | 英飞凌 **TC264D**（TC26xD_B-Step，BGA292） | — |
| 电机驱动 | **DRV8300** 三相栅极驱动 ×2（左/右无刷电机） | ATOM0 六路 PWM |
| 磁编码器 | **AS5047P** ×2（左右轮电角度） | 硬件 SPI3 + 独立片选 |
| 姿态传感器 | **IMU660RB** | 硬件 SPI0 |
| 显示屏 | **IPS200** 两寸 TFT | 硬件 SPI2 |
| 循迹传感器 | **16 路灰度**（74HC4067 模拟开关 + ADC） | GPIO 选通 + ADC0_CH6 |
| 无线模块 | WiFi-UART（串口转 WiFi） | UART2 + RTS/RST |
| 负压电机 | 外置电调（1000–2000µs 脉宽） | ATOM1_CH5 @ P02.5，300Hz |
| 输入 / 提示 | 4 按键 + 蜂鸣器 | GPIO |

> 引脚全表、冲突与启动风险警示见 **[docs/pinout.md](docs/pinout.md)**。

---

## 2. 软件架构

自底向上六层，上层依赖下层：

| 层 | 路径 | 职责 |
|---|---|---|
| 0 | `libraries/infineon_libraries/` | 英飞凌 **iLLD** 1.0.1.11.0——外设硬件抽象（第三方，Boost 1.0） |
| 1 | `libraries/zf_common/` | 逐飞通用层：类型定义、时钟、FIFO、字体、中断封装、伞形头 `zf_common_headfile.h` |
| 2 | `libraries/zf_driver/` | 逐飞外设驱动：ADC / PWM / SPI / UART / GPIO / PIT / DMA / Flash / 编码器 |
| 3 | `libraries/zf_device/` | 逐飞器件驱动：AS5047P、IMU660RB、IPS200、灰度 4067、WiFi-UART、按键 |
| 4 | `libraries/zf_components/` | 逐飞高层组件：printf 重定向、上位机协议 |
| 5 | `code/` | **自研控制算法**：FOC/SVPWM、Mahony AHRS、PID、DRV8300 驱动 |
| 6 | `user/` | **自研应用层**：主循环、ISR、菜单、巡线、路线、里程、遥测、参数 |

层 0–4 是**第三方库原样副本**（见 [NOTICE.md](NOTICE.md)）；**本工程的自研代码是 `code/` 与 `user/`，
共 39 个 `.c/.h` 文件、10,019 行**。

### 双核分工

- **CPU0**（`user/cpu0_main.c`）：全部系统初始化（时钟、外设、传感器、FOC 校准），
  之后进入 **10 状态菜单主循环**，负责显示、按键、遥测上报、参数 Flash 读写。
- **CPU1**（`user/cpu1_main.c`）：初始化后托管 **10kHz FOC 周期中断**（`CCU61_CH0` → `foc_ctrl_periodic_isr()`），
  读取编码器、运行 IMU+AHRS、执行 FOC 速度环、施加转向 PID、发布遥测快照。
- CPU0 在初始化完成后通过 `cpu_wait_event_ready()` 通知 CPU1 就绪。
- 全部 ISR 代码与数据置于 CPU1 scratchpad RAM（`cpu1_psram`/`cpu1_dsram`）以保证确定性。
- 链接脚本默认宿主为 CPU1（多数全局数据位于 CPU1）。

### 定时器与实时结构

| 模块 | 周期 | 频率 | 用途 |
|---|---|---|---|
| `CCU61_CH0` | 100 µs | **10 kHz** | 主 FOC 控制环（运行于 CPU1，全系统最延迟敏感） |
| `CCU61_CH1` | 1000 µs | 1 kHz | 次级周期任务 |
| `CCU60_CH0` | 5000 µs | 200 Hz | 低频任务 |
| `CCU60_CH1` | 5000 µs | 200 Hz | 低频任务 |

详见 **[docs/architecture.md](docs/architecture.md)**。

---

## 3. 三分钟跑起来

**前提**：**Tasking TriCore 编译器只随 AURIX Development Studio 提供，不在命令行环境中。**
因此本工程**无法用 `make` 或 CI 构建**——必须用 IDE。

1. 安装 **AURIX Development Studio v1.10.2**（Eclipse CDT + ADS 插件）。
2. `File → Import → General → Existing Projects into Workspace`，选择本仓库根目录。
3. 确认工程识别到器件 `TC26xD_B-Step`、封装 `bga292`（见 `.settings/com.infineon.aurix.buildsystem.prefs`）。
4. 确认 iLLD 路径：工程属性中 `AURIX-LIBRARY-PATH` 应指向 `Libraries/iLLD`、`Libraries/Infra`、`Libraries/Service`。
5. `Project → Build`（构建配置 `Debug`），产物为 `Debug/*.elf` 与 `.hex`。
6. 用 winIDEA / miniWiggler 下载到 TC264D 开发板。
7. **连接 WiFi 前必须填写 `user/wifi_cfg.h` 里的 `YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD` 占位符**，
   否则无线遥测不可用（但不影响车辆基本运行——WiFi 仅供调试）。

> `Debug/` 是 Eclipse 自动生成的构建产物，**不在本仓库中**；导入工程后 IDE 会自行生成。

---

## 4. 关键设计

**元素识别**：16 路灰度阵列逐通道采样后解算线位置，配合路口/分支判定区分 30 种电路元素；
二极管与三极管系列按分叉方向（左/右）与转弯与否分别处理。→ [docs/element-recognition.md](docs/element-recognition.md)

**中心枢轴转向**：原地转向时不做内轮清零，改用双轮里程计 + 线再检测作为退出条件，
避免依赖陀螺仪漂移。→ [docs/architecture.md](docs/architecture.md)

**里程控制**：`move_distance` 提供直线与巡线两种行走模式，供「定距巡线」等元素调用。

**无线调参**：参数系统由描述符表驱动，可通过 WiFi 在线修改并写入 Flash 持久化。→ [docs/tuning.md](docs/tuning.md)

**遥测协议**：快照式采集，ISR 填结构体、`wifi_service` 取快照并上报。→ [docs/telemetry-protocol.md](docs/telemetry-protocol.md)

### 4.1 工程难点：10kHz 控制环的双核硬实时架构

**问题**：双电机 FOC 要求 100µs 的确定性控制周期（10kHz），同时系统还需承担显示刷新、
按键处理、遥测上报与参数读写。上述任务若与控制环共用单核，将互相挤占执行时间，
控制周期抖动会直接体现为电机运行噪声与转向精度下降。

**方案**：

- **职责划分**：CPU0 承担全部系统初始化与人机逻辑；CPU1 仅托管 FOC 周期控制，主循环保持空闲。
  两核通过 `cpu_wait_event_ready()` 同步，确保控制中断不会在传感器与驱动尚未就绪时触发。
- **关键代码与数据置于本地 RAM**：`cpu1_main.c` 整体由
  `#pragma section all "cpu1_dsram"` 约束，使 ISR 代码与全局数据落入 CPU1
  scratchpad RAM。本地 RAM 不存在总线仲裁延迟，其价值在于保证**最坏执行时间**的
  确定性——在硬实时系统中，该指标优先于平均性能。
- **中断优先级显式定序**（取自 `user/isr_config.h`）：FOC 周期中断优先级为 **200**
  （TC264 取值范围 1–255，255 为最高）；遥测发送 `UART2_TX` 为 **16**、接收为 40、
  1kHz 任务为 33、200Hz 任务为 30。该定序从机制上保证遥测不抢占电机控制。

**中断入口与实现分离**：FOC 中断入口注册于 `user/cpu1_main.c`，
而 441 行的控制主体 `foc_ctrl_periodic_isr()` 位于 `user/isr.c`。
分置两个文件的目的，即让中断定义落入 CPU1 的 RAM 段。

### 4.2 工程难点：元素检测与路口检测的耦合导致漏判

**问题**：原始实现将元素检测置于路口检测之后，形成门控关系：

```
路口特征判定 → 计数器确认 → 再分流「是元素还是路口」
```

路口特征的判定条件为「线居中 **且** 出现宽线、双边或分支」，其严格程度显著高于元素特征。
多数元素仅要求「线居中」，无法满足路口特征，因此在上述结构下**不可能被触发**。
该缺陷不产生任何运行时错误，仅表现为赛道上的元素漏判——属静默失效，
在赛道复现前难以通过日志定位。

**方案**：将两者改为**互斥分支**，依路线类型先行分流。以 `route_plan_is_element()`
判定归属：元素路径使用元素确认计数 `element_confirm_cnt`，路口路径使用路口确认计数
`intersection_cnt`，同一帧仅进入其中一条路径。元素检测由此不再依赖路口判定。

**代价**：需将原路由事件处理函数拆分为 `handle_junction_event` 与
`handle_element_event`，并在 3 处复位路径同步清零新增计数器。
收益是元素与路口逻辑解耦，后续新增元素类型不再受路口判定条件约束。

### 4.3 工程难点：遥测发送占用主循环 30% 的 CPU 时间

**问题**：板端经 UART2（460800bps）向 WiFi 模块发送遥测数据，原实现采用逐字节轮询：
每写入一字节后需等待硬件 FIFO 完全排空。实测单帧 28 字节阻塞 CPU 约 **608µs**，
主循环以 2ms 周期调用，等价于约 **30%** 的 CPU 时间消耗于串口发送。

**约束**：WiFi 模块的 RTS 流控信号连接至 **P10.2（普通 GPIO）**，
而 ASCLIN2 的硬件 CTS 位于其他引脚。硬件接线无法变更，因此流控只能在软件层实现。
该约束决定了方案的形态与边界。

**方案**：

1. TX 缓冲区由 1 字节扩展至 **256 字节软件 FIFO**，发送接口改为写入 FIFO 后立即返回；
2. 新增 **TX 中断**负责将数据由软件 FIFO 搬运至硬件 FIFO（硬件 FIFO 深度 16）；
3. 中断内检查 RTS 电平：总线忙时**由中断主动禁用自身**，避免空转消耗 CPU，
   并置 `stalled` 标志；主循环每约 2ms 检测到 RTS 释放后重新使能中断并续传；
4. **AT 命令模式保留同步轮询**——该路径调用频率低，不引入额外复杂度。

**结果**：单帧 CPU 占用由 **608µs 降至 5µs 以下（约 120 倍）**，
主循环阻塞占比由 **30% 降至 0.3% 以下**。

> **边界说明**：链路本身的物理传输耗时**不变**（受 460800bps 波特率约束），
> 遥测帧率亦未提高。本次优化的实质是消除 CPU 空等，而非提升链路带宽。

### 4.4 工程难点：性能监视器读数存在系统性偏差

**问题**：性能监视器以 `busy_tick - delay_tick` 计算 CPU0 有效负载，
但两个变量分别测量**相邻且不重合**的时间区间，时序错位导致结果系统性偏大——
FOC 测试期间 CPU0 负载读数误报为 **100%**，看板数据失去参考价值。

**方案**：将事后相减改为事前隔离。仅累计两次延时结束之间的区间作为忙时，
延时区间直接排除在统计之外，以 `perf_monitor_mark_delay_end()` 替代
`add_delay_ticks()`。

**意义**：测量工具的缺陷会被误判为被测系统的缺陷。在依据监控数据做优化决策前，
须先确认测量链路本身可信。

---

## 5. 目录导览

```
code/        自研控制算法（8 个 .c/.h）——  FOC/SVPWM、AHRS、PID、DRV8300
user/        自研应用层（31 个 .c/.h + wifi_cfg.h + PyQt 上位机）
   ├── cpu0_main.c / cpu1_main.c   双核入口
   ├── isr.c / isr.h / isr_config.h  中断服务与配置
   ├── line_follow.c               巡线与元素识别
   ├── route_plan.c / .h           路线序列与 30 种元素定义
   ├── move_distance.c / .h        里程控制
   ├── wifi_service.c / .h         遥测与命令
   ├── params_flash.c / .h         参数系统与 Flash 持久化
   ├── menu.c / .h                 菜单页面
   ├── pc_wifi_uart_plot_qt.py     PyQt5 上位机（3,496 行）
   └── ...                         电池监测、负压电机、校准串口、性能监视、按键、蜂鸣器
libraries/   第三方库原样副本（634 个文件）—— 请勿在此提交改动
docs/        架构、元素识别、遥测协议、引脚、调参
libraries/doc/    逐飞原始版本说明与 GPL 许可声明
```

> **`libraries/` 是第三方库原样副本**（逐飞 TC264 开源库 V3.4.3 + 英飞凌 iLLD 1.0.1.11.0）。
> 需要的改动请放在 `code/` 或 `user/`，不要改库文件——理由见 [CONTRIBUTING.md](CONTRIBUTING.md)。

---

## 6. 参数与调参

参数系统（`user/params_flash.c`）用描述符表统一管理可调参数：每个参数有类型、默认值、
Flash 地址与所属菜单页。运行时可经 WiFi 改写，确认后写入 Flash 持久化。

调参顺序、菜单页用途与建议流程见 **[docs/tuning.md](docs/tuning.md)**。

---

## 7. 已知限制与分支说明

### 7.1 本仓库是快照式发布

**每个分支只有 1 个提交**，私有开发仓库中的完整历史（127 次提交）不公开。
这样做的原因是历史中包含一度提交过的 WiFi 口令与内部开发文档——即使当前文件已清理，
历史 blob 仍可被 `git log -S` 挖出。

### 7.2 两个分支，内容不同

| 分支 | 特征功能 |
|---|---|
| `feat/20260711-encoder-turn-exit` | 路线元素组、中心枢轴转向、编码器里程退出、可调直道加速、校准 UART |
| **`feat/20260718-adjustable-power-supply`（本分支）** | **看门狗喂狗、无屏/有屏双启动、双核性能监视器、`0x06` 遥测帧、可调电源驱动** |

两条线自 2026-06-01 起分叉后**互不包含**。**本分支含看门狗与无屏启动**；
而**路线元素组、中心枢轴转向、校准 UART 只在另一分支
`feat/20260711-encoder-turn-exit` 上**，本分支确实没有。这是刻意保留的两条开发线，不是遗漏。

### 7.3 本分支特有的实现

| 模块 | 路径 | 说明 |
|---|---|---|
| 看门狗 | `user/watchdog.c/.h` | 10kHz FOC 与 1kHz 中断内均喂狗；相关变量以 `volatile` 声明，防止编译器把喂狗优化掉 |
| 双启动模式 | `user/menu.c` 等 | 有屏/无屏两种启动路径：无屏模式下经超时判断直接进入比赛运行，无需人工按键 |
| 性能监视器 | `user/perf_monitor.c/.h` | 100ms 窗口统计双核占用率；FOC ISR 与 1kHz ISR 均夹 STM 计时，CPU0 主循环反推空闲占比 |
| 可调电源 | `libraries/zf_device/zf_device_adjustable_power.c/.h` | 可调电源器件驱动（本分支独有） |

> **注意**：`user/watchdog.c/.h` 与 `zf_device_adjustable_power.c/.h` 属本分支独有；
> 另一分支上没有这两个模块。

### 7.4 能力边界（如实说明）

- **无法命令行构建**：Tasking 编译器仅随 ADS 提供。本仓库不含 `Debug/` 构建产物，
  也没有 CI——所有验证靠静态检查 + 人在 IDE 中编译 + 实车验证。
- **无单元测试**：嵌入式裸机代码没有测试框架，验证依赖实车跑。
- **`zf_device_config.a` 是二进制**：98,860 字节的静态库，无法审计其内容（见 [NOTICE.md](NOTICE.md)）。
- **部分中文注释乱码**：源工程既有的历史状态（10 个文件含替换字符），**已核实乱码仅在注释内**，
  代码完好、不影响编译。未作修改，理由见 [NOTICE.md](NOTICE.md) 第 4 节。
- **引脚存在共享**：P15.4（屏背光与 SPI2 MISO 占位）、P33.10（蜂鸣器与编码器 SPI 总线）
  为已知共享点；P10.5/P10.6 有启动风险。详见 [docs/pinout.md](docs/pinout.md)。

---

## 8. 规模与许可

**规模**：自研 C 代码 39 个 `.c/.h`、10,019 行；PyQt5 上位机 3,496 行；
30 种电路元素类型；4 个 PIT 定时器（10kHz / 1kHz / 200Hz × 2）。

**许可**：本工程以 **GPL-3.0-or-later** 发布（根目录 [LICENSE](LICENSE)）。
这不是选择而是必然——自研代码与 GPL-3.0 的逐飞库静态链接为同一可执行文件。

**致谢与第三方归属**：

| 组件 | 版本 | 许可 |
|---|---|---|
| [逐飞科技 TC264 开源库](https://seekfree.taobao.com/) | V3.4.3 | **GPL-3.0-or-later**（[libraries/LICENSE](libraries/LICENSE)） |
| 英飞凌 iLLD（AURIX Low-Level Driver） | iLLD_1_0_1_11_0 | Boost Software License 1.0 |

对第三方库的每一处改动都在 **[NOTICE.md](NOTICE.md)** 中逐文件披露。
所有上游文件内的版权声明均原样保留。

**编码说明**：逐飞上游为 GBK 编码，本工程保持原样以便跟进上游更新；自研代码为 UTF-8。
工程编码规则见 `.settings/org.eclipse.core.resources.prefs`（工程默认 UTF-8，`libraries/` 指定 GBK）。
