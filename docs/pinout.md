# 引脚分配与冲突警示

> 本文基于分支 `feat/20260711-encoder-turn-exit` 的快照，逐项对照源码核实。
> 事实来源：各驱动的引脚宏定义、`user/key.h`、`code/bldc_drv8300_3pwm.h`、`user/neg_pressure_motor.h`。

## 1. 按模块的引脚表

### 1.1 WiFi-UART（UART2）

来源：`libraries/zf_device/zf_device_wifi_uart.h`

| 信号 | 引脚 | 宏 | 备注 |
|---|---|---|---|
| UART2 TX | P10.6 | `WIFI_UART_TX_PIN = UART2_RX_P10_6` | 该库宏命名方向与直觉相反，按注释理解 |
| UART2 RX | P10.5 | `WIFI_UART_RX_PIN = UART2_TX_P10_5` | **P10.5/P10.6 有启动风险标注** |
| RTS | P10.2 | `WIFI_UART_RTS_PIN` | 曾与编码器 SPI1 SCK 冲突，已由编码器改用 SPI3 解决 |
| RST | P11.6 | `WIFI_UART_RST_PIN` | 硬复位脚 |

### 1.2 IPS200 显示屏（硬件 SPI2）

来源：`libraries/zf_device/zf_device_ips200.h`

| 信号 | 引脚 | 宏 |
|---|---|---|
| SCK | P15.3 | `IPS200_SCL_PIN_SPI = SPI2_SCLK_P15_3` |
| MOSI | P15.5 | `IPS200_SDA_PIN_SPI = SPI2_MOSI_P15_5` |
| MISO（占位） | P15.4 | `IPS200_SDA_IN_PIN_SPI = SPI2_MISO_P15_4` |
| RST | P15.1 | `IPS200_RST_PIN_SPI` |
| DC | P15.0 | `IPS200_DC_PIN_SPI` |
| CS | P15.2 | `IPS200_CS_PIN_SPI` |
| BLK（背光） | **P15.4** | `IPS200_BLk_PIN_SPI` |

⚠️ **P15.4 双重占用**：IPS200 **没有 MISO 引脚**，但 `spi_init()` 需要传入一个枚举值，
故把 P15.4 声明为 MISO 占位；同时 P15.4 又是屏的背光控制脚。
即**同一个引脚既作背光输出、又在 SPI 初始化里被登记为 MISO 占位**。
由于屏不实际驱动 MISO，当前可工作，但**这是隐患**——改为并行接口或换屏时须重新核查。

### 1.3 IMU660RB（硬件 SPI0）

来源：`libraries/zf_device/zf_device_imu660rb.h`

| 信号 | 引脚 | 宏 |
|---|---|---|
| SCK | P20.11 | `IMU660RB_SPC_PIN = SPI0_SCLK_P20_11` |
| MOSI | P20.14 | `IMU660RB_SDI_PIN = SPI0_MOSI_P20_14` |
| MISO | P20.12 | `IMU660RB_SDO_PIN = SPI0_MISO_P20_12` |
| CS | P20.13 | `IMU660RB_CS_PIN` |

### 1.4 AS5047P 磁编码器 ×2（硬件 SPI3）

来源：`libraries/zf_device/zf_device_as5047p.h`

| 信号 | 引脚 | 宏 | 备注 |
|---|---|---|---|
| SCK | P33.11 | `AS5047P_DEFAULT_SCK_PIN = SPI3_SCLK_P33_11` | 已避开 WiFi RTS(P10.2) 与电机 PWM |
| MOSI | P33.12 | `AS5047P_DEFAULT_MOSI_PIN` | |
| MISO | P33.13 | `AS5047P_DEFAULT_MISO_PIN` | |
| CS 左 | P33.9 | `AS5047P_DEFAULT_CS1_PIN` | GPIO 手动控制（避开蜂鸣器的 P33.10） |
| CS 右 | P33.8 | `AS5047P_DEFAULT_CS2_PIN` | GPIO 手动控制 |

两颗编码器**共用 3 根 SPI 线**，靠独立片选区分。

### 1.5 DRV8300 三相驱动（双电机 6 路 PWM，ATOM0）

来源：`code/bldc_drv8300_3pwm.h`

| 电机 | 相位 | PWM 通道 @ 引脚 | 宏 |
|---|---|---|---|
| 左 | U | ATOM0_CH0 @ P21.2 | `BLDC_DRV8300_L_U_PWM` |
| 左 | V | ATOM0_CH1 @ P21.3 | `BLDC_DRV8300_L_V_PWM` |
| 左 | W | ATOM0_CH2 @ P21.4 | `BLDC_DRV8300_L_W_PWM` |
| 右 | U | ATOM0_CH3 @ P21.5 | `BLDC_DRV8300_R_U_PWM` |
| 右 | V | ATOM0_CH4 @ P22.3 | `BLDC_DRV8300_R_V_PWM` |
| 右 | W | ATOM0_CH5 @ P32.4 | `BLDC_DRV8300_R_W_PWM` |

### 1.6 负压电机（外置电调）

来源：`user/neg_pressure_motor.h`

| 项 | 值 | 宏 |
|---|---|---|
| PWM 通道 @ 引脚 | ATOM1_CH5 @ P02.5 | `NEG_PRESSURE_MOTOR_PWM_CH` |
| PWM 频率 | 300 Hz | `NEG_PRESSURE_MOTOR_PWM_FREQ_HZ` |
| 脉宽范围 | 1000–2000 µs（标准电调协议） | — |

### 1.7 按键与蜂鸣器

来源：`user/key.h`、`user/cpu0_main.c`

| 功能 | 引脚 | 宏 |
|---|---|---|
| KEY1 | P20.6 | `KEY1_PIN` |
| KEY2 | P20.7 | `KEY2_PIN` |
| KEY3 | P11.2 | `KEY3_PIN` |
| KEY4 | P11.3 | `KEY4_PIN` |
| 蜂鸣器 | **P33.10** | `INIT_BUZZER_PIN` |

⚠️ **P33.10 与编码器 SPI 总线同组**：蜂鸣器占用 P33.10，而编码器 SPI3 用 P33.8/9/11/12/13。
蜂鸣器仅在初始化阶段短暂鸣响，故当前不冲突；但**若让蜂鸣器在运行期持续发声，需重新评估**。
（历史提交中曾有 `buzzer.c` 残留文件被删除，原因即 ATOM0_CH4 与电机 PWM 冲突。）

### 1.8 灰度/光电管 74HC4067（**在用**）

来源：`libraries/zf_device/zf_device_gray_4067.h`

| 信号 | 引脚 / 通道 | 宏 |
|---|---|---|
| S0 | P00.0 | `GRAY_4067_S0_PIN` |
| S1 | P00.1 | `GRAY_4067_S1_PIN` |
| S2 | P00.2 | `GRAY_4067_S2_PIN` |
| S3 | P00.3 | `GRAY_4067_S3_PIN` |
| /EN | P00.4 | `GRAY_4067_EN_PIN` |
| ADC | ADC0_CH6_A6 | `GRAY_4067_ADC_CHANNEL` |

**16 路共用一根 ADC 线**，靠 S0–S3 分时选通。

> ⚠️ **纠正一处过时说法**：工程内一份旧的 IO 整理文档称「灰度 4067 当前未启用」，
> **该说法已过时**。实测源码中灰度是**在用**的：
> `user/cpu0_main.c:261-262` 调用 `gray_4067_init()` 与 `gray_4067_enable(1)`；
> `user/line_follow.c:1156` 调用 `gray_4067_read_12_raw()` 读取整帧。
> 巡线功能完全依赖这组引脚。

## 2. 冲突与风险

### 2.1 已解决的冲突

| 原冲突 | 解决方式 |
|---|---|
| 编码器 SPI1 SCK 原用 P10.2，与 WiFi RTS 同脚 | 编码器改用 SPI3（P33.11/12/13），不再占用 P10.2 |
| 右轮 PWM 原用 P02.5，与 `SPI3_MISO_P02_5` 可选映射冲突 | 编码器 MISO 改用 P33.13，右轮 PWM 迁至 P22.3/P32.4 |

### 2.2 现存的引脚共享（须知情）

| 引脚 | 共享方 | 风险 |
|---|---|---|
| P15.4 | IPS200 背光 + SPI2 MISO 占位 | 中——靠「屏不驱动 MISO」维持，换屏/改接口须重核 |
| P33.10 | 蜂鸣器 + 编码器 SPI 同组 | 低——蜂鸣器仅初始化期短鸣 |
| P10.5 / P10.6 | WiFi-UART TX/RX，且为**启动风险脚** | 中——影响上电成功率，见下 |

### 2.3 启动风险引脚

`P10.5` / `P10.6` 被工程内的「尽量不要使用的引脚」清单标为 boot 风险脚，
而当前正处于 WiFi-UART 的 UART2 TX/RX 上。
**若上电偶发失败**，优先怀疑这两个脚——可把 WiFi-UART 换到其它 UART 引脚验证。
WiFi 仅用于调试，换脚不影响比赛运行。

### 2.4 禁用引脚（不要使用）

| 引脚 | 原因 |
|---|---|
| P14.2 – P14.6 | 启动配置脚（boot configuration） |
| P20.2 | 仅下拉，无法可靠输出高电平 |
| P21.6 | 部分 TC264 批次不可用 |

## 3. 扩展建议

若需要**再加一颗无刷负压电机**（同样 3PWM）：

| 相位 | 建议通道 @ 引脚 |
|---|---|
| U | ATOM1_CH4 @ P02.4 |
| V | ATOM1_CH5 @ P02.5 |
| W | ATOM1_CH6 @ P02.6 |

理由：三路物理相邻且属同一 ATOM1 模块的相邻通道；而 ATOM0 的 CH0–CH5 已被双电机占满。

> **注意**：P02.5 当前被负压电机（有刷电调）占用。若要改为无刷三路 PWM，
> 需先解决 P02.5 的归属，并同步修改 `NEG_PRESSURE_MOTOR_PWM_CH`。

备选相邻组：`ATOM1_CH5_P02_5` / `ATOM1_CH6_P02_6` / `ATOM1_CH7_P02_7`。

## 4. 改引脚时的检查清单

1. 该引脚是否在 §2.4 的禁用清单里？
2. 是否与现有模块撞脚？逐一比对本文 §1 全部表格。
3. 是否为启动风险脚（P10.5/P10.6 一类）？
4. 目标外设的**引脚映射表**是否支持该脚（TC264 的引脚↔外设映射有硬约束，
   例如某 UART 只能在特定引脚组上）？
5. 改完必须查 `libraries/zf_device/zf_device_config.h` 是否受影响。

改引脚属 **R2 风险**（涉及外设初始化），改中断向量或 PWM 保护属 **R3**——
见 [CONTRIBUTING.md](../CONTRIBUTING.md)。
