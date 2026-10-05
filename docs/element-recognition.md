# 元素识别原理

> 本文基于分支 `feat/20260711-encoder-turn-exit` 的快照。
> 事实来源：`user/route_plan.h`（元素枚举）、`user/line_follow.c`（判定逻辑）。

## 1. 问题定义

赛道地面印有**电路元素**，车辆需要识别自己正压过/即将压过哪一种元素，并据此决定动作。
元素既包括单个元件符号（电阻、电容……），也包括**带分叉的符号**（二极管、三极管），
后者要求车辆在分叉点选择左/右分支。

## 2. 元素类型清单

`user/route_plan.h` 的 `route_turn_t` 枚举定义了全部取值。
**注意枚举的前三项不是元素，而是普通转向指令**，因此 `ROUTE_ELEM_COUNT` 计的是元素种类数（30），
而枚举值范围是 3–32：

```c
ROUTE_TURN_STRAIGHT = 0,   // 直行（非元素）
ROUTE_TURN_LEFT     = 1,   // 左转（非元素）
ROUTE_TURN_RIGHT    = 2,   // 右转（非元素）
ROUTE_ELEM_BUTTON   = 3,   // 元素从此开始
...
ROUTE_ELEM_DIODE3_FORK_R_TURN = 32,
```

### 2.1 元素全表（30 种）

| 枚举值 | 宏 | 中文名 | 类别 |
|---:|---|---|---|
| 3 | `ROUTE_ELEM_BUTTON` | 按键 | 元件 |
| 4 | `ROUTE_ELEM_LAMP` | 灯泡 | 元件 |
| 5 | `ROUTE_ELEM_BATTERY_1` | 电池_1型 | 元件 |
| 6 | `ROUTE_ELEM_BATTERY_2` | 电池_2型 | 元件 |
| 7 | `ROUTE_ELEM_INDUCTOR_1` | 电感_1型 | 元件 |
| 8 | `ROUTE_ELEM_INDUCTOR_2` | 电感_2型 | 元件 |
| 9 | `ROUTE_ELEM_CURRENT_SRC` | 电流源 | 元件 |
| 10 | `ROUTE_ELEM_CAPACITOR` | 电容 | 元件 |
| 11 | `ROUTE_ELEM_VOLTAGE_SRC` | 电压源 | 元件 |
| 12 | `ROUTE_ELEM_PWR_SUPPLY_2` | 电源_2型 | 元件 |
| 13 | `ROUTE_ELEM_RESISTOR_1` | 电阻_1型 | 元件 |
| 14 | `ROUTE_ELEM_RESISTOR_2` | 电阻_2型 | 元件 |
| 15 | `ROUTE_ELEM_FIXED_DIST` | 定距巡线 | 特殊（带距离参数） |
| 16 | `ROUTE_ELEM_DIODE_FORK_L` | 二极管_分叉_左 | 分叉 |
| 17 | `ROUTE_ELEM_DIODE_FORK_R` | 二级管_分叉_右 | 分叉 |
| 18 | `ROUTE_ELEM_LED` | 发光二极管 | 元件 |
| 19 | `ROUTE_ELEM_NOT_GATE` | 非门 | 元件 |
| 20 | `ROUTE_ELEM_TRANSISTOR_1` | 三极管_1型 | 元件 |
| 21 | `ROUTE_ELEM_TRANSISTOR_2` | 三极管_2型 | 元件 |
| 22 | `ROUTE_ELEM_DIODE_FORK_L_TURN` | 二极管_分叉_左_转弯 | 分叉+转弯 |
| 23 | `ROUTE_ELEM_DIODE_FORK_R_TURN` | 二级管_分叉_右_转弯 | 分叉+转弯 |
| 24 | `ROUTE_ELEM_TRANSISTOR_SIDE` | 侧进三极管 | 特殊行进方式 |
| 25 | `ROUTE_ELEM_DIODE2_FORK_L` | 二极管2型_分叉_左 | 分叉 |
| 26 | `ROUTE_ELEM_DIODE2_FORK_R` | 二极管2型_分叉_右 | 分叉 |
| 27 | `ROUTE_ELEM_DIODE2_FORK_L_TURN` | 二极管2型_分叉_左_转弯 | 分叉+转弯 |
| 28 | `ROUTE_ELEM_DIODE2_FORK_R_TURN` | 二极管2型_分叉_右_转弯 | 分叉+转弯 |
| 29 | `ROUTE_ELEM_DIODE3_FORK_L` | 二极管3型_分叉_左 | 分叉 |
| 30 | `ROUTE_ELEM_DIODE3_FORK_R` | 二极管3型_分叉_右 | 分叉 |
| 31 | `ROUTE_ELEM_DIODE3_FORK_L_TURN` | 二极管3型_分叉_左_转弯 | 分叉+转弯 |
| 32 | `ROUTE_ELEM_DIODE3_FORK_R_TURN` | 二极管3型_分叉_右_转弯 | 分叉+转弯 |

> 命名保留源码原样（含「二级管」这一处错别字），以便与代码逐字对应。

### 2.2 分类含义（合计 30 = 16 单符号 + 1 定距 + 13 分叉/转弯/侧进）

- **单符号元件（16 种）**：值 3–14 与 18–21——按键、灯泡、电池×2、电感×2、电流源、电容、
  电压源、电源_2型、电阻×2、发光二极管、非门、三极管×2。车辆正常压过即可，无需选边。
- **定距巡线（1 种，值 15）**：唯一带**距离参数**的元素——`route_plan_entry_t` 的
  `distance_mm` 与 `distance_mode` **只对该类型生效**，其余路线项忽略这两个字段。
- **分叉 / 转弯 / 侧进（13 种）**：
  - 二极管 1 型分叉：值 16、17（左 / 右）
  - 二极管 2 型分叉：值 22、23（左 / 右，均带转弯）
  - 侧进三极管：值 24
  - 二极管 3 型分叉：值 25、26（左 / 右）
  - 二极管 3 型分叉+转弯：值 27、28（左 / 右）
  - 编号 4 组分叉：值 29、30（左 / 右）
  - 编号 4 组分叉+转弯：值 31、32（左 / 右）

  这些元素要求在分叉点**选边**；带「转弯」的变体还要求选边后立即转向。

> **划分自检**（逐值枚举核对，无重叠无遗漏）：
> 16（值 3–14、18–21）+ 1（值 15）+ 13（值 16、17、22–32）= **30** ✓
> 与 `ROUTE_ELEM_COUNT = 30` 及 2.1 节的 30 行完全一致。

### 2.3 元素编号与「是否为元素」的判定

`route_plan.h` 提供两个辅助函数，判定逻辑集中在 `route_plan.c`：

| 函数 | 语义 |
|---|---|
| `route_plan_is_element(turn)` | 该路线项是否为元件类路线（区别于普通直行/左右转） |
| `route_plan_get_element_index(turn)` | 取元件编号（1..19）；非元件返回 0 |

**注意**：`get_element_index` 返回的是**元件编号**（另一套 1..19 的编号），
与枚举值（3..32）不是同一个编码体系。改动元素表时两者必须同步。

## 3. 感知层：16 路灰度

### 3.1 硬件

16 路灰度/光电管经 **74HC4067** 16 通道模拟开关选通后送入 **ADC0_CH6**——
即 16 路共用一根 ADC 线，靠 4 位地址线（S0–S3）+ 使能（/EN）分时选通。

引脚见 [pinout.md](pinout.md)。

### 3.2 采样

`user/line_follow.c` 的采样入口按通道数分两条路径（源码 `line_follow.c:1155`）：

```c
#if (LINE_FOLLOW_CH_COUNT == GRAY_4067_DEFAULT_CHANNEL_COUNT)
    gray_4067_read_12_raw(frame->raw);                              // 批量读路径
#else
    frame->raw[i] = gray_4067_read_raw(GRAY_4067_CH12_START_CHANNEL + i);  // 逐通道读路径
#endif
```

`LINE_FOLLOW_CH_COUNT` 决定走哪条。批量读路径减少选通/建立时间开销，
是提高帧率的主要手段（历史提交「灰度读取提速」即改此处）。

### 3.3 线位置解算

从 16 路二值化结果估算线在传感器阵列上的横向位置（`line_follow_estimate_local_peak_pos` 等）。
配套阈值经参数系统暴露（见 [tuning.md](tuning.md)），可在线调整。

**丢线恢复**：当有效通道数不足时进入恢复流程；恢复策略与阈值同样是可调参数
（历史提交「tighten lost-line recovery」即调此处）。

## 4. 判定层：路口与元素

判定在 `line_follow.c` 内完成，主要函数：

| 函数 | 职责 |
|---|---|
| `line_follow_sample_frame` | 组织一帧传感器数据（含批量读与逐通道读两条路径） |
| `line_follow_update` | 主更新：解算位置 → 路口判定 → 元素状态机推进 |
| `line_follow_handle_follow_stage` | 巡线行进阶段的处理（最大函数，173 行） |
| `line_follow_handle_element_event` | 元素事件到达时的处理 |
| `line_follow_handle_turn_stage` | 转向阶段的处理 |

判定依据主要是**横向线位置序列的形态**（例如线位置突变、出现多条候选线、
有效通道宽度的收窄/展宽），而非图像识别——传感器只有 16 个离散点。

## 5. 决策层：路线表

车辆按**预置路线序列**行驶，而非完全在线决策：

```c
typedef struct
{
    route_turn_t        turn;
    float               distance_mm;      // 仅 ROUTE_ELEM_FIXED_DIST 读取
    move_distance_mode_t distance_mode;   // 仅 ROUTE_ELEM_FIXED_DIST 生效
} route_plan_entry_t;
```

游标接口：

| 函数 | 语义 |
|---|---|
| `route_plan_set(plan, count)` | 设置路线序列（数组需常驻内存） |
| `route_plan_reset()` | 重置游标（每次开始运行前调用） |
| `route_plan_peek(&entry)` | 查看当前项但**不**推进游标 |
| `route_plan_commit()` | 提交当前项并推进游标 |
| `route_plan_has_next()` | 是否还有未执行项 |
| `route_plan_get_index()` | 已执行到的索引 |

**peek / commit 分离**的意义：动作执行成功后才推进游标，避免「动作失败但游标已前进」
导致后续元素全部错位。

## 6. 与里程控制的配合

元素动作的时长/距离由 `user/move_distance.c` 提供，支持两种模式
（`move_distance_mode_t`）：直线行走与巡线行走。
定距巡线元素通过 `distance_mode` 指定用哪种，通过 `distance_mm` 指定距离。

中心枢轴转向（原地转向）不依赖陀螺仪角度作为退出条件，而用**双轮里程计 + 线再检测**——
理由见 [architecture.md](architecture.md) 与 [tuning.md](tuning.md)。

## 7. 改元素表时的注意点

1. `route_turn_t` 枚举、`ROUTE_ELEM_COUNT`、`get_element_index` 的编号三者必须同步。
2. `ROUTE_ELEM_COUNT` 计的是**元素种类数**（30），不是枚举项总数（33）也不是最大枚举值。
   —— 历史上这里出过一次错：把 30 误改成 68，因为把它当成了路线条目数。
3. 新增带距离的元素需同时定义其 `distance_mode` 语义。
4. 判定阈值改动属 **R2**；新增元素状态机属 **R1**——见 [CONTRIBUTING.md](../CONTRIBUTING.md)。
