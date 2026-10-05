# 参数系统与调参流程

> 本文基于分支 `feat/20260711-encoder-turn-exit` 的快照。
> 事实来源：`user/params_flash.h/.c`、`user/menu.h`、`user/wifi_service.c`。

## 1. 设计目标

智能车的多数「手感」取决于几十个浮点参数（转向 PID、灰度阈值、加减速上限……）。
把这些参数**编译进固件**意味着每改一个值都要重新编译下载——赛场上不可接受。

因此工程实现了统一的参数系统：

- 参数有**名字**，可通过 WiFi 文本协议在线读写；
- 参数写入后可**持久化到 Flash**，断电不丢；
- 参数可**绑定到菜单页**，在屏上也能改；
- 参数可带**联动回调**，写入后自动做必要的重算。

## 2. 描述符结构

`user/params_flash.h` 的 `params_descriptor_t` 是参数系统的核心：

```c
typedef struct
{
    const char *key;          // WiFi 文本协议参数名
    const char *label;        // 菜单/上位机显示名
    params_value_type_t type; // 数值类型
    volatile void *value_ptr; // 指向 float / uint8_t
    float min_value;
    float max_value;
    float step;
    uint8_t decimals;         // 浮点显示精度
    uint16_t page_mask;       // 所属页面位图，bit = 1 << menu_state_t
    uint16_t flash_index;     // Flash 映射槽位；NONE 表示不持久化
    void (*apply)(void);      // 参数写入后的联动处理
} params_descriptor_t;
```

支持的类型（`params_value_type_t`）：

| 值 | 枚举 | 说明 |
|---:|---|---|
| 0 | `PARAMS_VALUE_FLOAT` | 浮点，受 `min/max/step/decimals` 约束与格式化 |
| 1 | `PARAMS_VALUE_U8` | 8 位无符号整数 |
| 2 | `PARAMS_VALUE_BOOL` | 布尔 |

**关键字段语义**：

- `value_ptr` 直接指向**业务代码里的那个变量**——不存在副本，改参数即改行为，无需同步。
- `page_mask` 用位图表示「本参数出现在哪些菜单页」。一页可含多个参数，一个参数也可属多页。
- `flash_index` 为 `PARAMS_FLASH_INDEX_NONE`（`0xFFFF`）时该参数**不持久化**——
  适合仅调试期使用的临时参数。
- `apply` 回调用于「改了这个值还需要做点别的」的场景，例如改了滤波系数后要重置滤波器状态。

## 3. Flash 持久化

来源：`user/params_flash.c`

| 常量 | 值 | 含义 |
|---|---|---|
| `PARAMS_FLASH_MAGIC` | `0x50415234` | 数据有效性标识（ASCII "PAR4"） |
| `PARAMS_FLASH_SECTOR_NUM` | 0 | 使用 Flash 扇区 0 |
| `PARAMS_FLASH_PAGE_INDEX` | 11 | 页索引 |
| `PARAMS_FLASH_CRC_SLOT` | 128 | 校验值存放槽位 |

**流程**：

| 函数 | 语义 |
|---|---|
| `params_load_from_flash()` | 上电时读 Flash，校验 magic 与 CRC 后把值写回 `value_ptr` |
| `params_save_to_flash()` | 把当前参数按 `flash_index` 写入 Flash |

**失效处理**：magic 或 CRC 不匹配时视为无有效数据，参数保持代码里的默认值。
因此**改参数默认值只需改代码里的初值**，Flash 里的旧值要么被 CRC 拦下，要么在下次保存时覆盖。

> ⚠️ 若改了参数**数量或顺序**导致 `flash_index` 语义变化，旧 Flash 数据会与新代码错位。
> 这类改动应同时**提升 magic 值**，让旧数据自动失效。

## 4. 菜单页

`user/menu.h` 的 `menu_state_t` 共 10 个状态：

| 值 | 枚举 | 用途 |
|---:|---|---|
| 0 | `MENU_MAIN` | 主界面 |
| 1 | `MENU_RUN` | 运行模式（比赛主逻辑入口） |
| 2 | `MENU_FOC_SPEED_DEBUG` | FOC 转速调试 |
| 3 | `MENU_LINE_FOLLOW_DEBUG` | 巡线调试 |
| 4 | `MENU_JUNCTION_DEBUG` | 路口/元素调试 |
| 5 | `MENU_GRAY_DEBUG` | 灰度传感器调试 |
| 6 | `MENU_IMU_DEBUG` | 姿态传感器调试 |
| 7 | `MENU_NEG_PRESSURE_DEBUG` | 负压电机调试 |
| 8 | `MENU_BLDC_OPEN_LOOP_TEST` | 无刷电机开环测试 |
| 9 | `MENU_WIFI_CONNECT` | WiFi 连接 |

**页面与上位机联动**：`menu_get_page_name()` 与 `menu_get_state_from_page_name()`
在枚举与字符串之间互转，供「上位机切换板端页面」的协议使用（`0x10` 命令）。
改枚举名会破坏该协议，须同步上位机。

## 5. 调参方式

### 5.1 经 WiFi 上位机（推荐）

1. 填写 `user/wifi_cfg.h` 的 `YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD`；
2. 启动上位机：`python user/pc_wifi_uart_plot_qt.py`；
3. 在参数面板按 `key` 名读写；
4. 改完调 `params_save_to_flash()`（或面板上的保存命令）持久化。

参数读写走 `wifi_service_try_handle_registered_param()`：
已注册参数的 `key` 会被加入可识别关键字集合，因此**新增参数即自动获得 WiFi 可调能力**，
无需改 `wifi_service.c`。

### 5.2 经板端菜单

按 `page_mask` 定位到所属页面，用 4 个按键在屏上增减。
适合赛场无 PC 时的微调。

## 6. 建议的调参顺序

由内到外，**每步只动一类参数**，否则无法归因：

1. **FOC 基础**：先确认电角度与转向正确（`MENU_FOC_SPEED_DEBUG`、开环测试页）。
   加减速上限 `FOC_MAX_ACCEL_RPM_PS` / `FOC_MAX_DECEL_RPM_PS` 属 PI 层，先设保守值。
2. **速度环**：看 `0x01` 帧的双轮**目标 / 实际**转速曲线是否跟随。
3. **灰度与线位置**：`MENU_GRAY_DEBUG` 看 16 路阈值，确认线位置解算稳定。
4. **转向 PID**：先 P，再 D，最后少量 I。观察 `0x02` 帧的 `line_pos` 与 `turn_delta_rpm`。
5. **元素判定阈值**：路口/元素进入的计数与锁定阈值，用 `0x02` 帧的
   `junction_enter_cnt` / `element_enter_cnt` 判断是否误触发。
6. **丢线恢复**：`lost_line_cnt` 与恢复参数。
7. **直道加速**：`STR_BST` 一类参数最后调，因为它会掩盖前面的问题。

## 7. 调参时的注意点

1. **一次只改一个参数**，并记录改动前后的现象——参数之间存在耦合
   （例如提高转向 P 会放大丢线概率）。
2. **改完先验证再持久化**：Flash 有擦写寿命，不要每改一次就保存。
3. **`apply` 回调可能重置状态**：某些参数写入会触发滤波器/积分器复位，
   行进中改动可能导致瞬间异常，建议停车后再改。
4. **不要依赖 WiFi 在线**：调参只是调试手段。比赛运行路径不得依赖任何参数下发，
   最终参数必须已持久化到 Flash。
5. 参数**默认值写在代码里**（`params_flash.c` 的描述符表）。赛事备份时的真实参数
   需要用 `params_save_to_flash()` 固化，并确认 Flash 数据与代码版本匹配。
