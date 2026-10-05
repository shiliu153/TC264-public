#ifndef _ROUTE_PLAN_H_
#define _ROUTE_PLAN_H_
/*********************************************************************************************************************
* 文件：route_plan.h
* 模块：应用层（第 6 层）—— 路线规划
* 职责：30 种电路元素类型枚举与路线表声明
* 依赖：zf_common_typedef.h
* 风险等级：R1
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "move_distance.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ROUTE_TURN_STRAIGHT = 0,
    ROUTE_TURN_LEFT     = 1,
    ROUTE_TURN_RIGHT    = 2,
    ROUTE_ELEM_BUTTON       = 3,  // 按键
    ROUTE_ELEM_LAMP         = 4,  // 灯泡
    ROUTE_ELEM_BATTERY_1    = 5,  // 电池_1型
    ROUTE_ELEM_BATTERY_2    = 6,  // 电池_2型
    ROUTE_ELEM_INDUCTOR_1   = 7,  // 电感_1型
    ROUTE_ELEM_INDUCTOR_2   = 8,  // 电感_2型
    ROUTE_ELEM_CURRENT_SRC  = 9,  // 电流源
    ROUTE_ELEM_CAPACITOR    = 10, // 电容
    ROUTE_ELEM_VOLTAGE_SRC  = 11, // 电压源
    ROUTE_ELEM_PWR_SUPPLY_2 = 12, // 电源_2型
    ROUTE_ELEM_RESISTOR_1   = 13, // 电阻_1型
    ROUTE_ELEM_RESISTOR_2   = 14, // 电阻_2型
    ROUTE_ELEM_FIXED_DIST   = 15, // 定距巡线
    ROUTE_ELEM_DIODE_FORK_L = 16, // 二极管_分叉_左
    ROUTE_ELEM_DIODE_FORK_R = 17, // 二级管_分叉_右
    ROUTE_ELEM_LED          = 18, // 发光二极管
    ROUTE_ELEM_NOT_GATE     = 19, // 非门
    ROUTE_ELEM_TRANSISTOR_1       = 20, // 三极管_1型
    ROUTE_ELEM_TRANSISTOR_2       = 21, // 三极管_2型
    ROUTE_ELEM_DIODE_FORK_L_TURN  = 22, // 二极管_分叉_左_转弯
    ROUTE_ELEM_DIODE_FORK_R_TURN  = 23, // 二级管_分叉_右_转弯
    ROUTE_ELEM_TRANSISTOR_SIDE    = 24, // 侧进三极管
    ROUTE_ELEM_DIODE2_FORK_L      = 25, // 二极管2型_分叉_左
    ROUTE_ELEM_DIODE2_FORK_R      = 26, // 二极管2型_分叉_右
    ROUTE_ELEM_DIODE2_FORK_L_TURN = 27, // 二极管2型_分叉_左_转弯
    ROUTE_ELEM_DIODE2_FORK_R_TURN = 28, // 二极管2型_分叉_右_转弯
    ROUTE_ELEM_DIODE3_FORK_L      = 29, // 二极管3型_分叉_左
    ROUTE_ELEM_DIODE3_FORK_R      = 30, // 二极管3型_分叉_右
    ROUTE_ELEM_DIODE3_FORK_L_TURN = 31, // 二极管3型_分叉_左_转弯
    ROUTE_ELEM_DIODE3_FORK_R_TURN = 32, // 二极管3型_分叉_右_转弯
} route_turn_t;

enum
{
    ROUTE_ELEM_COUNT = 30u,
};

typedef struct
{
    route_turn_t turn;
    float distance_mm;                  // 仅 ROUTE_ELEM_FIXED_DIST 读取该字段，其他路线项忽略。
    move_distance_mode_t distance_mode; // 仅 ROUTE_ELEM_FIXED_DIST 生效。
} route_plan_entry_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置路线序列
// 参数说明     plan            路线项数组指针
// 参数说明     count           路线长度
// 返回参数     void
// 使用示例     route_plan_set(g_route_plan, sizeof(g_route_plan) / sizeof(g_route_plan[0]));
// 备注信息     路线项需常驻内存；ROUTE_ELEM_FIXED_DIST 作为“指定距离元素”时才读取 distance_mm 与 distance_mode
//-------------------------------------------------------------------------------------------------------------------
void route_plan_set(const route_plan_entry_t *plan, uint8_t count);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置路线游标
// 返回参数     void
// 使用示例     route_plan_reset();
// 备注信息     建议在每次开始运行前调用
//-------------------------------------------------------------------------------------------------------------------
void route_plan_reset(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询是否还有未执行的路线项
// 返回参数     bool            true-仍有下一项 false-路线结束
//-------------------------------------------------------------------------------------------------------------------
bool route_plan_has_next(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查看当前路线项但不推进游标
// 参数说明     out_entry       输出路线项
// 返回参数     bool            true-成功 false-无下一项或输出指针为空
//-------------------------------------------------------------------------------------------------------------------
bool route_plan_peek(route_plan_entry_t *out_entry);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     提交当前路线项并推进游标
// 返回参数     bool            true-提交成功 false-无可提交项
//-------------------------------------------------------------------------------------------------------------------
bool route_plan_commit(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前路线索引
// 返回参数     uint8_t         已执行到的索引（从 0 开始）
//-------------------------------------------------------------------------------------------------------------------
uint8_t route_plan_get_index(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断路线项是否为元件类路线
// 参数说明     turn            路线枚举
// 返回参数     bool            true-元件路线 false-普通转向
//-------------------------------------------------------------------------------------------------------------------
bool route_plan_is_element(route_turn_t turn);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取元件编号（1..19）
// 参数说明     turn            路线枚举
// 返回参数     uint8_t         元件编号；非元件返回 0
//-------------------------------------------------------------------------------------------------------------------
uint8_t route_plan_get_element_index(route_turn_t turn);

#ifdef __cplusplus
}
#endif

#endif
