/*********************************************************************************************************************
* 文件：move_distance.c
* 模块：应用层（第 6 层）—— 里程控制
* 职责：里程计：直线与巡线两种行走模式及到位判定
* 依赖：zf_common_headfile.h、本目录 move_distance.h
* 风险等级：R2
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "move_distance.h"

#include <math.h>

#include "isr.h"

#ifndef MOVE_DISTANCE_PI
#define MOVE_DISTANCE_PI (3.14159265358979323846f)
#endif

typedef struct
{
    bool active;
    bool done;

    move_distance_mode_t mode;

    float speed_rpm;
    float target_mm;
    float traveled_mm;

    bool left_inited;
    bool right_inited;
    float last_mech_l;
    float last_mech_r;
} move_distance_ctx_t;

static volatile move_distance_ctx_t g_md;

typedef struct
{
    float traveled_mm;
    bool left_inited;
    bool right_inited;
    float last_mech_l;
    float last_mech_r;
    bool counter_active;
    float counter_left_mm;
    float counter_right_mm;
} move_distance_odom_ctx_t;

static volatile move_distance_odom_ctx_t g_md_odom;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度 wrap 到 (-π, π]
// 参数说明     rad             输入角度（rad）
// 返回参数     float           wrap 后角度（rad）
// 使用示例     md_wrap_pm_pi(rad);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------

static float md_wrap_pm_pi(float rad)
{
    while (rad > MOVE_DISTANCE_PI) rad -= 2.0f * MOVE_DISTANCE_PI;
    while (rad < -MOVE_DISTANCE_PI) rad += 2.0f * MOVE_DISTANCE_PI;
    return rad;
}

static float md_forward_rev_from_mech_delta(float mech_delta_rad,
                                            const foc_speed_loop_t *loop,
                                            float user_sign)
{
    float rev = mech_delta_rad / (2.0f * MOVE_DISTANCE_PI);

    if ((loop != NULL) && (loop->angle_cfg.direction < 0))
    {
        rev = -rev;
    }

    return rev * user_sign;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     里程计复位
// 返回参数     void
// 使用示例     move_distance_odom_reset();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void move_distance_odom_reset(void)
{
    g_md_odom.traveled_mm = 0.0f;
    g_md_odom.left_inited = false;
    g_md_odom.right_inited = false;
    g_md_odom.last_mech_l = 0.0f;
    g_md_odom.last_mech_r = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     里程计更新（不依赖定距任务）
// 参数说明     mech_l_rad      左轮机械角（rad）
// 参数说明     mech_l_valid    左轮角度是否有效
// 参数说明     mech_r_rad      右轮机械角（rad）
// 参数说明     mech_r_valid    右轮角度是否有效
// 返回参数     void
// 使用示例     move_distance_odom_update(ml, ml_ok, mr, mr_ok);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void move_distance_odom_update(float mech_l_rad, bool mech_l_valid,
                               float mech_r_rad, bool mech_r_valid)
{
    float mm_delta_sum = 0.0f;
    float left_mm_delta = 0.0f;
    float right_mm_delta = 0.0f;
    uint8_t used = 0u;
    bool left_delta_valid = false;
    bool right_delta_valid = false;

    if (mech_l_valid)
    {
        if (!g_md_odom.left_inited)
        {
            g_md_odom.last_mech_l = mech_l_rad;
            g_md_odom.left_inited = true;
        }
        else
        {
            float d = md_wrap_pm_pi(mech_l_rad - g_md_odom.last_mech_l);
            g_md_odom.last_mech_l = mech_l_rad;
            left_mm_delta = md_forward_rev_from_mech_delta(d, &g_loop_l, (float)FOC_LEFT_USER_RPM_SIGN)
                          * (float)MOVE_DISTANCE_MM_PER_MOTOR_REV;
            mm_delta_sum += left_mm_delta;
            left_delta_valid = true;
            used++;
        }
    }
    else
    {
        g_md_odom.left_inited = false;
    }

    if (mech_r_valid)
    {
        if (!g_md_odom.right_inited)
        {
            g_md_odom.last_mech_r = mech_r_rad;
            g_md_odom.right_inited = true;
        }
        else
        {
            float d = md_wrap_pm_pi(mech_r_rad - g_md_odom.last_mech_r);
            g_md_odom.last_mech_r = mech_r_rad;
            right_mm_delta = md_forward_rev_from_mech_delta(d, &g_loop_r, (float)FOC_RIGHT_USER_RPM_SIGN)
                           * (float)MOVE_DISTANCE_MM_PER_MOTOR_REV;
            mm_delta_sum += right_mm_delta;
            right_delta_valid = true;
            used++;
        }
    }
    else
    {
        g_md_odom.right_inited = false;
    }

    if (g_md_odom.counter_active)
    {
        if (left_delta_valid)
        {
            g_md_odom.counter_left_mm += left_mm_delta;
        }
        if (right_delta_valid)
        {
            g_md_odom.counter_right_mm += right_mm_delta;
        }
    }

    if (used == 0u)
    {
        return;
    }

    mm_delta_sum /= (float)used;
    g_md_odom.traveled_mm += mm_delta_sum;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取里程计累计距离
// 返回参数     float           已行驶距离（mm）
// 使用示例     move_distance_odom_get_traveled_mm();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float move_distance_odom_get_traveled_mm(void)
{
    return g_md_odom.traveled_mm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     独立人工里程计开始计数（清零并启动）
// 返回参数     void
// 使用示例     move_distance_counter_start();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void move_distance_counter_start(void)
{
    g_md_odom.counter_left_mm = 0.0f;
    g_md_odom.counter_right_mm = 0.0f;
    g_md_odom.counter_active = true;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     独立人工里程计清零并停止
// 返回参数     void
// 使用示例     move_distance_counter_clear();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void move_distance_counter_clear(void)
{
    g_md_odom.counter_active = false;
    g_md_odom.counter_left_mm = 0.0f;
    g_md_odom.counter_right_mm = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询独立人工里程计是否正在计数
// 返回参数     bool
// 使用示例     move_distance_counter_is_active();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
bool move_distance_counter_is_active(void)
{
    return g_md_odom.counter_active;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取独立人工里程计左轮累计距离
// 返回参数     float
// 使用示例     move_distance_counter_get_left_mm();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float move_distance_counter_get_left_mm(void)
{
    return g_md_odom.counter_left_mm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取独立人工里程计右轮累计距离
// 返回参数     float
// 使用示例     move_distance_counter_get_right_mm();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float move_distance_counter_get_right_mm(void)
{
    return g_md_odom.counter_right_mm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动定距直行
// 参数说明     speed_rpm        目标速度（RPM）
// 参数说明     distance_mm      目标距离（mm）
// 返回参数     void
// 使用示例     move_distance_start_mm(200.0f, 500.0f);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void move_distance_start_mm(float speed_rpm, float distance_mm)
{
    if (distance_mm <= 0.0f)
    {
        move_distance_stop();
        return;
    }

    g_md.active = true;
    g_md.done = false;
    g_md.mode = MOVE_DISTANCE_MODE_STRAIGHT;
    g_md.speed_rpm = speed_rpm;
    g_md.target_mm = distance_mm;
    g_md.traveled_mm = 0.0f;
    g_md.left_inited = false;
    g_md.right_inited = false;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动定距循迹
// 参数说明     speed_rpm        目标速度（RPM）
// 参数说明     distance_mm      目标距离（mm）
// 返回参数     void
// 使用示例     move_distance_start_mm_line_follow(200.0f, 500.0f);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void move_distance_start_mm_line_follow(float speed_rpm, float distance_mm)
{
    if (distance_mm <= 0.0f)
    {
        move_distance_stop();
        return;
    }

    g_md.active = true;
    g_md.done = false;
    g_md.mode = MOVE_DISTANCE_MODE_LINE_FOLLOW;
    g_md.speed_rpm = speed_rpm;
    g_md.target_mm = distance_mm;
    g_md.traveled_mm = 0.0f;
    g_md.left_inited = false;
    g_md.right_inited = false;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止定距功能并清状态
// 返回参数     void
// 使用示例     move_distance_stop();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void move_distance_stop(void)
{
    g_md.active = false;
    g_md.done = false;
    g_md.mode = MOVE_DISTANCE_MODE_STRAIGHT;
    g_md.speed_rpm = 0.0f;
    g_md.target_mm = 0.0f;
    g_md.traveled_mm = 0.0f;
    g_md.left_inited = false;
    g_md.right_inited = false;
    g_md.last_mech_l = 0.0f;
    g_md.last_mech_r = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询定距是否处于激活状态
// 返回参数     bool            true 激活 / false 未激活
// 使用示例     if (move_distance_is_active()) { ... }
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
bool move_distance_is_active(void)
{
    return g_md.active;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询定距是否完成
// 返回参数     bool            true 完成 / false 未完成
// 使用示例     if (move_distance_is_done()) { ... }
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
bool move_distance_is_done(void)
{
    return g_md.done;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前定距模式
// 返回参数     move_distance_mode_t  模式枚举
// 使用示例     move_distance_get_mode();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
move_distance_mode_t move_distance_get_mode(void)
{
    return g_md.mode;
}

void move_distance_set_mode(move_distance_mode_t mode)
{
    g_md.mode = mode;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取目标距离（mm）
// 返回参数     float           目标距离（mm）
// 使用示例     move_distance_get_target_mm();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float move_distance_get_target_mm(void)
{
    return g_md.target_mm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取已行驶距离（mm）
// 返回参数     float           已行驶距离（mm）
// 使用示例     move_distance_get_traveled_mm();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float move_distance_get_traveled_mm(void)
{
    return g_md.traveled_mm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     定距状态更新
// 参数说明     mech_l_rad      左轮机械角（rad）
// 参数说明     mech_l_valid    左轮角度是否有效
// 参数说明     mech_r_rad      右轮机械角（rad）
// 参数说明     mech_r_valid    右轮角度是否有效
// 参数说明     dt_s            采样周期（s）
// 返回参数     bool            true 本次更新触发“完成” / false 未完成
// 使用示例     move_distance_update(ml, ml_ok, mr, mr_ok, dt);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
bool move_distance_update(float mech_l_rad, bool mech_l_valid,
                          float mech_r_rad, bool mech_r_valid,
                          float dt_s)
{
    (void)dt_s;

    if (!g_md.active)
    {
        return false;
    }

    float rev_delta = 0.0f;
    uint8_t used = 0u;

    if (mech_l_valid)
    {
        if (!g_md.left_inited)
        {
            g_md.last_mech_l = mech_l_rad;
            g_md.left_inited = true;
        }
        else
        {
            float d = md_wrap_pm_pi(mech_l_rad - g_md.last_mech_l);
            g_md.last_mech_l = mech_l_rad;
            rev_delta += md_forward_rev_from_mech_delta(d, &g_loop_l, (float)FOC_LEFT_USER_RPM_SIGN);
            used++;
        }
    }
    else
    {
        g_md.left_inited = false;
    }

    if (mech_r_valid)
    {
        if (!g_md.right_inited)
        {
            g_md.last_mech_r = mech_r_rad;
            g_md.right_inited = true;
        }
        else
        {
            float d = md_wrap_pm_pi(mech_r_rad - g_md.last_mech_r);
            g_md.last_mech_r = mech_r_rad;
            rev_delta += md_forward_rev_from_mech_delta(d, &g_loop_r, (float)FOC_RIGHT_USER_RPM_SIGN);
            used++;
        }
    }
    else
    {
        g_md.right_inited = false;
    }

    if (used == 0u)
    {
        return false;
    }

    rev_delta /= (float)used;

    g_md.traveled_mm += rev_delta * (float)MOVE_DISTANCE_MM_PER_MOTOR_REV;

    if (fabsf(g_md.traveled_mm) >= g_md.target_mm)
    {
        g_md.active = false;
        g_md.done = true;
        return true;
    }

    return false;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取定距期望速度指令
// 参数说明     base_rpm_out         基础速度输出指针（RPM）
// 参数说明     delta_rpm_out        差速输出指针（RPM）
// 参数说明     override_delta_out   是否强制覆盖差速输出
// 返回参数     bool            true 有效 / false 无效（未激活）
// 使用示例     move_distance_get_cmd(&base, &delta, &override_delta);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
bool move_distance_get_cmd(float *base_rpm_out, float *delta_rpm_out, bool *override_delta_out)
{
    if (!g_md.active)
    {
        return false;
    }

    if (base_rpm_out) *base_rpm_out = g_md.speed_rpm;
    if (g_md.mode == MOVE_DISTANCE_MODE_STRAIGHT)
    {
        if (override_delta_out) *override_delta_out = true;
        if (delta_rpm_out) *delta_rpm_out = 0.0f;
    }
    else
    {
        if (override_delta_out) *override_delta_out = false;
    }
    return true;
}
