/*********************************************************************************************************************
* FOC/SVPWM 最小数学模块
*********************************************************************************************************************/

#include "foc_svpwm.h"

#include <math.h>

#include "Ifx_LutSincosF32.h"

#include "zf_device/zf_device_as5047p.h"
#include "bldc_drv8300_3pwm.h"
#include "pid.h"
#include "zf_driver/zf_driver_delay.h"

#ifndef FOC_PI
#define FOC_PI (3.14159265358979323846f)
#endif

#pragma section code "cpu1_psram"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将角度 wrap 到 [0, 2π)
// 参数说明     x               输入角度（rad）
// 返回参数     float           wrap 后的角度（rad）
// 使用示例     foc_wrap_0_2pi(theta);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------

float foc_wrap_0_2pi(float x)
{
    float y = fmodf(x, FOC_TWO_PI);
    return (y < 0.0f) ? (y + FOC_TWO_PI) : y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点限幅
// 参数说明     x               输入值
// 参数说明     lo              下限
// 参数说明     hi              上限
// 返回参数     float           限幅后的结果
// 使用示例     foc_clampf(u, -1.0f, 1.0f);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_clampf(float x, float lo, float hi)
{
    return clamp(x, lo, hi);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将角度 wrap 到 (-π, π]
// 参数说明     x               输入角度（rad）
// 返回参数     float           wrap 后的角度（rad）
// 使用示例     foc_wrap_pm_pi(delta);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_wrap_pm_pi(float x)
{
    // wrap 到 (-pi, pi] (已知 x 在 (-2pi, 2pi) 范围内，用 if 代替 while 提升速度)
    const float pi = FOC_PI;
    const float two_pi = FOC_TWO_PI;
    if(x >  pi)  x -= two_pi;
    if(x <= -pi) x += two_pi;
    return x;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     机械角速度估算（rad/s）
// 参数说明     loop            速度环上下文
// 参数说明     mech_rad        当前机械角（rad）
// 参数说明     dt              采样周期（s）
// 返回参数     float           估算得到的角速度（rad/s）
// 使用示例     foc_speed_estimate_radps(&loop, mech, dt);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_speed_estimate_radps(foc_speed_loop_t *loop, float mech_rad, float dt)
{
    if((!loop) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    if(!loop->has_last)
    {
        loop->last_mech_rad = mech_rad;
        loop->has_last = 1;
        return 0.0f;
    }

    float d = foc_wrap_pm_pi(mech_rad - loop->last_mech_rad);
    loop->last_mech_rad = mech_rad;

    float omega = d * 1000.0f;

    // Keep speed feedback sign consistent with the calibrated control direction.
    if(loop->angle_cfg.direction < 0)
    {
        omega = -omega;
    }

#if FOC_SPEED_EST_GLITCH_ENABLE
    {
        float abs_ref = (loop->desired_omega_m >= 0.0f) ? loop->desired_omega_m : -loop->desired_omega_m;
        float abs_target = (loop->target_omega_m >= 0.0f) ? loop->target_omega_m : -loop->target_omega_m;
        float limit_rpm;
        float limit_radps;

        if(abs_target > abs_ref)
        {
            abs_ref = abs_target;
        }

        limit_rpm = abs_ref / FOC_RPM_TO_RADPS;
        limit_rpm = limit_rpm * FOC_SPEED_EST_GLITCH_SCALE + FOC_SPEED_EST_GLITCH_MARGIN_RPM;
        limit_rpm = foc_clampf(limit_rpm, FOC_SPEED_EST_GLITCH_MIN_RPM, FOC_SPEED_EST_GLITCH_MAX_RPM);
        limit_radps = limit_rpm * FOC_RPM_TO_RADPS;

        if((omega > limit_radps) || (omega < -limit_radps))
        {
            loop->has_last = 0;
            return 0.0f;
        }
    }
#endif

    return omega;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度 PI 单步计算
// 参数说明     loop            速度环上下文
// 参数说明     dt              控制周期（s）
// 返回参数     float           vq 输出（归一化/限幅后）
// 使用示例     foc_speed_pi_step(&loop, dt);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_speed_pi_step(foc_speed_loop_t *loop, float dt)
{
    if((!loop) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    // --- 软启动 / 目标速度斜率限制 (Slew Rate Limiter) ---
    // 加速/减速使用不对称限幅：制动时可以更快降速，起步时平顺加速
    {
        const float accel_step = FOC_MAX_ACCEL_RPM_PS * FOC_RPM_TO_RADPS * dt;
        const float decel_step = FOC_MAX_DECEL_RPM_PS * FOC_RPM_TO_RADPS * dt;
        const float delta = loop->desired_omega_m - loop->target_omega_m;

        if (delta > accel_step) {
            loop->target_omega_m += accel_step;
        } else if (delta < -decel_step) {
            loop->target_omega_m -= decel_step;
        } else {
            loop->target_omega_m = loop->desired_omega_m;
        }
    }
    // ---------------------------------------------------

    float err = loop->target_omega_m - loop->omega_m;

    // 引入死区，减小微小设定误差导致的抖动
    if (err > -FOC_SPEED_DEADZONE_RADPS && err < FOC_SPEED_DEADZONE_RADPS)
    {
        err = 0.0f;
    }

    float err_last = loop->speed_pid.error;

    loop->speed_pid.last_error = err_last;
    loop->speed_pid.error = err;

    float du_p = loop->speed_pid.kp * (err - err_last);
    float du_i = loop->speed_pid.ki * err * dt;

    float integ_old = loop->speed_pid.integ;
    float integ_new = foc_clampf(integ_old + du_i, -loop->vq_limit, loop->vq_limit);
    float du_i_eff = integ_new - integ_old;

    float u_candidate = loop->vq_out + du_p + du_i_eff;
    float u_sat = foc_clampf(u_candidate, -loop->vq_limit, loop->vq_limit);
    if(u_candidate != u_sat)
    {
        if(((u_candidate > u_sat) && (du_i_eff > 0.0f)) || ((u_candidate < u_sat) && (du_i_eff < 0.0f)))
        {
            integ_new = integ_old;
            du_i_eff = 0.0f;
            u_candidate = loop->vq_out + du_p;
            u_sat = foc_clampf(u_candidate, -loop->vq_limit, loop->vq_limit);
        }
    }

    loop->speed_pid.integ = integ_new;
    loop->vq_out = u_sat;

    return loop->vq_out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度环使能时复位内部状态
// 参数说明     loop            速度环上下文
// 返回参数     void
// 使用示例     foc_speed_loop_reset_on_enable(&loop);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void foc_speed_loop_reset_on_enable(foc_speed_loop_t *loop)
{
    if(!loop)
    {
        return;
    }

    loop->has_last = 0;
    loop->last_mech_rad = 0.0f;
    loop->omega_m = 0.0f;
    
    loop->target_omega_m = 0.0f;
    loop->desired_omega_m = 0.0f;

    loop->vq_out = 0.0f;
    loop->speed_pid.integ = 0.0f;

    loop->speed_pid.error = 0.0f;
    loop->speed_pid.last_error = 0.0f;
}
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void foc_speed_loop_init_default(foc_speed_loop_t *loop)
{
    if(!loop)
    {
        return;
    }

    loop->angle_cfg.pole_pairs  = (uint16_t)FOC_DEFAULT_POLE_PAIRS;
    loop->angle_cfg.direction   = +1;
    loop->angle_cfg.elec_offset = 0.0f;
    loop->angle_cfg.elec_offset_fxp = 0;

    loop->target_omega_m = 0.0f;
    loop->desired_omega_m = 0.0f;
    loop->omega_m = 0.0f;
    loop->last_mech_rad = 0.0f;
    loop->has_last = 0;

    loop->speed_pid.kp = FOC_SPEED_KP;
    loop->speed_pid.ki = FOC_SPEED_KI;
    loop->speed_pid.kd = 0.0f;
    loop->speed_pid.integ = 0.0f;
    loop->speed_pid.error = 0.0f;
    loop->speed_pid.last_error = 0.0f;

    loop->vq_out = 0.0f;
    loop->vq_limit = foc_clampf(FOC_VQ_LIMIT_FRAC, 0.0f, 0.95f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置左右轮目标转速（RPM）
// 参数说明     loop_l          左轮速度环
// 参数说明     loop_r          右轮速度环
// 参数说明     left_rpm        左轮目标转速（RPM）
// 参数说明     right_rpm       右轮目标转速（RPM）
// 返回参数     void
// 使用示例     foc_speed_control_set_target_rpm(&loop_l, &loop_r, 200.0f, 200.0f);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void foc_speed_control_set_target_rpm(foc_speed_loop_t *loop_l, foc_speed_loop_t *loop_r, float left_rpm, float right_rpm)
{
    const float left_motor_rpm = foc_left_user_rpm_to_motor_rpm(left_rpm);
    const float right_motor_rpm = foc_right_user_rpm_to_motor_rpm(right_rpm);

    // RPM -> rad/s
    if(loop_l)
    {
        loop_l->desired_omega_m = left_motor_rpm * FOC_RPM_TO_RADPS;
    }
    if(loop_r)
    {
        loop_r->desired_omega_m = right_motor_rpm * FOC_RPM_TO_RADPS;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     SVPWM 上下文初始化
// 参数说明     ctx             SVPWM 上下文
// 参数说明     vbus            母线电压（V），<=0 时按 1.0 处理
// 参数说明     duty_max        PWM 最大计数（占空比满量程）
// 返回参数     void
// 使用示例     foc_svpwm_init(&svpwm, 1.0f, PWM_DUTY_MAX);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void foc_svpwm_init(foc_svpwm_t *ctx, float vbus, uint32_t duty_max)
{
    if(!ctx)
    {
        return;
    }

    ctx->vbus = (vbus <= 0.0f) ? 1.0f : vbus;
    ctx->inv_vbus = 1.0f / ctx->vbus;
    ctx->duty_max = (duty_max == 0u) ? 10000u : duty_max;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AS5047P 原始角度转机械弧度
// 参数说明     as5047p_angle_14bit  AS5047P 14bit 原始角度
// 返回参数     float           机械角（rad）
// 使用示例     foc_as5047p_raw_to_mech_rad(raw);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_as5047p_raw_to_mech_rad(uint16_t as5047p_angle_14bit)
{
    const float two_pi = 2.0f * FOC_PI;
    const float scale = two_pi / 16384.0f;
    return (float)(as5047p_angle_14bit & 0x3FFFu) * scale;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     机械角转电角
// 参数说明     mech_rad        机械角（rad）
// 参数说明     cfg             电角配置（极对数/方向/零偏）
// 返回参数     float           电角（rad，wrap 到 [0, 2π)）
// 使用示例     foc_mech_to_elec_rad(mech, &cfg);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_mech_to_elec_rad(float mech_rad, const foc_angle_cfg_t *cfg)
{
    if((!cfg) || (cfg->pole_pairs == 0u) || (cfg->direction == 0))
    {
        return foc_wrap_0_2pi(mech_rad);
    }

    float theta = (float)cfg->direction * (float)cfg->pole_pairs * mech_rad + cfg->elec_offset;
    return foc_wrap_0_2pi(theta);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电压 vdq 转三相 PWM 占空比（SVPWM）
// 参数说明     ctx             SVPWM 上下文
// 参数说明     theta_e_rad     电角（rad）
// 参数说明     vd              d 轴电压
// 参数说明     vq              q 轴电压
// 参数说明     duty_a          A 相占空比输出指针
// 参数说明     duty_b          B 相占空比输出指针
// 参数说明     duty_c          C 相占空比输出指针
// 返回参数     foc_svpwm_status_t  状态码
// 使用示例     foc_vdq_to_duty(&svpwm, theta, vd, vq, &da, &db, &dc);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
foc_svpwm_status_t foc_vdq_to_duty(const foc_svpwm_t *ctx,
                                  float theta_e_rad,
                                  float vd,
                                  float vq,
                                  uint32_t *duty_a,
                                  uint32_t *duty_b,
                                  uint32_t *duty_c)
{
    // Keep backward-compatible API: convert rad -> 12-bit fxp, then use fxp fast-path
    const foc_fxp_angle_t theta_fxp = (foc_fxp_angle_t)((uint32_t)IFX_LUT_F32_TO_FXPANGLE(theta_e_rad) & (uint32_t)FOC_ANGLE_FXP_MASK);
    return foc_vdq_to_duty_fxp(ctx, theta_fxp, vd, vq, duty_a, duty_b, duty_c);
}

foc_svpwm_status_t foc_vdq_to_duty_fxp(const foc_svpwm_t *ctx,
                                      foc_fxp_angle_t theta_e_fxp,
                                      float vd,
                                      float vq,
                                      uint32_t *duty_a,
                                      uint32_t *duty_b,
                                      uint32_t *duty_c)
{
    if((!ctx) || (!duty_a) || (!duty_b) || (!duty_c) || (ctx->duty_max == 0u))
    {
        return FOC_SVPWM_ERR_PARAM;
    }

    const Ifx_Lut_FxpAngle fxp_theta = (Ifx_Lut_FxpAngle)((uint32_t)theta_e_fxp & (uint32_t)FOC_ANGLE_FXP_MASK);
    const float sin_t = Ifx_LutSincosF32_sin(fxp_theta);
    const float cos_t = Ifx_LutSincosF32_sin((IFX_LUT_ANGLE_PI / 2) - fxp_theta);

    float mod_vd = vd * ctx->inv_vbus;
    float mod_vq = vq * ctx->inv_vbus;

    float valpha = mod_vd * cos_t - mod_vq * sin_t;
    float vbeta  = mod_vd * sin_t + mod_vq * cos_t;

    const float half_valpha = 0.5f * valpha;
    const float half_sqrt3_vbeta = FOC_SQRT3_DIV2 * vbeta;

    float va = valpha;
    float vb = -half_valpha + half_sqrt3_vbeta;
    float vc = -half_valpha - half_sqrt3_vbeta;

    float v_max = va;
    float v_min = va;
    if(vb > v_max) v_max = vb;
    if(vc > v_max) v_max = vc;
    if(vb < v_min) v_min = vb;
    if(vc < v_min) v_min = vc;

    float common_offset = 0.5f - 0.5f * (v_max + v_min);

    float da = va + common_offset;
    float db = vb + common_offset;
    float dc = vc + common_offset;

    if(da < 0.0f) da = 0.0f;
    if(da > 1.0f) da = 1.0f;
    if(db < 0.0f) db = 0.0f;
    if(db > 1.0f) db = 1.0f;
    if(dc < 0.0f) dc = 0.0f;
    if(dc > 1.0f) dc = 1.0f;

    *duty_a = (uint32_t)(da * (float)ctx->duty_max);
    *duty_b = (uint32_t)(db * (float)ctx->duty_max);
    *duty_c = (uint32_t)(dc * (float)ctx->duty_max);

    return FOC_SVPWM_OK;
}

//============================== 上电自检/标定（方向 + 电角零偏） ==============================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取编码器机械角（本地封装）
// 参数说明     enc             编码器设备
// 参数说明     mech_rad        机械角输出指针（rad）
// 返回参数     uint8_t         1 成功 / 0 失败
// 使用示例     foc_read_mech_rad_local(&enc, &mech);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8_t foc_read_mech_rad_local(as5047p_device_t *enc, float *mech_rad)
{
    uint16_t angle_raw = 0;
    if((!enc) || (!mech_rad))
    {
        return 0;
    }

    if(AS5047P_OK != as5047p_read_angle_raw(enc, &angle_raw))
    {
        return 0;
    }

    *mech_rad = foc_as5047p_raw_to_mech_rad(angle_raw);
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按固定时长施加开环电压（本地封装）
// 参数说明     svpwm           SVPWM 上下文
// 参数说明     motor           电机句柄
// 参数说明     theta_e         电角（rad）
// 参数说明     vd              d 轴电压
// 参数说明     vq              q 轴电压
// 参数说明     ms              持续时间（ms）
// 返回参数     void
// 使用示例     foc_apply_vdq_ms_local(&svpwm, &motor, 0.0f, vd, 0.0f, 200);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void foc_apply_vdq_ms_local(const foc_svpwm_t *svpwm,
                                  bldc_drv8300_3pwm_motor_t *motor,
                                  float theta_e,
                                  float vd,
                                  float vq,
                                  uint32_t ms)
{
    uint32_t i = 0;

    if((!svpwm) || (!motor))
    {
        return;
    }

    for(i = 0; i < ms; i++)
    {
        uint32_t da = 0, db = 0, dc = 0;
        if(FOC_SVPWM_OK == foc_vdq_to_duty(svpwm, theta_e, vd, vq, &da, &db, &dc))
        {
            bldc_drv8300_3pwm_motor_set_duty(motor, da, db, dc);
        }
        else
        {
            bldc_drv8300_3pwm_motor_stop(motor);
        }
        system_delay_ms(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     方向检测（本地封装）
// 参数说明     svpwm           SVPWM 上下文
// 参数说明     enc             编码器设备
// 参数说明     motor           电机句柄
// 参数说明     cfg             标定配置
// 返回参数     int8_t          +1 / -1（旋转方向）
// 使用示例     foc_detect_direction_local(&svpwm, &enc, &motor, &cfg);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static int8_t foc_detect_direction_local(const foc_svpwm_t *svpwm,
                                        as5047p_device_t *enc,
                                        bldc_drv8300_3pwm_motor_t *motor,
                                        const foc_calib_cfg_t *cfg)
{
    float mech0 = 0.0f;
    float mech1 = 0.0f;
    float theta = 0.0f;
    uint32_t i = 0;
    uint8_t mech0_ready = 0u;
    uint8_t mech1_ready = 0u;
    uint32_t settle_ms = 0u;
    uint32_t measure_ms = 0u;
    uint32_t measure_end_ms = 0u;
    uint32_t total_ms = 0u;

    if((!svpwm) || (!enc) || (!motor) || (!cfg))
    {
        return +1;
    }

    {
        const float vd_align = foc_clampf(cfg->align_vd_frac, 0.0f, 0.40f) * svpwm->vbus;
        foc_apply_vdq_ms_local(svpwm, motor, 0.0f, vd_align, 0.0f, cfg->align_ms);
    }

    settle_ms = cfg->dirtest_settle_ms;
    measure_ms = (cfg->dirtest_ms == 0u) ? 1u : cfg->dirtest_ms;
    measure_end_ms = settle_ms + measure_ms;
    total_ms = measure_end_ms + settle_ms;
    if(total_ms == 0u)
    {
        total_ms = 1u;
    }

    {
        const float vq_spin = foc_clampf(cfg->dirtest_vq_frac, 0.0f, 0.40f) * svpwm->vbus;

        if(settle_ms == 0u)
        {
            if(!foc_read_mech_rad_local(enc, &mech0))
            {
                bldc_drv8300_3pwm_motor_stop(motor);
                return +1;
            }
            mech0_ready = 1u;
        }

        for(i = 0; i < total_ms; i++)
        {
            theta = foc_wrap_0_2pi(theta + cfg->dirtest_step_rad);
            {
                uint32_t da = 0, db = 0, dc = 0;
                if(FOC_SVPWM_OK == foc_vdq_to_duty(svpwm, theta, 0.0f, vq_spin, &da, &db, &dc))
                {
                    bldc_drv8300_3pwm_motor_set_duty(motor, da, db, dc);
                }
                else
                {
                    bldc_drv8300_3pwm_motor_stop(motor);
                }
            }
            system_delay_ms(1);

            if((!mech0_ready) && ((i + 1u) >= settle_ms))
            {
                if(foc_read_mech_rad_local(enc, &mech0))
                {
                    mech0_ready = 1u;
                }
            }

            if((!mech1_ready) && ((i + 1u) >= measure_end_ms))
            {
                if(foc_read_mech_rad_local(enc, &mech1))
                {
                    mech1_ready = 1u;
                }
            }
        }
    }

    if((!mech0_ready) && (!foc_read_mech_rad_local(enc, &mech0)))
    {
        bldc_drv8300_3pwm_motor_stop(motor);
        return +1;
    }

    if((!mech1_ready) && (!foc_read_mech_rad_local(enc, &mech1)))
    {
        bldc_drv8300_3pwm_motor_stop(motor);
        return +1;
    }

    bldc_drv8300_3pwm_motor_stop(motor);

    {
        float d = foc_wrap_pm_pi(mech1 - mech0);
        if(fabsf(d) < cfg->dirtest_min_move_rad)
        {
            return +1;
        }
        return (d > 0.0f) ? +1 : -1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电角零偏标定（本地封装）
// 参数说明     svpwm           SVPWM 上下文
// 参数说明     enc             编码器设备
// 参数说明     motor           电机句柄
// 参数说明     pole_pairs      极对数
// 参数说明     direction       方向（+1/-1）
// 参数说明     cfg             标定配置
// 返回参数     float           电角零偏（rad，wrap 到 [0, 2π)）
// 使用示例     foc_calibrate_offset_local(&svpwm, &enc, &motor, pp, dir, &cfg);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static float foc_calibrate_offset_local(const foc_svpwm_t *svpwm,
                                       as5047p_device_t *enc,
                                       bldc_drv8300_3pwm_motor_t *motor,
                                       uint16_t pole_pairs,
                                       int8_t direction,
                                       const foc_calib_cfg_t *cfg)
{
    float mech = 0.0f;
    const float theta_align = 0.0f;

    if((!svpwm) || (!enc) || (!motor) || (!cfg) || (pole_pairs == 0u) || (direction == 0))
    {
        return 0.0f;
    }

    {
        const float vd_align = foc_clampf(cfg->align_vd_frac, 0.0f, 0.40f) * svpwm->vbus;
        foc_apply_vdq_ms_local(svpwm, motor, theta_align, vd_align, 0.0f, cfg->align_ms);
    }

    if(!foc_read_mech_rad_local(enc, &mech))
    {
        bldc_drv8300_3pwm_motor_stop(motor);
        return 0.0f;
    }

    bldc_drv8300_3pwm_motor_stop(motor);

    {
        float offset = theta_align - (float)direction * (float)pole_pairs * mech;
        return foc_wrap_0_2pi(offset);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     标定参数默认配置
// 参数说明     cfg             标定配置结构体
// 返回参数     void
// 使用示例     foc_calibrate_motor_default_cfg(&cfg);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void foc_calibrate_motor_default_cfg(foc_calib_cfg_t *cfg)
{
    if(!cfg)
    {
        return;
    }

    cfg->align_vd_frac = 0.05f;
    cfg->align_ms = 350u;

    cfg->dirtest_vq_frac = 0.08f;
    cfg->dirtest_settle_ms = 500u;
    cfg->dirtest_ms = 500u;
    cfg->dirtest_step_rad = 0.020f;
    cfg->dirtest_min_move_rad = 0.12f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机方向与零偏标定
// 参数说明     svpwm           SVPWM 上下文
// 参数说明     loop            速度环（保存方向/零偏）
// 参数说明     as5047p_dev      AS5047P 设备指针（void*）
// 参数说明     drv8300_motor    DRV8300 电机句柄指针（void*）
// 参数说明     cfg             标定配置
// 返回参数     uint8_t         1 成功 / 0 失败
// 使用示例     foc_calibrate_motor(&svpwm, &loop, &enc, &motor, &cfg);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
// 双电机同时施加 Vdq，每 1ms 更新一次占空比
static void foc_apply_vdq_both_ms_local(const foc_svpwm_t *svpwm,
                                        bldc_drv8300_3pwm_motor_t *ml,
                                        bldc_drv8300_3pwm_motor_t *mr,
                                        float theta_e_l,
                                        float theta_e_r,
                                        float vd,
                                        float vq,
                                        uint32_t ms)
{
    uint32_t i;
    if((!svpwm) || (!ml) || (!mr)) return;

    for(i = 0; i < ms; i++)
    {
        uint32_t da = 0, db = 0, dc = 0;
        if(FOC_SVPWM_OK == foc_vdq_to_duty(svpwm, theta_e_l, vd, vq, &da, &db, &dc))
            bldc_drv8300_3pwm_motor_set_duty(ml, da, db, dc);
        else
            bldc_drv8300_3pwm_motor_stop(ml);

        if(FOC_SVPWM_OK == foc_vdq_to_duty(svpwm, theta_e_r, vd, vq, &da, &db, &dc))
            bldc_drv8300_3pwm_motor_set_duty(mr, da, db, dc);
        else
            bldc_drv8300_3pwm_motor_stop(mr);

        system_delay_ms(1);
    }
}

uint8_t foc_calibrate_both_motors(const foc_svpwm_t *svpwm,
                                  foc_speed_loop_t *loop_l,
                                  foc_speed_loop_t *loop_r,
                                  void *as5047p_dev_l,
                                  void *as5047p_dev_r,
                                  void *drv8300_motor_l,
                                  void *drv8300_motor_r,
                                  const foc_calib_cfg_t *cfg)
{
    as5047p_device_t *el = (as5047p_device_t *)as5047p_dev_l;
    as5047p_device_t *er = (as5047p_device_t *)as5047p_dev_r;
    bldc_drv8300_3pwm_motor_t *ml = (bldc_drv8300_3pwm_motor_t *)drv8300_motor_l;
    bldc_drv8300_3pwm_motor_t *mr = (bldc_drv8300_3pwm_motor_t *)drv8300_motor_r;
    float mech_l0 = 0.0f, mech_l1 = 0.0f;
    float mech_r0 = 0.0f, mech_r1 = 0.0f;
    uint8_t l0_ready = 0u, l1_ready = 0u;
    uint8_t r0_ready = 0u, r1_ready = 0u;
    uint8_t result = 0u;
    int8_t dir_l, dir_r;
    float offset_l, offset_r;
    float theta_l = 0.0f, theta_r = 0.0f;
    uint32_t total_ms;
    uint32_t settle_ms, measure_ms, measure_end_ms;

    if((!svpwm) || (!loop_l) || (!loop_r) || (!el) || (!er) || (!ml) || (!mr) || (!cfg))
        return 0u;

    // ----- Phase 1: 对齐 (Vd 锁轴) -----
    {
        const float vd_align = foc_clampf(cfg->align_vd_frac, 0.0f, 0.40f) * svpwm->vbus;
        foc_apply_vdq_both_ms_local(svpwm, ml, mr, 0.0f, 0.0f, vd_align, 0.0f, cfg->align_ms);
    }

    // ----- Phase 2: 方向检测 (旋转 Vq 步进) -----
    settle_ms = cfg->dirtest_settle_ms;
    measure_ms = (cfg->dirtest_ms == 0u) ? 1u : cfg->dirtest_ms;
    measure_end_ms = settle_ms + measure_ms;
    total_ms = measure_end_ms + settle_ms;
    if(total_ms == 0u) total_ms = 1u;

    {
        const float vq_spin = foc_clampf(cfg->dirtest_vq_frac, 0.0f, 0.40f) * svpwm->vbus;
        uint32_t i;

        if(settle_ms == 0u)
        {
            if(foc_read_mech_rad_local(el, &mech_l0)) l0_ready = 1u;
            if(foc_read_mech_rad_local(er, &mech_r0)) r0_ready = 1u;
        }

        for(i = 0; i < total_ms; i++)
        {
            theta_l = foc_wrap_0_2pi(theta_l + cfg->dirtest_step_rad);
            theta_r = foc_wrap_0_2pi(theta_r + cfg->dirtest_step_rad);

            {
                uint32_t da = 0, db = 0, dc = 0;
                if(FOC_SVPWM_OK == foc_vdq_to_duty(svpwm, theta_l, 0.0f, vq_spin, &da, &db, &dc))
                    bldc_drv8300_3pwm_motor_set_duty(ml, da, db, dc);
                else
                    bldc_drv8300_3pwm_motor_stop(ml);

                if(FOC_SVPWM_OK == foc_vdq_to_duty(svpwm, theta_r, 0.0f, vq_spin, &da, &db, &dc))
                    bldc_drv8300_3pwm_motor_set_duty(mr, da, db, dc);
                else
                    bldc_drv8300_3pwm_motor_stop(mr);
            }
            system_delay_ms(1);

            if((!l0_ready) && ((i + 1u) >= settle_ms))
                if(foc_read_mech_rad_local(el, &mech_l0)) l0_ready = 1u;
            if((!r0_ready) && ((i + 1u) >= settle_ms))
                if(foc_read_mech_rad_local(er, &mech_r0)) r0_ready = 1u;

            if((!l1_ready) && ((i + 1u) >= measure_end_ms))
                if(foc_read_mech_rad_local(el, &mech_l1)) l1_ready = 1u;
            if((!r1_ready) && ((i + 1u) >= measure_end_ms))
                if(foc_read_mech_rad_local(er, &mech_r1)) r1_ready = 1u;
        }
    }

    bldc_drv8300_3pwm_motor_stop(ml);
    bldc_drv8300_3pwm_motor_stop(mr);

    // 计算方向
    if((!l0_ready) && (!foc_read_mech_rad_local(el, &mech_l0)))
        goto left_fail;
    if((!l1_ready) && (!foc_read_mech_rad_local(el, &mech_l1)))
        goto left_fail;
    {
        float dl = foc_wrap_pm_pi(mech_l1 - mech_l0);
        dir_l = (fabsf(dl) < cfg->dirtest_min_move_rad) ? +1 : ((dl > 0.0f) ? +1 : -1);
    }

    {
        float dr;
        if((!r0_ready) && (!foc_read_mech_rad_local(er, &mech_r0)))
            goto right_fail;
        if((!r1_ready) && (!foc_read_mech_rad_local(er, &mech_r1)))
            goto right_fail;
        dr = foc_wrap_pm_pi(mech_r1 - mech_r0);
        dir_r = (fabsf(dr) < cfg->dirtest_min_move_rad) ? +1 : ((dr > 0.0f) ? +1 : -1);
    }

    // ----- Phase 3: 零偏标定 -----
    {
        const float vd_align = foc_clampf(cfg->align_vd_frac, 0.0f, 0.40f) * svpwm->vbus;
        foc_apply_vdq_both_ms_local(svpwm, ml, mr, 0.0f, 0.0f, vd_align, 0.0f, cfg->align_ms);

        if(foc_read_mech_rad_local(el, &mech_l0))
        {
            offset_l = foc_wrap_0_2pi(0.0f - (float)dir_l * (float)loop_l->angle_cfg.pole_pairs * mech_l0);
            loop_l->angle_cfg.direction = dir_l;
            loop_l->angle_cfg.elec_offset = offset_l;
            loop_l->angle_cfg.elec_offset_fxp = (int32_t)(offset_l * ((float)FOC_ANGLE_FXP_RESOLUTION / (2.0f * (float)FOC_PI)));
            loop_l->has_last = 0; loop_l->omega_m = 0.0f; loop_l->vq_out = 0.0f;
            loop_l->speed_pid.error = 0.0f; loop_l->speed_pid.last_error = 0.0f;
            result |= 1u;
        }
        else goto left_fail;

        if(foc_read_mech_rad_local(er, &mech_r0))
        {
            offset_r = foc_wrap_0_2pi(0.0f - (float)dir_r * (float)loop_r->angle_cfg.pole_pairs * mech_r0);
            loop_r->angle_cfg.direction = dir_r;
            loop_r->angle_cfg.elec_offset = offset_r;
            loop_r->angle_cfg.elec_offset_fxp = (int32_t)(offset_r * ((float)FOC_ANGLE_FXP_RESOLUTION / (2.0f * (float)FOC_PI)));
            loop_r->has_last = 0; loop_r->omega_m = 0.0f; loop_r->vq_out = 0.0f;
            loop_r->speed_pid.error = 0.0f; loop_r->speed_pid.last_error = 0.0f;
            result |= 2u;
        }
        else goto right_fail;
    }

    bldc_drv8300_3pwm_motor_stop(ml);
    bldc_drv8300_3pwm_motor_stop(mr);
    return result;

left_fail:
    result &= ~1u;
    goto done;
right_fail:
    result &= ~2u;
done:
    bldc_drv8300_3pwm_motor_stop(ml);
    bldc_drv8300_3pwm_motor_stop(mr);
    return result;
}

uint8_t foc_calibrate_motor(const foc_svpwm_t *svpwm,
                            foc_speed_loop_t *loop,
                            void *as5047p_dev,
                            void *drv8300_motor,
                            const foc_calib_cfg_t *cfg)
{
    as5047p_device_t *enc = (as5047p_device_t *)as5047p_dev;
    bldc_drv8300_3pwm_motor_t *motor = (bldc_drv8300_3pwm_motor_t *)drv8300_motor;

    int8_t dir = +1;
    float offset = 0.0f;

    if((!svpwm) || (!loop) || (!enc) || (!motor) || (!cfg))
    {
        return 0;
    }

    dir = foc_detect_direction_local(svpwm, enc, motor, cfg);
    offset = foc_calibrate_offset_local(svpwm, enc, motor, loop->angle_cfg.pole_pairs, dir, cfg);

    loop->angle_cfg.direction = dir;
    loop->angle_cfg.elec_offset = offset;
    loop->angle_cfg.elec_offset_fxp = (int32_t)(offset * ((float)FOC_ANGLE_FXP_RESOLUTION / (2.0f * (float)FOC_PI)));

    loop->has_last = 0;
    loop->omega_m = 0.0f;
    loop->vq_out = 0.0f;
    loop->speed_pid.error = 0.0f;
    loop->speed_pid.last_error = 0.0f;

    return 1;
}

// ============================== SVPWM 查表 (vd=0) ==============================

#pragma section data "cpu1_dsram"
float g_foc_svpwm_lut_da[FOC_SVPWM_LUT_SIZE];
float g_foc_svpwm_lut_db[FOC_SVPWM_LUT_SIZE];
float g_foc_svpwm_lut_dc[FOC_SVPWM_LUT_SIZE];
#pragma section data restore

void foc_svpwm_lut_init(const foc_svpwm_t *ctx)
{
    if (!ctx || ctx->duty_max == 0u) return;

    for (uint32_t i = 0u; i < FOC_SVPWM_LUT_SIZE; i++)
    {
        foc_fxp_angle_t theta_fxp = (foc_fxp_angle_t)(i << 4);
        Ifx_Lut_FxpAngle fxp = (Ifx_Lut_FxpAngle)((uint32_t)theta_fxp & (uint32_t)FOC_ANGLE_FXP_MASK);
        float sin_t = Ifx_LutSincosF32_sin(fxp);
        float cos_t = Ifx_LutSincosF32_sin((IFX_LUT_ANGLE_PI / 2) - fxp);

        float valpha = -sin_t;
        float vbeta  =  cos_t;

        float va = valpha;
        float vb = -0.5f * valpha + FOC_SQRT3_DIV2 * vbeta;
        float vc = -0.5f * valpha - FOC_SQRT3_DIV2 * vbeta;

        float v_max = va, v_min = va;
        if (vb > v_max) v_max = vb; if (vc > v_max) v_max = vc;
        if (vb < v_min) v_min = vb; if (vc < v_min) v_min = vc;

        float co = 0.5f - 0.5f * (v_max + v_min);
        float da = va + co; if (da < 0.0f) da = 0.0f; if (da > 1.0f) da = 1.0f;
        float db = vb + co; if (db < 0.0f) db = 0.0f; if (db > 1.0f) db = 1.0f;
        float dc = vc + co; if (dc < 0.0f) dc = 0.0f; if (dc > 1.0f) dc = 1.0f;

        g_foc_svpwm_lut_da[i] = da;
        g_foc_svpwm_lut_db[i] = db;
        g_foc_svpwm_lut_dc[i] = dc;
    }
}

foc_svpwm_status_t foc_vq_to_duty_lut(const foc_svpwm_t *ctx,
                                       foc_fxp_angle_t theta_e_fxp,
                                       float vq_frac,
                                       uint32_t *duty_a,
                                       uint32_t *duty_b,
                                       uint32_t *duty_c)
{
    if ((!ctx) || (!duty_a) || (!duty_b) || (!duty_c) || (ctx->duty_max == 0u))
        return FOC_SVPWM_ERR_PARAM;

    float vq_abs = vq_frac;
    uint32_t theta = (uint32_t)theta_e_fxp & (uint32_t)FOC_ANGLE_FXP_MASK;

    if (vq_abs < 0.0f)
    {
        vq_abs = -vq_abs;
        theta = (theta + 2048u) & (uint32_t)FOC_ANGLE_FXP_MASK;
    }

    uint32_t idx = (theta >> 4) & 0xFFu;
    uint32_t idx2 = (idx + 1u) & 0xFFu;
    float frac = (float)(theta & 0x0Fu) * 0.0625f;

    float da_n = g_foc_svpwm_lut_da[idx] + frac * (g_foc_svpwm_lut_da[idx2] - g_foc_svpwm_lut_da[idx]);
    float db_n = g_foc_svpwm_lut_db[idx] + frac * (g_foc_svpwm_lut_db[idx2] - g_foc_svpwm_lut_db[idx]);
    float dc_n = g_foc_svpwm_lut_dc[idx] + frac * (g_foc_svpwm_lut_dc[idx2] - g_foc_svpwm_lut_dc[idx]);

    float scale = vq_abs * (float)ctx->duty_max;
    *duty_a = (uint32_t)(da_n * scale);
    *duty_b = (uint32_t)(db_n * scale);
    *duty_c = (uint32_t)(dc_n * scale);

    return FOC_SVPWM_OK;
}

#pragma section code restore
