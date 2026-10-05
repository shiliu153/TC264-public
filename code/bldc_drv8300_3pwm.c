/*********************************************************************************************************************
* DRV8300 3PWM 无刷电机底层驱动（最小可用）
*********************************************************************************************************************/

/*********************************************************************************************************************
* 文件：bldc_drv8300_3pwm.c
* 模块：控制算法层（第 5 层）—— 栅极驱动
* 职责：DRV8300 三相栅极驱动 PWM 初始化（ATOM0，双电机共 6 路）
* 依赖：zf_common_headfile.h、本目录 bldc_drv8300_3pwm.h
* 风险等级：R3
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "bldc_drv8300_3pwm.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     BLDC 占空比限幅
// 参数说明     duty            占空比
// 返回参数     uint32_t        限幅后的占空比
// 使用示例     bldc_drv8300_clamp_duty(duty);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------

static uint32_t bldc_drv8300_clamp_duty(uint32_t duty)
{
    if(duty > PWM_DUTY_MAX)
    {
        return PWM_DUTY_MAX;
    }
    return duty;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DRV8300 3PWM 电机初始化
// 参数说明     motor           电机句柄
// 参数说明     u_pwm           U 相 PWM 通道
// 参数说明     v_pwm           V 相 PWM 通道
// 参数说明     w_pwm           W 相 PWM 通道
// 参数说明     pwm_freq_hz     PWM 频率（Hz）
// 返回参数     void
// 使用示例     bldc_drv8300_3pwm_motor_init(&motor, u, v, w, 20000);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void bldc_drv8300_3pwm_motor_init(bldc_drv8300_3pwm_motor_t *motor,
                                 pwm_channel_enum u_pwm,
                                 pwm_channel_enum v_pwm,
                                 pwm_channel_enum w_pwm,
                                 uint32_t pwm_freq_hz)
{
    if(!motor)
    {
        return;
    }

    motor->u_pwm = u_pwm;
    motor->v_pwm = v_pwm;
    motor->w_pwm = w_pwm;
    motor->pwm_freq_hz = (pwm_freq_hz == 0u) ? BLDC_DRV8300_PWM_FREQ_HZ : pwm_freq_hz;

    pwm_init(motor->u_pwm, motor->pwm_freq_hz, 0);
    pwm_init(motor->v_pwm, motor->pwm_freq_hz, 0);
    pwm_init(motor->w_pwm, motor->pwm_freq_hz, 0);

    motor->inited = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DRV8300 3PWM 设置三相占空比
// 参数说明     motor           电机句柄
// 参数说明     duty_u          U 相占空比
// 参数说明     duty_v          V 相占空比
// 参数说明     duty_w          W 相占空比
// 返回参数     void
// 使用示例     bldc_drv8300_3pwm_motor_set_duty(&motor, du, dv, dw);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void bldc_drv8300_3pwm_motor_set_duty(bldc_drv8300_3pwm_motor_t *motor,
                                     uint32_t duty_u,
                                     uint32_t duty_v,
                                     uint32_t duty_w)
{
    if((!motor) || (!motor->inited))
    {
        return;
    }

    pwm_set_duty(motor->u_pwm, bldc_drv8300_clamp_duty(duty_u));
    pwm_set_duty(motor->v_pwm, bldc_drv8300_clamp_duty(duty_v));
    pwm_set_duty(motor->w_pwm, bldc_drv8300_clamp_duty(duty_w));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DRV8300 3PWM 电机停止（占空比清零）
// 参数说明     motor           电机句柄
// 返回参数     void
// 使用示例     bldc_drv8300_3pwm_motor_stop(&motor);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void bldc_drv8300_3pwm_motor_stop(bldc_drv8300_3pwm_motor_t *motor)
{
    if((!motor) || (!motor->inited))
    {
        return;
    }

    pwm_set_duty(motor->u_pwm, 0);
    pwm_set_duty(motor->v_pwm, 0);
    pwm_set_duty(motor->w_pwm, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DRV8300 3PWM 双电机默认初始化
// 参数说明     dual            双电机句柄
// 返回参数     void
// 使用示例     bldc_drv8300_3pwm_dual_init_default(&dual);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void bldc_drv8300_3pwm_dual_init_default(bldc_drv8300_3pwm_dual_t *dual)
{
    if(!dual)
    {
        return;
    }

    bldc_drv8300_3pwm_motor_init(&dual->left,
                                BLDC_DRV8300_L_U_PWM,
                                BLDC_DRV8300_L_V_PWM,
                                BLDC_DRV8300_L_W_PWM,
                                BLDC_DRV8300_PWM_FREQ_HZ);

    bldc_drv8300_3pwm_motor_init(&dual->right,
                                BLDC_DRV8300_R_U_PWM,
                                BLDC_DRV8300_R_V_PWM,
                                BLDC_DRV8300_R_W_PWM,
                                BLDC_DRV8300_PWM_FREQ_HZ);
}
