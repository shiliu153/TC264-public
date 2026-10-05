/*********************************************************************************************************************
* 文件：neg_pressure_motor.c
* 模块：应用层（第 6 层）—— 负压电机
* 职责：外置电调 PWM 控制（1000-2000us 脉宽）与解锁时序
* 依赖：zf_driver_pwm.h、本目录 neg_pressure_motor.h
* 风险等级：R2
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "neg_pressure_motor.h"

#ifndef NEG_PRESSURE_MOTOR_PWM_CH
#define NEG_PRESSURE_MOTOR_PWM_CH             (ATOM1_CH5_P02_5)
#endif

#ifndef NEG_PRESSURE_MOTOR_PWM_FREQ_HZ
#define NEG_PRESSURE_MOTOR_PWM_FREQ_HZ        (300u)
#endif

#ifndef NEG_PRESSURE_MOTOR_ENABLE_DEFAULT
#define NEG_PRESSURE_MOTOR_ENABLE_DEFAULT     (0u)
#endif

#ifndef NEG_PRESSURE_MOTOR_TARGET_PERCENT_DEFAULT
#define NEG_PRESSURE_MOTOR_TARGET_PERCENT_DEFAULT (0.0f)
#endif

#ifndef NEG_PRESSURE_MOTOR_PULSE_MIN_US
#define NEG_PRESSURE_MOTOR_PULSE_MIN_US       (1000.0f)
#endif

#ifndef NEG_PRESSURE_MOTOR_PULSE_MAX_US
#define NEG_PRESSURE_MOTOR_PULSE_MAX_US       (2000.0f)
#endif

#ifndef NEG_PRESSURE_MOTOR_ARM_TIME_S
#define NEG_PRESSURE_MOTOR_ARM_TIME_S         (1.0f)
#endif

#ifndef NEG_PRESSURE_MOTOR_RAMP_US_PER_S
#define NEG_PRESSURE_MOTOR_RAMP_US_PER_S      (1000.0f)
#endif

extern volatile uint8_t g_neg_pressure_enable;
extern volatile float g_neg_pressure_target_percent;
extern volatile float g_neg_pressure_output_pulse_us;
extern volatile uint8_t g_run_enable;

static float neg_pressure_motor_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float neg_pressure_motor_percent_to_pulse_us(float percent)
{
    float clamped_percent = neg_pressure_motor_clampf(percent, 0.0f, 100.0f);
    return NEG_PRESSURE_MOTOR_PULSE_MIN_US
         + (clamped_percent * (NEG_PRESSURE_MOTOR_PULSE_MAX_US - NEG_PRESSURE_MOTOR_PULSE_MIN_US) / 100.0f);
}

static uint32_t neg_pressure_motor_pulse_to_duty(float pulse_us)
{
    float clamped_pulse_us = neg_pressure_motor_clampf(
        pulse_us,
        NEG_PRESSURE_MOTOR_PULSE_MIN_US,
        NEG_PRESSURE_MOTOR_PULSE_MAX_US
    );
    float period_us = 1000000.0f / (float)NEG_PRESSURE_MOTOR_PWM_FREQ_HZ;
    float duty_f = clamped_pulse_us * (float)PWM_DUTY_MAX / period_us;

    if (duty_f < 0.0f)
    {
        duty_f = 0.0f;
    }
    if (duty_f > (float)PWM_DUTY_MAX)
    {
        duty_f = (float)PWM_DUTY_MAX;
    }

    return (uint32_t)(duty_f + 0.5f);
}

static void neg_pressure_motor_apply_pulse(neg_pressure_motor_t *motor, float pulse_us)
{
    if (NULL == motor)
    {
        return;
    }

    motor->current_pulse_us = neg_pressure_motor_clampf(
        pulse_us,
        NEG_PRESSURE_MOTOR_PULSE_MIN_US,
        NEG_PRESSURE_MOTOR_PULSE_MAX_US
    );
    pwm_set_duty(motor->pwm_ch, neg_pressure_motor_pulse_to_duty(motor->current_pulse_us));
    g_neg_pressure_output_pulse_us = motor->current_pulse_us;
}

void neg_pressure_motor_init(neg_pressure_motor_t *motor)
{
    if (NULL == motor)
    {
        return;
    }

    motor->pwm_ch = NEG_PRESSURE_MOTOR_PWM_CH;
    motor->state = NEG_PRESSURE_MOTOR_STATE_DISABLED;
    motor->current_pulse_us = NEG_PRESSURE_MOTOR_PULSE_MIN_US;
    motor->target_pulse_us = NEG_PRESSURE_MOTOR_PULSE_MIN_US;
    motor->arm_elapsed_s = 0.0f;
    motor->active = 0u;

    pwm_init(
        motor->pwm_ch,
        NEG_PRESSURE_MOTOR_PWM_FREQ_HZ,
        neg_pressure_motor_pulse_to_duty(NEG_PRESSURE_MOTOR_PULSE_MIN_US)
    );

    g_neg_pressure_enable = NEG_PRESSURE_MOTOR_ENABLE_DEFAULT;
    g_neg_pressure_target_percent = NEG_PRESSURE_MOTOR_TARGET_PERCENT_DEFAULT;
    g_neg_pressure_output_pulse_us = NEG_PRESSURE_MOTOR_PULSE_MIN_US;

    neg_pressure_motor_stop(motor);
}

void neg_pressure_motor_stop(neg_pressure_motor_t *motor)
{
    if (NULL == motor)
    {
        return;
    }

    motor->state = NEG_PRESSURE_MOTOR_STATE_DISABLED;
    motor->target_pulse_us = NEG_PRESSURE_MOTOR_PULSE_MIN_US;
    motor->arm_elapsed_s = 0.0f;
    motor->active = 0u;
    neg_pressure_motor_apply_pulse(motor, NEG_PRESSURE_MOTOR_PULSE_MIN_US);
}

void neg_pressure_motor_update(neg_pressure_motor_t *motor, float dt, uint8_t run_enable)
{
    float target_percent;

    if ((NULL == motor) || (dt <= 0.0f))
    {
        return;
    }

    if (!run_enable)
    {
        neg_pressure_motor_stop(motor);
        return;
    }

    target_percent = neg_pressure_motor_clampf(g_neg_pressure_target_percent, 0.0f, 100.0f);
    g_neg_pressure_target_percent = target_percent;

    {
        float max_pct = (g_run_enable != 0u) ? NEG_PRESSURE_MOTOR_AUTO_MAX_PERCENT : 100.0f;
        if (target_percent > max_pct)
        {
            target_percent = max_pct;
        }
    }
    motor->target_pulse_us = neg_pressure_motor_percent_to_pulse_us(target_percent);

    if (NEG_PRESSURE_MOTOR_STATE_DISABLED == motor->state)
    {
        motor->state = NEG_PRESSURE_MOTOR_STATE_ARMING;
        motor->arm_elapsed_s = 0.0f;
        motor->active = 0u;
        neg_pressure_motor_apply_pulse(motor, NEG_PRESSURE_MOTOR_PULSE_MIN_US);
    }

    if (NEG_PRESSURE_MOTOR_STATE_ARMING == motor->state)
    {
        motor->arm_elapsed_s += dt;
        neg_pressure_motor_apply_pulse(motor, NEG_PRESSURE_MOTOR_PULSE_MIN_US);
        motor->active = 0u;

        if (motor->arm_elapsed_s >= NEG_PRESSURE_MOTOR_ARM_TIME_S)
        {
            motor->arm_elapsed_s = NEG_PRESSURE_MOTOR_ARM_TIME_S;
            motor->state = NEG_PRESSURE_MOTOR_STATE_RUNNING;
        }
        return;
    }

    motor->current_pulse_us = motor->target_pulse_us;

    neg_pressure_motor_apply_pulse(motor, motor->current_pulse_us);
    motor->active = (uint8_t)(motor->current_pulse_us > (NEG_PRESSURE_MOTOR_PULSE_MIN_US + 0.5f));
}

void neg_pressure_motor_beep(neg_pressure_motor_t *motor, uint32_t duration_ms, float percent)
{
    if (!motor || duration_ms == 0u) return;
    if (duration_ms > 500u) duration_ms = 500u;

    float pct = (percent < 0.0f) ? 0.0f : ((percent > 100.0f) ? 100.0f : percent);
    float pulse_us = NEG_PRESSURE_MOTOR_PULSE_MIN_US
                   + (pct * (NEG_PRESSURE_MOTOR_PULSE_MAX_US - NEG_PRESSURE_MOTOR_PULSE_MIN_US) / 100.0f);

    neg_pressure_motor_apply_pulse(motor, pulse_us);
    system_delay_ms(duration_ms);
    neg_pressure_motor_apply_pulse(motor, NEG_PRESSURE_MOTOR_PULSE_MIN_US);
}
