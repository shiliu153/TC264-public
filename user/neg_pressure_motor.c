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
#define NEG_PRESSURE_MOTOR_RAMP_US_PER_S      (500.0f)
#endif

extern volatile uint8_t g_neg_pressure_enable;
extern volatile float g_neg_pressure_target_percent;
extern volatile float g_neg_pressure_output_pulse_us;

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
    float pulse_step_us;

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

    pulse_step_us = NEG_PRESSURE_MOTOR_RAMP_US_PER_S * dt;
    if ((motor->target_pulse_us - motor->current_pulse_us) > pulse_step_us)
    {
        motor->current_pulse_us += pulse_step_us;
    }
    else if ((motor->current_pulse_us - motor->target_pulse_us) > pulse_step_us)
    {
        motor->current_pulse_us -= pulse_step_us;
    }
    else
    {
        motor->current_pulse_us = motor->target_pulse_us;
    }

    neg_pressure_motor_apply_pulse(motor, motor->current_pulse_us);
    motor->active = (uint8_t)(motor->current_pulse_us > (NEG_PRESSURE_MOTOR_PULSE_MIN_US + 0.5f));
}
