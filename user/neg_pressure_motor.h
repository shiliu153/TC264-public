#ifndef _neg_pressure_motor_h_
#define _neg_pressure_motor_h_

#include "zf_common_headfile.h"
#include "zf_driver/zf_driver_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    NEG_PRESSURE_MOTOR_STATE_DISABLED = 0,
    NEG_PRESSURE_MOTOR_STATE_ARMING   = 1,
    NEG_PRESSURE_MOTOR_STATE_RUNNING  = 2,
} neg_pressure_motor_state_t;

typedef struct
{
    pwm_channel_enum           pwm_ch;
    neg_pressure_motor_state_t state;
    float                      current_pulse_us;
    float                      target_pulse_us;
    float                      arm_elapsed_s;
    uint8_t active;
} neg_pressure_motor_t;

void neg_pressure_motor_init(neg_pressure_motor_t *motor);
void neg_pressure_motor_update(neg_pressure_motor_t *motor, float dt, uint8_t run_enable);
void neg_pressure_motor_stop(neg_pressure_motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif
