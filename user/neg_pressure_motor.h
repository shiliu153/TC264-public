#ifndef _neg_pressure_motor_h_
#define _neg_pressure_motor_h_
/*********************************************************************************************************************
* 文件：neg_pressure_motor.h
* 模块：应用层（第 6 层）—— 负压电机
* 职责：负压电机对外接口声明
* 依赖：zf_common_typedef.h
* 风险等级：R2
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "zf_driver/zf_driver_pwm.h"

#ifndef NEG_PRESSURE_MOTOR_AUTO_MAX_PERCENT
#define NEG_PRESSURE_MOTOR_AUTO_MAX_PERCENT       (80.0f)
#endif

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
void neg_pressure_motor_beep(neg_pressure_motor_t *motor, uint32_t duration_ms, float percent);

#ifdef __cplusplus
}
#endif

#endif
