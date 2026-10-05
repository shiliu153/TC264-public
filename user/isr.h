#ifndef _isr_h
#define _isr_h
/*********************************************************************************************************************
* 文件：isr.h
* 模块：应用层（第 6 层）—— 中断服务
* 职责：中断服务对外接口与运行时命令声明
* 依赖：zf_common_typedef.h
* 风险等级：R3
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/

#define HARDWARE_STRIPPED

#include "zf_common_headfile.h"

#include "bldc_drv8300_3pwm.h"
#include "foc_svpwm.h"
#include "imu_ahrs.h"
#include "neg_pressure_motor.h"
#include "pid.h"

#ifndef FOC_CTRL_PIT_INDEX
#define FOC_CTRL_PIT_INDEX            (CCU61_CH0)
#endif

#ifndef FOC_CTRL_PERIOD_US
#define FOC_CTRL_PERIOD_US            (100u)
#endif

#ifndef STARTUP_DELTA_RAMP_MS
#define STARTUP_DELTA_RAMP_MS         (100u)
#endif

#ifndef FOC_SPEED_ESTIMATE_HZ
#define FOC_SPEED_ESTIMATE_HZ         (1000u)
#endif

typedef enum
{
	SYSTEM_STOP_REASON_NONE = 0,
	SYSTEM_STOP_REASON_LOST_LINE = 1,
	SYSTEM_STOP_REASON_ROUTE_DONE = 2,
	SYSTEM_STOP_REASON_LOW_BATTERY = 3,
	SYSTEM_STOP_REASON_ERROR = 4,
	SYSTEM_STOP_REASON_ROLLOVER = 5,
} system_stop_reason_t;

void foc_ctrl_periodic_isr(void);

extern volatile float           g_motor_mech_l_rad;
extern volatile float           g_motor_mech_r_rad;
extern volatile uint8_t         g_motor_mech_l_valid;
extern volatile uint8_t         g_motor_mech_r_valid;
extern volatile float           g_motor_left_actual_rpm;
extern volatile float           g_motor_right_actual_rpm;
extern volatile float           g_motor_elec_l_deg;
extern volatile float           g_motor_elec_r_deg;
extern volatile float           g_vbus;


extern foc_svpwm_t              g_svpwm;
extern bldc_drv8300_3pwm_dual_t g_pwm;
extern neg_pressure_motor_t     g_neg_pressure_motor;
extern as5047p_device_t         g_enc_l;
extern as5047p_device_t         g_enc_r;
extern foc_speed_loop_t         g_loop_l;
extern foc_speed_loop_t         g_loop_r;

extern PID                     g_steer_pid;

extern volatile uint8_t         g_run_enable;
extern volatile system_stop_reason_t g_stop_reason;
extern volatile uint8_t         g_neg_pressure_enable;
extern volatile float           g_neg_pressure_target_percent;
extern volatile float           g_neg_pressure_output_pulse_us;

extern volatile uint8_t         g_paper_start_pending;
extern volatile uint8_t         g_paper_start_neg_enable;

extern volatile float           g_base_speed_rpm;
extern volatile float           g_straight_boost_rpm;
extern volatile float           g_turn_delta_rpm;
extern volatile float           g_target_left_rpm;
extern volatile float           g_target_right_rpm;

extern volatile float           g_att_roll_deg;
extern volatile float           g_att_pitch_deg;
extern volatile float           g_att_yaw_deg;

extern volatile float           g_gyro_x_dps_f;
extern volatile float           g_gyro_y_dps_f;
extern volatile float           g_gyro_z_dps_f;
extern volatile float           g_gyro_lp_alpha;
extern volatile float           g_gyro_bias_x;
extern volatile float           g_gyro_bias_y;
extern volatile float           g_gyro_bias_z;
extern volatile float           g_as5047p_lp_alpha;
extern volatile float           g_as5047p_speed_avg_samples;
extern volatile float           g_as5047p_speed_zero_deadband_rpm;
extern volatile uint16_t        g_lf_actual_hz;
extern volatile uint32_t        g_cd_ticks;

// 速度斜坡已统一由 PI 层的 FOC_MAX_ACCEL_RPM_PS / FOC_MAX_DECEL_RPM_PS 控制，
// 1kHz ISR 和 FOC ISR 不再做命令级爬坡，直接发布目标值。

extern imu_ahrs_t               g_ahrs;

#endif

