/*********************************************************************************************************************
* TC264 Opensourec Library (TC264 开源库)
*
* FOC/SVPWM 最小数学模块（电角度 -> 三相占空比）
* - 输入：电角度 theta_e(rad)、电压指令 Vd/Vq（单位：V 或归一化到母线的同一单位）
* - 输出：三相占空比 (0..PWM_DUTY_MAX)
*
* 说明：
* - 该模块不依赖电流采样，适合“电压模式 FOC”（开环/速度环）。
* - 若要闭环电流 FOC，需要在上层引入相电流采样（ADC）并做 Id/Iq PI。
*********************************************************************************************************************/

#ifndef _code_foc_svpwm_h_
#define _code_foc_svpwm_h_

#include "zf_common/zf_common_typedef.h"
#include "pid.h"

#ifndef FOC_PI
#define FOC_PI                 (3.14159265358979323846f)
#define FOC_TWO_PI             (6.28318530717958647692f)
#define FOC_RPM_TO_RADPS       (0.10471975511965977461f)
#define FOC_SQRT3_DIV2         (0.86602540378443864676f)
#define FOC_RAD_TO_FXP         (651.898646904403295309f)
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    FOC_SVPWM_OK = 0,
    FOC_SVPWM_ERR_PARAM = 1,
} foc_svpwm_status_t;

typedef struct
{
    float vbus;            
    float inv_vbus;
    uint32_t duty_max; 
} foc_svpwm_t;

#ifndef FOC_DEFAULT_POLE_PAIRS
#define FOC_DEFAULT_POLE_PAIRS      (7u)
#endif

// 正 RPM = 小车前进。
// 若某侧车轮“前进”对应电机内部正转方向的反向，则将该侧符号设为 -1。
#ifndef FOC_LEFT_USER_RPM_SIGN
#define FOC_LEFT_USER_RPM_SIGN      (-1.0f)
#endif

#ifndef FOC_RIGHT_USER_RPM_SIGN
#define FOC_RIGHT_USER_RPM_SIGN     (1.0f)
#endif


typedef struct
{
    uint16_t pole_pairs;
    int8_t   direction;
    float  elec_offset;
    int32_t elec_offset_fxp;
} foc_angle_cfg_t;

// 12-bit fixed-point angle in [0, 4096) representing [0, 2π)
typedef int32_t foc_fxp_angle_t;

#ifndef FOC_ANGLE_FXP_BITS
#define FOC_ANGLE_FXP_BITS (12)
#endif

#define FOC_ANGLE_FXP_RESOLUTION (1u << FOC_ANGLE_FXP_BITS)
#define FOC_ANGLE_FXP_MASK       (FOC_ANGLE_FXP_RESOLUTION - 1u)

// AS5047P is 14-bit (0..16383). Mapping to 12-bit is exact: 16384 / 4096 = 4.
static inline foc_fxp_angle_t foc_as5047p_raw_to_mech_fxp(uint16_t as5047p_angle_14bit)
{
    return (foc_fxp_angle_t)((as5047p_angle_14bit & 0x3FFFu) >> 2);
}

static inline foc_fxp_angle_t foc_mech_fxp_to_elec_fxp(foc_fxp_angle_t mech_fxp, const foc_angle_cfg_t *cfg)
{
    if((!cfg) || (cfg->pole_pairs == 0u) || (cfg->direction == 0))
    {
        return 0;
    }

    int32_t theta = (int32_t)mech_fxp * (int32_t)cfg->pole_pairs;
    if(cfg->direction < 0)
    {
        theta = -theta;
    }

    theta += cfg->elec_offset_fxp;

    return (foc_fxp_angle_t)((uint32_t)theta & (uint32_t)FOC_ANGLE_FXP_MASK);
}

static inline float foc_left_user_rpm_to_motor_rpm(float rpm)
{
    return rpm * (float)FOC_LEFT_USER_RPM_SIGN;
}

static inline float foc_right_user_rpm_to_motor_rpm(float rpm)
{
    return rpm * (float)FOC_RIGHT_USER_RPM_SIGN;
}

static inline float foc_left_motor_rpm_to_user_rpm(float rpm)
{
    return rpm * (float)FOC_LEFT_USER_RPM_SIGN;
}

static inline float foc_right_motor_rpm_to_user_rpm(float rpm)
{
    return rpm * (float)FOC_RIGHT_USER_RPM_SIGN;
}

typedef struct
{
    foc_angle_cfg_t angle_cfg;

    float desired_omega_m;
    float target_omega_m;
    float omega_m;

    float last_mech_rad;
    uint8_t has_last;

    PID speed_pid;
    float vq_limit;

    float vq_out;
} foc_speed_loop_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点限幅
// 参数说明     x               输入值
// 参数说明     lo              下限
// 参数说明     hi              上限
// 返回参数     float           限幅后的结果
// 使用示例     foc_clampf(u, -1.0f, 1.0f);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_clampf(float x, float lo, float hi);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将角度 wrap 到 [0, 2π)
// 参数说明     x               输入角度（rad）
// 返回参数     float           wrap 后的角度（rad）
// 使用示例     foc_wrap_0_2pi(theta);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_wrap_0_2pi(float x);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将角度 wrap 到 (-π, π]
// 参数说明     x               输入角度（rad）
// 返回参数     float           wrap 后的角度（rad）
// 使用示例     foc_wrap_pm_pi(delta);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_wrap_pm_pi(float x);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     机械角速度估算（rad/s）
// 参数说明     loop            速度环上下文
// 参数说明     mech_rad        当前机械角（rad）
// 参数说明     dt              采样周期（s）
// 返回参数     float           估算得到的角速度（rad/s）
// 使用示例     foc_speed_estimate_radps(&loop, mech, dt);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_speed_estimate_radps(foc_speed_loop_t *loop, float mech_rad, float dt);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度 PI 单步计算
// 参数说明     loop            速度环上下文
// 参数说明     dt              控制周期（s）
// 返回参数     float           vq 输出（归一化/限幅后）
// 使用示例     foc_speed_pi_step(&loop, dt);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_speed_pi_step(foc_speed_loop_t *loop, float dt);

// 速度环上升沿复位：用于 g_run_enable 从 0->1 时，避免首次输出突变
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度环使能时复位内部状态
// 参数说明     loop            速度环上下文
// 返回参数     void
// 使用示例     foc_speed_loop_reset_on_enable(&loop);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void foc_speed_loop_reset_on_enable(foc_speed_loop_t *loop);

// 速度环默认参数（可在工程中通过宏覆盖）
#ifndef FOC_SPEED_KP
#define FOC_SPEED_KP                    (0.02f)
#endif

#ifndef FOC_SPEED_KI
#define FOC_SPEED_KI                    (2.0f)
#endif

#ifndef FOC_VQ_LIMIT_FRAC
#define FOC_VQ_LIMIT_FRAC               (0.70f)
#endif

#ifndef FOC_SPEED_DEADZONE_RADPS
#define FOC_SPEED_DEADZONE_RADPS        (0.1f)
#endif

// ======================== 速度估算抗毛刺参数 ========================
#ifndef FOC_SPEED_EST_GLITCH_ENABLE
#define FOC_SPEED_EST_GLITCH_ENABLE      (1)
#endif

#ifndef FOC_SPEED_EST_GLITCH_SCALE
#define FOC_SPEED_EST_GLITCH_SCALE       (4.0f)
#endif

#ifndef FOC_SPEED_EST_GLITCH_MARGIN_RPM
#define FOC_SPEED_EST_GLITCH_MARGIN_RPM  (300.0f)
#endif

#ifndef FOC_SPEED_EST_GLITCH_MIN_RPM
#define FOC_SPEED_EST_GLITCH_MIN_RPM     (800.0f)
#endif

#ifndef FOC_SPEED_EST_GLITCH_MAX_RPM
#define FOC_SPEED_EST_GLITCH_MAX_RPM     (6000.0f)
#endif

// ======================== 软起动限制保护参数 ========================
#ifndef FOC_MAX_ACCEL_RPM_PS
#define FOC_MAX_ACCEL_RPM_PS            (10000.0f)
#endif

#ifndef FOC_MAX_DECEL_RPM_PS
#define FOC_MAX_DECEL_RPM_PS            (20000.0f)
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度环默认参数初始化
// 参数说明     loop            速度环上下文
// 返回参数     void
// 使用示例     foc_speed_loop_init_default(&loop);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void foc_speed_loop_init_default(foc_speed_loop_t *loop);

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
void foc_speed_control_set_target_rpm(foc_speed_loop_t *loop_l, foc_speed_loop_t *loop_r, float left_rpm, float right_rpm);

//============================== 上电自检/标定（方向 + 电角零偏） ==============================
typedef struct
{
    float align_vd_frac;        // 对齐用 Vd = frac * vbus
    uint32_t align_ms;          // 对齐时长

    float dirtest_vq_frac;      // 方向测试用 Vq = frac * vbus
    uint32_t dirtest_settle_ms; // 方向测试空转时长（用于进入匀速段）
    uint32_t dirtest_ms;        // 方向测试测量时长（在匀速段取样）
    float dirtest_step_rad;     // 每 1ms 的电角步进(rad)
    float dirtest_min_move_rad; // 机械角变化小于该阈值则认为测试不可靠
} foc_calib_cfg_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     标定参数默认配置
// 参数说明     cfg             标定配置结构体
// 返回参数     void
// 使用示例     foc_calibrate_motor_default_cfg(&cfg);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void foc_calibrate_motor_default_cfg(foc_calib_cfg_t *cfg);

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
uint8_t foc_calibrate_motor(const foc_svpwm_t *svpwm,
                            foc_speed_loop_t *loop,
                            void *as5047p_dev,
                            void *drv8300_motor,
                            const foc_calib_cfg_t *cfg);

uint8_t foc_calibrate_both_motors(const foc_svpwm_t *svpwm,
                                  foc_speed_loop_t *loop_l,
                                  foc_speed_loop_t *loop_r,
                                  void *as5047p_dev_l,
                                  void *as5047p_dev_r,
                                  void *drv8300_motor_l,
                                  void *drv8300_motor_r,
                                  const foc_calib_cfg_t *cfg);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     SVPWM 上下文初始化
// 参数说明     ctx             SVPWM 上下文
// 参数说明     vbus            母线电压（V），<=0 时按 1.0 处理
// 参数说明     duty_max        PWM 最大计数（占空比满量程）
// 返回参数     void
// 使用示例     foc_svpwm_init(&svpwm, 1.0f, PWM_DUTY_MAX);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void foc_svpwm_init(foc_svpwm_t *ctx, float vbus, uint32_t duty_max);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AS5047P 原始角度转机械弧度
// 参数说明     as5047p_angle_14bit  AS5047P 14bit 原始角度
// 返回参数     float           机械角（rad）
// 使用示例     foc_as5047p_raw_to_mech_rad(raw);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_as5047p_raw_to_mech_rad(uint16_t as5047p_angle_14bit);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     机械角转电角
// 参数说明     mech_rad        机械角（rad）
// 参数说明     cfg             电角配置（极对数/方向/零偏）
// 返回参数     float           电角（rad，wrap 到 [0, 2π)）
// 使用示例     foc_mech_to_elec_rad(mech, &cfg);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
float foc_mech_to_elec_rad(float mech_rad, const foc_angle_cfg_t *cfg);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AS5047P 原始角度直接转电角（内联）
// 参数说明     as5047p_angle_14bit  AS5047P 14bit 原始角度
// 参数说明     cfg             电角配置
// 返回参数     float           电角（rad）
// 使用示例     foc_as5047p_raw_to_elec_rad(raw, &cfg);
// 备注信息     头文件内联
//-------------------------------------------------------------------------------------------------------------------
static inline float foc_as5047p_raw_to_elec_rad(uint16_t as5047p_angle_14bit, const foc_angle_cfg_t *cfg)
{
    return foc_mech_to_elec_rad(foc_as5047p_raw_to_mech_rad(as5047p_angle_14bit), cfg);
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
                                  uint32_t *duty_c);

// Fixed-point electrical angle version (theta_e_fxp: 0..4095 == 0..2π)
foc_svpwm_status_t foc_vdq_to_duty_fxp(const foc_svpwm_t *ctx,
                                      foc_fxp_angle_t theta_e_fxp,
                                      float vd,
                                      float vq,
                                      uint32_t *duty_a,
                                      uint32_t *duty_b,
                                      uint32_t *duty_c);

#ifdef __cplusplus
}
#endif

#define FOC_SVPWM_LUT_SIZE              (256u)

extern float g_foc_svpwm_lut_da[FOC_SVPWM_LUT_SIZE];
extern float g_foc_svpwm_lut_db[FOC_SVPWM_LUT_SIZE];
extern float g_foc_svpwm_lut_dc[FOC_SVPWM_LUT_SIZE];

void foc_svpwm_lut_init(const foc_svpwm_t *ctx);

foc_svpwm_status_t foc_vq_to_duty_lut(const foc_svpwm_t *ctx,
                                       foc_fxp_angle_t theta_e_fxp,
                                       float vq_frac,
                                       uint32_t *duty_a,
                                       uint32_t *duty_b,
                                       uint32_t *duty_c);

#endif
