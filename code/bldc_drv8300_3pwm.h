/*********************************************************************************************************************
* TC264 Opensourec Library (TC264 开源库)
*
* DRV8300 3PWM 无刷电机底层驱动（最小可用）
* - 适配：DRV8300 三路 PWM 输入模式（通常为 3 路高边 PWM，低边由驱动内部互补/死区处理）
* - 提供：双电机（左/右）三相 PWM 初始化与占空比更新
*
* 说明：
* - 本文件只做“PWM 输出层”的封装，不做换相/FOC/SVPWM。
* - 上层可结合 AS5047P 角度实现 FOC，或实现六步换相。
*********************************************************************************************************************/

#ifndef _code_bldc_drv8300_3pwm_h_
#define _code_bldc_drv8300_3pwm_h_

/*********************************************************************************************************************
* 文件：bldc_drv8300_3pwm.h
* 模块：控制算法层（第 5 层）—— 栅极驱动
* 职责：DRV8300 驱动对外接口与 PWM 通道定义
* 依赖：zf_common_typedef.h
* 风险等级：R3
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "zf_common/zf_common_typedef.h"
#include "zf_driver/zf_driver_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

// 默认 PWM 频率（Hz）
#ifndef BLDC_DRV8300_PWM_FREQ_HZ
#define BLDC_DRV8300_PWM_FREQ_HZ      (20000u)
#endif

#ifndef BLDC_DRV8300_L_U_PWM
#define BLDC_DRV8300_L_U_PWM          (ATOM0_CH4_P22_3)
#endif
#ifndef BLDC_DRV8300_L_V_PWM
#define BLDC_DRV8300_L_V_PWM          (ATOM0_CH5_P32_4)
#endif
#ifndef BLDC_DRV8300_L_W_PWM
#define BLDC_DRV8300_L_W_PWM          (ATOM0_CH3_P21_5)
#endif

#ifndef BLDC_DRV8300_R_U_PWM
#define BLDC_DRV8300_R_U_PWM          (ATOM0_CH2_P21_4)
#endif
#ifndef BLDC_DRV8300_R_V_PWM
#define BLDC_DRV8300_R_V_PWM          (ATOM0_CH1_P21_3)
#endif
#ifndef BLDC_DRV8300_R_W_PWM
#define BLDC_DRV8300_R_W_PWM          (ATOM0_CH0_P21_2)
#endif

typedef struct
{
    pwm_channel_enum u_pwm;
    pwm_channel_enum v_pwm;
    pwm_channel_enum w_pwm;
    uint32_t         pwm_freq_hz;
    uint8_t          inited;
} bldc_drv8300_3pwm_motor_t;

typedef struct
{
    bldc_drv8300_3pwm_motor_t left;
    bldc_drv8300_3pwm_motor_t right;
} bldc_drv8300_3pwm_dual_t;

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
                                 uint32_t pwm_freq_hz);

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
                                     uint32_t duty_w);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DRV8300 3PWM 电机停止（占空比清零）
// 参数说明     motor           电机句柄
// 返回参数     void
// 使用示例     bldc_drv8300_3pwm_motor_stop(&motor);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void bldc_drv8300_3pwm_motor_stop(bldc_drv8300_3pwm_motor_t *motor);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DRV8300 3PWM 双电机默认初始化
// 参数说明     dual            双电机句柄
// 返回参数     void
// 使用示例     bldc_drv8300_3pwm_dual_init_default(&dual);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void bldc_drv8300_3pwm_dual_init_default(bldc_drv8300_3pwm_dual_t *dual);

#ifdef __cplusplus
}
#endif

#endif
