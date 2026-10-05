/*********************************************************************************************************************
* 文件：isr.c
* 模块：应用层（第 6 层）—— 中断服务
* 职责：全部中断服务程序、FOC 周期控制环、运行时命令结构体
* 依赖：zf_common_headfile.h、本目录 isr.h、isr_config.h
* 风险等级：R3
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "isr_config.h"
#include "isr.h"

#if defined(__TASKING__)
    #include "infineon_libraries/iLLD/TC26B/Tricore/Cpu/Irq/IfxCpu_Irq.h"
#else
    #ifdef IFX_INTERRUPT
        #undef IFX_INTERRUPT
    #endif
    #define IFX_INTERRUPT(isr, vectabNum, prio) void isr(void)
#endif

#include "zf_driver/zf_driver_pit.h"
#include "zf_driver/zf_driver_exti.h"
#include "zf_driver/zf_driver_dma.h"

#include "IfxGtm_Atom.h"
#include "bldc_drv8300_3pwm.h"
#include "foc_svpwm.h"
#include "pid.h"

#include "zf_device/zf_device_imu660rb.h"
#include "imu_ahrs.h"

#include "menu.h"
#include "line_follow.h"
#include "move_distance.h"
#include "neg_pressure_motor.h"
#include "route_plan.h"
#include "wifi_service.h"
#include "battery_monitor.h"
#include "perf_monitor.h"

#include "IfxAsclin_Asc.h"
#include "zf_driver/zf_driver_gpio.h"

extern volatile uint8 wifi_uart_tx_stalled;

#pragma section code "cpu1_psram"

//============================== FOC 速度环 ==============================

#ifndef FOC_ENCODER_HOLD_CYCLES
#define FOC_ENCODER_HOLD_CYCLES         (150u)
#endif

#ifndef FOC_ISR_TIME_PROFILE_ENABLE
#define FOC_ISR_TIME_PROFILE_ENABLE     (1)
#endif

#ifndef FOC_ISR_TIME_PROFILE_PIN
#define FOC_ISR_TIME_PROFILE_PIN        (P02_6)
#endif

#if FOC_ISR_TIME_PROFILE_ENABLE
#define FOC_ISR_TIME_PROFILE_HIGH()     gpio_high(FOC_ISR_TIME_PROFILE_PIN)
#define FOC_ISR_TIME_PROFILE_LOW()      gpio_low(FOC_ISR_TIME_PROFILE_PIN)
#else
#define FOC_ISR_TIME_PROFILE_HIGH()     ((void)0)
#define FOC_ISR_TIME_PROFILE_LOW()      ((void)0)
#endif

// 直接 ATOM0 PWM 寄存器写入 — 省掉 pwm_set_duty 的查表及函数调用开销
// SVPWM 输出 [0, PWM_DUTY_MAX], 硬件周期在 pwm_init 中设置, 需等比缩放
extern uint32_t g_foc_pwm_period;

#define _PWM_W(ch, d)  IfxGtm_Atom_Ch_setCompareOneShadow(&MODULE_GTM.ATOM[0], (ch), \
    (uint32_t)(((uint64_t)(d) * (uint64_t)g_foc_pwm_period) / (uint64_t)PWM_DUTY_MAX))
#define _PWM_L(da,db,dc)  do { _PWM_W(4,(da)); _PWM_W(5,(db)); _PWM_W(3,(dc)); } while(0)
#define _PWM_R(da,db,dc)  do { _PWM_W(2,(da)); _PWM_W(1,(db)); _PWM_W(0,(dc)); } while(0)
#define _PWM_L_STOP()     _PWM_L(0,0,0)
#define _PWM_R_STOP()     _PWM_R(0,0,0)
#define _PWM_BOTH_STOP()  do { _PWM_L_STOP(); _PWM_R_STOP(); } while(0)

static inline float foc_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

typedef struct
{
    uint32_t seq;
    uint8_t run_enable;
    uint8_t reserved0;
    uint16_t reserved1;
    float left_target_rpm;
    float right_target_rpm;
} foc_runtime_cmd_t;

static volatile foc_runtime_cmd_t g_foc_runtime_cmd = {0u, 0u, 0u, 0u, 0.0f, 0.0f};

static void foc_runtime_cmd_publish_isr(uint8_t run_enable, float left_target_rpm, float right_target_rpm)
{
    uint32_t seq = g_foc_runtime_cmd.seq;

    g_foc_runtime_cmd.seq = (uint32_t)(seq + 1u);
    g_foc_runtime_cmd.run_enable = (uint8_t)(run_enable ? 1u : 0u);
    g_foc_runtime_cmd.left_target_rpm = left_target_rpm;
    g_foc_runtime_cmd.right_target_rpm = right_target_rpm;
    g_foc_runtime_cmd.seq = (uint32_t)(seq + 2u);
}

static void foc_runtime_cmd_load(uint8_t *run_enable, float *left_target_rpm, float *right_target_rpm)
{
    uint32_t seq0;
    uint32_t seq1;
    uint8_t run_local;
    float left_local;
    float right_local;

    if ((run_enable == NULL) || (left_target_rpm == NULL) || (right_target_rpm == NULL))
    {
        return;
    }

    do
    {
        seq0 = g_foc_runtime_cmd.seq;
        run_local = g_foc_runtime_cmd.run_enable;
        left_local = g_foc_runtime_cmd.left_target_rpm;
        right_local = g_foc_runtime_cmd.right_target_rpm;
        seq1 = g_foc_runtime_cmd.seq;
    } while (((seq0 & 1u) != 0u) || (seq0 != seq1));

    *run_enable = run_local;
    *left_target_rpm = left_local;
    *right_target_rpm = right_local;
}

static float foc_get_runtime_vbus(void)
{
    float vbus = battery_monitor_get_voltage_v();

    if (vbus <= 0.0f)
    {
        return 1.0f;
    }

    float min_v = 6.6f;
    float max_v = 8.4f;

    if (g_battery_cells == 4) 
    {
        min_v = 13.2f;
        max_v = 16.8f;
    }
    else if (g_battery_cells == 3)
    {
        min_v = 9.9f;
        max_v = 12.6f;
    }
    else if (g_battery_cells == 6)
    {
        min_v = 19.8f;
        max_v = 25.2f;
    }
    else 
    {
        min_v = 6.6f;
        max_v = 8.4f;
    }

    return foc_clampf(vbus, min_v, max_v);
}

volatile float   g_motor_mech_l_rad = 0.0f;
volatile float   g_motor_mech_r_rad = 0.0f;
volatile uint8_t g_motor_mech_l_valid = 0u;
volatile uint8_t g_motor_mech_r_valid = 0u;
volatile float g_motor_left_actual_rpm = 0.0f;
volatile float g_motor_right_actual_rpm = 0.0f;
volatile float g_motor_elec_l_deg = 0.0f;
volatile float g_motor_elec_r_deg = 0.0f;

volatile float g_vbus = 8.4f;
volatile float g_inv_vbus = 0.1190476f;
volatile uint16_t g_lf_actual_hz = 0u;
volatile uint32_t g_cd_ticks = 0u;

/**
 * @brief 灰度传感器读值
 */
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     CCU60 CH1 PIT 中断服务
// 返回参数     void
// 使用示例     cc60_pit_ch1_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    uint32_t _isr_t0 = perf_monitor_get_tick();
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);

    line_follow_update(0.001f);
    perf_monitor_record_cpu0_isr(_isr_t0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     CCU60 CH0 PIT 中断服务（IMU 采样与姿态更新）
// 返回参数     void
// 使用示例     cc60_pit_ch0_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    uint32_t _isr_t0 = perf_monitor_get_tick();
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);

    static uint8_t vbus_cnt = 0;
    if (++vbus_cnt >= 10)
    {
        vbus_cnt = 0;
        g_vbus = foc_get_runtime_vbus();
        if (g_vbus <= 0.0f) g_vbus = 1.0f;
        g_inv_vbus = 1.0f / g_vbus;
    }

    imu660rb_get_acc();
    imu660rb_get_gyro();

    if (imu_ahrs_check_stale(imu660rb_acc_x, imu660rb_acc_y, imu660rb_acc_z, imu660rb_gyro_x, imu660rb_gyro_y, imu660rb_gyro_z))
    {
        g_run_enable = 0;
    }

    float gx = imu660rb_gyro_transition(imu660rb_gyro_x) - g_gyro_bias_x;
    float gy = imu660rb_gyro_transition(imu660rb_gyro_y) - g_gyro_bias_y;
    float gz = imu660rb_gyro_transition(imu660rb_gyro_z) - g_gyro_bias_z;


    static uint8_t inited = 0u;
    const float alpha = (float)g_gyro_lp_alpha;
    if (!inited)
    {
        g_gyro_x_dps_f = gx;
        g_gyro_y_dps_f = gy;
        g_gyro_z_dps_f = gz;
        inited = 1u;
    }
    else
    {
        g_gyro_x_dps_f += alpha * (gx - g_gyro_x_dps_f);
        g_gyro_y_dps_f += alpha * (gy - g_gyro_y_dps_f);
        g_gyro_z_dps_f += alpha * (gz - g_gyro_z_dps_f);
    }


    float ax = imu660rb_acc_transition(imu660rb_acc_x);
    float ay = imu660rb_acc_transition(imu660rb_acc_y);
    float az = imu660rb_acc_transition(imu660rb_acc_z);

    {
        static float acc_lx = 0.0f, acc_ly = 0.0f, acc_lz = 0.0f;
        static uint8_t acc_lp_inited = 0u;
        if (!acc_lp_inited)
        {
            acc_lx = ax; acc_ly = ay; acc_lz = az;
            acc_lp_inited = 1u;
        }
        else
        {
            const float acc_alpha = 0.05f;
            acc_lx += acc_alpha * (ax - acc_lx);
            acc_ly += acc_alpha * (ay - acc_ly);
            acc_lz += acc_alpha * (az - acc_lz);
        }
        ax = acc_lx; ay = acc_ly; az = acc_lz;
    }

    imu_ahrs_update_imu(&g_ahrs, (float)g_gyro_x_dps_f, (float)g_gyro_y_dps_f, (float)g_gyro_z_dps_f, ax, ay, az, 0.005f);

    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    imu_ahrs_get_euler_rad(&g_ahrs, &roll, &pitch, &yaw);
    const float rad2deg = 57.295779513082320876f;
    g_att_roll_deg = roll * rad2deg;
    g_att_pitch_deg = pitch * rad2deg;
    g_att_yaw_deg = yaw * rad2deg;

    // UART0 200Hz 位置/停止输出: >=4 通道命中发 S, 否则发 P,pos
    {
        uint16_t bits = line_follow_get_last_bits();
        uint8_t hit_cnt = 0;
        {
            uint16_t x = bits;
            while (x) { hit_cnt++; x &= x - 1; }
        }

        if (hit_cnt >= 4) {
            uart_write_buffer(UART_0, (const uint8 *)"S\r\n", 3u);
        } else {
            float pos;
            if (line_follow_get_last_pos(&pos)) {
                char buf[16];
                int len = snprintf(buf, sizeof(buf), "P,%.2f\r\n", (double)pos);
                if (len > 0 && len < (int)sizeof(buf)) {
                    uart_write_buffer(UART_0, (const uint8 *)buf, (uint32)len);
                }
            }
        }
    }

    perf_monitor_record_cpu0_isr(_isr_t0);
}

static inline void foc_motor_control_step(
    foc_speed_loop_t *loop,
    uint8_t is_left,
    foc_svpwm_t *svpwm_ctx,
    uint8_t control_angle_ok,
    foc_fxp_angle_t theta_hold,
    uint8_t enc_fail_count,
    float dt)
{
    float vq_frac = foc_speed_pi_step(loop, dt);
    uint32_t da = 0, db = 0, dc = 0;

    if (enc_fail_count > 0 && enc_fail_count < (uint8_t)FOC_ENCODER_HOLD_CYCLES)
    {
        float ramp_factor = 1.0f - ((float)enc_fail_count / (float)FOC_ENCODER_HOLD_CYCLES);
        vq_frac *= ramp_factor;
    }

    if (control_angle_ok)
    {
        if(FOC_SVPWM_OK == foc_vq_to_duty_lut(svpwm_ctx, theta_hold, vq_frac, &da, &db, &dc))
        {
            if (is_left) _PWM_L(da, db, dc); else _PWM_R(da, db, dc);
        }
        else
        {
            if (is_left) _PWM_L_STOP(); else _PWM_R_STOP();
        }
    }
    else
    {
        if (is_left) _PWM_L_STOP(); else _PWM_R_STOP();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     CCU61 CH0 PIT 中断服务（FOC 控制周期）
// 返回参数     void
// 使用示例     cc61_pit_ch0_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
void foc_ctrl_periodic_isr(void)
{
#ifdef HARDWARE_STRIPPED
    {
        interrupt_global_enable(0);
        pit_clear_flag(CCU61_CH0);
        perf_monitor_record_foc_isr(perf_monitor_get_tick());
        return;
    }
#endif
    const float dt = (float)FOC_CTRL_PERIOD_US * 1e-6f;
    uint32_t t0 = perf_monitor_get_tick();
    const float radps_to_rpm = 9.549296585513720147f;
    const float speed_est_period_s = 1.0f / (float)FOC_SPEED_ESTIMATE_HZ;
    const float raw_to_mech_rad = FOC_TWO_PI / 16384.0f;
    const float as5047_alpha = (float)g_as5047p_lp_alpha;
    float speed_avg_samples_f = g_as5047p_speed_avg_samples;
    uint16_t speed_avg_samples = 1u;
    float speed_zero_deadband_rpm = g_as5047p_speed_zero_deadband_rpm;
    float speed_zero_deadband_radps = 0.0f;
    uint16_t angle_raw_l = 0;
    uint16_t angle_raw_r = 0;
    uint8_t angle_ok_l = 0u;
    uint8_t angle_ok_r = 0u;
    uint8_t control_angle_ok_l = 0u;
    uint8_t control_angle_ok_r = 0u;
    float mech_l = 0.0f;
    float mech_r = 0.0f;
    static uint8_t last_run_enable = 0;
    static uint8_t enc_fail_count_l = 0u;
    static uint8_t enc_fail_count_r = 0u;
    static uint8_t mech_hold_valid_l = 0u;
    static uint8_t mech_hold_valid_r = 0u;
    static float mech_hold_l = 0.0f;
    static float mech_hold_r = 0.0f;
    static foc_fxp_angle_t theta_hold_l = 0;
    static foc_fxp_angle_t theta_hold_r = 0;
    static uint16_t speed_avg_cnt_l = 0u;
    static uint16_t speed_avg_cnt_r = 0u;
    static float speed_avg_sum_l = 0.0f;
    static float speed_avg_sum_r = 0.0f;
    static float speed_est_elapsed_l = 0.0f;
    static float speed_est_elapsed_r = 0.0f;
    uint8_t cmd_run_enable = 0u;
    float left_target_rpm = 0.0f;
    float right_target_rpm = 0.0f;
    uint8_t speed_sample_l = 0u;
    uint8_t speed_sample_r = 0u;
    foc_svpwm_t svpwm_ctx;

    interrupt_global_enable(0);                    
    pit_clear_flag(CCU61_CH0);
    FOC_ISR_TIME_PROFILE_HIGH();

    if (speed_avg_samples_f < 1.0f)
    {
        speed_avg_samples_f = 1.0f;
    }
    else if (speed_avg_samples_f > 100.0f)
    {
        speed_avg_samples_f = 100.0f;
    }
    speed_avg_samples = (uint16_t)(speed_avg_samples_f + 0.5f);
    if (speed_avg_samples < 1u)
    {
        speed_avg_samples = 1u;
    }

    if (speed_zero_deadband_rpm < 0.0f)
    {
        speed_zero_deadband_rpm = 0.0f;
    }
    else if (speed_zero_deadband_rpm > 20.0f)
    {
        speed_zero_deadband_rpm = 20.0f;
    }
    speed_zero_deadband_radps = speed_zero_deadband_rpm * FOC_RPM_TO_RADPS;

    foc_runtime_cmd_load(&cmd_run_enable, &left_target_rpm, &right_target_rpm);

    if(!g_run_enable)
    {
        cmd_run_enable = 0u;
    }

    uint8_t run_now = 0;
    uint8_t falling_edge = 0;
    {
        run_now = (uint8_t)(cmd_run_enable ? 1u : 0u);
        falling_edge = (!run_now && last_run_enable);
        if(run_now && (!last_run_enable))
        {
            foc_speed_loop_reset_on_enable(&g_loop_l);
            foc_speed_loop_reset_on_enable(&g_loop_r);
            speed_avg_cnt_l = 0u;
            speed_avg_cnt_r = 0u;
            speed_avg_sum_l = 0.0f;
            speed_avg_sum_r = 0.0f;
            speed_est_elapsed_l = 0.0f;
            speed_est_elapsed_r = 0.0f;
        }
        last_run_enable = run_now;
    }

    if(AS5047P_OK == as5047p_read_angle_raw(&g_enc_l, &angle_raw_l))
    {
        uint8_t recovered = enc_fail_count_l;
        angle_ok_l = 1u;
        control_angle_ok_l = 1u;
        mech_l = (float)((uint16_t)angle_raw_l & 0x3FFFu) * raw_to_mech_rad;
        theta_hold_l = foc_mech_fxp_to_elec_fxp(foc_as5047p_raw_to_mech_fxp((uint16_t)angle_raw_l), &g_loop_l.angle_cfg);
        mech_hold_l = mech_l;
        mech_hold_valid_l = 1u;
        enc_fail_count_l = 0u;
        if(recovered)
        {
            g_loop_l.has_last = 0u;
            speed_avg_cnt_l = 0u;
            speed_avg_sum_l = 0.0f;
            speed_est_elapsed_l = 0.0f;
        }
        else
        {
            speed_sample_l = 1u;
        }
    }
    else if(mech_hold_valid_l && (enc_fail_count_l < (uint8_t)FOC_ENCODER_HOLD_CYCLES))
    {
        enc_fail_count_l++;
        control_angle_ok_l = 1u;
        mech_hold_l += g_loop_l.omega_m * dt;
        mech_l = mech_hold_l;
        foc_fxp_angle_t mech_fxp = (foc_fxp_angle_t)(mech_l * FOC_RAD_TO_FXP);
        theta_hold_l = foc_mech_fxp_to_elec_fxp(mech_fxp & FOC_ANGLE_FXP_MASK, &g_loop_l.angle_cfg);
    }
    else
    {
    }

    if(AS5047P_OK == as5047p_read_angle_raw(&g_enc_r, &angle_raw_r))
    {
        uint8_t recovered = enc_fail_count_r;
        angle_ok_r = 1u;
        control_angle_ok_r = 1u;
        mech_r = (float)((uint16_t)angle_raw_r & 0x3FFFu) * raw_to_mech_rad;
        theta_hold_r = foc_mech_fxp_to_elec_fxp(foc_as5047p_raw_to_mech_fxp((uint16_t)angle_raw_r), &g_loop_r.angle_cfg);
        mech_hold_r = mech_r;
        mech_hold_valid_r = 1u;
        enc_fail_count_r = 0u;
        if(recovered)
        {
            g_loop_r.has_last = 0u;
            speed_avg_cnt_r = 0u;
            speed_avg_sum_r = 0.0f;
            speed_est_elapsed_r = 0.0f;
        }
        else
        {
            speed_sample_r = 1u;
        }
    }
    else if(mech_hold_valid_r && (enc_fail_count_r < (uint8_t)FOC_ENCODER_HOLD_CYCLES))
    {
        enc_fail_count_r++;
        control_angle_ok_r = 1u;
        mech_hold_r += g_loop_r.omega_m * dt;
        mech_r = mech_hold_r;
        foc_fxp_angle_t mech_fxp = (foc_fxp_angle_t)(mech_r * FOC_RAD_TO_FXP);
        theta_hold_r = foc_mech_fxp_to_elec_fxp(mech_fxp & FOC_ANGLE_FXP_MASK, &g_loop_r.angle_cfg);
    }
    else
    {
    }

    g_motor_mech_l_rad = mech_l;
    g_motor_mech_r_rad = mech_r;
    g_motor_mech_l_valid = angle_ok_l;
    g_motor_mech_r_valid = angle_ok_r;

    {
        static uint8_t elec_deg_cnt = 0u;
        if (++elec_deg_cnt >= 10u)
        {
            elec_deg_cnt = 0u;
            const float elec_deg_scale = (360.0f / (float)FOC_ANGLE_FXP_RESOLUTION);
            if (control_angle_ok_l)
                g_motor_elec_l_deg = (float)((uint32_t)theta_hold_l & (uint32_t)FOC_ANGLE_FXP_MASK) * elec_deg_scale;
            if (control_angle_ok_r)
                g_motor_elec_r_deg = (float)((uint32_t)theta_hold_r & (uint32_t)FOC_ANGLE_FXP_MASK) * elec_deg_scale;
        }
    }

    if(control_angle_ok_l)
    {
        if(speed_sample_l)
        {
            speed_est_elapsed_l += dt;

            // 速度估计固定在 1kHz，避免 10kHz 下量化噪声放大
            if(speed_est_elapsed_l >= speed_est_period_s)
            {
                float omega_raw = foc_speed_estimate_radps(&g_loop_l, mech_l, speed_est_elapsed_l);
                speed_est_elapsed_l = 0.0f;

                speed_avg_sum_l += omega_raw;
                speed_avg_cnt_l++;
                if(speed_avg_cnt_l >= speed_avg_samples)
                {
                    float omega_avg = speed_avg_sum_l / (float)speed_avg_cnt_l;
                    g_loop_l.omega_m += as5047_alpha * (omega_avg - g_loop_l.omega_m);

                    if(foc_absf(g_loop_l.omega_m) < speed_zero_deadband_radps)
                    {
                        g_loop_l.omega_m = 0.0f;
                    }

                    speed_avg_cnt_l = 0u;
                    speed_avg_sum_l = 0.0f;
                }
            }
        }
        g_motor_left_actual_rpm = foc_left_motor_rpm_to_user_rpm(g_loop_l.omega_m * radps_to_rpm);
    }
    else
    {
        g_loop_l.has_last = 0u;
        g_loop_l.omega_m = 0.0f;
        speed_avg_cnt_l = 0u;
        speed_avg_sum_l = 0.0f;
        speed_est_elapsed_l = 0.0f;
        g_motor_left_actual_rpm = 0.0f;
    }
    if(control_angle_ok_r)
    {
        if(speed_sample_r)
        {
            speed_est_elapsed_r += dt;

            if(speed_est_elapsed_r >= speed_est_period_s)
            {
                float omega_raw = foc_speed_estimate_radps(&g_loop_r, mech_r, speed_est_elapsed_r);
                speed_est_elapsed_r = 0.0f;

                speed_avg_sum_r += omega_raw;
                speed_avg_cnt_r++;
                if(speed_avg_cnt_r >= speed_avg_samples)
                {
                    float omega_avg = speed_avg_sum_r / (float)speed_avg_cnt_r;
                    g_loop_r.omega_m += as5047_alpha * (omega_avg - g_loop_r.omega_m);

                    if(foc_absf(g_loop_r.omega_m) < speed_zero_deadband_radps)
                    {
                        g_loop_r.omega_m = 0.0f;
                    }

                    speed_avg_cnt_r = 0u;
                    speed_avg_sum_r = 0.0f;
                }
            }
        }
        g_motor_right_actual_rpm = foc_right_motor_rpm_to_user_rpm(g_loop_r.omega_m * radps_to_rpm);
    }
    else
    {
        g_loop_r.has_last = 0u;
        g_loop_r.omega_m = 0.0f;
        speed_avg_cnt_r = 0u;
        speed_avg_sum_r = 0.0f;
        speed_est_elapsed_r = 0.0f;
        g_motor_right_actual_rpm = 0.0f;
    }
    
    if (g_menu_state == MENU_BLDC_OPEN_LOOP_TEST)
    {
        extern volatile float g_open_loop_hz;
        extern volatile float g_open_loop_vq;
        extern volatile uint8_t g_open_loop_run;

        svpwm_ctx = g_svpwm;
        svpwm_ctx.vbus = g_vbus;
        svpwm_ctx.inv_vbus = g_inv_vbus;

        if (g_open_loop_run)
        {
            static foc_fxp_angle_t dummy_theta_fxp = 0;
            const foc_fxp_angle_t delta = (foc_fxp_angle_t)(g_open_loop_hz * dt * (float)FOC_ANGLE_FXP_RESOLUTION);
            dummy_theta_fxp = (foc_fxp_angle_t)((uint32_t)(dummy_theta_fxp + delta) & (uint32_t)FOC_ANGLE_FXP_MASK);

            uint32_t da, db, dc;

            if (FOC_SVPWM_OK == foc_vq_to_duty_lut(&svpwm_ctx, dummy_theta_fxp, g_open_loop_vq, &da, &db, &dc))
            {
                _PWM_L(da, db, dc);
                _PWM_R(da, db, dc);
            }
            else
            {
                _PWM_BOTH_STOP();
            }
        }
        else
        {
            _PWM_BOTH_STOP();
        }
        perf_monitor_record_foc_isr(t0);
        FOC_ISR_TIME_PROFILE_LOW();
        return;
    }

    {
        static uint8_t foc_test_last_run = 0u;
        static float left_test_cmd_rpm = 0.0f;
        static float right_test_cmd_rpm = 0.0f;
        const uint8_t foc_test_was_running = foc_test_last_run;

        if ((g_menu_state == MENU_FOC_SPEED_DEBUG) && (g_foc_test_run != 0u))
        {
            const float left_target = g_foc_test_left_target_rpm;
            const float right_target = g_foc_test_right_target_rpm;
            const float test_ramp_step = 2000.0f * dt;

            if (!foc_test_last_run)
            {
                left_test_cmd_rpm = 0.0f;
                right_test_cmd_rpm = 0.0f;
                foc_speed_loop_reset_on_enable(&g_loop_l);
                foc_speed_loop_reset_on_enable(&g_loop_r);
            }
            foc_test_last_run = 1u;

            if (left_target > (left_test_cmd_rpm + test_ramp_step))
            {
                left_test_cmd_rpm += test_ramp_step;
            }
            else if (left_target < (left_test_cmd_rpm - test_ramp_step))
            {
                left_test_cmd_rpm -= test_ramp_step;
            }
            else
            {
                left_test_cmd_rpm = left_target;
            }

            if (right_target > (right_test_cmd_rpm + test_ramp_step))
            {
                right_test_cmd_rpm += test_ramp_step;
            }
            else if (right_target < (right_test_cmd_rpm - test_ramp_step))
            {
                right_test_cmd_rpm -= test_ramp_step;
            }
            else
            {
                right_test_cmd_rpm = right_target;
            }

            g_target_left_rpm = left_test_cmd_rpm;
            g_target_right_rpm = right_test_cmd_rpm;
            foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, left_test_cmd_rpm, right_test_cmd_rpm);

            svpwm_ctx = g_svpwm;
            svpwm_ctx.vbus = g_vbus;
            svpwm_ctx.inv_vbus = g_inv_vbus;

            foc_motor_control_step(&g_loop_l, 1, &svpwm_ctx, control_angle_ok_l, theta_hold_l, enc_fail_count_l, dt);
            foc_motor_control_step(&g_loop_r, 0, &svpwm_ctx, control_angle_ok_r, theta_hold_r, enc_fail_count_r, dt);

            perf_monitor_record_foc_isr(t0);
            FOC_ISR_TIME_PROFILE_LOW();
            return;
        }

        // Safety interlock:
        // If we were running FOC test in the previous ISR tick and now we are not,
        // stop PWM deterministically and clear the test flag. Otherwise the last duty
        // could remain latched when leaving the FOC page (cmd_run_enable may already be 0,
        // so the normal falling-edge stop path won't execute).
        if (foc_test_was_running)
        {
            g_foc_test_run = 0u;
            g_target_left_rpm = 0.0f;
            g_target_right_rpm = 0.0f;
            foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);
            foc_speed_loop_reset_on_enable(&g_loop_l);
            foc_speed_loop_reset_on_enable(&g_loop_r);
            _PWM_BOTH_STOP();
        }

        foc_test_last_run = 0u;
        left_test_cmd_rpm = 0.0f;
        right_test_cmd_rpm = 0.0f;
    }

    static float current_left_target = 0.0f;
    static float current_right_target = 0.0f;

    if(!cmd_run_enable)
    {
        if (falling_edge) // Only on falling edge
        {
            current_left_target = 0.0f;
            current_right_target = 0.0f;
            foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);
            _PWM_BOTH_STOP();

            speed_avg_cnt_l = 0u;
            speed_avg_cnt_r = 0u;
            speed_avg_sum_l = 0.0f;
            speed_avg_sum_r = 0.0f;
            speed_est_elapsed_l = 0.0f;
            speed_est_elapsed_r = 0.0f;
        }

        perf_monitor_record_foc_isr(t0);
        FOC_ISR_TIME_PROFILE_LOW();
        return;
    }

    if (left_target_rpm == 0.0f && right_target_rpm == 0.0f)
    {
    }

    current_left_target = left_target_rpm;
    current_right_target = right_target_rpm;

    foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, current_left_target, current_right_target);

    svpwm_ctx = g_svpwm;
    svpwm_ctx.vbus = g_vbus;
    svpwm_ctx.inv_vbus = g_inv_vbus;

    //================ 左电机 ================
    foc_motor_control_step(&g_loop_l, 1, &svpwm_ctx, control_angle_ok_l, theta_hold_l, enc_fail_count_l, dt);

    //================ 右电机 ================
    foc_motor_control_step(&g_loop_r, 0, &svpwm_ctx, control_angle_ok_r, theta_hold_r, enc_fail_count_r, dt);

    perf_monitor_record_foc_isr(t0);
    FOC_ISR_TIME_PROFILE_LOW();

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     CCU61 CH1 PIT 中断服务
// 返回参数     void
// 使用示例     cc61_pit_ch1_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
#ifdef HARDWARE_STRIPPED
    {
        static uint8_t last_run_enable = 0u;
        interrupt_global_enable(0);
        pit_clear_flag(CCU61_CH1);
        uint32_t t0 = perf_monitor_get_tick();

        if (g_cd_ticks > 0u) { g_cd_ticks--; }

        // LF rate monitoring
        {
            static uint16_t lf_rate_timer = 0u;
            static uint16_t lf_cnt_last = 0u;
            static uint8_t lf_rate_inited = 0u;
            lf_rate_timer++;
            if (!lf_rate_inited) {
                lf_cnt_last = line_follow_get_frame_cnt();
                lf_rate_inited = 1u;
            } else if (lf_rate_timer >= 250u) {
                uint16_t cnt_now = line_follow_get_frame_cnt();
                g_lf_actual_hz = (uint16_t)((uint16_t)(cnt_now - lf_cnt_last) * 4u);
                lf_cnt_last = cnt_now;
                lf_rate_timer = 0u;
            }
        }

        // Edge detection: reset route + IMU on run_enable rising edge
        {
            uint8_t run_now = (uint8_t)(g_run_enable ? 1u : 0u);
            if (run_now && !last_run_enable) {
                route_plan_reset();
                imu_ahrs_reset_attitude_only(&g_ahrs);
            }
            last_run_enable = run_now;
        }

        wifi_service_capture_snapshot_isr();
        perf_monitor_record_isr1k(t0);
        return;
    }
#endif
    const float dt = 0.001f;
    uint32_t t0 = perf_monitor_get_tick();
    static uint8_t last_run_enable = 0u;
    static float base_cmd_rpm = 0.0f;
    static float delta_cmd_rpm = 0.0f;
    static uint16_t startup_ramp_ticks = 0u;
    uint8_t neg_pressure_run_request;
    float mech_l = g_motor_mech_l_rad;
    float mech_r = g_motor_mech_r_rad;
    bool mech_l_valid = (g_motor_mech_l_valid != 0u);
    bool mech_r_valid = (g_motor_mech_r_valid != 0u);

    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH1);

    if (g_cd_ticks > 0u)
    {
        g_cd_ticks--;
    }

    {
        static uint16_t lf_rate_timer = 0u;
        static uint16_t lf_cnt_last = 0u;
        static uint8_t lf_rate_inited = 0u;
        lf_rate_timer++;
        if (!lf_rate_inited)
        {
            lf_cnt_last = line_follow_get_frame_cnt();
            lf_rate_inited = 1u;
        }
        else if (lf_rate_timer >= 250u)
        {
            uint16_t cnt_now = line_follow_get_frame_cnt();
            g_lf_actual_hz = (uint16_t)((uint16_t)(cnt_now - lf_cnt_last) * 4u);
            lf_cnt_last = cnt_now;
            lf_rate_timer = 0u;
        }
    }

    {
        uint8_t run_now = (uint8_t)(g_run_enable ? 1u : 0u);
        if(run_now && (!last_run_enable))
        {
            route_plan_reset();
            move_distance_odom_reset();
            base_cmd_rpm = 0.0f;
            delta_cmd_rpm = 0.0f;
            startup_ramp_ticks = 0u;
            imu_ahrs_reset_attitude_only(&g_ahrs);
        }
        last_run_enable = run_now;
    }

    // 独立人工里程计只依赖编码器，不应绑定在主流程 run_enable 上。
    move_distance_odom_update(mech_l, mech_l_valid, mech_r, mech_r_valid);
    neg_pressure_run_request = (uint8_t)((g_run_enable != 0u)
                                      || g_paper_start_neg_enable
                                      || ((g_menu_state == MENU_NEG_PRESSURE_DEBUG) && (g_neg_pressure_enable != 0u)));

    if(!g_run_enable)
    {
        base_cmd_rpm = 0.0f;
        delta_cmd_rpm = 0.0f;
        g_target_left_rpm = 0.0f;
        g_target_right_rpm = 0.0f;
        foc_runtime_cmd_publish_isr(0u, 0.0f, 0.0f);
        // 负压风扇采用“或关系”：
        // 1) 主程序运行时自动开启；
        // 2) 待机时只有在 NEG 调试页内手动使能才开启；
        // 其余状态一律关闭。
        neg_pressure_motor_update(&g_neg_pressure_motor, dt, neg_pressure_run_request);
        wifi_service_capture_snapshot_isr();
        perf_monitor_record_isr1k(t0);
        return;
    }

    if(move_distance_is_active())
    {
        move_distance_mode_t distance_mode = move_distance_get_mode();

        if(move_distance_update(mech_l, mech_l_valid, mech_r, mech_r_valid, dt))
        {
            if (distance_mode == MOVE_DISTANCE_MODE_LINE_FOLLOW || distance_mode == MOVE_DISTANCE_MODE_STRAIGHT_LF)
            {
                line_follow_on_distance_complete();
                if (g_run_enable != 0u)
                {
                    goto distance_complete_handled;
                }
            }
            else
            {
                if (g_stop_reason == SYSTEM_STOP_REASON_NONE)
                {
                    g_stop_reason = SYSTEM_STOP_REASON_ROUTE_DONE;
                }
                g_run_enable = 0u;
            }

            g_target_left_rpm = 0.0f;
            g_target_right_rpm = 0.0f;
            foc_runtime_cmd_publish_isr(0u, 0.0f, 0.0f);
            neg_pressure_motor_update(&g_neg_pressure_motor, dt, neg_pressure_run_request);
            wifi_service_capture_snapshot_isr();
            perf_monitor_record_isr1k(t0);
            return;
        }
    }

distance_complete_handled:

    {
        float base_ref = g_base_speed_rpm;
        {
            route_plan_entry_t next_entry;
            if (route_plan_peek(&next_entry))
            {
                route_turn_t t = next_entry.turn;
                if (t != ROUTE_TURN_LEFT && t != ROUTE_TURN_RIGHT
                    && t != ROUTE_ELEM_DIODE_FORK_L_TURN && t != ROUTE_ELEM_DIODE_FORK_R_TURN
                    && t != ROUTE_ELEM_DIODE2_FORK_L_TURN && t != ROUTE_ELEM_DIODE2_FORK_R_TURN
                    && t != ROUTE_ELEM_DIODE3_FORK_L_TURN && t != ROUTE_ELEM_DIODE3_FORK_R_TURN)
                {
                    base_ref += g_straight_boost_rpm;
                }
            }
        }
        float delta_ref = g_turn_delta_rpm;
        bool explicit_turn_active = line_follow_is_explicit_turn_active();
        bool override_delta = false;
        float left;
        float right;

        if (explicit_turn_active)
        {
            base_ref = 0.0f;
            g_loop_l.max_accel_rpm_ps = 80000.0f;
            g_loop_r.max_accel_rpm_ps = 80000.0f;
        }
        else if (startup_ramp_ticks < (uint16_t)(STARTUP_DELTA_RAMP_MS))
        {
            g_loop_l.max_accel_rpm_ps = 80000.0f;
            g_loop_r.max_accel_rpm_ps = 80000.0f;
        }
        else
        {
            g_loop_l.max_accel_rpm_ps = 3000.0f;
            g_loop_r.max_accel_rpm_ps = 3000.0f;
        }

        (void)move_distance_get_cmd(&base_ref, &delta_ref, &override_delta);
        (void)override_delta;

        // 指令直接发布，斜坡由 PI 层统一控制
        base_cmd_rpm = base_ref;
        delta_cmd_rpm = delta_ref;

        // 起步阶段 delta 从 0 线性爬坡，避免左右轮目标不同导致起步偏航
        {
            const uint16_t ramp_max = (uint16_t)(STARTUP_DELTA_RAMP_MS);
            if (startup_ramp_ticks < ramp_max)
            {
                startup_ramp_ticks++;
                float ramp_factor = (float)startup_ramp_ticks / (float)ramp_max;
                delta_cmd_rpm *= ramp_factor;
            }
        }

        left = base_cmd_rpm - delta_cmd_rpm;
        right = base_cmd_rpm + delta_cmd_rpm;

        // 转弯结束后两轮对称归零，避免一端正一转负导致接元件时加速不对称偏航
        {
            static uint16_t post_turn_sync_frames = 0u;
            static bool was_explicit_turn = false;

            if (was_explicit_turn && !explicit_turn_active)
            {
                post_turn_sync_frames = 50u;
            }
            was_explicit_turn = explicit_turn_active;

            if (post_turn_sync_frames > 0u)
            {
                left = 0.0f;
                right = 0.0f;
                g_loop_l.max_accel_rpm_ps = 80000.0f;
                g_loop_r.max_accel_rpm_ps = 80000.0f;
                post_turn_sync_frames--;
            }
        }

        g_target_left_rpm = left;
        g_target_right_rpm = right;
    }

    foc_runtime_cmd_publish_isr(1u, g_target_left_rpm, g_target_right_rpm);
    neg_pressure_motor_update(&g_neg_pressure_motor, dt, neg_pressure_run_request);
    wifi_service_capture_snapshot_isr();
    perf_monitor_record_isr1k(t0);
}
// **************************** PIT�жϺ��� ****************************


// **************************** �ⲿ�жϺ��� ****************************
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外部中断服务（EXTI CH0/CH4）
// 返回参数     void
// 使用示例     exti_ch0_ch4_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    if(exti_flag_get(ERU_CH4_REQ8_P33_7))           // ͨ��4�ж�
    {
        exti_flag_clear(ERU_CH4_REQ8_P33_7);
    }

    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // ͨ��0�ж�
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������




    }

}

// ��������ͷpclk����Ĭ��ռ���� 1ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外部中断服务（EXTI CH1/CH5）
// 返回参数     void
// 使用示例     exti_ch1_ch5_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // ͨ��1�ж�
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������




    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // ͨ��5�ж�
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������




    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外部中断服务（EXTI CH2/CH6）
// 返回参数     void
// 使用示例     exti_ch2_ch6_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // ͨ��2�ж�
    {
        exti_flag_clear(ERU_CH2_REQ7_P00_4);
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������



    }
    if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // ͨ��6�ж�
    {
        exti_flag_clear(ERU_CH6_REQ9_P20_0);
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷPCLK ����ʹ������ͷ �벻Ҫ��������������




    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外部中断服务（EXTI CH3/CH7）
// 返回参数     void
// 使用示例     exti_ch3_ch7_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

    if(exti_flag_get(ERU_CH3_REQ3_P10_3))           // ͨ��3�ж�
    {
        exti_flag_clear(ERU_CH3_REQ3_P10_3);
    }

    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // ͨ��7�ж�
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������
        // ע��: ���ⲿ�ж�Ϊ����ͷ���ж� ����ʹ������ͷ �벻Ҫ��������������



    }
}
// **************************** �ⲿ�жϺ��� ****************************


// **************************** DMA�жϺ��� ****************************
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA 通道 6 中断服务
// 返回参数     void
// 使用示例     dma_ch6_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(dma_ch6_isr, 0, DMA_INT_PRIO_1)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    clear_dma_flag(IfxDma_ChannelId_6);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA 通道 7 中断服务
// 返回参数     void
// 使用示例     dma_ch7_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(dma_ch7_isr, 0, DMA_INT_PRIO_2)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    clear_dma_flag(IfxDma_ChannelId_7);
}
// **************************** DMA�жϺ��� ****************************


// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART0 发送中断服务
// 返回参数     void
// 使用示例     uart0_tx_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART0 接收中断服务
// 返回参数     void
// 使用示例     uart0_rx_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

#if DEBUG_UART_USE_INTERRUPT                        // ������� debug �����ж�
        debug_interrupr_handler();                  // ���� debug ���ڽ��մ������� ���ݻᱻ debug ���λ�������ȡ
#endif                                              // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
}


// ����1Ĭ�����ӵ�����ͷ���ô���
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART1 发送中断服务
// 返回参数     void
// 使用示例     uart1_tx_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��




}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART1 接收中断服务
// 返回参数     void
// 使用示例     uart1_rx_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
}

// ����2Ĭ�����ӵ�����ת����ģ��
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART2 发送中断服务
// 返回参数     void
// 使用示例     uart2_tx_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 打开中断嵌套
    if (gpio_get_level(WIFI_UART_RTS_PIN) == 0) {
        IfxAsclin_Asc_isrTransmit(&uart2_handle);
    } else {
        uart_tx_interrupt(UART_2, 0);
        wifi_uart_tx_stalled = 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART2 接收中断服务
// 返回参数     void
// 使用示例     uart2_rx_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    wireless_module_uart_handler();                 // ����ģ��ͳһ�ص�����



}
// ����3Ĭ�����ӵ�GPS��λģ��
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART3 发送中断服务
// 返回参数     void
// 使用示例     uart3_tx_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART3 接收中断服务
// 返回参数     void
// 使用示例     uart3_rx_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

    gnss_uart_callback();                           // GNSS���ڻص�����



}

// ����ͨѶ�����ж�
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART0 错误中断服务
// 返回参数     void
// 使用示例     uart0_er_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART1 错误中断服务
// 返回参数     void
// 使用示例     uart1_er_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART2 错误中断服务
// 返回参数     void
// 使用示例     uart2_er_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART3 错误中断服务
// 返回参数     void
// 使用示例     uart3_er_isr();
// 备注信息     中断内部调用
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
#pragma section code restore
