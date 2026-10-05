#include "zf_common_headfile.h"

#include "menu.h"
#include "isr.h"
#include "zf_driver/zf_driver_pit.h"
#include "IfxGtm_Atom.h"
#include "zf_device/zf_device_ips200.h"
#include "zf_device/zf_device_gray_4067.h"
#include "zf_device/zf_device_imu660rb.h"
#include "zf_device/zf_device_as5047p.h"
#include "zf_device/zf_device_wifi_uart.h"

#include "key.h"
#include "wifi_cfg.h"
#include "wifi_service.h"

#include "battery_monitor.h"
#include "neg_pressure_motor.h"

#include "params_flash.h"
#include "pid.h"
#include "route_plan.h"
#include "line_follow.h"
#include "watchdog.h"
#include "perf_monitor.h"
#pragma section all "cpu0_dsram"

#ifndef WIFI_UART_TEST_STREAM_ENABLE
#define WIFI_UART_TEST_STREAM_ENABLE    (0)
#endif

// ============================ 全局实例 ============================
foc_svpwm_t              g_svpwm;
bldc_drv8300_3pwm_dual_t g_pwm;
neg_pressure_motor_t     g_neg_pressure_motor;
as5047p_device_t         g_enc_l;
as5047p_device_t         g_enc_r;
foc_speed_loop_t         g_loop_l;
foc_speed_loop_t         g_loop_r;

PID                      g_steer_pid;

imu_ahrs_t               g_ahrs;
volatile float           g_att_roll_deg = 0.0f;
volatile float           g_att_pitch_deg = 0.0f;
volatile float           g_att_yaw_deg = 0.0f;

volatile float           g_gyro_x_dps_f = 0.0f;
volatile float           g_gyro_y_dps_f = 0.0f;
volatile float           g_gyro_z_dps_f = 0.0f;
volatile float           g_gyro_lp_alpha = 0.1f;
volatile float           g_as5047p_lp_alpha = 0.4f;
volatile float           g_as5047p_speed_avg_samples = 1.0f;
volatile float           g_as5047p_speed_zero_deadband_rpm = 2.0f;

volatile uint8_t         g_run_enable = 0;
volatile system_stop_reason_t g_stop_reason = SYSTEM_STOP_REASON_NONE;
volatile uint8_t         g_neg_pressure_enable = 0;
volatile float           g_neg_pressure_target_percent = 0.0f;
volatile float           g_neg_pressure_output_pulse_us = 1000.0f;

volatile uint8_t  g_pwm_ready = 0;
uint32_t g_foc_pwm_period = PWM_DUTY_MAX;

static void init_fail_halt(const char *step)
{
    ips200_clear();
    ips200_show_string(0, 0,  "INIT FAIL");
    ips200_show_string(0, 16, (char *)step);

    g_run_enable = 0;
    if(g_pwm_ready)
    {
        bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
        bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
        neg_pressure_motor_stop(&g_neg_pressure_motor);
    }

    while(TRUE)
    {
        system_delay_ms(400);
    }
}

// 全局速度目标（RPM）
volatile float           g_base_speed_rpm = 200.0f;
volatile float           g_turn_delta_rpm = 0.0f;
volatile float           g_target_left_rpm = 0.0f;
volatile float           g_target_right_rpm = 0.0f;

volatile menu_state_t    g_menu_state = MENU_MAIN;
volatile uint8_t         g_headless_mode = 0;

static const route_plan_entry_t g_route_turns[] = { 
    { .turn = ROUTE_ELEM_PWR_SUPPLY_2},
    { .turn = ROUTE_TURN_STRAIGHT}, 
    { .turn = ROUTE_ELEM_DIODE_FORK_R_TURN},
    { .turn = ROUTE_ELEM_VOLTAGE_SRC},
    { .turn = ROUTE_ELEM_NOT_GATE},
    { .turn = ROUTE_ELEM_DIODE_FORK_R_TURN},
    { .turn = ROUTE_ELEM_LED},
    { .turn = ROUTE_ELEM_DIODE_FORK_L_TURN},
    { .turn = ROUTE_TURN_RIGHT},
    { .turn = ROUTE_TURN_STRAIGHT},
    { .turn = ROUTE_TURN_RIGHT},
    { .turn = ROUTE_ELEM_LAMP},
    { .turn = ROUTE_ELEM_CAPACITOR},
    { .turn = ROUTE_TURN_RIGHT},
    { .turn = ROUTE_ELEM_NOT_GATE},
    { .turn = ROUTE_TURN_STRAIGHT},
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     CPU0 主函数
// 返回参数     int             返回值（通常不返回/忽略）
// 使用示例     core0_main();
// 备注信息     入口函数
//-------------------------------------------------------------------------------------------------------------------


int core0_main(void)
{
    clock_init();
    watchdog_init();
    debug_init();

    // ---------------- 外设初始化 ----------------
    ips200_init(IPS200_TYPE_SPI);
    ips200_show_string(0,0,"init");

    keys_init();

    ips200_show_string(0, 16, "wifi auto...");
#if WIFI_BOOT_AUTO_CONNECT_ENABLE
    if (wifi_uart_boot_auto_connect(WIFI_SSID_TEST, WIFI_PASSWORD_TEST,
                                    WIFI_BOOT_CONNECT_TIMEOUT_MS, WIFI_BOOT_ACK_TIMEOUT_MS))
    {
        ips200_show_string(0, 32, "wifi ok");
    }
    else
    {
        ips200_show_string(0, 32, "wifi fail");
    }
#else
    ips200_show_string(0, 32, "wifi off");
#endif

    battery_monitor_init();


    gray_4067_init();
    gray_4067_enable(1);

    if(imu660rb_init())
    {
        init_fail_halt("IMU660RB");
    }
    imu_ahrs_init(&g_ahrs, 0.5f, 0.05f);

    // ---------------- 电机/编码器/FOC 初始化 ----------------
    as5047p_init(&g_enc_l,
                 AS5047P_DEFAULT_SPI,
                 AS5047P_DEFAULT_SCK_PIN,
                 AS5047P_DEFAULT_MOSI_PIN,
                 AS5047P_DEFAULT_MISO_PIN,
                 AS5047P_DEFAULT_CS1_PIN,
                 AS5047P_SPI_MODE,
                 AS5047P_SPI_BAUD);

    as5047p_init(&g_enc_r,
                 AS5047P_DEFAULT_SPI,
                 AS5047P_DEFAULT_SCK_PIN,
                 AS5047P_DEFAULT_MOSI_PIN,
                 AS5047P_DEFAULT_MISO_PIN,
                 AS5047P_DEFAULT_CS2_PIN,
                 AS5047P_SPI_MODE,
                 AS5047P_SPI_BAUD);

    {
        uint16_t angle = 0;
        if(AS5047P_OK != as5047p_read_angle_raw(&g_enc_l, &angle))
        {
            init_fail_halt("ENC_L AS5047P");
        }
        if(AS5047P_OK != as5047p_read_angle_raw(&g_enc_r, &angle))
        {
            init_fail_halt("ENC_R AS5047P");
        }
    }
    bldc_drv8300_3pwm_dual_init_default(&g_pwm);
    neg_pressure_motor_init(&g_neg_pressure_motor);
    g_pwm_ready = 1;
    foc_svpwm_init(&g_svpwm, 1.0f, PWM_DUTY_MAX);

    foc_speed_loop_init_default(&g_loop_l);
    foc_speed_loop_init_default(&g_loop_r);
    pid_init(&g_steer_pid, 10,0,0);
    line_follow_apply_config();

    g_run_enable = 0;
    bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
    neg_pressure_motor_stop(&g_neg_pressure_motor);
    system_delay_ms(50);

    foc_calib_cfg_t calib_cfg;
    foc_calibrate_motor_default_cfg(&calib_cfg);
    {
        uint8_t cal_result = foc_calibrate_both_motors(&g_svpwm,
            &g_loop_l, &g_loop_r, &g_enc_l, &g_enc_r,
            &g_pwm.left, &g_pwm.right, &calib_cfg);
        if(!(cal_result & 1u))
            init_fail_halt("FOC CAL L");
        if(!(cal_result & 2u))
            init_fail_halt("FOC CAL R");
    }
    system_delay_ms(50);

    g_run_enable = 0;
    g_target_left_rpm = 0.0f;
    g_target_right_rpm = 0.0f;
    bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
    neg_pressure_motor_stop(&g_neg_pressure_motor);
    neg_pressure_motor_stop(&g_neg_pressure_motor);


    params_load_from_flash();
    route_plan_set(g_route_turns, (uint8_t)(sizeof(g_route_turns) / sizeof(g_route_turns[0])));

    gpio_init(P02_6, GPO, GPIO_LOW, GPO_PUSH_PULL);

    g_foc_pwm_period = IfxGtm_Atom_Ch_getCompareZero(&MODULE_GTM.ATOM[0], 0);

    foc_svpwm_lut_init(&g_svpwm);

    pit_us_init(FOC_CTRL_PIT_INDEX, FOC_CTRL_PERIOD_US);
    pit_us_init(CCU61_CH1, 1000u);
    pit_us_init(CCU60_CH1, 2000);
    pit_us_init(CCU60_CH0, 5000);
    cpu_wait_event_ready();
    perf_monitor_init();

    // ======== 启动模式选择：2s 内 K4→菜单 / K1或超时→比赛 ========
    ips200_show_string(0, 0, "K4:Menu  K1:Race");
    system_delay_ms(300);

    {
        uint8_t startup_key = 0;
        int i;
        for (i = 0; i < 40; i++)
        {
            startup_key = get_key_value();
            if (startup_key != 0u) break;
            system_delay_ms(50);
        }
        if (startup_key == 4u)
        {
            g_headless_mode = 0;
            g_menu_state = MENU_MAIN;
        }
        else
        {
            g_headless_mode = 1;
            g_menu_state = MENU_RUN;
            motor_beep(100);
        }
    }
    ips200_clear();
    // =============================================================

    while (TRUE)
    {
        uint32_t idle_start = perf_monitor_get_tick();

        if (g_menu_state == MENU_MAIN)
        {
            menu_show_main();
        }
        else if (g_menu_state == MENU_RUN)
        {
            menu_run_task();
        }
        else if (g_menu_state == MENU_FOC_SPEED_DEBUG)
        {
            menu_foc_speed_debug_task();
        }
        else if (g_menu_state == MENU_LINE_FOLLOW_DEBUG)
        {
            menu_line_follow_debug_task();
        }
        else if (g_menu_state == MENU_JUNCTION_DEBUG)
        {
            menu_junction_debug_task();
        }
        else if (g_menu_state == MENU_GRAY_DEBUG)
        {
            menu_gray_debug_task();
        }
        else if (g_menu_state == MENU_IMU_DEBUG)
        {
            menu_imu_debug_task();
        }
        else if (g_menu_state == MENU_NEG_PRESSURE_DEBUG)
        {
            menu_neg_pressure_debug_task();
        }
        else if (g_menu_state == MENU_BLDC_OPEN_LOOP_TEST)
        {
            menu_bldc_open_loop_test_task();
        }
        else if (g_menu_state == MENU_WIFI_CONNECT)
        {
            menu_wifi_connect_task();
        }
        else
        {
            g_menu_state = MENU_MAIN;
        }

#if WIFI_UART_TEST_STREAM_ENABLE
        {
            static uint32 wifi_test_cnt = 0;
            char tx_buf[32];
            uint32 tx_len = (uint32)sprintf(tx_buf, "%lu\n", (unsigned long)wifi_test_cnt++);
            wifi_uart_send_buffer((const uint8 *)tx_buf, tx_len);
        }
#endif

        for (uint8_t _i = 0u; _i < 25u; _i++)
        {
            wifi_service_periodic();
            system_delay_ms(2);
        }
        watchdog_service_cpu0();
        perf_monitor_record_cpu0_idle(idle_start);
    }
}

#pragma section all restore
