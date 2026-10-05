/*********************************************************************************************************************
* 文件：cpu0_main.c
* 模块：应用层（第 6 层）—— CPU0 主控
* 职责：CPU0 全部系统初始化，之后进入 10 状态菜单主循环；比赛路线配置
* 依赖：zf_common_headfile.h、本目录各模块头文件
* 风险等级：R3
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "zf_common_headfile.h"

#include "menu.h"
#include "isr.h"
#include "zf_driver/zf_driver_pit.h"
#include "IfxGtm_Atom.h"
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
#include "perf_monitor.h"
#include "calib_uart.h"
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
volatile float           g_gyro_lp_alpha = 1.0f;
volatile float           g_gyro_bias_x = 0.0f;
volatile float           g_gyro_bias_y = 0.0f;
volatile float           g_gyro_bias_z = 0.0f;
volatile float           g_as5047p_lp_alpha = 0.4f;
volatile float           g_as5047p_speed_avg_samples = 1.0f;
volatile float           g_as5047p_speed_zero_deadband_rpm = 1.0f;

volatile uint8_t         g_run_enable = 0;
volatile system_stop_reason_t g_stop_reason = SYSTEM_STOP_REASON_NONE;
volatile uint8_t         g_neg_pressure_enable = 0;
volatile float           g_neg_pressure_target_percent = 0.0f;
volatile float           g_neg_pressure_output_pulse_us = 1000.0f;

volatile uint8_t         g_paper_start_pending = 0;
volatile uint8_t         g_paper_start_neg_enable = 0;

static volatile uint8_t  g_pwm_ready = 0;
uint32_t g_foc_pwm_period = PWM_DUTY_MAX;

static void init_fail_halt(const char *step)
{
    (void)step;
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
volatile float           g_base_speed_rpm = 1000.0f;
volatile float           g_straight_boost_rpm = 200.0f;
volatile float           g_turn_delta_rpm = 0.0f;
volatile float           g_target_left_rpm = 0.0f;
volatile float           g_target_right_rpm = 0.0f;

volatile menu_state_t    g_menu_state = MENU_MAIN;

static const route_plan_entry_t g_route_turns[] = {
    { .turn = ROUTE_ELEM_LED},                     // 1  发光二极管
    { .turn = ROUTE_ELEM_CURRENT_SRC},              // 2  电流源
    { .turn = ROUTE_TURN_RIGHT},                    // 3  右转
    { .turn = ROUTE_ELEM_RESISTOR_1},               // 4  电阻1
    { .turn = ROUTE_TURN_RIGHT},                    // 5  右转
    { .turn = ROUTE_ELEM_NOT_GATE},                 // 6  非门
    { .turn = ROUTE_TURN_LEFT},                     // 7  左转
    { .turn = ROUTE_ELEM_TRANSISTOR_SIDE},          // 8  侧进三极管
    { .turn = ROUTE_ELEM_LED},                      // 9  发光二极管
    { .turn = ROUTE_ELEM_DIODE_FORK_L_TURN},        //10  二极管1型左转
    { .turn = ROUTE_TURN_LEFT},                     //11  左转
    { .turn = ROUTE_ELEM_INDUCTOR_2},               //12  电感2
    { .turn = ROUTE_TURN_RIGHT},                    //13  右转
    { .turn = ROUTE_TURN_RIGHT},                    //14  右转
    { .turn = ROUTE_ELEM_CAPACITOR},                //15  电容
    { .turn = ROUTE_TURN_LEFT},                     //16  左转
    { .turn = ROUTE_TURN_LEFT},                     //17  左转
    { .turn = ROUTE_ELEM_LED},                      //18  发光二极管
    { .turn = ROUTE_ELEM_CAPACITOR},                //19  电容
    { .turn = ROUTE_ELEM_DIODE2_FORK_L_TURN},        //20  二极管1型左转
    { .turn = ROUTE_TURN_LEFT},                     //21  左转
    { .turn = ROUTE_ELEM_LED},                      //22  发光二极管
    { .turn = ROUTE_TURN_STRAIGHT},                 //23  直行
    { .turn = ROUTE_ELEM_CAPACITOR},                //24  电容
    { .turn = ROUTE_TURN_LEFT},                     //25  左转
    { .turn = ROUTE_TURN_RIGHT},                    //26  右转
    { .turn = ROUTE_TURN_STRAIGHT},                 //27  直行
    { .turn = ROUTE_ELEM_INDUCTOR_1},               //28  电感1
    { .turn = ROUTE_ELEM_TRANSISTOR_SIDE},          //29  侧进三极管
    { .turn = ROUTE_TURN_RIGHT},                    //30  右转
    { .turn = ROUTE_TURN_STRAIGHT},                 //31  直行
    { .turn = ROUTE_ELEM_RESISTOR_1},               //32  电阻1
    { .turn = ROUTE_TURN_RIGHT},                    //33  右转
    { .turn = ROUTE_ELEM_TRANSISTOR_SIDE},          //34  侧进三极管
    { .turn = ROUTE_ELEM_CURRENT_SRC},              //35  电流源
    { .turn = ROUTE_TURN_RIGHT},                    //36  右转
    { .turn = ROUTE_ELEM_FIXED_DIST, .distance_mm = 150.0f, .distance_mode = MOVE_DISTANCE_MODE_LINE_FOLLOW}, //37  定距巡线200mm
    { .turn = ROUTE_ELEM_DIODE2_FORK_R_TURN},        //38  二极管1型右转
    { .turn = ROUTE_ELEM_RESISTOR_1},               //39  电阻1
    { .turn = ROUTE_TURN_STRAIGHT},                 //40  直行
    { .turn = ROUTE_TURN_RIGHT},                    //41  右转
    { .turn = ROUTE_ELEM_RESISTOR_1},               //42  电阻1
    { .turn = ROUTE_TURN_STRAIGHT},                 //43  直行
    { .turn = ROUTE_ELEM_RESISTOR_1},               //42  电阻1
    { .turn = ROUTE_TURN_STRAIGHT},                 //43  直行 
    { .turn = ROUTE_TURN_RIGHT},                    //44  右转
    { .turn = ROUTE_ELEM_TRANSISTOR_SIDE},          //45  侧进三极管
    { .turn = ROUTE_ELEM_CAPACITOR},                //46  电容
    { .turn = ROUTE_TURN_STRAIGHT},                 //47  直行
    { .turn = ROUTE_ELEM_RESISTOR_1},               //48  电阻1
    { .turn = ROUTE_TURN_STRAIGHT},                 //49  直行
    { .turn = ROUTE_ELEM_INDUCTOR_1},               //50  电感1
    { .turn = ROUTE_TURN_RIGHT},                    //51  右转
    { .turn = ROUTE_TURN_RIGHT},                    //52  右转
    { .turn = ROUTE_ELEM_LED},                      //53  发光二极管
    { .turn = ROUTE_TURN_STRAIGHT},                 //54  直行
    { .turn = ROUTE_ELEM_INDUCTOR_2},               //55  电感2
    { .turn = ROUTE_ELEM_NOT_GATE},                 //56  非门
    { .turn = ROUTE_TURN_RIGHT},                    //57  右转
    { .turn = ROUTE_ELEM_DIODE3_FORK_R_TURN},        //58  二极管1右转
    { .turn = ROUTE_ELEM_CAPACITOR},                //59  电容
    { .turn = ROUTE_TURN_STRAIGHT},                 //60  直行
    { .turn = ROUTE_ELEM_RESISTOR_1},               //61  电阻1
    { .turn = ROUTE_TURN_STRAIGHT},                 //62  直行
    { .turn = ROUTE_ELEM_INDUCTOR_1},               //63  电感1
    { .turn = ROUTE_TURN_STRAIGHT},                 //64  直行
    { .turn = ROUTE_TURN_STRAIGHT},                 //65  直行
    { .turn = ROUTE_TURN_RIGHT},                    //66  右转
    { .turn = ROUTE_ELEM_RESISTOR_1},               //67  电阻1
    { .turn = ROUTE_ELEM_LED},                      //68  发光二极管
    { .turn = ROUTE_ELEM_CURRENT_SRC}, 
    { .turn = ROUTE_TURN_RIGHT},
    { .turn = ROUTE_ELEM_RESISTOR_1},
    { .turn = ROUTE_TURN_STRAIGHT},
    { .turn = ROUTE_ELEM_TRANSISTOR_SIDE},
    { .turn = ROUTE_ELEM_CAPACITOR},
    { .turn = ROUTE_ELEM_LED},
    { .turn = ROUTE_TURN_STRAIGHT},



    // { .turn = ROUTE_ELEM_LED},                     // 1  发光二极管
    // { .turn = ROUTE_ELEM_CURRENT_SRC},              // 2  电流源
    // { .turn = ROUTE_TURN_RIGHT},                    // 3  右转
    // { .turn = ROUTE_ELEM_RESISTOR_1},               // 4  电阻1
    // { .turn = ROUTE_TURN_RIGHT},                    // 5  右转
    // { .turn = ROUTE_ELEM_NOT_GATE},                 // 6  非门
    // { .turn = ROUTE_TURN_STRAIGHT},
    // { .turn = ROUTE_TURN_RIGHT},
    // { .turn = ROUTE_TURN_RIGHT},
    // { .turn = ROUTE_ELEM_RESISTOR_1},
    // { .turn = ROUTE_ELEM_LED}, 
    // { .turn = ROUTE_ELEM_CURRENT_SRC}, 
    // { .turn = ROUTE_TURN_RIGHT},
    // { .turn = ROUTE_ELEM_RESISTOR_1},
    // { .turn = ROUTE_TURN_STRAIGHT},
    // { .turn = ROUTE_ELEM_TRANSISTOR_SIDE},
    // { .turn = ROUTE_ELEM_CAPACITOR},
    // { .turn = ROUTE_ELEM_LED},
    // { .turn = ROUTE_TURN_STRAIGHT},
    // { .turn = ROUTE_ELEM_RESISTOR_1},





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
    debug_init();
    uint8_t wifi_skip = 0;

    // ---------------- 外设初始化 ----------------
    keys_init();
#ifndef HARDWARE_STRIPPED
    neg_pressure_motor_init(&g_neg_pressure_motor);
#endif

    // ---------------- 启动模式选择：2s 内 K4 跳过 WiFi ----------------
    {
        uint8_t startup_key = 0;
        int i;
        for (i = 0; i < 40; i++)
        {
            startup_key = get_key_value();
            if (startup_key == 4u)
            {
                wifi_skip = 1;
                break;
            }
            system_delay_ms(50);
        }
#if WIFI_BOOT_AUTO_CONNECT_ENABLE
        if (!wifi_skip)
        {
            (void)wifi_uart_boot_auto_connect(WIFI_SSID_TEST, WIFI_PASSWORD_TEST,
                                            WIFI_BOOT_CONNECT_TIMEOUT_MS, WIFI_BOOT_ACK_TIMEOUT_MS);
        }
#else
        (void)wifi_skip;
#endif
    }

    battery_monitor_init();


    gray_4067_init();
    gray_4067_enable(1);

    if(imu660rb_init())
    {
        init_fail_halt("IMU660RB");
    }
    imu_ahrs_init(&g_ahrs, 0.5f, 0.05f);

    // ---------------- 电机/编码器/FOC 初始化 ----------------
#ifndef HARDWARE_STRIPPED
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
    g_pwm_ready = 1;
    foc_svpwm_init(&g_svpwm, 1.0f, PWM_DUTY_MAX);
    foc_svpwm_lut_init(&g_svpwm);

    foc_speed_loop_init_default(&g_loop_l);
    foc_speed_loop_init_default(&g_loop_r);
    pid_init(&g_steer_pid, 10,0,0);

    g_run_enable = 0;
    bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
    neg_pressure_motor_stop(&g_neg_pressure_motor);
    system_delay_ms(50);

    if (wifi_skip)
    {
        neg_pressure_motor_beep(&g_neg_pressure_motor, 200, 10.0f);
    }

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
#endif // HARDWARE_STRIPPED

    line_follow_apply_config();


    params_load_from_flash();
    route_plan_set(g_route_turns, (uint8_t)(sizeof(g_route_turns) / sizeof(g_route_turns[0])));

    gpio_init(P02_6, GPO, GPIO_LOW, GPO_PUSH_PULL);
    // buzzer_init();  // ATOM0_CH4 与电机 PWM 冲突,暂时禁用

#ifndef HARDWARE_STRIPPED
    g_foc_pwm_period = IfxGtm_Atom_Ch_getCompareZero(&MODULE_GTM.ATOM[0], 0);
#endif

    pit_us_init(FOC_CTRL_PIT_INDEX, FOC_CTRL_PERIOD_US);
    pit_us_init(CCU61_CH1, 1000u);
    pit_us_init(CCU60_CH1, 1000);
    pit_us_init(CCU60_CH0, 5000);
    cpu_wait_event_ready();
    perf_monitor_init();
    while (TRUE)
    {
        if (g_menu_state == MENU_MAIN)
        {
            menu_show_main();
        }
        else if (g_menu_state == MENU_RUN)
        {
            menu_run_task();
        }
#ifndef HARDWARE_STRIPPED
        else if (g_menu_state == MENU_FOC_SPEED_DEBUG)
        {
            menu_foc_speed_debug_task();
        }
#endif
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
#ifndef HARDWARE_STRIPPED
        else if (g_menu_state == MENU_NEG_PRESSURE_DEBUG)
        {
            menu_neg_pressure_debug_task();
        }
        else if (g_menu_state == MENU_BLDC_OPEN_LOOP_TEST)
        {
            menu_bldc_open_loop_test_task();
        }
#endif
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
            calib_uart_poll();
            wifi_service_periodic();
            system_delay_ms(2);
            perf_monitor_mark_delay_end();
        }
    }
}

#pragma section all restore
