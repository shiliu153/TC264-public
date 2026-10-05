/*********************************************************************************************************************
* 文件：menu.c
* 模块：应用层（第 6 层）—— 显示菜单
* 职责：显示菜单系统：10 个调试/信息页的渲染与按键处理
* 依赖：zf_common_headfile.h、本目录 menu.h
* 风险等级：R1
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "menu.h"
#include "isr.h"
#include "params_flash.h"
#include "line_follow.h"
#include "calib_uart.h"
#include "move_distance.h"
#include "neg_pressure_motor.h"
#include "wifi_service.h"
#include "battery_monitor.h"
#include "perf_monitor.h"

#include "zf_device/zf_device_wifi_uart.h"
#include "zf_device/zf_device_gray_4067.h"
#include "zf_device/zf_device_imu660rb.h"

#include "imu_ahrs.h"
#include "zf_driver/zf_driver_timer.h"

#include "key.h"

#include "wifi_cfg.h"

static void menu_service_wait_ms(uint32_t wait_ms);
static uint8_t menu_wifi_ready(void);
static uint8_t menu_ascii_equal_ignore_case(uint8_t lhs, uint8_t rhs);

static uint8_t popcount_u16(uint16_t x)
{
    uint8_t count = 0;
    while (x)
    {
        count++;
        x &= x - 1;
    }
    return count;
}
static uint8_t menu_page_name_matches(const uint8_t *name, uint32_t name_len, const char *keyword);
static void menu_draw_neg_pressure_runtime_status(void);

typedef struct
{
    menu_state_t state;
    const char *page_name;
} menu_page_map_t;

static const menu_page_map_t g_menu_page_maps[] = {
    { MENU_MAIN, "MAIN" },
    { MENU_RUN, "RUN" },
    { MENU_FOC_SPEED_DEBUG, "FOC" },
    { MENU_LINE_FOLLOW_DEBUG, "LINE" },
    { MENU_JUNCTION_DEBUG, "JUNCTION" },
    { MENU_GRAY_DEBUG, "GRAY" },
    { MENU_IMU_DEBUG, "IMU" },
    { MENU_NEG_PRESSURE_DEBUG, "NEG" },
    { MENU_BLDC_OPEN_LOOP_TEST, "OPENLOOP" },
    { MENU_WIFI_CONNECT, "WIFI" },
};

volatile float g_open_loop_hz = 0.001f;
volatile float g_open_loop_vq = 0.001f;
volatile uint8_t g_open_loop_run = 0;
volatile uint8_t g_foc_test_run = 0u;
volatile float g_foc_test_left_target_rpm = 0.0f;
volatile float g_foc_test_right_target_rpm = 0.0f;

const char *menu_get_page_name(menu_state_t state)
{
    uint32_t i;

    for (i = 0u; i < (uint32_t)(sizeof(g_menu_page_maps) / sizeof(g_menu_page_maps[0])); ++i)
    {
        if (g_menu_page_maps[i].state == state)
        {
            return g_menu_page_maps[i].page_name;
        }
    }

    return "MAIN";
}

uint8_t menu_get_state_from_page_name(const uint8_t *page_name, uint32_t page_name_len, menu_state_t *out_state)
{
    uint32_t begin = 0u;
    uint32_t end = page_name_len;
    uint32_t i;

    if ((NULL == page_name) || (NULL == out_state) || (0u == page_name_len))
    {
        return 0u;
    }

    while ((begin < page_name_len)
        && ((page_name[begin] == (uint8_t)' ') || (page_name[begin] == (uint8_t)'\t')))
    {
        begin++;
    }

    while ((end > begin)
        && ((page_name[end - 1u] == (uint8_t)' ') || (page_name[end - 1u] == (uint8_t)'\t')))
    {
        end--;
    }

    if (end <= begin)
    {
        return 0u;
    }

    page_name += begin;
    page_name_len = end - begin;

    for (i = 0u; i < (uint32_t)(sizeof(g_menu_page_maps) / sizeof(g_menu_page_maps[0])); ++i)
    {
        if (menu_page_name_matches(page_name, page_name_len, g_menu_page_maps[i].page_name))
        {
            *out_state = g_menu_page_maps[i].state;
            return 1u;
        }
    }

    if (menu_page_name_matches(page_name, page_name_len, "FOC_TEST")
        || menu_page_name_matches(page_name, page_name_len, "FOC_SPEED_DEBUG"))
    {
        *out_state = MENU_FOC_SPEED_DEBUG;
        return 1u;
    }

    if (menu_page_name_matches(page_name, page_name_len, "LINE_FOLLOW")
        || menu_page_name_matches(page_name, page_name_len, "LINE_FOLLOW_DEBUG"))
    {
        *out_state = MENU_LINE_FOLLOW_DEBUG;
        return 1u;
    }

    if (menu_page_name_matches(page_name, page_name_len, "NEG_PRESSURE")
        || menu_page_name_matches(page_name, page_name_len, "NEG_PRESSURE_DEBUG"))
    {
        *out_state = MENU_NEG_PRESSURE_DEBUG;
        return 1u;
    }

    if (menu_page_name_matches(page_name, page_name_len, "BLDC_OPEN_LOOP")
        || menu_page_name_matches(page_name, page_name_len, "OPEN_LOOP"))
    {
        *out_state = MENU_BLDC_OPEN_LOOP_TEST;
        return 1u;
    }

    return 0u;
}

static uint8_t menu_ascii_equal_ignore_case(uint8_t lhs, uint8_t rhs)
{
    if ((lhs >= (uint8_t)'a') && (lhs <= (uint8_t)'z'))
    {
        lhs = (uint8_t)(lhs - (uint8_t)('a' - 'A'));
    }
    if ((rhs >= (uint8_t)'a') && (rhs <= (uint8_t)'z'))
    {
        rhs = (uint8_t)(rhs - (uint8_t)('a' - 'A'));
    }
    return (uint8_t)(lhs == rhs);
}

static uint8_t menu_page_name_matches(const uint8_t *name, uint32_t name_len, const char *keyword)
{
    uint32_t keyword_len = 0u;
    uint32_t i;

    if ((NULL == name) || (NULL == keyword))
    {
        return 0u;
    }

    while ('\0' != keyword[keyword_len])
    {
        keyword_len++;
    }

    if (name_len != keyword_len)
    {
        return 0u;
    }

    for (i = 0u; i < name_len; ++i)
    {
        if (!menu_ascii_equal_ignore_case(name[i], (uint8_t)keyword[i]))
        {
            return 0u;
        }
    }

    return 1u;
}

static void menu_service_wait_ms(uint32_t wait_ms)
{
    const uint32_t slice_ms = 5u;
    uint32_t remain_ms = wait_ms;

    while (remain_ms > 0u)
    {
        uint32_t step_ms = (remain_ms > slice_ms) ? slice_ms : remain_ms;
        calib_uart_poll();
        wifi_service_periodic();
        system_delay_ms(step_ms);
        remain_ms -= step_ms;
    }
    perf_monitor_mark_delay_end();
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示主菜单并处理菜单选择
// 返回参数     void
// 使用示例     menu_show_main();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void menu_show_main(void)
{
    int last_bat_deciv = -1;
    int last_bat_percent = -1;

    while (g_menu_state == MENU_MAIN)
    {
        float battery_v = battery_monitor_get_voltage_v();
        int bat_deciv = (int)(battery_v * 10.0f + 0.5f);
        int bat_percent = (int)battery_monitor_get_percent();

        if ((bat_deciv != last_bat_deciv) || (bat_percent != last_bat_percent))
        {
            last_bat_deciv = bat_deciv;
            last_bat_percent = bat_percent;
        }

        uint8_t key = get_key_value();
        if (key == 1u)
        {
#ifndef HARDWARE_STRIPPED
            neg_pressure_motor_beep(&g_neg_pressure_motor, 200, 10.0f);
#endif
            g_menu_state = MENU_RUN;
            break;
        }

        menu_service_wait_ms(80u);
    }
}

void menu_wifi_connect_task(void)
{
    while (g_menu_state == MENU_WIFI_CONNECT)
    {
        uint8_t key = get_key_value();
        if (key == 1u)
        {
#ifndef HARDWARE_STRIPPED
            neg_pressure_motor_beep(&g_neg_pressure_motor, 200, 10.0f);
#endif
            g_menu_state = MENU_RUN;
            break;
        }

        menu_service_wait_ms(80u);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     FOC 速度环调试菜单任务
// 返回参数     void
// 使用示例     menu_foc_speed_debug_task();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------

// --------------------------------------------------------
// 通用注册参数菜单引擎
// --------------------------------------------------------

static void menu_registry_format_value(const params_descriptor_t *desc, char *buf, uint32_t buf_len)
{
    char fmt[8];

    if ((desc == NULL) || (buf == NULL) || (buf_len == 0u))
    {
        return;
    }

    switch (desc->type)
    {
    case PARAMS_VALUE_BOOL:
        snprintf(buf, buf_len, "%s", (params_get_value_as_float(desc) >= 0.5f) ? "ON" : "OFF");
        break;
    case PARAMS_VALUE_U8:
        snprintf(buf, buf_len, "%u", (unsigned)(uint8_t)params_get_value_as_float(desc));
        break;
    case PARAMS_VALUE_FLOAT:
    default:
        snprintf(fmt, sizeof(fmt), "%%.%uf", (unsigned)desc->decimals);
        snprintf(buf, buf_len, fmt, (double)params_get_value_as_float(desc));
        break;
    }
}


static void menu_draw_foc_speed_runtime_status(void)
{
}

static void menu_draw_neg_pressure_runtime_status(void)
{
    char buf[64];
    const char *state_text = "IDLE";
    uint8_t enable_now = (uint8_t)((g_run_enable != 0u)
                                || ((g_menu_state == MENU_NEG_PRESSURE_DEBUG) && (g_neg_pressure_enable != 0u)));

    switch (g_neg_pressure_motor.state)
    {
    case NEG_PRESSURE_MOTOR_STATE_ARMING:
        state_text = "ARM";
        break;
    case NEG_PRESSURE_MOTOR_STATE_RUNNING:
        state_text = "RUN";
        break;
    default:
        state_text = "IDLE";
        break;
    }

    }

static void menu_generic_edit_registry_task(const char *title, menu_state_t page_state, void (*runtime_draw)(void))
{
    int sel = 0;
    uint8_t redraw = 1u;
    uint8_t refresh_tick = 0u;
    int param_count = (int)params_get_page_item_count(page_state);

    while (g_menu_state == page_state)
    {
        if (redraw || (refresh_tick >= 12u))
        {
            int start_idx = (sel / 6) * 6;
            for (int i = 0; i < 6; i++)
            {
                int idx = i + start_idx;
                char buf[64];

                if (idx < param_count)
                {
                    const params_descriptor_t *desc = params_get_page_item(page_state, (uint16_t)idx);
                    char mark = (idx == sel) ? '>' : ' ';
                    char val_buf[32];

                    menu_registry_format_value(desc, val_buf, (uint32_t)sizeof(val_buf));
                    snprintf(buf, sizeof(buf), "%c %-12s: %-10s", mark, desc->label, val_buf);
                }
                else
                {
                    snprintf(buf, sizeof(buf), "                        ");
                }
            }

            redraw = 0u;
            refresh_tick = 0u;
        }

        if (runtime_draw != NULL)
        {
            runtime_draw();
        }

        uint8_t key = get_key_value();
        if (key == 1u)
        {
#ifndef HARDWARE_STRIPPED
            neg_pressure_motor_beep(&g_neg_pressure_motor, 200, 10.0f);
#endif
            g_menu_state = MENU_RUN;
            break;
        }
        
        menu_service_wait_ms(80u);
        if (refresh_tick < 255u)
        {
            refresh_tick++;
        }
    }
}

void menu_foc_speed_debug_task(void)
{
    menu_generic_edit_registry_task("- FOC Speed Debug -", MENU_FOC_SPEED_DEBUG, menu_draw_foc_speed_runtime_status);

    g_foc_test_run = 0u;
    g_run_enable = 0u;
    g_stop_reason = SYSTEM_STOP_REASON_NONE;
    foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Run 页面任务
// 返回参数     void
// 使用示例     menu_run_task();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void menu_run_task(void)
{
    bool running = false;

    g_run_enable = 0;
#ifndef HARDWARE_STRIPPED
    foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);
#endif

    while (g_menu_state == MENU_RUN)
    {
        char buf[32];
        float battery_v = battery_monitor_get_voltage_v();
        float battery_pin_v = battery_monitor_get_pin_voltage_v();
        uint16_t battery_raw = battery_monitor_get_raw();
        uint8_t battery_percent = battery_monitor_get_percent();
        uint8_t battery_low = battery_monitor_is_low();
        uint8_t battery_critical = battery_monitor_is_critical();
        float odom_left_mm = move_distance_counter_get_left_mm();
        float odom_right_mm = move_distance_counter_get_right_mm();
        uint8_t odom_active = (uint8_t)(move_distance_counter_is_active() ? 1u : 0u);

        (void)battery_v;
        (void)battery_pin_v;
        (void)battery_raw;
        (void)battery_percent;
        (void)battery_low;
        (void)odom_left_mm;
        (void)odom_right_mm;
        (void)odom_active;

        if (g_paper_start_pending)
        {
            uint32_t countdown_t0 = STM0_TIM0.U;

            uint16_t bits = line_follow_get_last_bits();
            uint8_t hit_count = popcount_u16(bits);

            if (hit_count == 16u)
            {
                (void)wifi_uart_send_buffer((const uint8_t *)"EVENT:WAIT_PAPER\n",
                    (uint32_t)(sizeof("EVENT:WAIT_PAPER\n") - 1u));
                uint8_t confirm_cnt = 0u;
                while (confirm_cnt < 5u)
                {
                    menu_service_wait_ms(10u);
                    if (!g_paper_start_pending)
                    {
                        break;
                    }
                    bits = line_follow_get_last_bits();
                    hit_count = popcount_u16(bits);
                    if (hit_count <= 7u)
                    {
                        confirm_cnt++;
                    }
                    else
                    {
                        confirm_cnt = 0u;
                    }
                }

                if (!g_paper_start_pending)
                {
                    continue;
                }

                (void)wifi_uart_send_buffer((const uint8_t *)"EVENT:PAPER_GONE\n",
                    (uint32_t)(sizeof("EVENT:PAPER_GONE\n") - 1u));
            }
            else
            {
                (void)wifi_uart_send_buffer((const uint8_t *)"EVENT:NO_PAPER_SKIP\n",
                    (uint32_t)(sizeof("EVENT:NO_PAPER_SKIP\n") - 1u));
            }

            g_paper_start_neg_enable = 1u;

            {
                uint32_t t_elapsed = (STM0_TIM0.U - countdown_t0) / 100000u;
                char ev_buf[40];
                int ev_len = sprintf(ev_buf, "EVENT:GO_4s,%lums\n", (unsigned long)t_elapsed);
                (void)wifi_uart_send_buffer((const uint8_t *)ev_buf, (uint32_t)ev_len);
            }
            g_cd_ticks = 1000u;
            while (g_cd_ticks > 0u) { wifi_service_periodic(); system_delay_ms(5); }

            if (!g_paper_start_pending)
            {
                g_paper_start_neg_enable = 0u;
                continue;
            }

            {
                uint32_t t_elapsed = (STM0_TIM0.U - countdown_t0) / 100000u;
                char ev_buf[40];
                int ev_len = sprintf(ev_buf, "EVENT:GO_3s,%lums\n", (unsigned long)t_elapsed);
                (void)wifi_uart_send_buffer((const uint8_t *)ev_buf, (uint32_t)ev_len);
            }
            g_cd_ticks = 1000u;
            while (g_cd_ticks > 0u) { wifi_service_periodic(); system_delay_ms(5); }

            if (!g_paper_start_pending)
            {
                g_paper_start_neg_enable = 0u;
                continue;
            }

            {
                uint32_t t_elapsed = (STM0_TIM0.U - countdown_t0) / 100000u;
                char ev_buf[40];
                int ev_len = sprintf(ev_buf, "EVENT:GO_2s,%lums\n", (unsigned long)t_elapsed);
                (void)wifi_uart_send_buffer((const uint8_t *)ev_buf, (uint32_t)ev_len);
            }
            g_cd_ticks = 1000u;
            while (g_cd_ticks > 0u) { wifi_service_periodic(); system_delay_ms(5); }

            if (!g_paper_start_pending)
            {
                g_paper_start_neg_enable = 0u;
                continue;
            }

            {
                uint32_t t_elapsed = (STM0_TIM0.U - countdown_t0) / 100000u;
                char ev_buf[40];
                int ev_len = sprintf(ev_buf, "EVENT:GO_1s,%lums\n", (unsigned long)t_elapsed);
                (void)wifi_uart_send_buffer((const uint8_t *)ev_buf, (uint32_t)ev_len);
            }
            g_cd_ticks = 1000u;
            while (g_cd_ticks > 0u) { wifi_service_periodic(); system_delay_ms(5); }

            {
                uint16_t motor_timeout = 0u;
                while ((g_neg_pressure_motor.state != NEG_PRESSURE_MOTOR_STATE_RUNNING)
                       || (g_neg_pressure_motor.current_pulse_us < (g_neg_pressure_motor.target_pulse_us - 20.0f)))
                {
                    if (!g_paper_start_pending)
                    {
                        break;
                    }
                    menu_service_wait_ms(10u);
                    motor_timeout++;
                    if (motor_timeout >= 300u)
                    {
                        break;
                    }
                }
                }

            g_paper_start_pending = 0u;
            g_run_enable = 1u;
            g_paper_start_neg_enable = 0u;

            {
                uint32_t countdown_ticks = STM0_TIM0.U - countdown_t0;
                uint32_t countdown_ms = countdown_ticks / 100000u;
                char go_buf[40];
                int go_len = sprintf(go_buf, "EVENT:GO,%lums\n", (unsigned long)countdown_ms);
                (void)wifi_uart_send_buffer((const uint8_t *)go_buf, (uint32_t)go_len);
            }

            {
                uint16_t release_timeout = 0u;
                while (get_key_value() == 1u && release_timeout < 300u)
                {
                    menu_service_wait_ms(10u);
                    release_timeout++;
                }
            }
        }

        uint8_t key = get_key_value();
        if (key == 1u)
        {
            if (running)
            {
                g_stop_reason = SYSTEM_STOP_REASON_NONE;
                g_run_enable = 0;
                g_paper_start_pending = 0u;
                g_paper_start_neg_enable = 0u;
#ifndef HARDWARE_STRIPPED
                foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);
                bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
                bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
                neg_pressure_motor_stop(&g_neg_pressure_motor);
#endif
                running = false;
            }
            else
            {
                if (battery_critical)
                {
                    g_stop_reason = SYSTEM_STOP_REASON_LOW_BATTERY;
                    menu_service_wait_ms(400u);
                    continue;
                }
                if (!line_follow_control_ready())
                {
                    menu_service_wait_ms(400u);
                    continue;
                }
                if (g_paper_start_pending == 0u)
                {
                    g_stop_reason = SYSTEM_STOP_REASON_NONE;
                    wifi_service_prepare_main_run_start();
                    g_paper_start_pending = 1u;
                    g_paper_start_neg_enable = 1u;
#ifndef HARDWARE_STRIPPED
                    g_neg_pressure_target_percent = NEG_PRESSURE_MOTOR_AUTO_MAX_PERCENT;
#endif
                    running = true;
                }
            }
        }

        if (running && (0u == g_run_enable) && (0u == g_paper_start_pending))
        {
            running = false;
            switch (g_stop_reason)
            {
            case SYSTEM_STOP_REASON_LOST_LINE:
                break;
            case SYSTEM_STOP_REASON_ROUTE_DONE:
                menu_service_wait_ms(1500u);
                break;
            case SYSTEM_STOP_REASON_LOW_BATTERY:
                break;
            case SYSTEM_STOP_REASON_ERROR:
                break;
            case SYSTEM_STOP_REASON_ROLLOVER:
                break;
            default:
                break;
            }
            menu_service_wait_ms(500u);
            break;
        }

        menu_service_wait_ms(10u);
    }

    g_paper_start_pending = 0u;
    g_paper_start_neg_enable = 0u;
    g_run_enable = 0;
#ifndef HARDWARE_STRIPPED
    foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
    neg_pressure_motor_stop(&g_neg_pressure_motor);
#endif

    params_save_to_flash();

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器调试页面任务
// 返回参数     void
// 使用示例     menu_gray_debug_task();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void menu_gray_debug_task(void)
{
    uint16_t v[GRAY_4067_DEFAULT_CHANNEL_COUNT];

    while (g_menu_state == MENU_GRAY_DEBUG)
    {
        gray_4067_read_12_raw(v);

        uint8_t key = get_key_value();
        if (key == 1u)
        {
#ifndef HARDWARE_STRIPPED
            neg_pressure_motor_beep(&g_neg_pressure_motor, 200, 10.0f);
#endif
            g_menu_state = MENU_RUN;
            break;
        }

        menu_service_wait_ms(100u);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     循迹调试页面任务
// 返回参数     void
// 使用示例     menu_line_follow_debug_task();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void menu_line_follow_debug_task(void)
{
    int selected_param_index = 0;
    bool edit_mode = false;
    const int page_param_count = (int)params_get_page_item_count(MENU_LINE_FOLLOW_DEBUG);
    const int param_count = page_param_count + 2;

    while (g_menu_state == MENU_LINE_FOLLOW_DEBUG)
    {

        uint8_t key = get_key_value();
        if (key == 1u)
        {
#ifndef HARDWARE_STRIPPED
            neg_pressure_motor_beep(&g_neg_pressure_motor, 200, 10.0f);
#endif
            g_menu_state = MENU_RUN;
            break;
        }

        menu_service_wait_ms(50u);
    }

    params_save_to_flash();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     路口调试页面任务
// 返回参数     void
// 使用示例     menu_junction_debug_task();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void menu_junction_debug_task(void)
{
    int selected_param_index = 0;
    bool edit_mode = false;
    const int param_count = (int)params_get_page_item_count(MENU_JUNCTION_DEBUG);
    const int display_count = 5;

    while (g_menu_state == MENU_JUNCTION_DEBUG)
    {

        uint8_t key = get_key_value();
        if (key == 1u)
        {
#ifndef HARDWARE_STRIPPED
            neg_pressure_motor_beep(&g_neg_pressure_motor, 200, 10.0f);
#endif
            g_menu_state = MENU_RUN;
            break;
        }

        menu_service_wait_ms(50u);
    }

    params_save_to_flash();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU 调试页面任务
// 返回参数     void
// 使用示例     menu_imu_debug_task();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------

void menu_imu_debug_task(void)
{
    menu_generic_edit_registry_task("- IMU Debug -", MENU_IMU_DEBUG, NULL);
}

void menu_neg_pressure_debug_task(void)
{
    menu_generic_edit_registry_task("- NegPressure ESC -", MENU_NEG_PRESSURE_DEBUG, menu_draw_neg_pressure_runtime_status);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷电机开环测试菜单
// 返回参数     void
// 使用示例     menu_bldc_open_loop_test_task();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void menu_bldc_open_loop_test_task(void)
{
    g_open_loop_run = 0u;
    menu_generic_edit_registry_task("- BLDC OpenLoop -", MENU_BLDC_OPEN_LOOP_TEST, NULL);
    g_open_loop_run = 0u;
}
