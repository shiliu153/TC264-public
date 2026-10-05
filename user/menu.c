#include "zf_common_headfile.h"
#include "menu.h"
#include "isr.h"
#include "params_flash.h"
#include "line_follow.h"
#include "move_distance.h"
#include "neg_pressure_motor.h"
#include "wifi_service.h"
#include "battery_monitor.h"

#include "zf_device/zf_device_wifi_uart.h"
#include "zf_device/zf_device_gray_4067.h"
#include "zf_device/zf_device_imu660rb.h"

#include "imu_ahrs.h"
#include "zf_driver/zf_driver_timer.h"

#include "key.h"

#include "wifi_cfg.h"

static void menu_service_wait_ms(uint32_t wait_ms);
static uint8_t menu_wifi_ready(void);
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
    { MENU_WIFI_CONNECT, "WIFI" },
};

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

    return 0u;
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
        if (!ascii_equal_ignore_case(name[i], (uint8_t)keyword[i]))
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
        wifi_service_periodic();
        system_delay_ms(step_ms);
        remain_ms -= step_ms;
    }
}

static uint8_t menu_wifi_ready(void)
{
    const uint8_t *ip = wifi_uart_information.wifi_uart_local_ip;
    if ((NULL == ip) || (0 == ip[0]))
    {
        return 0u;
    }
    if ((ip[0] == '0') && (ip[1] == '.') && (ip[2] == '0') && (ip[3] == '.') && (ip[4] == '0') && (ip[5] == '.') && (ip[6] == '0'))
    {
        return 0u;
    }
    return 1u;
}

#define MAIN_MENU_ITEM_COUNT 9

static const char *menu_turn_to_str(route_turn_t t)
{
    switch (t)
    {
    case ROUTE_TURN_LEFT: return "L";
    case ROUTE_TURN_RIGHT: return "R";
    default: return "S";
    }
}

static const char *menu_lf_state_to_str(uint8_t s)
{
    switch (s)
    {
    case 0u: return "FOL";
    case 1u: return "PRE";
    case 2u: return "TL";
    case 3u: return "TR";
    default: return "?";
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示主菜单并处理菜单选择
// 返回参数     void
// 使用示例     menu_show_main();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void menu_show_main(void)
{
    int sel = 0;
    const int param_count = MAIN_MENU_ITEM_COUNT;
    int last_sel = -1;
    int last_bat_deciv = -1;
    int last_bat_percent = -1;

    const char *items[MAIN_MENU_ITEM_COUNT] = {
        "Run",
        "FOC Speed Debug",
        "Line Follow Debug",
        "Junction Debug",
        "Gray Debug",
        "IMU Debug",
        "NegPressure ESC",
        "BLDC OpenLoop Test",
        "WiFi Connect",
    };

    ips200_clear();
    ips200_show_string(0, 16 + MAIN_MENU_ITEM_COUNT * 16 + 8, "K2:Up K3:Down");
    ips200_show_string(0, 16 + MAIN_MENU_ITEM_COUNT * 16 + 24, "K1/K4:Enter");
    for (int i = 0; i < MAIN_MENU_ITEM_COUNT; i++)
    {
        char buf[32];
        sprintf(buf, "%c %-20s", (i == sel) ? '>' : ' ', items[i]);
        ips200_show_string(0, 16 + (uint16)(i * 16), buf);
    }
    last_sel = sel;

    while (g_menu_state == MENU_MAIN){
        float battery_v = battery_monitor_get_voltage_v();
        int bat_deciv = (int)(battery_v * 10.0f + 0.5f);
        int bat_percent = (int)battery_monitor_get_percent();

        if ((bat_deciv != last_bat_deciv) || (bat_percent != last_bat_percent))
        {
            char title_buf[32];
            sprintf(title_buf, "= Main %5.2fV %3u%%", battery_v, (unsigned int)bat_percent);
            ips200_show_string(0, 0, title_buf);
            last_bat_deciv = bat_deciv;
            last_bat_percent = bat_percent;
        }

        if (sel != last_sel)
        {
            char buf[32];
            if ((last_sel >= 0) && (last_sel < MAIN_MENU_ITEM_COUNT))
            {
                sprintf(buf, "  %-20s", items[last_sel]);
                ips200_show_string(0, 16 + (uint16)(last_sel * 16), buf);
            }
            sprintf(buf, "> %-20s", items[sel]);
            ips200_show_string(0, 16 + (uint16)(sel * 16), buf);
            last_sel = sel;
        }

        uint8_t key = get_key_value();
        if (key == 2u)
        {
            sel = (sel - 1 + param_count) % param_count;
        }
        else if (key == 3u)
        {
            sel = (sel + 1) % param_count;
        }
        else if (key == 1u || key == 4u)
        {
            switch (sel)
            {
            case 0:
                g_menu_state = MENU_RUN;
                break;
            case 1:
                g_menu_state = MENU_FOC_SPEED_DEBUG;
                break;
            case 2:
                g_menu_state = MENU_LINE_FOLLOW_DEBUG;
                break;
            case 3:
                g_menu_state = MENU_JUNCTION_DEBUG;
                break;
            case 4:
                g_menu_state = MENU_GRAY_DEBUG;
                break;
            case 5:
                g_menu_state = MENU_IMU_DEBUG;
                break;
            case 6:
                g_menu_state = MENU_NEG_PRESSURE_DEBUG;
                break;
            case 7:
                g_menu_state = MENU_WIFI_CONNECT;
                break;
            default:
                break;
            }
        }

        menu_service_wait_ms(80u);
    }
    ips200_clear();
}

void menu_wifi_connect_task(void)
{
    ips200_clear();

    while (g_menu_state == MENU_WIFI_CONNECT)
    {
        uint8_t aborted = 0u;
        uint8_t key;
        uint8_t connected = (uint8_t)(WIFI_UART_SERVER_ON == wifi_uart_information.wifi_uart_connect_state);
        uint8_t wifi_ready = menu_wifi_ready();

        ips200_show_string(0, 0, "--- WiFi Connect ---");
        ips200_show_string(0, 16,
            connected ? "State: TCP Connected" : (wifi_ready ? "State: WiFi Ready" : "State: Disconnected"));
        ips200_show_string(0, 32, WIFI_SSID_TEST);
        ips200_show_string(0, 48, wifi_ready ? "K1:Exit K4:TCP Link" : "K1:Exit K4:WiFi Init");
        ips200_show_string(0, 64, "K2:Abort when busy");

        key = get_key_value();
        if (key == 1u)
        {
            g_menu_state = MENU_MAIN;
            break;
        }

        if (key == 4u)
        {
            if (connected)
            {
                ips200_show_string(0, 80, "already connected   ");
                menu_service_wait_ms(400u);
            }
            else if (!wifi_ready)
            {
                ips200_show_string(0, 80, "wifi connecting... ");
                uint8_t wifi_retries = 0u;
                const uint8_t WIFI_MAX_RETRIES = 5u;
                while (g_menu_state == MENU_WIFI_CONNECT)
                {
                    uint8_t busy_key = get_key_value();
                    if ((busy_key == 2u) || (busy_key == 1u))
                    {
                        aborted = 1u;
                        ips200_show_string(0, 96, "wifi abort        ");
                        menu_service_wait_ms(300u);
                        break;
                    }

                    if (0u == wifi_uart_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST, WIFI_UART_STATION))
                    {
                        ips200_show_string(0, 96, "wifi connected    ");
                        ips200_show_string(0, 112, "press K4 for TCP  ");
                        menu_service_wait_ms(400u);
                        break;
                    }

                    if (++wifi_retries >= WIFI_MAX_RETRIES)
                    {
                        ips200_show_string(0, 96, "wifi max retries  ");
                        menu_service_wait_ms(400u);
                        break;
                    }

                    menu_service_wait_ms(500u);
                }

                if ((!aborted) && (!menu_wifi_ready()))
                {
                    ips200_show_string(0, 128, "wifi incomplete   ");
                }
            }
            else
            {
                if (1 != WIFI_UART_AUTO_CONNECT)
                {
                    ips200_show_string(0, 80, "tcp connecting...  ");
                    uint8_t tcp_retries = 0u;
                    const uint8_t TCP_MAX_RETRIES = 5u;
                    while (g_menu_state == MENU_WIFI_CONNECT)
                    {
                        uint8_t busy_key = get_key_value();
                        if ((busy_key == 2u) || (busy_key == 1u))
                        {
                            aborted = 1u;
                            ips200_show_string(0, 96, "tcp abort         ");
                            menu_service_wait_ms(300u);
                            break;
                        }

                        if (0u == wifi_uart_connect_tcp_servers(WIFI_UART_TARGET_IP, WIFI_UART_TARGET_PORT, WIFI_TCP_TRANSFER_MODE))
                        {
                            ips200_show_string(0, 96, "tcp connected     ");
                            break;
                        }

                        if (++tcp_retries >= TCP_MAX_RETRIES)
                        {
                            ips200_show_string(0, 96, "tcp max retries   ");
                            menu_service_wait_ms(400u);
                            break;
                        }

                        menu_service_wait_ms(500u);
                    }
                }

                if (aborted)
                {
                    ips200_show_string(0, 144, "connect aborted   ");
                }
                else if (WIFI_UART_SERVER_ON == wifi_uart_information.wifi_uart_connect_state)
                {
                    ips200_show_string(0, 144, "tcp ok            ");
                }
                else
                {
                    ips200_show_string(0, 144, "connect incomplete");
                }
            }
        }

        menu_service_wait_ms(80u);
    }

    if (g_menu_state == MENU_WIFI_CONNECT)
    {
        g_menu_state = MENU_MAIN;
    }
    ips200_clear();
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

static void menu_registry_adjust_value(const params_descriptor_t *desc, float dir)
{
    float value;

    if (desc == NULL)
    {
        return;
    }

    if (desc->type == PARAMS_VALUE_BOOL)
    {
        value = (params_get_value_as_float(desc) >= 0.5f) ? 0.0f : 1.0f;
    }
    else
    {
        value = params_get_value_as_float(desc) + dir * desc->step;
    }

    params_set_value_from_float(desc, value);
}

static void menu_draw_foc_speed_runtime_status(void)
{
    char buf[64];
    float vq_l_pu = g_loop_l.vq_out;
    float vq_r_pu = g_loop_r.vq_out;
    float vbus = g_vbus;
    float vq_l_v = vq_l_pu * vbus;
    float vq_r_v = vq_r_pu * vbus;

    snprintf(buf, sizeof(buf), "Vq PU L:%+0.2f R:%+0.2f", (double)vq_l_pu, (double)vq_r_pu);
    ips200_show_string(0, 160, buf);

    snprintf(buf, sizeof(buf), "Vq  V L:%+0.2f R:%+0.2f", (double)vq_l_v, (double)vq_r_v);
    ips200_show_string(0, 176, buf);
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

    snprintf(buf, sizeof(buf), "State:%s En:%u Act:%u",
             state_text,
             (unsigned)enable_now,
             (unsigned)(g_neg_pressure_motor.active ? 1u : 0u));
    ips200_show_string(0, 160, buf);

    snprintf(buf, sizeof(buf), "Thr:%3.0f%% Out:%4.0fus",
             (double)g_neg_pressure_target_percent,
             (double)g_neg_pressure_output_pulse_us);
    ips200_show_string(0, 176, buf);
}

typedef struct
{
    const char *label;
    void (*action)(void);
    void (*format)(char *buf, uint32_t buf_size);
} menu_extra_action_t;

static void menu_generic_edit_registry_task_ex(const char *title, menu_state_t page_state,
                                                void (*runtime_draw)(void),
                                                uint8_t items_per_page,
                                                const menu_extra_action_t *extra_actions,
                                                uint8_t extra_count)
{
    int sel = 0;
    int edit_mode = 0;
    uint8_t redraw = 1u;
    uint8_t refresh_tick = 0u;
    int param_count = (int)params_get_page_item_count(page_state);
    int total_items = param_count + (int)extra_count;
    const char *k4_hint = (extra_count > 0u) ? "K4:Edit/Act" : "K4:Edit";

    ips200_clear();

    while (g_menu_state == page_state)
    {
        if (redraw || (refresh_tick >= 12u))
        {
            ips200_show_string(0, 0, title);
            {
                char hint[32];
                snprintf(hint, sizeof(hint), "K1:Exit %s", k4_hint);
                ips200_show_string(0, 16, hint);
            }
            ips200_show_string(0, 32, "K2:+ K3:-  K4:Ed");

            int start_idx = (sel / (int)items_per_page) * (int)items_per_page;
            for (uint8_t i = 0u; i < items_per_page; i++)
            {
                int idx = (int)i + start_idx;
                char buf[48];
                uint16_t y = (uint16_t)(48 + (int)i * 16);

                if (idx < param_count)
                {
                    const params_descriptor_t *desc = params_get_page_item(page_state, (uint16_t)idx);
                    char mark = (idx == sel) ? (edit_mode ? '*' : '>') : ' ';
                    char val_buf[24];

                    menu_registry_format_value(desc, val_buf, (uint32_t)sizeof(val_buf));
                    snprintf(buf, sizeof(buf), "%c%-10s:%s", mark, desc->label, val_buf);
                }
                else if (idx < total_items)
                {
                    uint8_t act_idx = (uint8_t)(idx - param_count);
                    const menu_extra_action_t *act = &extra_actions[act_idx];
                    char mark = (idx == sel) ? '>' : ' ';
                    char val_buf[32];

                    if (act->format != NULL)
                    {
                        act->format(val_buf, (uint32_t)sizeof(val_buf));
                        snprintf(buf, sizeof(buf), "%c %-12s: %-10s", mark, act->label, val_buf);
                    }
                    else
                    {
                        snprintf(buf, sizeof(buf), "%c %-12s", mark, act->label);
                    }
                }
                else
                {
                    snprintf(buf, sizeof(buf), "                        ");
                }

                ips200_show_string(0, y, buf);
            }

            redraw = 0u;
            refresh_tick = 0u;
        }

        if (runtime_draw != NULL)
        {
            runtime_draw();
        }

        {
            uint8_t key = get_key_value();
            if (key == 1u) {
                params_save_to_flash();
                g_menu_state = MENU_MAIN;
                break;
            }
            else if (key == 4u)
            {
                if (sel >= param_count)
                {
                    uint8_t act_idx = (uint8_t)(sel - param_count);
                    if (act_idx < extra_count && extra_actions[act_idx].action != NULL)
                    {
                        extra_actions[act_idx].action();
                        redraw = 1u;
                    }
                }
                else if (param_count > 0)
                {
                    const params_descriptor_t *desc = params_get_page_item(page_state, (uint16_t)sel);
                    if ((desc != NULL) && (desc->type == PARAMS_VALUE_BOOL))
                    {
                        menu_registry_adjust_value(desc, 1.0f);
                    }
                    else
                    {
                        edit_mode = !edit_mode;
                    }
                    redraw = 1u;
                }
            }
            else if (edit_mode && (key == 2u || key == 3u) && sel < param_count)
            {
                const params_descriptor_t *desc = params_get_page_item(page_state, (uint16_t)sel);
                float dir = (key == 2u) ? 1.0f : -1.0f;
                menu_registry_adjust_value(desc, dir);
                redraw = 1u;
            }
            else if (!edit_mode && (key == 2u) && (total_items > 0)) { sel = (sel - 1 + total_items) % total_items; redraw = 1u; }
            else if (!edit_mode && (key == 3u) && (total_items > 0)) { sel = (sel + 1) % total_items; redraw = 1u; }
        }

        menu_service_wait_ms(80u);
        if (refresh_tick < 255u)
        {
            refresh_tick++;
        }
    }
}

static void menu_generic_edit_registry_task(const char *title, menu_state_t page_state, void (*runtime_draw)(void))
{
    menu_generic_edit_registry_task_ex(title, page_state, runtime_draw, 6u, NULL, 0u);
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

    if (!g_headless_mode) { ips200_clear(); }
    if (!g_headless_mode) { ips200_show_string(0, 0, "--- Run ---"); }

    g_run_enable = 0;
    foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);

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

        if (!g_headless_mode) { ips200_show_string(0, 16, "K1:Exit"); }
        if (!g_headless_mode) { ips200_show_string(0, 32, "K4:Start/Stop"); }
        sprintf(buf, "VBAT:%4.2fV %3u%%", battery_v, (unsigned int)battery_percent);
        if (!g_headless_mode) { ips200_show_string(0, 48, buf); }

        if (battery_critical)
        {
            if (!g_headless_mode) { ips200_show_string(0, 64, "BAT: CRITICAL    "); }
        }
        else if (battery_low)
        {
            if (!g_headless_mode) { ips200_show_string(0, 64, "BAT: LOW         "); }
        }
        else
        {
            if (!g_headless_mode) { ips200_show_string(0, 64, "BAT: NORMAL      "); }
        }

        sprintf(buf, "A12:%4.2fV R:%4u", battery_pin_v, (unsigned int)battery_raw);
        if (!g_headless_mode) { ips200_show_string(0, 96, buf); }
        sprintf(buf, "ODOM:%s", odom_active ? "ON " : "OFF");
        if (!g_headless_mode) { ips200_show_string(0, 112, buf); }
        sprintf(buf, "L:%5.0f R:%5.0f", (double)odom_left_mm, (double)odom_right_mm);
        if (!g_headless_mode) { ips200_show_string(0, 128, buf); }

        uint8_t key = get_key_value();
        if (key == 1u)
        {
            g_menu_state = MENU_MAIN;
            break;
        }
        else if (key == 4u)
        {
            running = !running;
            if (running)
            {
                if (battery_critical)
                {
                    g_stop_reason = SYSTEM_STOP_REASON_LOW_BATTERY;
                    running = false;
                    if (!g_headless_mode) { ips200_show_string(0, 80, "Low battery stop "); }
                    if (g_headless_mode) { int _i; for (_i=0;_i<5;_i++) { motor_beep(30); system_delay_ms(80); } }
                    menu_service_wait_ms(400u);
                    continue;
                }
                if (!line_follow_control_ready())
                {
                    running = false;
                    if (!g_headless_mode) { ips200_show_string(0, 80, "LF calib invalid "); }
                    menu_service_wait_ms(400u);
                    continue;
                }
                menu_service_wait_ms(1000u);
                g_stop_reason = SYSTEM_STOP_REASON_NONE;
                g_run_enable = 1;
                if (g_headless_mode) { motor_beep(100); }
            }
            else
            {
                g_stop_reason = SYSTEM_STOP_REASON_NONE;
                g_run_enable = 0;
                foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);
                bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
                bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
                neg_pressure_motor_stop(&g_neg_pressure_motor);
                if (g_headless_mode) { motor_beep(50); system_delay_ms(100); motor_beep(50); }
            }
        }

        if (running && (0u == g_run_enable))
        {
            running = false;
            switch (g_stop_reason)
            {
            case SYSTEM_STOP_REASON_LOST_LINE:
                if (!g_headless_mode) { ips200_show_string(0, 80, "STOP: LOST LINE  "); }
                break;
            case SYSTEM_STOP_REASON_ROUTE_DONE:
                if (!g_headless_mode) { ips200_show_string(0, 80, "STOP: ROUTE DONE "); }
                break;
            case SYSTEM_STOP_REASON_LOW_BATTERY:
                if (!g_headless_mode) { ips200_show_string(0, 80, "STOP: LOW BAT    "); }
                if (g_headless_mode) { int _i; for (_i=0;_i<3;_i++) { motor_beep(30); system_delay_ms(80); } }
                break;
            case SYSTEM_STOP_REASON_ERROR:
                if (!g_headless_mode) { ips200_show_string(0, 80, "STOP: ERROR      "); }
                if (g_headless_mode) { int _i; for (_i=0;_i<5;_i++) { motor_beep(30); system_delay_ms(80); } }
                break;
            case SYSTEM_STOP_REASON_ROLLOVER:
                if (!g_headless_mode) { ips200_show_string(0, 80, "STOP: ROLLOVER   "); }
                break;
            default:
                if (!g_headless_mode) { ips200_show_string(0, 80, "AUTO STOP        "); }
                break;
            }
            menu_service_wait_ms(500u);
            break;
        }

        if (running)
        {
            if (!g_headless_mode) { ips200_show_string(0, 80, "RUNNING          "); }
        }
        else
        {
            if (!g_headless_mode) { ips200_show_string(0, 80, "STOPPED          "); }
        }

        menu_service_wait_ms(50u);
    }

    g_run_enable = 0;
    foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
    neg_pressure_motor_stop(&g_neg_pressure_motor);

    params_save_to_flash();

    if (g_menu_state == MENU_RUN)
    {
        g_menu_state = MENU_MAIN;
    }
    if (!g_headless_mode) { ips200_clear(); }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器调试页面任务
// 返回参数     void
// 使用示例     menu_gray_debug_task();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void menu_gray_debug_task(void)
{
    ips200_clear();
    ips200_show_string(0, 0, "--- Gray Debug ---");
    ips200_show_string(0, 16, "K1: Exit");

    uint16_t v[GRAY_4067_DEFAULT_CHANNEL_COUNT];
    const uint16_t col_x[2] = {0u, 120u};
    const uint16_t row_y0 = 32u;
    const uint8_t rows_per_col = (uint8_t)((GRAY_4067_DEFAULT_CHANNEL_COUNT + 1u) / 2u);

    while (g_menu_state == MENU_GRAY_DEBUG)
    {
        gray_4067_read_12_raw(v);

        uint16_t min_v = v[0];
        uint16_t max_v = v[0];

        for (int i = 0; i < GRAY_4067_DEFAULT_CHANNEL_COUNT; i++)
        {
            char buf[24];
            uint16_t x = col_x[(uint16_t)(i / rows_per_col)];
            uint16_t y = (uint16_t)(row_y0 + (uint16_t)(i % rows_per_col) * 16u);

            if (v[i] < min_v)
            {
                min_v = v[i];
            }
            if (v[i] > max_v)
            {
                max_v = v[i];
            }

                sprintf(buf, "Y%02u:%4u",
                    (unsigned)(((uint8_t)GRAY_4067_CH12_START_CHANNEL + (uint8_t)i) & 0x0Fu),
                    (unsigned)v[i]);
            ips200_show_string(x, y, buf);
        }

        {
            char buf[24];
            sprintf(buf, "MIN:%4u MAX:%4u", (unsigned)min_v, (unsigned)max_v);
            ips200_show_string(0, 144, buf);
        }

        {
            char buf[24];
            uint8_t start_ch = (uint8_t)GRAY_4067_CH12_START_CHANNEL & 0x0Fu;
            uint8_t end_ch = (uint8_t)((GRAY_4067_CH12_START_CHANNEL + GRAY_4067_DEFAULT_CHANNEL_COUNT - 1u) & 0x0Fu);
            sprintf(buf, "Y%02u->Y%02u N:%u", (unsigned)start_ch, (unsigned)end_ch, (unsigned)GRAY_4067_DEFAULT_CHANNEL_COUNT);
            ips200_show_string(0, 160, buf);
        }

        uint8_t key = get_key_value();
        if (key == 1u)
        {
            g_menu_state = MENU_MAIN;
            break;
        }

        menu_service_wait_ms(100u);
    }

    if (g_menu_state == MENU_GRAY_DEBUG)
    {
        g_menu_state = MENU_MAIN;
    }
    ips200_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 循迹调试页面 — 额外操作与状态绘制回调
//-------------------------------------------------------------------------------------------------------------------
static void menu_lf_cal_action(void)
{
    g_run_enable = 0;
    foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);

    if (line_follow_calib_is_active())
    {
        (void)line_follow_calib_stop();
    }
    else
    {
        line_follow_calib_start();
    }
}

static void menu_lf_cal_format(char *buf, uint32_t buf_size)
{
    if (buf != NULL)
    {
        snprintf(buf, buf_size, "%s", line_follow_calib_is_active() ? "Stop" : "Start");
    }
}

static void menu_lf_clear_action(void)
{
    line_follow_calib_clear();
}

static void menu_lf_clear_format(char *buf, uint32_t buf_size)
{
    if (buf != NULL)
    {
        snprintf(buf, buf_size, "%s", line_follow_calib_is_valid() ? "OK" : "NA");
    }
}

static void menu_draw_line_follow_debug_status(void)
{
    char buf[48];
    float pos = 0.0f;
    bool pos_ok;
    line_follow_junction_debug_t dbg;

    pos_ok = line_follow_get_last_pos(&pos);
    line_follow_get_junction_debug(&dbg);

    snprintf(buf, sizeof(buf), "P%c%4.1f h%u s%u G%4.0f L%u d%4.0f",
        pos_ok ? ':' : '!',
        (double)pos,
        (unsigned)dbg.hit_count,
        (unsigned)dbg.lf_state,
        (double)g_gyro_z_dps_f,
        (unsigned)dbg.lost_line_cnt,
        (double)g_turn_delta_rpm);
    ips200_show_string(0, 112, buf);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     循迹调试页面任务
// 返回参数     void
// 使用示例     menu_line_follow_debug_task();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void menu_line_follow_debug_task(void)
{
    static const menu_extra_action_t actions[] = {
        { "Cal",   menu_lf_cal_action,   menu_lf_cal_format },
        { "Clear", menu_lf_clear_action, menu_lf_clear_format },
    };

    menu_generic_edit_registry_task_ex("--- Line Follow Debug ---", MENU_LINE_FOLLOW_DEBUG,
                                        menu_draw_line_follow_debug_status,
                                        4u, actions, 2u);

    params_save_to_flash();

    if (g_menu_state == MENU_LINE_FOLLOW_DEBUG)
    {
        g_menu_state = MENU_MAIN;
    }
    ips200_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 路口调试页面 — 状态绘制回调
//-------------------------------------------------------------------------------------------------------------------
static void menu_draw_junction_debug_status(void)
{
    char buf[48];
    line_follow_junction_debug_t d;
    line_follow_get_junction_debug(&d);

    float pre_delta_mm = fabsf(d.odom_mm - d.preturn_start_mm);

    snprintf(buf, sizeof(buf), "st:%s pend:%s idx:%u",
        menu_lf_state_to_str(d.lf_state),
        menu_turn_to_str(d.pending_turn),
        (unsigned)route_plan_get_index());
    ips200_show_string(0, 136, buf);

    snprintf(buf, sizeof(buf), "hit:%2u pos:%s%5.2f",
        (unsigned)d.hit_count,
        d.pos_valid ? "" : "NA",
        (double)d.pos);
    ips200_show_string(0, 152, buf);

    snprintf(buf, sizeof(buf), "bits:%04X L:%u R:%u",
        (unsigned)(d.bits),
        (unsigned)(d.left_edge ? 1u : 0u),
        (unsigned)(d.right_edge ? 1u : 0u));
    ips200_show_string(0, 168, buf);

    snprintf(buf, sizeof(buf), "N:%u W:%u BE:%u EB:%u",
        (unsigned)(d.near_center ? 1u : 0u),
        (unsigned)(d.wide_hit ? 1u : 0u),
        (unsigned)(d.both_edge ? 1u : 0u),
        (unsigned)(d.edge_branch ? 1u : 0u));
    ips200_show_string(0, 184, buf);

    snprintf(buf, sizeof(buf), "yaw:%5.1f t:%4.2f lost:%u",
        (double)d.yaw_delta_deg,
        (double)d.turn_time_s,
        (unsigned)d.lost_line_cnt);
    ips200_show_string(0, 200, buf);

    snprintf(buf, sizeof(buf), "is:%u lock:%4.2f preD:%5.1f",
        (unsigned)d.isect_cnt,
        (double)d.isect_lock_s,
        (double)pre_delta_mm);
    ips200_show_string(0, 216, buf);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     路口调试页面任务
// 返回参数     void
// 使用示例     menu_junction_debug_task();
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void menu_junction_debug_task(void)
{
    menu_generic_edit_registry_task_ex("--- Junction Debug ---", MENU_JUNCTION_DEBUG,
                                        menu_draw_junction_debug_status,
                                        5u, NULL, 0u);

    params_save_to_flash();

    if (g_menu_state == MENU_JUNCTION_DEBUG)
    {
        g_menu_state = MENU_MAIN;
    }
    ips200_clear();
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

