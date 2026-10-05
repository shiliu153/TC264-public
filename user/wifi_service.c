#include "zf_common_headfile.h"

#include "wifi_service.h"

#include "menu.h"
#include "perf_monitor.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "isr.h"

#include "battery_monitor.h"

#include "line_follow.h"
#include "move_distance.h"
#include "neg_pressure_motor.h"

#include "zf_device/zf_device_wifi_uart.h"
#include "zf_device/zf_device_gray_4067.h"
#include "params_flash.h"

extern volatile float g_gyro_lp_alpha;
extern volatile float g_as5047p_lp_alpha;
extern volatile float g_as5047p_speed_avg_samples;
extern volatile float g_as5047p_speed_zero_deadband_rpm;

#ifndef WIFI_UART_TELEMETRY_ENABLE
#define WIFI_UART_TELEMETRY_ENABLE        (1)
#endif

#ifndef WIFI_UART_TELEMETRY_FAST_PERIOD_RUN_MS
#define WIFI_UART_TELEMETRY_FAST_PERIOD_RUN_MS   (16u)
#endif

#ifndef WIFI_UART_TELEMETRY_FAST_PERIOD_IDLE_MS
#define WIFI_UART_TELEMETRY_FAST_PERIOD_IDLE_MS  (50u)
#endif

#ifndef WIFI_UART_TELEMETRY_ROUTE_PERIOD_RUN_MS
#define WIFI_UART_TELEMETRY_ROUTE_PERIOD_RUN_MS  (50u)
#endif

#ifndef WIFI_UART_TELEMETRY_ROUTE_PERIOD_IDLE_MS
#define WIFI_UART_TELEMETRY_ROUTE_PERIOD_IDLE_MS (100u)
#endif

#ifndef WIFI_UART_TELEMETRY_POWER_PERIOD_MS
#define WIFI_UART_TELEMETRY_POWER_PERIOD_MS      (100u)
#endif

#ifndef WIFI_UART_TELEMETRY_GRAY_PERIOD_MS
#define WIFI_UART_TELEMETRY_GRAY_PERIOD_MS       (50u)
#endif

#ifndef WIFI_UART_CMD_LINE_MAX
#define WIFI_UART_CMD_LINE_MAX                   (96u)
#endif

#ifndef WIFI_UART_RX_STREAM_MAX
#define WIFI_UART_RX_STREAM_MAX                  (512u)
#endif

#ifndef WIFI_UART_TELEMETRY_LINE_MAX
#define WIFI_UART_TELEMETRY_LINE_MAX             (96u)
#endif

#ifndef WIFI_UART_TELEMETRY_MAGIC0
#define WIFI_UART_TELEMETRY_MAGIC0               (0xA5u)
#endif

#ifndef WIFI_UART_TELEMETRY_MAGIC1
#define WIFI_UART_TELEMETRY_MAGIC1               (0x5Au)
#endif

#ifndef WIFI_UART_TELEMETRY_TYPE_FAST
#define WIFI_UART_TELEMETRY_TYPE_FAST            (0x01u)
#endif

#ifndef WIFI_UART_TELEMETRY_TYPE_ROUTE
#define WIFI_UART_TELEMETRY_TYPE_ROUTE           (0x02u)
#endif

#ifndef WIFI_UART_TELEMETRY_TYPE_POWER
#define WIFI_UART_TELEMETRY_TYPE_POWER           (0x03u)
#endif

#ifndef WIFI_UART_TELEMETRY_TYPE_GRAY
#define WIFI_UART_TELEMETRY_TYPE_GRAY            (0x04u)
#endif

#ifndef WIFI_UART_TELEMETRY_TYPE_GRAY_RAW
#define WIFI_UART_TELEMETRY_TYPE_GRAY_RAW        (0x05u)
#endif

#ifndef WIFI_UART_TELEMETRY_TYPE_PERF
#define WIFI_UART_TELEMETRY_TYPE_PERF            (0x06u)
#endif

#ifndef WIFI_UART_TELEMETRY_PERF_PERIOD_MS
#define WIFI_UART_TELEMETRY_PERF_PERIOD_MS       (200u)
#endif

#ifndef WIFI_UART_CONTROL_TYPE_CMD
#define WIFI_UART_CONTROL_TYPE_CMD               (0x10u)
#endif

#ifndef WIFI_UART_CONTROL_TYPE_ACK
#define WIFI_UART_CONTROL_TYPE_ACK               (0x11u)
#endif

#ifndef WIFI_UART_CONTROL_OPCODE_START
#define WIFI_UART_CONTROL_OPCODE_START           (0x01u)
#endif

#ifndef WIFI_UART_CONTROL_OPCODE_PAUSE
#define WIFI_UART_CONTROL_OPCODE_PAUSE           (0x02u)
#endif

#ifndef WIFI_UART_CONTROL_OPCODE_ODOM_START
#define WIFI_UART_CONTROL_OPCODE_ODOM_START      (0x03u)
#endif

#ifndef WIFI_UART_CONTROL_OPCODE_ODOM_CLEAR
#define WIFI_UART_CONTROL_OPCODE_ODOM_CLEAR      (0x04u)
#endif

#ifndef WIFI_UART_CONTROL_OPCODE_NPON
#define WIFI_UART_CONTROL_OPCODE_NPON            (0x05u)
#endif

#ifndef WIFI_UART_CONTROL_OPCODE_NPOFF
#define WIFI_UART_CONTROL_OPCODE_NPOFF           (0x06u)
#endif

#ifndef WIFI_UART_CONTROL_STATUS_OK
#define WIFI_UART_CONTROL_STATUS_OK              (0x00u)
#endif

#ifndef WIFI_UART_CONTROL_STATUS_BUSY
#define WIFI_UART_CONTROL_STATUS_BUSY            (0x01u)
#endif

#ifndef WIFI_UART_CONTROL_STATUS_INVALID_ARG
#define WIFI_UART_CONTROL_STATUS_INVALID_ARG     (0x02u)
#endif

#ifndef WIFI_UART_CONTROL_STATUS_CALIB_NOT_READY
#define WIFI_UART_CONTROL_STATUS_CALIB_NOT_READY (0x03u)
#endif

#ifndef WIFI_UART_CONTROL_STATUS_FOC_TEST_RUNNING
#define WIFI_UART_CONTROL_STATUS_FOC_TEST_RUNNING (0x04u)
#endif

#ifndef WIFI_UART_CONTROL_STATUS_UNKNOWN_CMD
#define WIFI_UART_CONTROL_STATUS_UNKNOWN_CMD     (0x05u)
#endif

#ifndef WIFI_UART_TELEMETRY_ANGLE_SCALE
#define WIFI_UART_TELEMETRY_ANGLE_SCALE          (100.0f)
#endif

#ifndef WIFI_UART_TELEMETRY_GYRO_SCALE
#define WIFI_UART_TELEMETRY_GYRO_SCALE           (10.0f)
#endif

#ifndef WIFI_UART_TELEMETRY_SPEED_SCALE
#define WIFI_UART_TELEMETRY_SPEED_SCALE          (10.0f)
#endif

#ifndef WIFI_UART_TELEMETRY_VQ_SCALE
#define WIFI_UART_TELEMETRY_VQ_SCALE             (1000.0f)
#endif

#ifndef WIFI_UART_TELEMETRY_VOLT_SCALE
#define WIFI_UART_TELEMETRY_VOLT_SCALE           (100.0f)
#endif

#ifndef WIFI_UART_TELEMETRY_NP_PERCENT_SCALE
#define WIFI_UART_TELEMETRY_NP_PERCENT_SCALE     (10.0f)
#endif

#ifndef WIFI_UART_TELEMETRY_NP_PULSE_US_SCALE
#define WIFI_UART_TELEMETRY_NP_PULSE_US_SCALE    (1.0f)
#endif

#ifndef WIFI_UART_TELEMETRY_GRAY_SHIFT
#define WIFI_UART_TELEMETRY_GRAY_SHIFT           (4u)
#endif

#ifndef WIFI_UART_TELEMETRY_ODOM_SCALE
#define WIFI_UART_TELEMETRY_ODOM_SCALE           (1.0f)
#endif

static float g_wifi_att_zero_roll_deg = 0.0f;
static float g_wifi_att_zero_pitch_deg = 0.0f;
static float g_wifi_att_zero_yaw_deg = 0.0f;
static volatile wifi_service_telemetry_snapshot_t g_wifi_snapshot_buffers[2];
static volatile uint8_t g_wifi_snapshot_ready_index = 0u;
static volatile uint8_t g_wifi_snapshot_ready = 0u;
static volatile uint8_t g_wifi_gray_stream_enable = 0u;
static volatile uint8_t g_wifi_gray_raw_enable = 0u;
static uint8_t g_wifi_rx_stream[WIFI_UART_RX_STREAM_MAX];
static uint32_t g_wifi_rx_stream_len = 0u;
static uint8_t g_wifi_last_ok_cmd_seq = 0u;
static uint8_t g_wifi_last_ok_cmd_opcode = 0u;
static uint8_t g_wifi_last_ok_cmd_valid = 0u;
static uint16_t g_wifi_cmd_monotonic = 0u;
static uint16_t g_wifi_last_ok_cmd_monotonic = 0u;

static float wifi_service_wrap_deg(float deg);
static void wifi_service_poll_command(void);
static void wifi_service_handle_command(const uint8_t *cmd, uint32_t len);
static void wifi_service_handle_binary_frame(const uint8_t *frame, uint32_t frame_len);
static uint8_t wifi_service_process_rx_stream(void);
static uint8_t wifi_service_cmd_equals(const uint8_t *cmd, uint32_t len, const char *keyword);
static uint8_t wifi_service_cmd_starts_with(const uint8_t *cmd, uint32_t len, const char *keyword);
static uint8_t wifi_service_ascii_equal_ignore_case(uint8_t lhs, uint8_t rhs);
static void wifi_service_force_pause(void);
static uint8_t wifi_service_try_start(void);
static void wifi_service_prepare_main_run_start(void);
static uint8_t wifi_service_neg_pressure_run_requested(void);
static void wifi_service_try_set_np_enable(uint8_t enable);
static void wifi_service_try_set_gray_stream(uint8_t enable);
static void wifi_service_send_page_changed_event(void);
static void wifi_service_send_page_params(menu_state_t page_state);
static void wifi_service_send_param_line_float(const char *name, float value);
static void wifi_service_send_param_line_u8(const char *name, uint8_t value);
static void wifi_service_send_param_line(const params_descriptor_t *desc);
static uint8_t wifi_service_try_handle_registered_param(const uint8_t *cmd, uint32_t len);
static void wifi_service_send_telemetry_periodic(void);
static void wifi_service_send_frame(uint8_t type, uint8_t flags, const uint8_t *payload, uint32_t payload_len);
static void wifi_service_send_control_ack(uint8_t seq, uint8_t opcode, uint8_t status);
static void wifi_service_send_fast_frame(uint8_t seq, const wifi_service_telemetry_snapshot_t *snapshot);
static void wifi_service_send_route_frame(uint8_t seq, const wifi_service_telemetry_snapshot_t *snapshot);
static void wifi_service_send_power_frame(uint8_t seq, const wifi_service_telemetry_snapshot_t *snapshot);
static void wifi_service_send_gray_frame(uint8_t seq, const wifi_service_telemetry_snapshot_t *snapshot);
static void wifi_service_send_perf_frame(uint8_t seq);
static uint8_t wifi_service_execute_control_opcode(uint8_t opcode, const uint8_t *args, uint32_t arg_len);
static uint32_t wifi_service_frame_append_u8(uint8_t *buf, uint32_t pos, uint32_t max_len, uint8_t value);
static uint32_t wifi_service_frame_append_u16_le(uint8_t *buf, uint32_t pos, uint32_t max_len, uint16_t value);
static uint32_t wifi_service_frame_append_s16_scaled(uint8_t *buf, uint32_t pos, uint32_t max_len, float value, float scale);
static sint16 wifi_service_float_to_s16_scaled(float value, float scale);
static uint8_t wifi_service_frame_checksum(const uint8_t *buf, uint32_t start, uint32_t len);
static void wifi_service_copy_snapshot(wifi_service_telemetry_snapshot_t *dst, volatile const wifi_service_telemetry_snapshot_t *src);
static uint8_t wifi_service_load_latest_snapshot(wifi_service_telemetry_snapshot_t *out);

void wifi_service_periodic(void)
{
    battery_monitor_periodic();
    wifi_service_poll_command();
    wifi_service_send_page_changed_event();
    wifi_service_send_telemetry_periodic();
    wifi_service_poll_command();
}

void wifi_service_capture_snapshot_isr(void)
{
#if WIFI_UART_TELEMETRY_ENABLE
    uint8_t write_index = (uint8_t)(g_wifi_snapshot_ready_index ^ 1u);
    volatile wifi_service_telemetry_snapshot_t *snapshot = &g_wifi_snapshot_buffers[write_index];
    uint8_t np_requested = wifi_service_neg_pressure_run_requested();
    line_follow_junction_debug_t line_dbg;
    float line_pos = 0.0f;
    bool line_pos_valid = line_follow_get_last_pos(&line_pos);

    line_follow_get_junction_debug(&line_dbg);

    snapshot->roll_deg = (float)g_att_roll_deg;
    snapshot->pitch_deg = (float)g_att_pitch_deg;
    snapshot->yaw_deg = (float)g_att_yaw_deg;
    snapshot->gyro_z_dps = (float)g_gyro_z_dps_f;
    snapshot->left_target_rpm = (float)g_target_left_rpm;
    snapshot->right_target_rpm = (float)g_target_right_rpm;
    snapshot->left_actual_rpm = g_motor_left_actual_rpm;
    snapshot->right_actual_rpm = g_motor_right_actual_rpm;
    snapshot->left_elec_angle_deg = g_motor_elec_l_deg;
    snapshot->right_elec_angle_deg = g_motor_elec_r_deg;
    snapshot->battery_voltage_v = battery_monitor_get_voltage_v();
    snapshot->battery_raw = battery_monitor_get_raw();
    snapshot->battery_percent = battery_monitor_get_percent();
    if (g_run_enable)
    {
        snapshot->battery_flags = 0u;
    }
    else
    {
        snapshot->battery_flags = (uint8_t)((battery_monitor_is_low() ? 0x01u : 0u)
                                          | (battery_monitor_is_critical() ? 0x02u : 0u));
    }
    snapshot->np_target_percent = g_neg_pressure_target_percent;
    snapshot->np_output_pulse_us = g_neg_pressure_output_pulse_us;
    snapshot->np_flags = (uint8_t)((np_requested ? 0x01u : 0u)
                                 | (g_neg_pressure_motor.active ? 0x02u : 0u)
                                 | ((g_neg_pressure_motor.state == NEG_PRESSURE_MOTOR_STATE_ARMING) ? 0x04u : 0u)
                                 | ((g_neg_pressure_motor.state == NEG_PRESSURE_MOTOR_STATE_RUNNING) ? 0x08u : 0u));
    snapshot->line_pos = line_pos;
    snapshot->turn_delta_rpm = (float)g_turn_delta_rpm;
    snapshot->line_bits = line_dbg.bits;
    snapshot->run_enable = (uint8_t)(g_run_enable ? 1u : 0u);
    snapshot->line_pos_valid = (uint8_t)(line_pos_valid ? 1u : 0u);
    snapshot->line_follow_ready = (uint8_t)(line_follow_control_ready() ? 1u : 0u);
    snapshot->lf_state = line_dbg.lf_state;
    snapshot->pending_turn = (uint8_t)line_dbg.pending_turn;
    snapshot->route_index = route_plan_get_index();
    snapshot->lost_line_cnt = line_dbg.lost_line_cnt;
    snapshot->stop_reason = (uint8_t)g_stop_reason;
    snapshot->junction_lock_cnt = line_follow_get_junction_lock_cnt();
    snapshot->element_enter_cnt = line_follow_get_element_enter_cnt();
    snapshot->junction_enter_cnt = line_follow_get_junction_enter_cnt();
    snapshot->lf_actual_hz = g_lf_actual_hz;
    snapshot->calib_active = (uint8_t)line_follow_calib_is_active();
    snapshot->calib_samples_hi = (uint8_t)(line_follow_calib_get_samples() >> 8);
    snapshot->calib_valid = (uint8_t)line_follow_calib_is_valid();
    snapshot->turn_yaw_deg = line_dbg.yaw_delta_deg;
    snapshot->odom_left_mm = move_distance_counter_get_left_mm();
    snapshot->odom_right_mm = move_distance_counter_get_right_mm();
    snapshot->odom_count_active = (uint8_t)(move_distance_counter_is_active() ? 1u : 0u);
    snapshot->left_vq_out = g_loop_l.vq_out;
    snapshot->right_vq_out = g_loop_r.vq_out;

    if (g_wifi_gray_stream_enable != 0u)
    {
        uint16_t gray[GRAY_4067_DEFAULT_CHANNEL_COUNT] = {0};
        line_follow_get_last_raw(gray, (uint8_t)GRAY_4067_DEFAULT_CHANNEL_COUNT);
        for (uint8_t i = 0u; i < (uint8_t)GRAY_4067_DEFAULT_CHANNEL_COUNT; ++i)
        {
            snapshot->gray[i] = gray[i];
        }
        snapshot->gray_valid = 1u;
    }
    else
    {
        snapshot->gray_valid = 0u;
    }

    g_wifi_snapshot_ready_index = write_index;
    g_wifi_snapshot_ready = 1u;
#endif
}

void wifi_service_get_att_zero(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (NULL != roll_deg)
    {
        *roll_deg = g_wifi_att_zero_roll_deg;
    }
    if (NULL != pitch_deg)
    {
        *pitch_deg = g_wifi_att_zero_pitch_deg;
    }
    if (NULL != yaw_deg)
    {
        *yaw_deg = g_wifi_att_zero_yaw_deg;
    }
}

void wifi_service_set_att_zero(float roll_deg, float pitch_deg, float yaw_deg)
{
    g_wifi_att_zero_roll_deg = roll_deg;
    g_wifi_att_zero_pitch_deg = pitch_deg;
    g_wifi_att_zero_yaw_deg = yaw_deg;
}

static void wifi_service_send_telemetry_periodic(void)
{
#if WIFI_UART_TELEMETRY_ENABLE
    static uint32_t last_fast_ms = 0u;
    static uint32_t last_route_ms = 0u;
    static uint32_t last_power_ms = 0u;
    static uint32_t last_gray_ms = 0u;
    static uint8_t fast_seq = 0u;
    static uint8_t route_seq = 0u;
    static uint8_t power_seq = 0u;
    static uint8_t gray_seq = 0u;
    static float last_roll_deg = 0.0f;
    static float last_pitch_deg = 0.0f;
    static float last_yaw_deg = 0.0f;
    static uint8_t next_slot = 0u;
    wifi_service_telemetry_snapshot_t snapshot;
    uint32_t now_ms;
    uint32_t fast_period_ms;
    uint32_t route_period_ms;
    uint8_t sent;

    if (WIFI_UART_SERVER_ON != wifi_uart_information.wifi_uart_connect_state)
    {
        return;
    }

    now_ms = (uint32_t)system_getval_ms();
    sent = 0u;

    switch (next_slot)
    {
    case 0u:
        if (!wifi_service_load_latest_snapshot(&snapshot))
        {
            break;
        }

        fast_period_ms = snapshot.run_enable ? (uint32_t)WIFI_UART_TELEMETRY_FAST_PERIOD_RUN_MS
                                             : (uint32_t)WIFI_UART_TELEMETRY_FAST_PERIOD_IDLE_MS;

        if (isfinite(snapshot.roll_deg))
        {
            last_roll_deg = snapshot.roll_deg;
        }
        if (isfinite(snapshot.pitch_deg))
        {
            last_pitch_deg = snapshot.pitch_deg;
        }
        if (isfinite(snapshot.yaw_deg))
        {
            last_yaw_deg = snapshot.yaw_deg;
        }

        snapshot.roll_deg = last_roll_deg;
        snapshot.pitch_deg = last_pitch_deg;
        snapshot.yaw_deg = last_yaw_deg;

        if ((uint32_t)(now_ms - last_fast_ms) >= fast_period_ms)
        {
            last_fast_ms = now_ms;
            wifi_service_send_fast_frame(fast_seq++, &snapshot);
            sent = 1u;
        }
        break;

    case 1u:
        if (!wifi_service_load_latest_snapshot(&snapshot))
        {
            break;
        }

        route_period_ms = snapshot.run_enable ? (uint32_t)WIFI_UART_TELEMETRY_ROUTE_PERIOD_RUN_MS
                                              : (uint32_t)WIFI_UART_TELEMETRY_ROUTE_PERIOD_IDLE_MS;

        if (isfinite(snapshot.roll_deg))
        {
            last_roll_deg = snapshot.roll_deg;
        }
        if (isfinite(snapshot.pitch_deg))
        {
            last_pitch_deg = snapshot.pitch_deg;
        }
        if (isfinite(snapshot.yaw_deg))
        {
            last_yaw_deg = snapshot.yaw_deg;
        }

        snapshot.roll_deg = last_roll_deg;
        snapshot.pitch_deg = last_pitch_deg;
        snapshot.yaw_deg = last_yaw_deg;

        if ((uint32_t)(now_ms - last_route_ms) >= route_period_ms)
        {
            last_route_ms = now_ms;
            wifi_service_send_route_frame(route_seq++, &snapshot);
            sent = 1u;
        }
        break;

    case 2u:
        if (!wifi_service_load_latest_snapshot(&snapshot))
        {
            break;
        }

        if ((uint32_t)(now_ms - last_power_ms) >= (uint32_t)WIFI_UART_TELEMETRY_POWER_PERIOD_MS)
        {
            last_power_ms = now_ms;
            wifi_service_send_power_frame(power_seq++, &snapshot);
            sent = 1u;
        }
        break;

    case 3u:
        if ((g_wifi_gray_stream_enable != 0u)
            && wifi_service_load_latest_snapshot(&snapshot)
            && snapshot.gray_valid
            && ((uint32_t)(now_ms - last_gray_ms) >= (uint32_t)WIFI_UART_TELEMETRY_GRAY_PERIOD_MS))
        {
            last_gray_ms = now_ms;
            wifi_service_send_gray_frame(gray_seq++, &snapshot);
            sent = 1u;
        }
        break;

    case 4u:
        {
            static uint32_t last_perf_ms = 0u;
            static uint8_t perf_seq = 0u;
            if ((uint32_t)(now_ms - last_perf_ms) >= (uint32_t)WIFI_UART_TELEMETRY_PERF_PERIOD_MS)
            {
                last_perf_ms = now_ms;
                wifi_service_send_perf_frame(perf_seq++);
                sent = 1u;
            }
        }
        break;

    default:
        break;
    }

    (void)sent;
    {
        uint8_t ns = next_slot + 1u;
        next_slot = (ns >= 5u) ? 0u : ns;
    }
#endif
}

static void wifi_service_send_perf_frame(uint8_t seq)
{
    uint8_t buf[24];
    uint32_t pos = 0u;
    perf_monitor_output_t out;
    perf_monitor_output(&out);

    pos = wifi_service_frame_append_u8(buf, pos, sizeof(buf), WIFI_UART_TELEMETRY_TYPE_PERF);
    buf[pos++] = seq;
    pos = wifi_service_frame_append_u16_le(buf, pos, sizeof(buf), out.cpu0_load_x100);
    pos = wifi_service_frame_append_u16_le(buf, pos, sizeof(buf), out.cpu1_load_x100);
    pos = wifi_service_frame_append_u16_le(buf, pos, sizeof(buf), out.foc_avg_us);
    pos = wifi_service_frame_append_u16_le(buf, pos, sizeof(buf), out.isr1k_avg_us);
    buf[pos++] = out.foc_load_pct;
    pos = wifi_service_frame_append_u16_le(buf, pos, sizeof(buf), out.window_ms);
    pos = wifi_service_frame_append_u16_le(buf, pos, sizeof(buf), 0u);
    pos = wifi_service_frame_append_u16_le(buf, pos, sizeof(buf), 0u);

    wifi_service_send_frame(WIFI_UART_TELEMETRY_TYPE_PERF, 0u, buf, pos);
    (void)seq;
}

static void wifi_service_send_frame(uint8_t type, uint8_t flags, const uint8_t *payload, uint32_t payload_len)
{
    uint8_t frame[WIFI_UART_TELEMETRY_LINE_MAX];
    uint32_t len = 0u;

    if ((payload == NULL) || (payload_len > (uint32_t)(sizeof(frame) - 6u)))
    {
        return;
    }

    len = wifi_service_frame_append_u8(frame, len, (uint32_t)sizeof(frame), (uint8_t)WIFI_UART_TELEMETRY_MAGIC0);
    len = wifi_service_frame_append_u8(frame, len, (uint32_t)sizeof(frame), (uint8_t)WIFI_UART_TELEMETRY_MAGIC1);
    len = wifi_service_frame_append_u8(frame, len, (uint32_t)sizeof(frame), type);
    len = wifi_service_frame_append_u8(frame, len, (uint32_t)sizeof(frame), flags);
    len = wifi_service_frame_append_u8(frame, len, (uint32_t)sizeof(frame), (uint8_t)payload_len);
    for (uint32_t i = 0u; i < payload_len; ++i)
    {
        len = wifi_service_frame_append_u8(frame, len, (uint32_t)sizeof(frame), payload[i]);
    }
    len = wifi_service_frame_append_u8(frame, len, (uint32_t)sizeof(frame), wifi_service_frame_checksum(frame, 2u, len - 2u));
    (void)wifi_uart_send_buffer(frame, len);
}

static void wifi_service_send_control_ack(uint8_t seq, uint8_t opcode, uint8_t status)
{
    uint8_t payload[3];

    payload[0] = seq;
    payload[1] = opcode;
    payload[2] = status;
    wifi_service_send_frame((uint8_t)WIFI_UART_CONTROL_TYPE_ACK, 0u, payload, (uint32_t)sizeof(payload));
}

static void wifi_service_send_fast_frame(uint8_t seq, const wifi_service_telemetry_snapshot_t *snapshot)
{
    uint8_t payload[22];
    uint32_t len = 0u;
    uint8_t status_flags;

    if (snapshot == NULL)
    {
        return;
    }

    status_flags = (uint8_t)((snapshot->run_enable ? 0x01u : 0u)
                           | (snapshot->line_pos_valid ? 0x02u : 0u)
                           | (snapshot->line_follow_ready ? 0x04u : 0u)
                           | (snapshot->odom_count_active ? 0x08u : 0u));

    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), seq);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload),
        wifi_service_wrap_deg(snapshot->roll_deg - g_wifi_att_zero_roll_deg), WIFI_UART_TELEMETRY_ANGLE_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload),
        wifi_service_wrap_deg(snapshot->pitch_deg - g_wifi_att_zero_pitch_deg), WIFI_UART_TELEMETRY_ANGLE_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload),
        wifi_service_wrap_deg(snapshot->yaw_deg - g_wifi_att_zero_yaw_deg), WIFI_UART_TELEMETRY_ANGLE_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->gyro_z_dps, WIFI_UART_TELEMETRY_GYRO_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->left_target_rpm, WIFI_UART_TELEMETRY_SPEED_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->right_target_rpm, WIFI_UART_TELEMETRY_SPEED_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->left_actual_rpm, WIFI_UART_TELEMETRY_SPEED_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->right_actual_rpm, WIFI_UART_TELEMETRY_SPEED_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->left_vq_out, WIFI_UART_TELEMETRY_VQ_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->right_vq_out, WIFI_UART_TELEMETRY_VQ_SCALE);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), status_flags);
    wifi_service_send_frame((uint8_t)WIFI_UART_TELEMETRY_TYPE_FAST, 0u, payload, len);
}

static void wifi_service_send_route_frame(uint8_t seq, const wifi_service_telemetry_snapshot_t *snapshot)
{
    uint8_t payload[29];
    uint32_t len = 0u;

    if (snapshot == NULL)
    {
        return;
    }

    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), seq);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->line_pos, WIFI_UART_TELEMETRY_ANGLE_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->turn_delta_rpm, WIFI_UART_TELEMETRY_SPEED_SCALE);
    len = wifi_service_frame_append_u16_le(payload, len, (uint32_t)sizeof(payload), snapshot->line_bits);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->lf_state);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->pending_turn);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->route_index);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->lost_line_cnt);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->stop_reason);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->junction_lock_cnt);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->element_enter_cnt);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->junction_enter_cnt);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->lf_actual_hz);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->calib_active);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->calib_samples_hi);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->calib_valid);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->turn_yaw_deg, WIFI_UART_TELEMETRY_ANGLE_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->left_elec_angle_deg, WIFI_UART_TELEMETRY_ANGLE_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->right_elec_angle_deg, WIFI_UART_TELEMETRY_ANGLE_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->odom_left_mm, WIFI_UART_TELEMETRY_ODOM_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->odom_right_mm, WIFI_UART_TELEMETRY_ODOM_SCALE);
    wifi_service_send_frame((uint8_t)WIFI_UART_TELEMETRY_TYPE_ROUTE, 0u, payload, len);
}

static void wifi_service_send_power_frame(uint8_t seq, const wifi_service_telemetry_snapshot_t *snapshot)
{
    uint8_t payload[10];
    uint32_t len = 0u;

    if (snapshot == NULL)
    {
        return;
    }

    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), seq);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->battery_voltage_v, WIFI_UART_TELEMETRY_VOLT_SCALE);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->battery_flags);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->battery_percent);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->np_target_percent, WIFI_UART_TELEMETRY_NP_PERCENT_SCALE);
    len = wifi_service_frame_append_s16_scaled(payload, len, (uint32_t)sizeof(payload), snapshot->np_output_pulse_us, WIFI_UART_TELEMETRY_NP_PULSE_US_SCALE);
    len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), snapshot->np_flags);
    wifi_service_send_frame((uint8_t)WIFI_UART_TELEMETRY_TYPE_POWER, 0u, payload, len);
}

static void wifi_service_send_gray_frame(uint8_t seq, const wifi_service_telemetry_snapshot_t *snapshot)
{
    uint8_t payload[1u + GRAY_4067_DEFAULT_CHANNEL_COUNT * 2u];
    uint32_t len = 0u;

    if ((snapshot == NULL) || !snapshot->gray_valid)
    {
        return;
    }

    if (g_wifi_gray_raw_enable != 0u)
    {
        len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), seq);
        for (uint8_t i = 0u; i < (uint8_t)GRAY_4067_DEFAULT_CHANNEL_COUNT; ++i)
        {
            uint16_t raw_value = snapshot->gray[i];
            len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), (uint8_t)(raw_value & 0xFFu));
            len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), (uint8_t)((raw_value >> 8u) & 0xFFu));
        }
        wifi_service_send_frame((uint8_t)WIFI_UART_TELEMETRY_TYPE_GRAY_RAW, 0u, payload, len);
    }
    else
    {
        len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), seq);
        for (uint8_t i = 0u; i < (uint8_t)GRAY_4067_DEFAULT_CHANNEL_COUNT; ++i)
        {
            uint16_t raw_value = snapshot->gray[i];
            uint8_t packed = (uint8_t)((raw_value >> WIFI_UART_TELEMETRY_GRAY_SHIFT) & 0xFFu);
            len = wifi_service_frame_append_u8(payload, len, (uint32_t)sizeof(payload), packed);
        }
        wifi_service_send_frame((uint8_t)WIFI_UART_TELEMETRY_TYPE_GRAY, 0u, payload, len);
    }
}

static uint32_t wifi_service_frame_append_u8(uint8_t *buf, uint32_t pos, uint32_t max_len, uint8_t value)
{
    if ((NULL == buf) || (pos >= max_len))
    {
        return pos;
    }

    buf[pos] = value;
    return pos + 1u;
}

static uint32_t wifi_service_frame_append_u16_le(uint8_t *buf, uint32_t pos, uint32_t max_len, uint16_t value)
{
    pos = wifi_service_frame_append_u8(buf, pos, max_len, (uint8_t)(value & 0xFFu));
    pos = wifi_service_frame_append_u8(buf, pos, max_len, (uint8_t)((value >> 8) & 0xFFu));
    return pos;
}

static uint32_t wifi_service_frame_append_s16_scaled(uint8_t *buf, uint32_t pos, uint32_t max_len, float value, float scale)
{
    sint16 packed = wifi_service_float_to_s16_scaled(value, scale);
    return wifi_service_frame_append_u16_le(buf, pos, max_len, (uint16_t)packed);
}

static sint16 wifi_service_float_to_s16_scaled(float value, float scale)
{
    float scaled_f = value * scale;
    int32_t scaled_i = (int32_t)(scaled_f + ((scaled_f >= 0.0f) ? 0.5f : -0.5f));

    if (scaled_i > 32767)
    {
        scaled_i = 32767;
    }
    else if (scaled_i < -32768)
    {
        scaled_i = -32768;
    }

    return (sint16)scaled_i;
}

static const uint8_t wifi_service_crc8_table[256] = {
    0x00u, 0x07u, 0x0Eu, 0x09u, 0x1Cu, 0x1Bu, 0x12u, 0x15u,
    0x38u, 0x3Fu, 0x36u, 0x31u, 0x24u, 0x23u, 0x2Au, 0x2Du,
    0x70u, 0x77u, 0x7Eu, 0x79u, 0x6Cu, 0x6Bu, 0x62u, 0x65u,
    0x48u, 0x4Fu, 0x46u, 0x41u, 0x54u, 0x53u, 0x5Au, 0x5Du,
    0xE0u, 0xE7u, 0xEEu, 0xE9u, 0xFCu, 0xFBu, 0xF2u, 0xF5u,
    0xD8u, 0xDFu, 0xD6u, 0xD1u, 0xC4u, 0xC3u, 0xCAu, 0xCDu,
    0x90u, 0x97u, 0x9Eu, 0x99u, 0x8Cu, 0x8Bu, 0x82u, 0x85u,
    0xA8u, 0xAFu, 0xA6u, 0xA1u, 0xB4u, 0xB3u, 0xBAu, 0xBDu,
    0xC7u, 0xC0u, 0xC9u, 0xCEu, 0xDBu, 0xDCu, 0xD5u, 0xD2u,
    0xFFu, 0xF8u, 0xF1u, 0xF6u, 0xE3u, 0xE4u, 0xEDu, 0xEAu,
    0xB7u, 0xB0u, 0xB9u, 0xBEu, 0xABu, 0xACu, 0xA5u, 0xA2u,
    0x8Fu, 0x88u, 0x81u, 0x86u, 0x93u, 0x94u, 0x9Du, 0x9Au,
    0x27u, 0x20u, 0x29u, 0x2Eu, 0x3Bu, 0x3Cu, 0x35u, 0x32u,
    0x1Fu, 0x18u, 0x11u, 0x16u, 0x03u, 0x04u, 0x0Du, 0x0Au,
    0x57u, 0x50u, 0x59u, 0x5Eu, 0x4Bu, 0x4Cu, 0x45u, 0x42u,
    0x6Fu, 0x68u, 0x61u, 0x66u, 0x73u, 0x74u, 0x7Du, 0x7Au,
    0x89u, 0x8Eu, 0x87u, 0x80u, 0x95u, 0x92u, 0x9Bu, 0x9Cu,
    0xB1u, 0xB6u, 0xBFu, 0xB8u, 0xADu, 0xAAu, 0xA3u, 0xA4u,
    0xF9u, 0xFEu, 0xF7u, 0xF0u, 0xE5u, 0xE2u, 0xEBu, 0xECu,
    0xC1u, 0xC6u, 0xCFu, 0xC8u, 0xDDu, 0xDAu, 0xD3u, 0xD4u,
    0x69u, 0x6Eu, 0x67u, 0x60u, 0x75u, 0x72u, 0x7Bu, 0x7Cu,
    0x51u, 0x56u, 0x5Fu, 0x58u, 0x4Du, 0x4Au, 0x43u, 0x44u,
    0x19u, 0x1Eu, 0x17u, 0x10u, 0x05u, 0x02u, 0x0Bu, 0x0Cu,
    0x21u, 0x26u, 0x2Fu, 0x28u, 0x3Du, 0x3Au, 0x33u, 0x34u,
    0x4Eu, 0x49u, 0x40u, 0x47u, 0x52u, 0x55u, 0x5Cu, 0x5Bu,
    0x76u, 0x71u, 0x78u, 0x7Fu, 0x6Au, 0x6Du, 0x64u, 0x63u,
    0x3Eu, 0x39u, 0x30u, 0x37u, 0x22u, 0x25u, 0x2Cu, 0x2Bu,
    0x06u, 0x01u, 0x08u, 0x0Fu, 0x1Au, 0x1Du, 0x14u, 0x13u,
    0xAEu, 0xA9u, 0xA0u, 0xA7u, 0xB2u, 0xB5u, 0xBCu, 0xBBu,
    0x96u, 0x91u, 0x98u, 0x9Fu, 0x8Au, 0x8Du, 0x84u, 0x83u,
    0xDEu, 0xD9u, 0xD0u, 0xD7u, 0xC2u, 0xC5u, 0xCCu, 0xCBu,
    0xE6u, 0xE1u, 0xE8u, 0xEFu, 0xFAu, 0xFDu, 0xF4u, 0xF3u,
};

static uint8_t wifi_service_frame_checksum(const uint8_t *buf, uint32_t start, uint32_t len)
{
    uint8_t crc = 0x00u;

    if (NULL == buf)
    {
        return 0u;
    }

    for (uint32_t i = 0u; i < len; ++i)
    {
        crc = wifi_service_crc8_table[crc ^ buf[start + i]];
    }

    return crc;
}

static void wifi_service_copy_snapshot(wifi_service_telemetry_snapshot_t *dst, volatile const wifi_service_telemetry_snapshot_t *src)
{
    if ((NULL == dst) || (NULL == src))
    {
        return;
    }

    dst->roll_deg = src->roll_deg;
    dst->pitch_deg = src->pitch_deg;
    dst->yaw_deg = src->yaw_deg;
    dst->gyro_z_dps = src->gyro_z_dps;
    dst->left_target_rpm = src->left_target_rpm;
    dst->right_target_rpm = src->right_target_rpm;
    dst->left_actual_rpm = src->left_actual_rpm;
    dst->right_actual_rpm = src->right_actual_rpm;
    dst->left_elec_angle_deg = src->left_elec_angle_deg;
    dst->right_elec_angle_deg = src->right_elec_angle_deg;
    dst->battery_voltage_v = src->battery_voltage_v;
    dst->battery_raw = src->battery_raw;
    dst->battery_flags = src->battery_flags;
    dst->battery_percent = src->battery_percent;
    dst->np_target_percent = src->np_target_percent;
    dst->np_output_pulse_us = src->np_output_pulse_us;
    dst->np_flags = src->np_flags;
    dst->line_pos = src->line_pos;
    dst->turn_delta_rpm = src->turn_delta_rpm;
    dst->line_bits = src->line_bits;
    dst->run_enable = src->run_enable;
    dst->line_pos_valid = src->line_pos_valid;
    dst->line_follow_ready = src->line_follow_ready;
    dst->lf_state = src->lf_state;
    dst->pending_turn = src->pending_turn;
    dst->route_index = src->route_index;
    dst->lost_line_cnt = src->lost_line_cnt;
    dst->stop_reason = src->stop_reason;
    dst->junction_lock_cnt = src->junction_lock_cnt;
    dst->element_enter_cnt = src->element_enter_cnt;
    dst->junction_enter_cnt = src->junction_enter_cnt;
    dst->lf_actual_hz = src->lf_actual_hz;
    dst->calib_active = src->calib_active;
    dst->calib_samples_hi = src->calib_samples_hi;
    dst->calib_valid = src->calib_valid;
    dst->turn_yaw_deg = src->turn_yaw_deg;
    dst->odom_left_mm = src->odom_left_mm;
    dst->odom_right_mm = src->odom_right_mm;
    dst->odom_count_active = src->odom_count_active;
    dst->left_vq_out = src->left_vq_out;
    dst->right_vq_out = src->right_vq_out;
    dst->gray_valid = src->gray_valid;

    for (uint8_t i = 0u; i < (uint8_t)GRAY_4067_DEFAULT_CHANNEL_COUNT; ++i)
    {
        dst->gray[i] = src->gray[i];
    }
}

static uint8_t wifi_service_load_latest_snapshot(wifi_service_telemetry_snapshot_t *out)
{
    uint8_t ready_index;

    if (NULL == out)
    {
        return 0u;
    }

    interrupt_global_disable();
    if (!g_wifi_snapshot_ready)
    {
        interrupt_global_enable(0u);
        return 0u;
    }

    ready_index = g_wifi_snapshot_ready_index;
    wifi_service_copy_snapshot(out, &g_wifi_snapshot_buffers[ready_index]);
    g_wifi_snapshot_ready = 0u;
    interrupt_global_enable(0u);
    return 1u;
}

static void wifi_service_poll_command(void)
{
    uint8_t rx_buf[64];
    uint32_t rx_len;

    while ((rx_len = wifi_uart_read_buffer(rx_buf, (uint32_t)sizeof(rx_buf))) > 0u)
    {
        if ((g_wifi_rx_stream_len + rx_len) > (uint32_t)sizeof(g_wifi_rx_stream))
        {
            g_wifi_rx_stream_len = 0u;
            (void)wifi_uart_send_buffer((const uint8_t *)"ERR:CMD_OVF\n", (uint32_t)(sizeof("ERR:CMD_OVF\n") - 1u));
        }
        else
        {
            memcpy(&g_wifi_rx_stream[g_wifi_rx_stream_len], rx_buf, rx_len);
            g_wifi_rx_stream_len += rx_len;
        }

        while (wifi_service_process_rx_stream())
        {
        }
    }
}

static void wifi_service_handle_binary_frame(const uint8_t *frame, uint32_t frame_len)
{
    const uint8_t *payload;
    uint32_t payload_len;
    uint8_t type;
    uint8_t seq;
    uint8_t opcode;
    uint8_t status;

    if ((frame == NULL) || (frame_len < 6u))
    {
        return;
    }

    type = frame[2];
    payload_len = frame[4];
    payload = &frame[5];
    if (type != (uint8_t)WIFI_UART_CONTROL_TYPE_CMD)
    {
        return;
    }

    if (payload_len < 2u)
    {
        return;
    }

    seq = payload[0];
    opcode = payload[1];

    {
        uint16_t cmd_id = ++g_wifi_cmd_monotonic;

        if (g_wifi_last_ok_cmd_valid
            && (seq == g_wifi_last_ok_cmd_seq)
            && (opcode == g_wifi_last_ok_cmd_opcode)
            && ((uint16_t)(cmd_id - g_wifi_last_ok_cmd_monotonic) < 128u))
        {
            wifi_service_send_control_ack(seq, opcode, (uint8_t)WIFI_UART_CONTROL_STATUS_OK);
            return;
        }

        status = wifi_service_execute_control_opcode(opcode, &payload[2], payload_len - 2u);
        if (status == (uint8_t)WIFI_UART_CONTROL_STATUS_OK)
        {
            g_wifi_last_ok_cmd_seq = seq;
            g_wifi_last_ok_cmd_opcode = opcode;
            g_wifi_last_ok_cmd_valid = 1u;
            g_wifi_last_ok_cmd_monotonic = cmd_id;
        }
    }
    wifi_service_send_control_ack(seq, opcode, status);
}

static uint8_t wifi_service_process_rx_stream(void)
{
    uint32_t frame_len;
    uint32_t line_end = 0xFFFFFFFFu;
    uint32_t magic_index = 0xFFFFFFFFu;
    static uint32_t last_progress_ms = 0u;

    if (g_wifi_rx_stream_len == 0u)
    {
        last_progress_ms = (uint32_t)system_getval_ms();
        return 0u;
    }

    if ((g_wifi_rx_stream[0] == (uint8_t)'\r') || (g_wifi_rx_stream[0] == (uint8_t)'\n'))
    {
        memmove(&g_wifi_rx_stream[0], &g_wifi_rx_stream[1], g_wifi_rx_stream_len - 1u);
        g_wifi_rx_stream_len--;
        return 1u;
    }

    if ((g_wifi_rx_stream_len >= 2u)
        && (g_wifi_rx_stream[0] == (uint8_t)WIFI_UART_TELEMETRY_MAGIC0)
        && (g_wifi_rx_stream[1] == (uint8_t)WIFI_UART_TELEMETRY_MAGIC1))
    {
        if (g_wifi_rx_stream_len < 6u)
        {
            return 0u;
        }

        frame_len = 6u + (uint32_t)g_wifi_rx_stream[4];
        if (frame_len > (uint32_t)sizeof(g_wifi_rx_stream))
        {
            memmove(&g_wifi_rx_stream[0], &g_wifi_rx_stream[1], g_wifi_rx_stream_len - 1u);
            g_wifi_rx_stream_len--;
            return 1u;
        }

        if (g_wifi_rx_stream_len < frame_len)
        {
            return 0u;
        }

        if (g_wifi_rx_stream[frame_len - 1u] != wifi_service_frame_checksum(g_wifi_rx_stream, 2u, frame_len - 3u))
        {
            uint32_t next_valid = 0xFFFFFFFFu;

            for (uint32_t idx = 1u; (idx + 5u) < g_wifi_rx_stream_len; ++idx)
            {
                if ((g_wifi_rx_stream[idx] == (uint8_t)WIFI_UART_TELEMETRY_MAGIC0)
                    && (g_wifi_rx_stream[idx + 1u] == (uint8_t)WIFI_UART_TELEMETRY_MAGIC1))
                {
                    uint32_t candidate_len = 6u + (uint32_t)g_wifi_rx_stream[idx + 4u];
                    if ((candidate_len <= (uint32_t)sizeof(g_wifi_rx_stream))
                        && ((idx + candidate_len) <= g_wifi_rx_stream_len)
                        && (g_wifi_rx_stream[idx + candidate_len - 1u]
                            == wifi_service_frame_checksum(g_wifi_rx_stream, idx + 2u, candidate_len - 3u)))
                    {
                        next_valid = idx;
                        break;
                    }
                }
            }

            if (next_valid != 0xFFFFFFFFu)
            {
                memmove(&g_wifi_rx_stream[0], &g_wifi_rx_stream[next_valid],
                        g_wifi_rx_stream_len - next_valid);
                g_wifi_rx_stream_len -= next_valid;
            }
            else
            {
                memmove(&g_wifi_rx_stream[0], &g_wifi_rx_stream[1], g_wifi_rx_stream_len - 1u);
                g_wifi_rx_stream_len--;
            }
            return 1u;
        }

        wifi_service_handle_binary_frame(g_wifi_rx_stream, frame_len);
        memmove(&g_wifi_rx_stream[0], &g_wifi_rx_stream[frame_len], g_wifi_rx_stream_len - frame_len);
        g_wifi_rx_stream_len -= frame_len;
        return 1u;
    }

    for (uint32_t i = 0u; i < g_wifi_rx_stream_len; ++i)
    {
        if ((g_wifi_rx_stream[i] == (uint8_t)'\r') || (g_wifi_rx_stream[i] == (uint8_t)'\n'))
        {
            line_end = i;
            break;
        }
        if ((i + 1u < g_wifi_rx_stream_len)
            && (g_wifi_rx_stream[i] == (uint8_t)WIFI_UART_TELEMETRY_MAGIC0)
            && (g_wifi_rx_stream[i + 1u] == (uint8_t)WIFI_UART_TELEMETRY_MAGIC1))
        {
            magic_index = i;
            break;
        }
    }

    if (line_end != 0xFFFFFFFFu)
    {
        if (line_end >= (uint32_t)WIFI_UART_CMD_LINE_MAX)
        {
            (void)wifi_uart_send_buffer((const uint8_t *)"ERR:CMD_OVF\n", (uint32_t)(sizeof("ERR:CMD_OVF\n") - 1u));
        }
        else if (line_end > 0u)
        {
            wifi_service_handle_command(g_wifi_rx_stream, line_end);
        }

        while ((line_end < g_wifi_rx_stream_len)
            && ((g_wifi_rx_stream[line_end] == (uint8_t)'\r') || (g_wifi_rx_stream[line_end] == (uint8_t)'\n')))
        {
            line_end++;
        }
        memmove(&g_wifi_rx_stream[0], &g_wifi_rx_stream[line_end], g_wifi_rx_stream_len - line_end);
        g_wifi_rx_stream_len -= line_end;
        return 1u;
    }

    if (magic_index != 0xFFFFFFFFu)
    {
        memmove(&g_wifi_rx_stream[0], &g_wifi_rx_stream[magic_index], g_wifi_rx_stream_len - magic_index);
        g_wifi_rx_stream_len -= magic_index;
        return 1u;
    }

    if (g_wifi_rx_stream_len >= (uint32_t)WIFI_UART_CMD_LINE_MAX)
    {
        g_wifi_rx_stream_len = 0u;
        (void)wifi_uart_send_buffer((const uint8_t *)"ERR:CMD_OVF\n", (uint32_t)(sizeof("ERR:CMD_OVF\n") - 1u));
        return 1u;
    }

    {
        uint32_t now_ms = (uint32_t)system_getval_ms();
        if ((uint32_t)(now_ms - last_progress_ms) >= 1000u)
        {
            g_wifi_rx_stream_len = 0u;
            last_progress_ms = now_ms;
            return 1u;
        }
    }

    return 0u;
}

static void wifi_service_handle_command(const uint8_t *cmd, uint32_t len)
{
    menu_state_t page_state = MENU_MAIN;

    if ((NULL == cmd) || (0u == len))
    {
        return;
    }

    if (wifi_service_cmd_equals(cmd, len, "PAGE?"))
    {
        char page_buf[48];
        const char *page_name = menu_get_page_name(g_menu_state);
        int n = snprintf(page_buf, sizeof(page_buf), "PAGE:%s\n", page_name);
        if (n > 0)
        {
            (void)wifi_uart_send_buffer((const uint8_t *)page_buf, (uint32_t)n);
        }
        return;
    }

    if (wifi_service_cmd_starts_with(cmd, len, "PAGE:"))
    {
        if (menu_get_state_from_page_name(cmd + (sizeof("PAGE:") - 1u), len - (sizeof("PAGE:") - 1u), &page_state))
        {
            char page_buf[48];
            const char *page_name;
            int n;

            // Safety interlock:
            // If FOC test is running, switching to a non-FOC page must stop outputs.
            // Otherwise the last PWM duty may remain latched after leaving the test page.
            if ((g_foc_test_run != 0u) && (page_state != MENU_FOC_SPEED_DEBUG))
            {
                wifi_service_force_pause();
            }

            g_menu_state = page_state;
            (void)wifi_uart_send_buffer((const uint8_t *)"ACK:PAGE\n", 9u);

            page_name = menu_get_page_name(g_menu_state);
            n = snprintf(page_buf, sizeof(page_buf), "PAGE:%s\n", page_name);
            if (n > 0)
            {
                (void)wifi_uart_send_buffer((const uint8_t *)page_buf, (uint32_t)n);
            }
        }
        else
        {
            (void)wifi_uart_send_buffer((const uint8_t *)"ERR:PAGE\n", 9u);
        }
        return;
    }

    if (wifi_service_cmd_equals(cmd, len, "PAGEPARAM?"))
    {
        wifi_service_send_page_params(g_menu_state);
        return;
    }

    if (wifi_service_cmd_starts_with(cmd, len, "PAGEPARAM:"))
    {
        if (menu_get_state_from_page_name(cmd + (sizeof("PAGEPARAM:") - 1u), len - (sizeof("PAGEPARAM:") - 1u), &page_state))
        {
            wifi_service_send_page_params(page_state);
        }
        else
        {
            (void)wifi_uart_send_buffer((const uint8_t *)"ERR:PAGEPARAM\n", 14u);
        }
        return;
    }

    if (wifi_service_cmd_equals(cmd, len, "GRAY?"))
    {
        wifi_service_send_param_line_u8("GRAY", g_wifi_gray_stream_enable);
        return;
    }

    if (wifi_service_cmd_starts_with(cmd, len, "GRAY:"))
    {
        char value_buf[8];
        uint32_t value_len = len - (sizeof("GRAY:") - 1u);
        char *end_ptr = NULL;
        double parsed = 0.0;

        if ((value_len == 0u) || (value_len >= (uint32_t)sizeof(value_buf)))
        {
            (void)wifi_uart_send_buffer((const uint8_t *)"ERR:GRAY\n", (uint32_t)(sizeof("ERR:GRAY\n") - 1u));
            return;
        }

        for (uint32_t i = 0u; i < value_len; ++i)
        {
            value_buf[i] = (char)cmd[(sizeof("GRAY:") - 1u) + i];
        }
        value_buf[value_len] = '\0';
        parsed = strtod(value_buf, &end_ptr);
        if ((end_ptr == value_buf) || ((end_ptr != NULL) && (*end_ptr != '\0')) || ((parsed != 0.0) && (parsed != 1.0)))
        {
            (void)wifi_uart_send_buffer((const uint8_t *)"ERR:GRAY\n", (uint32_t)(sizeof("ERR:GRAY\n") - 1u));
            return;
        }

        wifi_service_try_set_gray_stream((uint8_t)(parsed >= 0.5 ? 1u : 0u));
        (void)wifi_uart_send_buffer((const uint8_t *)"ACK:GRAY\n", (uint32_t)(sizeof("ACK:GRAY\n") - 1u));
        return;
    }

    if (wifi_service_cmd_equals(cmd, len, "GRAY_RAW?"))
    {
        wifi_service_send_param_line_u8("GRAY_RAW", g_wifi_gray_raw_enable);
        return;
    }

    if (wifi_service_cmd_starts_with(cmd, len, "GRAY_RAW:"))
    {
        char value_buf[8];
        uint32_t value_len = len - (sizeof("GRAY_RAW:") - 1u);
        char *end_ptr = NULL;
        double parsed = 0.0;

        if ((value_len == 0u) || (value_len >= (uint32_t)sizeof(value_buf)))
        {
            (void)wifi_uart_send_buffer((const uint8_t *)"ERR:GRAY_RAW\n", (uint32_t)(sizeof("ERR:GRAY_RAW\n") - 1u));
            return;
        }

        for (uint32_t i = 0u; i < value_len; ++i)
        {
            value_buf[i] = (char)cmd[(sizeof("GRAY_RAW:") - 1u) + i];
        }
        value_buf[value_len] = '\0';
        parsed = strtod(value_buf, &end_ptr);
        if ((end_ptr == value_buf) || ((end_ptr != NULL) && (*end_ptr != '\0')) || ((parsed != 0.0) && (parsed != 1.0)))
        {
            (void)wifi_uart_send_buffer((const uint8_t *)"ERR:GRAY_RAW\n", (uint32_t)(sizeof("ERR:GRAY_RAW\n") - 1u));
            return;
        }

        g_wifi_gray_raw_enable = (uint8_t)(parsed >= 0.5 ? 1u : 0u);
        (void)wifi_uart_send_buffer((const uint8_t *)"ACK:GRAY_RAW\n", (uint32_t)(sizeof("ACK:GRAY_RAW\n") - 1u));
        return;
    }

    // ---- 灰度校准远程控制 ----
    if (wifi_service_cmd_equals(cmd, len, "CALIB:START"))
    {
        line_follow_calib_start();
        (void)wifi_uart_send_buffer((const uint8_t *)"ACK:CALIB\n", 10u);
        return;
    }
    if (wifi_service_cmd_equals(cmd, len, "CALIB:STOP"))
    {
        line_follow_calib_stop();
        (void)wifi_uart_send_buffer((const uint8_t *)"ACK:CALIB\n", 10u);
        return;
    }
    if (wifi_service_cmd_equals(cmd, len, "CALIB:CLEAR"))
    {
        line_follow_calib_clear();
        (void)wifi_uart_send_buffer((const uint8_t *)"ACK:CALIB\n", 10u);
        return;
    }
    if (wifi_service_cmd_equals(cmd, len, "CALIB?"))
    {
        uint16_t min_buf[16], max_buf[16];
        char resp[384];
        int n;

        line_follow_calib_get_minmax(min_buf, max_buf, 16);

        n = snprintf(resp, sizeof(resp),
            "CALIB:%u,%lu,%u\r\n",
            (unsigned)line_follow_calib_is_active(),
            (unsigned long)line_follow_calib_get_samples(),
            (unsigned)line_follow_calib_is_valid());
        if (n > 0) (void)wifi_uart_send_buffer((const uint8_t *)resp, (uint32_t)n);

        n = snprintf(resp, sizeof(resp),
            "MIN:%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n"
            "MAX:%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
            (unsigned)min_buf[0], (unsigned)min_buf[1], (unsigned)min_buf[2], (unsigned)min_buf[3],
            (unsigned)min_buf[4], (unsigned)min_buf[5], (unsigned)min_buf[6], (unsigned)min_buf[7],
            (unsigned)min_buf[8], (unsigned)min_buf[9], (unsigned)min_buf[10], (unsigned)min_buf[11],
            (unsigned)min_buf[12], (unsigned)min_buf[13], (unsigned)min_buf[14], (unsigned)min_buf[15],
            (unsigned)max_buf[0], (unsigned)max_buf[1], (unsigned)max_buf[2], (unsigned)max_buf[3],
            (unsigned)max_buf[4], (unsigned)max_buf[5], (unsigned)max_buf[6], (unsigned)max_buf[7],
            (unsigned)max_buf[8], (unsigned)max_buf[9], (unsigned)max_buf[10], (unsigned)max_buf[11],
            (unsigned)max_buf[12], (unsigned)max_buf[13], (unsigned)max_buf[14], (unsigned)max_buf[15]);
        if (n > 0) (void)wifi_uart_send_buffer((const uint8_t *)resp, (uint32_t)n);
        return;
    }

    if (wifi_service_cmd_equals(cmd, len, "PAUSE") || wifi_service_cmd_equals(cmd, len, "STOP"))
    {
        wifi_service_force_pause();
        (void)wifi_uart_send_buffer((const uint8_t *)"ACK:PAUSE\n", 10u);
        return;
    }

    if (wifi_service_cmd_equals(cmd, len, "START"))
    {
        if (g_foc_test_run != 0u)
        {
            (void)wifi_uart_send_buffer((const uint8_t *)"NACK:START:TST_RUN\n", (uint32_t)(sizeof("NACK:START:TST_RUN\n") - 1u));
            return;
        }

        if (wifi_service_try_start())
        {
            (void)wifi_uart_send_buffer((const uint8_t *)"ACK:START\n", 10u);
        }
        else
        {
            (void)wifi_uart_send_buffer((const uint8_t *)"NACK:START:CALIB\n", 17u);
        }
        return;
    }

    if (wifi_service_cmd_equals(cmd, len, "ODOM_START"))
    {
        move_distance_counter_start();
        (void)wifi_uart_send_buffer((const uint8_t *)"ACK:ODOM_START\n", (uint32_t)(sizeof("ACK:ODOM_START\n") - 1u));
        return;
    }

    if (wifi_service_cmd_equals(cmd, len, "ODOM_CLEAR"))
    {
        move_distance_counter_clear();
        (void)wifi_uart_send_buffer((const uint8_t *)"ACK:ODOM_CLEAR\n", (uint32_t)(sizeof("ACK:ODOM_CLEAR\n") - 1u));
        return;
    }

    if (wifi_service_cmd_equals(cmd, len, "NPON"))
    {
        wifi_service_try_set_np_enable(1u);
        (void)wifi_uart_send_buffer((const uint8_t *)"ACK:NPON\n", 9u);
        return;
    }

    if (wifi_service_cmd_equals(cmd, len, "NPOFF"))
    {
        wifi_service_try_set_np_enable(0u);
        (void)wifi_uart_send_buffer((const uint8_t *)"ACK:NPOFF\n", 10u);
        return;
    }

    if (wifi_service_cmd_equals(cmd, len, "SAVE"))
    {
        if (g_run_enable)
        {
            (void)wifi_uart_send_buffer((const uint8_t *)"ERR:RUNNING\n", 12u);
            return;
        }
        params_save_to_flash();
        (void)wifi_uart_send_buffer((const uint8_t *)"ACK:SAVE\n", 9u);
        return;
    }

    if (wifi_service_try_handle_registered_param(cmd, len))
    {
        return;
    }

    (void)wifi_uart_send_buffer((const uint8_t *)"ERR:UNKNOWN\n", (uint32_t)(sizeof("ERR:UNKNOWN\n") - 1u));
}

static void wifi_service_send_page_changed_event(void)
{
    static uint8_t page_report_valid = 0u;
    static menu_state_t last_page_state = MENU_MAIN;

    if (WIFI_UART_SERVER_ON != wifi_uart_information.wifi_uart_connect_state)
    {
        page_report_valid = 0u;
        return;
    }

    if ((!page_report_valid) || (last_page_state != g_menu_state))
    {
        char page_buf[48];
        int n = snprintf(page_buf, sizeof(page_buf), "PAGE:%s\n", menu_get_page_name(g_menu_state));
        if (n > 0)
        {
            (void)wifi_uart_send_buffer((const uint8_t *)page_buf, (uint32_t)n);
        }
        last_page_state = g_menu_state;
        page_report_valid = 1u;
    }
}

static void wifi_service_send_param_line_float(const char *name, float value)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "PARAM:%s=%f\n", name, (double)value);
    if (n > 0)
    {
        (void)wifi_uart_send_buffer((const uint8_t *)buf, (uint32_t)n);
    }
}

static void wifi_service_send_param_line_u8(const char *name, uint8_t value)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "PARAM:%s=%f\n", name, (double)value);
    if (n > 0)
    {
        (void)wifi_uart_send_buffer((const uint8_t *)buf, (uint32_t)n);
    }
}

static void wifi_service_send_param_line(const params_descriptor_t *desc)
{
    if (desc == NULL)
    {
        return;
    }

    if (desc->type == PARAMS_VALUE_FLOAT)
    {
        wifi_service_send_param_line_float(desc->key, params_get_value_as_float(desc));
    }
    else
    {
        wifi_service_send_param_line_u8(desc->key, (uint8_t)params_get_value_as_float(desc));
    }
}

static uint8_t wifi_service_try_handle_registered_param(const uint8_t *cmd, uint32_t len)
{
    const params_descriptor_t *desc;
    uint32_t colon_index;

    if ((cmd == NULL) || (len == 0u))
    {
        return 0u;
    }

    if (cmd[len - 1u] == (uint8_t)'?')
    {
        desc = params_find_descriptor((const char *)cmd, len - 1u);
        if (desc != NULL)
        {
            wifi_service_send_param_line(desc);
            return 1u;
        }
        return 0u;
    }

    for (colon_index = 0u; colon_index < len; ++colon_index)
    {
        if (cmd[colon_index] == (uint8_t)':')
        {
            char value_buf[32];
            uint32_t value_len;
            char *end_ptr = NULL;
            double parsed_value;
            char ack_buf[48];
            int ack_len;

            desc = params_find_descriptor((const char *)cmd, colon_index);
            if (desc == NULL)
            {
                return 0u;
            }

            if (g_run_enable)
            {
                (void)wifi_uart_send_buffer((const uint8_t *)"ERR:RUNNING\n", 12u);
                return 1u;
            }

            value_len = len - colon_index - 1u;
            if ((value_len == 0u) || (value_len >= (uint32_t)sizeof(value_buf)))
            {
                (void)wifi_uart_send_buffer((const uint8_t *)"ERR:PARAM\n", 10u);
                return 1u;
            }

            for (uint32_t i = 0u; i < value_len; ++i)
            {
                value_buf[i] = (char)cmd[colon_index + 1u + i];
            }
            value_buf[value_len] = '\0';

            parsed_value = strtod(value_buf, &end_ptr);
            if ((end_ptr == value_buf) || ((end_ptr != NULL) && (*end_ptr != '\0')))
            {
                (void)wifi_uart_send_buffer((const uint8_t *)"ERR:PARAM\n", 10u);
                return 1u;
            }

            params_set_value_from_float(desc, (float)parsed_value);

            ack_len = snprintf(ack_buf, sizeof(ack_buf), "ACK:%s\n", desc->key);
            if (ack_len > 0)
            {
                (void)wifi_uart_send_buffer((const uint8_t *)ack_buf, (uint32_t)ack_len);
            }
            return 1u;
        }
    }

    return 0u;
}

static void wifi_service_send_page_params(menu_state_t page_state)
{
    char head_buf[64];
    const char *page_name = menu_get_page_name(page_state);
    int n = snprintf(head_buf, sizeof(head_buf), "PAGEPARAM:%s\n", page_name);
    if (n > 0)
    {
        (void)wifi_uart_send_buffer((const uint8_t *)head_buf, (uint32_t)n);
    }

    for (uint16_t i = 0u; i < params_get_page_item_count(page_state); ++i)
    {
        wifi_service_send_param_line(params_get_page_item(page_state, i));
    }

    n = snprintf(head_buf, sizeof(head_buf), "PAGEPARAM_END:%s\n", page_name);
    if (n > 0)
    {
        (void)wifi_uart_send_buffer((const uint8_t *)head_buf, (uint32_t)n);
    }
}

static uint8_t wifi_service_cmd_equals(const uint8_t *cmd, uint32_t len, const char *keyword)
{
    uint32_t keyword_len = 0u;
    uint32_t index = 0u;

    if ((NULL == cmd) || (NULL == keyword))
    {
        return 0u;
    }

    while ('\0' != keyword[keyword_len])
    {
        keyword_len++;
    }

    if (len != keyword_len)
    {
        return 0u;
    }

    for (index = 0u; index < len; ++index)
    {
        if (!wifi_service_ascii_equal_ignore_case(cmd[index], (uint8_t)keyword[index]))
        {
            return 0u;
        }
    }

    return 1u;
}

static uint8_t wifi_service_cmd_starts_with(const uint8_t *cmd, uint32_t len, const char *keyword)
{
    uint32_t keyword_len = 0u;
    uint32_t index = 0u;

    if ((NULL == cmd) || (NULL == keyword))
    {
        return 0u;
    }

    while ('\0' != keyword[keyword_len])
    {
        keyword_len++;
    }

    if (len < keyword_len)
    {
        return 0u;
    }

    for (index = 0u; index < keyword_len; ++index)
    {
        if (!wifi_service_ascii_equal_ignore_case(cmd[index], (uint8_t)keyword[index]))
        {
            return 0u;
        }
    }

    return 1u;
}

static uint8_t wifi_service_ascii_equal_ignore_case(uint8_t lhs, uint8_t rhs)
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

static uint8_t wifi_service_neg_pressure_run_requested(void)
{
    return (uint8_t)((g_run_enable != 0u)
                  || ((g_menu_state == MENU_NEG_PRESSURE_DEBUG) && (g_neg_pressure_enable != 0u)));
}

static void wifi_service_force_pause(void)
{
    g_run_enable = 0u;
    g_foc_test_run = 0u;
    g_neg_pressure_enable = 0u;
    g_stop_reason = SYSTEM_STOP_REASON_NONE;
    g_turn_delta_rpm = 0.0f;
    g_target_left_rpm = 0.0f;
    g_target_right_rpm = 0.0f;
    move_distance_stop();
    line_follow_reset();
    foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);
    foc_speed_loop_reset_on_enable(&g_loop_l);
    foc_speed_loop_reset_on_enable(&g_loop_r);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
    neg_pressure_motor_stop(&g_neg_pressure_motor);
}

static void wifi_service_prepare_main_run_start(void)
{
    g_stop_reason = SYSTEM_STOP_REASON_NONE;
    g_foc_test_run = 0u;
    g_neg_pressure_enable = 0u;
    g_turn_delta_rpm = 0.0f;
    g_target_left_rpm = 0.0f;
    g_target_right_rpm = 0.0f;

    move_distance_stop();
    move_distance_counter_start();
    line_follow_reset();
    foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);
    foc_speed_loop_reset_on_enable(&g_loop_l);
    foc_speed_loop_reset_on_enable(&g_loop_r);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
    bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
    neg_pressure_motor_stop(&g_neg_pressure_motor);
}

static uint8_t wifi_service_try_start(void)
{
    if (g_run_enable != 0u)
    {
        return 1u;
    }

    if (!line_follow_control_ready())
    {
        return 0u;
    }

    wifi_service_prepare_main_run_start();
    g_run_enable = 1u;
    return 1u;
}

static void wifi_service_try_set_np_enable(uint8_t enable)
{
    g_neg_pressure_enable = (uint8_t)(enable ? 1u : 0u);
    if (!wifi_service_neg_pressure_run_requested())
    {
        neg_pressure_motor_stop(&g_neg_pressure_motor);
    }
}

static void wifi_service_try_set_gray_stream(uint8_t enable)
{
    g_wifi_gray_stream_enable = (uint8_t)(enable ? 1u : 0u);
}

static uint8_t wifi_service_execute_control_opcode(uint8_t opcode, const uint8_t *args, uint32_t arg_len)
{
    (void)args;

    if (arg_len != 0u && opcode != (uint8_t)WIFI_UART_CONTROL_OPCODE_PAUSE)
    {
        return (uint8_t)WIFI_UART_CONTROL_STATUS_INVALID_ARG;
    }

    switch (opcode)
    {
    case WIFI_UART_CONTROL_OPCODE_START:
        if (g_foc_test_run != 0u)
        {
            return (uint8_t)WIFI_UART_CONTROL_STATUS_FOC_TEST_RUNNING;
        }
        if (g_run_enable != 0u)
        {
            return (uint8_t)WIFI_UART_CONTROL_STATUS_BUSY;
        }
        if (!line_follow_control_ready())
        {
            return (uint8_t)WIFI_UART_CONTROL_STATUS_CALIB_NOT_READY;
        }
        wifi_service_prepare_main_run_start();
        g_run_enable = 1u;
        return (uint8_t)WIFI_UART_CONTROL_STATUS_OK;

    case WIFI_UART_CONTROL_OPCODE_PAUSE:
        wifi_service_force_pause();
        return (uint8_t)WIFI_UART_CONTROL_STATUS_OK;

    case WIFI_UART_CONTROL_OPCODE_ODOM_START:
        move_distance_counter_start();
        return (uint8_t)WIFI_UART_CONTROL_STATUS_OK;

    case WIFI_UART_CONTROL_OPCODE_ODOM_CLEAR:
        move_distance_counter_clear();
        return (uint8_t)WIFI_UART_CONTROL_STATUS_OK;

    case WIFI_UART_CONTROL_OPCODE_NPON:
        wifi_service_try_set_np_enable(1u);
        return (uint8_t)WIFI_UART_CONTROL_STATUS_OK;

    case WIFI_UART_CONTROL_OPCODE_NPOFF:
        wifi_service_try_set_np_enable(0u);
        return (uint8_t)WIFI_UART_CONTROL_STATUS_OK;

    default:
        return (uint8_t)WIFI_UART_CONTROL_STATUS_UNKNOWN_CMD;
    }
}

static float wifi_service_wrap_deg(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}
