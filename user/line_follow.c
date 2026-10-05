/*********************************************************************************************************************
* 文件：line_follow.c
* 模块：应用层（第 6 层）—— 巡线控制
* 职责：16 路灰度采样与线位置解算、路口/元素识别、转向 PID、丢线恢复
* 依赖：zf_device_gray_4067.h、zf_common_headfile.h、本目录 line_follow.h
* 风险等级：R2
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "line_follow.h"

#include <math.h>

#include "isr.h"
#include "menu.h"
#include "move_distance.h"
#include "pid.h"
#include "route_plan.h"
#include "zf_device_gray_4067.h"

#ifndef LINE_FOLLOW_INVERT
#define LINE_FOLLOW_INVERT          (0)
#endif

#ifndef LINE_FOLLOW_ROUTE_PLAN_ONLY
#define LINE_FOLLOW_ROUTE_PLAN_ONLY (1)
#endif

#ifndef LINE_FOLLOW_REVERSE_CHANNEL_ORDER
// 当前硬件接线下，15 号通道在左侧、0 号通道在右侧。
#define LINE_FOLLOW_REVERSE_CHANNEL_ORDER (1)
#endif

// 模块职责：
// 1. 感知层：读取灰度、完成校准、阈值判线与位置估计；
// 2. 路由层：识别路线项、维护预转弯/转弯状态机、与定距模块协作；
// 3. 控制层：输出循迹差速、处理丢线、决定何时停机。

typedef enum
{
    LF_STATE_FOLLOW = 0,
    LF_STATE_PRE_TURN,
    LF_STATE_TURN_LEFT_90,
    LF_STATE_TURN_RIGHT_90,
} lf_state_t;

typedef enum
{
    LF_FEATURE_NEAR_CENTER = 0x01u,
    LF_FEATURE_WIDE_HIT    = 0x02u,
    LF_FEATURE_BOTH_EDGE   = 0x04u,
    LF_FEATURE_EDGE_BRANCH = 0x08u,
    LF_FEATURE_LEFT_EDGE   = 0x10u,
    LF_FEATURE_RIGHT_EDGE  = 0x20u,
} lf_feature_flag_t;

typedef struct
{
    uint8_t min_hit_count;
    float max_abs_pos;
    uint8_t min_confirm_cnt;
    uint8_t required_flags;
    uint8_t forbidden_flags;
    float distance_mm;
} line_follow_elem_trigger_cfg_t;

typedef struct
{
    uint16_t raw[LINE_FOLLOW_CH_COUNT];
    float scores[LINE_FOLLOW_CH_COUNT];
    uint16_t bits;
    uint8_t hit_count;
    float pos;
    uint8_t pos_valid;
    uint8_t left_edge;
    uint8_t right_edge;
    uint8_t near_center;
    uint8_t wide_hit;
    uint8_t both_edge;
    uint8_t edge_branch;
    uint8_t looks_like_intersection;
    uint8_t feature_flags;
} line_follow_frame_t;

typedef struct
{
    lf_state_t state;
    route_turn_t pending_turn;
    float turn_yaw0_deg;
    float turn_time_s;
    float pre_turn_start_mm;
    float turn_start_mm;
    float last_seen_pos;
    float last_delta_rpm;
    float lost_line_time_s;
    float pos_filt;
    float last_pos_meas;
    float pos_rate_filt;
    uint8_t corner_cnt_l;
    uint8_t corner_cnt_r;
    uint8_t lost_line_cnt;
    uint8_t intersection_cnt;
    uint8_t last_pos_meas_valid;
    float intersection_lock_s;
    uint8_t element_confirm_cnt;
} line_follow_runtime_t;

typedef struct
{
    uint8_t apply_delta;
    uint8_t hold_last_delta;
    float delta_rpm;
} line_follow_decision_t;

static const uint16_t LF_CALIB_MIN_SPAN = 50u;
static const uint8_t LF_CALIB_MIN_GOOD_CH = (uint8_t)((LINE_FOLLOW_CH_COUNT * 3u) / 4u);
static const float LF_WEIGHT_FLOOR = 0.08f;
static const float LF_POS_LP_ALPHA = 0.35f;
static const float LF_POS_D_LP_ALPHA = 0.30f;
static const float LF_LOCAL_PEAK_NEIGHBOR_RATIO = 0.20f;
static const float LF_TURN_MAX_ODOM_MM = 350.0f;
static const uint8_t LF_LOST_LINE_HOLD_FRAMES = 3u;
static const float LF_LOST_LINE_SEARCH_RPM = 120.0f;
static const float LF_LOST_LINE_STOP_S = 0.3f;
static const uint8_t LF_LOST_LINE_MIN_HIT_COUNT = 2u;
static const uint8_t LF_CORNER_CONFIRM_FRAMES = 2u;
static const uint8_t LF_CORNER_HIT_MAX = 4u;
static const float LF_CORNER_POS_TH = 3.5f;

static uint16_t g_lf_calib_min[LINE_FOLLOW_CH_COUNT];
static uint16_t g_lf_calib_max[LINE_FOLLOW_CH_COUNT];
static uint16_t g_lf_calib_thr[LINE_FOLLOW_CH_COUNT];
static uint16_t g_lf_last_raw[LINE_FOLLOW_CH_COUNT];

static uint8_t g_lf_calib_active = 0u;
static uint8_t g_lf_calib_valid = 0u;
static uint8_t g_lf_calib_inited = 0u;
static uint32_t g_lf_calib_samples = 0u;

static float g_lf_last_pos = 0.0f;
static uint8_t g_lf_last_pos_valid = 0u;
static uint16_t g_lf_last_bits = 0u;

static line_follow_runtime_t g_lf_runtime = {
    LF_STATE_FOLLOW, ROUTE_TURN_STRAIGHT, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0u, 0u, 0u, 0u, 0u, 0.0f,
    0u
};

static line_follow_junction_debug_t g_lf_junc_dbg;
static volatile uint8_t g_junction_lock_complete_cnt = 0u;
static volatile uint8_t g_element_enter_cnt = 0u;
static volatile uint8_t g_junction_enter_cnt = 0u;
static volatile uint16_t g_lf_frame_cnt = 0u;
static uint8_t g_fork_turn_pending = 0u;

volatile line_follow_config_t g_line_follow_cfg = {
    60.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    50.0f,
    6u,
    6.5f,
    2u,
    0.02f,
    4u,
    1800.0f,
    20.0f,
    0.08f,
    55.0f,
    55.0f,
    0.5f,
    1.0f,
    -1.0f,
    0.0f
};

// 触发表字段顺序：{min_hit_count, max_abs_pos, min_confirm_cnt, required_flags, forbidden_flags, distance_mm}
static const line_follow_elem_trigger_cfg_t g_lf_elem_trigger_cfg[ROUTE_ELEM_COUNT] = {
    {4u,  5.0f, 2u, LF_FEATURE_WIDE_HIT, 0u,              150.0f},  // 按键
    {6u,  5.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,           150.0f},  // 灯泡
    {8u, 1.6f, 2u, LF_FEATURE_NEAR_CENTER, 0u,           150.0f},  // 电池_1型
    {8u, 1.6f, 3u, LF_FEATURE_NEAR_CENTER|LF_FEATURE_BOTH_EDGE,0u,150.0f}, // 电池_2型
    {6u,  6.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u, 150.0f},  // 电感_1型
    {4u,  6.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u, 150.0f},  // 电感_2型
    {4u,  5.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,            200.0f},  // 电流源
    {6u,  6.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,            200.0f},  // 电容
    {6u,  6.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,            200.0f},  // 电压源
    {8u,  6.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,            200.0f},  // 电源_2型
    {4u,  7.0f, 3u, LF_FEATURE_NEAR_CENTER, 0u,            200.0f}, // 电阻_1型
    {4u,  7.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,            200.0f},  // 电阻_2型
    {1u,  6.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,            200.0f},  // 定距巡线
    {6u,  2.4f, 2u, LF_FEATURE_NEAR_CENTER, 0u,             150.0f},  // 二极管_分叉_左
    {6u,  4.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,              150.0f},  // 二级管_分叉_右
    {6u,  6.0f, 3u, LF_FEATURE_NEAR_CENTER, 0u,            180.0f}, // 发光二极管
    {5u,  6.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,             200.0f},  // 非门
    {7u,  3.5f, 2u, LF_FEATURE_WIDE_HIT, 0u,               200.0f},  // 三极管_1型
    {8u,  2.4f, 2u, LF_FEATURE_LEFT_EDGE, 0u,              200.0f},  // 三极管_2型
    {6u,  4.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,             80.0f},  // 二极管_分叉_左_转弯
    {6u,  4.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,              80.0f},  // 二级管_分叉_右_转弯
    {4u,  7.5f, 2u, LF_FEATURE_NEAR_CENTER, 0u,              150.0f},  // 侧进三极管
    {6u,  2.4f, 2u, LF_FEATURE_NEAR_CENTER, 0u,             100.0f},  // 二极管2型_分叉_左
    {6u,  2.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,              100.0f},  // 二极管2型_分叉_右
    {4u,  4.0, 2u, LF_FEATURE_NEAR_CENTER, 0u,             50.0f},  // 二极管2型_分叉_左_转弯
    {4u,  4.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,              50.0f},  // 二极管2型_分叉_右_转弯
    {6u,  4.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,             50.0f},  // 二极管3型_分叉_左
    {6u,  4.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,              50.0f},  // 二极管3型_分叉_右
    {6u,  4.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,             50.0f},  // 二极管3型_分叉_左_转弯
    {6u,  4.0f, 2u, LF_FEATURE_NEAR_CENTER, 0u,              50.0f},  // 二极管3型_分叉_右_转弯
};

static float lf_clampf(float value, float min_value, float max_value);
static float lf_wrap_deg(float deg);
static float line_follow_unit_sign(float value, float fallback);
static float line_follow_coord_from_index(uint8_t index);
static float line_follow_calc_channel_score(uint16_t raw, uint16_t min_value, uint16_t max_value);
static uint8_t line_follow_estimate_local_peak_pos(const float *scores, float *pos_meas_out);
static uint8_t line_follow_build_feature_flags(const line_follow_frame_t *frame);
static uint8_t line_follow_match_element_trigger(route_turn_t turn, const line_follow_frame_t *frame);
static void line_follow_reset_steer_pid(void);
static void line_follow_reset_position_filter(void);
static void line_follow_reset_control_state(void);
static void line_follow_runtime_reset(void);
static void line_follow_reset_follow_transients(void);
static float line_follow_calc_follow_delta_rpm(float err, float pos_rate_filt, float gyro_z_dps);
static float line_follow_get_signed_turn_yaw_delta_deg(void);
static uint8_t line_follow_turn_exit_line_ready(const line_follow_frame_t *frame);
static void line_follow_calib_update_raw(const uint16_t *raw);
static void line_follow_apply_delta(float delta_rpm);
static float line_follow_pick_search_sign(void);
static void line_follow_lock_intersection(void);
static void line_follow_update_intersection_lock(float dt);
static void line_follow_begin_turn(lf_state_t state, route_turn_t turn, float start_mm);
static void line_follow_enter_pre_turn(route_turn_t turn);
static float line_follow_get_route_speed_rpm(void);
static void line_follow_start_distance_element(const route_plan_entry_t *entry);
static uint8_t line_follow_stop_if_route_done(void);
static uint8_t line_follow_handle_junction_event(const route_plan_entry_t *entry, const line_follow_frame_t *frame);
static uint8_t line_follow_handle_element_event(const route_plan_entry_t *entry, const line_follow_frame_t *frame);
static void line_follow_finish_turn(uint8_t commit_route);
static void line_follow_sample_frame(line_follow_frame_t *frame, float dt);
static void line_follow_decision_clear(line_follow_decision_t *decision);
static void line_follow_decision_set_delta(line_follow_decision_t *decision, float delta_rpm);
static void line_follow_decision_hold_last(line_follow_decision_t *decision);
static void line_follow_commit_decision(const line_follow_decision_t *decision);
static uint8_t line_follow_handle_pre_turn_stage(float dt, line_follow_decision_t *decision);
static uint8_t line_follow_handle_turn_stage(const line_follow_frame_t *frame, float dt, line_follow_decision_t *decision);
static uint8_t line_follow_handle_follow_stage(const line_follow_frame_t *frame, float dt, line_follow_decision_t *decision);
static void line_follow_update_debug(const line_follow_frame_t *frame);

static float lf_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float lf_wrap_deg(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

static float line_follow_unit_sign(float value, float fallback)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }
    if (value < 0.0f)
    {
        return -1.0f;
    }
    return (fallback >= 0.0f) ? 1.0f : -1.0f;
}

static float line_follow_coord_from_index(uint8_t index)
{
    float coord = (float)(2 * (int32_t)index - (int32_t)(LINE_FOLLOW_CH_COUNT - 1u)) * 0.5f;

    if (LINE_FOLLOW_REVERSE_CHANNEL_ORDER)
    {
        coord = -coord;
    }

    return coord;
}

static float line_follow_calc_channel_score(uint16_t raw, uint16_t min_value, uint16_t max_value)
{
    float span;
    float value;

    if (max_value <= min_value)
    {
        return 0.0f;
    }

    span = (float)((uint32_t)max_value - (uint32_t)min_value);
    value = ((float)raw - (float)min_value) / span;
    value = lf_clampf(value, 0.0f, 1.0f);

    if (!LINE_FOLLOW_INVERT)
    {
        value = 1.0f - value;
    }

    if (value <= LF_WEIGHT_FLOOR)
    {
        return 0.0f;
    }

    value = (value - LF_WEIGHT_FLOOR) / (1.0f - LF_WEIGHT_FLOOR);
    return lf_clampf(value, 0.0f, 1.0f);
}

static uint8_t line_follow_estimate_local_peak_pos(const float *scores, float *pos_meas_out)
{
    uint8_t peak_index = 0u;
    float peak_score = 0.0f;
    uint8_t start;
    uint8_t end;
    float weighted_sum = 0.0f;
    float weight_total = 0.0f;

    if ((scores == NULL) || (pos_meas_out == NULL))
    {
        return 0u;
    }

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        if (scores[i] > peak_score)
        {
            peak_score = scores[i];
            peak_index = i;
        }
    }

    if (peak_score <= 1e-6f)
    {
        return 0u;
    }

    start = peak_index;
    while ((start > 0u) && (scores[start - 1u] >= (peak_score * LF_LOCAL_PEAK_NEIGHBOR_RATIO)))
    {
        start--;
    }

    end = peak_index;
    while ((end + 1u < LINE_FOLLOW_CH_COUNT) && (scores[end + 1u] >= (peak_score * LF_LOCAL_PEAK_NEIGHBOR_RATIO)))
    {
        end++;
    }

    for (uint8_t i = start; i <= end; ++i)
    {
        float score = scores[i];
        float weight;

        if (score <= 1e-6f)
        {
            continue;
        }

        weight = score * score;
        weighted_sum += weight * line_follow_coord_from_index(i);
        weight_total += weight;
    }

    if (weight_total <= 1e-6f)
    {
        *pos_meas_out = line_follow_coord_from_index(peak_index);
    }
    else
    {
        *pos_meas_out = weighted_sum / weight_total;
    }

    return 1u;
}

static uint8_t line_follow_build_feature_flags(const line_follow_frame_t *frame)
{
    uint8_t flags = 0u;

    if (frame == NULL)
    {
        return 0u;
    }

    if (frame->near_center) flags |= LF_FEATURE_NEAR_CENTER;
    if (frame->wide_hit) flags |= LF_FEATURE_WIDE_HIT;
    if (frame->both_edge) flags |= LF_FEATURE_BOTH_EDGE;
    if (frame->edge_branch) flags |= LF_FEATURE_EDGE_BRANCH;
    if (frame->left_edge) flags |= LF_FEATURE_LEFT_EDGE;
    if (frame->right_edge) flags |= LF_FEATURE_RIGHT_EDGE;

    return flags;
}

static uint8_t line_follow_match_element_trigger(route_turn_t turn, const line_follow_frame_t *frame)
{
    uint8_t elem_index;
    const line_follow_elem_trigger_cfg_t *cfg;

    if (frame == NULL)
    {
        return 0u;
    }

    elem_index = route_plan_get_element_index(turn);
    if ((elem_index == 0u) || (elem_index > ROUTE_ELEM_COUNT))
    {
        return 0u;
    }

    cfg = &g_lf_elem_trigger_cfg[elem_index - 1u];

    if (frame->hit_count < cfg->min_hit_count)
    {
        return 0u;
    }
    if (fabsf(frame->pos) > cfg->max_abs_pos)
    {
        return 0u;
    }
    if ((frame->feature_flags & cfg->required_flags) != cfg->required_flags)
    {
        return 0u;
    }
    if ((frame->feature_flags & cfg->forbidden_flags) != 0u)
    {
        return 0u;
    }

    return 1u;
}

static void line_follow_reset_steer_pid(void)
{
    g_steer_pid.integ = 0.0f;
    g_steer_pid.error = 0.0f;
    g_steer_pid.last_error = 0.0f;
}

static void line_follow_reset_position_filter(void)
{
    g_lf_runtime.pos_filt = 0.0f;
    g_lf_runtime.last_pos_meas = 0.0f;
    g_lf_runtime.last_pos_meas_valid = 0u;
    g_lf_runtime.pos_rate_filt = 0.0f;
    g_lf_last_pos = 0.0f;
    g_lf_last_pos_valid = 0u;
}

static void line_follow_reset_control_state(void)
{
    line_follow_reset_steer_pid();
    line_follow_apply_delta(0.0f);

    g_lf_runtime.state = LF_STATE_FOLLOW;
    g_lf_runtime.pending_turn = ROUTE_TURN_STRAIGHT;
    g_lf_runtime.turn_yaw0_deg = 0.0f;
    g_lf_runtime.turn_time_s = 0.0f;
    g_lf_runtime.pre_turn_start_mm = 0.0f;
    g_lf_runtime.turn_start_mm = 0.0f;
    g_lf_runtime.last_seen_pos = 0.0f;
    g_lf_runtime.lost_line_time_s = 0.0f;
    g_lf_runtime.corner_cnt_l = 0u;
    g_lf_runtime.corner_cnt_r = 0u;
    g_lf_runtime.lost_line_cnt = 0u;
    g_lf_runtime.intersection_cnt = 0u;
    g_lf_runtime.intersection_lock_s = 0.0f;
    g_lf_runtime.element_confirm_cnt = 0u;
    g_junction_lock_complete_cnt = 0u;
    g_element_enter_cnt = 0u;
    g_junction_enter_cnt = 0u;
}

static void line_follow_runtime_reset(void)
{
    line_follow_reset_control_state();
    line_follow_reset_position_filter();
}

static void line_follow_reset_follow_transients(void)
{
    g_lf_runtime.state = LF_STATE_FOLLOW;
    g_lf_runtime.pending_turn = ROUTE_TURN_STRAIGHT;
    g_lf_runtime.turn_yaw0_deg = 0.0f;
    g_lf_runtime.turn_time_s = 0.0f;
    g_lf_runtime.pre_turn_start_mm = 0.0f;
    g_lf_runtime.turn_start_mm = 0.0f;
    g_lf_runtime.last_delta_rpm = 0.0f;
    g_lf_runtime.lost_line_time_s = 0.0f;
    g_lf_runtime.corner_cnt_l = 0u;
    g_lf_runtime.corner_cnt_r = 0u;
    g_lf_runtime.lost_line_cnt = 0u;
    g_lf_runtime.intersection_cnt = 0u;
    g_lf_runtime.intersection_lock_s = 0.0f;
    g_lf_runtime.element_confirm_cnt = 0u;
    g_junction_lock_complete_cnt = 0u;
    g_element_enter_cnt = 0u;
    g_junction_enter_cnt = 0u;
    line_follow_reset_steer_pid();
    line_follow_apply_delta(0.0f);
}

static float line_follow_calc_follow_delta_rpm(float err, float pos_rate_filt, float gyro_z_dps)
{
    float p_term = g_steer_pid.kp * err;
    float p2_term = g_line_follow_cfg.steer_kp2 * err * fabsf(err);
    float d_img_term = g_steer_pid.kd * pos_rate_filt;
    float d_gyro_term = g_line_follow_cfg.steer_gkd * gyro_z_dps;
    float out = p_term + p2_term + d_img_term + d_gyro_term;

    out = lf_clampf(out, -g_line_follow_cfg.delta_rpm_limit, g_line_follow_cfg.delta_rpm_limit);
    return -out;
}

static float line_follow_get_signed_turn_yaw_delta_deg(void)
{
    float raw_delta = lf_wrap_deg(g_att_yaw_deg - g_lf_runtime.turn_yaw0_deg);
    return line_follow_unit_sign(g_line_follow_cfg.turn_yaw_sign, 1.0f) * raw_delta;
}

void line_follow_apply_config(void)
{
    g_line_follow_cfg.steer_kp = lf_clampf(g_line_follow_cfg.steer_kp, 0.0f, 100.0f);
    g_line_follow_cfg.steer_kd = lf_clampf(g_line_follow_cfg.steer_kd, 0.0f, 100.0f);
    g_line_follow_cfg.steer_kp2 = lf_clampf(g_line_follow_cfg.steer_kp2, 0.0f, 100.0f);
    g_line_follow_cfg.steer_gkd = lf_clampf(g_line_follow_cfg.steer_gkd, 0.0f, 100.0f);
    g_line_follow_cfg.turn_speed_scale = lf_clampf(g_line_follow_cfg.turn_speed_scale, 0.0f, 0.01f);

    g_line_follow_cfg.pre_turn_distance_mm = lf_clampf(g_line_follow_cfg.pre_turn_distance_mm, 0.0f, 500.0f);
    g_line_follow_cfg.intersection_hit_min = (uint8_t)lf_clampf((float)g_line_follow_cfg.intersection_hit_min, 1.0f, (float)LINE_FOLLOW_CH_COUNT);
    g_line_follow_cfg.intersection_pos_max = lf_clampf(g_line_follow_cfg.intersection_pos_max, 0.0f, 10.0f);
    g_line_follow_cfg.intersection_confirm_frames = (uint8_t)lf_clampf((float)g_line_follow_cfg.intersection_confirm_frames, 1.0f, 10.0f);
    g_line_follow_cfg.intersection_lock_s = lf_clampf(g_line_follow_cfg.intersection_lock_s, 0.0f, 2.0f);
    g_line_follow_cfg.intersection_hit_min_edge = (uint8_t)lf_clampf((float)g_line_follow_cfg.intersection_hit_min_edge, 1.0f, (float)LINE_FOLLOW_CH_COUNT);

    g_line_follow_cfg.delta_rpm_limit = lf_clampf(g_line_follow_cfg.delta_rpm_limit, 0.0f, 3000.0f);
    g_line_follow_cfg.turn_min_yaw_deg = lf_clampf(g_line_follow_cfg.turn_min_yaw_deg, 0.0f, 180.0f);
    g_line_follow_cfg.turn_min_time_s = lf_clampf(g_line_follow_cfg.turn_min_time_s, 0.0f, 5.0f);
    g_line_follow_cfg.turn_done_yaw_deg_l = lf_clampf(g_line_follow_cfg.turn_done_yaw_deg_l, 0.0f, 180.0f);
    g_line_follow_cfg.turn_done_yaw_deg_r = lf_clampf(g_line_follow_cfg.turn_done_yaw_deg_r, 0.0f, 180.0f);
    g_line_follow_cfg.turn_timeout_s = lf_clampf(g_line_follow_cfg.turn_timeout_s, 0.0f, 10.0f);
    if (g_line_follow_cfg.turn_min_time_s > g_line_follow_cfg.turn_timeout_s)
    {
        g_line_follow_cfg.turn_min_time_s = g_line_follow_cfg.turn_timeout_s;
    }
    if (g_line_follow_cfg.turn_done_yaw_deg_l < g_line_follow_cfg.turn_min_yaw_deg)
    {
        g_line_follow_cfg.turn_done_yaw_deg_l = g_line_follow_cfg.turn_min_yaw_deg;
    }
    if (g_line_follow_cfg.turn_done_yaw_deg_r < g_line_follow_cfg.turn_min_yaw_deg)
    {
        g_line_follow_cfg.turn_done_yaw_deg_r = g_line_follow_cfg.turn_min_yaw_deg;
    }
    g_line_follow_cfg.turn_yaw_sign = line_follow_unit_sign(g_line_follow_cfg.turn_yaw_sign, 1.0f);
    g_line_follow_cfg.turn_output_sign = line_follow_unit_sign(g_line_follow_cfg.turn_output_sign, -1.0f);
    g_line_follow_cfg.route_finish_coast_mm = lf_clampf(g_line_follow_cfg.route_finish_coast_mm, 0.0f, 1000.0f);

    g_steer_pid.kp = g_line_follow_cfg.steer_kp;
    g_steer_pid.ki = 0.0f;
    g_steer_pid.kd = g_line_follow_cfg.steer_kd;
}

void line_follow_reset(void)
{
    line_follow_runtime_reset();
}

void line_follow_on_distance_complete(void)
{
    if (g_fork_turn_pending != 0u)
    {
        if (g_fork_turn_pending == 1u)
        {
            g_lf_runtime.pending_turn = ROUTE_TURN_LEFT;
            line_follow_enter_pre_turn(ROUTE_TURN_LEFT);
        }
        else
        {
            g_lf_runtime.pending_turn = ROUTE_TURN_RIGHT;
            line_follow_enter_pre_turn(ROUTE_TURN_RIGHT);
        }
        g_fork_turn_pending = 0u;
        return;
    }

    if (route_plan_has_next())
    {
        line_follow_reset_follow_transients();
        return;
    }

    (void)line_follow_stop_if_route_done();
}

bool line_follow_control_ready(void)
{
    return (bool)(g_lf_calib_inited && g_lf_calib_valid);
}

void line_follow_get_junction_debug(line_follow_junction_debug_t *out)
{
    if (out != NULL)
    {
        *out = g_lf_junc_dbg;
    }
}

static void line_follow_calib_update_raw(const uint16_t *raw)
{
    if ((!g_lf_calib_active) || (raw == NULL))
    {
        return;
    }

    if (!g_lf_calib_inited)
    {
        for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
        {
            g_lf_calib_min[i] = raw[i];
            g_lf_calib_max[i] = raw[i];
            g_lf_calib_thr[i] = raw[i];
        }
        g_lf_calib_inited = 1u;
        g_lf_calib_samples = 0u;
        return;
    }

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        if (raw[i] < g_lf_calib_min[i]) g_lf_calib_min[i] = raw[i];
        if (raw[i] > g_lf_calib_max[i]) g_lf_calib_max[i] = raw[i];
        g_lf_calib_thr[i] = (uint16_t)(((uint32_t)g_lf_calib_min[i] * 65u + (uint32_t)g_lf_calib_max[i] * 35u) / 100u);
    }

    g_lf_calib_samples++;
}

void line_follow_calib_start(void)
{
    g_lf_calib_active = 1u;
    g_lf_calib_valid = 0u;
    g_lf_calib_inited = 0u;
    g_lf_calib_samples = 0u;
}

bool line_follow_calib_stop(void)
{
    uint8_t good_count = 0u;

    g_lf_calib_active = 0u;

    if (!g_lf_calib_inited)
    {
        g_lf_calib_valid = 0u;
        return false;
    }

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        if ((g_lf_calib_max[i] >= g_lf_calib_min[i]) && ((g_lf_calib_max[i] - g_lf_calib_min[i]) >= LF_CALIB_MIN_SPAN))
        {
            good_count++;
        }
    }

    g_lf_calib_valid = (uint8_t)(good_count >= LF_CALIB_MIN_GOOD_CH);

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        uint16_t min_value = g_lf_calib_min[i];
        uint16_t max_value = g_lf_calib_max[i];

        if ((max_value > min_value) && ((max_value - min_value) >= LF_CALIB_MIN_SPAN))
        {
            g_lf_calib_thr[i] = (uint16_t)(((uint32_t)min_value * 65u + (uint32_t)max_value * 35u) / 100u);
        }
        else
        {
            g_lf_calib_thr[i] = 2048u;
            g_lf_calib_min[i] = 500u;
            g_lf_calib_max[i] = 3500u;
        }
    }

    return (bool)g_lf_calib_valid;
}

void line_follow_calib_clear(void)
{
    g_lf_calib_active = 0u;
    g_lf_calib_valid = 0u;
    g_lf_calib_inited = 0u;
    g_lf_calib_samples = 0u;

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        g_lf_calib_min[i] = 0u;
        g_lf_calib_max[i] = 0u;
        g_lf_calib_thr[i] = 0u;
    }
}

bool line_follow_calib_is_active(void)
{
    return (bool)g_lf_calib_active;
}

bool line_follow_calib_is_valid(void)
{
    return (bool)g_lf_calib_valid;
}

uint32_t line_follow_calib_get_samples(void)
{
    return g_lf_calib_samples;
}

void line_follow_calib_get_minmax(uint16_t *min_out, uint16_t *max_out, uint8_t len)
{
    if ((min_out == NULL) || (max_out == NULL) || (len != (uint8_t)LINE_FOLLOW_CH_COUNT))
    {
        return;
    }

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        min_out[i] = g_lf_calib_min[i];
        max_out[i] = g_lf_calib_max[i];
    }
}

void line_follow_calib_set_minmax(const uint16_t *min_in, const uint16_t *max_in, uint8_t len, bool valid)
{
    if ((min_in == NULL) || (max_in == NULL) || (len != (uint8_t)LINE_FOLLOW_CH_COUNT))
    {
        return;
    }

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        uint16_t min_value = min_in[i];
        uint16_t max_value = max_in[i];

        if (max_value < min_value)
        {
            uint16_t temp = min_value;
            min_value = max_value;
            max_value = temp;
        }

        g_lf_calib_min[i] = min_value;
        g_lf_calib_max[i] = max_value;
        g_lf_calib_thr[i] = (uint16_t)(((uint32_t)min_value * 65u + (uint32_t)max_value * 35u) / 100u);
    }

    g_lf_calib_active = 0u;
    g_lf_calib_inited = 1u;
    g_lf_calib_valid = (uint8_t)(valid ? 1u : 0u);
}

void line_follow_calib_get_thresholds(uint16_t *thr_out, uint8_t len)
{
    if ((thr_out == NULL) || (len != (uint8_t)LINE_FOLLOW_CH_COUNT))
    {
        return;
    }

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        thr_out[i] = g_lf_calib_thr[i];
    }
}

void line_follow_calib_set_thresholds(const uint16_t *thr_in, uint8_t len, bool valid)
{
    if ((thr_in == NULL) || (len != (uint8_t)LINE_FOLLOW_CH_COUNT))
    {
        return;
    }

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        g_lf_calib_thr[i] = thr_in[i];
    }

    g_lf_calib_active = 0u;
    g_lf_calib_inited = 1u;
    g_lf_calib_valid = (uint8_t)(valid ? 1u : 0u);
}

void line_follow_get_last_raw(uint16_t *raw_out, uint8_t len)
{
    if ((raw_out == NULL) || (len != (uint8_t)LINE_FOLLOW_CH_COUNT))
    {
        return;
    }

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        raw_out[i] = g_lf_last_raw[i];
    }
}

bool line_follow_get_last_pos(float *pos_out)
{
    if (pos_out == NULL)
    {
        return false;
    }

    *pos_out = g_lf_last_pos;
    return (bool)g_lf_last_pos_valid;
}

uint16_t line_follow_get_last_bits(void)
{
    return g_lf_last_bits;
}

bool line_follow_is_explicit_turn_active(void)
{
    return (bool)((g_lf_runtime.state == LF_STATE_TURN_LEFT_90)
               || (g_lf_runtime.state == LF_STATE_TURN_RIGHT_90));
}

uint8_t line_follow_get_junction_lock_cnt(void)
{
    return g_junction_lock_complete_cnt;
}

uint8_t line_follow_get_element_enter_cnt(void)
{
    return g_element_enter_cnt;
}

uint8_t line_follow_get_junction_enter_cnt(void)
{
    return g_junction_enter_cnt;
}

uint16_t line_follow_get_frame_cnt(void)
{
    return g_lf_frame_cnt;
}

static void line_follow_apply_delta(float delta_rpm)
{
    float clamped = lf_clampf(delta_rpm, -g_line_follow_cfg.delta_rpm_limit, g_line_follow_cfg.delta_rpm_limit);
    g_turn_delta_rpm = clamped;
    g_lf_runtime.last_delta_rpm = clamped;
}

static float line_follow_pick_search_sign(void)
{
    if (g_lf_runtime.last_delta_rpm > 1e-3f)
    {
        return 1.0f;
    }
    if (g_lf_runtime.last_delta_rpm < -1e-3f)
    {
        return -1.0f;
    }
    if (g_lf_runtime.last_seen_pos > 1e-3f)
    {
        return 1.0f;
    }
    if (g_lf_runtime.last_seen_pos < -1e-3f)
    {
        return -1.0f;
    }
    return 1.0f;
}

static void line_follow_lock_intersection(void)
{
    g_lf_runtime.intersection_cnt = 0u;
    g_lf_runtime.intersection_lock_s = g_line_follow_cfg.intersection_lock_s;
    if (g_junction_lock_complete_cnt < 255u)
    {
        g_junction_lock_complete_cnt++;
    }
}

static void line_follow_update_intersection_lock(float dt)
{
    if (g_lf_runtime.intersection_lock_s <= 0.0f)
    {
        return;
    }

    g_lf_runtime.intersection_lock_s -= dt;
    if (g_lf_runtime.intersection_lock_s < 0.0f)
    {
        g_lf_runtime.intersection_lock_s = 0.0f;
    }
    g_lf_runtime.intersection_cnt = 0u;
}

static void line_follow_begin_turn(lf_state_t state, route_turn_t turn, float start_mm)
{
    g_lf_runtime.state = state;
    g_lf_runtime.pending_turn = turn;
    g_lf_runtime.turn_start_mm = start_mm;
    g_lf_runtime.turn_yaw0_deg = g_att_yaw_deg;
    g_lf_runtime.turn_time_s = 0.0f;
    g_lf_runtime.corner_cnt_l = 0u;
    g_lf_runtime.corner_cnt_r = 0u;
    g_lf_runtime.lost_line_cnt = 0u;
    g_lf_runtime.lost_line_time_s = 0.0f;
    line_follow_reset_steer_pid();
}

static void line_follow_enter_pre_turn(route_turn_t turn)
{
    g_lf_runtime.pending_turn = turn;
    g_lf_runtime.pre_turn_start_mm = move_distance_odom_get_traveled_mm();
    g_lf_runtime.state = LF_STATE_PRE_TURN;
    line_follow_apply_delta(0.0f);
}

static float line_follow_get_route_speed_rpm(void)
{
    float speed_rpm = fabsf(g_base_speed_rpm);

    if (speed_rpm < 1.0f)
    {
        speed_rpm = 120.0f;
    }

    return speed_rpm;
}

static void line_follow_start_distance_element(const route_plan_entry_t *entry)
{
    float speed_rpm;

    if ((entry == NULL) || (entry->turn != ROUTE_ELEM_FIXED_DIST))
    {
        return;
    }

    if ((entry->distance_mm <= 0.0f) || move_distance_is_active())
    {
        return;
    }

    speed_rpm = line_follow_get_route_speed_rpm();

    if (entry->distance_mode == MOVE_DISTANCE_MODE_LINE_FOLLOW)
    {
        move_distance_start_mm_line_follow(speed_rpm, entry->distance_mm);
    }
    else
    {
        move_distance_start_mm(speed_rpm, entry->distance_mm);
        move_distance_set_mode(MOVE_DISTANCE_MODE_STRAIGHT_LF);
    }
}

static uint8_t line_follow_stop_if_route_done(void)
{
    if (route_plan_has_next())
    {
        return 0u;
    }

    g_stop_reason = SYSTEM_STOP_REASON_ROUTE_DONE;

    if (move_distance_is_active())
    {
        line_follow_apply_delta(0.0f);
        return 1u;
    }

    if (g_line_follow_cfg.route_finish_coast_mm > 0.0f)
    {
        float coast_speed_rpm = fabsf(g_base_speed_rpm);

        if (coast_speed_rpm < 1.0f)
        {
            coast_speed_rpm = 120.0f;
        }

        if (!move_distance_is_active())
        {
            move_distance_start_mm(coast_speed_rpm, g_line_follow_cfg.route_finish_coast_mm);
        }

        line_follow_apply_delta(0.0f);
        return 1u;
    }

    g_run_enable = 0u;
    g_target_left_rpm = 0.0f;
    g_target_right_rpm = 0.0f;
    line_follow_apply_delta(0.0f);
    return 1u;
}

static uint8_t line_follow_handle_junction_event(const route_plan_entry_t *entry, const line_follow_frame_t *frame)
{
    if ((entry == NULL) || (frame == NULL))
    {
        return 0u;
    }
    (void)frame;

    line_follow_lock_intersection();
    if (g_junction_enter_cnt < 255u)
    {
        g_junction_enter_cnt++;
    }

    if (entry->turn == ROUTE_TURN_RIGHT)
    {
        line_follow_enter_pre_turn(ROUTE_TURN_RIGHT);
        return 1u;
    }
    if (entry->turn == ROUTE_TURN_LEFT)
    {
        line_follow_enter_pre_turn(ROUTE_TURN_LEFT);
        return 1u;
    }

    // ROUTE_TURN_STRAIGHT: 路线已提交，返回 0 让调用方继续执行本帧 PID 控制，
    // 而非跳过后半段。intersection_lock_s 已在上方设置，防止立即重复检测。
    g_lf_runtime.pending_turn = ROUTE_TURN_STRAIGHT;
    (void)route_plan_commit();
    return line_follow_stop_if_route_done();
}

static uint8_t line_follow_handle_element_event(const route_plan_entry_t *entry, const line_follow_frame_t *frame)
{
    uint8_t elem_index;

    if ((entry == NULL) || (frame == NULL))
    {
        return 0u;
    }
    (void)frame;

    elem_index = route_plan_get_element_index(entry->turn);
    if ((elem_index == 0u) || (elem_index > ROUTE_ELEM_COUNT))
    {
        return 0u;
    }

    if ((entry->turn == ROUTE_ELEM_DIODE_FORK_L_TURN) || (entry->turn == ROUTE_ELEM_DIODE_FORK_R_TURN)
        || (entry->turn == ROUTE_ELEM_DIODE2_FORK_L_TURN) || (entry->turn == ROUTE_ELEM_DIODE2_FORK_R_TURN)
        || (entry->turn == ROUTE_ELEM_DIODE3_FORK_L_TURN) || (entry->turn == ROUTE_ELEM_DIODE3_FORK_R_TURN))
    {
        const line_follow_elem_trigger_cfg_t *cfg = &g_lf_elem_trigger_cfg[elem_index - 1u];

        line_follow_lock_intersection();
        if (g_junction_enter_cnt < 255u)
        {
            g_junction_enter_cnt++;
        }
        if ((cfg->distance_mm > 0.0f) && !move_distance_is_active())
        {
            g_fork_turn_pending = ((entry->turn == ROUTE_ELEM_DIODE_FORK_L_TURN) || (entry->turn == ROUTE_ELEM_DIODE2_FORK_L_TURN) || (entry->turn == ROUTE_ELEM_DIODE3_FORK_L_TURN)) ? 1u : 2u;
            move_distance_start_mm(line_follow_get_route_speed_rpm(), cfg->distance_mm);
            move_distance_set_mode(MOVE_DISTANCE_MODE_STRAIGHT_LF);
        }
        else
        {
            // 无定距或定距已在运行：直接执行转弯
            route_turn_t actual_turn = ((entry->turn == ROUTE_ELEM_DIODE_FORK_L_TURN) || (entry->turn == ROUTE_ELEM_DIODE2_FORK_L_TURN) || (entry->turn == ROUTE_ELEM_DIODE3_FORK_L_TURN))
                                       ? ROUTE_TURN_LEFT : ROUTE_TURN_RIGHT;
            (void)route_plan_commit();
            line_follow_enter_pre_turn(actual_turn);
        }
        return 1u;
    }

    line_follow_lock_intersection();
    if (g_junction_enter_cnt < 255u)
    {
        g_junction_enter_cnt++;
    }
    if (g_element_enter_cnt < 255u)
    {
        g_element_enter_cnt++;
    }
    g_lf_runtime.pending_turn = entry->turn;
    (void)route_plan_commit();

    if (entry->turn == ROUTE_ELEM_FIXED_DIST)
    {
        line_follow_start_distance_element(entry);
    }
    else
    {
        const line_follow_elem_trigger_cfg_t *cfg = &g_lf_elem_trigger_cfg[elem_index - 1u];

        if ((cfg->distance_mm > 0.0f) && !move_distance_is_active())
        {
            move_distance_start_mm(line_follow_get_route_speed_rpm(), cfg->distance_mm);
            move_distance_set_mode(MOVE_DISTANCE_MODE_STRAIGHT_LF);
        }
    }

    line_follow_apply_delta(0.0f);
    (void)line_follow_stop_if_route_done();
    return 1u;
}

static void line_follow_finish_turn(uint8_t commit_route)
{
    if (commit_route && (g_lf_runtime.pending_turn != ROUTE_TURN_STRAIGHT))
    {
        (void)route_plan_commit();
        (void)line_follow_stop_if_route_done();
    }

    g_lf_runtime.state = LF_STATE_FOLLOW;
    g_lf_runtime.pending_turn = ROUTE_TURN_STRAIGHT;
    g_lf_runtime.turn_time_s = 0.0f;
    g_lf_runtime.lost_line_cnt = 0u;
    g_lf_runtime.lost_line_time_s = 0.0f;
    g_lf_runtime.element_confirm_cnt = 0u;
    line_follow_reset_steer_pid();
    line_follow_apply_delta(0.0f);
}

static void line_follow_sample_frame(line_follow_frame_t *frame, float dt)
{
    if (frame == NULL)
    {
        return;
    }

#if (LINE_FOLLOW_CH_COUNT == GRAY_4067_DEFAULT_CHANNEL_COUNT)
    gray_4067_read_12_raw(frame->raw);
#else
    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        frame->raw[i] = gray_4067_read_raw((uint8_t)(GRAY_4067_CH12_START_CHANNEL + i));
    }
#endif

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        g_lf_last_raw[i] = frame->raw[i];
    }

    line_follow_calib_update_raw(frame->raw);

    frame->bits = 0u;
    frame->hit_count = 0u;
    frame->pos = 0.0f;
    frame->pos_valid = 0u;
    frame->left_edge = 0u;
    frame->right_edge = 0u;
    frame->near_center = 0u;
    frame->wide_hit = 0u;
    frame->both_edge = 0u;
    frame->edge_branch = 0u;
    frame->looks_like_intersection = 0u;
    frame->feature_flags = 0u;

    if (!g_lf_calib_inited)
    {
        g_lf_last_bits = 0u;
        line_follow_reset_position_filter();
        return;
    }

    for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
    {
        uint16_t threshold = g_lf_calib_thr[i];
        uint8_t hit;

        frame->scores[i] = line_follow_calc_channel_score(frame->raw[i], g_lf_calib_min[i], g_lf_calib_max[i]);

        if (LINE_FOLLOW_INVERT)
        {
            hit = (uint8_t)(frame->raw[i] > threshold);
        }
        else
        {
            hit = (uint8_t)(frame->raw[i] < threshold);
        }

        if (hit != 0u)
        {
            frame->bits |= (uint16_t)(1u << i);
            frame->hit_count++;
        }
    }

    g_lf_last_bits = frame->bits;

    {
        float pos_meas = 0.0f;
        uint8_t pos_meas_valid = line_follow_estimate_local_peak_pos(frame->scores, &pos_meas);

        if (pos_meas_valid)
        {
            if (g_lf_runtime.last_pos_meas_valid)
            {
                float pos_rate_meas = (pos_meas - g_lf_runtime.last_pos_meas) / dt;
                g_lf_runtime.pos_rate_filt += LF_POS_D_LP_ALPHA * (pos_rate_meas - g_lf_runtime.pos_rate_filt);
            }
            else
            {
                g_lf_runtime.pos_rate_filt = 0.0f;
            }

            g_lf_runtime.last_pos_meas = pos_meas;
            g_lf_runtime.last_pos_meas_valid = 1u;

            g_lf_runtime.pos_filt += LF_POS_LP_ALPHA * (pos_meas - g_lf_runtime.pos_filt);
            frame->pos = g_lf_runtime.pos_filt;
            frame->pos_valid = 1u;
            g_lf_last_pos = frame->pos;
            g_lf_last_pos_valid = 1u;
            g_lf_runtime.last_seen_pos = frame->pos;
        }
        else
        {
            line_follow_reset_position_filter();
        }
    }

#if LINE_FOLLOW_REVERSE_CHANNEL_ORDER
    frame->left_edge = (uint8_t)(((frame->bits >> (LINE_FOLLOW_CH_COUNT - 1u)) & 0x0001u) != 0u);
    frame->right_edge = (uint8_t)((frame->bits & 0x0001u) != 0u);
#else
    frame->left_edge = (uint8_t)((frame->bits & 0x0001u) != 0u);
    frame->right_edge = (uint8_t)(((frame->bits >> (LINE_FOLLOW_CH_COUNT - 1u)) & 0x0001u) != 0u);
#endif
    frame->near_center = (uint8_t)(frame->pos_valid && (fabsf(frame->pos) <= g_line_follow_cfg.intersection_pos_max));
    frame->wide_hit = (uint8_t)(frame->hit_count >= g_line_follow_cfg.intersection_hit_min);
    frame->both_edge = (uint8_t)(frame->left_edge && frame->right_edge);
    frame->edge_branch = (uint8_t)(((frame->left_edge || frame->right_edge) != 0u)
                                && (frame->hit_count >= g_line_follow_cfg.intersection_hit_min_edge));
    frame->looks_like_intersection = (uint8_t)(frame->near_center
                                && (frame->wide_hit || frame->both_edge || frame->edge_branch));
    frame->feature_flags = line_follow_build_feature_flags(frame);
}

static void line_follow_decision_clear(line_follow_decision_t *decision)
{
    if (decision != NULL)
    {
        decision->apply_delta = 0u;
        decision->hold_last_delta = 0u;
        decision->delta_rpm = 0.0f;
    }
}

static void line_follow_decision_set_delta(line_follow_decision_t *decision, float delta_rpm)
{
    if (decision != NULL)
    {
        decision->apply_delta = 1u;
        decision->hold_last_delta = 0u;
        decision->delta_rpm = delta_rpm;
    }
}

static void line_follow_decision_hold_last(line_follow_decision_t *decision)
{
    if (decision != NULL)
    {
        decision->apply_delta = 0u;
        decision->hold_last_delta = 1u;
        decision->delta_rpm = 0.0f;
    }
}

static void line_follow_commit_decision(const line_follow_decision_t *decision)
{
    if (decision == NULL)
    {
        return;
    }

    if (decision->hold_last_delta)
    {
        g_turn_delta_rpm = g_lf_runtime.last_delta_rpm;
    }
    else if (decision->apply_delta)
    {
        line_follow_apply_delta(decision->delta_rpm);
    }
}

static uint8_t line_follow_handle_pre_turn_stage(float dt, line_follow_decision_t *decision)
{
    float traveled_mm;

    if (g_lf_runtime.state != LF_STATE_PRE_TURN)
    {
        return 0u;
    }

    traveled_mm = fabsf(move_distance_odom_get_traveled_mm() - g_lf_runtime.pre_turn_start_mm);
    if (traveled_mm < g_line_follow_cfg.pre_turn_distance_mm)
    {
        line_follow_decision_set_delta(decision, 0.0f);
        return 1u;
    }

    if (g_lf_runtime.pending_turn == ROUTE_TURN_RIGHT)
    {
        line_follow_begin_turn(LF_STATE_TURN_RIGHT_90, ROUTE_TURN_RIGHT, move_distance_odom_get_traveled_mm());
    }
    else if (g_lf_runtime.pending_turn == ROUTE_TURN_LEFT)
    {
        line_follow_begin_turn(LF_STATE_TURN_LEFT_90, ROUTE_TURN_LEFT, move_distance_odom_get_traveled_mm());
    }
    else
    {
        g_lf_runtime.state = LF_STATE_FOLLOW;
        g_lf_runtime.pending_turn = ROUTE_TURN_STRAIGHT;
        g_lf_runtime.turn_time_s = 0.0f;
        g_lf_runtime.lost_line_cnt = 0u;
        g_lf_runtime.lost_line_time_s = 0.0f;
        line_follow_reset_steer_pid();
    }

    return 0u;
}

static uint8_t line_follow_turn_exit_line_ready(const line_follow_frame_t *frame)
{
    if (frame == NULL)
    {
        return 0u;
    }

    if ((!frame->pos_valid) || (frame->hit_count == 0u))
    {
        return 0u;
    }

    if (frame->looks_like_intersection || frame->both_edge || frame->wide_hit)
    {
        return 0u;
    }

    if (fabsf(frame->pos) > g_line_follow_cfg.intersection_pos_max)
    {
        return 0u;
    }

    return 1u;
}

static uint8_t line_follow_handle_turn_stage(const line_follow_frame_t *frame, float dt, line_follow_decision_t *decision)
{
    float signed_yaw_delta;
    float abs_yaw_delta;
    float turn_odom_mm;
    uint8_t guard_ready;
    uint8_t line_exit_ready;
    float turn_dir_sign;

    if ((g_lf_runtime.state != LF_STATE_TURN_LEFT_90) && (g_lf_runtime.state != LF_STATE_TURN_RIGHT_90))
        return 0u;

    g_lf_runtime.turn_time_s += dt;

    signed_yaw_delta = line_follow_get_signed_turn_yaw_delta_deg();
    abs_yaw_delta = fabsf(lf_wrap_deg(g_att_yaw_deg - g_lf_runtime.turn_yaw0_deg));
    turn_odom_mm = fabsf(move_distance_odom_get_traveled_mm() - g_lf_runtime.turn_start_mm);
    line_exit_ready = line_follow_turn_exit_line_ready(frame);

    if (g_lf_runtime.state == LF_STATE_TURN_LEFT_90)
    {
        guard_ready = (uint8_t)((g_lf_runtime.turn_time_s >= g_line_follow_cfg.turn_min_time_s)
                             && (signed_yaw_delta >= g_line_follow_cfg.turn_min_yaw_deg));

        if (guard_ready && (signed_yaw_delta >= g_line_follow_cfg.turn_done_yaw_deg_l))
        {
            line_follow_finish_turn(1u);
            line_follow_decision_set_delta(decision, 0.0f);
            return 1u;
        }
    }
    else
    {
        guard_ready = (uint8_t)((g_lf_runtime.turn_time_s >= g_line_follow_cfg.turn_min_time_s)
                             && (signed_yaw_delta <= (-g_line_follow_cfg.turn_min_yaw_deg)));

        if (guard_ready && (signed_yaw_delta <= (-g_line_follow_cfg.turn_done_yaw_deg_r)))
        {
            line_follow_finish_turn(1u);
            line_follow_decision_set_delta(decision, 0.0f);
            return 1u;
        }
    }

    if ((g_lf_runtime.turn_time_s >= g_line_follow_cfg.turn_min_time_s)
        && (abs_yaw_delta >= ((g_lf_runtime.state == LF_STATE_TURN_LEFT_90) ? g_line_follow_cfg.turn_done_yaw_deg_l : g_line_follow_cfg.turn_done_yaw_deg_r))
        && line_exit_ready)
    {
        line_follow_finish_turn(1u);
        line_follow_decision_set_delta(decision, 0.0f);
        return 1u;
    }

    if ((g_lf_runtime.turn_time_s >= g_line_follow_cfg.turn_timeout_s)
        || (turn_odom_mm >= LF_TURN_MAX_ODOM_MM))
    {
        line_follow_finish_turn(0u);
        line_follow_decision_set_delta(decision, 0.0f);
        return 1u;
    }

    turn_dir_sign = (g_lf_runtime.state == LF_STATE_TURN_LEFT_90) ? 1.0f : -1.0f;
    line_follow_decision_set_delta(
        decision,
        turn_dir_sign * (-g_line_follow_cfg.turn_output_sign) * g_line_follow_cfg.delta_rpm_limit
    );
    return 1u;
}

static uint8_t line_follow_handle_follow_stage(const line_follow_frame_t *frame, float dt, line_follow_decision_t *decision)
{
    float search_delta;
    float err;

    if ((frame == NULL) || (g_lf_runtime.state != LF_STATE_FOLLOW))
    {
        return 0u;
    }

    if (frame->hit_count == 0u)
    {
        if (g_lf_runtime.lost_line_cnt < 255u)
        {
            g_lf_runtime.lost_line_cnt++;
        }
        g_lf_runtime.lost_line_time_s += dt;

        if (g_lf_runtime.lost_line_cnt <= LF_LOST_LINE_HOLD_FRAMES)
        {
            line_follow_decision_hold_last(decision);
            return 1u;
        }

        // 丢线不直接干预编码器里程；搜索/纠偏期间只要轮子仍在转，里程仍继续累计。
        // 这里仅决定循迹控制策略，以及在超时后以 LOST_LINE 原因停机。
        search_delta = line_follow_pick_search_sign() * LF_LOST_LINE_SEARCH_RPM;
        line_follow_decision_set_delta(decision, search_delta);
        if (g_lf_runtime.lost_line_time_s < LF_LOST_LINE_STOP_S)
        {
            return 1u;
        }

        g_stop_reason = SYSTEM_STOP_REASON_LOST_LINE;
        g_run_enable = 0u;
        g_target_left_rpm = 0.0f;
        g_target_right_rpm = 0.0f;
        line_follow_runtime_reset();
        line_follow_decision_set_delta(decision, 0.0f);
        return 1u;
    }

    g_lf_runtime.lost_line_cnt = 0u;
    g_lf_runtime.lost_line_time_s = 0.0f;

    // 命中了若干通道但没有得到可靠位置时，不能把 pos=0 当成”线在中心”。
    // 否则上层会看到 line error 被压成 0，控制也会错误地继续直行。
    if (!frame->pos_valid)
    {
        line_follow_decision_hold_last(decision);
        return 1u;
    }

    line_follow_update_intersection_lock(dt);

    if (!move_distance_is_active())
    {
        if (g_lf_runtime.intersection_lock_s <= 0.0f)
        {
            if (route_plan_has_next())
            {
                route_plan_entry_t next_entry;

                if (route_plan_peek(&next_entry))
                {
                    if (route_plan_is_element(next_entry.turn))
                    {
                        if (line_follow_match_element_trigger(next_entry.turn, frame))
                        {
                            if (g_lf_runtime.element_confirm_cnt < 255u)
                            {
                                g_lf_runtime.element_confirm_cnt++;
                            }
                        }
                        else
                        {
                            g_lf_runtime.element_confirm_cnt = 0u;
                        }

                        {
                            uint8_t elem_index = route_plan_get_element_index(next_entry.turn);

                            if ((elem_index > 0u) && (elem_index <= ROUTE_ELEM_COUNT)
                                && (g_lf_runtime.element_confirm_cnt >= g_lf_elem_trigger_cfg[elem_index - 1u].min_confirm_cnt))
                            {
                                if (line_follow_handle_element_event(&next_entry, frame))
                                {
                                    g_lf_runtime.element_confirm_cnt = 0u;
                                    line_follow_decision_set_delta(decision, 0.0f);
                                    return 1u;
                                }
                            }
                        }
                    }
                    else
                    {
                        if (frame->looks_like_intersection)
                        {
                            if (g_lf_runtime.intersection_cnt < 255u)
                            {
                                g_lf_runtime.intersection_cnt++;
                            }
                        }
                        else
                        {
                            g_lf_runtime.intersection_cnt = 0u;
                        }

                        if (g_lf_runtime.intersection_cnt >= g_line_follow_cfg.intersection_confirm_frames)
                        {
                            if (line_follow_handle_junction_event(&next_entry, frame))
                            {
                                line_follow_decision_set_delta(decision, 0.0f);
                                return 1u;
                            }
                        }
                    }
                }
            }
        }

#if !LINE_FOLLOW_ROUTE_PLAN_ONLY
        if (frame->right_edge && (frame->hit_count <= LF_CORNER_HIT_MAX) && (frame->pos >= LF_CORNER_POS_TH))
        {
            if (g_lf_runtime.corner_cnt_r < 255u) g_lf_runtime.corner_cnt_r++;
        }
        else
        {
            g_lf_runtime.corner_cnt_r = 0u;
        }

        if (frame->left_edge && (frame->hit_count <= LF_CORNER_HIT_MAX) && (frame->pos <= -LF_CORNER_POS_TH))
        {
            if (g_lf_runtime.corner_cnt_l < 255u) g_lf_runtime.corner_cnt_l++;
        }
        else
        {
            g_lf_runtime.corner_cnt_l = 0u;
        }

        if (g_lf_runtime.corner_cnt_r >= LF_CORNER_CONFIRM_FRAMES)
        {
            line_follow_begin_turn(LF_STATE_TURN_RIGHT_90, ROUTE_TURN_STRAIGHT, move_distance_odom_get_traveled_mm());
            line_follow_decision_set_delta(decision, 0.0f);
            return 1u;
        }
        if (g_lf_runtime.corner_cnt_l >= LF_CORNER_CONFIRM_FRAMES)
        {
            line_follow_begin_turn(LF_STATE_TURN_LEFT_90, ROUTE_TURN_STRAIGHT, move_distance_odom_get_traveled_mm());
            line_follow_decision_set_delta(decision, 0.0f);
            return 1u;
        }
#endif
    }
    else
    {
        g_lf_runtime.intersection_cnt = 0u;
        g_lf_runtime.element_confirm_cnt = 0u;
        g_lf_runtime.corner_cnt_l = 0u;
        g_lf_runtime.corner_cnt_r = 0u;
    }

    err = frame->pos;
    g_steer_pid.last_error = g_steer_pid.error;
    g_steer_pid.error = err;
    g_steer_pid.integ = 0.0f;

    line_follow_decision_set_delta(
        decision,
        line_follow_calc_follow_delta_rpm(err, g_lf_runtime.pos_rate_filt, g_gyro_z_dps_f)
    );
    return 1u;
}

static void line_follow_update_debug(const line_follow_frame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    g_lf_junc_dbg.bits = frame->bits;
    g_lf_junc_dbg.hit_count = frame->hit_count;
    g_lf_junc_dbg.pos = frame->pos;
    g_lf_junc_dbg.pos_valid = (bool)frame->pos_valid;
    g_lf_junc_dbg.left_edge = (bool)frame->left_edge;
    g_lf_junc_dbg.right_edge = (bool)frame->right_edge;
    g_lf_junc_dbg.near_center = (bool)frame->near_center;
    g_lf_junc_dbg.wide_hit = (bool)frame->wide_hit;
    g_lf_junc_dbg.both_edge = (bool)frame->both_edge;
    g_lf_junc_dbg.edge_branch = (bool)frame->edge_branch;
    g_lf_junc_dbg.looks_like_isect = (bool)frame->looks_like_intersection;
    g_lf_junc_dbg.isect_cnt = g_lf_runtime.intersection_cnt;
    g_lf_junc_dbg.isect_lock_s = g_lf_runtime.intersection_lock_s;
    g_lf_junc_dbg.lf_state = (uint8_t)g_lf_runtime.state;
    g_lf_junc_dbg.pending_turn = g_lf_runtime.pending_turn;
    g_lf_junc_dbg.yaw_delta_deg = line_follow_get_signed_turn_yaw_delta_deg();
    g_lf_junc_dbg.turn_time_s = g_lf_runtime.turn_time_s;
    g_lf_junc_dbg.lost_line_cnt = g_lf_runtime.lost_line_cnt;
    g_lf_junc_dbg.odom_mm = move_distance_odom_get_traveled_mm();
    g_lf_junc_dbg.preturn_start_mm = g_lf_runtime.pre_turn_start_mm;
}

void line_follow_update(float dt)
{
    line_follow_frame_t frame;
    line_follow_decision_t decision;

    g_lf_frame_cnt++;

    if (dt <= 0.0f)
    {
        return;
    }

    // 页面停留在 FOC/GRAY 调试页时，不应把主循迹逻辑整段禁掉；
    // 真正需要互斥的是“FOC 测试闭环正在接管电机”的场景。
    if ((g_menu_state == MENU_FOC_SPEED_DEBUG) && (g_foc_test_run != 0u))
    {
        line_follow_runtime_reset();
        return;
    }

    line_follow_decision_clear(&decision);
    line_follow_sample_frame(&frame, dt);

    if (!g_run_enable)
    {
        // 停机态也需要保留本周期刚采样出的 pos/bits，供板端页面与上位机调试观察。
        // 同时继续计算“观测差速”，但不下发到速度目标。
        line_follow_reset_control_state();
        if (line_follow_control_ready() && frame.pos_valid)
        {
            g_turn_delta_rpm = line_follow_calc_follow_delta_rpm(frame.pos, g_lf_runtime.pos_rate_filt, g_gyro_z_dps_f);
        }
        line_follow_update_debug(&frame);
        return;
    }

    if (!line_follow_control_ready())
    {
        line_follow_decision_set_delta(&decision, 0.0f);
        line_follow_update_debug(&frame);
        line_follow_commit_decision(&decision);
        return;
    }

    if (move_distance_is_active() && (move_distance_get_mode() != MOVE_DISTANCE_MODE_LINE_FOLLOW))
    {
        line_follow_decision_set_delta(&decision, 0.0f);
        line_follow_update_debug(&frame);
        line_follow_commit_decision(&decision);
        return;
    }

    if (line_follow_handle_pre_turn_stage(dt, &decision))
    {
        line_follow_update_debug(&frame);
        line_follow_commit_decision(&decision);
        return;
    }

    if (line_follow_handle_turn_stage(&frame, dt, &decision))
    {
        line_follow_update_debug(&frame);
        line_follow_commit_decision(&decision);
        return;
    }

    (void)line_follow_handle_follow_stage(&frame, dt, &decision);
    line_follow_update_debug(&frame);
    line_follow_commit_decision(&decision);
}
