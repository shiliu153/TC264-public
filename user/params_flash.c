/*********************************************************************************************************************
* 文件：params_flash.c
* 模块：应用层（第 6 层）—— 参数系统
* 职责：可 WiFi 配置的浮点参数系统：描述符表、取值与 Flash 持久化
* 依赖：zf_driver_flash.h、zf_common_headfile.h、本目录 params_flash.h
* 风险等级：R2
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "params_flash.h"

#include <math.h>

#include "battery_monitor.h"
#include "bldc_drv8300_3pwm.h"
#include "foc_svpwm.h"
#include "isr.h"
#include "line_follow.h"
#include "neg_pressure_motor.h"
#include "pid.h"
#include "zf_driver_flash.h"

#ifndef PARAMS_FLASH_MAGIC
#define PARAMS_FLASH_MAGIC      (0x50415234u)
#endif

#ifndef PARAMS_FLASH_SECTOR_NUM
#define PARAMS_FLASH_SECTOR_NUM (0u)
#endif

#ifndef PARAMS_FLASH_PAGE_INDEX
#define PARAMS_FLASH_PAGE_INDEX (11u)
#endif

#define PARAMS_FLASH_CRC_SLOT   (128u)

typedef enum
{
    PARAMS_IDX_MAGIC = 0,

    PARAMS_IDX_SPEED_KP_L = 1,
    PARAMS_IDX_SPEED_KI_L,
    PARAMS_IDX_SPEED_KP_R,
    PARAMS_IDX_SPEED_KI_R,

    PARAMS_IDX_VQ_LIMIT_L,
    PARAMS_IDX_VQ_LIMIT_R,

    PARAMS_IDX_BASE_SPEED_RPM,

    PARAMS_IDX_LF_KP,
    PARAMS_IDX_LF_KI,
    PARAMS_IDX_LF_KD,

    PARAMS_IDX_LF_CALIB_VALID,
    PARAMS_IDX_LF_CALIB_MIN_0,
    PARAMS_IDX_LF_CALIB_MIN_LAST = (PARAMS_IDX_LF_CALIB_MIN_0 + (LINE_FOLLOW_CH_COUNT - 1u)),
    PARAMS_IDX_LF_CALIB_MAX_0 = (PARAMS_IDX_LF_CALIB_MIN_0 + LINE_FOLLOW_CH_COUNT),
    PARAMS_IDX_LF_CALIB_MAX_LAST = (PARAMS_IDX_LF_CALIB_MAX_0 + (LINE_FOLLOW_CH_COUNT - 1u)),
    PARAMS_IDX_LF_CALIB_THR_0 = (PARAMS_IDX_LF_CALIB_MAX_0 + LINE_FOLLOW_CH_COUNT),
    PARAMS_IDX_LF_CALIB_THR_LAST = (PARAMS_IDX_LF_CALIB_THR_0 + (LINE_FOLLOW_CH_COUNT - 1u)),

    PARAMS_IDX_LF_PRETURN_MM = (PARAMS_IDX_LF_CALIB_THR_0 + LINE_FOLLOW_CH_COUNT),
    PARAMS_IDX_LF_ISECT_HIT_MIN,
    PARAMS_IDX_LF_ISECT_POS_MAX,
    PARAMS_IDX_LF_ISECT_CONFIRM_FRAMES,
    PARAMS_IDX_LF_ISECT_LOCK_S,
    PARAMS_IDX_LF_ISECT_HIT_MIN_EDGE,

    PARAMS_IDX_LF_DELTA_RPM_LIMIT,
    PARAMS_IDX_RESERVED_LF_TURN_CENTER_POS_TH,
    PARAMS_IDX_RESERVED_LF_TURN_CENTER_CONFIRM_FRAMES,
    PARAMS_IDX_LF_TURN_MIN_YAW_DEG,
    PARAMS_IDX_LF_TURN_MIN_TIME_S,
    PARAMS_IDX_LF_TURN_DONE_YAW_DEG_L,
    PARAMS_IDX_LF_TURN_TIMEOUT_S,
    PARAMS_IDX_LF_ROUTE_FINISH_COAST_MM,

    PARAMS_IDX_LF_KP2,
    PARAMS_IDX_LF_GKD,
    PARAMS_IDX_TURN_SPEED_SCALE,

    PARAMS_IDX_BATTERY_CELLS,

    PARAMS_IDX_OPEN_LOOP_HZ,
    PARAMS_IDX_OPEN_LOOP_VQ,

    PARAMS_IDX_GYRO_LP_ALPHA,

    PARAMS_IDX_AS5047P_LP_ALPHA,
    PARAMS_IDX_AS5047P_SPEED_AVG_SAMPLES,
    PARAMS_IDX_AS5047P_SPEED_ZERO_DEADBAND_RPM,
    PARAMS_IDX_LF_TURN_YAW_SIGN,
    PARAMS_IDX_LF_TURN_OUTPUT_SIGN,
    PARAMS_IDX_LF_TURN_DONE_YAW_DEG_R,
    PARAMS_IDX_STRAIGHT_BOOST_RPM,

    PARAMS_WORD_COUNT = (PARAMS_IDX_STRAIGHT_BOOST_RPM + 1),
} params_flash_index_t;

extern volatile float g_open_loop_hz;
extern volatile float g_open_loop_vq;
extern volatile uint8_t g_open_loop_run;
extern volatile float g_gyro_lp_alpha;
extern volatile float g_as5047p_lp_alpha;
extern volatile float g_as5047p_speed_avg_samples;
extern volatile float g_as5047p_speed_zero_deadband_rpm;
extern volatile float g_straight_boost_rpm;

static float params_clampf(float value, float min_value, float max_value);
static float params_sanitize_float(float value, float fallback);
static uint8_t params_ascii_equal_ignore_case(uint8_t lhs, uint8_t rhs);
static uint16_t params_page_mask(menu_state_t page);
static uint8_t params_descriptor_in_page(const params_descriptor_t *desc, menu_state_t page);
static uint8_t params_neg_pressure_run_requested(void);
static void params_apply_drive_limits(void);
static void params_apply_line_follow(void);
static void params_apply_foc_test_toggle(void);
static void params_apply_neg_pressure_enable(void);
static void params_write_flash_value(const params_descriptor_t *desc);
static void params_load_flash_value(const params_descriptor_t *desc);

static const params_descriptor_t g_params_registry[] = {
    {
        "BAS_SPD", "Base Speed", PARAMS_VALUE_FLOAT, &g_base_speed_rpm,
        0.0f, 2000.0f, 1000.0f, 1u,
        (uint16_t)((1u << MENU_RUN) | (1u << MENU_FOC_SPEED_DEBUG)),
        (uint16_t)PARAMS_IDX_BASE_SPEED_RPM, NULL
    },
    {
        "STR_BST", "Straight Boost", PARAMS_VALUE_FLOAT, &g_straight_boost_rpm,
        0.0f, 2000.0f, 10.0f, 0u,
        (uint16_t)((1u << MENU_RUN)),
        (uint16_t)PARAMS_IDX_STRAIGHT_BOOST_RPM, NULL
    },
    {
        "BAT_CEL", "Batt Cells", PARAMS_VALUE_U8, &g_battery_cells,
        2.0f, 6.0f, 2.0f, 0u,
        (uint16_t)((1u << MENU_RUN) | (1u << MENU_FOC_SPEED_DEBUG) | (1u << MENU_WIFI_CONNECT)),
        (uint16_t)PARAMS_IDX_BATTERY_CELLS, NULL
    },

    {
        "KP_L", "L Kp", PARAMS_VALUE_FLOAT, &g_loop_l.speed_pid.kp,
        0.0f, 100.0f, 0.0065f, 3u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        (uint16_t)PARAMS_IDX_SPEED_KP_L, NULL
    },
    {
        "KI_L", "L Ki", PARAMS_VALUE_FLOAT, &g_loop_l.speed_pid.ki,
        0.0f, 100.0f, 0.295f, 3u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        (uint16_t)PARAMS_IDX_SPEED_KI_L, NULL
    },
    {
        "VQL_L", "L Vq", PARAMS_VALUE_FLOAT, &g_loop_l.vq_limit,
        0.0f, 0.95f, 0.95f, 4u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        (uint16_t)PARAMS_IDX_VQ_LIMIT_L, params_apply_drive_limits
    },
    {
        "KP_R", "R Kp", PARAMS_VALUE_FLOAT, &g_loop_r.speed_pid.kp,
        0.0f, 100.0f, 0.0065f, 3u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        (uint16_t)PARAMS_IDX_SPEED_KP_R, NULL
    },
    {
        "KI_R", "R Ki", PARAMS_VALUE_FLOAT, &g_loop_r.speed_pid.ki,
        0.0f, 100.0f, 0.301f, 3u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        (uint16_t)PARAMS_IDX_SPEED_KI_R, NULL
    },
    {
        "VQL_R", "R Vq", PARAMS_VALUE_FLOAT, &g_loop_r.vq_limit,
        0.0f, 0.95f, 0.95f, 4u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        (uint16_t)PARAMS_IDX_VQ_LIMIT_R, params_apply_drive_limits
    },
#ifndef HARDWARE_STRIPPED
    {
        "A5_LP_A", "Spd LPF", PARAMS_VALUE_FLOAT, &g_as5047p_lp_alpha,
        0.001f, 1.0f, 0.4f, 3u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        (uint16_t)PARAMS_IDX_AS5047P_LP_ALPHA, NULL
    },
    {
        "A5_AVG_N", "Avg N", PARAMS_VALUE_FLOAT, &g_as5047p_speed_avg_samples,
        1.0f, 100.0f, 1.0f, 1u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        (uint16_t)PARAMS_IDX_AS5047P_SPEED_AVG_SAMPLES, NULL
    },
    {
        "A5_ZDB_R", "ZeroDB", PARAMS_VALUE_FLOAT, &g_as5047p_speed_zero_deadband_rpm,
        0.0f, 20.0f, 1.0f, 1u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        (uint16_t)PARAMS_IDX_AS5047P_SPEED_ZERO_DEADBAND_RPM, NULL
    },
#endif
    {
        "TST_RUN", "Test Run", PARAMS_VALUE_BOOL, &g_foc_test_run,
        0.0f, 1.0f, 1.0f, 0u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        PARAMS_FLASH_INDEX_NONE, params_apply_foc_test_toggle
    },
    {
        "LTST_RPM", "LTest RPM", PARAMS_VALUE_FLOAT, &g_foc_test_left_target_rpm,
        -4000.0f, 4000.0f, 20.0f, 0u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        PARAMS_FLASH_INDEX_NONE, NULL
    },
    {
        "RTST_RPM", "RTest RPM", PARAMS_VALUE_FLOAT, &g_foc_test_right_target_rpm,
        -4000.0f, 4000.0f, 20.0f, 0u,
        (uint16_t)(1u << MENU_FOC_SPEED_DEBUG),
        PARAMS_FLASH_INDEX_NONE, NULL
    },

    {
        "STR_KP", "STR Kp", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.steer_kp,
        0.0f, 100.0f, 60.0f, 2u,
        (uint16_t)(1u << MENU_LINE_FOLLOW_DEBUG),
        (uint16_t)PARAMS_IDX_LF_KP, params_apply_line_follow
    },
    {
        "STR_KD", "STR Kd", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.steer_kd,
        0.0f, 100.0f, 0.0f, 2u,
        (uint16_t)(1u << MENU_LINE_FOLLOW_DEBUG),
        (uint16_t)PARAMS_IDX_LF_KD, params_apply_line_follow
    },
    {
        "LF_KP2", "LF Kp2", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.steer_kp2,
        0.0f, 100.0f, 0.0f, 2u,
        (uint16_t)(1u << MENU_LINE_FOLLOW_DEBUG),
        (uint16_t)PARAMS_IDX_LF_KP2, params_apply_line_follow
    },
    {
        "LF_GKD", "LF Gkd", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.steer_gkd,
        0.0f, 100.0f, 0.0f, 2u,
        (uint16_t)(1u << MENU_LINE_FOLLOW_DEBUG),
        (uint16_t)PARAMS_IDX_LF_GKD, params_apply_line_follow
    },
    {
        "TRN_SCL", "TurnScale", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.turn_speed_scale,
        0.0f, 0.01f, 0.0f, 3u,
        (uint16_t)(1u << MENU_LINE_FOLLOW_DEBUG),
        (uint16_t)PARAMS_IDX_TURN_SPEED_SCALE, params_apply_line_follow
    },

    {
        "PR_TURN", "Pre(mm)", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.pre_turn_distance_mm,
        0.0f, 500.0f, 50.0f, 1u,
        (uint16_t)(1u << MENU_JUNCTION_DEBUG),
        (uint16_t)PARAMS_IDX_LF_PRETURN_MM, params_apply_line_follow
    },
    {
        "IS_HITM", "HitMin", PARAMS_VALUE_U8, &g_line_follow_cfg.intersection_hit_min,
        1.0f, (float)LINE_FOLLOW_CH_COUNT, 6.0f, 0u,
        (uint16_t)((1u << MENU_JUNCTION_DEBUG) | (1u << MENU_GRAY_DEBUG)),
        (uint16_t)PARAMS_IDX_LF_ISECT_HIT_MIN, params_apply_line_follow
    },
    {
        "IS_PMAX", "PosMax", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.intersection_pos_max,
        0.0f, 6.0f, 4.0f, 2u,
        (uint16_t)(1u << MENU_JUNCTION_DEBUG),
        (uint16_t)PARAMS_IDX_LF_ISECT_POS_MAX, params_apply_line_follow
    },
    {
        "IS_CFRM", "Cfm", PARAMS_VALUE_U8, &g_line_follow_cfg.intersection_confirm_frames,
        1.0f, 10.0f, 1.0f, 0u,
        (uint16_t)((1u << MENU_JUNCTION_DEBUG) | (1u << MENU_GRAY_DEBUG)),
        (uint16_t)PARAMS_IDX_LF_ISECT_CONFIRM_FRAMES, params_apply_line_follow
    },
    {
        "IS_LCKS", "Lock(s)", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.intersection_lock_s,
        0.0f, 2.0f, 0.02f, 2u,
        (uint16_t)(1u << MENU_JUNCTION_DEBUG),
        (uint16_t)PARAMS_IDX_LF_ISECT_LOCK_S, params_apply_line_follow
    },
    {
        "IS_HIME", "EdgeMin", PARAMS_VALUE_U8, &g_line_follow_cfg.intersection_hit_min_edge,
        1.0f, (float)LINE_FOLLOW_CH_COUNT, 4.0f, 0u,
        (uint16_t)((1u << MENU_JUNCTION_DEBUG) | (1u << MENU_GRAY_DEBUG)),
        (uint16_t)PARAMS_IDX_LF_ISECT_HIT_MIN_EDGE, params_apply_line_follow
    },
    {
        "DRPM_LM", "dLim", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.delta_rpm_limit,
        0.0f, 3000.0f, 1800.0f, 1u,
        (uint16_t)((1u << MENU_JUNCTION_DEBUG) | (1u << MENU_LINE_FOLLOW_DEBUG)),
        (uint16_t)PARAMS_IDX_LF_DELTA_RPM_LIMIT, params_apply_line_follow
    },
    {
        "T_MINYW", "MinYaw", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.turn_min_yaw_deg,
        0.0f, 180.0f, 20.0f, 1u,
        (uint16_t)((1u << MENU_JUNCTION_DEBUG) | (1u << MENU_LINE_FOLLOW_DEBUG)),
        (uint16_t)PARAMS_IDX_LF_TURN_MIN_YAW_DEG, params_apply_line_follow
    },
    {
        "T_MINTS", "MinT", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.turn_min_time_s,
        0.0f, 5.0f, 0.08f, 2u,
        (uint16_t)((1u << MENU_JUNCTION_DEBUG) | (1u << MENU_LINE_FOLLOW_DEBUG)),
        (uint16_t)PARAMS_IDX_LF_TURN_MIN_TIME_S, params_apply_line_follow
    },
    {
        "T_DNYL", "DoneYawL", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.turn_done_yaw_deg_l,
        0.0f, 180.0f, 55.0f, 1u,
        (uint16_t)((1u << MENU_JUNCTION_DEBUG) | (1u << MENU_LINE_FOLLOW_DEBUG)),
        (uint16_t)PARAMS_IDX_LF_TURN_DONE_YAW_DEG_L, params_apply_line_follow
    },
    {
        "T_DNYR", "DoneYawR", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.turn_done_yaw_deg_r,
        0.0f, 180.0f, 55.0f, 1u,
        (uint16_t)((1u << MENU_JUNCTION_DEBUG) | (1u << MENU_LINE_FOLLOW_DEBUG)),
        (uint16_t)PARAMS_IDX_LF_TURN_DONE_YAW_DEG_R, params_apply_line_follow
    },
    {
        "T_TIMES", "Timeout", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.turn_timeout_s,
        0.0f, 10.0f, 0.5f, 2u,
        (uint16_t)((1u << MENU_JUNCTION_DEBUG) | (1u << MENU_LINE_FOLLOW_DEBUG)),
        (uint16_t)PARAMS_IDX_LF_TURN_TIMEOUT_S, params_apply_line_follow
    },
    {
        "T_YSGN", "YawSign", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.turn_yaw_sign,
        -1.0f, 1.0f, 1.0f, 0u,
        (uint16_t)(1u << MENU_JUNCTION_DEBUG),
        (uint16_t)PARAMS_IDX_LF_TURN_YAW_SIGN, params_apply_line_follow
    },
    {
        "T_OSGN", "OutSign", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.turn_output_sign,
        -1.0f, 1.0f, -1.0f, 0u,
        (uint16_t)(1u << MENU_JUNCTION_DEBUG),
        (uint16_t)PARAMS_IDX_LF_TURN_OUTPUT_SIGN, params_apply_line_follow
    },
    {
        "FIN_CST", "Coast", PARAMS_VALUE_FLOAT, &g_line_follow_cfg.route_finish_coast_mm,
        0.0f, 1000.0f, 0.0f, 1u,
        (uint16_t)(1u << MENU_JUNCTION_DEBUG),
        (uint16_t)PARAMS_IDX_LF_ROUTE_FINISH_COAST_MM, params_apply_line_follow
    },
    {
        "GY_LP_A", "LPF Alpha", PARAMS_VALUE_FLOAT, &g_gyro_lp_alpha,
        0.001f, 1.0f, 1.0f, 3u,
        (uint16_t)(1u << MENU_IMU_DEBUG),
        (uint16_t)PARAMS_IDX_GYRO_LP_ALPHA, NULL
    },

#ifndef HARDWARE_STRIPPED
    {
        "NP_EN", "ESC Enable", PARAMS_VALUE_BOOL, &g_neg_pressure_enable,
        0.0f, 1.0f, 1.0f, 0u,
        (uint16_t)(1u << MENU_NEG_PRESSURE_DEBUG),
        PARAMS_FLASH_INDEX_NONE, params_apply_neg_pressure_enable
    },
    {
        "NP_THR", "Throttle%", PARAMS_VALUE_FLOAT, &g_neg_pressure_target_percent,
        0.0f, 100.0f, 1.0f, 0u,
        (uint16_t)(1u << MENU_NEG_PRESSURE_DEBUG),
        PARAMS_FLASH_INDEX_NONE, NULL
    },
#endif

    {
        "OL_RUN", "Run", PARAMS_VALUE_BOOL, &g_open_loop_run,
        0.0f, 1.0f, 1.0f, 0u,
        (uint16_t)(1u << MENU_BLDC_OPEN_LOOP_TEST),
        PARAMS_FLASH_INDEX_NONE, NULL
    },
    {
        "OL_HZ", "Elec Hz", PARAMS_VALUE_FLOAT, &g_open_loop_hz,
        0.0f, 100.0f, 1.0f, 1u,
        (uint16_t)(1u << MENU_BLDC_OPEN_LOOP_TEST),
        (uint16_t)PARAMS_IDX_OPEN_LOOP_HZ, NULL
    },
    {
        "OL_VQ", "Target Vq", PARAMS_VALUE_FLOAT, &g_open_loop_vq,
        0.0f, 1.0f, 0.05f, 2u,
        (uint16_t)(1u << MENU_BLDC_OPEN_LOOP_TEST),
        (uint16_t)PARAMS_IDX_OPEN_LOOP_VQ, NULL
    },
};

static float params_clampf(float value, float min_value, float max_value)
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

static float params_sanitize_float(float value, float fallback)
{
    if (!isfinite(value))
    {
        return fallback;
    }
    return value;
}

static uint8_t params_ascii_equal_ignore_case(uint8_t lhs, uint8_t rhs)
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

static uint16_t params_page_mask(menu_state_t page)
{
    uint8_t shift = (uint8_t)page;
    if (shift >= 16u)
    {
        return 0u;
    }
    return (uint16_t)(1u << shift);
}

static uint8_t params_descriptor_in_page(const params_descriptor_t *desc, menu_state_t page)
{
    if (desc == NULL)
    {
        return 0u;
    }
    return (uint8_t)((desc->page_mask & params_page_mask(page)) != 0u);
}

static uint8_t params_neg_pressure_run_requested(void)
{
#ifdef HARDWARE_STRIPPED
    return 0u;
#else
    return (uint8_t)((g_run_enable != 0u)
                  || ((g_menu_state == MENU_NEG_PRESSURE_DEBUG) && (g_neg_pressure_enable != 0u)));
#endif
}

static void params_apply_drive_limits(void)
{
    g_loop_l.vq_limit = params_clampf(g_loop_l.vq_limit, 0.0f, 0.95f);
    g_loop_r.vq_limit = params_clampf(g_loop_r.vq_limit, 0.0f, 0.95f);

    g_loop_l.speed_pid.kp = params_clampf(g_loop_l.speed_pid.kp, 0.0f, 100.0f);
    g_loop_l.speed_pid.ki = params_clampf(g_loop_l.speed_pid.ki, 0.0f, 100.0f);
    g_loop_r.speed_pid.kp = params_clampf(g_loop_r.speed_pid.kp, 0.0f, 100.0f);
    g_loop_r.speed_pid.ki = params_clampf(g_loop_r.speed_pid.ki, 0.0f, 100.0f);

    g_loop_l.speed_pid.integ = params_clampf(g_loop_l.speed_pid.integ, -g_loop_l.vq_limit, g_loop_l.vq_limit);
    g_loop_r.speed_pid.integ = params_clampf(g_loop_r.speed_pid.integ, -g_loop_r.vq_limit, g_loop_r.vq_limit);
    g_loop_l.vq_out = params_clampf(g_loop_l.vq_out, -g_loop_l.vq_limit, g_loop_l.vq_limit);
    g_loop_r.vq_out = params_clampf(g_loop_r.vq_out, -g_loop_r.vq_limit, g_loop_r.vq_limit);
}

static void params_apply_line_follow(void)
{
    line_follow_apply_config();
}

static void params_apply_foc_test_toggle(void)
{
    g_run_enable = 0u;
    g_stop_reason = SYSTEM_STOP_REASON_NONE;

    if (g_foc_test_run == 0u)
    {
        g_target_left_rpm = 0.0f;
        g_target_right_rpm = 0.0f;
#ifndef HARDWARE_STRIPPED
        foc_speed_control_set_target_rpm(&g_loop_l, &g_loop_r, 0.0f, 0.0f);
        foc_speed_loop_reset_on_enable(&g_loop_l);
        foc_speed_loop_reset_on_enable(&g_loop_r);
        bldc_drv8300_3pwm_motor_stop(&g_pwm.left);
        bldc_drv8300_3pwm_motor_stop(&g_pwm.right);
#endif
    }
}

static void params_apply_neg_pressure_enable(void)
{
#ifndef HARDWARE_STRIPPED
    if (!params_neg_pressure_run_requested())
    {
        neg_pressure_motor_stop(&g_neg_pressure_motor);
    }
#endif
}

uint16_t params_get_count(void)
{
    return (uint16_t)(sizeof(g_params_registry) / sizeof(g_params_registry[0]));
}

const params_descriptor_t *params_get_descriptor(uint16_t index)
{
    if (index >= params_get_count())
    {
        return NULL;
    }
    return &g_params_registry[index];
}

uint16_t params_get_page_item_count(menu_state_t page)
{
    uint16_t count = 0u;

    for (uint16_t i = 0u; i < params_get_count(); ++i)
    {
        if (params_descriptor_in_page(&g_params_registry[i], page))
        {
            count++;
        }
    }

    return count;
}

const params_descriptor_t *params_get_page_item(menu_state_t page, uint16_t page_index)
{
    uint16_t count = 0u;

    for (uint16_t i = 0u; i < params_get_count(); ++i)
    {
        if (!params_descriptor_in_page(&g_params_registry[i], page))
        {
            continue;
        }

        if (count == page_index)
        {
            return &g_params_registry[i];
        }

        count++;
    }

    return NULL;
}

const params_descriptor_t *params_find_descriptor(const char *key, uint32_t key_len)
{
    if ((key == NULL) || (key_len == 0u))
    {
        return NULL;
    }

    for (uint16_t i = 0u; i < params_get_count(); ++i)
    {
        const params_descriptor_t *desc = &g_params_registry[i];
        uint32_t desc_len = 0u;

        while (desc->key[desc_len] != '\0')
        {
            desc_len++;
        }

        if (desc_len != key_len)
        {
            continue;
        }

        {
            uint32_t match = 1u;
            for (uint32_t j = 0u; j < key_len; ++j)
            {
                if (!params_ascii_equal_ignore_case((uint8_t)key[j], (uint8_t)desc->key[j]))
                {
                    match = 0u;
                    break;
                }
            }

            if (match != 0u)
            {
                return desc;
            }
        }
    }

    return NULL;
}

float params_get_value_as_float(const params_descriptor_t *desc)
{
    if ((desc == NULL) || (desc->value_ptr == NULL))
    {
        return 0.0f;
    }

    switch (desc->type)
    {
    case PARAMS_VALUE_FLOAT:
        return *(volatile float *)desc->value_ptr;
    case PARAMS_VALUE_BOOL:
    case PARAMS_VALUE_U8:
        return (float)(*(volatile uint8_t *)desc->value_ptr);
    default:
        return 0.0f;
    }
}

void params_apply_descriptor(const params_descriptor_t *desc)
{
    if ((desc != NULL) && (desc->apply != NULL))
    {
        desc->apply();
    }
}

void params_set_value_from_float(const params_descriptor_t *desc, float value)
{
    float clamped;

    if ((desc == NULL) || (desc->value_ptr == NULL))
    {
        return;
    }

    clamped = params_sanitize_float(value, desc->min_value);
    clamped = params_clampf(clamped, desc->min_value, desc->max_value);

    switch (desc->type)
    {
    case PARAMS_VALUE_FLOAT:
        *(volatile float *)desc->value_ptr = clamped;
        break;
    case PARAMS_VALUE_U8:
        *(volatile uint8_t *)desc->value_ptr = (uint8_t)clamped;
        break;
    case PARAMS_VALUE_BOOL:
        *(volatile uint8_t *)desc->value_ptr = (uint8_t)((clamped >= 0.5f) ? 1u : 0u);
        break;
    default:
        return;
    }

    params_apply_descriptor(desc);
}

void params_apply_all_descriptors(void)
{
    for (uint16_t i = 0u; i < params_get_count(); ++i)
    {
        params_apply_descriptor(&g_params_registry[i]);
    }
}

static void params_write_flash_value(const params_descriptor_t *desc)
{
    if ((desc == NULL) || (desc->flash_index == PARAMS_FLASH_INDEX_NONE))
    {
        return;
    }

    switch (desc->type)
    {
    case PARAMS_VALUE_FLOAT:
        flash_union_buffer[desc->flash_index].float_type = *(volatile float *)desc->value_ptr;
        break;
    case PARAMS_VALUE_U8:
    case PARAMS_VALUE_BOOL:
        flash_union_buffer[desc->flash_index].uint32_type = (uint32_t)(*(volatile uint8_t *)desc->value_ptr);
        break;
    default:
        break;
    }
}

static void params_load_flash_value(const params_descriptor_t *desc)
{
    if ((desc == NULL) || (desc->flash_index == PARAMS_FLASH_INDEX_NONE))
    {
        return;
    }

    if (desc->type == PARAMS_VALUE_FLOAT)
    {
        float fallback = params_get_value_as_float(desc);
        float value = params_sanitize_float(flash_union_buffer[desc->flash_index].float_type, fallback);
        params_set_value_from_float(desc, value);
        return;
    }

    {
        uint32_t raw_u32 = flash_union_buffer[desc->flash_index].uint32_type;
        float fallback = params_get_value_as_float(desc);

        if (raw_u32 > 255u)
        {
            params_set_value_from_float(desc, fallback);
        }
        else
        {
            params_set_value_from_float(desc, (float)raw_u32);
        }
    }
}

static uint16_t params_crc16_ccitt(const uint16_t *data, uint16_t halfword_count)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0u; i < halfword_count; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0u; j < 8u; j++)
        {
            if (crc & 1u)
                crc = (uint16_t)((crc >> 1) ^ 0x8408u);
            else
                crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

void params_save_to_flash(void)
{
    flash_buffer_clear();

    flash_union_buffer[PARAMS_IDX_MAGIC].uint32_type = PARAMS_FLASH_MAGIC;

    for (uint16_t i = 0u; i < params_get_count(); ++i)
    {
        params_write_flash_value(&g_params_registry[i]);
    }

    {
        uint16_t min_values[LINE_FOLLOW_CH_COUNT];
        uint16_t max_values[LINE_FOLLOW_CH_COUNT];
        uint16_t thr_values[LINE_FOLLOW_CH_COUNT];

        line_follow_calib_get_minmax(min_values, max_values, (uint8_t)LINE_FOLLOW_CH_COUNT);
        line_follow_calib_get_thresholds(thr_values, (uint8_t)LINE_FOLLOW_CH_COUNT);

        if (line_follow_calib_is_valid())
        {
            flash_union_buffer[PARAMS_IDX_LF_CALIB_VALID].uint32_type = 1u;
            for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
            {
                flash_union_buffer[PARAMS_IDX_LF_CALIB_MIN_0 + i].uint32_type = (uint32_t)min_values[i];
                flash_union_buffer[PARAMS_IDX_LF_CALIB_MAX_0 + i].uint32_type = (uint32_t)max_values[i];
                flash_union_buffer[PARAMS_IDX_LF_CALIB_THR_0 + i].uint32_type = (uint32_t)thr_values[i];
            }
        }
        else
        {
            flash_union_buffer[PARAMS_IDX_LF_CALIB_VALID].uint32_type = 0u;
        }
    }

    {
        uint16_t crc = params_crc16_ccitt(
            (const uint16_t *)&flash_union_buffer[0],
            (uint16_t)(PARAMS_FLASH_CRC_SLOT * 2u)
        );
        flash_union_buffer[PARAMS_FLASH_CRC_SLOT].uint32_type = (uint32_t)crc;
    }

    (void)flash_write_page_from_buffer(PARAMS_FLASH_SECTOR_NUM, PARAMS_FLASH_PAGE_INDEX);
}

void params_load_from_flash(void)
{
    flash_read_page_to_buffer(PARAMS_FLASH_SECTOR_NUM, PARAMS_FLASH_PAGE_INDEX);

    if (flash_union_buffer[PARAMS_IDX_MAGIC].uint32_type != PARAMS_FLASH_MAGIC)
    {
        params_apply_all_descriptors();
        return;
    }

    {
        uint16_t computed = params_crc16_ccitt(
            (const uint16_t *)&flash_union_buffer[0],
            (uint16_t)(PARAMS_FLASH_CRC_SLOT * 2u)
        );
        uint16_t stored = (uint16_t)(flash_union_buffer[PARAMS_FLASH_CRC_SLOT].uint32_type & 0xFFFFu);
        if (computed != stored)
        {
            params_apply_all_descriptors();
            return;
        }
    }

    for (uint16_t i = 0u; i < params_get_count(); ++i)
    {
        params_load_flash_value(&g_params_registry[i]);
    }

    {
        uint16_t min_values[LINE_FOLLOW_CH_COUNT];
        uint16_t max_values[LINE_FOLLOW_CH_COUNT];
        uint16_t thr_values[LINE_FOLLOW_CH_COUNT];
        uint8_t valid = (uint8_t)(flash_union_buffer[PARAMS_IDX_LF_CALIB_VALID].uint32_type != 0u);

        for (uint8_t i = 0u; i < LINE_FOLLOW_CH_COUNT; ++i)
        {
            min_values[i] = (uint16_t)(flash_union_buffer[PARAMS_IDX_LF_CALIB_MIN_0 + i].uint32_type & 0xFFFFu);
            max_values[i] = (uint16_t)(flash_union_buffer[PARAMS_IDX_LF_CALIB_MAX_0 + i].uint32_type & 0xFFFFu);
            thr_values[i] = (uint16_t)(flash_union_buffer[PARAMS_IDX_LF_CALIB_THR_0 + i].uint32_type & 0xFFFFu);
        }

        line_follow_calib_set_minmax(min_values, max_values, (uint8_t)LINE_FOLLOW_CH_COUNT, (bool)valid);
        line_follow_calib_set_thresholds(thr_values, (uint8_t)LINE_FOLLOW_CH_COUNT, (bool)valid);
    }

    line_follow_apply_config();
}
