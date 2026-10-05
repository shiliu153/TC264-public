#define _wifi_service_h_

#include "zf_common_headfile.h"

#include "zf_device/zf_device_gray_4067.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
	float gyro_z_dps;
	float left_target_rpm;
	float right_target_rpm;
	float left_actual_rpm;
	float right_actual_rpm;
	float left_elec_angle_deg;
	float right_elec_angle_deg;
	float battery_voltage_v;
	uint16_t battery_raw;
	uint8_t battery_flags;
	uint8_t battery_percent; // 0..100 (%).
	float np_target_percent;
	float np_output_pulse_us;
	uint8_t np_flags;
	float line_pos;
	float turn_delta_rpm;
	uint16_t line_bits;
	uint8_t run_enable;
	uint8_t line_pos_valid;
	uint8_t line_follow_ready;
	uint8_t lf_state;
	uint8_t pending_turn;
	uint8_t route_index;
	uint8_t lost_line_cnt;
	uint8_t stop_reason;
	uint8_t junction_lock_cnt;
	uint8_t element_enter_cnt;
	uint8_t junction_enter_cnt;
	uint8_t lf_actual_hz;
	uint8_t calib_active;
	uint8_t calib_samples_hi;
	uint8_t calib_valid;
	float turn_yaw_deg;
	float odom_left_mm;
	float odom_right_mm;
	uint8_t odom_count_active;
	float left_vq_out;
	float right_vq_out;
	uint16_t gray[GRAY_4067_DEFAULT_CHANNEL_COUNT];
	uint8_t gray_valid;
} wifi_service_telemetry_snapshot_t;

void wifi_service_periodic(void);
void wifi_service_capture_snapshot_isr(void);
void wifi_service_get_att_zero(float *roll_deg, float *pitch_deg, float *yaw_deg);
void wifi_service_set_att_zero(float roll_deg, float pitch_deg, float yaw_deg);

#ifdef __cplusplus
}
#endif

