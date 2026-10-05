#ifndef _battery_monitor_h_
#define _battery_monitor_h_
/*********************************************************************************************************************
* 文件：battery_monitor.h
* 模块：应用层（第 6 层）—— 电池监测
* 职责：电池监测对外接口与阈值声明
* 依赖：zf_common_typedef.h
* 风险等级：R2
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint8_t g_battery_cells;

void battery_monitor_init(void);
void battery_monitor_periodic(void);

float battery_monitor_get_voltage_v(void);
uint16_t battery_monitor_get_raw(void);
float battery_monitor_get_pin_voltage_v(void);
uint8_t battery_monitor_get_percent(void);

uint8_t battery_monitor_is_low(void);
uint8_t battery_monitor_is_critical(void);

#ifdef __cplusplus
}
#endif

#endif
