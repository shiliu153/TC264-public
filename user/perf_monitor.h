#ifndef PERF_MONITOR_H
#define PERF_MONITOR_H
/*********************************************************************************************************************
* 文件：perf_monitor.h
* 模块：应用层（第 6 层）—— 性能监视
* 职责：性能监视对外接口与输出结构体声明
* 依赖：zf_common_typedef.h
* 风险等级：R1
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/

#include "zf_common_headfile.h"

typedef struct
{
    uint16_t cpu0_load_x100;
    uint16_t cpu1_load_x100;
    uint16_t foc_avg_us;
    uint16_t isr1k_avg_us;
    uint8_t  foc_load_pct;
    uint16_t window_ms;
    uint16_t debug_flush_count;
    uint16_t debug_busy_snapshot;
} perf_monitor_output_t;

void perf_monitor_init(void);
uint32_t perf_monitor_get_tick(void);
void perf_monitor_tick_cpu0(void);  /* call from periodic context */
void perf_monitor_mark_delay_end(void);
void perf_monitor_record_cpu0_isr(uint32_t t0);
void perf_monitor_record_foc_isr(uint32_t t0);
void perf_monitor_record_isr1k(uint32_t t0);
void perf_monitor_output(perf_monitor_output_t *out);

#endif
