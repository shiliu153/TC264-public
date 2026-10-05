#ifndef PERF_MONITOR_H
#define PERF_MONITOR_H

#include "zf_common_headfile.h"

typedef struct
{
    uint16_t cpu0_load_x100;
    uint16_t cpu1_load_x100;
    uint16_t foc_avg_us;
    uint16_t isr1k_avg_us;
    uint8_t  foc_load_pct;
    uint16_t window_ms;
} perf_monitor_output_t;

void perf_monitor_init(void);
uint32_t perf_monitor_get_tick(void);
void perf_monitor_record_cpu0_idle(uint32_t idle_start);
void perf_monitor_record_foc_isr(uint32_t t0);
void perf_monitor_record_isr1k(uint32_t t0);
void perf_monitor_output(perf_monitor_output_t *out);

#endif
