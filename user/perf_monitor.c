/*********************************************************************************************************************
* 文件：perf_monitor.c
* 模块：应用层（第 6 层）—— 性能监视
* 职责：双核 100ms 窗口 CPU 占用率统计
* 依赖：zf_common_headfile.h、本目录 perf_monitor.h
* 风险等级：R1
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "IfxStm.h"
#include "perf_monitor.h"

#define PERF_FLUSH_MIN_TICKS 500000u

static uint32_t g_perf_cpu0_busy_tick;
static uint32_t g_perf_last_delay_end;
static uint32_t g_perf_cpu0_isr_tick;
static uint32_t g_perf_cpu1_isr_tick;
static uint32_t g_perf_cpu1_foc_tick;
static uint32_t g_perf_cpu1_isr1k_tick;
static uint32_t g_perf_cpu1_foc_count;
static uint32_t g_perf_cpu1_isr1k_count;

static perf_monitor_output_t g_perf_output;

static uint32_t g_last_flush_stm;

void perf_monitor_init(void)
{
    g_perf_cpu0_busy_tick = 0u;
    g_perf_last_delay_end = IfxStm_getLower(&MODULE_STM0);
    g_perf_cpu0_isr_tick = 0u;
    g_perf_cpu1_isr_tick = 0u;
    g_perf_cpu1_foc_tick = 0u;
    g_perf_cpu1_isr1k_tick = 0u;
    g_perf_cpu1_foc_count = 0u;
    g_perf_cpu1_isr1k_count = 0u;
    g_last_flush_stm = IfxStm_getLower(&MODULE_STM0);
}

uint32_t perf_monitor_get_tick(void)
{
    return IfxStm_getLower(&MODULE_STM0);
}

void perf_monitor_tick_cpu0(void)
{
    uint32_t now = IfxStm_getLower(&MODULE_STM0);
    g_perf_cpu0_busy_tick += (now - g_perf_last_delay_end);

    uint32_t elapsed = (now - g_last_flush_stm);
    if (elapsed < PERF_FLUSH_MIN_TICKS) return;

    {
        uint32_t work_tick = g_perf_cpu0_busy_tick;
        uint32_t total = work_tick + g_perf_cpu0_isr_tick;
        if (total > elapsed) total = elapsed;
        uint16_t instant = (uint16_t)(((uint64_t)total * 10000u) / (uint64_t)elapsed);
        /* EMA smooth: alpha = 0.25 */
        g_perf_output.cpu0_load_x100 = (uint16_t)(
            ((uint32_t)instant + (uint32_t)g_perf_output.cpu0_load_x100 * 3u) / 4u);
    }

    if (elapsed > 0u)
    {
        uint16_t instant1 = (uint16_t)(((uint64_t)g_perf_cpu1_isr_tick * 10000u) / (uint64_t)elapsed);
        g_perf_output.cpu1_load_x100 = (uint16_t)(
            ((uint32_t)instant1 + (uint32_t)g_perf_output.cpu1_load_x100 * 3u) / 4u);
        g_perf_output.foc_load_pct = (uint8_t)(
            ((uint64_t)g_perf_cpu1_foc_tick * 100u) / (uint64_t)elapsed);
    }

    g_perf_output.foc_avg_us = (g_perf_cpu1_foc_count > 0u)
        ? (uint16_t)((g_perf_cpu1_foc_tick / 100u) / g_perf_cpu1_foc_count) : 0u;
    g_perf_output.isr1k_avg_us = (g_perf_cpu1_isr1k_count > 0u)
        ? (uint16_t)((g_perf_cpu1_isr1k_tick / 100u) / g_perf_cpu1_isr1k_count) : 0u;
    g_perf_output.window_ms = (uint16_t)(elapsed / 100000u);

    {
        static uint16_t flush_cnt = 0u;
        flush_cnt++;
        g_perf_output.debug_flush_count = flush_cnt;
    }
    g_perf_output.debug_busy_snapshot = (uint16_t)(g_perf_cpu0_busy_tick / 100000u);

    g_perf_cpu0_busy_tick = 0u;
    g_perf_cpu0_isr_tick = 0u;
    g_perf_cpu1_isr_tick = 0u;
    g_perf_cpu1_foc_tick = 0u;
    g_perf_cpu1_isr1k_tick = 0u;
    g_perf_cpu1_foc_count = 0u;
    g_perf_cpu1_isr1k_count = 0u;
    g_last_flush_stm = now;
}

void perf_monitor_mark_delay_end(void)
{
    g_perf_last_delay_end = IfxStm_getLower(&MODULE_STM0);
}

void perf_monitor_record_cpu0_isr(uint32_t t0)
{
    uint32_t t1 = IfxStm_getLower(&MODULE_STM0);
    g_perf_cpu0_isr_tick += (t1 - t0);
}

void perf_monitor_record_foc_isr(uint32_t t0)
{
    uint32_t t1 = IfxStm_getLower(&MODULE_STM0);
    uint32_t elapsed = t1 - t0;
    g_perf_cpu1_isr_tick += elapsed;
    g_perf_cpu1_foc_tick += elapsed;
    g_perf_cpu1_foc_count++;
}

void perf_monitor_record_isr1k(uint32_t t0)
{
    uint32_t t1 = IfxStm_getLower(&MODULE_STM0);
    uint32_t elapsed = t1 - t0;
    g_perf_cpu1_isr_tick += elapsed;
    g_perf_cpu1_isr1k_tick += elapsed;
    g_perf_cpu1_isr1k_count++;
}

void perf_monitor_output(perf_monitor_output_t *out)
{
    if (out != NULL)
    {
        *out = g_perf_output;
    }
}
