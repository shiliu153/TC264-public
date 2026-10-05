#include "IfxStm.h"
#include "perf_monitor.h"

#define PERF_WINDOW_TICK_TARGET  100000u

static uint32_t g_perf_cpu0_idle_tick;
static uint32_t g_perf_cpu1_isr_tick;
static uint32_t g_perf_cpu1_foc_tick;
static uint32_t g_perf_cpu1_isr1k_tick;
static uint32_t g_perf_cpu1_foc_count;
static uint32_t g_perf_cpu1_isr1k_count;
static uint32_t g_perf_window_start;

static perf_monitor_output_t g_perf_output;

void perf_monitor_init(void)
{
    g_perf_cpu0_idle_tick = 0u;
    g_perf_cpu1_isr_tick = 0u;
    g_perf_cpu1_foc_tick = 0u;
    g_perf_cpu1_isr1k_tick = 0u;
    g_perf_cpu1_foc_count = 0u;
    g_perf_cpu1_isr1k_count = 0u;
    g_perf_window_start = IfxStm_getLower(&MODULE_STM0);
}

uint32_t perf_monitor_get_tick(void)
{
    return IfxStm_getLower(&MODULE_STM0);
}

void perf_monitor_record_cpu0_idle(uint32_t idle_start)
{
    uint32_t now = IfxStm_getLower(&MODULE_STM0);
    g_perf_cpu0_idle_tick += (now - idle_start);

    uint32_t elapsed = (now - g_perf_window_start);
    if (elapsed >= PERF_WINDOW_TICK_TARGET * 100u)
    {
        uint32_t busy_tick = (elapsed > g_perf_cpu0_idle_tick)
                           ? (elapsed - g_perf_cpu0_idle_tick) : 0u;

        g_perf_output.cpu0_load_x100 = (uint16_t)(
            ((uint64_t)busy_tick * 10000u) / (uint64_t)elapsed);

        if (elapsed > 0u)
        {
            g_perf_output.cpu1_load_x100 = (uint16_t)(
                ((uint64_t)g_perf_cpu1_isr_tick * 10000u) / (uint64_t)elapsed);
            g_perf_output.foc_load_pct = (uint8_t)(
                ((uint64_t)g_perf_cpu1_foc_tick * 100u) / (uint64_t)elapsed);
        }
        else
        {
            g_perf_output.cpu1_load_x100 = 0u;
            g_perf_output.foc_load_pct = 0u;
        }

        g_perf_output.foc_avg_us = (g_perf_cpu1_foc_count > 0u)
            ? (uint16_t)((g_perf_cpu1_foc_tick / 100u) / g_perf_cpu1_foc_count) : 0u;
        g_perf_output.isr1k_avg_us = (g_perf_cpu1_isr1k_count > 0u)
            ? (uint16_t)((g_perf_cpu1_isr1k_tick / 100u) / g_perf_cpu1_isr1k_count) : 0u;
        g_perf_output.window_ms = (uint16_t)(elapsed / 100000u);

        g_perf_cpu0_idle_tick = 0u;
        g_perf_cpu1_isr_tick = 0u;
        g_perf_cpu1_foc_tick = 0u;
        g_perf_cpu1_isr1k_tick = 0u;
        g_perf_cpu1_foc_count = 0u;
        g_perf_cpu1_isr1k_count = 0u;
        g_perf_window_start = now;
    }
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
