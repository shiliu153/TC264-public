/*********************************************************************************************************************
* 文件：battery_monitor.c
* 模块：应用层（第 6 层）—— 电池监测
* 职责：锂电池 ADC 电压监测与低/临界阈值判定
* 依赖：zf_driver_adc.h、本目录 battery_monitor.h
* 风险等级：R2
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "battery_monitor.h"
#include "isr.h"

#include "zf_driver/zf_driver_adc.h"

#ifndef BATTERY_ADC_CHANNEL
#define BATTERY_ADC_CHANNEL                 (ADC1_CH5_A21)
#endif

#ifndef BATTERY_ADC_REF_V
#define BATTERY_ADC_REF_V                   (3.3f)
#endif

#ifndef BATTERY_ADC_MAX_COUNT
#define BATTERY_ADC_MAX_COUNT               (4095.0f)
#endif

#ifndef BATTERY_DIVIDER_R_HIGH_OHM
#define BATTERY_DIVIDER_R_HIGH_OHM          (220000.0f)
#endif

#ifndef BATTERY_DIVIDER_R_LOW_OHM
#define BATTERY_DIVIDER_R_LOW_OHM           (20000.0f)
#endif

#ifndef BATTERY_VOLT_CALIB_GAIN
#define BATTERY_VOLT_CALIB_GAIN             (0.9512f)
#endif

#ifndef BATTERY_MONITOR_PERIOD_MS
#define BATTERY_MONITOR_PERIOD_MS           (100u)
#endif

#ifndef BATTERY_MONITOR_AVG_SAMPLES
#define BATTERY_MONITOR_AVG_SAMPLES         (8u)
#endif

#ifndef BATTERY_MONITOR_LP_ALPHA
#define BATTERY_MONITOR_LP_ALPHA            (0.25f)
#endif

#ifndef BATTERY_LOW_WARN_PER_CELL_V
#define BATTERY_LOW_WARN_PER_CELL_V         (3.55f)
#endif

#ifndef BATTERY_LOW_CRITICAL_PER_CELL_V
#define BATTERY_LOW_CRITICAL_PER_CELL_V     (3.40f)
#endif

#ifndef BATTERY_LOW_DEBOUNCE_MS
#define BATTERY_LOW_DEBOUNCE_MS             (500u)
#endif

#ifndef BATTERY_SOC_EMPTY_PER_CELL_V
#define BATTERY_SOC_EMPTY_PER_CELL_V        (3.30f)
#endif

#ifndef BATTERY_SOC_FULL_PER_CELL_V
#define BATTERY_SOC_FULL_PER_CELL_V         (4.20f)
#endif

volatile uint8_t g_battery_cells = 2;

static volatile uint16_t g_battery_raw = 0u;
static volatile float g_battery_pin_voltage_v = 0.0f;
static volatile float g_battery_voltage_v = 0.0f;
static volatile uint8_t g_battery_inited = 0u;
static volatile uint8_t g_battery_low_debounce_cnt = 0u;
static volatile uint8_t g_battery_critical_debounce_cnt = 0u;

static float battery_monitor_raw_to_voltage(uint16_t raw)
{
    const float divider_ratio = (BATTERY_DIVIDER_R_HIGH_OHM + BATTERY_DIVIDER_R_LOW_OHM) / BATTERY_DIVIDER_R_LOW_OHM;
    float adc_voltage = ((float)raw * BATTERY_ADC_REF_V) / BATTERY_ADC_MAX_COUNT;

    return adc_voltage * divider_ratio * BATTERY_VOLT_CALIB_GAIN;
}

static float battery_monitor_raw_to_pin_voltage(uint16_t raw)
{
    return ((float)raw * BATTERY_ADC_REF_V) / BATTERY_ADC_MAX_COUNT;
}

void battery_monitor_init(void)
{
    adc_init(BATTERY_ADC_CHANNEL, ADC_12BIT);

#ifndef HARDWARE_STRIPPED
    {
        uint16_t single_raw = adc_convert(BATTERY_ADC_CHANNEL);
        printf("[BAT_INIT] single_raw=%u mean_raw=", (unsigned int)single_raw);
    }
#endif

    g_battery_raw = adc_mean_filter_convert(BATTERY_ADC_CHANNEL, BATTERY_MONITOR_AVG_SAMPLES);
    g_battery_pin_voltage_v = battery_monitor_raw_to_pin_voltage(g_battery_raw);
    g_battery_voltage_v = battery_monitor_raw_to_voltage(g_battery_raw);
    g_battery_inited = 1u;

#ifndef HARDWARE_STRIPPED
    printf("%u pin=%.3fV batt=%.3fV\r\n",
           (unsigned int)g_battery_raw, (double)g_battery_pin_voltage_v, (double)g_battery_voltage_v);
#endif
}

void battery_monitor_periodic(void)
{
    static uint32_t last_ms = 0u;
    uint32_t now_ms;
    uint16_t raw;
    float pin_voltage_v;
    float voltage_v;

    if (!g_battery_inited)
    {
        return;
    }

    now_ms = (uint32_t)system_getval_ms();
    if ((uint32_t)(now_ms - last_ms) < BATTERY_MONITOR_PERIOD_MS)
    {
        return;
    }
    last_ms = now_ms;

    raw = adc_mean_filter_convert(BATTERY_ADC_CHANNEL, BATTERY_MONITOR_AVG_SAMPLES);
    pin_voltage_v = battery_monitor_raw_to_pin_voltage(raw);
    voltage_v = battery_monitor_raw_to_voltage(raw);

    g_battery_raw = raw;
    g_battery_pin_voltage_v += BATTERY_MONITOR_LP_ALPHA * (pin_voltage_v - g_battery_pin_voltage_v);
    g_battery_voltage_v += BATTERY_MONITOR_LP_ALPHA * (voltage_v - g_battery_voltage_v);

#ifndef HARDWARE_STRIPPED
    {
        static uint16_t _dbg_cnt = 0u;
        if (++_dbg_cnt >= 10u)
        {
            uint16_t single_raw = adc_convert(BATTERY_ADC_CHANNEL);
            _dbg_cnt = 0u;
            printf("[BAT] raw=%u(avg) single=%u pin=%.3fV batt=%.3fV\r\n",
                   (unsigned int)raw, (unsigned int)single_raw,
                   (double)pin_voltage_v, (double)voltage_v);
        }
    }
#endif

    {
        const float low_warn_v     = (float)g_battery_cells * BATTERY_LOW_WARN_PER_CELL_V;
        const float low_critical_v = (float)g_battery_cells * BATTERY_LOW_CRITICAL_PER_CELL_V;
        const uint8_t debounce_samples = (uint8_t)(BATTERY_LOW_DEBOUNCE_MS / BATTERY_MONITOR_PERIOD_MS);

        if (g_battery_voltage_v <= low_warn_v)
        {
            if (g_battery_low_debounce_cnt < debounce_samples)
                g_battery_low_debounce_cnt++;
        }
        else
        {
            g_battery_low_debounce_cnt = 0u;
        }

        if (g_battery_voltage_v <= low_critical_v)
        {
            if (g_battery_critical_debounce_cnt < debounce_samples)
                g_battery_critical_debounce_cnt++;
        }
        else
        {
            g_battery_critical_debounce_cnt = 0u;
        }
    }
}

float battery_monitor_get_voltage_v(void)
{
    return g_battery_voltage_v;
}

uint16_t battery_monitor_get_raw(void)
{
    return g_battery_raw;
}

float battery_monitor_get_pin_voltage_v(void)
{
    return g_battery_pin_voltage_v;
}

uint8_t battery_monitor_get_percent(void)
{
    const float pack_empty_v = (float)g_battery_cells * BATTERY_SOC_EMPTY_PER_CELL_V;
    const float pack_full_v = (float)g_battery_cells * BATTERY_SOC_FULL_PER_CELL_V;

    if (pack_full_v <= (pack_empty_v + 0.01f))
    {
        return 0u;
    }

    if (g_battery_voltage_v <= pack_empty_v)
    {
        return 0u;
    }

    if (g_battery_voltage_v >= pack_full_v)
    {
        return 100u;
    }

    {
        float percent = (g_battery_voltage_v - pack_empty_v) * 100.0f / (pack_full_v - pack_empty_v);
        if (percent < 0.0f) percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        return (uint8_t)(percent + 0.5f);
    }
}

uint8_t battery_monitor_is_low(void)
{
    const uint8_t debounce_samples = (uint8_t)(BATTERY_LOW_DEBOUNCE_MS / BATTERY_MONITOR_PERIOD_MS);
    return (uint8_t)(g_battery_low_debounce_cnt >= debounce_samples);
}

uint8_t battery_monitor_is_critical(void)
{
    const uint8_t debounce_samples = (uint8_t)(BATTERY_LOW_DEBOUNCE_MS / BATTERY_MONITOR_PERIOD_MS);
    return (uint8_t)(g_battery_critical_debounce_cnt >= debounce_samples);
}