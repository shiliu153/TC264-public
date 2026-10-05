#include "zf_device_gray_4067.h"
#include "zf_driver/zf_driver_gpio.h"

static uint8_t g_gray_4067_enabled = 0u;

static inline uint8_t gray_4067_is_channel_0_or_15(uint8_t channel)
{
    uint8_t ch = (uint8_t)(channel & 0x0Fu);
    return (uint8_t)((ch == 0u) || (ch == 15u));
}

static inline void gray_4067_write_select(uint8_t channel)
{
    gpio_set_level(GRAY_4067_S0_PIN, (channel >> 0u) & 0x01u);
    gpio_set_level(GRAY_4067_S1_PIN, (channel >> 1u) & 0x01u);
    gpio_set_level(GRAY_4067_S2_PIN, (channel >> 2u) & 0x01u);
    gpio_set_level(GRAY_4067_S3_PIN, (channel >> 3u) & 0x01u);
}

static inline uint16_t gray_4067_read_selected_channel(void)
{
    uint8_t discard_samples = (uint8_t)GRAY_4067_DISCARD_SAMPLES;

    for(uint8_t i = 0u; i < discard_samples; i++)
    {
        (void)adc_convert(GRAY_4067_ADC_CHANNEL);
    }

#if GRAY_4067_POST_DISCARD_DELAY_US
    if(discard_samples > 0u)
    {
        system_delay_us(GRAY_4067_POST_DISCARD_DELAY_US);
    }
#endif

#if (GRAY_4067_SAMPLE_AVG_COUNT <= 1)
    return adc_convert(GRAY_4067_ADC_CHANNEL);
#else
    uint32_t sum = 0u;
    for(uint8_t i = 0u; i < (uint8_t)GRAY_4067_SAMPLE_AVG_COUNT; i++)
    {
        sum += adc_convert(GRAY_4067_ADC_CHANNEL);
    }
    return (uint16_t)(sum / (uint32_t)GRAY_4067_SAMPLE_AVG_COUNT);
#endif
}

void gray_4067_init(void)
{
    gpio_init(GRAY_4067_S0_PIN, GPO, GPIO_LOW,  GPO_PUSH_PULL);
    gpio_init(GRAY_4067_S1_PIN, GPO, GPIO_LOW,  GPO_PUSH_PULL);
    gpio_init(GRAY_4067_S2_PIN, GPO, GPIO_LOW,  GPO_PUSH_PULL);
    gpio_init(GRAY_4067_S3_PIN, GPO, GPIO_LOW,  GPO_PUSH_PULL);

    // /EN 引脚
    gpio_init(GRAY_4067_EN_PIN, GPO,
              (GRAY_4067_EN_ACTIVE_LEVEL ? GPIO_LOW : GPIO_HIGH),
              GPO_PUSH_PULL);

    // 先禁用，再配置 ADC，最后使能
    gray_4067_enable(0);

    adc_init(GRAY_4067_ADC_CHANNEL, GRAY_4067_ADC_RESOLUTION);

    gray_4067_set_channel((uint8_t)GRAY_4067_CH12_START_CHANNEL);
    gray_4067_enable(1);
}

void gray_4067_enable(uint8_t enable)
{
    g_gray_4067_enabled = (uint8_t)(enable ? 1u : 0u);
    gpio_set_level(GRAY_4067_EN_PIN,
                   g_gray_4067_enabled ? GRAY_4067_EN_ACTIVE_LEVEL : (GRAY_4067_EN_ACTIVE_LEVEL ? GPIO_LOW : GPIO_HIGH));
}

void gray_4067_set_channel(uint8_t channel)
{
    uint8_t ch = (uint8_t)(channel & 0x0Fu);

#if GRAY_4067_SWITCH_WITH_DISABLE
    uint8_t restore_enable = g_gray_4067_enabled;
    if(restore_enable)
    {
        gray_4067_enable(0u);
#if GRAY_4067_DISABLE_GUARD_US
        system_delay_us(GRAY_4067_DISABLE_GUARD_US);
#endif
    }
#endif

    gray_4067_write_select(ch);

    if(GRAY_4067_SETTLE_US)
    {
        system_delay_us(GRAY_4067_SETTLE_US);
    }

#if GRAY_4067_SWITCH_WITH_DISABLE
    if(restore_enable)
    {
        gray_4067_enable(1u);
#if GRAY_4067_DISABLE_GUARD_US
        system_delay_us(GRAY_4067_DISABLE_GUARD_US);
#endif
    }
#endif

#if GRAY_4067_CH0_CH15_EXTRA_SETTLE_US
    if(gray_4067_is_channel_0_or_15(ch))
    {
        system_delay_us(GRAY_4067_CH0_CH15_EXTRA_SETTLE_US);
    }
#endif
}

uint16_t gray_4067_read_raw(uint8_t channel)
{
    gray_4067_set_channel(channel);
    return gray_4067_read_selected_channel();
}

void gray_4067_read_all_raw(uint16_t out_buffer[16])
{
    if(NULL == out_buffer)
    {
        return;
    }

    uint8_t i;
    for(i = 0; i < 16; i++)
    {
        out_buffer[i] = gray_4067_read_raw(i);
#if GRAY_4067_INTER_CHANNEL_DELAY_US
        if(i < 15u)
        {
            system_delay_us(GRAY_4067_INTER_CHANNEL_DELAY_US);
        }
#endif
    }

#if GRAY_4067_BATCH_RESTORE_TO_START
    gray_4067_set_channel(0u);
#if GRAY_4067_BATCH_RESTORE_PRIME_SAMPLE
    (void)adc_convert(GRAY_4067_ADC_CHANNEL);
#endif
#endif
}

void gray_4067_read_12_raw(uint16_t out_buffer[GRAY_4067_DEFAULT_CHANNEL_COUNT])
{
    if(NULL == out_buffer)
    {
        return;
    }

    uint8_t i;
    for(i = 0; i < GRAY_4067_DEFAULT_CHANNEL_COUNT; i++)
    {
        out_buffer[i] = gray_4067_read_raw((uint8_t)(GRAY_4067_CH12_START_CHANNEL + i));
#if GRAY_4067_INTER_CHANNEL_DELAY_US
        if(i + 1u < (uint8_t)GRAY_4067_DEFAULT_CHANNEL_COUNT)
        {
            system_delay_us(GRAY_4067_INTER_CHANNEL_DELAY_US);
        }
#endif
    }

#if GRAY_4067_BATCH_RESTORE_TO_START
    gray_4067_set_channel((uint8_t)GRAY_4067_CH12_START_CHANNEL);
#if GRAY_4067_BATCH_RESTORE_PRIME_SAMPLE
    (void)adc_convert(GRAY_4067_ADC_CHANNEL);
#endif
#endif
}
