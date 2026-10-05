#include "key.h"

#define KEY_HOLD_TIMEOUT_MS 3000u

void keys_init(void)
{
    gpio_init(KEY1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

uint8_t get_key_value(void)
{
    uint32_t elapsed;

    if (!gpio_get_level(KEY1_PIN))
    {
        system_delay_ms(20);
        elapsed = 0u;
        while (!gpio_get_level(KEY1_PIN))
        {
            system_delay_ms(1);
            if (++elapsed >= KEY_HOLD_TIMEOUT_MS) { return 1; }
        }
        return 1;
    }
    if (!gpio_get_level(KEY2_PIN))
    {
        system_delay_ms(20);
        elapsed = 0u;
        while (!gpio_get_level(KEY2_PIN))
        {
            system_delay_ms(1);
            if (++elapsed >= KEY_HOLD_TIMEOUT_MS) { return 2; }
        }
        return 2;
    }
    if (!gpio_get_level(KEY3_PIN))
    {
        system_delay_ms(20);
        elapsed = 0u;
        while (!gpio_get_level(KEY3_PIN))
        {
            system_delay_ms(1);
            if (++elapsed >= KEY_HOLD_TIMEOUT_MS) { return 3; }
        }
        return 3;
    }
    if (!gpio_get_level(KEY4_PIN))
    {
        system_delay_ms(20);
        elapsed = 0u;
        while (!gpio_get_level(KEY4_PIN))
        {
            system_delay_ms(1);
            if (++elapsed >= KEY_HOLD_TIMEOUT_MS) { return 4; }
        }
        return 4;
    }
    return 0;
}
