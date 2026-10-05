/*********************************************************************************************************************
* 文件：key.c
* 模块：应用层（第 6 层）—— 按键输入
* 职责：4 键消抖输入
* 依赖：zf_driver_gpio.h、本目录 key.h
* 风险等级：R1
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "key.h"
void keys_init(void)
{
    gpio_init(KEY1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

uint8_t get_key_value(void)
{
    if(!gpio_get_level(KEY1_PIN)) { system_delay_ms(20); while(!gpio_get_level(KEY1_PIN)); return 1; }
    if(!gpio_get_level(KEY2_PIN)) { system_delay_ms(20); while(!gpio_get_level(KEY2_PIN)); return 2; }
    if(!gpio_get_level(KEY3_PIN)) { system_delay_ms(20); while(!gpio_get_level(KEY3_PIN)); return 3; }
    if(!gpio_get_level(KEY4_PIN)) { system_delay_ms(20); while(!gpio_get_level(KEY4_PIN)); return 4; }
    return 0;
}
