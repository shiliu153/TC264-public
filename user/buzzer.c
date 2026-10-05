/*********************************************************************************************************************
* 文件：buzzer.c
* 模块：应用层（第 6 层）—— 蜂鸣器
* 职责：蜂鸣器 GPIO 输出控制
* 依赖：zf_driver_gpio.h、本目录 buzzer.h
* 风险等级：R0
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "buzzer.h"
#include "zf_common_headfile.h"

void buzzer_init(void)
{
    pwm_init(BUZZER_PIN, BUZZER_FREQ_HZ, 0);
}

void buzzer_beep(uint32_t freq_hz, uint32_t duration_ms)
{
    if (freq_hz == 0u) freq_hz = BUZZER_FREQ_HZ;
    pwm_init(BUZZER_PIN, freq_hz, PWM_DUTY_MAX / 2u);
    system_delay_ms(duration_ms);
    pwm_set_duty(BUZZER_PIN, 0);
}

void buzzer_stop(void)
{
    pwm_set_duty(BUZZER_PIN, 0);
}
