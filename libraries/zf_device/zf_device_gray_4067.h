/*********************************************************************************************************************
* TC264 Opensourec Library (TC264 开源库)
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件为：74HC4067 16路模拟多路复用器 + ADC 采样的最小驱动
* 用途：配合 16 路灰度(光电管)传感器模块（默认 Y0~Y15），实现通道选择与ADC采样。
*
* 接线约定（可按需修改下方宏）：
*   74HC4067   ->  MCU
*   S0..S3     ->  GPIO 输出
*   /EN        ->  GPIO 输出（默认低使能）
*   SIG/COM    ->  ADC 输入（单通道）
********************************************************************************************************************/

#ifndef _zf_device_gray_4067_h_
#define _zf_device_gray_4067_h_

#include "zf_common/zf_common_typedef.h"
#include "zf_driver/zf_driver_gpio.h"
#include "zf_driver/zf_driver_adc.h"
#include "zf_driver/zf_driver_delay.h"

//=================================================配置：引脚选择（根据枚举自行决定）==================================

#ifndef GRAY_4067_S0_PIN
#define GRAY_4067_S0_PIN             (P00_0)
#endif
#ifndef GRAY_4067_S1_PIN
#define GRAY_4067_S1_PIN             (P00_1)
#endif
#ifndef GRAY_4067_S2_PIN
#define GRAY_4067_S2_PIN             (P00_2)
#endif
#ifndef GRAY_4067_S3_PIN
#define GRAY_4067_S3_PIN             (P00_3)
#endif

#ifndef GRAY_4067_EN_PIN
#define GRAY_4067_EN_PIN             (P00_4)          // 74HC4067 的 /EN
#endif

#ifndef GRAY_4067_ADC_CHANNEL
#define GRAY_4067_ADC_CHANNEL        (ADC0_CH6_A6)
#endif

#ifndef GRAY_4067_ADC_RESOLUTION
#define GRAY_4067_ADC_RESOLUTION     (ADC_12BIT)
#endif

#ifndef GRAY_4067_EN_ACTIVE_LEVEL
#define GRAY_4067_EN_ACTIVE_LEVEL    (GPIO_LOW)
#endif

#ifndef GRAY_4067_SETTLE_US
#define GRAY_4067_SETTLE_US          (3)
#endif

#ifndef GRAY_4067_SWITCH_WITH_DISABLE
#define GRAY_4067_SWITCH_WITH_DISABLE     (1)
#endif

#ifndef GRAY_4067_DISABLE_GUARD_US
#define GRAY_4067_DISABLE_GUARD_US        (1)
#endif

#ifndef GRAY_4067_DISCARD_SAMPLES
#ifdef GRAY_4067_DISCARD_FIRST_SAMPLE
#define GRAY_4067_DISCARD_SAMPLES         ((GRAY_4067_DISCARD_FIRST_SAMPLE) ? (1u) : (0u))
#else
#define GRAY_4067_DISCARD_SAMPLES         (1u)
#endif
#endif

#ifndef GRAY_4067_POST_DISCARD_DELAY_US
#define GRAY_4067_POST_DISCARD_DELAY_US  (2)
#endif

#ifndef GRAY_4067_INTER_CHANNEL_DELAY_US
#define GRAY_4067_INTER_CHANNEL_DELAY_US (1)
#endif

#ifndef GRAY_4067_CH0_CH15_EXTRA_SETTLE_US
#define GRAY_4067_CH0_CH15_EXTRA_SETTLE_US (0)
#endif

#ifndef GRAY_4067_BATCH_RESTORE_TO_START
#define GRAY_4067_BATCH_RESTORE_TO_START   (1)
#endif

#ifndef GRAY_4067_BATCH_RESTORE_PRIME_SAMPLE
#define GRAY_4067_BATCH_RESTORE_PRIME_SAMPLE (1)
#endif

#ifndef GRAY_4067_DEFAULT_CHANNEL_COUNT
#define GRAY_4067_DEFAULT_CHANNEL_COUNT  (16)
#endif

#ifndef GRAY_4067_CH12_START_CHANNEL
#define GRAY_4067_CH12_START_CHANNEL     (0u)
#endif

#if ((GRAY_4067_CH12_START_CHANNEL + GRAY_4067_DEFAULT_CHANNEL_COUNT) > 16u)
#error "GRAY_4067 channel config invalid: start + count must be <= 16"
#endif

#ifndef GRAY_4067_SAMPLE_AVG_COUNT
#define GRAY_4067_SAMPLE_AVG_COUNT       (1u)
#endif


//=================================================接口==================================
void    gray_4067_init              (void);
void    gray_4067_enable            (uint8_t enable);
void    gray_4067_set_channel       (uint8_t channel);
uint16_t gray_4067_read_raw         (uint8_t channel);
void    gray_4067_read_all_raw      (uint16_t out_buffer[16]);
void    gray_4067_read_12_raw       (uint16_t out_buffer[GRAY_4067_DEFAULT_CHANNEL_COUNT]);
//=================================================接口==================================

#endif
