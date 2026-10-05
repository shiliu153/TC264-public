/*********************************************************************************************************************
* TC264 Opensourec Library （TC264 开源库） —— 一个基于官方 SDK 接口的底层开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 的许可证信息
* 您可以自由根据实际需求将本软件应用于任何非商业用途，但必须保留本版权声明信息。
* 亦可以在 GPL（GNU General Public License）或 GNU通用公共许可证（即 GPL 的）第3版
* （即 GPL3.0）或（选择适用的）任何更高版本的条款下发布和/或修改。
*
* 本开源库的发布是希望能够发挥价值，但没有做出任何明示或暗示的担保
* 包括但不限于适销性或者适合特定用途的保证
* 更多详细信息请参见 GPL
*
* 您应该已经接收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 特别声明：
* 本开源库使用 GPL3.0 开源许可协议 如果与上述描述存在冲突以英文版本为准
* 具体可以查看 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件
* 该许可证位于 libraries 文件夹 根目录下的 LICENSE 文件
* 欢迎各位使用并传播本产品 修改产品时请保留逐飞科技的版权声明文件说明
*
* 文件名称          zf_device_adjustable_power
* 公司名称          成都逐飞科技有限责任公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 官方店铺          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2026-07-18       shiliu153           first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块接线           单片机接线
*                   本模块通过 DRV8300 驱动板的单路半桥输出 PWM，经 LC 低通滤波得到可调直流电压。
*                   Vout ≈ duty * Vbus / PWM_DUTY_MAX
*                   PWM 引脚由调用者指定（pwm_channel_enum），详见 zf_driver_pwm.h
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_adjustable_power_h_
#define _zf_device_adjustable_power_h_

#include "zf_common_typedef.h"
#include "zf_driver/zf_driver_pwm.h"

#define ADJUSTABLE_POWER_DUTY_MAX           PWM_DUTY_MAX                                // 占空比满量程 10000

#ifndef ADJUSTABLE_POWER_DEFAULT_FREQ_HZ
#define ADJUSTABLE_POWER_DEFAULT_FREQ_HZ    (20000u)                                    // 默认 PWM 频率 20kHz
#endif

typedef struct
{
    pwm_channel_enum   channel;                                                         // PWM 通道
    uint32             freq_hz;                                                         // PWM 频率 (Hz)
    uint32             current_duty;                                                    // 当前占空比
    float              vbus;                                                            // 母线电压 (V)
    uint8              enabled;                                                         // 使能状态
    uint8              inited;                                                          // 初始化标记
} adjustable_power_t;

//====================================================可调电源 外部接口====================================================
void    adjustable_power_init               (adjustable_power_t *dev, pwm_channel_enum ch, uint32 freq_hz);
void    adjustable_power_set_duty           (adjustable_power_t *dev, uint32 duty);
void    adjustable_power_set_duty_percent   (adjustable_power_t *dev, float percent);
void    adjustable_power_set_bus_voltage    (adjustable_power_t *dev, float vbus);
void    adjustable_power_set_voltage        (adjustable_power_t *dev, float voltage);
void    adjustable_power_enable             (adjustable_power_t *dev);
void    adjustable_power_disable            (adjustable_power_t *dev);
uint32  adjustable_power_get_duty           (adjustable_power_t *dev);
float   adjustable_power_get_voltage        (adjustable_power_t *dev);
uint8   adjustable_power_is_enabled         (adjustable_power_t *dev);
//====================================================可调电源 外部接口====================================================

#endif
