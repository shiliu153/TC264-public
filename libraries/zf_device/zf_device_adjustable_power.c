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

#include "zf_device_adjustable_power.h"
#include "zf_common_debug.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     可调电源初始化
// 参数说明     dev             可调电源设备指针
// 参数说明     ch              PWM 通道（单个半桥）
// 参数说明     freq_hz         PWM 频率 (Hz)，0 则使用默认 20kHz
// 返回参数     void
// 使用示例     adjustable_power_t ps; adjustable_power_init(&ps, ATOM0_CH0_P00_0, 20000);
// 备注信息     初始化后输出关闭（duty=0），需调 enable 开启
//-------------------------------------------------------------------------------------------------------------------
void adjustable_power_init(adjustable_power_t *dev, pwm_channel_enum ch, uint32 freq_hz)
{
    if(!dev)
    {
        return;
    }

    dev->channel      = ch;
    dev->freq_hz      = (freq_hz == 0u) ? ADJUSTABLE_POWER_DEFAULT_FREQ_HZ : freq_hz;
    dev->current_duty = 0u;
    dev->vbus         = 1.0f;
    dev->enabled      = 0u;
    dev->inited       = 1u;

    pwm_init(ch, dev->freq_hz, 0u);
    pwm_set_duty(ch, 0u);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置占空比
// 参数说明     dev             可调电源设备指针
// 参数说明     duty            占空比 (0 ~ ADJUSTABLE_POWER_DUTY_MAX)
// 返回参数     void
// 使用示例     adjustable_power_set_duty(&ps, 5000);   // 50% 占空比
// 备注信息     占空比超出范围会触发断言
//-------------------------------------------------------------------------------------------------------------------
void adjustable_power_set_duty(adjustable_power_t *dev, uint32 duty)
{
    if((!dev) || (!dev->inited))
    {
        return;
    }

    zf_assert(duty <= ADJUSTABLE_POWER_DUTY_MAX);

    dev->current_duty = duty;

    if(dev->enabled)
    {
        pwm_set_duty(dev->channel, duty);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置占空比百分比
// 参数说明     dev             可调电源设备指针
// 参数说明     percent         占空比百分比 (0.0 ~ 100.0)
// 返回参数     void
// 使用示例     adjustable_power_set_duty_percent(&ps, 50.0f);   // 50% 占空比
// 备注信息     百分比超出范围会触发断言
//-------------------------------------------------------------------------------------------------------------------
void adjustable_power_set_duty_percent(adjustable_power_t *dev, float percent)
{
    if((!dev) || (!dev->inited))
    {
        return;
    }

    zf_assert(percent >= 0.0f && percent <= 100.0f);

    uint32 duty = (uint32)(percent * (float)ADJUSTABLE_POWER_DUTY_MAX / 100.0f);
    adjustable_power_set_duty(dev, duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置母线电压（用于电压换算）
// 参数说明     dev             可调电源设备指针
// 参数说明     vbus            母线电压 (V)，必须 > 0
// 返回参数     void
// 使用示例     adjustable_power_set_bus_voltage(&ps, 12.0f);   // 12V 母线
// 备注信息     仅影响后续 set_voltage 的占空比换算，不改变当前输出
//-------------------------------------------------------------------------------------------------------------------
void adjustable_power_set_bus_voltage(adjustable_power_t *dev, float vbus)
{
    if((!dev) || (!dev->inited))
    {
        return;
    }

    zf_assert(vbus > 0.0f);

    dev->vbus = vbus;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置目标输出电压（需先通过 set_bus_voltage 配置母线电压）
// 参数说明     dev             可调电源设备指针
// 参数说明     voltage         目标电压 (V)，范围 0 ~ vbus
// 返回参数     void
// 使用示例     adjustable_power_set_bus_voltage(&ps, 12.0f);
//              adjustable_power_set_voltage(&ps, 5.0f);   // 输出 5V
// 备注信息     内部自动换算为占空比 = voltage / vbus * PWM_DUTY_MAX
//-------------------------------------------------------------------------------------------------------------------
void adjustable_power_set_voltage(adjustable_power_t *dev, float voltage)
{
    if((!dev) || (!dev->inited))
    {
        return;
    }

    zf_assert(voltage >= 0.0f);
    zf_assert(voltage <= dev->vbus);

    uint32 duty = (uint32)(voltage / dev->vbus * (float)ADJUSTABLE_POWER_DUTY_MAX);
    adjustable_power_set_duty(dev, duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     使能输出
// 参数说明     dev             可调电源设备指针
// 返回参数     void
// 使用示例     adjustable_power_enable(&ps);
// 备注信息     将 current_duty 写入 PWM 通道
//-------------------------------------------------------------------------------------------------------------------
void adjustable_power_enable(adjustable_power_t *dev)
{
    if((!dev) || (!dev->inited))
    {
        return;
    }

    dev->enabled = 1u;
    pwm_set_duty(dev->channel, dev->current_duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     关闭输出
// 参数说明     dev             可调电源设备指针
// 返回参数     void
// 使用示例     adjustable_power_disable(&ps);
// 备注信息     将 PWM 占空比置零，保留 current_duty 不变
//-------------------------------------------------------------------------------------------------------------------
void adjustable_power_disable(adjustable_power_t *dev)
{
    if((!dev) || (!dev->inited))
    {
        return;
    }

    dev->enabled = 0u;
    pwm_set_duty(dev->channel, 0u);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前占空比
// 参数说明     dev             可调电源设备指针
// 返回参数     uint32          当前占空比 (0 ~ ADJUSTABLE_POWER_DUTY_MAX)
// 使用示例     uint32 d = adjustable_power_get_duty(&ps);
// 备注信息     返回的是 internal current_duty，与是否 enable 无关
//-------------------------------------------------------------------------------------------------------------------
uint32 adjustable_power_get_duty(adjustable_power_t *dev)
{
    if((!dev) || (!dev->inited))
    {
        return 0u;
    }

    return dev->current_duty;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前输出电压估算值
// 参数说明     dev             可调电源设备指针
// 返回参数     float           估算电压 (V)
// 使用示例     float v = adjustable_power_get_voltage(&ps);
// 备注信息     V_est = current_duty * vbus / PWM_DUTY_MAX，仅供参考
//-------------------------------------------------------------------------------------------------------------------
float adjustable_power_get_voltage(adjustable_power_t *dev)
{
    if((!dev) || (!dev->inited))
    {
        return 0.0f;
    }

    return (float)dev->current_duty * dev->vbus / (float)ADJUSTABLE_POWER_DUTY_MAX;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询使能状态
// 参数说明     dev             可调电源设备指针
// 返回参数     uint8           1 = 已使能，0 = 未使能
// 使用示例     if(adjustable_power_is_enabled(&ps)) { ... }
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 adjustable_power_is_enabled(adjustable_power_t *dev)
{
    if((!dev) || (!dev->inited))
    {
        return 0u;
    }

    return dev->enabled;
}
