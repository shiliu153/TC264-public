/*********************************************************************************************************************
* TC264 Opensourec Library 锟斤拷锟斤拷TC264 锟斤拷源锟解）锟斤拷一锟斤拷锟斤拷锟节官凤拷 SDK 锟接口的碉拷锟斤拷锟斤拷锟斤拷源锟斤拷
* Copyright (c) 2022 SEEKFREE 锟斤拷煽萍锟�
*
* 锟斤拷锟侥硷拷锟斤拷 TC264 锟斤拷源锟斤拷锟揭伙拷锟斤拷锟�
*
* TC264 锟斤拷源锟斤拷 锟斤拷锟斤拷锟斤拷锟斤拷锟�
* 锟斤拷锟斤拷锟皆革拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷岱拷锟斤拷锟� GPL锟斤拷GNU General Public License锟斤拷锟斤拷 GNU通锟矫癸拷锟斤拷锟斤拷锟斤拷证锟斤拷锟斤拷锟斤拷锟斤拷
* 锟斤拷 GPL 锟侥碉拷3锟芥（锟斤拷 GPL3.0锟斤拷锟斤拷锟斤拷选锟斤拷模锟斤拷魏魏锟斤拷锟斤拷陌姹撅拷锟斤拷锟斤拷路锟斤拷锟斤拷锟�/锟斤拷锟睫革拷锟斤拷
*
* 锟斤拷锟斤拷源锟斤拷姆锟斤拷锟斤拷锟较ｏ拷锟斤拷锟斤拷芊锟斤拷锟斤拷锟斤拷茫锟斤拷锟斤拷锟轿达拷锟斤拷锟斤拷锟斤拷魏蔚谋锟街�
* 锟斤拷锟斤拷没锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟皆伙拷锟绞猴拷锟截讹拷锟斤拷途锟侥憋拷证
* 锟斤拷锟斤拷细锟斤拷锟斤拷渭锟� GPL
*
* 锟斤拷应锟斤拷锟斤拷锟秸碉拷锟斤拷锟斤拷源锟斤拷锟酵憋拷盏锟揭伙拷锟� GPL 锟侥革拷锟斤拷
* 锟斤拷锟矫伙拷校锟斤拷锟斤拷锟斤拷<https://www.gnu.org/licenses/>
*
* 锟斤拷锟斤拷注锟斤拷锟斤拷
* 锟斤拷锟斤拷源锟斤拷使锟斤拷 GPL3.0 锟斤拷源锟斤拷锟斤拷证协锟斤拷 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷为锟斤拷锟侥版本
* 锟斤拷锟斤拷锟斤拷锟斤拷英锟侥帮拷锟斤拷 libraries/doc 锟侥硷拷锟斤拷锟铰碉拷 GPL3_permission_statement.txt 锟侥硷拷锟斤拷
* 锟斤拷锟斤拷证锟斤拷锟斤拷锟斤拷 libraries 锟侥硷拷锟斤拷锟斤拷 锟斤拷锟斤拷锟侥硷拷锟斤拷锟铰碉拷 LICENSE 锟侥硷拷
* 锟斤拷迎锟斤拷位使锟矫诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 锟斤拷锟睫革拷锟斤拷锟斤拷时锟斤拷锟诫保锟斤拷锟斤拷煽萍锟斤拷陌锟饺拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
*
* 锟侥硷拷锟斤拷锟斤拷          zf_common_headfile
* 锟斤拷司锟斤拷锟斤拷          锟缴讹拷锟斤拷煽萍锟斤拷锟斤拷薰锟剿�
* 锟芥本锟斤拷息          锟介看 libraries/doc 锟侥硷拷锟斤拷锟斤拷 version 锟侥硷拷 锟芥本说锟斤拷
* 锟斤拷锟斤拷锟斤拷锟斤拷          ADS v1.10.2
* 锟斤拷锟斤拷平台          TC264D
* 锟斤拷锟斤拷锟斤拷锟斤拷          https://seekfree.taobao.com/
*
* 锟睫改硷拷录
* 锟斤拷锟斤拷              锟斤拷锟斤拷                锟斤拷注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_

//===================================================C锟斤拷锟斤拷 锟斤拷锟斤拷锟斤拷===================================================
#include "math.h"
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "pid.h"
//===================================================C锟斤拷锟斤拷 锟斤拷锟斤拷锟斤拷===================================================

//===================================================芯片 SDK 锟阶诧拷===================================================
#include "ifxAsclin_reg.h"
#include "SysSe/Bsp/Bsp.h"
#include "IfxCcu6_Timer.h"
#include "IfxScuEru.h"
//===================================================芯片 SDK 锟阶诧拷===================================================

//====================================================锟斤拷源锟解公锟斤拷锟斤拷====================================================
#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_common_font.h"
#include "zf_common_function.h"
#include "zf_common_interrupt.h"
#include "isr_config.h"
//====================================================锟斤拷源锟解公锟斤拷锟斤拷====================================================

//===================================================芯片锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷===================================================
#include "zf_driver_adc.h"
#include "zf_driver_delay.h"
#include "zf_driver_dma.h"
#include "zf_driver_encoder.h"
#include "zf_driver_exti.h"
#include "zf_driver_flash.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"
#include "zf_driver_pwm.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_spi.h"
#include "zf_driver_spi_multi.h"
#include "zf_driver_soft_spi.h"
#include "zf_driver_uart.h"
#include "zf_driver_timer.h"
//===================================================芯片锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷===================================================

//===================================================锟斤拷锟斤拷璞革拷锟斤拷锟斤拷锟�===================================================
#include "zf_device_absolute_encoder.h"
#include "zf_device_ble6a20.h"
#include "zf_device_bluetooth_ch9141.h"
#include "zf_device_gnss.h"
#include "zf_device_camera.h"
#include "zf_device_dl1a.h"
#include "zf_device_dl1b.h"
#include "zf_device_as5047p.h"
#include "zf_device_icm20602.h"
#include "zf_device_imu660ra.h"
#include "zf_device_imu660rb.h"
#include "zf_device_imu660rx.h"
#include "zf_device_imu963ra.h"
#include "zf_device_ips114.h"
#include "zf_device_ips200.h"
#include "zf_device_ips200pro.h"
#include "zf_device_menc15a.h"
#include "zf_device_mpu6050.h"
#include "zf_device_oled.h"
#include "zf_device_ov7725.h"
#include "zf_device_scc8660.h"
#include "zf_device_tft180.h"
#include "zf_device_gray_4067.h"
#include "zf_device_tsl1401.h"
#include "zf_device_type.h"
#include "zf_device_uart_receiver.h"
#include "zf_device_virtual_oscilloscope.h"
#include "zf_device_wifi_uart.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_wireless_uart.h"
//===================================================锟斤拷锟斤拷璞革拷锟斤拷锟斤拷锟�===================================================

//====================================================应锟斤拷锟斤拷锟斤拷锟�====================================================
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
//====================================================应锟斤拷锟斤拷锟斤拷锟�====================================================

//====================================================鐢ㄦ埛宸ョ▼妯″潡====================================================
#include "bldc_drv8300_3pwm.h"
#include "foc_svpwm.h"
//====================================================鐢ㄦ埛宸ョ▼妯″潡====================================================

//=====================================================锟矫伙拷锟斤拷======================================================

//=====================================================锟矫伙拷锟斤拷======================================================

#endif

