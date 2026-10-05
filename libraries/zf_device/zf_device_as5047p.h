#ifndef _zf_device_as5047p_h_
#define _zf_device_as5047p_h_

#include "zf_common_typedef.h"
#include "zf_driver/zf_driver_gpio.h"
#include "zf_driver/zf_driver_spi.h"
#include "zf_driver/zf_driver_delay.h"

//=================================================AS5047P 帧格式=================================================
// 命令帧（MCU->AS5047P）：
// [15] PARITY (偶校验)
// [14] R/W (1=read, 0=write)
// [13:0] ADDR
//
// 数据帧（AS5047P->MCU）：
// [15] PARITY (偶校验)
// [14] EF (error flag)
// [13:0] DATA
//=================================================AS5047P 帧格式=================================================

// 常用寄存器地址
#define AS5047P_REG_ANGLEUNC            (0x3FFF)
#define AS5047P_REG_ANGLECOM            (0x3FFE)

#ifndef AS5047P_SPI_MODE
#define AS5047P_SPI_MODE                (SPI_MODE1)
#endif
#ifndef AS5047P_SPI_BAUD
#define AS5047P_SPI_BAUD                (10u * 1000u * 1000u)
#endif

#ifndef AS5047P_DEFAULT_SPI
#define AS5047P_DEFAULT_SPI              (SPI_3)
#endif

#ifndef AS5047P_DEFAULT_SCK_PIN
#define AS5047P_DEFAULT_SCK_PIN          (SPI3_SCLK_P33_11)
#endif

#ifndef AS5047P_DEFAULT_MOSI_PIN
#define AS5047P_DEFAULT_MOSI_PIN         (SPI3_MOSI_P33_12)
#endif

#ifndef AS5047P_DEFAULT_MISO_PIN
#define AS5047P_DEFAULT_MISO_PIN         (SPI3_MISO_P33_13)
#endif

// 两路编码器默认 CS
#ifndef AS5047P_DEFAULT_CS1_PIN
#define AS5047P_DEFAULT_CS1_PIN          (P33_9)
#endif

#ifndef AS5047P_DEFAULT_CS2_PIN
#define AS5047P_DEFAULT_CS2_PIN          (P33_8)
#endif

// SPI 片选保持时间（最小延时，单位 us）
#ifndef AS5047P_CS_DELAY_US
#define AS5047P_CS_DELAY_US             (1u)
#endif

// 返回码
typedef enum
{
    AS5047P_OK = 0,
    AS5047P_ERR_PARITY = 1,
    AS5047P_ERR_DEVICE = 2,
    AS5047P_ERR_PARAM  = 3,
}as5047p_status_enum;

typedef struct
{
    spi_index_enum       spi;
    spi_mode_enum        mode;
    uint32               baud;
    spi_sck_pin_enum     sck_pin;
    spi_mosi_pin_enum    mosi_pin;
    spi_miso_pin_enum    miso_pin;

    // CS 支持两种方式：
    // 1) use_hw_cs=0：使用 GPIO 手动控制（兼容旧接口）
    // 2) use_hw_cs=1：使用 SPI 硬件片选 spi_cs_pin_enum（由底层 QSPI 自动拉低/拉高）
    uint8                use_hw_cs;
    gpio_pin_enum        cs_pin;
    spi_cs_pin_enum      hw_cs_pin;

    uint8                inited;
}as5047p_device_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 AS5047P 设备（配置 SPI + CS 引脚）
// 参数说明     dev             AS5047P 设备
// 参数说明     spi             SPI 号
// 参数说明     sck_pin         SCK 引脚
// 参数说明     mosi_pin        MOSI 引脚
// 参数说明     miso_pin        MISO 引脚
// 参数说明     cs_pin          片选引脚（GPIO 手动控制）
// 参数说明     mode            SPI 模式
// 参数说明     baud            SPI 波特率
// 返回参数     void
// 使用示例     as5047p_init(&enc, SPI_1, ..., P11_10, SPI_MODE1, 10000000);
// 备注信息     初始化会发送一次 NOP 清流水
//-------------------------------------------------------------------------------------------------------------------
void as5047p_init(as5047p_device_t *dev,
                  spi_index_enum spi,
                  spi_sck_pin_enum sck_pin,
                  spi_mosi_pin_enum mosi_pin,
                  spi_miso_pin_enum miso_pin,
                  gpio_pin_enum cs_pin,
                  spi_mode_enum mode,
                  uint32 baud);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 AS5047P 设备（SPI 硬件片选方式，使用 spi_cs_pin_enum）
// 参数说明     dev             AS5047P 设备
// 参数说明     spi             SPI 号
// 参数说明     sck_pin         SCK 引脚
// 参数说明     mosi_pin        MOSI 引脚
// 参数说明     miso_pin        MISO 引脚
// 参数说明     cs_pin          SPI 硬件片选（spi_cs_pin_enum，如 SPI3_CS1_P33_9）
// 参数说明     mode            SPI 模式
// 参数说明     baud            SPI 波特率
// 返回参数     void
// 使用示例     as5047p_init_hwcs(&enc, SPI_3, ..., SPI3_CS1_P33_9, SPI_MODE1, 10000000);
// 备注信息     初始化会发送一次 NOP 清流水
//-------------------------------------------------------------------------------------------------------------------
void as5047p_init_hwcs(as5047p_device_t *dev,
                       spi_index_enum spi,
                       spi_sck_pin_enum sck_pin,
                       spi_mosi_pin_enum mosi_pin,
                       spi_miso_pin_enum miso_pin,
                       spi_cs_pin_enum cs_pin,
                       spi_mode_enum mode,
                       uint32 baud);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     使用默认 SPI 配置初始化 AS5047P（CS 由调用者指定）
// 参数说明     dev             AS5047P 设备
// 参数说明     cs_pin          片选引脚（GPIO 手动控制）
// 返回参数     void
// 使用示例     as5047p_init_default(&enc, P11_10);
// 备注信息     默认 SPI 口/引脚由宏 AS5047P_DEFAULT_* 指定
//-------------------------------------------------------------------------------------------------------------------
void as5047p_init_default(as5047p_device_t *dev, gpio_pin_enum cs_pin);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     默认初始化两路 AS5047P（使用默认 CS1/CS2）
// 参数说明     dev1            设备 1
// 参数说明     dev2            设备 2
// 返回参数     void
// 使用示例     as5047p_dual_init_default(&enc_l, &enc_r);
// 备注信息     便于双电机项目快速使用
//-------------------------------------------------------------------------------------------------------------------
void as5047p_dual_init_default(as5047p_device_t *dev1, as5047p_device_t *dev2);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 14bit 原始角度（ANGLEUNC，0~16383）
// 参数说明     dev             AS5047P 设备
// 参数说明     angle_14bit     输出角度原始值
// 返回参数     as5047p_status_enum   状态码
// 使用示例     as5047p_read_angle_raw(&enc, &angle);
// 备注信息     返回的是绝对角度原始计数（未做单位换算）
//-------------------------------------------------------------------------------------------------------------------
as5047p_status_enum as5047p_read_angle_raw(as5047p_device_t *dev, uint16 *angle_14bit);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 AS5047P 寄存器（两帧流水读）
// 参数说明     dev             AS5047P 设备
// 参数说明     addr            14bit 寄存器地址
// 参数说明     out_data        输出 14bit 数据
// 返回参数     as5047p_status_enum   状态码
// 使用示例     st = as5047p_read_register(&enc, AS5047P_REG_ANGLEUNC, &data);
// 备注信息     会检查偶校验与错误位 EF
//-------------------------------------------------------------------------------------------------------------------
as5047p_status_enum as5047p_read_register(as5047p_device_t *dev, uint16 addr, uint16 *out_data);

//=================================================AS5047P 转速解析=================================================
// 基于相邻两次 14bit 角度差分估算转速（RPM），自动处理 0/16383 回绕。
// 说明：该算法依赖固定周期 dt_s（秒）以及连续采样；第一次调用返回 0。

#ifndef AS5047P_ANGLE_CPR
#define AS5047P_ANGLE_CPR               (16384)
#endif

typedef struct
{
    uint16 last_angle_14bit;
    uint8  has_last;
}as5047p_speed_estimator_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置转速估算器状态
// 参数说明     est             转速估算器
// 返回参数     void
// 使用示例     as5047p_speed_estimator_reset(&est);
// 备注信息     第一次 update 将返回 0
//-------------------------------------------------------------------------------------------------------------------
void as5047p_speed_estimator_reset(as5047p_speed_estimator_t *est);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     转速解析（RPM）：由相邻角度差分估算并处理回绕
// 参数说明     est             转速估算器
// 参数说明     angle_14bit     当前角度原始值(0~16383)
// 参数说明     dt_s            采样周期（秒）
// 返回参数     float           转速（RPM，带符号）
// 使用示例     rpm = as5047p_speed_estimator_update_rpm(&est, angle, 0.001f);
// 备注信息     需等间隔采样；角度递增为正方向
//-------------------------------------------------------------------------------------------------------------------
float as5047p_speed_estimator_update_rpm(as5047p_speed_estimator_t *est, uint16 angle_14bit, float dt_s);

#endif
