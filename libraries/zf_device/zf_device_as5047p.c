/*********************************************************************************************************************
* AS5047P 磁编码器驱动（SPI）
********************************************************************************************************************/

#include "zf_device_as5047p.h"

#define AS5047P_CMD_READ_BIT     (0x4000u)
#define AS5047P_ADDR_MASK        (0x3FFFu)
#define AS5047P_DATA_MASK        (0x3FFFu)
#define AS5047P_ERR_FLAG_BIT     (0x4000u)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算 16bit 数据偶校验（偶数个 1 返回 1）
// 参数说明     v               16bit 数据
// 返回参数     uint8           1: 偶校验通过  0: 奇校验
// 使用示例     ok = as5047p_even_parity_u16(resp);
// 备注信息     内部函数
//-------------------------------------------------------------------------------------------------------------------
static uint8 as5047p_even_parity_u16(uint16 v)
{
    // 返回 1 表示 v 的 bit[0..15] 中 1 的个数为偶数
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (uint8)((~v) & 1u);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     生成 AS5047P 16bit 命令帧（含偶校验位）
// 参数说明     addr            14bit 寄存器地址
// 参数说明     is_read         1:读 0:写
// 返回参数     uint16          命令帧
// 使用示例     cmd = as5047p_make_command(AS5047P_REG_ANGLEUNC, 1);
// 备注信息     内部函数
//-------------------------------------------------------------------------------------------------------------------
static uint16 as5047p_make_command(uint16 addr, uint8 is_read)
{
    uint16 frame = (uint16)(addr & AS5047P_ADDR_MASK);
    if(is_read)
    {
        frame |= AS5047P_CMD_READ_BIT;
    }

    // 按常见 AMS 绝对编码器协议：bit15 为偶校验位，使整个 16bit 帧的 1 的个数为偶数
    // 先算 bit[0..14] 的 1 的个数，决定是否置位 bit15
    uint16 frame_wo_parity = (uint16)(frame & 0x7FFFu);

    // 如果 bit[0..14] 的 1 的个数为奇数，则置 parity=1，使总数变偶数
    uint16 temp = frame_wo_parity;
    temp ^= temp >> 8;
    temp ^= temp >> 4;
    temp ^= temp >> 2;
    temp ^= temp >> 1;
    
    if(temp & 1u)
    {
        frame |= 0x8000u;
    }
    else
    {
        frame &= 0x7FFFu;
    }

    return frame;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     拉低 CS（带最小保持时间）
// 参数说明     dev             AS5047P 设备
// 返回参数     void
// 使用示例     as5047p_cs_low(dev);
// 备注信息     内部函数
//-------------------------------------------------------------------------------------------------------------------
static void as5047p_cs_low(const as5047p_device_t *dev)
{
    gpio_low(dev->cs_pin);
    if(AS5047P_CS_DELAY_US)
    {
        system_delay_us(AS5047P_CS_DELAY_US);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     拉高 CS（带最小保持时间）
// 参数说明     dev             AS5047P 设备
// 返回参数     void
// 使用示例     as5047p_cs_high(dev);
// 备注信息     内部函数
//-------------------------------------------------------------------------------------------------------------------
static void as5047p_cs_high(const as5047p_device_t *dev)
{
    if(AS5047P_CS_DELAY_US)
    {
        system_delay_us(AS5047P_CS_DELAY_US);
    }
    gpio_high(dev->cs_pin);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     传输一个 16bit 帧（手动控制 CS）
// 参数说明     dev             AS5047P 设备
// 参数说明     tx              发送 16bit 数据
// 返回参数     uint16          接收 16bit 数据
// 使用示例     rx = as5047p_transfer16(dev, tx);
// 备注信息     内部函数
//-------------------------------------------------------------------------------------------------------------------
static uint16 as5047p_transfer16(as5047p_device_t *dev, uint16 tx)
{
    uint16 rx = 0;
    if(dev->use_hw_cs)
    {
        // 硬件 CS：spi_transfer_16bit 内部会根据 spi_init 配置自动控制 CS
        spi_transfer_16bit(dev->spi, &tx, &rx, 1);
    }
    else
    {
        // GPIO 手动 CS
        as5047p_cs_low(dev);
        spi_transfer_16bit(dev->spi, &tx, &rx, 1);
        as5047p_cs_high(dev);
    }
    return rx;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查 AS5047P 返回帧（偶校验 + 错误位）
// 参数说明     resp            返回 16bit 数据
// 返回参数     as5047p_status_enum   状态码
// 使用示例     st = as5047p_check_response(resp);
// 备注信息     内部函数
//-------------------------------------------------------------------------------------------------------------------
static as5047p_status_enum as5047p_check_response(uint16 resp)
{
    // 检查偶校验：整个 16bit 帧应为偶校验
    if(!as5047p_even_parity_u16(resp))
    {
        return AS5047P_ERR_PARITY;
    }
    // 检查错误位
    if(resp & AS5047P_ERR_FLAG_BIT)
    {
        return AS5047P_ERR_DEVICE;
    }
    return AS5047P_OK;
}

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
                  uint32 baud)
{
    if(!dev)
    {
        return;
    }

    dev->spi = spi;
    dev->sck_pin = sck_pin;
    dev->mosi_pin = mosi_pin;
    dev->miso_pin = miso_pin;
    dev->use_hw_cs = 0;
    dev->cs_pin = cs_pin;
    dev->hw_cs_pin = SPI_CS_NULL;
    dev->mode = mode;
    dev->baud = baud;

    spi_init(dev->spi, dev->mode, dev->baud, dev->sck_pin, dev->mosi_pin, dev->miso_pin, SPI_CS_NULL);

    gpio_init(dev->cs_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    dev->inited = 1;

    {
        uint16 dummy = 0;
        if(AS5047P_OK != as5047p_read_register(dev, AS5047P_REG_ANGLEUNC, &dummy))
        {
            dev->inited = 0;
        }
    }
}

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
                       uint32 baud)
{
    if(!dev)
    {
        return;
    }

    dev->spi = spi;
    dev->sck_pin = sck_pin;
    dev->mosi_pin = mosi_pin;
    dev->miso_pin = miso_pin;
    dev->use_hw_cs = 1;
    dev->hw_cs_pin = cs_pin;
    dev->cs_pin = (gpio_pin_enum)0;
    dev->mode = mode;
    dev->baud = baud;

    spi_init(dev->spi, dev->mode, dev->baud, dev->sck_pin, dev->mosi_pin, dev->miso_pin, dev->hw_cs_pin);

    dev->inited = 1;

    {
        uint16 dummy = 0;
        if(AS5047P_OK != as5047p_read_register(dev, AS5047P_REG_ANGLEUNC, &dummy))
        {
            dev->inited = 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     使用默认 SPI 配置初始化 AS5047P（CS 由调用者指定）
// 参数说明     dev             AS5047P 设备
// 参数说明     cs_pin          片选引脚（GPIO 手动控制）
// 返回参数     void
// 使用示例     as5047p_init_default(&enc, P11_10);
// 备注信息     默认 SPI 口/引脚由宏 AS5047P_DEFAULT_* 指定
//-------------------------------------------------------------------------------------------------------------------
void as5047p_init_default(as5047p_device_t *dev, gpio_pin_enum cs_pin)
{
    as5047p_init(dev,
                AS5047P_DEFAULT_SPI,
                AS5047P_DEFAULT_SCK_PIN,
                AS5047P_DEFAULT_MOSI_PIN,
                AS5047P_DEFAULT_MISO_PIN,
                cs_pin,
                AS5047P_SPI_MODE,
                AS5047P_SPI_BAUD);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     默认初始化两路 AS5047P（使用默认 CS1/CS2）
// 参数说明     dev1            设备 1
// 参数说明     dev2            设备 2
// 返回参数     void
// 使用示例     as5047p_dual_init_default(&enc_l, &enc_r);
// 备注信息     便于双电机项目快速使用
//-------------------------------------------------------------------------------------------------------------------
void as5047p_dual_init_default(as5047p_device_t *dev1, as5047p_device_t *dev2)
{
    as5047p_init_default(dev1, AS5047P_DEFAULT_CS1_PIN);
    as5047p_init_default(dev2, AS5047P_DEFAULT_CS2_PIN);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 AS5047P 寄存器（两帧流水读）
// 参数说明     dev             AS5047P 设备
// 参数说明     addr            14bit 寄存器地址
// 参数说明     out_data        输出 14bit 数据
// 返回参数     as5047p_status_enum   状态码
// 使用示例     st = as5047p_read_register(&enc, AS5047P_REG_ANGLEUNC, &data);
// 备注信息     会检查偶校验与错误位 EF
//-------------------------------------------------------------------------------------------------------------------
as5047p_status_enum as5047p_read_register(as5047p_device_t *dev, uint16 addr, uint16 *out_data)
{
    if((!dev) || (!out_data) || (!dev->inited))
    {
        return AS5047P_ERR_PARAM;
    }

    uint16 cmd = as5047p_make_command(addr, 1);
    uint16 nop = as5047p_make_command(0x0000u, 0);

    // 第一次发读命令，返回的是上一条命令的响应（忽略但可检查）
    (void)as5047p_transfer16(dev, cmd);

    // 第二次发 NOP，读取本次命令返回数据
    uint16 resp = as5047p_transfer16(dev, nop);

    as5047p_status_enum st = as5047p_check_response(resp);
    if(st != AS5047P_OK)
    {
        return st;
    }

    *out_data = (uint16)(resp & AS5047P_DATA_MASK);
    return AS5047P_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 14bit 原始角度（ANGLEUNC，0~16383）
// 参数说明     dev             AS5047P 设备
// 参数说明     angle_14bit     输出角度原始值
// 返回参数     as5047p_status_enum   状态码
// 使用示例     as5047p_read_angle_raw(&enc, &angle);
// 备注信息     返回的是绝对角度原始计数（未做单位换算）
//-------------------------------------------------------------------------------------------------------------------
as5047p_status_enum as5047p_read_angle_raw(as5047p_device_t *dev, uint16 *angle_14bit)
{
    uint16 data = 0;
    as5047p_status_enum st = as5047p_read_register(dev, AS5047P_REG_ANGLEUNC, &data);
    if(st != AS5047P_OK)
    {
        return st;
    }
    *angle_14bit = (uint16)(data & AS5047P_DATA_MASK);
    return AS5047P_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置转速估算器状态
// 参数说明     est             转速估算器
// 返回参数     void
// 使用示例     as5047p_speed_estimator_reset(&est);
// 备注信息     第一次 update 将返回 0
//-------------------------------------------------------------------------------------------------------------------
void as5047p_speed_estimator_reset(as5047p_speed_estimator_t *est)
{
    if(!est)
    {
        return;
    }
    est->last_angle_14bit = 0;
    est->has_last = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     转速解析（RPM）：由相邻角度差分估算并处理回绕
// 参数说明     est             转速估算器
// 参数说明     angle_14bit     当前角度原始值(0~16383)
// 参数说明     dt_s            采样周期（秒）
// 返回参数     float           转速（RPM，带符号）
// 使用示例     rpm = as5047p_speed_estimator_update_rpm(&est, angle, 0.001f);
// 备注信息     需等间隔采样；角度递增为正方向
//-------------------------------------------------------------------------------------------------------------------
float as5047p_speed_estimator_update_rpm(as5047p_speed_estimator_t *est, uint16 angle_14bit, float dt_s)
{
    if((!est) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    // AS5047P 角度为 14bit（0~16383）
    angle_14bit = (uint16)(angle_14bit & 0x3FFFu);

    if(!est->has_last)
    {
        est->last_angle_14bit = angle_14bit;
        est->has_last = 1;
        return 0.0f;
    }

    int32 delta = (int32)angle_14bit - (int32)est->last_angle_14bit;
    est->last_angle_14bit = angle_14bit;

    // 处理回绕：取最短角度差（单位：count）
    const int32 half_cpr = (int32)(AS5047P_ANGLE_CPR / 2);
    if(delta > half_cpr)
    {
        delta -= (int32)AS5047P_ANGLE_CPR;
    }
    else if(delta < -half_cpr)
    {
        delta += (int32)AS5047P_ANGLE_CPR;
    }

    // count -> rev -> rpm
    const float k = 60.0f / ((float)AS5047P_ANGLE_CPR * dt_s);
    return (float)delta * k;
}
