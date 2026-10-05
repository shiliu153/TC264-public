/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_driver_spi
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.10.2
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-15       pudding            first version
* 2023-04-28       pudding            �޸����SPIͬʱʹ�ÿ��ܲ�����ͻ������
* 2025-02-07       pudding            �޸�16λͨѶ���쳣��λ����
********************************************************************************************************************/

#include "IFXQSPI_REGDEF.h"
#include "IfxQspi_SpiMaster.h"
#include "IfxQspi.h"
#include "zf_common_debug.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"

#define MAX_BAUD    50000000
Ifx_QSPI_BACON      bacon[4];
spi_cs_pin_enum     spi_cs_pin;

void spi_mux (spi_index_enum spi_n, spi_sck_pin_enum sck_pin, spi_mosi_pin_enum mosi_pin, spi_miso_pin_enum miso_pin, spi_cs_pin_enum cs_pin, IfxQspi_SpiMaster_Pins *set_pin, IfxQspi_SpiMaster_Output *set_cs)
{
    set_pin->mrstMode  = IfxPort_InputMode_pullDown;
    set_pin->mtsrMode  = IfxPort_OutputMode_pushPull;
    set_pin->sclkMode  = IfxPort_OutputMode_pushPull;
    set_pin->pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    set_cs->driver     = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    set_cs->mode       = IfxPort_OutputMode_pushPull;

    switch(spi_n)
    {
        case SPI_0:
        {
            if      (SPI0_SCLK_P20_11 == sck_pin)   set_pin->sclk = &IfxQspi0_SCLK_P20_11_OUT;
            else if (SPI0_SCLK_P20_13 == sck_pin)   set_pin->sclk = &IfxQspi0_SCLK_P20_13_OUT;
            else    zf_assert(FALSE);

            if      (SPI0_MOSI_P20_12 == mosi_pin)  set_pin->mtsr = &IfxQspi0_MTSR_P20_12_OUT;
            else if (SPI0_MOSI_P20_14 == mosi_pin)  set_pin->mtsr = &IfxQspi0_MTSR_P20_14_OUT;
            else    zf_assert(FALSE);

            if      (SPI0_MISO_P20_12 == miso_pin)  set_pin->mrst = &IfxQspi0_MRSTA_P20_12_IN;
            else    zf_assert(FALSE);

            if      (SPI0_CS0_P20_8   == cs_pin)    set_cs->pin   = &IfxQspi0_SLSO0_P20_8_OUT;
            else if (SPI0_CS1_P20_9   == cs_pin)    set_cs->pin   = &IfxQspi0_SLSO1_P20_9_OUT;
            else if (SPI0_CS2_P20_13  == cs_pin)    set_cs->pin   = &IfxQspi0_SLSO2_P20_13_OUT;
            else if (SPI0_CS3_P11_10  == cs_pin)    set_cs->pin   = &IfxQspi0_SLSO3_P11_10_OUT;
            else if (SPI0_CS4_P11_11  == cs_pin)    set_cs->pin   = &IfxQspi0_SLSO4_P11_11_OUT;
            else if (SPI0_CS5_P11_2   == cs_pin)    set_cs->pin   = &IfxQspi0_SLSO5_P11_2_OUT;
            else if (SPI0_CS6_P20_10  == cs_pin)    set_cs->pin   = &IfxQspi0_SLSO6_P20_10_OUT;
            else if (SPI0_CS7_P33_5   == cs_pin)    set_cs->pin   = &IfxQspi0_SLSO7_P33_5_OUT;
            else if (SPI0_CS8_P20_6   == cs_pin)    set_cs->pin   = &IfxQspi0_SLSO8_P20_6_OUT;
            else if (SPI0_CS9_P20_3   == cs_pin)    set_cs->pin   = &IfxQspi0_SLSO9_P20_3_OUT;
            else if (SPI0_CS13_P15_0  == cs_pin)    set_cs->pin   = &IfxQspi0_SLSO13_P15_0_OUT;
            else if (SPI_CS_NULL      == cs_pin)    set_cs->pin   = NULL_PTR;
            else    zf_assert(FALSE);

        }break;

        case SPI_1:
        {
            if      (SPI1_SCLK_P10_2  == sck_pin)   set_pin->sclk = &IfxQspi1_SCLK_P10_2_OUT;
            else if (SPI1_SCLK_P11_6  == sck_pin)   set_pin->sclk = &IfxQspi1_SCLK_P11_6_OUT;
            else    zf_assert(FALSE);

            if      (SPI1_MOSI_P10_1  == mosi_pin)  set_pin->mtsr = &IfxQspi1_MTSR_P10_1_OUT;
            else if (SPI1_MOSI_P10_3  == mosi_pin)  set_pin->mtsr = &IfxQspi1_MTSR_P10_3_OUT;
            else if (SPI1_MOSI_P11_9  == mosi_pin)  set_pin->mtsr = &IfxQspi1_MTSR_P11_9_OUT;
            else    zf_assert(FALSE);

            if      (SPI1_MISO_P10_1  == miso_pin)  set_pin->mrst = &IfxQspi1_MRSTA_P10_1_IN;
            else if (SPI1_MISO_P11_3  == miso_pin)  set_pin->mrst = &IfxQspi1_MRSTB_P11_3_IN;
            else    zf_assert(FALSE);

            if      (SPI1_CS0_P20_8   == cs_pin )   set_cs->pin   = &IfxQspi1_SLSO0_P20_8_OUT;
            else if (SPI1_CS1_P20_9   == cs_pin )   set_cs->pin   = &IfxQspi1_SLSO1_P20_9_OUT;
            else if (SPI1_CS2_P20_13  == cs_pin )   set_cs->pin   = &IfxQspi1_SLSO2_P20_13_OUT;
            else if (SPI1_CS3_P11_10  == cs_pin )   set_cs->pin   = &IfxQspi1_SLSO3_P11_10_OUT;
            else if (SPI1_CS4_P11_11  == cs_pin )   set_cs->pin   = &IfxQspi1_SLSO4_P11_11_OUT;
            else if (SPI1_CS5_P11_2   == cs_pin )   set_cs->pin   = &IfxQspi1_SLSO5_P11_2_OUT;
            else if (SPI1_CS6_P33_10  == cs_pin )   set_cs->pin   = &IfxQspi1_SLSO6_P33_10_OUT;
            else if (SPI1_CS7_P33_5   == cs_pin )   set_cs->pin   = &IfxQspi1_SLSO7_P33_5_OUT;
            else if (SPI1_CS8_P10_4   == cs_pin )   set_cs->pin   = &IfxQspi1_SLSO8_P10_4_OUT;
            else if (SPI1_CS9_P10_5   == cs_pin )   set_cs->pin   = &IfxQspi1_SLSO9_P10_5_OUT;
            else if (SPI_CS_NULL      == cs_pin)    set_cs->pin   = NULL_PTR;
            else    zf_assert(FALSE);
        }break;

        case SPI_2:
        {
            if      (SPI2_SCLK_P13_0  == sck_pin)   set_pin->sclk = &IfxQspi2_SCLKN_P13_0_OUT;
            else if (SPI2_SCLK_P13_1  == sck_pin)   set_pin->sclk = &IfxQspi2_SCLKP_P13_1_OUT;
            else if (SPI2_SCLK_P15_3  == sck_pin)   set_pin->sclk = &IfxQspi2_SCLK_P15_3_OUT;
            else if (SPI2_SCLK_P15_6  == sck_pin)   set_pin->sclk = &IfxQspi2_SCLK_P15_6_OUT;
            else if (SPI2_SCLK_P15_8  == sck_pin)   set_pin->sclk = &IfxQspi2_SCLK_P15_8_OUT;
            else    zf_assert(FALSE);

            if      (SPI2_MOSI_P13_3  == mosi_pin)  set_pin->mtsr = &IfxQspi2_MTSRP_P13_3_OUT;
            else if (SPI2_MOSI_P15_5  == mosi_pin)  set_pin->mtsr = &IfxQspi2_MTSR_P15_5_OUT;
            else if (SPI2_MOSI_P15_6  == mosi_pin)  set_pin->mtsr = &IfxQspi2_MTSR_P15_6_OUT;
            else    zf_assert(FALSE);

            if      (SPI2_MISO_P15_2  == miso_pin)  set_pin->mrst = &IfxQspi2_MRSTE_P15_2_IN;
            else if (SPI2_MISO_P15_4  == miso_pin)  set_pin->mrst = &IfxQspi2_MRSTA_P15_4_IN;
            else if (SPI2_MISO_P15_7  == miso_pin)  set_pin->mrst = &IfxQspi2_MRSTB_P15_7_IN;
            else if (SPI2_MISO_P21_2  == miso_pin)  set_pin->mrst = &IfxQspi2_MRSTCN_P21_2_IN;
            else if (SPI2_MISO_P21_3  == miso_pin)  set_pin->mrst = &IfxQspi2_MRSTCP_P21_3_IN;
            else    zf_assert(FALSE);

            if      (SPI2_CS0_P15_2   == cs_pin)    set_cs->pin   = &IfxQspi2_SLSO0_P15_2_OUT;
            else if (SPI2_CS1_P14_2   == cs_pin)    set_cs->pin   = &IfxQspi2_SLSO1_P14_2_OUT;
            else if (SPI2_CS2_P14_6   == cs_pin)    set_cs->pin   = &IfxQspi2_SLSO2_P14_6_OUT;
            else if (SPI2_CS3_P14_3   == cs_pin)    set_cs->pin   = &IfxQspi2_SLSO3_P14_3_OUT;
            else if (SPI2_CS5_P15_1   == cs_pin)    set_cs->pin   = &IfxQspi2_SLSO5_P15_1_OUT;
            else if (SPI2_CS6_P33_13  == cs_pin)    set_cs->pin   = &IfxQspi2_SLSO6_P33_13_OUT;
            else if (SPI2_CS7_P20_10  == cs_pin)    set_cs->pin   = &IfxQspi2_SLSO7_P20_10_OUT;
            else if (SPI2_CS8_P20_6   == cs_pin)    set_cs->pin   = &IfxQspi2_SLSO8_P20_6_OUT;
            else if (SPI2_CS9_P20_3   == cs_pin)    set_cs->pin   = &IfxQspi2_SLSO9_P20_3_OUT;
            else if (SPI_CS_NULL      == cs_pin)    set_cs->pin   = NULL_PTR;
            else    zf_assert(FALSE);
        }break;

        case SPI_3:
        {
            if      (SPI3_SCLK_P02_7  == sck_pin)   set_pin->sclk = &IfxQspi3_SCLK_P02_7_OUT;
            else if (SPI3_SCLK_P22_0  == sck_pin)   set_pin->sclk = &IfxQspi3_SCLKN_P22_0_OUT;
            else if (SPI3_SCLK_P22_1  == sck_pin)   set_pin->sclk = &IfxQspi3_SCLKP_P22_1_OUT;
            else if (SPI3_SCLK_P22_3  == sck_pin)   set_pin->sclk = &IfxQspi3_SCLK_P22_3_OUT;
            else if (SPI3_SCLK_P33_11 == sck_pin)   set_pin->sclk = &IfxQspi3_SCLK_P33_11_OUT;
            else    zf_assert(FALSE);

            if      (SPI3_MOSI_P02_6  == mosi_pin)  set_pin->mtsr = &IfxQspi3_MTSR_P02_6_OUT;
            else if (SPI3_MOSI_P10_6  == mosi_pin)  set_pin->mtsr = &IfxQspi3_MTSR_P10_6_OUT;
            else if (SPI3_MOSI_P22_0  == mosi_pin)  set_pin->mtsr = &IfxQspi3_MTSR_P22_0_OUT;
            else if (SPI3_MOSI_P22_3  == mosi_pin)  set_pin->mtsr = &IfxQspi3_MTSRP_P22_3_OUT;
            else if (SPI3_MOSI_P33_12 == mosi_pin)  set_pin->mtsr = &IfxQspi3_MTSR_P33_12_OUT;
            else    zf_assert(FALSE);

            if      (SPI3_MISO_P02_5  == miso_pin)  set_pin->mrst = &IfxQspi3_MRSTA_P02_5_IN;
            else if (SPI3_MISO_P22_1  == miso_pin)  set_pin->mrst = &IfxQspi3_MRSTE_P22_1_IN;
            else if (SPI3_MISO_P21_2  == miso_pin)  set_pin->mrst = &IfxQspi3_MRSTFN_P21_2_IN;
            else if (SPI3_MISO_P21_3  == miso_pin)  set_pin->mrst = &IfxQspi3_MRSTFP_P21_3_IN;
            else if (SPI3_MISO_P33_13 == miso_pin)  set_pin->mrst = &IfxQspi3_MRSTD_P33_13_IN;
            else    zf_assert(FALSE);

            if      (SPI3_CS0_P02_4   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO0_P02_4_OUT;
            else if (SPI3_CS1_P02_0   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO1_P02_0_OUT;
            else if (SPI3_CS1_P33_9   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO1_P33_9_OUT;
            else if (SPI3_CS2_P02_1   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO2_P02_1_OUT;
            else if (SPI3_CS2_P33_8   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO2_P33_8_OUT;
            else if (SPI3_CS3_P02_2   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO3_P02_2_OUT;
            else if (SPI3_CS4_P02_3   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO4_P02_3_OUT;
            else if (SPI3_CS5_P02_8   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO5_P02_8_OUT;
            else if (SPI3_CS6_P00_8   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO6_P00_8_OUT;
            else if (SPI3_CS7_P00_9   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO7_P00_9_OUT;
            else if (SPI3_CS7_P33_7   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO7_P33_7_OUT;
            else if (SPI3_CS8_P10_5   == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO8_P10_5_OUT;
            else if (SPI3_CS11_P33_10 == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO11_P33_10_OUT;
            else if (SPI3_CS12_P22_2  == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO12_P22_2_OUT;
            else if (SPI3_CS13_P23_1  == cs_pin)    set_cs->pin   = &IfxQspi3_SLSO13_P23_1_OUT;
            else if (SPI_CS_NULL      == cs_pin)    set_cs->pin   = NULL_PTR;
            else    zf_assert(FALSE);
        }break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI������ջ���������
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ���ز���       void
// ʹ��ʾ��       spi_clear_fifo(SPI_1);
// ��ע��Ϣ       �ڲ����ã��û��������
//-------------------------------------------------------------------------------------------------------------------
static void spi_clear_fifo (Ifx_QSPI *moudle)
{
    uint32 fifo_num;
    // ��֮ǰfifo�е�����ȫ����ȡ����
    fifo_num = moudle->STATUS.B.RXFIFOLEVEL;
    while(fifo_num --)
    {
        (uint8)IfxQspi_readReceiveFifo(moudle);
    }

}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿ�д 8bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       data            ����
// ���ز���       void
// ʹ��ʾ��       spi_write_8bit(SPI_1, 0x11);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_write_8bit (spi_index_enum spi_n, const uint8 data)
{
    volatile Ifx_QSPI *moudle;                                  // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);          // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);  // �������ݺ�CS����

    IfxQspi_writeTransmitFifo(moudle, data);                    // �����͵�����д�뻺����

    while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    while(moudle->STATUS.B.PT1F == 0);                          // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                         // ������ͽ�����־λ
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿ�д 8bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       *data           ���ݴ�Ż�����
// ����˵��       len             ����������
// ���ز���       void
// ʹ��ʾ��       spi_write_8bit_array(SPI_1, data, 64);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_write_8bit_array (spi_index_enum spi_n, const uint8 *data, uint32 len)
{
    volatile Ifx_QSPI *moudle;                                  // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);          // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);// �������ݺ�CS��������Ϊ��

    do
    {
        if(len == 1)
        {
            IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);  // �������ݺ�CS����
        }

        IfxQspi_writeTransmitFifo(moudle, *data ++);                    // �����͵�����д�뻺����

        while(moudle->STATUS.B.TXFIFOLEVEL != 0);               // �ȴ��������
    }while(-- len);

    while(moudle->STATUS.B.PT1F == 0);                          // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                         // ������ͽ�����־λ
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿ�д 16bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       data            ����
// ���ز���       void
// ʹ��ʾ��       spi_write_16bit(SPI_1, 0x1101);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_write_16bit (spi_index_enum spi_n, const uint16 data)
{
    volatile Ifx_QSPI *moudle;                                  // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);          // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);// �������ݺ�CS��������Ϊ��

    IfxQspi_writeTransmitFifo(moudle, (uint8)((data & 0xFF00) >> 8)); // �����͵�����д�뻺����

    while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);  // �������ݺ�CS����

    IfxQspi_writeTransmitFifo(moudle, (uint8)(data & 0x00FF));  // �����͵�����д�뻺����

    while(moudle->STATUS.B.PT1F == 0);                          // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                         // ������ͽ�����־λ
}


//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿ�д 16bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       *data           ���ݴ�Ż�����
// ����˵��       len             ����������
// ���ز���       void
// ʹ��ʾ��       spi_write_16bit_array(SPI_1, data, 64);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_write_16bit_array (spi_index_enum spi_n, const uint16 *data, uint32 len)
{
    volatile Ifx_QSPI *moudle;                                      // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);              // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);    // �������ݺ�CS��������Ϊ��

    do
    {

        IfxQspi_writeTransmitFifo(moudle, (uint8)((*data & 0xFF00) >> 8));  // �����͵�����д�뻺����

        while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

        if(len == 1)                                                // ���һ������
        {
            IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);      // �������ݺ�CS����
        }

        IfxQspi_writeTransmitFifo(moudle, (uint8)(*data++ & 0x00FF));       // �����͵�����д�뻺����

        while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    }while(-- len);

    while(moudle->STATUS.B.PT1F == 0);                              // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                             // ������ͽ�����־λ
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿ��򴫸����ļĴ���д 8bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       register_name   �Ĵ�����ַ
// ����˵��       data            ����
// ���ز���       void
// ʹ��ʾ��        spi_write_8bit_register(SPI_1, 0x11, 0x01);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_write_8bit_register (spi_index_enum spi_n, const uint8 register_name, const uint8 data)
{
    volatile Ifx_QSPI *moudle;                                  // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);          // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);    // �������ݺ�CS��������Ϊ��

    spi_clear_fifo(moudle);                                         // ������ջ�����

    IfxQspi_writeTransmitFifo(moudle, register_name);           // �����͵�����д�뻺����

    while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);  // �������ݺ�CS����

    IfxQspi_writeTransmitFifo(moudle, data);                    // �����͵�����д�뻺����

    while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    while(moudle->STATUS.B.PT1F == 0);                          // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                         // ������ͽ�����־λ
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿ��򴫸����ļĴ���д 8bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       register_name   �Ĵ�����ַ
// ����˵��       *data           ���ݴ�Ż�����
// ����˵��       len             ����������
// ���ز���       void
// ʹ��ʾ��       spi_write_8bit_registers(SPI_1, 0x11, data, 32);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_write_8bit_registers (spi_index_enum spi_n, const uint8 register_name, const uint8 *data, uint32 len)
{
    volatile Ifx_QSPI *moudle;                                  // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);          // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);        // �������ݺ�CS��������Ϊ��

    IfxQspi_writeTransmitFifo(moudle, register_name);           // �����͵�����д�뻺����

    while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    do
    {
        if(len == 1)                                            // ���һ������
        {
            IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);  // �������ݺ�CS����
        }

        IfxQspi_writeTransmitFifo(moudle, *data ++);            // �����͵�����д�뻺����

        while(moudle->STATUS.B.TXFIFOLEVEL != 0);               // �ȴ��������

    }while(-- len);

    while(moudle->STATUS.B.PT1F == 0);                          // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                         // ������ͽ�����־λ
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿ��򴫸����ļĴ���д 16bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       register_name   �Ĵ�����ַ
// ����˵��       data            ����
// ���ز���       void
// ʹ��ʾ��       spi_write_16bit_register(SPI_1, 0x1011, 0x0101);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_write_16bit_register (spi_index_enum spi_n, const uint16 register_name, const uint16 data)
{
    volatile Ifx_QSPI *moudle;                                  // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);          // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);// �������ݺ�CS��������Ϊ��

    IfxQspi_writeTransmitFifo(moudle, (uint8)((register_name & 0xFF00) >> 8));  // �����͵�����д�뻺����

    while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    IfxQspi_writeTransmitFifo(moudle, (uint8)(register_name & 0x00FF));         // �����͵�����д�뻺����

    while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    IfxQspi_writeTransmitFifo(moudle, (uint8)((data & 0xFF00) >> 8));           // �����͵�����д�뻺����

    while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);  // �������ݺ�CS����

    IfxQspi_writeTransmitFifo(moudle, (uint8)(data & 0x00FF));  // �����͵�����д�뻺����

    while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    while(moudle->STATUS.B.PT1F == 0);                          // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                         // ������ͽ�����־λ
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿ��򴫸����ļĴ���д 16bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       register_name   �Ĵ�����ַ
// ����˵��       *data           ���ݴ�Ż�����
// ����˵��       len             ����������
// ���ز���       void
// ʹ��ʾ��       spi_write_16bit_registers(SPI_1, 0x1011, data, 32);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_write_16bit_registers (spi_index_enum spi_n, const uint16 register_name, const uint16 *data, uint32 len)
{
    volatile Ifx_QSPI *moudle;                                  // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);          // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);// �������ݺ�CS��������Ϊ��

    IfxQspi_writeTransmitFifo(moudle, (uint8)((register_name & 0xFF00) >> 8));  // �����͵�����д�뻺����

    while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    IfxQspi_writeTransmitFifo(moudle, (uint8)(register_name & 0x00FF));         // �����͵�����д�뻺����

    while(moudle->STATUS.B.TXFIFOLEVEL != 0);                   // �ȴ��������

    do
    {
        IfxQspi_writeTransmitFifo(moudle, (uint8)((*data & 0xFF00) >> 8));      // �����͵�����д�뻺����

        while(moudle->STATUS.B.TXFIFOLEVEL != 0);               // �ȴ��������

        if(len == 1)                                            // ���һ������
        {
           IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);           // �������ݺ�CS����
        }

        IfxQspi_writeTransmitFifo(moudle, (uint8)(*data ++ & 0x00FF));          // �����͵�����д�뻺����

        while(moudle->STATUS.B.TXFIFOLEVEL != 0);               // �ȴ��������

    }while(-- len);

    while(moudle->STATUS.B.PT1F == 0);                          // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                         // ������ͽ�����־λ
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿڶ� 8bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       register_name   �Ĵ�����ַ
// ���ز���       uint8           ����
// ʹ��ʾ��       spi_read_8bit(SPI_1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 spi_read_8bit (spi_index_enum spi_n)
{
    volatile Ifx_QSPI *moudle;                                  // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);          // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);  // �������ݺ�CS����

    spi_clear_fifo(moudle);                                     // ������ջ�����

    IfxQspi_writeTransmitFifo(moudle, 0);                       // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 0);                   // �ȴ��������

    while(moudle->STATUS.B.PT1F == 0);                          // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                         // ������ͽ�����־λ

    return (uint8)IfxQspi_readReceiveFifo(moudle);              // ���ؽ��յ�������
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿڶ� 8bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       *data           ���ݴ�Ż�����
// ����˵��       len             ���ͻ���������
// ���ز���       void
// ʹ��ʾ��       spi_read_8bit_array(SPI_1, data, 64);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_read_8bit_array (spi_index_enum spi_n, uint8 *data, uint32 len)
{
    volatile Ifx_QSPI *moudle;                                  // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);          // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);// �������ݺ�CS��������Ϊ��

    spi_clear_fifo(moudle);                                     // ������ջ�����

    do
    {
        if(len == 1)                                            // ���һ������
        {
           IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);           // �������ݺ�CS����
        }

        IfxQspi_writeTransmitFifo(moudle, 0);                   // �����͵�����д�뻺����

        while(moudle->STATUS.B.RXFIFOLEVEL == 0);               // �ȴ��������

        *data++ = (uint8)IfxQspi_readReceiveFifo(moudle);       // ������յ�������
    }while(-- len);

    while(moudle->STATUS.B.PT1F == 0);                          // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                         // ������ͽ�����־λ
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿڶ� 16bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       register_name   �Ĵ�����ַ
// ���ز���       uint16          ����
// ʹ��ʾ��       spi_read_16bit(SPI_1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint16 spi_read_16bit (spi_index_enum spi_n)
{
    uint16 data = 0;

    volatile Ifx_QSPI *moudle;                                  // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);          // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);// �������ݺ�CS��������Ϊ��

    spi_clear_fifo(moudle);                                     // ������ջ�����

    IfxQspi_writeTransmitFifo(moudle, 0);                       // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 0);                   // �ȴ��������

    data = (uint16)IfxQspi_readReceiveFifo(moudle);             // ������յ�������

    IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);  // �������ݺ�CS����

    IfxQspi_writeTransmitFifo(moudle, 0);                       // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 0);                   // �ȴ��������

    data = ((data << 8) | (uint16)IfxQspi_readReceiveFifo(moudle));             // ��Ͻ��յ�������

    while(moudle->STATUS.B.PT1F == 0);                          // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                         // ������ͽ�����־λ

    return data;                                                // ���ؽ��յ�������
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿڶ� 16bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       *data           ���ݴ�Ż�����
// ����˵��       len             ���ͻ���������
// ���ز���       void
// ʹ��ʾ��       spi_read_16bit_array(SPI_1, data, 64);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_read_16bit_array (spi_index_enum spi_n, uint16 *data, uint32 len)
{
    volatile Ifx_QSPI *moudle;                                      // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);              // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);    // �������ݺ�CS��������Ϊ��

    spi_clear_fifo(moudle);                                         // ������ջ�����

    do
    {
        IfxQspi_writeTransmitFifo(moudle, 0);                       // �����͵�����д�뻺����

        while(moudle->STATUS.B.RXFIFOLEVEL == 0);                   // �ȴ��������

        *data = (uint16)IfxQspi_readReceiveFifo(moudle);            // ������յ�������

        if(len == 1)
        {
            IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);              // �������ݺ�CS����
        }

        IfxQspi_writeTransmitFifo(moudle, 0);                       // �����͵�����д�뻺����

        while(moudle->STATUS.B.RXFIFOLEVEL == 0);                   // �ȴ��������

        *data = ((*data << 8) | (uint16)IfxQspi_readReceiveFifo(moudle));            // ��Ͻ��յ�������

        data ++;

    }while(-- len);

    while(moudle->STATUS.B.PT1F == 0);                              // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                             // ������ͽ�����־λ
}


//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿڴӴ������ļĴ����� 8bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       register_name   �Ĵ�����ַ
// ���ز���       uint8           ����
// ʹ��ʾ��       spi_read_8bit_register(SPI_1, 0x11);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 spi_read_8bit_register (spi_index_enum spi_n, const uint8 register_name)
{
    volatile Ifx_QSPI *moudle;                                      // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);              // ��ȡģ���ַ

    spi_clear_fifo(moudle);                                         // ������ջ�����

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);    // �������ݺ�CS��������Ϊ��

    IfxQspi_writeTransmitFifo(moudle, register_name);               // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 0);                       // �ȴ��������

    spi_clear_fifo(moudle);                                         // ������ջ�����

    IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);      // �������ݺ�CS����

    IfxQspi_writeTransmitFifo(moudle, 0);                           // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 0);                       // �ȴ��������

    while(moudle->STATUS.B.PT1F == 0);                              // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                             // ������ͽ�����־λ

    return (uint8)IfxQspi_readReceiveFifo(moudle);                  // ���ؽ��յ�������
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿڴӴ������ļĴ����� 8bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       register_name   �Ĵ�����ַ
// ����˵��       *data           ���ݴ�Ż�����
// ����˵��       len             ���ͻ���������
// ���ز���       void
// ʹ��ʾ��       spi_read_8bit_registers(SPI_1, 0x11, data, 32);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_read_8bit_registers (spi_index_enum spi_n, const uint8 register_name, uint8 *data, uint32 len)
{
    volatile Ifx_QSPI *moudle;                                      // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);              // ��ȡģ���ַ

    spi_clear_fifo(moudle);                                         // ������ջ�����

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);    // �������ݺ�CS��������Ϊ��

    IfxQspi_writeTransmitFifo(moudle, register_name);               // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 0);                       // �ȴ��������

    spi_clear_fifo(moudle);                                         // ������ջ�����

    do
    {
        if(len == 1)
        {
            IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);              // �������ݺ�CS����
        }

        IfxQspi_writeTransmitFifo(moudle, 0);                       // �����͵�����д�뻺����

        while(moudle->STATUS.B.RXFIFOLEVEL == 0);                   // �ȴ��������

        *data ++ = (uint8)IfxQspi_readReceiveFifo(moudle);          // ������յ�������

    }while(-- len);

    while(moudle->STATUS.B.PT1F == 0);                              // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                             // ������ͽ�����־λ

}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿڴӴ������ļĴ����� 16bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       register_name   �Ĵ�����ַ
// ���ز���       uint16          ����
// ʹ��ʾ��       spi_read_16bit_register(SPI_1, 0x1011);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint16 spi_read_16bit_register (spi_index_enum spi_n, const uint16 register_name)
{
    uint16 data = 0;

    volatile Ifx_QSPI *moudle;                                      // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);              // ��ȡģ���ַ

    spi_clear_fifo(moudle);                                         // ������ջ�����

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);    // �������ݺ�CS��������Ϊ��

    IfxQspi_writeTransmitFifo(moudle, (uint8)((register_name & 0xFF00) >> 8));      // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 0);                       // �ȴ��������   �״η������� ��ʱ���ջ�����������

    IfxQspi_writeTransmitFifo(moudle, (uint8)(register_name & 0x00FF));             // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 1);                       // �ȴ��������   �ٴη������� ��ʱ���ջ�������1������

    spi_clear_fifo(moudle);                                         // ������ջ�����

    IfxQspi_writeTransmitFifo(moudle, 0);                           // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 0);                       // �ȴ��������

    data |= (uint8)IfxQspi_readReceiveFifo(moudle);                 // ������յ�������

    IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);      // �������ݺ�CS����

    IfxQspi_writeTransmitFifo(moudle, 0);                           // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 0);                       // �ȴ��������

    data = ((data << 8) | (uint16)IfxQspi_readReceiveFifo(moudle)); // ������յ�������

    while(moudle->STATUS.B.PT1F == 0);                              // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                             // ������ͽ�����־λ

    return data;
}


//-------------------------------------------------------------------------------------------------------------------
// �������       SPI �ӿڴӴ������ļĴ����� 16bit ����
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       register_name   �Ĵ�����ַ
// ����˵��       *data           ���ݴ�Ż�����
// ����˵��       len             ���ͻ���������
// ���ز���       void
// ʹ��ʾ��       spi_read_16bit_registers(SPI_1, 0x1101, data, 32);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_read_16bit_registers (spi_index_enum spi_n, const uint16 register_name, uint16 *data, uint32 len)
{
    volatile Ifx_QSPI *moudle;                                      // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);              // ��ȡģ���ַ

    spi_clear_fifo(moudle);                                         // ������ջ�����

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);    // �������ݺ�CS��������Ϊ��

    IfxQspi_writeTransmitFifo(moudle, (uint8)((register_name & 0xFF00) >> 8));      // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 0);                       // �ȴ��������   �״η������� ��ʱ���ջ�����������

    IfxQspi_writeTransmitFifo(moudle, (uint8)(register_name & 0x00FF));             // �����͵�����д�뻺����

    while(moudle->STATUS.B.RXFIFOLEVEL == 1);                       // �ȴ��������   �ٴη������� ��ʱ���ջ�������1������

    spi_clear_fifo(moudle);                                         // ������ջ�����

    do
    {
        IfxQspi_writeTransmitFifo(moudle, 0);                       // �����͵�����д�뻺����

        while(moudle->STATUS.B.RXFIFOLEVEL == 0);                   // �ȴ��������

        *data |= (uint8)IfxQspi_readReceiveFifo(moudle);            // ������յ�������

        if(len == 1)
        {
            IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);              // �������ݺ�CS����
        }

        IfxQspi_writeTransmitFifo(moudle, 0);                       // �����͵�����д�뻺����

        while(moudle->STATUS.B.RXFIFOLEVEL == 0);                   // �ȴ��������

        *data = ((*data << 8) | (uint16)IfxQspi_readReceiveFifo(moudle));             // ������յ�������

        data ++;
    }while(-- len);

    while(moudle->STATUS.B.PT1F == 0);                              // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                             // ������ͽ�����־λ
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI 8bit ���ݴ��� ���������������ͬʱ���е�
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       write_buffer    ���͵����ݻ�������ַ
// ����˵��       read_buffer     ��������ʱ���յ������ݵĴ洢��ַ(����Ҫ������ NULL)
// ����˵��       len             ����������
// ���ز���       void
// ʹ��ʾ��       spi_transfer_8bit(SPI_1, buf, buf, 1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_transfer_8bit (spi_index_enum spi_n, const uint8 *write_buffer, uint8 *read_buffer, uint32 len)
{
    volatile Ifx_QSPI *moudle;                                      // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);              // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);    // �������ݺ�CS��������Ϊ��

    spi_clear_fifo(moudle);                                         // ������ջ�����

    do
    {
        if(len == 1)
        {
            IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);              // �������ݺ�CS����
        }

        IfxQspi_writeTransmitFifo(moudle, *write_buffer ++);                        // �����͵�����д�뻺����

        if(read_buffer != NULL)
        {
            while(moudle->STATUS.B.RXFIFOLEVEL == 0);               // �ȴ��������
            *read_buffer ++ = (uint8)IfxQspi_readReceiveFifo(moudle);               // ������յ�������
        }
        else
        {
            while(moudle->STATUS.B.TXFIFOLEVEL != 0);               // �ȴ��������
        }
    }while(-- len);

    while(moudle->STATUS.B.PT1F == 0);                              // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                             // ������ͽ�����־λ
}

//-------------------------------------------------------------------------------------------------------------------
// �������       SPI 16bit ���ݴ��� ���������������ͬʱ���е�
// ����˵��       spi_n           SPI ģ��� ���� zf_driver_spi.h �� spi_index_enum ö���嶨��
// ����˵��       write_buffer    ���͵����ݻ�������ַ
// ����˵��       read_buffer     ��������ʱ���յ������ݵĴ洢��ַ(����Ҫ������ NULL)
// ����˵��       len             ����������
// ���ز���       void
// ʹ��ʾ��       spi_transfer_16bit(SPI_1, buf, buf, 1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_transfer_16bit (spi_index_enum spi_n, const uint16 *write_buffer, uint16 *read_buffer, uint32 len)
{
    volatile Ifx_QSPI *moudle;                                      // ����SPIģ�����

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);              // ��ȡģ���ַ

    IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon[spi_n].U);    // �������ݺ�CS��������Ϊ��

    spi_clear_fifo(moudle);                                         // ������ջ�����

    do
    {
        IfxQspi_writeTransmitFifo(moudle, (uint8)((*write_buffer & 0xFF00) >> 8));        // �����͵�����д�뻺����

        if(read_buffer != NULL)
        {
            while(moudle->STATUS.B.RXFIFOLEVEL == 0);               // �ȴ��������
            *read_buffer = ((uint16)IfxQspi_readReceiveFifo(moudle) & 0x00FF);          // ������յ�������
        }
        else
        {
            while(moudle->STATUS.B.TXFIFOLEVEL != 0);               // �ȴ��������
        }

        if(len == 1)
        {
            IfxQspi_writeBasicConfigurationEndStream(moudle, bacon[spi_n].U);                  // �������ݺ�CS����
        }

        IfxQspi_writeTransmitFifo(moudle, (uint8)(*write_buffer & 0x00FF));             // �����͵�����д�뻺����

        if(read_buffer != NULL)
        {
            while(moudle->STATUS.B.RXFIFOLEVEL == 0);               // �ȴ��������
            *read_buffer = (*read_buffer << 8) | (uint16)IfxQspi_readReceiveFifo(moudle);// ������յ�������
        }
        else
        {
            while(moudle->STATUS.B.TXFIFOLEVEL != 0);               // �ȴ��������
        }

        write_buffer ++;
        read_buffer ++;
    }while(-- len);

    while(moudle->STATUS.B.PT1F == 0);                              // �ȴ�������־λ

    IfxQspi_clearAllEventFlags(moudle);                             // ������ͽ�����־λ
}

//-------------------------------------------------------------------------------------------------------------------
//  �������      SPI��ʼ��
//  ����˵��      spi_n           ѡ��SPIģ��(SPI_1-SPI_4)
//  ����˵��      mode            SPIģʽ 0��CPOL=0 CPHA=0    1��CPOL=0 CPHA=1   2��CPOL=1 CPHA=0   3��CPOL=1 CPHA=1  // ����ϸ�ڿ����в�������
//  ����˵��      baud            ����SPI�Ĳ�����
//  ����˵��      cs_pin          ѡ��SPIƬѡ����
//  ����˵��      sck_pin         ѡ��SPIʱ������
//  ����˵��      mosi_pin        ѡ��SPI MOSI����
//  ����˵��      miso_pin        ѡ��SPI MISO����
//  ���ز���      void
//  ʹ��ʾ��      spi_init(SPI_2, SPI_MODE0, 1*1000*1000, SPI2_SCLK_P15_3, SPI2_MOSI_P15_5, SPI2_MISO_P15_4, SPI2_CS0_P15_2); // Ӳ��SPI��ʼ��  ģʽ0 ������Ϊ1Mhz
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_init (spi_index_enum spi_n, spi_mode_enum mode, uint32 baud, spi_sck_pin_enum sck_pin, spi_mosi_pin_enum mosi_pin, spi_miso_pin_enum miso_pin, spi_cs_pin_enum cs_pin)
{
    IfxQspi_SpiMaster_Config MasterConfig;
    IfxQspi_SpiMaster MasterHandle;
    IfxQspi_SpiMaster_Channel MasterChHandle;
    IfxQspi_SpiMaster_Pins MasterPins;
    IfxQspi_SpiMaster_Output SlsoPin;
    IfxQspi_SpiMaster_ChannelConfig MasterChConfig;
    volatile Ifx_QSPI *moudle;

    // ��������Ƿ���ȷ
    // ����Ӵ˴�������ԣ���˵����ʼ��SPIʱ��ģ��ź����Ų�����ͬһģ��
    zf_assert(spi_n == (sck_pin / 100));
    zf_assert(spi_n == (mosi_pin / 100));
    zf_assert(spi_n == (miso_pin / 100));
    if(SPI_CS_NULL != cs_pin)
    {
        zf_assert(spi_n == (cs_pin / 100));
    }
    spi_cs_pin  = cs_pin;

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);

    spi_mux(spi_n, sck_pin, mosi_pin, miso_pin, cs_pin, &MasterPins, &SlsoPin);

    IfxQspi_SpiMaster_initModuleConfig(&MasterConfig, moudle);
    MasterConfig.base.mode              = SpiIf_Mode_master;
    MasterConfig.base.maximumBaudrate   = MAX_BAUD;
    MasterConfig.base.isrProvider       = IfxSrc_Tos_cpu0;

    MasterConfig.pins = &MasterPins;
    IfxQspi_SpiMaster_initModule(&MasterHandle, &MasterConfig);
    IfxQspi_SpiMaster_initChannelConfig(&MasterChConfig, &MasterHandle);

    MasterChConfig.base.baudrate = (float)baud;
    switch(mode)
    {
        case 0:
        {
            MasterChConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleLow;                       // CPOL
            MasterChConfig.base.mode.shiftClock    = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;  // CPHA
        }break;
        case 1:
        {
            MasterChConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleLow;
            MasterChConfig.base.mode.shiftClock    = SpiIf_ShiftClock_shiftTransmitDataOnLeadingEdge;
        }break;
        case 2:
        {
            MasterChConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleHigh;
            MasterChConfig.base.mode.shiftClock    = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;
        }break;
        case 3:
        {
            MasterChConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleHigh;
            MasterChConfig.base.mode.shiftClock    = SpiIf_ShiftClock_shiftTransmitDataOnLeadingEdge;
        }break;
    }

    MasterChConfig.base.mode.dataHeading    = SpiIf_DataHeading_msbFirst;
    MasterChConfig.base.mode.dataWidth      = 8;
    MasterChConfig.base.mode.csActiveLevel  = Ifx_ActiveState_low;
    MasterChConfig.sls.output               = SlsoPin;
    if(SPI_CS_NULL == cs_pin)
    {
        MasterChConfig.base.mode.loopback   = 1;
    }
    IfxQspi_SpiMaster_initChannel(&MasterChHandle, &MasterChConfig);

    if(SPI_CS_NULL == cs_pin)
    {
        IfxQspi_SpiMaster *handle   = MasterChConfig.base.driver->driver;
        Ifx_QSPI          *qspiSFR  = handle->qspi;
        qspiSFR->GLOBALCON.B.LB     = 0;
        IfxQspi_setSlaveSelectOutputControl(moudle, IfxQspi_ChannelId_0, FALSE, FALSE);
        spi_cs_pin                  = SPI_CS_NULL;
        switch(spi_n)
        {
            case SPI_0: cs_pin      = SPI0_CS0_P20_8; break;
            case SPI_1: cs_pin      = SPI1_CS0_P20_8; break;
            case SPI_2: cs_pin      = SPI2_CS0_P15_2; break;
            case SPI_3: cs_pin      = SPI3_CS0_P02_4; break;
        }
    }

    IfxQspi_configPT1Event(moudle, IfxQspi_PhaseTransitionEvent_endOfFrame);

    bacon[spi_n].U = moudle->BACON.U;
    bacon[spi_n].B.DL      = 7;        // Data Length
    bacon[spi_n].B.IDLE    = 1;        // Idle Delay Length
    bacon[spi_n].B.IPRE    = 1;        // Prescaler for the Idle Delay
    bacon[spi_n].B.LEAD    = 1;        // Leading Delay Length
    bacon[spi_n].B.LPRE    = 1;        // Prescaler for the Leading Delay
    bacon[spi_n].B.MSB     = 1;        // Shift MSB or LSB First
    bacon[spi_n].B.PARTYP  = 0;        // Parity Type
    bacon[spi_n].B.BYTE    = 0;        // Byte
    bacon[spi_n].B.TRAIL   = 1;        // Trailing Delay Length
    bacon[spi_n].B.TPRE    = 1;        // Prescaler for the Trailing Delay
    bacon[spi_n].B.CS      = cs_pin%102/6-3;
}

