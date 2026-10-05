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
* �ļ�����          zf_driver_uart
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.10.2
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-15       pudding            first version
* 2023-09-27       pudding            �޸Ĵ��ڷ��ͺͽ��յ�ͨ���߼�����ֱ�Ӵӵײ��ȡ����д�����ݣ�����ͨ���жϴ���
* 2023-10-07       pudding            ����ͳһ�ص�����ָ��
********************************************************************************************************************/

#include "IFXPORT.h"
#include "ifxAsclin_reg.h"
#include "ifxCpu_Irq.h"
#include "IFXASCLIN_CFG.h"
#include "SysSe/Bsp/Bsp.h"
#include "isr_config.h"
#include "zf_common_debug.h"
#include "zf_common_function.h"
#include "zf_driver_uart.h"


// ��������handle����
IfxAsclin_Asc uart0_handle;
IfxAsclin_Asc uart1_handle;
IfxAsclin_Asc uart2_handle;
IfxAsclin_Asc uart3_handle;

// ����һ��ascConfig�Ľṹ�������ֻ���ڴ��ڳ�ʼ��
static IfxAsclin_Asc_Config uart_config;


// �������ڻ�������
static uint8 uart0_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart0_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart1_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart1_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart2_tx_buffer[256 + sizeof(Ifx_Fifo) + 8];
static uint8 uart2_rx_buffer[16 + sizeof(Ifx_Fifo) + 8];
static uint8 uart3_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart3_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];

//-------------------------------------------------------------------------------------------------------------------
// �������       �����ж����ȼ�����
// ����˵��       uart_n          ����ģ��� ���� zf_driver_uart.h �� uart_index_enum ö���嶨��
// ���ز���       void
// ʹ��ʾ��       uart_set_interrupt_priority(UART_1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void uart_set_interrupt_priority (uart_index_enum uartn)
{
    switch(uartn)
    {
        case UART_0:
        {
            uart_config.interrupt.txPriority    = UART0_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART0_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART0_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART0_INT_SERVICE;
        }break;
        case UART_1:
        {
            uart_config.interrupt.txPriority    = UART1_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART1_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART1_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART1_INT_SERVICE;
        }break;
        case UART_2:
        {
            uart_config.interrupt.txPriority    = UART2_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART2_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART2_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART2_INT_SERVICE;
        }break;
        case UART_3:
        {
            uart_config.interrupt.txPriority    = UART3_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART3_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART3_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART3_INT_SERVICE;
        }break;
        default: zf_assert(FALSE);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������       ���ô��ڻ�����
// ����˵��       uart_n          ����ģ��� ���� zf_driver_uart.h �� uart_index_enum ö���嶨��
// ���ز���       void
// ʹ��ʾ��       uart_set_buffer(UART_1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void uart_set_buffer (uart_index_enum uartn)
{
    switch(uartn)
    {
        case UART_0:
        {
            uart_config.txBuffer     = &uart0_tx_buffer;
            uart_config.rxBuffer     = &uart0_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_1:
        {
            uart_config.txBuffer     = &uart1_tx_buffer;
            uart_config.rxBuffer     = &uart1_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_2:
        {
            uart_config.txBuffer     = &uart2_tx_buffer;
            uart_config.rxBuffer     = &uart2_rx_buffer;
            uart_config.txBufferSize = 256;
            uart_config.rxBufferSize = 16;
        }break;
        case UART_3:
        {
            uart_config.txBuffer     = &uart3_tx_buffer;
            uart_config.rxBuffer     = &uart3_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        default: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������       ��ȡ�����ж�������Ϣ
// ����˵��       uart_n          ����ģ��� ���� zf_driver_uart.h �� uart_index_enum ö���嶨��
// ���ز���       void
// ʹ��ʾ��       uart_get_handle(UART_1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
IfxAsclin_Asc* uart_get_handle (uart_index_enum uartn)
{
    IfxAsclin_Asc* uart_handle = NULL;
    switch(uartn)
    {
        case UART_0: uart_handle = &uart0_handle; break;
        case UART_1: uart_handle = &uart1_handle; break;
        case UART_2: uart_handle = &uart2_handle; break;
        case UART_3: uart_handle = &uart3_handle; break;
        default: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);
    }
    return uart_handle;
}


void uart_mux (uart_index_enum uartn, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin, uint32 *set_tx_pin, uint32 *set_rx_pin)
{
    switch(uartn)
    {
        case UART_0:
        {
            if     (UART0_TX_P14_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin0_TX_P14_0_OUT;
            else if(UART0_TX_P14_1  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin0_TX_P14_1_OUT;
            else if(UART0_TX_P15_2  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin0_TX_P15_2_OUT;
            else if(UART0_TX_P15_3  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin0_TX_P15_3_OUT;
            else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

            if     (UART0_RX_P14_1  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin0_RXA_P14_1_IN;
            else if(UART0_RX_P15_3  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin0_RXB_P15_3_IN;
            else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

        }break;
        case UART_1:
        {
            if     (UART1_TX_P02_2  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P02_2_OUT;
            else if(UART1_TX_P11_12 == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P11_12_OUT;
            else if(UART1_TX_P15_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P15_0_OUT;
            else if(UART1_TX_P15_1  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P15_1_OUT;
            else if(UART1_TX_P15_4  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P15_4_OUT;
            else if(UART1_TX_P15_5  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P15_5_OUT;
            else if(UART1_TX_P20_10 == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P20_10_OUT;
            else if(UART1_TX_P33_12 == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P33_12_OUT;
            else if(UART1_TX_P33_13 == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P33_13_OUT;
            else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

            if     (UART1_RX_P15_1  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXA_P15_1_IN;
            else if(UART1_RX_P15_5  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXB_P15_5_IN;
            else if(UART1_RX_P20_9  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXC_P20_9_IN;
            else if(UART1_RX_P11_10 == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXE_P11_10_IN;
            else if(UART1_RX_P33_13 == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXF_P33_13_IN;
            else if(UART1_RX_P02_3  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXG_P02_3_IN;
            else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

        }break;
        case UART_2:
        {
            if     (UART2_TX_P02_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P02_0_OUT;
            else if(UART2_TX_P10_5  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P10_5_OUT;
            else if(UART2_TX_P14_2  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P14_2_OUT;
            else if(UART2_TX_P14_3  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P14_3_OUT;
            else if(UART2_TX_P33_8  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P33_8_OUT;
            else if(UART2_TX_P33_9  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P33_9_OUT;
            else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

            if     (UART2_RX_P14_3  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin2_RXA_P14_3_IN;
            else if(UART2_RX_P02_1  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin2_RXB_P02_1_IN;
            else if(UART2_RX_P10_6  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin2_RXD_P10_6_IN;
            else if(UART2_RX_P33_8  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin2_RXE_P33_8_IN;
            else if(UART2_RX_P02_0  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin2_RXG_P02_0_IN;

            else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

        }break;
        case UART_3:
        {
            if     (UART3_TX_P00_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P00_0_OUT;
            else if(UART3_TX_P00_1  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P00_1_OUT;
            else if(UART3_TX_P15_6  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P15_6_OUT;
            else if(UART3_TX_P15_7  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P15_7_OUT;
            else if(UART3_TX_P20_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P20_0_OUT;
            else if(UART3_TX_P20_3  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P20_3_OUT;
            else if(UART3_TX_P21_7  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P21_7_OUT;
            else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

            if     (UART3_RX_P15_7  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin3_RXA_P15_7_IN;
            else if(UART3_RX_P20_3  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin3_RXC_P20_3_IN;
            else if(UART3_RX_P00_1  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin3_RXE_P00_1_IN;
            else if(UART3_RX_P21_6  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin3_RXF_P21_6_IN;
            else IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);

        }break;
        default:break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������       ���ڷ���д��
// ����˵��       uart_n          ����ģ��� ���� zf_driver_uart.h �� uart_index_enum ö���嶨��
// ����˵��       dat             ��Ҫ���͵��ֽ�
// ���ز���       void
// ʹ��ʾ��       uart_write_byte(UART_1, 0xA5);                    // ������1�ķ��ͻ�����д��0xA5��д�����Ȼ�ᷢ�����ݣ����ǻ����CPU�ڴ��ڵ�ִ��ʱ
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void uart_write_byte (uart_index_enum uart_n, const uint8 dat)
{
    IfxAsclin_Asc* uart_handle;
    uart_handle = uart_get_handle(uart_n);

    IfxAsclin_write8(uart_handle->asclin, &dat, 1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������       ���ڷ�������
// ����˵��       uart_n          ����ģ��� ���� zf_driver_uart.h �� uart_index_enum ö���嶨��
// ����˵��       *buff           Ҫ���͵������ַ
// ����˵��       len             ���ͳ���
// ���ز���       void
// ʹ��ʾ��       uart_write_buffer(UART_1, &a[0], 5);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void uart_write_buffer (uart_index_enum uart_n, const uint8 *buff, uint32 len)
{
    IfxAsclin_Asc* uart_handle;
    uart_handle = uart_get_handle(uart_n);

    IfxAsclin_write8(uart_handle->asclin, buff, len);
}



//-------------------------------------------------------------------------------------------------------------------
// �������       ���ڷ����ַ���
// ����˵��       uart_n          ����ģ��� ���� zf_driver_uart.h �� uart_index_enum ö���嶨��
// ����˵��       *str            Ҫ���͵��ַ�����ַ
// ���ز���       void
// ʹ��ʾ��       uart_write_string(UART_1, "seekfree");
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void uart_write_string (uart_index_enum uart_n, const char *str)
{
    while(*str)
    {
        uart_write_byte(uart_n, *str++);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������       ��ȡ���ڽ��յ����ݣ�whlie�ȴ���
// ����˵��       uart_n          ����ģ��� ���� zf_driver_uart.h �� uart_index_enum ö���嶨��
// ����˵��       *dat            �������ݵĵ�ַ
// ���ز���       uint8           ���յ�����
// ʹ��ʾ��       uint8 dat = uart_read_byte(UART_1);             // ���� UART_1 ����  ������ dat ������
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 uart_read_byte (uart_index_enum uart_n)
{

    uint8 return_num = 0;
    IfxAsclin_Asc* uart_handle;
    uart_handle = uart_get_handle(uart_n);
    while(IfxAsclin_getRxFifoFillLevel(uart_handle->asclin) == 0);
    IfxAsclin_read8(uart_handle->asclin, &return_num, 1);

    return return_num;
}


//-------------------------------------------------------------------------------------------------------------------
// �������       ��ȡ���ڽ��յ����ݣ���ѯ���գ�
// ����˵��       uart_n          ����ģ��� ���� zf_driver_uart.h �� uart_index_enum ö���嶨��
// ����˵��       *dat            �������ݵĵ�ַ
// ���ز���       uint8           1�����ճɹ�   0��δ���յ�����
// ʹ��ʾ��       uint8 dat; uart_query_byte(UART_1, &dat);       // ���� UART_1 ����  ������ dat ������
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 uart_query_byte (uart_index_enum uart_n, uint8 *dat)
{
    uint8 return_num = 0;
    IfxAsclin_Asc* uart_handle;
    uart_handle = uart_get_handle(uart_n);
    if(IfxAsclin_getRxFifoFillLevel(uart_handle->asclin) > 0)
    {
        IfxAsclin_read8(uart_handle->asclin, dat, 1);
        return_num = 1;
    }
    return return_num;
}

//-------------------------------------------------------------------------------------------------------------------
// �������       ���ڷ����ж�����
// ����˵��       uart_n           ����ģ���
// ����˵��       status          1�����ж�   0���ر��ж�
// ���ز���       void
// ʹ��ʾ��       uart_tx_interrupt(UART_1, 1);                   // �򿪴���1�����ж�
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void uart_tx_interrupt (uart_index_enum uart_n, uint32 status)
{
    Ifx_ASCLIN      *asclinSFR = uart_config.asclin;
    volatile Ifx_SRC_SRCR *src;
    volatile Ifx_ASCLIN *moudle = IfxAsclin_getAddress((IfxAsclin_Index)uart_n);

    IfxAsclin_Asc_initModuleConfig(&uart_config, moudle); // ��ʼ�������ýṹ��
    src = IfxAsclin_getSrcPointerTx(asclinSFR);
    IfxAsclin_enableTxFifoFillLevelFlag(asclinSFR, (boolean)status);
    if(status)
    {
        IfxSrc_enable(src);
    }
    else
    {
        IfxSrc_disable(src);
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������       ���ڽ����ж�����
// ����˵��       uart_n           ����ģ���
// ����˵��       status          1�����ж�   0���ر��ж�
// ���ز���       void
// ʹ��ʾ��       uart_rx_interrupt(UART_1, 1);                   // �򿪴���1�����ж�
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt (uart_index_enum uart_n, uint32 status)
{
    Ifx_ASCLIN      *asclinSFR = uart_config.asclin;
    volatile Ifx_SRC_SRCR *src;
    volatile Ifx_ASCLIN *moudle = IfxAsclin_getAddress((IfxAsclin_Index)uart_n);

    IfxAsclin_Asc_initModuleConfig(&uart_config, moudle); // ��ʼ�������ýṹ��
    src = IfxAsclin_getSrcPointerRx(asclinSFR);
    IfxAsclin_enableRxFifoFillLevelFlag(asclinSFR, (boolean)status);
    if(status)
    {
        IfxSrc_enable(src);
    }
    else
    {
        IfxSrc_disable(src);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     sbus��ʼ��
// ����˵��     uartn       ����ͨ��(UART_0,UART_1,UART_2,UART_3)
// ����˵��     baud        ���ڲ�����
// ����˵��     tx_pin      ���ڷ������ź�
// ����˵��     rx_pin      ���ڽ������ź�
// ���ز���     void
// ʹ��ʾ��     uart_sbus_init(UART_2, 100000, UART2_TX_P10_5, UART2_RX_P10_6);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void uart_sbus_init (uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{

    boolean interrupt_state = disableInterrupts();

    volatile Ifx_ASCLIN *moudle = IfxAsclin_getAddress((IfxAsclin_Index)uartn);

    IfxAsclin_Asc_initModuleConfig(&uart_config, moudle);   // ��ʼ�������ýṹ��

    uart_set_buffer(uartn);                                 // ���û�����

    uart_set_interrupt_priority(uartn);                     // �����ж����ȼ�

    uart_config.clockSource           = IfxAsclin_ClockSource_ascFastClock;     // ʹ�ø���ʱ�� �������6.25M
    uart_config.baudrate.prescaler    = 4;
    uart_config.baudrate.baudrate     = (float32)baud;
    uart_config.baudrate.oversampling = IfxAsclin_OversamplingFactor_8;

    uart_config.frame.stopBit         =  IfxAsclin_StopBit_2;                    //ֹͣλ
    uart_config.frame.parityType      = IfxAsclin_ParityType_even;               //żУ��
    uart_config.frame.dataLength      = IfxAsclin_DataLength_8;
    uart_config.frame.parityBit       = TRUE;                                   //����У��

    IfxAsclin_Asc_Pins pins;                                                    // ��������
    pins.cts = NULL;
    pins.rts = NULL;
    uart_mux(uartn, tx_pin, rx_pin, (uint32 *)&pins.tx, (uint32 *)&pins.rx);
    pins.rxMode = IfxPort_InputMode_pullUp;
    pins.txMode = IfxPort_OutputMode_pushPull;
    pins.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    uart_config.pins = &pins;

    IfxAsclin_Asc_initModule(uart_get_handle(uartn), &uart_config);
    uart_rx_interrupt(uartn, 1);
    uart_tx_interrupt(uartn, 0);
    restoreInterrupts(interrupt_state);

}

//-------------------------------------------------------------------------------------------------------------------
//  �������      ���ڳ�ʼ��
//  ����˵��      uartn           ����ģ���(UART_0,UART_1,UART_2,UART_3)
//  ����˵��      baud            ���ڲ�����
//  ����˵��      tx_pin          ���ڷ�������
//  ����˵��      rx_pin          ���ڽ�������
//  ���ز���      uint32          ʵ�ʲ�����
//  ʹ��ʾ��      uart_init(UART_0,115200,UART0_TX_P14_0,UART0_RX_P14_1);       // ��ʼ������0 ������115200 ��������ʹ��P14_0 ��������ʹ��P14_1
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void uart_init (uart_index_enum uart_n, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{
    boolean interrupt_state = disableInterrupts();

    volatile Ifx_ASCLIN *moudle = IfxAsclin_getAddress((IfxAsclin_Index)uart_n);

    IfxAsclin_Asc_initModuleConfig(&uart_config, moudle);   // ��ʼ�������ýṹ��

    uart_set_buffer(uart_n);                                // ���û�����

    uart_set_interrupt_priority(uart_n);                    // �����ж����ȼ�

    uart_config.clockSource           = IfxAsclin_ClockSource_ascFastClock;     // ʹ�ø���ʱ�� �������6.25M
    uart_config.baudrate.prescaler    = 4;
    uart_config.baudrate.baudrate     = (float32)baud;
    uart_config.baudrate.oversampling = IfxAsclin_OversamplingFactor_8;

    IfxAsclin_Asc_Pins pins;                                // ��������
    pins.cts = NULL;
    pins.rts = NULL;
    uart_mux(uart_n, tx_pin, rx_pin, (uint32 *)&pins.tx, (uint32 *)&pins.rx);
    pins.rxMode = IfxPort_InputMode_pullUp;
    pins.txMode = IfxPort_OutputMode_pushPull;
    pins.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    uart_config.pins = &pins;

    IfxAsclin_Asc_initModule(uart_get_handle(uart_n), &uart_config);
    uart_rx_interrupt(uart_n, 0);
    uart_tx_interrupt(uart_n, 0);
    restoreInterrupts(interrupt_state);
}
