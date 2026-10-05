/*********************************************************************************************************************
* 文件：calib_uart.c
* 模块：应用层（第 6 层）—— 校准串口
* 职责：UART_0 校准命令接收与解析
* 依赖：zf_driver_uart.h、本目录 calib_uart.h
* 风险等级：R1
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "calib_uart.h"
#include "line_follow.h"
#include "params_flash.h"

#define CALIB_RX_BUF_SIZE  32u

static uint8_t  g_calib_rx_buf[CALIB_RX_BUF_SIZE];
static uint8_t  g_calib_rx_idx;

static void calib_uart_reply(const char *msg)
{
    uint32_t sr = interrupt_global_disable();
    uint32_t len = 0u;
    while (msg[len] != '\0') { len++; }
    uart_write_buffer(UART_0, (const uint8_t *)msg, len);
    interrupt_global_enable(sr);
}

static void calib_uart_reply_ok(void)
{
    calib_uart_reply("C:OK\r\n");
}

static void calib_uart_reply_fail(const char *code)
{
    char buf[24];
    int len = snprintf(buf, sizeof(buf), "C:FAIL,%s\r\n", code);
    if (len > 0 && len < (int)sizeof(buf))
    {
        calib_uart_reply(buf);
    }
}

static void calib_uart_handle_line(const char *line)
{
    if (line[0] != 'C' || line[1] != ':')
        return;

    const char *cmd = &line[2];

    if (strcmp(cmd, "START") == 0)
    {
        if (line_follow_calib_is_active())
        {
            calib_uart_reply_fail("STATE");
        }
        else
        {
            line_follow_calib_start();
            calib_uart_reply_ok();
        }
    }
    else if (strcmp(cmd, "STOP") == 0)
    {
        if (!line_follow_calib_is_active())
        {
            calib_uart_reply_fail("STATE");
        }
        else
        {
            if (line_follow_calib_stop())
            {
                params_save_to_flash();
                calib_uart_reply_ok();
            }
            else
            {
                calib_uart_reply_fail("SPAN");
            }
        }
    }
    else if (strcmp(cmd, "CLEAR") == 0)
    {
        line_follow_calib_clear();
        calib_uart_reply_ok();
    }
    else if (strcmp(cmd, "?") == 0)
    {
        if (line_follow_calib_is_active())
        {
            char buf[24];
            uint32_t samples = line_follow_calib_get_samples();
            int len = snprintf(buf, sizeof(buf), "C:ACTIVE,%lu\r\n", (unsigned long)samples);
            if (len > 0 && len < (int)sizeof(buf))
            {
                calib_uart_reply(buf);
            }
        }
        else if (line_follow_calib_is_valid())
        {
            calib_uart_reply("C:READY\r\n");
        }
        else
        {
            calib_uart_reply("C:IDLE\r\n");
        }
    }
}

static void calib_uart_rx_byte(uint8_t byte)
{
    if (byte == '\n')
    {
        if (g_calib_rx_idx > 0u && g_calib_rx_idx < CALIB_RX_BUF_SIZE)
        {
            g_calib_rx_buf[g_calib_rx_idx] = '\0';
            calib_uart_handle_line((const char *)g_calib_rx_buf);
        }
        g_calib_rx_idx = 0u;
    }
    else if (byte == '\r')
    {
        /* skip */
    }
    else
    {
        if (g_calib_rx_idx < (CALIB_RX_BUF_SIZE - 1u))
        {
            g_calib_rx_buf[g_calib_rx_idx++] = byte;
        }
        else
        {
            g_calib_rx_idx = 0u;
        }
    }
}

void calib_uart_poll(void)
{
    uint8_t buf[16];
    uint32_t n;

    while ((n = debug_read_ring_buffer(buf, (uint32_t)sizeof(buf))) > 0u)
    {
        for (uint32_t i = 0u; i < n; i++)
        {
            calib_uart_rx_byte(buf[i]);
        }
    }
}
