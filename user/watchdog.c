#include "IfxScuWdt.h"
#include "watchdog.h"

// 时钟源 IfxScu_WDTCON1_IR_divBy16384: 100MHz / 16384 ≈ 6104Hz
// CPU0 超时 ~50ms: 6104 * 0.050 ≈ 305 → 取 300
#define WDT_CPU0_RELOAD  300u
// CPU1 超时 ~30ms: 6104 * 0.030 ≈ 183 → 取 180
#define WDT_CPU1_RELOAD  180u

static volatile uint16 g_cpu0_wdt_password;
static volatile uint16 g_cpu1_wdt_password;

void watchdog_init(void)
{
    IfxScuWdt_Config cfg;

    // ==================== CPU0 WDT ====================
    IfxScuWdt_initConfig(&cfg);
    cfg.reload = WDT_CPU0_RELOAD;
    IfxScuWdt_initCpuWatchdog(&MODULE_SCU.WDTCPU[0], &cfg);
    IfxScuWdt_enableCpuWatchdog(cfg.password);
    g_cpu0_wdt_password = IfxScuWdt_getCpuWatchdogPassword();

    // ==================== CPU1 WDT ====================
    IfxScuWdt_initConfig(&cfg);
    cfg.reload = WDT_CPU1_RELOAD;
    IfxScuWdt_initCpuWatchdog(&MODULE_SCU.WDTCPU[1], &cfg);
    // enableCpuWatchdog() uses IfxCpu_getCoreIndex() internally; from CPU0
    // it can only manipulate CPU0's WDT. Manually enable CPU1's WDT here.
    {
        Ifx_SCU_WDTCPU *const wdt1 = &MODULE_SCU.WDTCPU[1];
        IfxScuWdt_clearCpuEndinitInline(wdt1, cfg.password);
        wdt1->CON1.B.DR = 0;
        IfxScuWdt_setCpuEndinitInline(wdt1, cfg.password);
    }
    g_cpu1_wdt_password = IfxScuWdt_getCpuWatchdogPasswordInline(&MODULE_SCU.WDTCPU[1]);
}

void watchdog_service_cpu0(void)
{
    IfxScuWdt_serviceCpuWatchdog(g_cpu0_wdt_password);
}

void watchdog_service_cpu1(void)
{
    IfxScuWdt_serviceCpuWatchdog(g_cpu1_wdt_password);
}
