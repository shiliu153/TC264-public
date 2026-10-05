#ifndef _buzzer_h_
#define _buzzer_h_
/*********************************************************************************************************************
* 文件：buzzer.h
* 模块：应用层（第 6 层）—— 蜂鸣器
* 职责：蜂鸣器对外接口声明
* 依赖：zf_common_typedef.h
* 风险等级：R0
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUZZER_PIN      ATOM0_CH4_P02_4
#define BUZZER_FREQ_HZ  (4000u)

void buzzer_init(void);
void buzzer_beep(uint32_t freq_hz, uint32_t duration_ms);
void buzzer_stop(void);

#ifdef __cplusplus
}
#endif

#endif
