#ifndef _user_key_h_
#define _user_key_h_
/*********************************************************************************************************************
* 文件：key.h
* 模块：应用层（第 6 层）—— 按键输入
* 职责：按键对外接口与按键编号声明
* 依赖：zf_common_typedef.h
* 风险等级：R1
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）；第三方库归属见 NOTICE.md
********************************************************************************************************************/

#include "zf_common_headfile.h"

#ifndef KEY1_PIN
#define KEY1_PIN                (P20_7)
#endif
#ifndef KEY2_PIN
#define KEY2_PIN                (P11_2)
#endif
#ifndef KEY3_PIN
#define KEY3_PIN                (P11_3)
#endif
#ifndef KEY4_PIN
#define KEY4_PIN                (P20_6)
#endif

void keys_init(void);
uint8_t get_key_value(void);

#endif
