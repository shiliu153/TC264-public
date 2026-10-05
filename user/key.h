#ifndef _user_key_h_
#define _user_key_h_

#include "zf_common_headfile.h"

#ifndef KEY1_PIN
#define KEY1_PIN                (P20_6)
#endif
#ifndef KEY2_PIN
#define KEY2_PIN                (P20_7)
#endif
#ifndef KEY3_PIN
#define KEY3_PIN                (P11_2)
#endif
#ifndef KEY4_PIN
#define KEY4_PIN                (P11_3)
#endif

void keys_init(void);
uint8_t get_key_value(void);

#endif
