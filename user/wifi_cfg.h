/*********************************************************************************************************************
* 文件：wifi_cfg.h
* 模块：应用层（第 6 层）—— WiFi 连接配置
* 职责：WiFi 模块的 SSID / 口令 / 上位机地址与连接超时配置。
*
* ⚠️ 使用前必须填写：把下面的 YOUR_WIFI_SSID / YOUR_WIFI_PASSWORD 换成你自己的。
*    本文件已进入公开仓库，请勿把真实凭据提交进来——一旦提交，即使事后改掉，
*    旧口令仍留在 git 历史里（`git log -S` 可以挖出来）。
*
* 依赖：无
* 风险等级：R1（业务逻辑）
* 许可：GPL-3.0-or-later（见仓库根 LICENSE）
********************************************************************************************************************/

#ifndef _user_wifi_cfg_h_
#define _user_wifi_cfg_h_


#ifndef WIFI_SSID_TEST
#define WIFI_SSID_TEST          "YOUR_WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD_TEST
#define WIFI_PASSWORD_TEST      "YOUR_WIFI_PASSWORD"
#endif

#ifndef WIFI_UART_TARGET_IP
#define WIFI_UART_TARGET_IP     "192.168.137.1"
#endif

#ifndef WIFI_UART_TARGET_PORT
#define WIFI_UART_TARGET_PORT   "9000"
#endif

#ifndef WIFI_TCP_TRANSFER_MODE
#define WIFI_TCP_TRANSFER_MODE  WIFI_UART_SERIANET
#endif

#ifndef WIFI_BOOT_AUTO_CONNECT_ENABLE
#define WIFI_BOOT_AUTO_CONNECT_ENABLE   (0)
#endif

#ifndef WIFI_BOOT_CONNECT_TIMEOUT_MS
#define WIFI_BOOT_CONNECT_TIMEOUT_MS    (15000u)
#endif

#ifndef WIFI_BOOT_ACK_TIMEOUT_MS
#define WIFI_BOOT_ACK_TIMEOUT_MS        (12000u)
#endif

#endif
