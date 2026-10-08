/*********************************************************************************************************************
* 文件名称          buttons.h
* 适用平台          RT1064DVL6A
* 描述              主板四键的极简处理（发车 / 停车）

* 为什么还保留按键：
*       本工程无屏，调参靠 WiFi 上传图像到逐飞助手；但"发车/停车"这两个动作
*       在现场没有电脑时仍然需要，而按键与 WiFi SPI(D12~D15,B16,B17)、
*       VOFA(UART6 B2/B3) 都没有引脚冲突，所以留着不占资源。
*
* 按键定义（逐飞主板四键，引脚在 libraries/zf_device/zf_device_key.h 的 KEY_LIST）：
*       KEY_1 (C15)  发车
*       KEY_2 (C14)  停车
*       KEY_3 (C13)  预留（当前不处理）
*       KEY_4 (C12)  预留（当前不处理）
*
* 扫描节拍：100Hz，PIT_CH1调用buttons_scan_isr保存事件；K2直接清PWM。
*          buttons_poll在主循环消费锁存事件，K1在主循环发车。
*********************************************************************************************************************/

#ifndef __BUTTONS_H_
#define __BUTTONS_H_

#include "zf_common_headfile.h"

/* 初始化（初始化四键，扫描周期取 config.h 的 KEY_SCAN_PERIOD_MS） */
void buttons_init(void);
/* 10ms中断扫描并锁存事件；K2直接停车，不等待WiFi主循环。 */
void buttons_scan_isr(void);

/* 主循环消费锁存事件：K1短按发车、确认K2停车；日志留在app主循环 */
void buttons_poll(void);

#endif
