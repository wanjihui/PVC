#ifndef MCAR_APP_H
#define MCAR_APP_H
#include "zf_common_headfile.h"

void app_init(void);     // 初始化电机为0、IMU/编码器/控制/按键/WiFi，上电等待K1
void app_poll(void);     // 主循环：按键动作、可选循迹、停车重连、WiFi发送
void app_tick_isr(void); // PIT_CH0每5ms：编码器、IMU、起步斜坡/陀螺纠偏/运动学、四轮速度环
void app_keys_isr(void); // PIT_CH1每10ms：锁存K1/K2事件，K2直接清PWM
#endif
