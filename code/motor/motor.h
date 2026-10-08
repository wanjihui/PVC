/*********************************************************************************************************************
* 文件名称          motor.h
* 适用平台          RT1064DVL6A
* 描述              四路 DRV8701E 电机驱动 + 麦轮逆运动学
*
* 硬件：两个 DRV8701E 双路模块 = 四路 DIR + PWM
*       左前 M1  DIR=C9   PWM=C8  (PWM2_MODULE1_CHA)
*       右前 M2  DIR=C7   PWM=C6  (PWM2_MODULE0_CHA)
*       左后 M3  DIR=D2   PWM=D3  (PWM2_MODULE3_CHB)
*       右后 M4  DIR=C10  PWM=C11 (PWM2_MODULE2_CHB)
*
* 用户确认滚子轴向（俯视且车头朝上）：左前/右后=左上—右下(\)，右前/左后=左下—右上(/)
* 坐标约定（车体坐标系）：x 前进为正，y 向左为正，omega 逆时针为正
*********************************************************************************************************************/

#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

/* 四轮索引，顺序与 config.h 的 M1~M4 一致 */
typedef enum
{
    WHEEL_FL = 0,                                                           // 左前
    WHEEL_FR = 1,                                                           // 右前
    WHEEL_RL = 2,                                                           // 左后
    WHEEL_RR = 3,                                                           // 右后
    WHEEL_NUM = 4,
} wheel_index_enum;

/* 初始化四路 DIR 为输出、四路 PWM 为 17kHz 占空比 0 */
void  motor_init(void);

/* 直接给单个轮子设占空比，范围 -MOTOR_DUTY_MAX ~ +MOTOR_DUTY_MAX（正值 = 车前进方向）
* 内部会应用 config.h 的 MOTORx_DIR_INVERT 方向补偿 */
void  motor_set_wheel(wheel_index_enum wheel, int16 duty);

/* 四轮一起设，内部做限幅 */
void  motor_set_wheels(int16 duty_fl, int16 duty_fr, int16 duty_rl, int16 duty_rr);

/* 四轮同时停（PWM 归零，DIR 保持） */
void  motor_stop(void);

/* ---- 麦轮逆运动学：车体速度 → 四轮速度目标（cm/s） ----
* vx 前进速度 cm/s（正=前进）；vy 横移速度 cm/s（正=向左平移）；omega 自转角速度 deg/s（正=逆时针）
* 结果写入 out[WHEEL_NUM]，供速度环当作 Target 使用 */
void  mecanum_inverse(float vx, float vy, float omega, float out[WHEEL_NUM]);

/* 便捷封装：只前进（vy=0, omega=0） */
void  mecanum_drive(float vx, float out[WHEEL_NUM]);

/* 原地自转 */
void  mecanum_spin(float omega, float out[WHEEL_NUM]);

/* 麦轮几何参数（mm），供运动学与里程计算共用 */
float motor_get_wheelbase_mm(void);
float motor_get_track_mm(void);
float motor_get_wheel_dia_mm(void);

#endif
