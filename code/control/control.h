/*********************************************************************************************************************
* 文件名称          control.h
* 适用平台          RT1064DVL6A
* 描述              运行状态机 + 视觉→运动决策 + 速度规划 + 麦轮运动学调用
*
* 分工：
*       pid.c     内环：四轮速度闭环（200Hz 中断）
*       control.c 外环：跟摄像头帧率走（50Hz 主循环），产出四轮速度目标
*
* 本车为麦轮底盘，没有舵机，原 summer 工程的 servo_* 全部去掉，
* 转向改由"自转角速度 ω"实现（mecanum_inverse 的第三个参数）。
*********************************************************************************************************************/

#ifndef CONTROL_H
#define CONTROL_H

#include "zf_common_headfile.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "vision.h"
#include "pid.h"

/* ==================== 可调参数（菜单直接绑定这些变量） ==================== */

/* 视觉预瞄 */
extern uint8  ctl_preview_row;                                              // 前瞻基准行 asc_far（越小看得越远）
extern float  ctl_deadband;                                                 // 偏差死区（像素）

/* 转向环（麦轮用自转角速度代替舵机角度）：
*       omega = CONTROL_IMAGE_TURN_SIGN*(kp*e + kp2*e*|e| + kd*(e-e_last)) - gyro_kd*gyro_z
* 视觉项经CONTROL_IMAGE_TURN_SIGN映射，输出作为麦轮逆运动学的ω（deg/s）。 */
extern float  ctl_omega_kp1;
extern float  ctl_omega_kp2;
extern float  ctl_omega_kd;
extern float  ctl_gyro_kd;
extern float  ctl_omega_max;                                                // 自转角速度限幅（deg/s）

/* 速度规划 */
extern float  ctl_speed_straight;                                           // 直道目标速度（cm/s）
extern float  ctl_speed_curve;                                              // 弯道目标速度（cm/s）

/* 横移（麦轮特有）：把偏差的一部分映射成横向速度，实现"平移贴线"
* 0 = 完全不用横移（纯航向控制，先用这个上车验证） */
extern float  ctl_lateral_gain;

/* ==================== 运行状态机 ==================== */
typedef enum
{
    CAR_IDLE      = 0,                                                      // 空闲，等待发车
    CAR_LAUNCHING = 1,                                                      // 起步开环中
    CAR_RUNNING   = 2,                                                      // 正常运行
    CAR_STOP      = 3,                                                      // 停车制动
} CarState;

/* ==================== 停车原因（诊断用，VOFA 通道19） ==================== */
typedef enum
{
    STOP_REASON_NONE    = 0,                                                // 运行中
    STOP_REASON_ZEBRA   = 1,                                                // 斑马线
    STOP_REASON_LOST    = 2,                                                // 出界
    STOP_REASON_MANUAL  = 3,                                                // 菜单/按键
    STOP_REASON_STALL   = 4,                                                // 堵转
} StopReason;

extern volatile CarState car_state;
extern volatile uint8    car_cmd;                                           // 菜单代理：0=停车 1=发车
extern volatile uint8    stop_reason;
extern volatile float    gyro_z_dbg;                                        // 诊断：陀螺仪 Z（deg/s）
extern volatile float    omega_dbg;                                         // 诊断：自转角速度指令（deg/s）
extern volatile float    speed_target_dbg;                                  // 诊断：前进速度目标（cm/s）
extern volatile float    vy_dbg;                                            // 诊断：横移速度目标（cm/s）

/* ==================== 接口 ==================== */
void    control_init(void);                                                 // 初始化（速度环 + 视觉 reset + 零偏）
void    control_update(void);                                               // 50Hz 主循环：视觉→运动决策→四轮目标
void    control_vofa_send(void);                                            // 遥测上行（VOFA Firewater）
void    control_state_launch(void);                                         // IDLE → LAUNCHING
void    control_state_stop(uint8 reason);                                   // 任意 → STOP（带原因）
void    control_stop_poll(void);                                            // STOP 状态制动轮询，停稳后回 IDLE
void    control_sync_params(void);                                          // 参数同步（菜单改参后调用）
uint8   control_stall_watchdog(void);                                       // 堵转检测，返回 1 表示已触发停车

#endif
