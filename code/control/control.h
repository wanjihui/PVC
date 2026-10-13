/*********************************************************************************************************************
* 文件名称          control.h
* 适用平台          RT1064DVL6A
* 描述              运行状态机 + 视觉→运动决策 + 弯道速度规划 + 麦轮运动学调用
*
* 分工：
*       pid.c     内环：四轮速度闭环（200Hz 中断）
*       control.c 主循环按新图像更新视觉指令；5ms中断叠加起步斜坡/陀螺阻尼并分配四轮目标
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

/* 直道巡航上限；弯道速度由config限速，且不超过此在线值 */
extern float  ctl_speed_target; // cm/s；起步500ms包络、降速/出弯恢复均作用于vx

/* 横移（麦轮特有）：把偏差的一部分映射成横向速度，实现"平移贴线"
* 0 = 完全不用横移（纯航向控制，先用这个上车验证） */
extern float  ctl_lateral_gain;

extern volatile uint32 ctl_ramp_ms; // ms，仅停车调整

/* 弯道规划运行参数；config中的宏提供复位默认值，在线修改不写Flash。 */
extern uint8 ctl_curve_enable, ctl_curve_confirm_frames;
extern float ctl_curve_normal_cms, ctl_curve_tight_cms;
extern float ctl_curve_enter_dps, ctl_curve_tight_dps;
extern float ctl_curve_exit_dps, ctl_curve_exit_err_px, ctl_curve_exit_gyro_dps;
extern volatile uint32 ctl_curve_exit_stable_ms, ctl_curve_frame_max_age_ms;
extern float ctl_curve_decel_cms2, ctl_curve_accel_cms2;
void control_curve_params_changed(void); // 调参临界区内调用；运行中保留已确认弯级

/* ==================== 运行状态机 ==================== */
typedef enum
{
    CAR_IDLE      = 0,                                                      // 空闲，等待发车
    CAR_RUNNING   = 2,                                                      // 正常运行
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
void    control_update(void);                                               // 新图像主循环：更新视觉转向/横移缓存，不推进斜坡
void    control_tick_isr(void);                                             // 每5ms、IMU后/PID前：起步/弯速规划+陀螺阻尼→四轮目标
void    control_state_launch(void);                                         // IDLE → RUNNING
void    control_state_stop(uint8 reason);                                   // 立即清PWM → IDLE（记录原因）
void    control_sync_params(void);                                          // 参数同步（菜单改参后调用）
uint8   control_stall_watchdog(void);                                       // 堵转检测，返回 1 表示已触发停车

#endif
