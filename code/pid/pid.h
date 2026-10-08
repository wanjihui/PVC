/*********************************************************************************************************************
* 文件名称          pid.h
* 适用平台          RT1064DVL6A
* 描述              增量式 PID + 四轮速度环
*
* 移植自 summer-smartcar/smartcar/code/pid（原为双轮），本工程扩展为四轮。
* 速度环目标单位统一为 cm/s。
*********************************************************************************************************************/

#ifndef __PID_H_
#define __PID_H_

#include "zf_common_headfile.h"
#include "motor.h"

/**********************************************************/
/*[S] 增量式 PID [S]---------------------------------------*/
/**********************************************************/
typedef struct
{
    float Target;                                                           // 目标速度（cm/s）
    float Actual;                                                           // 实际速度（cm/s）
    float Out;                                                              // 反馈累加输出（duty）
    float OutCmd;                                                           // 最终输出 = Out + 前馈（duty），真正下发给电机
    float Kff;                                                              // 速度前馈增益（duty / (cm/s)）
    float Kp, Ki, Kd;
    float Error0;                                                           // 本次误差
    float Error1;                                                           // 上次误差
    float Error2;                                                           // 上上次误差
    float OutMax;                                                           // 输出上限
    float OutMin;                                                           // 输出下限
    float OutDeltaMax;                                                      // 单次增量限幅（>0 生效，防打滑空转）
} PID_INC_t;
/**********************************************************/
/*[E] 增量式 PID [E]---------------------------------------*/
/**********************************************************/

/**********************************************************/
/*[S] 四轮速度环 [S]--------------------------------------*/
/**********************************************************/
extern PID_INC_t Motor_PID[WHEEL_NUM];                                      // 四轮速度环，索引用 wheel_index_enum

extern volatile uint8 Speed_PID_Enable;                                     // 速度环总开关
typedef struct
{
    float target[WHEEL_NUM];
    float actual[WHEEL_NUM];
    float duty[WHEEL_NUM]; // 实际提交给motor_set_wheel的整数百分比，按逻辑前进方向表示
} speed_pid_snapshot_t;
void Speed_PID_Snapshot(speed_pid_snapshot_t *out); // 主循环短临界区复制，网络发送不进入临界区
void PID_INC_Init(PID_INC_t *p);
void PID_INC_Update(PID_INC_t *p);

/* 初始化四轮速度环（清中间量 + 从 config.h 载入 Kp/Ki/Kd/Kff） */
void Speed_PID_Init(void);

/* 四轮速度环一次计算并下发 PWM，必须在 200Hz 中断里调用 */
void Speed_PID_Crtl(void);

/* 把菜单里改过的参数同步到 PID 结构（菜单调参后调用） */
void Speed_PID_Sync_Params(void);
/**********************************************************/
/*[E] 四轮速度环 [E]--------------------------------------*/
/**********************************************************/

#endif
