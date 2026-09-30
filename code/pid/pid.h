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
extern volatile uint8 Speed_PID_StopBrake;                                  // 停车制动阶段标志

/* 起步开环状态：pid.c 拥有并在 200Hz 内环里推进，control.c 在 50Hz 外环里读取
* （control.c 判断"何时切闭环"，pid.c 判断"这一拍该开环还是跑 PID"） */
extern volatile uint8   launch_openloop;                                    // 1=开环中，直接给固定占空比
extern volatile float   launch_ol_duty;                                     // 开环占空比
extern volatile uint16  launch_ol_tick;                                     // 开环已持续 tick 数（200Hz）
extern volatile uint16  launch_transition_tick;                             // 切闭环后的过渡保护计时

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
