/*********************************************************************************************************************
* 文件名称          pid.c
* 适用平台          RT1064DVL6A
* 描述              增量式 PID + 四轮速度环
*
* 移植自 summer-smartcar/smartcar/code/pid（原为双轮），扩展为四轮麦轮。
* 调用节拍：Speed_PID_Crtl() 必须在 200Hz 定时中断（PIT_CH0，5ms）里调用。
* 单位约定：Target / Actual 都是 cm/s；输出 OutCmd 是 duty（±MOTOR_DUTY_MAX）。
*********************************************************************************************************************/

#include "pid.h"
#include "encoder.h"
#include "config.h"

PID_INC_t Motor_PID[WHEEL_NUM];                                             // 四轮速度环，索引 wheel_index_enum

volatile uint8 Speed_PID_Enable    = 0;                                     // 速度环总开关（发车后置 1）
volatile uint8 Speed_PID_StopBrake = 0;                                     // 停车制动阶段：禁止正向输出，防 0 附近正反抖动

/* ---- 起步开环（Round77 思路：低速段闭环容易卡顿，直接开环给固定占空比冲过去）---- */
volatile uint8   launch_openloop          = 0;                              // 1=开环中
volatile float   launch_ol_duty           = (float)CAR_LAUNCH_DUTY;         // 开环占空比
volatile uint16  launch_ol_tick           = 0;                              // 开环已持续 tick 数（200Hz）
volatile uint16  launch_transition_tick   = 0;                             // 切闭环后的过渡计时
static   float   launch_transition_delta  = 8.0f;                          // 过渡期单次输出变化限幅

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增量式 PID 结构初始化
//-------------------------------------------------------------------------------------------------------------------
void PID_INC_Init(PID_INC_t *p)
{
    if (p == NULL) return;

    p->Target   = 0.0f;
    p->Actual   = 0.0f;
    p->Out      = 0.0f;
    p->OutCmd   = 0.0f;
    p->Error0   = 0.0f;
    p->Error1   = 0.0f;
    p->Error2   = 0.0f;
    p->OutDeltaMax = 0.0f;                                                  // 0 = 不限制
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增量式 PID 单步计算
// 功能描述     Δu = Kp·(e0-e1) + Ki·e0 + Kd·(e0-2e1+e2)
//              输出累加后限幅；Target>Actual（要加速）时叠加速度前馈，刹车时不叠加
//-------------------------------------------------------------------------------------------------------------------
void PID_INC_Update(PID_INC_t *p)
{
    float delta;
    float out;

    if (p == NULL) return;

    p->Error0 = p->Target - p->Actual;

    delta = p->Kp * (p->Error0 - p->Error1)
          + p->Ki *  p->Error0
          + p->Kd * (p->Error0 - 2.0f * p->Error1 + p->Error2);

    /* 过渡期限制单次输出变化，抑制切闭环瞬间的冲击 */
    if (launch_transition_tick > 0)
    {
        if (delta >  launch_transition_delta) delta =  launch_transition_delta;
        if (delta < -launch_transition_delta) delta = -launch_transition_delta;
    }
    /* 打滑保护：单次增量限幅（OutDeltaMax>0 时生效） */
    if (p->OutDeltaMax > 0.0f)
    {
        if (delta >  p->OutDeltaMax) delta =  p->OutDeltaMax;
        if (delta < -p->OutDeltaMax) delta = -p->OutDeltaMax;
    }

    out = p->Out + delta;

    if (out > p->OutMax) out = p->OutMax;
    if (out < p->OutMin) out = p->OutMin;

    p->Out = out;

    /* 速度前馈：只在需要加速时叠加（刹车/减速靠反馈），Round72 教训：FF 过大会饱和 */
    if (p->Target > p->Actual)
    {
        p->OutCmd = p->Out + p->Kff * (p->Target - p->Actual);
    }
    else
    {
        p->OutCmd = p->Out;
    }

    if (p->OutCmd >  (float)MOTOR_DUTY_MAX) p->OutCmd =  (float)MOTOR_DUTY_MAX;
    if (p->OutCmd < -(float)MOTOR_DUTY_MAX) p->OutCmd = -(float)MOTOR_DUTY_MAX;

    /* 停车制动阶段：不允许正向输出，避免在 0 附近正反抖动 */
    if (Speed_PID_StopBrake && p->OutCmd > 0.0f) p->OutCmd = 0.0f;

    p->Error2 = p->Error1;
    p->Error1 = p->Error0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四轮速度环初始化
// 功能描述     清零中间量并载入 config.h 的 Kp/Ki/Kd/Kff 与输出限幅
//-------------------------------------------------------------------------------------------------------------------
void Speed_PID_Init(void)
{
    uint8 i;

    for (i = 0; i < WHEEL_NUM; i++)
    {
        PID_INC_Init(&Motor_PID[i]);
        Motor_PID[i].OutMax =  (float)MOTOR_DUTY_MAX;                       // 正转上限
        Motor_PID[i].OutMin = -(float)MOTOR_DUTY_MAX;                       // 反转上限
    }

    Speed_PID_Sync_Params();
    Speed_PID_Enable = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把 PID 参数同步进结构体（初始化 / 菜单改参后调用）
// 备注信息     菜单实时改 Kp/Ki/Kd 时，本函数负责把菜单变量搬到 PID 结构里
//-------------------------------------------------------------------------------------------------------------------
void Speed_PID_Sync_Params(void)
{
    Motor_PID[WHEEL_FL].Kp = SPEED_KP_FL;  Motor_PID[WHEEL_FL].Ki = SPEED_KI_FL;  Motor_PID[WHEEL_FL].Kd = SPEED_KD_FL;
    Motor_PID[WHEEL_FR].Kp = SPEED_KP_FR;  Motor_PID[WHEEL_FR].Ki = SPEED_KI_FR;  Motor_PID[WHEEL_FR].Kd = SPEED_KD_FR;
    Motor_PID[WHEEL_RL].Kp = SPEED_KP_RL;  Motor_PID[WHEEL_RL].Ki = SPEED_KI_RL;  Motor_PID[WHEEL_RL].Kd = SPEED_KD_RL;
    Motor_PID[WHEEL_RR].Kp = SPEED_KP_RR;  Motor_PID[WHEEL_RR].Ki = SPEED_KI_RR;  Motor_PID[WHEEL_RR].Kd = SPEED_KD_RR;

    Motor_PID[WHEEL_FL].Kff = SPEED_FF;
    Motor_PID[WHEEL_FR].Kff = SPEED_FF;
    Motor_PID[WHEEL_RL].Kff = SPEED_FF;
    Motor_PID[WHEEL_RR].Kff = SPEED_FF;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四轮速度环一次计算并下发 PWM（200Hz 中断调用）
// 功能描述     1) 从编码器刷新四轮 Actual
//              2) 开环阶段：直接给固定占空比，不计 PID
//              3) 闭环阶段：四轮各自 PID → duty → motor_set_wheel
//-------------------------------------------------------------------------------------------------------------------
void Speed_PID_Crtl(void)
{
    uint8 i;
#if ENCODER_ENABLE
    int16 duty;

    /* ---- 1. 刷新实际速度（cm/s）---- */
    for (i = 0; i < WHEEL_NUM; i++)
    {
        Motor_PID[i].Actual = encoder_get_speed_cms((wheel_index_enum)i);
    }

    if (launch_transition_tick > 0) launch_transition_tick--;

    /* ---- 2. 起步开环：不做 PID，直接给固定占空比 ----
    * 低速段闭环容易在 0 附近正反抖动/卡顿，开环冲过这个区间更干脆。 */
    if (launch_openloop)
    {
        launch_ol_tick++;
        duty = (int16)launch_ol_duty;
        motor_set_wheels(duty, duty, duty, duty);
        return;
    }

    /* ---- 3. 闭环 ---- */
    if (!Speed_PID_Enable)
    {
        motor_stop();
        return;
    }

    for (i = 0; i < WHEEL_NUM; i++)
    {
        PID_INC_Update(&Motor_PID[i]);
        motor_set_wheel((wheel_index_enum)i, (int16)Motor_PID[i].OutCmd);
    }
#else
    /* 没有编码器时不积分、不执行速度PID；Actual=0仅表示反馈不可用。
     * Target依然是运动学轮速指令，线性映射只用于临时验证方向和视觉纠偏。 */
    for (i = 0; i < WHEEL_NUM; i++)
    {
        float cmd = 0.0f;
        if (Speed_PID_Enable && !Speed_PID_StopBrake)
        {
            cmd = Motor_PID[i].Target * MOTOR_OPEN_LOOP_GAIN;
            if (cmd >  MOTOR_OPEN_LOOP_MAX) cmd =  MOTOR_OPEN_LOOP_MAX;
            if (cmd < -MOTOR_OPEN_LOOP_MAX) cmd = -MOTOR_OPEN_LOOP_MAX;
        }
        Motor_PID[i].Actual = 0.0f;
        Motor_PID[i].Out = 0.0f;
        Motor_PID[i].OutCmd = cmd;
        motor_set_wheel((wheel_index_enum)i, (int16)cmd);
    }
#endif
}
