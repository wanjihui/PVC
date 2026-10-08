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

/* 遥测一次复制四轮目标/本拍实际/已下发PWM，防止读到混合的新旧内环数据。 */
void Speed_PID_Snapshot(speed_pid_snapshot_t *out)
{
    uint8 i;
    uint32 irq_state;
    if (out == NULL) return;
    irq_state = interrupt_global_disable();
    for (i = 0; i < WHEEL_NUM; i++)
    {
        out->target[i] = Motor_PID[i].Target;
        out->actual[i] = Motor_PID[i].Actual;
        out->duty[i] = (float)(int16)Motor_PID[i].OutCmd;
    }
    interrupt_global_enable(irq_state);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四轮速度环一次计算并下发 PWM（200Hz 中断调用）
// 功能描述     1) 从编码器刷新四轮 Actual
//              2) 发车后四轮各自 PID → duty → motor_set_wheel
//-------------------------------------------------------------------------------------------------------------------
void Speed_PID_Crtl(void)
{
    uint8 i;
#if ENCODER_CLOSED_LOOP_ENABLE

    /* ---- 1. 刷新实际速度（cm/s）---- */
    for (i = 0; i < WHEEL_NUM; i++)
    {
        Motor_PID[i].Actual = encoder_get_speed_cms((wheel_index_enum)i);
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
    /* 闭环未启用时不积分、不执行速度PID；Actual=0仅表示反馈不可用。
     * Target依然是运动学轮速指令，线性映射只用于临时验证方向和视觉纠偏。 */
    for (i = 0; i < WHEEL_NUM; i++)
    {
        float cmd = 0.0f;
        if (Speed_PID_Enable)
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
