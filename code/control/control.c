/*********************************************************************************************************************
* 文件名称          control.c
* 适用平台          RT1064DVL6A
* 描述              运行状态机 + 视觉→运动决策 + 速度规划 + 麦轮运动学
*
* 数据流（与 summer-smartcar 相同的分层思想，转向环换成麦轮自转）：
*
*   mt9v03x_image ──> vis_deal() ──> g_vis.err ─┬─> 转向环 ──> ω(deg/s) ─┐
*                                              └─> 速度规划 ─> vx(cm/s) ─┤
*                                                                        ├─> mecanum_inverse()
*                                              vy(cm/s，横移，可选) ─────┘        │
*                                                                                v
*                                                        四轮 Target(cm/s) ─> Speed_PID_Crtl()
*                                                                          （200Hz 中断）
*
* 麦轮相比舵机车多出的自由度是"横移"：本工程按控制律把偏差的一部分映射成 vy，
* 由 ctl_lateral_gain 控制强度（默认 0，即先只用自转转向，上车验证后再逐步加横移）。
*********************************************************************************************************************/

#include "control.h"
#include "config.h"

/* ==================== 可调参数实体 ==================== */
uint8  ctl_preview_row   = VIS_PREVIEW_ROW_DEFAULT;
float  ctl_deadband      = 2.0f;
float  ctl_omega_kp1     = 0.60f;
float  ctl_omega_kp2     = 0.004f;
float  ctl_omega_kd      = 0.30f;
float  ctl_gyro_kd       = 0.08f;
float  ctl_omega_max     = 120.0f;
float  ctl_speed_straight = 120.0f;
float  ctl_speed_curve    = 80.0f;
float  ctl_lateral_gain   = 0.0f;

/* ==================== 运行状态 ==================== */
volatile CarState car_state    = CAR_IDLE;
volatile uint8    car_cmd      = 0;
volatile uint8    stop_reason  = STOP_REASON_NONE;
volatile float    gyro_z_dbg   = 0.0f;
volatile float    omega_dbg    = 0.0f;
volatile float    speed_target_dbg = 0.0f;
volatile float    vy_dbg            = 0.0f;

/* 内部状态 */
static float  err_last        = 0.0f;                                       // 上一帧偏差（D 项用）
static float  speed_filtered  = 0.0f;                                       // 速度 LPF 状态
static uint16 stall_count     = 0;                                          // 堵转连续帧计数
static uint16 stop_tick       = 0;                                          // 停车制动计时（50Hz 帧）

/* 停车制动参数 */
#define STOP_BRAKE_DUTY       (50)                                          // 停车阶段全力制动占空比
#define STOP_TICK_MAX         (30)                                          // 30 帧 × 20ms = 0.6s 超时强制结束
#define STOP_SPEED_EPS_CMS    (5.0f)                                        // 四轮速度均低于此值视为停稳

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     控制层初始化
// 功能描述     参数同步 + 速度环初始化 + 视觉状态复位 + 陀螺仪零偏校准
// 备注信息     control_init() 必须在 mt9v03x_init() 与 imu_init() 之后调用
//-------------------------------------------------------------------------------------------------------------------
void control_init(void)
{
    control_sync_params();
    Speed_PID_Init();
    vision_reset();
    cross_reset();
    encoder_reset_distance();

    car_state   = CAR_IDLE;
    car_cmd     = 0;
    stop_reason = STOP_REASON_NONE;

    imu_set_bias_tracking(1);                                               // 静止，开零偏跟踪
    motor_stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     参数同步（菜单改参后调用）
//-------------------------------------------------------------------------------------------------------------------
void control_sync_params(void)
{
    asc_far = ctl_preview_row;                                              // 视觉前瞻行（vision.c 的全局量）
    Speed_PID_Sync_Params();

    /* 菜单可能改了 PID 增益，这里把限幅也统一刷一遍 */
    Motor_PID[WHEEL_FL].OutMax =  (float)MOTOR_DUTY_MAX;
    Motor_PID[WHEEL_FR].OutMax =  (float)MOTOR_DUTY_MAX;
    Motor_PID[WHEEL_RL].OutMax =  (float)MOTOR_DUTY_MAX;
    Motor_PID[WHEEL_RR].OutMax =  (float)MOTOR_DUTY_MAX;
    Motor_PID[WHEEL_FL].OutMin = -(float)MOTOR_DUTY_MAX;
    Motor_PID[WHEEL_FR].OutMin = -(float)MOTOR_DUTY_MAX;
    Motor_PID[WHEEL_RL].OutMin = -(float)MOTOR_DUTY_MAX;
    Motor_PID[WHEEL_RR].OutMin = -(float)MOTOR_DUTY_MAX;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发车：IDLE → LAUNCHING
// 功能描述     清里程、复位视觉与 PID 中间量、进入开环起步
//-------------------------------------------------------------------------------------------------------------------
void control_state_launch(void)
{
    uint8 i;

    if (car_state != CAR_IDLE && car_state != CAR_STOP) return;

    err_last       = 0.0f;
    speed_filtered = 0.0f;
    stall_count    = 0;
    stop_tick      = 0;
    stop_reason    = STOP_REASON_NONE;

    for (i = 0; i < WHEEL_NUM; i++)
    {
        PID_INC_Init(&Motor_PID[i]);
        Motor_PID[i].OutMax =  (float)MOTOR_DUTY_MAX;
        Motor_PID[i].OutMin = -(float)MOTOR_DUTY_MAX;
    }
    Speed_PID_Sync_Params();

    control_sync_params();
    vision_reset();
    cross_reset();
    encoder_reset_distance();

    launch_ol_tick         = 0;
    launch_transition_tick = 0;
    launch_ol_duty         = (float)CAR_LAUNCH_DUTY;
#if ENCODER_ENABLE
    launch_openloop        = 1;                                             // 编码器模式：固定PWM起步后切速度闭环
#else
    launch_openloop        = 0;                                             // 无编码器：跳过40%起步冲刺，直接用限幅开环映射
#endif
    Speed_PID_StopBrake    = 0;
    Speed_PID_Enable       = 1;

    imu_set_bias_tracking(0);                                               // 行驶中禁止零偏跟踪

#if ENCODER_ENABLE
    car_state = CAR_LAUNCHING;
#else
    car_state = CAR_RUNNING;
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停车：任意状态 → STOP（记录原因）
// 参数说明     reason      StopReason 枚举值
//-------------------------------------------------------------------------------------------------------------------
void control_state_stop(uint8 reason)
{
    if (car_state == CAR_IDLE) return;

    stop_reason       = reason;
    car_state         = CAR_STOP;
    stop_tick         = 0;
    launch_openloop   = 0;
    Speed_PID_Enable  = 0;                                                  // 关 PID，改由 control_stop_poll 制动
    Speed_PID_StopBrake = 1;
#if !ENCODER_ENABLE
    /* 无速度反馈不能判断停稳或做反向制动，立即撤掉PWM，靠机械阻力滑停。
     * 此处不依赖摄像头新帧，KEY_2在没有新图像时也能关掉输出。 */
    motor_stop();
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     STOP 状态的制动轮询（50Hz 每帧调用）
// 功能描述     1) 四轮 Target 归零（速度环会反向刹车）
//              2) 停稳或超时后：PWM 归零、停车灯灭、回 IDLE
//-------------------------------------------------------------------------------------------------------------------
void control_stop_poll(void)
{
    uint8 i;
#if ENCODER_ENABLE
    float vmax = 0.0f;
    float v;

    if (car_state != CAR_STOP) return;

    stop_tick++;

    for (i = 0; i < WHEEL_NUM; i++)
    {
        Motor_PID[i].Target = 0.0f;                                         // 目标 0 → 速度环反向制动
        v = encoder_get_speed_cms((wheel_index_enum)i);
        if (v < 0.0f) v = -v;
        if (v > vmax) vmax = v;
    }

    Speed_PID_Enable    = 1;                                                // 保证制动期间速度环在工作
    Speed_PID_StopBrake = 1;                                                // 禁止正向输出，避免 0 附近抖动

    if (vmax < STOP_SPEED_EPS_CMS || stop_tick >= STOP_TICK_MAX)
    {
        Speed_PID_Enable    = 0;
        Speed_PID_StopBrake = 0;
        motor_stop();                                                       // 彻底断电

        for (i = 0; i < WHEEL_NUM; i++)
        {
            Motor_PID[i].Target = 0.0f;
            Motor_PID[i].Out    = 0.0f;
            Motor_PID[i].OutCmd = 0.0f;
        }

        imu_set_bias_tracking(1);                                           // 停稳，重新开零偏跟踪
        car_cmd   = 0;
        car_state = CAR_IDLE;
    }
#else
    if (car_state != CAR_STOP) return;
    Speed_PID_Enable = 0;
    Speed_PID_StopBrake = 0;
    launch_openloop = 0;
    motor_stop();
    for (i = 0; i < WHEEL_NUM; i++)
    {
        Motor_PID[i].Target = 0.0f;
        Motor_PID[i].Out = 0.0f;
        Motor_PID[i].OutCmd = 0.0f;
    }
    omega_dbg = 0.0f;
    speed_target_dbg = 0.0f;
    vy_dbg = 0.0f;
    imu_set_bias_tracking(1);
    car_cmd = 0;
    car_state = CAR_IDLE;
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     转向环：偏差 → 自转角速度指令
// 参数说明     err         赛道中线列坐标-画面中心（像素，>0 赛道在画面右侧）
// 返回参数     自转角速度 ω（deg/s，>0 逆时针 / 车头向左）
// 功能描述     ω = IMAGE_TURN_SIGN·(kp1·e + kp2·e·|e| + kd·Δe) - gyro_kd·gyro_z，再限幅
// 备注信息     正常未镜像图像：err>0时应右转，omega<0；陀螺阻尼始终抵消当前旋转
//-------------------------------------------------------------------------------------------------------------------
static float omega_calc(float err)
{
    float e;
    float delta;
    float omega;

    e = err;

    /* 死区：小偏差归零，抑制直道微摆 */
    if (e <  ctl_deadband && e > -ctl_deadband) e = 0.0f;

    delta = e - err_last;
    err_last = e;

    omega = CONTROL_IMAGE_TURN_SIGN * (ctl_omega_kp1 * e
          + ctl_omega_kp2 * e * ((e >= 0.0f) ? e : -e)                      // 二次 P：大偏差强力纠偏
          + ctl_omega_kd  * delta)
          - ctl_gyro_kd   * gyro_z_dbg;                                     // 陀螺阻尼：抑制甩尾

    if (omega >  ctl_omega_max) omega =  ctl_omega_max;
    if (omega < -ctl_omega_max) omega = -ctl_omega_max;

    return omega;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度规划：偏差 → 前进速度目标
// 功能描述     |err| 小于死区按直道速度跑；|err| 增大线性降到弯道速度
//-------------------------------------------------------------------------------------------------------------------
static float speed_plan(float err)
{
    float a;
    float target;

    a = (err >= 0.0f) ? err : -err;                                         // |err|

    if (a <= CAR_SPEED_DEADBAND_CMS)
    {
        target = ctl_speed_straight;                                        // 直道
    }
    else if (a >= 50.0f)
    {
        target = ctl_speed_curve;                                           // 大弯
    }
    else
    {
        /* 中间线性过渡：4px→直道速度，50px→弯道速度 */
        float t = (a - CAR_SPEED_DEADBAND_CMS) / (50.0f - CAR_SPEED_DEADBAND_CMS);
        target = ctl_speed_straight + t * (ctl_speed_curve - ctl_speed_straight);
    }

    return target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     堵转检测（50Hz 每帧调用）
// 返回参数     1=已触发停车 0=正常
// 功能描述     目标速度不低但四轮实际都几乎不动，持续 STALL_FRAMES 帧 → 判定被挡住/出界卡住
//-------------------------------------------------------------------------------------------------------------------
uint8 control_stall_watchdog(void)
{
#if ENCODER_ENABLE
    uint8 i;
    uint8 all_slow = 1;
    float v;

    if (car_state != CAR_RUNNING) { stall_count = 0; return 0; }

    if (speed_target_dbg < STALL_TARGET_MIN_CMS) { stall_count = 0; return 0; }

    for (i = 0; i < WHEEL_NUM; i++)
    {
        v = encoder_get_speed_cms((wheel_index_enum)i);
        if (v < 0.0f) v = -v;
        if (v >= STALL_ACTUAL_MAX_CMS) { all_slow = 0; break; }
    }

    if (all_slow)
    {
        stall_count++;
        if (stall_count >= STALL_FRAMES)
        {
            stall_count = 0;
            control_state_stop(STOP_REASON_STALL);
            return 1;
        }
    }
    else
    {
        stall_count = 0;
    }

    return 0;
#else
    /* 编码器未接时Actual恒为0，不能拿它判断堵转。 */
    stall_count = 0;
    return 0;
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外环主更新（50Hz，跟摄像头帧率走）
// 功能描述     按状态推进：IDLE 静止 / LAUNCHING 开环起步 / RUNNING 循迹 / STOP 制动
// 备注信息     本函数只产出四轮 Target，实际 PWM 由 200Hz 的 Speed_PID_Crtl() 下发
//-------------------------------------------------------------------------------------------------------------------
void control_update(void)
{
    float out[WHEEL_NUM];
    float omega = 0.0f;
    float vx    = 0.0f;
    float vy    = 0.0f;
    float rpm;
    uint8 i;

    gyro_z_dbg = imu_get_gyro_z_dps();

    switch (car_state)
    {
        case CAR_IDLE:
        {
            for (i = 0; i < WHEEL_NUM; i++) Motor_PID[i].Target = 0.0f;
            omega_dbg = 0.0f;
            speed_target_dbg = 0.0f;
            vy_dbg    = 0.0f;
            break;
        }

        case CAR_LAUNCHING:
        {
            /* 开环阶段由 pid.c 直接给固定占空比，这里只判断何时切闭环：
            * 两轮以上达到 CAR_LAUNCH_EXIT_CMS，或开环超时 */
            rpm = (encoder_get_speed_cms(WHEEL_FL) + encoder_get_speed_cms(WHEEL_RR)) * 0.5f;

            if (rpm >= CAR_LAUNCH_EXIT_CMS ||
                launch_ol_tick >= (uint16)(CAR_LAUNCH_TIMEOUT_MS / SPEED_LOOP_PERIOD_MS))
            {
                launch_openloop        = 0;                                 // 退出开环
                launch_transition_tick = 60;                                // 过渡保护 60×5ms = 0.3s
                car_state              = CAR_RUNNING;

                /* 闭环目标锁在退出速度附近，避免瞬间跳变 */
                speed_filtered = rpm > CAR_LAUNCH_EXIT_CMS ? rpm : CAR_LAUNCH_EXIT_CMS;
            }
            else
            {
                speed_target_dbg = CAR_LAUNCH_EXIT_CMS;
            }

            omega_dbg = 0.0f;
            break;
        }

        case CAR_RUNNING:
        {
            /* ---- 1. 转向环：err → ω ---- */
            omega = omega_calc(g_vis.err);

            /* ---- 2. 速度规划：err → vx，并做加速 LPF ---- */
            {
                float target = speed_plan(g_vis.err);
                float alpha  = (target > speed_filtered) ? CAR_ACCEL_ALPHA_SLOW    // 加速：慢爬
                                                        : 1.0f;                    // 减速：立即响应
                speed_filtered += alpha * (target - speed_filtered);
                vx = speed_filtered;
            }

            /* ---- 3. 横移（麦轮特有）：偏差的一部分映射为横向速度 ----
            * err>0 赛道在画面右侧，应右移 → vy为负（默认横移增益仍为0） */
            vy = CONTROL_IMAGE_TURN_SIGN * g_vis.err * ctl_lateral_gain;

            /* ---- 4. 逆运动学：车体速度 → 四轮速度目标 ---- */
            mecanum_inverse(vx, vy, omega, out);

            for (i = 0; i < WHEEL_NUM; i++)
            {
                Motor_PID[i].Target = out[i];
            }

            omega_dbg        = omega;
            speed_target_dbg = vx;
            vy_dbg           = vy;
            break;
        }

        case CAR_STOP:
        {
            control_stop_poll();
            break;
        }

        default:
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     遥测上行（VOFA+ Firewater 协议：逗号分隔 + 换行）
// 功能描述     20 通道，只在主循环打印；沿用逐飞库printf重定向到UART1
// 备注信息     默认每两帧约25Hz，115200；前4通道无编码器时为0占位，非测得轮速
//-------------------------------------------------------------------------------------------------------------------
void control_vofa_send(void)
{
    printf(
                "%.1f,%.1f,%.1f,%.1f,"      /* 1-4  四轮实际速度 cm/s   FL FR RL RR */
                "%.1f,%.1f,%.1f,%.1f,"      /* 5-8  四轮目标速度 cm/s   FL FR RL RR */
                "%.1f,%.1f,%.1f,%.1f,"      /* 9-12 四轮输出 duty        FL FR RL RR */
                "%.1f,%.1f,%.1f,%.1f,"      /* 13-16 err / steer(ω) / gyro_z / 前进目标 */
                "%u,%u,%u,%.1f\r\n",        /* 17-20 视觉耗时us / 状态 / 停车原因 / 横移 */
                Motor_PID[WHEEL_FL].Actual, Motor_PID[WHEEL_FR].Actual,
                Motor_PID[WHEEL_RL].Actual, Motor_PID[WHEEL_RR].Actual,
                Motor_PID[WHEEL_FL].Target, Motor_PID[WHEEL_FR].Target,
                Motor_PID[WHEEL_RL].Target, Motor_PID[WHEEL_RR].Target,
                Motor_PID[WHEEL_FL].OutCmd, Motor_PID[WHEEL_FR].OutCmd,
                Motor_PID[WHEEL_RL].OutCmd, Motor_PID[WHEEL_RR].OutCmd,
                g_vis.err, omega_dbg, gyro_z_dbg, speed_target_dbg,
                (unsigned int)vis_time_us,
                (unsigned int)car_state,
                (unsigned int)stop_reason,
                vy_dbg);

}
