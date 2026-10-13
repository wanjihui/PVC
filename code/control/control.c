/*********************************************************************************************************************
* 文件名称          control.c
* 适用平台          RT1064DVL6A
* 描述              运行状态机 + 视觉指令 + 起步/弯道速度规划 + 陀螺阻尼/麦轮运动学
*
* 数据流（与 summer-smartcar 相同的分层思想，转向环换成麦轮自转）：
*
*   mt9v03x_image ──> vis_deal() ──> g_vis.err ─┬─> 转向环 ──> ω(deg/s) ─┐
*                                              └─> 弯道速度规划 ─> vx(cm/s) ─┤
*                                                                        ├─> mecanum_inverse()
*                                              vy(cm/s，横移，可选) ─────┘        │
*                                                                                v
*                                                        四轮 Target(cm/s) ─> Speed_PID_Crtl()
*                                                                          （200Hz 中断）
*
* 麦轮相比舵机车多出的自由度是"横移"：本工程按控制律把偏差的一部分映射成 vy，
* 由 ctl_lateral_gain 控制强度（本轮默认0.10cm/s/px，少量横移与转向共同贴线）。
*********************************************************************************************************************/

#include "control.h"
#include "config.h"

#if CAR_START_RAMP_MS == 0
#error "CAR_START_RAMP_MS must be positive."
#endif
#if CTL_CURVE_CONFIRM_FRAMES == 0 || CTL_CURVE_CONFIRM_FRAMES > 255
#error "CTL_CURVE_CONFIRM_FRAMES must be 1..255."
#endif

/* ==================== 可调参数实体 ====================
 * 初值全部来自 config.h 的 CTL_*_DEFAULT；这里只做首次装载。
 * 通过MCAR参数协议校验后在短临界区修改；config.h是重新上电初值，不写Flash。 */
uint8  ctl_preview_row   = VIS_PREVIEW_ROW_DEFAULT;
float  ctl_deadband      = CTL_DEADBAND_PX_DEFAULT;
float  ctl_omega_kp1     = CTL_OMEGA_KP1_DEFAULT;
float  ctl_omega_kp2     = CTL_OMEGA_KP2_DEFAULT;
float  ctl_omega_kd      = CTL_OMEGA_KD_DEFAULT;
float  ctl_gyro_kd       = CTL_GYRO_KD_DEFAULT;   // 符号由 IMU_MOUNT_YAW_SIGN(-1) 保证：gyro_z 左转正
float  ctl_omega_max     = CTL_OMEGA_MAX_DPS_DEFAULT;
float  ctl_speed_target  = CAR_TARGET_SPEED_CMS;
float  ctl_lateral_gain  = CTL_LATERAL_GAIN_DEFAULT;
volatile uint32 ctl_ramp_ms = CAR_START_RAMP_MS; // 在线参数；仅停车允许改
uint8 ctl_curve_enable = CTL_CURVE_SPEED_ENABLE;
uint8 ctl_curve_confirm_frames = CTL_CURVE_CONFIRM_FRAMES;
float ctl_curve_normal_cms = CTL_CURVE_NORMAL_CMS, ctl_curve_tight_cms = CTL_CURVE_TIGHT_CMS;
float ctl_curve_enter_dps = CTL_CURVE_ENTER_DPS, ctl_curve_tight_dps = CTL_CURVE_TIGHT_DPS;
float ctl_curve_exit_dps = CTL_CURVE_EXIT_DPS, ctl_curve_exit_err_px = CTL_CURVE_EXIT_ERR_PX;
float ctl_curve_exit_gyro_dps = CTL_CURVE_EXIT_GYRO_DPS;
volatile uint32 ctl_curve_exit_stable_ms = CTL_CURVE_EXIT_STABLE_MS;
volatile uint32 ctl_curve_frame_max_age_ms = CTL_CURVE_FRAME_MAX_AGE_MS;
float ctl_curve_decel_cms2 = CTL_CURVE_DECEL_CMS2, ctl_curve_accel_cms2 = CTL_CURVE_ACCEL_CMS2;

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
static uint16 stall_count     = 0;                                          // 堵转连续帧计数
static uint32 ramp_elapsed_ms = 0; // 只按5ms PIT步进，与主循环/WiFi/摄像头刷新快慢无关
static volatile float vision_omega_command = 0.0f, vision_vy_command = 0.0f;
#if APP_VISION_TRACKING_ENABLE
static volatile uint8 curve_level = 0; // 0直道、1普通弯、2紧弯；单弯只升级，稳定出弯才释放
static uint8 curve_candidate = 0, curve_confirm_count = 0; // 主循环连续新帧确认状态
static uint32 curve_stable_ms = 0; // 5ms节拍累积真实低转速/小偏差稳定时长
static volatile uint32 curve_frame_age_ms = CTL_CURVE_FRAME_MAX_AGE_MS; // 最近视觉指令年龄
static volatile float curve_demand_abs = 0.0f, curve_error_abs = 0.0f; // deg/s和px，用于恢复判据

/* 发车/停车复位弯道缓存，避免第二次发车继承前一次的限速或稳定计时。 */
static void curve_reset(void)
{
    curve_level = curve_candidate = curve_confirm_count = 0;
    curve_stable_ms = 0;
    curve_frame_age_ms = ctl_curve_frame_max_age_ms;
    curve_demand_abs = curve_error_abs = 0.0f;
}

/* 每张新帧按未叠加陀螺的视觉转向需求确认弯道；两帧抑制单次差分尖峰。
 * 已经进入紧弯后不在弯中反复加速，只有200ms稳定出弯条件才回到直道。 */
static void curve_update_frame(float demand, float error)
{
    uint8 requested;
    curve_demand_abs = demand < 0.0f ? -demand : demand;
    curve_error_abs = error < 0.0f ? -error : error;
    if (curve_frame_age_ms > ctl_curve_frame_max_age_ms)
        curve_candidate = curve_confirm_count = 0; // 间隔过久的两帧不能算连续确认
    curve_frame_age_ms = 0;
    requested = curve_demand_abs >= ctl_curve_tight_dps ? 2u :
                (curve_demand_abs >= ctl_curve_enter_dps ? 1u : 0u);
    if (requested <= curve_level)
    { curve_candidate = curve_confirm_count = 0; return; }
    if (requested != curve_candidate) { curve_candidate = requested; curve_confirm_count = 0; }
    if (curve_confirm_count < ctl_curve_confirm_frames) curve_confirm_count++;
    if (curve_confirm_count >= ctl_curve_confirm_frames)
    { curve_level = requested; curve_stable_ms = 0; curve_candidate = curve_confirm_count = 0; }
}

/* 5ms更新vx：与网络/主循环耗时无关；起步包络不超过500ms线性目标，
 * 减速250、出弯恢复100cm/s²，只改变四轮目标的共同前进分量。 */
static void curve_speed_tick(uint8 launching)
{
    float limit = ctl_speed_target;
    float gyro_abs = gyro_z_dbg < 0.0f ? -gyro_z_dbg : gyro_z_dbg;
    float launch_limit = ctl_speed_target * ((float)ramp_elapsed_ms / (float)ctl_ramp_ms);
    float desired, rise, fall;
    uint8 fresh = curve_frame_age_ms <= ctl_curve_frame_max_age_ms;
    if (curve_frame_age_ms <= ctl_curve_frame_max_age_ms)
        curve_frame_age_ms += SPEED_LOOP_PERIOD_MS; // 到期后饱和，不会绕回假装新帧
    if (curve_level && fresh && curve_demand_abs <= ctl_curve_exit_dps &&
        curve_error_abs <= ctl_curve_exit_err_px && gyro_abs <= ctl_curve_exit_gyro_dps)
    {
        curve_stable_ms += SPEED_LOOP_PERIOD_MS;
        if (curve_stable_ms >= ctl_curve_exit_stable_ms)
        { curve_level = 0; curve_stable_ms = 0; curve_candidate = curve_confirm_count = 0; }
    }
    else curve_stable_ms = 0;
    if (curve_level == 2 && limit > ctl_curve_tight_cms) limit = ctl_curve_tight_cms;
    else if (curve_level == 1 && limit > ctl_curve_normal_cms) limit = ctl_curve_normal_cms;
    desired = launch_limit < limit ? launch_limit : limit;
    rise = launching ? ctl_speed_target * (float)SPEED_LOOP_PERIOD_MS / (float)ctl_ramp_ms :
                       ctl_curve_accel_cms2 * (float)SPEED_LOOP_PERIOD_MS / 1000.0f;
    fall = ctl_curve_decel_cms2 * (float)SPEED_LOOP_PERIOD_MS / 1000.0f;
    if (speed_target_dbg > desired)
    { speed_target_dbg -= fall; if (speed_target_dbg < desired) speed_target_dbg = desired; }
    else if (fresh && speed_target_dbg < desired)
    { speed_target_dbg += rise; if (speed_target_dbg > desired) speed_target_dbg = desired; }
}
#endif

void control_curve_params_changed(void)
{
#if APP_VISION_TRACKING_ENABLE
    curve_candidate = curve_confirm_count = 0;
    curve_stable_ms = 0;
    curve_frame_age_ms = ctl_curve_frame_max_age_ms + SPEED_LOOP_PERIOD_MS; // 等待新帧，旧缓存不能因放宽年龄而变新鲜
    if (car_state != CAR_RUNNING || !ctl_curve_enable) curve_reset();
#endif
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     控制层初始化
// 功能描述     参数同步 + 速度环初始化 + 视觉状态复位 + 陀螺仪零偏校准
// 备注信息     IMU/编码器初始化后调用；视觉模式先初始化相机与图像计时器。
//-------------------------------------------------------------------------------------------------------------------
void control_init(void)
{
    control_sync_params();
    Speed_PID_Init();
#if APP_VISION_TRACKING_ENABLE
    vision_reset();
    cross_reset();
#endif
    encoder_reset_distance();

    car_state   = CAR_IDLE;
    car_cmd     = 0;
    stop_reason = STOP_REASON_NONE;
    ramp_elapsed_ms = 0;
    vision_omega_command = vision_vy_command = 0.0f;
#if APP_VISION_TRACKING_ENABLE
    curve_reset();
#endif
    gyro_z_dbg = omega_dbg = speed_target_dbg = vy_dbg = 0.0f;

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
// 函数简介     K1发车：进入RUNNING且全程闭环；500ms起步包络同时受弯道限速约束。
//-------------------------------------------------------------------------------------------------------------------
void control_state_launch(void)
{
    if (car_state != CAR_IDLE) return;
    /* 在允许内环输出前，完整复位四轮PID和视觉状态。 */
    Speed_PID_Enable = 0;
    err_last = 0.0f;
    stall_count = 0;
    ramp_elapsed_ms = 0;
    vision_omega_command = vision_vy_command = 0.0f;
#if APP_VISION_TRACKING_ENABLE
    curve_reset();
#endif
    omega_dbg = speed_target_dbg = vy_dbg = 0.0f;
    stop_reason = STOP_REASON_NONE;
    Speed_PID_Init();
    control_sync_params();
#if APP_VISION_TRACKING_ENABLE
    vision_reset();
    cross_reset();
#endif
    encoder_reset_distance();
    imu_set_bias_tracking(0);
    car_state = CAR_RUNNING;
    car_cmd = 1;
#if APP_VISION_TRACKING_ENABLE
    /* 先缓存视觉指令；下一拍5ms中断叠加陀螺阻尼与起步速度，再生成轮速目标。 */
    control_update();
#else
    /* 调速模式不读图像；5ms斜坡和陀螺反馈仍有效，四轮PID全过程运行。 */
#endif
    Speed_PID_Enable = 1;
}

// 函数简介     停车：立即关闭速度环、清目标和PWM，回到等待K1状态。
//-------------------------------------------------------------------------------------------------------------------
void control_state_stop(uint8 reason)
{
    uint8 i;
    uint32 irq_state = interrupt_global_disable();
    Speed_PID_Enable = 0;
    ramp_elapsed_ms = 0;
    vision_omega_command = vision_vy_command = 0.0f;
#if APP_VISION_TRACKING_ENABLE
    curve_reset();
#endif
    for (i = 0; i < WHEEL_NUM; i++) PID_INC_Init(&Motor_PID[i]);
    motor_stop();
    omega_dbg = 0.0f;
    speed_target_dbg = 0.0f;
    vy_dbg = 0.0f;
    stop_reason = reason;
    stall_count = 0;
    car_cmd = 0;
    car_state = CAR_IDLE;
    interrupt_global_enable(irq_state);
    imu_set_bias_tracking(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     转向环：偏差 → 自转角速度指令
// 参数说明     err         赛道中线列坐标-画面中心（像素，>0 赛道在画面右侧）
// 返回参数     自转角速度 ω（deg/s，>0 逆时针 / 车头向左）
// 功能描述     只按新图像计算视觉P/二次P/D；陀螺阻尼与最终限幅由5ms任务完成。
// 备注信息     正常未镜像图像：err>0时应右转，omega<0；不重复对旧图像更新D项。
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
          + ctl_omega_kd  * delta);

    return omega;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     堵转检测（50Hz 每帧调用）
// 返回参数     1=已触发停车 0=正常
// 功能描述     目标速度不低但四轮实际都几乎不动，持续 STALL_FRAMES 帧 → 判定被挡住/出界卡住
//-------------------------------------------------------------------------------------------------------------------
uint8 control_stall_watchdog(void)
{
#if ENCODER_CLOSED_LOOP_ENABLE
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
    /* 闭环未开启时不使用轮速反馈判堵转；编码器硬件采样仍正常运行。 */
    stall_count = 0;
    return 0;
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主循环每个新图像只更新视觉指令缓存；5ms任务负责四轮目标。
//-------------------------------------------------------------------------------------------------------------------
void control_update(void)
{
    uint32 irq_state = interrupt_global_disable();
    if (car_state != CAR_RUNNING)
    {
        vision_omega_command = vision_vy_command = 0.0f;
        interrupt_global_enable(irq_state);
        return;
    }
#if APP_VISION_TRACKING_ENABLE
    vision_omega_command = omega_calc(g_vis.err);
    if (ctl_curve_enable) curve_update_frame(vision_omega_command,g_vis.err);
    /* 与转向共用像素死区；赛道在右侧(err>0)时vy<0，向右横移。
     * 只修位置，不构成独立的车头航向闭环；四轮速度PID仍跟踪合成目标。 */
    vision_vy_command = (g_vis.err < ctl_deadband && g_vis.err > -ctl_deadband)
                      ? 0.0f : CONTROL_IMAGE_TURN_SIGN * g_vis.err * ctl_lateral_gain;
#else
    vision_omega_command = vision_vy_command = 0.0f;
#endif
    interrupt_global_enable(irq_state);
}

/* 固定5ms节拍：最新IMU→陀螺阻尼，起步/弯道规划→vx，再逆运动学分配四轮。
 * 在RUNNING从第一拍就执行纠偏和PID；只限制前进目标，不直接设定PWM。
 * 主循环暂无新图像时保持视觉指令缓存，但斜坡/陀螺反馈仍每拍更新。 */
void control_tick_isr(void)
{
    uint8 i;
#if APP_VISION_TRACKING_ENABLE
    uint8 launching;
#endif
    float out[WHEEL_NUM];
    if (car_state != CAR_RUNNING || !Speed_PID_Enable) return;
#if APP_VISION_TRACKING_ENABLE
    launching = ramp_elapsed_ms < ctl_ramp_ms;
#endif
    if (ramp_elapsed_ms < ctl_ramp_ms)
    {
        ramp_elapsed_ms += SPEED_LOOP_PERIOD_MS;
        if (ramp_elapsed_ms > ctl_ramp_ms) ramp_elapsed_ms = ctl_ramp_ms;
    }
    gyro_z_dbg = imu_get_gyro_z_dps(); // 使用已实测安装符号-1，控制反馈左转正、右转负
#if APP_VISION_TRACKING_ENABLE
    if (ctl_curve_enable) curve_speed_tick(launching);
    else
#endif
    speed_target_dbg = ctl_speed_target * ((float)ramp_elapsed_ms / (float)ctl_ramp_ms);
    omega_dbg = vision_omega_command - ctl_gyro_kd * gyro_z_dbg;
    if (omega_dbg > ctl_omega_max) omega_dbg = ctl_omega_max;
    if (omega_dbg < -ctl_omega_max) omega_dbg = -ctl_omega_max;
    vy_dbg = vision_vy_command;
    mecanum_inverse(speed_target_dbg, vy_dbg, omega_dbg, out);
    for (i = 0; i < WHEEL_NUM; i++) Motor_PID[i].Target = out[i];
}
