/* 当前APP_IMU_VERIFY_ENABLE=0：摄像头循迹，K1发车、K2停车。
 * 正常调度：新图像更新视觉指令缓存；PIT每5ms采编码器/IMU，
 * 推进500ms起步斜坡、陀螺阻尼、运动学和四轮PID。K1发车、K2立即停车。
 * 摄像头上传由主循环新帧触发，不受K1/K2门控；IMU与兼容参数包自动上传，停车目标/PWM为0。
 */
#include "app.h"
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "vision.h"
#include "control.h"
#include "pid.h"
#include "buttons.h"
#include "wifi.h"

#if SPEED_LOOP_PERIOD_MS != IMU_PERIOD_MS
#error "Shared sensor/control timer requires equal IMU and encoder periods."
#endif

#if !APP_IMU_VERIFY_ENABLE
static volatile uint32 elapsed_ms; // PIT累加的板上采样时间，不是电脑收包时间
static volatile uint8 imu_ready;
static volatile uint8 sensors_ready;
static volatile uint8 drive_allowed; // 允许控制，不等于已按K1发车
static uint8 network_ready;
static uint32 network_retry_at;
static CarState reported_state; // 主循环上次记录的状态，用于补记ISR触发的停车
#if APP_VISION_TRACKING_ENABLE
static uint8 camera_ready;
static uint16 lost_frames;
static uint16 zebra_rises;
#endif

void app_tick_isr(void)
{
    elapsed_ms += SPEED_LOOP_PERIOD_MS;
    if (!sensors_ready) return;
    // 顺序固定：采编码器/IMU→500ms起步斜坡/陀螺纠偏/轮速分配→四轮PI/PID。
    encoder_pit_callback();
    if (imu_ready) imu_update();
    // Speed_PID_Enable由K1/K2改变；停车时内环清PWM，不自行发车。
    if (drive_allowed)
    {
        control_tick_isr(); // 新图像缓存继续纠偏；vx每5ms增加3cm/s，到150后保持
        Speed_PID_Crtl();
    }
}

void app_keys_isr(void)
{
    if (drive_allowed) buttons_scan_isr(); // 锁存K1；K2直接清PWM，不依赖WiFi返回
}

void app_init(void)
{
    uint8 result;
#if APP_VISION_TRACKING_ENABLE
    uint8 attempt;
#endif
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    motor_init(); // 上电第一步PWM=0
    interrupt_global_enable(0);
    system_delay_ms(300);
    wifi_log("\r\nMCAR_APP built %s %s\r\n", __DATE__, __TIME__);
    wifi_log("[INIT] MOTOR CONFIGURED PWM=0\r\n");
    wifi_log("[INIT] IMU BEGIN; keep car still during calibration\r\n");
    // 先静止校准IMU，校准完成后才启动PIT，避免初始化与采样同时访问SPI。
    result = imu_init();
    imu_ready = (result == 0);
    if (imu_ready) wifi_log("[INIT] IMU OK\r\n");
    else wifi_log("[ERR][INIT] IMU FAIL stage=%u\r\n", (unsigned int)result);
    encoder_init();
    wifi_log("[INIT] ENCODER CONFIGURED x1 counts/cm=%.5f; FL=C2/C24 FR=C0/C1\r\n",
             (double)ENCODER_COUNTS_PER_CM);
#if APP_VISION_TRACKING_ENABLE
    result = 1;
    for (attempt = 0; attempt < 3; attempt++)
    {
        result = mt9v03x_init();
        if (result == 0) break;
        system_delay_ms(200);
    }
    camera_ready = (result == 0);
    if (camera_ready) wifi_log("[INIT] CAMERA OK\r\n");
    else wifi_log("[ERR][INIT] CAMERA FAIL\r\n");
    vision_init();
#endif
    control_init(); // 清PID、目标与运行状态，上电CAR_IDLE、速度环未开启
    reported_state = CAR_IDLE;
    buttons_init();
    drive_allowed = APP_DRIVE_ENABLE && imu_ready;
#if APP_VISION_TRACKING_ENABLE
    drive_allowed = drive_allowed && camera_ready;
#endif
    sensors_ready = 1;
    pit_ms_init(PIT_CH0, SPEED_LOOP_PERIOD_MS);
    if (drive_allowed) pit_ms_init(PIT_CH1, KEY_PERIOD_MS);
    // 网络初始化期间固定采样继续，速度环仍处于停车状态。
    result = wifi_init();
    network_ready = (result == 0);
    network_retry_at = elapsed_ms;
    if (network_ready) wifi_log("[INIT] WIFI OK module=%s target=%s tx=ASYNC_EDMA ch=30/31\r\n", wifi_spi_ip_addr_port, WIFI_UDP_TARGET_IP);
    else wifi_log("[ERR][INIT] WIFI FAIL stage=%u\r\n", (unsigned int)result);
    wifi_log("[READY] drive=%u closed_loop=%u vision=%u target=%.1fcm/s ramp=%lums gyro_kd=%.3f; K1 START K2 STOP\r\n",
             (unsigned int)drive_allowed, (unsigned int)ENCODER_CLOSED_LOOP_ENABLE,
             (unsigned int)APP_VISION_TRACKING_ENABLE, (double)CAR_TARGET_SPEED_CMS,
             (unsigned long)CAR_START_RAMP_MS, (double)ctl_gyro_kd);
#if APP_VISION_TRACKING_ENABLE && CTL_CURVE_SPEED_ENABLE
    wifi_log("[READY] CURVE straight=%.0f normal=%.0f tight=%.0f cm/s; demand=%.0f/%.0f dps; decel=%.0f accel=%.0f cm/s2\r\n",
             (double)CAR_TARGET_SPEED_CMS,(double)CTL_CURVE_NORMAL_CMS,(double)CTL_CURVE_TIGHT_CMS,
             (double)CTL_CURVE_ENTER_DPS,(double)CTL_CURVE_TIGHT_DPS,
             (double)CTL_CURVE_DECEL_CMS2,(double)CTL_CURVE_ACCEL_CMS2);
#endif
    // 连通后请求启动日志补发，不需要K1；接下来的主循环分步推进，失败缓存保留。
    if (network_ready) (void)wifi_flush_log();
}

static void app_buttons_poll(void)
{
    CarState previous;
    if (!drive_allowed) return;
    previous = reported_state;
    buttons_poll(); // K1清PID后设置四轮目标并开启；K2关速度环并立即清PWM
    reported_state = car_state;
    if (previous != CAR_RUNNING && car_state == CAR_RUNNING)
    {
#if APP_VISION_TRACKING_ENABLE
        lost_frames = 0;
        zebra_rises = 0;
#endif
        wifi_log("[RUN] START target=%.1fcm/s vision=%u ms=%lu\r\n",
                 (double)ctl_speed_target, (unsigned int)APP_VISION_TRACKING_ENABLE,
                 (unsigned long)elapsed_ms);
    }
    else if (previous == CAR_RUNNING && car_state == CAR_IDLE)
        wifi_log("[RUN] STOP reason=%u ms=%lu\r\n", (unsigned int)stop_reason, (unsigned long)elapsed_ms);
}

#if APP_VISION_TRACKING_ENABLE
static uint8 app_vision_poll(void)
{
    VisionEvent event;
    CarState previous;
    /* 没相机或没新帧就直接返回0，不重复对旧图像算视觉外环。 */
    if (!camera_ready || !vision_capture_frame()) return 0;
    /* vis_deal做二值化、滤波、搜边/补线、中线与固定前瞻偏差计算，更新g_vis。
     * 返回的是赛道事件；图像偏差由control_update从g_vis.err读取。 */
    event = vis_deal();
    if (!drive_allowed) return 1; // 仍有图像可供上位机观察，只是不产生运动指令
    previous = car_state;
    if (car_state == CAR_RUNNING)
    {
        /* 以下事件只决定是否停车，不区分直弯、不改变150cm/s固定前进目标。 */
        if (event == VISION_EVT_ZEBRA)
        {
            lost_frames = 0;
            if (++zebra_rises >= ZEBRA_STOP_RISES) control_state_stop(STOP_REASON_ZEBRA); // 第2次上升沿
        }
        else if (event == VISION_EVT_LOST)
        {
            if (++lost_frames >= LOST_STOP_FRAMES) control_state_stop(STOP_REASON_LOST); // 连续3帧出界
        }
        else lost_frames = 0; // 正常帧打断连续出界；斑马线累计次数不因此清零
    }
    else lost_frames = 0;
    /* 每个新图像只缓存转向/横移；PIT_CH0每5ms叠加起步vx和最新陀螺反馈，
     * 分配四轮Target后调用PID输出PWM，不因起步而关闭任何纠偏。 */
    control_update();
    control_stall_watchdog(); // 已有堵转判据；闭环开关为0时不生效，内部可触发停车
    if (previous == CAR_RUNNING && car_state != CAR_RUNNING)
    {
        wifi_log("[RUN] STOP reason=%u\r\n", (unsigned int)stop_reason);
        reported_state = car_state;
    }
    return 1; // 告知通信层本圈有一帧新图像，是否立刻上传由图像发送周期决定
}

#endif

void app_poll(void)
{
    uint8 new_frame = 0, result;
#if APP_VISION_TRACKING_ENABLE
    uint32 wifi_start;
#endif
    // 1. 每圈先消费K1/K2，不依赖图像出帧来执行停车。
    app_buttons_poll();
    // 2. 消费新图像快照，更新视觉外环。没新帧时沿用上一组目标。
#if APP_VISION_TRACKING_ENABLE
    new_frame = app_vision_poll();
#else
    // 可选固定四轮目标调速模式，不消费摄像头帧。
#endif
    // 3. 仅停车重连，避免入网等待阻塞正在行驶时的按键主循环。
    if (!network_ready && car_state == CAR_IDLE &&
        (uint32)(elapsed_ms - network_retry_at) >= WIFI_RETRY_MS)
    {
        result = wifi_init();
        network_ready = (result == 0);
        network_retry_at = elapsed_ms;
        if (network_ready) wifi_log("[INIT] WIFI RECONNECTED\r\n");
    }
    // 4. 摄像头只要主循环持续产生新帧就上传；K1/K2只控制运动，IMU/图像/日志独立上传。
#if APP_VISION_TRACKING_ENABLE
    wifi_start = timer_get(GPT_TIM_1);
#endif
    if (network_ready) wifi_poll(elapsed_ms, new_frame);
    else { wifi_cancel_data(); wifi_cancel_camera(); }
    if (network_ready) wifi_poll_log(elapsed_ms); // 停车仍可推进图像和必要日志
    if (network_ready && !wifi_is_ready())
    {
        network_ready = 0;
        network_retry_at = elapsed_ms; // 故障后5秒，且仅停车允许同步入网
    }
#if APP_VISION_TRACKING_ENABLE
    if (network_ready) wifi_record_service_cost(timer_get(GPT_TIM_1) - wifi_start);
#endif
    system_delay_ms(APP_MAIN_LOOP_DELAY_MS); // 不承担5ms采样定时；0=全速（图像上传更快）
}

#else
/* IMU符号验证入口：main仍调用app_init/app_poll，所有采样在主循环。
 * 不启动PIT，不初始化编码器/相机/按键，不调用控制分配或速度环。
 * 电机只初始化为0，car_state始终IDLE；WiFi数据无需按K1即可接收。 */
static uint8 verify_imu_ready, verify_network_ready;
static uint32 verify_ms, verify_clock_us, verify_remainder_us;
static uint32 verify_sample_us, verify_retry_ms;
#if APP_IMU_VERIFY_TEXT_LOG_HZ > 0
static uint32 verify_text_ms;
#endif

void app_tick_isr(void) {} // 验证模式不允许ISR重复采样IMU
void app_keys_isr(void) {} // 验证模式K1不能开启电机

void app_init(void)
{
    uint8 result;
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    motor_init();
    Speed_PID_Enable = 0;
    car_state = CAR_IDLE;
    interrupt_global_enable(0); // WiFi eDMA完成中断仍需工作
    system_delay_ms(300);
    wifi_log("\r\nMCAR_IMU_VERIFY built %s %s\r\n", __DATE__, __TIME__);
    wifi_log("[INIT] MOTOR PWM=0; camera/encoder/keys/control/PIT OFF\r\n");
    wifi_log("[INIT] IMU BEGIN; keep car still during calibration\r\n");
    result = imu_init();
    verify_imu_ready = (result == 0);
    if (verify_imu_ready) wifi_log("[INIT] IMU OK; main-loop sampling target=%ums\r\n", (unsigned int)IMU_PERIOD_MS);
    else wifi_log("[ERR][INIT] IMU FAIL stage=%u\r\n", (unsigned int)result);
    timer_init(GPT_TIM_1, TIMER_US);
    timer_start(GPT_TIM_1); // 原库timer_init只配置不启动，主循环周期判断必须先开计数
    timer_clear(GPT_TIM_1);
    verify_clock_us = verify_sample_us = timer_get(GPT_TIM_1);
    verify_ms = verify_remainder_us = 0;
    result = wifi_init();
    verify_network_ready = (result == 0);
    if (verify_network_ready) wifi_log("[INIT] WIFI OK module=%s target=%s tx=ASYNC_EDMA ch=30/31\r\n", wifi_spi_ip_addr_port, WIFI_UDP_TARGET_IP);
    else wifi_log("[ERR][INIT] WIFI FAIL stage=%u\r\n", (unsigned int)result);
    wifi_log("[READY] IMU VERIFY; auto data=%ums; motors OFF; raw gz_dps is sensor axis\r\n", (unsigned int)WIFI_DATA_PERIOD_MS);
    if (verify_network_ready) (void)wifi_flush_log();
    verify_retry_ms = verify_ms;
}

void app_poll(void)
{
    uint8 result;
    uint32 now_us = timer_get(GPT_TIM_1), wifi_start;
    /* GPT连续计时，兼容32位回绕；不依赖未启动的PIT毫秒数。
     * 到期仅采样一次，不用突发补采伪造丢失样本；imu_update记录实际dt/迟采样。 */
    verify_remainder_us += (uint32)(now_us - verify_clock_us);
    verify_clock_us = now_us;
    verify_ms += verify_remainder_us / 1000u;
    verify_remainder_us %= 1000u;
    if (verify_imu_ready && (uint32)(now_us - verify_sample_us) >= IMU_PERIOD_MS * 1000u)
    {
        verify_sample_us = now_us;
        imu_update();
    }
    if (!verify_network_ready && (uint32)(verify_ms - verify_retry_ms) >= WIFI_RETRY_MS)
    {
        result = wifi_init();
        verify_network_ready = (result == 0);
        verify_retry_ms = verify_ms;
        if (verify_network_ready) wifi_log("[INIT] WIFI RECONNECTED; IMU VERIFY\r\n");
    }
#if APP_IMU_VERIFY_TEXT_LOG_HZ > 0
    if (verify_imu_ready && (uint32)(verify_ms - verify_text_ms) >= 1000u / APP_IMU_VERIFY_TEXT_LOG_HZ)
    {
        imu_snapshot_t sample;
        verify_text_ms = verify_ms;
        imu_snapshot(&sample);
        wifi_log("[IMU] gz_raw=%+.2f dps gyro_control=%+.2f dps az=%+.3f g dt=%luus late=%lu\r\n",
                 (double)sample.gyro_dps[2], (double)imu_get_gyro_z_dps(), (double)sample.acc_g[2],
                 (unsigned long)sample.interval_us, (unsigned long)sample.late_samples);
    }
#endif
    wifi_start = timer_get(GPT_TIM_1);
    if (verify_network_ready)
    {
        wifi_poll_imu_verify(verify_ms); // 148字节兼容包，无编码器/PID/视觉调用
        wifi_poll_log(verify_ms);        // 初始化日志和IMU数据都由异步WiFi状态机发送
        if (!wifi_is_ready()) { verify_network_ready = 0; verify_retry_ms = verify_ms; }
        wifi_record_service_cost(timer_get(GPT_TIM_1) - wifi_start);
    }
    system_delay_ms(1);
}
#endif
