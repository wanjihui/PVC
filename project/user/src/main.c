/*********************************************************************************************************************
* 文件名称          main.c
* 适用平台          RT1064DVL6A
* 描述              McarPVC —— 四电机麦轮小车 + 总钻风 MT9V03X 视觉（暂用printf串口调参，无编码器）
*
* 工程基座：逐飞科技 I.MX-RT1064 开源库（libraries/ 原样拷贝，未改动）
* 参考示例：
*       E2_01_encoder_quadrature_demo       四路正交编码器
*       E3_04_drv8701e_double_motor_demo    DRV8701E 电机驱动（两块板 = 四电机）
*       E4_01_icm20602_demo                 ICM20602 六轴
*       E6_09_wifi_spi_mt9v03x_demo         总钻风 MT9V03X + WiFi SPI 图像上传
* 图像算法：移植自 summer-smartcar/smartcar/code/vision（见 code/vision/vision.c 文件头说明）
*
* 调参方式：printf → UART1 B12/B13（默认115200）→ VOFA FireWater 20通道。
*           WiFi SPI与编码器用config.h宏暂时关闭，相关代码保留。
*
* 节拍：
*       PIT_CH0  5ms   200Hz   编码器采样 + 四轮速度环 + IMU           ← 内环
*       PIT_CH1  10ms  100Hz   按键扫描
*       主循环   跟帧   50Hz   视觉搜线 + 转向/速度规划 + 上行传输      ← 外环
*********************************************************************************************************************/

#include "zf_common_headfile.h"

#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "pid.h"
#include "vision.h"
#include "control.h"
#include "buttons.h"
#if WIFI_SPI_ENABLE
#include "wifi.h"
#endif

/* 主循环节拍统计 */
static uint32 frame_count   = 0;                                            // 摄像头帧计数
static uint16 lost_count    = 0;                                            // 出界连续帧计数
static uint16 zebra_rises   = 0;                                            // 斑马线上升沿累计

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     外设初始化
// 功能描述     顺序敏感：时钟 → 调试串口 → 摄像头 → IMU → 电机/编码器 → 视觉 → 控制 → 无线 → 中断
// 备注信息     imu_init() 内含陀螺仪零偏校准，执行时车必须静止
//-------------------------------------------------------------------------------------------------------------------
static void system_init(void)
{
    clock_init(SYSTEM_CLOCK_600M);                                          // 不可删除
    debug_init();                                                           // 调试串口（LPUART1，B12/B13）

    system_delay_ms(300);                                                   // 等主板其他外设上电完成

    /* ---- 总钻风 MT9V03X（CSI 并口）---- */
    {
        uint16 retry = 0;
        while (mt9v03x_init())
        {
            retry++;
            printf("\r\nMT9V03X init error, retry %d", retry);
            system_delay_ms(200);
            if (retry > 10) break;                                          // 不卡死，继续往下走
        }
    }

    /* ---- ICM20602 陀螺仪 ---- */
    if (imu_init())
    {
        printf("\r\nICM20602 init error!");
    }

    /* ---- 电机与编码器 ---- */
    motor_init();
#if ENCODER_ENABLE
    encoder_init();
#endif

    /* ---- 视觉：GPT1 微秒计时 ----
    * vision.c 的分段耗时统计依赖它，必须在第一次 vis_deal() 之前初始化 */
    vision_init();

    /* ---- 控制层 ---- */
    control_init();

    /* ---- 按键（无屏，仅用于手动发车/停车）---- */
    buttons_init();

    /* printf已由debug_init()接到UART1，不另开UART6。 */

    /* ---- 周期中断 ----
    * PIT_CH0 200Hz：编码器 + 速度环 + IMU
    * PIT_CH1 100Hz：按键 */
    pit_ms_init(PIT_CH0, SPEED_LOOP_PERIOD_MS);
    pit_ms_init(PIT_CH1, KEY_PERIOD_MS);

    interrupt_global_enable(0);

    printf("\r\n==== McarPVC ready (printf VOFA, baud=%u, encoder=%u) ====",
           (unsigned int)VOFA_UART_BAUD, (unsigned int)ENCODER_ENABLE);
    printf("\r\nwheelbase=%.0fmm track=%.0fmm wheel=%.0fmm",
           motor_get_wheelbase_mm(), motor_get_track_mm(), motor_get_wheel_dia_mm());

    /* ---- WiFi SPI：放最后初始化 ----
    * 连不上只影响调参（没有图像/曲线），不影响跑车，所以失败不卡死 */
#if WIFI_SPI_ENABLE
    if (wifi_init())
    {
        printf("\r\nWiFi unavailable: 仍可跑车，但没有图像与曲线上传");
    }
#endif

    printf("\r\nKEY_1=START  KEY_2=STOP\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主函数
//-------------------------------------------------------------------------------------------------------------------
int main(void)
{
    system_init();

    while (1)
    {
        /* 按键不依赖摄像头出帧；每次重新发车清掉本次运行的视觉停车计数。 */
        CarState state_before_buttons = car_state;
        buttons_poll();
        if (state_before_buttons != CAR_RUNNING && state_before_buttons != CAR_LAUNCHING &&
            (car_state == CAR_RUNNING || car_state == CAR_LAUNCHING))
        {
            lost_count = 0;
            zebra_rises = 0;
        }
#if !ENCODER_ENABLE
        if (car_state == CAR_STOP) control_stop_poll();
#endif

        /* ================= 1. 视觉：每帧处理一次（约 50Hz） ================= */
        if (mt9v03x_finish_flag)
        {
            VisionEvent evt;

            mt9v03x_finish_flag = 0;
            frame_count++;

            evt = vis_deal();                                               // 搜线 → 更新 g_vis.err / center_line / 轨迹数组

            /* ---- 停车决策（事件制，与 summer-smartcar 思路一致）---- */
            if (car_state == CAR_RUNNING || car_state == CAR_LAUNCHING)
            {
                if (evt == VISION_EVT_ZEBRA)
                {
                    lost_count = 0;                                        // 斑马线打断连续出界计数
                    zebra_rises++;
                    if (zebra_rises >= ZEBRA_STOP_RISES)
                    {
                        control_state_stop(STOP_REASON_ZEBRA);              // 第 2 次上升沿 = 终点
                    }
                }
                else if (evt == VISION_EVT_LOST)
                {
                    lost_count++;
                    if (lost_count >= LOST_STOP_FRAMES)
                    {
                        control_state_stop(STOP_REASON_LOST);               // 连续出界 → 停车
                    }
                }
                else
                {
                    lost_count = 0;
                }
            }
            else
            {
                lost_count = 0;
            }

            /* ---- 外环：状态机 + 视觉→运动决策 ---- */
            control_update();

            /* ---- 堵转兜底 ---- */
            control_stall_watchdog();

            /* ---- printf VOFA遥测（UART1，FireWater纯数字行）---- */
#if (VOFA_SEND_DIV <= 1)
            control_vofa_send();
#else
            if ((frame_count % VOFA_SEND_DIV) == 0) control_vofa_send();
#endif

            /* ---- WiFi SPI 上传：图像（降频）+ 8 通道曲线 ----
            * 传输在主循环里做，200Hz 速度环中断不受影响；
            * 代价只是发送期间视觉帧率下降，这是无线传图的固有代价。 */
#if WIFI_SPI_ENABLE
            wifi_poll();
#endif
        }

    }
}
