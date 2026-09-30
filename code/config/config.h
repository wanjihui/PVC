/*********************************************************************************************************************
* 文件名称          config.h
* 适用平台          RT1064DVL6A（逐飞科技 I.MX-RT1064 开源库）
* 描述              McarPVC 四电机麦轮小车 —— 全局配置中心
*
* 本文件是所有"硬件引脚"与"可调参数默认值"的唯一真源。
* 上层模块（motor / encoder / imu / pid / control / menu / vision）只引用这里的宏，
* 不允许在各自 .c 里硬编码引脚号或魔数，方便换车、换引脚时只改一处。
*
* 目录约定（参考 summer-smartcar 布局）：
*       code/config    本文件
*       code/vision    图像处理（移植自 summer-smartcar/smartcar/code/vision）
*       code/motor     四路 DRV8701E 电机 + 麦轮运动学
*       code/encoder   四路正交编码器
*       code/imu       ICM20602 陀螺仪（SPI）
*       code/pid       增量式 PID + 四轮速度环
*       code/control   运行状态机 + 速度规划
*       code/wifi      WiFi SPI 上传（图像 + 边界/中线 + 示波器数据）
*       code/buttons   按键（无屏，仅发车/停车）
*********************************************************************************************************************/

#ifndef __CONFIG_H_
#define __CONFIG_H_

/* ⚠ WiFi 的目标 IP / 端口不在这里配，在库头文件里改：
*       libraries/zf_device/zf_device_wifi_spi.h 的
*           WIFI_SPI_TARGET_IP       上位机（逐飞助手）IP    默认 "192.168.2.21"
*           WIFI_SPI_TARGET_PORT     上位机端口              默认 "8086"
*           WIFI_SPI_LOCAL_PORT      本机端口                默认 "6666"
*
* 为什么不能像其他参数那样放在本文件里覆盖：
*   zf_device_wifi_spi.h 被 zf_common_headfile.h 第 115 行包含，
*   而本文件必须先 #include "zf_common_headfile.h"（就在下面这行），
*   所以在 config.h 里 define 这些宏只会在库之后生效 → 触发 -Wmacro-redefined 警告，
*   而且它**抢不到**，真正生效的仍是库里的值（这是实测踩过的坑，不要再试）。
*   tools/check_wifi_cfg.py 会读出库里实际生效的值并提示是否还是出厂默认。 */

#include "zf_common_headfile.h"

/*********************************************************************************************************************
*                                            一、硬件引脚分配
* 说明：以下引脚来自逐飞官方示例，请与实物接线逐条核对后再上车
*       E2_01_encoder_quadrature_demo      四路正交编码器
*       E3_04_drv8701e_double_motor_demo   DRV8701E 电机驱动（×2 模块 = 四电机）
*       E4_01_icm20602_demo                ICM20602 六轴
*       E6_09_wifi_spi_mt9v03x_demo        总钻风 MT9V03X（CSI 并口）
*********************************************************************************************************************/

/*-------------------------------------- 电机：DRV8701E ×2 模块 → 四轮 -------------------------------------*/
/* 机械布局（俯视，车头朝上）：
*          前
*      M1(左前)  M2(右前)
*      M3(左后)  M4(右后)
*          后
*/
#define MOTOR1_DIR_PIN          (C9 )                                       // 左前 DIR
#define MOTOR1_PWM_PIN          (PWM2_MODULE1_CHA_C8)                       // 左前 PWM  17kHz
#define MOTOR2_DIR_PIN          (C7 )                                       // 右前 DIR
#define MOTOR2_PWM_PIN          (PWM2_MODULE0_CHA_C6)                       // 右前 PWM
#define MOTOR3_DIR_PIN          (D2 )                                       // 左后 DIR
#define MOTOR3_PWM_PIN          (PWM2_MODULE3_CHB_D3)                       // 左后 PWM
#define MOTOR4_DIR_PIN          (C10)                                       // 右后 DIR
#define MOTOR4_PWM_PIN          (PWM2_MODULE2_CHB_C11)                      // 右后 PWM

#define MOTOR_PWM_FREQ          (17000)                                     // PWM 频率 17kHz（与示例一致）
#define MOTOR_DUTY_MAX          (60   )                                     // 占空比限幅（%），对应 pwm_set_duty 的 0~60

/* 电机安装方向修正：0=该轮正占空比使车前进；1=取反（电机线接反/轮子镜像安装时改这里，不要改接线）
* 麦轮四轮需要左右镜像安装，实际转向常与"正占空比前进"不一致，故留此补偿表。
* 上车第一步就是让四个轮子同时正转，用本表把方向逐个纠正过来。 */
#define MOTOR1_DIR_INVERT       (0)
#define MOTOR2_DIR_INVERT       (0)
#define MOTOR3_DIR_INVERT       (0)
#define MOTOR4_DIR_INVERT       (0)

/*-------------------------------------- 编码器：四路正交解码 -------------------------------------*/
#ifndef ENCODER_ENABLE
#define ENCODER_ENABLE          (0)                                         // 编码器未到：0=禁用硬件采样和轮速 PID；到货标定后改 1
#endif
#define MOTOR_OPEN_LOOP_GAIN    (0.10f)                                     // 临时映射：轮速指令(cm/s) × 此系数 → PWM百分比，非实测速率
#define MOTOR_OPEN_LOOP_MAX     (20)                                        // 无编码器调试时四轮占空比限制 ±20%

#define ENCODER_1               (QTIMER1_ENCODER1)
#define ENCODER_1_A             (QTIMER1_ENCODER1_CH1_C0 )
#define ENCODER_1_B             (QTIMER1_ENCODER1_CH2_C1 )
#define ENCODER_2               (QTIMER1_ENCODER2)
#define ENCODER_2_A             (QTIMER1_ENCODER2_CH1_C2 )
#define ENCODER_2_B             (QTIMER1_ENCODER2_CH2_C24)
#define ENCODER_3               (QTIMER2_ENCODER1)
#define ENCODER_3_A             (QTIMER2_ENCODER1_CH1_C3 )
#define ENCODER_3_B             (QTIMER2_ENCODER1_CH2_C4 )
#define ENCODER_4               (QTIMER2_ENCODER2)
#define ENCODER_4_A             (QTIMER2_ENCODER2_CH1_C5 )
#define ENCODER_4_B             (QTIMER2_ENCODER2_CH2_C25)

/* 编码器方向修正：0=正转计数递增；1=取反。与 MOTORx_DIR_INVERT 配合，
* 目标是"车轮前进时该轮编码器读数与速度目标同号" */
#define ENCODER_1_INVERT        (0)
#define ENCODER_2_INVERT        (0)
#define ENCODER_3_INVERT        (0)
#define ENCODER_4_INVERT        (0)

/*-------------------------------------- IMU：ICM20602（硬件 SPI4） -------------------------------------*/
/* 引脚由 libraries/zf_device/zf_device_icm20602.h 固定：SCK=C23 MOSI=C22 MISO=C21 CS=C20
* 如需改动请直接改该头文件的宏，不要在业务层重复定义 */
#define IMU_MOUNT_YAW_SIGN      (1.0f)                                      // 陀螺仪 Z 轴符号：车头左转时 gyro_z 应为正，否则改 -1.0f

/*-------------------------------------- 显示：无屏 -------------------------------------*/
/* 本工程不使用 IPS200 屏幕，理由：
*   1) 逐飞官方明确说明"使用高速 WiFi SPI 模块时无法使用并口屏幕（因为引脚有共用）"；
*   2) 实测引脚冲突是 7 个全部相撞，不是"可能冲突"：
*        WiFi SPI : SCK=D12  MOSI=D14  CS=D13  INT=B17  RST=B16  (MISO=D15)
*        IPS200   : D4=D12  D6=D14    D5=D13  D1=B17   D0=B16
*   3) 调参改为"WiFi SPI 上传图像 + 边界/中线"到逐飞助手上位机看，信息量比小屏更大。
* 因此 vision.c 里的 vis_draw()/vis_bin_draw() 与整个菜单模块已移除。 */

/*-------------------------------------- WiFi SPI 模块（高速无线，用于传图像 + 示波器数据） -------------------------------------*/
/* 目标 IP / 端口在库头文件中配置；暂时不初始化、连接或上传 WiFi SPI。 */
#define WIFI_SPI_ENABLE         (0)                                         // 暂时关闭；需要无线上传时改回 1
#define WIFI_SSID               "SEEKFREE"                                  // 要连接的 WiFi 名称（改成你的）
#define WIFI_PASSWORD           "12345678"                                  // WiFi 密码（无密码填 NULL）

/* 图像上传降频：主循环 50Hz，但一帧 188*120 图像经无线传输耗时较长（百毫秒级），
* 每帧都传会把主循环拖慢。取 N 表示"每 N 帧传一次"，默认 25 → 约 2Hz。
* 注意：传输在主循环里做（不在 200Hz 中断），所以**不会影响速度环**，
*       真正被拖慢的只是视觉帧率；调参时若嫌慢就调小，稳定跑车时应调大或置 0 关闭。 */
#define WIFI_IMG_SEND_DIV       (25)                                        // 每 N 帧上传一次图像；0 = 不上传图像

/* 是否上传边界/中线（XY_BOUNDARY，3 条线：左边界/中线/右边界）。
* 开启后上位机能直接看到算法提取的中线，是无屏调参的主力手段。 */
#define WIFI_SEND_BOUNDARY      (1)

/*-------------------------------------- 按键：主板四键 C15/C14/C13/C12 -------------------------------------*/
/* 按键与 WiFi SPI(D12~D15,B16,B17) 及 VOFA(UART6 B2/B3) 均无引脚冲突，故保留，
* 用于无电脑时的手动发车/停车。 */
#define KEY_SCAN_PERIOD_MS      (10)                                        // 按键扫描周期（100Hz）
#define KEY1_FUNC               (1)                                         // KEY_1：发车
#define KEY2_FUNC               (1)                                         // KEY_2：停车

/*********************************************************************************************************************
*                                            二、时间节拍（各控制环频率）
* 参考 summer-smartcar 的分层思想：
*       视觉/决策 = 外环（跟摄像头帧率走）
*       电机速度   = 内环（独立定时器中断，必须比外环快）
*********************************************************************************************************************/
#define SPEED_LOOP_PERIOD_MS    (5 )                                        // 速度环 + 编码器采样周期 → 200Hz（PIT_CH0）
#define IMU_PERIOD_MS           (5 )                                        // 陀螺仪采样周期 → 200Hz（与速度环同中断，省一个 PIT）
#define KEY_PERIOD_MS           (10)                                        // 按键扫描 → 100Hz（PIT_CH1）

/*********************************************************************************************************************
*                                            三、麦轮底盘几何与运动学
* 逆运动学（车体坐标系：x 前进为正，y 向左为正，ω 逆时针为正）：
*       v_fl = vx - vy - ω*(L+W)/2
*       v_fr = vx + vy + ω*(L+W)/2
*       v_rl = vx + vy - ω*(L+W)/2
*       v_rr = vx - vy + ω*(L+W)/2
* 其中 L=轴距(前后轮中心距)，W=轮距(左右轮中心距)，单位 mm
*********************************************************************************************************************/
#define CHASSIS_WHEELBASE_MM    (150.0f)                                    // L 轴距：前后轮中心距，请按实车测量
#define CHASSIS_TRACK_MM        (160.0f)                                    // W 轮距：左右轮中心距，请按实车测量
#define CHASSIS_WHEEL_DIA_MM    (65.0f )                                    // 麦轮直径（常见 60/65/80mm）
#define CHASSIS_GEAR_RATIO      (1.0f  )                                    // 减速比（直连电机填 1.0）

/* 麦轮辊子方向修正（整体符号开关）
*
* 标准麦轮（辊子成"X 形"，辊子轴向：左前/右后 = 右下，右前/左后 = 左下）的运动学特征：
*       纯前进/后退 → 四轮同向同速
*       纯横移      → **对角成对反号**（左前/右后 为一组，右前/左后 为另一组），四轮代数和为 0
*       纯原地自转  → 左侧两轮反向、右侧两轮正向
*
* 若实车横移方向或自转方向与指令相反，用下面两个符号整体取反修正，
* 不要去改单个电机的 MOTORx_DIR_INVERT（那会同时破坏前进方向与自转方向的一致性）。
* 可用 tools/verify_mecanum.py 在没有硬件时先验证符号约定。 */
#define MECANUM_Y_SIGN          (1.0f)                                      // 横移方向整体取反用 -1.0f
#define MECANUM_W_SIGN          (1.0f)                                      // 自转方向整体取反用 -1.0f
#define CONTROL_IMAGE_TURN_SIGN (-1.0f)                                     // 图像err=赛道中线-画面中心；正err应右转(omega<0)，镜像摄像头再核对

/*********************************************************************************************************************
*                                            四、速度环参数（增量式 PID）
* Target/Actual 单位统一为 cm/s。轮速 → cm/s 的换算见 encoder.c
*********************************************************************************************************************/
#define SPEED_KP_FL             (1.0f )                                     // 左前 比例
#define SPEED_KI_FL             (0.025f)                                    // 左前 积分
#define SPEED_KD_FL             (0.10f)                                     // 左前 微分
#define SPEED_KP_FR             (1.2f )                                     // 右前（右侧摩擦/阻力通常与左侧不同，允许单独标定）
#define SPEED_KI_FR             (0.025f)
#define SPEED_KD_FR             (0.10f)
#define SPEED_KP_RL             (1.0f )
#define SPEED_KI_RL             (0.025f)
#define SPEED_KD_RL             (0.10f)
#define SPEED_KP_RR             (1.2f )
#define SPEED_KI_RR             (0.025f)
#define SPEED_KD_RR             (0.10f)
#define SPEED_FF                (0.10f)                                     // 速度前馈（duty / (cm/s)）

/*********************************************************************************************************************
*                                            五、速度规划与限幅
*********************************************************************************************************************/
#define CAR_SPEED_MAX_CMS       (220.0f)                                    // 直线最高速度 cm/s
#define CAR_SPEED_MIN_CMS       (160.0f)                                    // 弯道最低速度 cm/s
#define CAR_LAUNCH_DUTY         (40   )                                     // 起步开环占空比
#define CAR_LAUNCH_EXIT_CMS     (180.0f)                                    // 开环退出阈值：两轮实际 ≥ 该速度即切闭环
#define CAR_LAUNCH_TIMEOUT_MS   (1000 )                                     // 开环最长持续时间
#define CAR_ACCEL_ALPHA_FAST    (0.15f)                                     // 直道加速 LPF 系数
#define CAR_ACCEL_ALPHA_SLOW    (0.05f)                                     // 弯道加速 LPF 系数
#define CAR_SPEED_DEADBAND_CMS  (4.0f )                                     // |err| 小于此值按直道目标速度跑

/* 注意：视觉算法参数（前瞻行、Otsu 间隔、滤波行数、斑马线/出界阈值等）
* 全部集中在 code/vision/vision.h 与 vision.c 内部，以保证 vision.c 与
* summer-smartcar 原文件保持接近逐行一致，便于后续对照原工程调参。
* 上车最常用的两个入口：
*       asc_far（vision.h 的 VIS_PREVIEW_ROW_DEFAULT，菜单 PID>preview 可实时改）
*       VIS_FILTER_BOTTOM_ROWS（底部连通滤波行数，影响视觉耗时） */

/*********************************************************************************************************************
*                                            六、舵机（本车不用舵机，保留宏以便复用转向算法）
*********************************************************************************************************************/
#define SERVO_ENABLE            (0)                                         // 麦轮车不需要舵机，置 0
#define SERVO_CENTER            (95.2f)                                     // 中位角
#define SERVO_MAX_CHA           (14.0f)                                     // 最大偏角（机械限位）

/*********************************************************************************************************************
*                                            七、串口上行（VOFA+ 调参）
* 沿用逐飞 E10_printf_debug_log_demo：debug_init() 后 printf 走 UART1 B12/B13。
* VOFA 选择 FireWater、115200、8N1。每两帧发送20通道，约25Hz。
*********************************************************************************************************************/
#define VOFA_UART_INDEX         (DEBUG_UART_INDEX)                           // printf实际使用的调试串口，不另外初始化UART6
#define VOFA_UART_BAUD          (DEBUG_UART_BAUDRATE)                        // 与库的printf重定向保持一致，默认115200
#define VOFA_SEND_DIV           (2)                                         // 每两帧发送一次（约25Hz），给115200串口留出带宽

/*********************************************************************************************************************
*                                            九、保护阈值
*********************************************************************************************************************/
#define STALL_TARGET_MIN_CMS    (120.0f)                                    // 堵转判定：目标速度需高于此值
#define STALL_ACTUAL_MAX_CMS    (25.0f )                                    // 堵转判定：四轮实际速度均低于此值
#define STALL_FRAMES            (15   )                                     // 堵转判定：连续帧数（50Hz 下 15 帧 = 0.3s）
#define LOST_STOP_FRAMES        (3    )                                     // 出界连续帧数 → 停车
#define ZEBRA_STOP_RISES        (2    )                                     // 斑马线上升沿次数 → 停车

#endif
