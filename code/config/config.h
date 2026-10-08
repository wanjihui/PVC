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

/* 应用UDP目标、端口和调度周期统一在本文件配置；库内默认TCP配置不参与运行。 */

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
#define MOTOR_DUTY_MAX          (100) // PWM百分比物理范围；原项目60%上限已移除

/* 应用配置：只有一个运行入口，采样与通信始终使用正式模块。
 * 0保持停车、采样和视觉观察；1允许初始化通过后KEY1发车、KEY2停车。
 * 编码器闭环开关独立于硬件采样，核对每厘米计数和PID后才启用闭环。 */
#define APP_DRIVE_ENABLE         (1)
/* 同一app_poll流程：0暂停视觉循迹、四轮固定目标调速；1接回视觉外环。 */
#ifndef APP_VISION_TRACKING_ENABLE
#define APP_VISION_TRACKING_ENABLE (1)
#endif

/* IMU 符号验证模式：只跑 IMU + WiFi，用来核对三角度与三轴角速度/加速度的符号。
 * 1 = 主循环只初始化 IMU 与 WiFi，周期上传角度（不需要 KEY1 发车、不初始化摄像头/
 *     视觉/编码器/按键/速度环；电机只把 PWM 置 0）；上位机选“综合验证”，
 *     8081 参数包前 36 字节 = angles[3](deg) + gyro_dps[3](deg/s) + acc_g[3](g)。
 * 0 = 正常循迹流程；安装符号已实测，当前默认0，K1发车、K2停车。 */
#ifndef APP_IMU_VERIFY_ENABLE
#define APP_IMU_VERIFY_ENABLE    (0)
#endif
/* 验证模式下的文本日志频率（Hz，同时走 UART1 和 8082 日志口）；0 = 关闭 */
#ifndef APP_IMU_VERIFY_TEXT_LOG_HZ
#define APP_IMU_VERIFY_TEXT_LOG_HZ (0)
#endif

#define WIFI_UDP_TARGET_IP       "192.168.50.117"
#define WIFI_DATA_PORT           "8081"
#define WIFI_LOG_PORT            "8082"
#define WIFI_CAMERA_PORT         "8083"
#define WIFI_UDP_LOCAL_PORT      "6666"
#define WIFI_DATA_PERIOD_MS      (50)
#define WIFI_IMAGE_PERIOD_MS     (100) // 上传启动目标10fps；实际完整帧率取决于SPI/网络/主循环
#define WIFI_CAMERA_BURST_CHUNKS (4u)  // 图像连续小批量发送，减少参数/图像之间的socket切换
#define WIFI_CAMERA_BURST_MS     (20u) // 首片完成后的批次时间预算，之后让出最新参数

/* 主循环末尾延时（ms）。图像上传靠"每圈推进一步分片"，所以主循环圈数直接决定帧率上限：
 *   native 模型（tests/native_framework/camera_throughput_after_100_build.log）：
 *     主循环 1ms/圈 + 图像周期100ms → 9.80 fps；主循环 4ms/圈 → 4.00 fps。
 * 设 0 = 主循环全速（图像更快，但空转耗电、日志/按键轮询更密）；设 1 = 与原行为一致。 */
#ifndef APP_MAIN_LOOP_DELAY_MS
#define APP_MAIN_LOOP_DELAY_MS   (1u)
#endif

/* 图像分片"有界补发"：主循环里原本每圈只推进一步分片，一帧 17 片就要 17 圈，
 * 这是真车帧率（实测估算 4~9 fps）最直接的瓶颈。
 * 这里允许同一圈内多推进几步，用两条约束兜住风险：
 *   WIFI_CAMERA_DRAIN_STEPS：一圈最多推进几步（1 = 关闭该功能，与原行为一致）
 *   WIFI_CAMERA_DRAIN_US   ：一圈补发的总时间预算（µs）
 * 另外：本轮若刚处理过新图像，则不补发，保证视觉外环节拍优先。
 * 副作用：补发期间参数包(8081)/日志会被更快推进，但单圈最坏变长到约 1+DRAIN_US。 */
#ifndef WIFI_CAMERA_DRAIN_STEPS
#define WIFI_CAMERA_DRAIN_STEPS  (4u)
#endif
#ifndef WIFI_CAMERA_DRAIN_US
#define WIFI_CAMERA_DRAIN_US     (8000u)
#endif
#define WIFI_LOG_PERIOD_MS       (1000)
#define WIFI_RETRY_MS            (5000)
#define IMU_CAL_SAMPLES          (300)
#define IMU_FUSION_KP            (2.0f)

/* 电机安装方向修正：0=该轮正占空比使车前进；1=取反（电机线接反/轮子镜像安装时改这里，不要改接线）
* 麦轮四轮需要左右镜像安装，实际转向常与"正占空比前进"不一致，故留此补偿表。
* 上车第一步就是让四个轮子同时正转，用本表把方向逐个纠正过来。 */
#define MOTOR1_DIR_INVERT       (0)
#define MOTOR2_DIR_INVERT       (1)
#define MOTOR3_DIR_INVERT       (0)
#define MOTOR4_DIR_INVERT       (0)

/*-------------------------------------- 编码器：四路正交解码 -------------------------------------*/
#ifndef ENCODER_CLOSED_LOOP_ENABLE
#define ENCODER_CLOSED_LOOP_ENABLE          (1)                            // 当前开启四路编码器反馈，使用初调PI参数
#endif
#define MOTOR_OPEN_LOOP_GAIN    (0.10f)                                     // 临时映射：轮速指令(cm/s) × 此系数 → PWM百分比，非实测速率
#define MOTOR_OPEN_LOOP_MAX     (20)                                        // 无编码器调试时四轮占空比限制 ±20%

/* 1024线编码器，恢复逐飞原库一倍频（A上升沿计数，B决定方向）；68齿轮与车轮同轴并直接啮合30齿编码器齿轮。
 * 每厘米计数 = 1024 × 1 × (68/30) / 19.5 ≈ 119.02906。
 * 原始计数仍为整数；速度/里程分别用 cm/s、cm。 */
#define ENCODER_LINES_PER_REV    (1024.0f)
#define ENCODER_DECODE_MULTIPLIER (1.0f)
#define ENCODER_WHEEL_GEAR_TEETH  (68.0f)
#define ENCODER_SHAFT_GEAR_TEETH  (30.0f)
#define ENCODER_COUNTS_PER_CM    (ENCODER_LINES_PER_REV * ENCODER_DECODE_MULTIPLIER * \
                                 (ENCODER_WHEEL_GEAR_TEETH / ENCODER_SHAFT_GEAR_TEETH) / \
                                 CHASSIS_WHEEL_CIRCUMFERENCE_CM)

/* 2026-10-07手动转轮确认：原FL/FR字段对应反了。
 * 编号仍按FL/FR/RL/RR，交换完整硬件通道与方向补偿，而非只交换UDP字段。 */
#define ENCODER_1               (QTIMER1_ENCODER2)                         // 实际左前 FL
#define ENCODER_1_A             (QTIMER1_ENCODER2_CH1_C2 )
#define ENCODER_1_B             (QTIMER1_ENCODER2_CH2_C24)
#define ENCODER_2               (QTIMER1_ENCODER1)                         // 实际右前 FR
#define ENCODER_2_A             (QTIMER1_ENCODER1_CH1_C0 )
#define ENCODER_2_B             (QTIMER1_ENCODER1_CH2_C1 )
#define ENCODER_3               (QTIMER2_ENCODER1)
#define ENCODER_3_A             (QTIMER2_ENCODER1_CH1_C3 )
#define ENCODER_3_B             (QTIMER2_ENCODER1_CH2_C4 )
#define ENCODER_4               (QTIMER2_ENCODER2)
#define ENCODER_4_A             (QTIMER2_ENCODER2_CH1_C5 )
#define ENCODER_4_B             (QTIMER2_ENCODER2_CH2_C25)

/* 编码器方向修正：0=正转计数递增；1=取反。与 MOTORx_DIR_INVERT 配合，
* 目标是"车轮前进时该轮编码器读数与速度目标同号" */
#define ENCODER_1_INVERT        (1)                                         // 随原QTIMER1_ENCODER2通道交换，保持前转为正
#define ENCODER_2_INVERT        (0)                                         // 随原QTIMER1_ENCODER1通道交换
#define ENCODER_3_INVERT        (1)                                         // 用户实测正转原始计数为负，反向归一
#define ENCODER_4_INVERT        (0)

/*-------------------------------------- IMU：ICM20602（硬件 SPI4） -------------------------------------*/
/* 引脚由 libraries/zf_device/zf_device_icm20602.h 固定：SCK=C23 MOSI=C22 MISO=C21 CS=C20
* 如需改动请直接改该头文件的宏，不要在业务层重复定义 */
#define IMU_MOUNT_YAW_SIGN      (-1.0f)                                      // 2026-10-08实测gz左转负/右转正，控制反馈反号：左正右负（上传原始gz不变）

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
#define WIFI_SSID               "ASC_Circuit"                               // 当前电脑连接的WiFi
#define WIFI_PASSWORD           "1145141919810"                             // 用户提供的WiFi密码；运行日志不输出

/* 图像上传降频：主循环 50Hz，但一帧 188*120 图像经无线传输耗时较长（百毫秒级），
* 每帧都传会把主循环拖慢。取 N 表示"每 N 帧传一次"，默认 25 → 约 2Hz。
* 注意：传输在主循环里做（不在 200Hz 中断），所以**不会影响速度环**，
*       真正被拖慢的只是视觉帧率；调参时若嫌慢就调小，稳定跑车时应调大或置 0 关闭。 */

/* 是否上传边界/中线（XY_BOUNDARY，3 条线：左边界/中线/右边界）。
* 开启后上位机能直接看到算法提取的中线，是无屏调参的主力手段。 */
#define WIFI_SEND_BOUNDARY      (1)

/*-------------------------------------- 按键：主板四键 C15/C14/C13/C12 -------------------------------------*/
/* 按键与 WiFi SPI(D12~D15,B16,B17) 及 VOFA(UART6 B2/B3) 均无引脚冲突，故保留，
* 用于无电脑时的手动发车/停车。 */
#define KEY_STOP_DEBOUNCE_MS    (20) // K2连续按下20ms由扫描中断清PWM
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
#define CHASSIS_WHEELBASE_MM    (200.0f)                                    // 用户实测 L=20cm，完整前后轮中心距
#define CHASSIS_TRACK_MM        (180.0f)                                    // 用户实测 W=18cm，完整左右轮中心距
#define CHASSIS_WHEEL_CIRCUMFERENCE_CM (19.5f)                              // 用户提供车轮周长，单位cm
#define CHASSIS_WHEEL_DIA_MM    (CHASSIS_WHEEL_CIRCUMFERENCE_CM * 10.0f / 3.14159265358979323846f) // 周长推算等效直径，约62.07043mm
#define CHASSIS_GEAR_RATIO      (4.25f)                                     // 用户外部传动68/16；未包含可能存在的电机内部减速

/* 麦轮辊子方向修正（整体符号开关）
*
* 标准麦轮（辊子成"X 形"，辊子轴向：俯视且车头朝上：左前/右后 = 左上—右下(\)，右前/左后 = 左下—右上(/)）的运动学特征：
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

/*-------------------------------------- 转向环参数（视觉 err → 自转 ω）--------------------------------------
 * 公式（control.c 的 omega_calc / control_tick_isr）：
 *      e      = 赛道中线 − 画面中心 (px)，e>0 表示赛道在画面右侧
 *      ω_vis  = TURN_SIGN * ( KP1*e + KP2*e*|e| + KD*(e − e上次) )        TURN_SIGN=−1
 *      ω_out  = clamp( ω_vis − GYRO_KD*gyro_z , ±OMEGA_MAX )              gyro_z 已归一化为"左转正"
 * ω → 四轮（motor.c 的 mecanum_inverse）：
 *      rot[cm/s] = (轴距200+轮距180)/2/1000 × ω[rad/s] × 100 = 0.3316 × ω[deg/s]
 *      FL/RL = vx − rot，FR/RR = vx + rot
 * 转弯半径 R = v/ω：150cm/s 时 120°/s → 72cm（急弯必然转不过去）；240°/s → 36cm；360°/s → 24cm
 * 建议调参顺序：KP1（普通偏差的转向力度）→ OMEGA_MAX（急弯能否转过来）→ GYRO_KD（摆头）
 *              → KP2（大偏差补强）→ KD（入弯超调）→ 仍不够再动速度环 SPEED_KP/KI */
#define CTL_OMEGA_KP1_DEFAULT      (1.60f)  // 线性比例 (deg/s)/px。0.6=转向无力；>2.5 易画龙/超调
#define CTL_OMEGA_KP2_DEFAULT      (0.012f) // 二次项 (deg/s)/px²，只在 |e|>60px 起作用，专治急弯
#define CTL_OMEGA_KD_DEFAULT       (0.30f)  // 微分 (deg/s)/(px/帧)：抑制入弯超调；视觉噪声大时不宜再大
#define CTL_OMEGA_MAX_DPS_DEFAULT  (240.0f) // 自转限幅 (deg/s)：卡住急弯的关键，见上面 R 的算法
#define CTL_GYRO_KD_DEFAULT        (0.20f)  // 陀螺阻尼 0~1：抑制摆头；过大转向变钝。符号由 IMU_MOUNT_YAW_SIGN 保证
#define CTL_DEADBAND_PX_DEFAULT    (2.0f)   // 偏差死区 (px)：直道防抖
#define CTL_LATERAL_GAIN_DEFAULT   (0.0f)   // 横移 vy 增益：0=关闭（纯航向控制）；麦轮横移贴线可试 0.1~0.5

/* 弯道降速：|e| 超过 CTL_CORNER_ERR_PX 后，前进速度最多降到 (1−RATIO)。
 * 原理：R = v/ω，弯里降速等于直接放大转向能力；设 RATIO=0 即关闭该功能。 */
#define CTL_CORNER_ERR_PX_DEFAULT  (100.0f) // 触发降速的偏差门槛 (px)
#define CTL_CORNER_SLOW_RATIO_DEFAULT (0.45f) // 最大降速比例：150cm/s × (1−0.45) ≈ 82cm/s

#define CONTROL_GYRO_KD_DEFAULT (CTL_GYRO_KD_DEFAULT) // 兼容旧名，等价于 CTL_GYRO_KD_DEFAULT

/*********************************************************************************************************************
*                                            四、速度环参数（增量式 PID）
* Target/Actual 单位统一为 cm/s。轮速 → cm/s 的换算见 encoder.c
*********************************************************************************************************************/
/* 待实车调参的初始PI值。Ki为每5ms一步的离散积分系数，不再额外乘周期。
 * 巡航150cm/s；500ms斜坡首拍目标3cm/s，静止时首拍约0.303%反馈输出。
 * 四轮先相同，之后根据目标/实际/占空比曲线分别调整。 */
/* 2026-10-08 实跑数据（数据记录_20261008_212130）显示：Kp=0.10 时四轮实际速度比目标低 15~20%
 *   FL/RL 目标127 实际122、FR/RR 目标152 实际138 cm/s，而占空比只有 11~17%
 *   → 不是功率不够，是增益太小、跟随太软；转弯时外侧轮提不上速，实际自转速率只有指令的 73%。
 * 本轮提高 Kp/Ki，验证 duty 与 speed 的跟踪误差能否收敛到 ±5cm/s 以内。 */
#define SPEED_KP_FL             (0.20f)
#define SPEED_KI_FL             (0.0030f)
#define SPEED_KD_FL             (0.0f)
#define SPEED_KP_FR             (0.20f)
#define SPEED_KI_FR             (0.0030f)
#define SPEED_KD_FR             (0.0f)
#define SPEED_KP_RL             (0.20f)
#define SPEED_KI_RL             (0.0030f)
#define SPEED_KD_RL             (0.0f)
#define SPEED_KP_RR             (0.20f)
#define SPEED_KI_RR             (0.0030f)
#define SPEED_KD_RR             (0.0f)
/* 前馈：仅当 Target>Actual（加速）时叠加 duty = Kff*(Target-Actual)，用来减小加速滞后。
 * 0=关闭；0.02~0.05 起步更跟手；过大会在弯道加速段顶到占空比上限。 */
#define SPEED_FF                (0.02f)

/*********************************************************************************************************************
*                                            五、固定前进速度
*********************************************************************************************************************/
#define CAR_TARGET_SPEED_CMS    (150.0f) // 固定车体前进速度：直弯同速，cm/s
#define CAR_START_RAMP_MS       (500u)   // K1后目标vx从0线性升到巡航速度，5ms节拍推进；K2立即停车

/* 注意：视觉算法参数（前瞻行、Otsu 间隔、滤波行数、斑马线/出界阈值等）
* 全部集中在 code/vision/vision.h 与 vision.c 内部，以保证 vision.c 与
* summer-smartcar 原文件保持接近逐行一致，便于后续对照原工程调参。
* 上车最常用的两个入口：
*       ctl_preview_row（vision.h 的 VIS_PREVIEW_ROW_DEFAULT，固定前瞻中心行）
*       VIS_FILTER_BOTTOM_ROWS（底部连通滤波行数，影响视觉耗时） */

/* UART1只保留启动及事件日志备份；运行参数通过UDP上传。 */

/*********************************************************************************************************************
*                                            九、保护阈值
*********************************************************************************************************************/
#define STALL_TARGET_MIN_CMS    (120.0f)                                    // 堵转判定：目标速度需高于此值
#define STALL_ACTUAL_MAX_CMS    (25.0f )                                    // 堵转判定：四轮实际速度均低于此值
#define STALL_FRAMES            (15   )                                     // 堵转判定：连续帧数（50Hz 下 15 帧 = 0.3s）
#define LOST_STOP_FRAMES        (3    )                                     // 出界连续帧数 → 停车
#define ZEBRA_STOP_RISES        (2    )                                     // 斑马线上升沿次数 → 停车

#endif
