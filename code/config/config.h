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
#ifndef WIFI_SINGLE_PORT_ENABLE
#define WIFI_SINGLE_PORT_ENABLE  (1) // 1=固定单socket，MUDP类型头区分参数/日志/图像；0=旧三端口
#endif
#define WIFI_MONITOR_PORT        "8081" // 单端口上位机监听端口；模块不因包类型重建socket
#define WIFI_DATA_PORT           "8081"
#define WIFI_LOG_PORT            "8082"
#define WIFI_CAMERA_PORT         "8083"
#define WIFI_UDP_LOCAL_PORT      "6666"
#define WIFI_DATA_PERIOD_MS      (50)
#define WIFI_IMAGE_PERIOD_MS     (66) // 约15fps启动目标；完整帧率仍需用上位机实测
#define WIFI_CAMERA_BURST_CHUNKS (4u)  // 图像连续小批量发送后让出参数，单端口不切socket
#define WIFI_CAMERA_BURST_MS     (20u) // 首片完成后的批次时间预算，之后让出最新参数

/* 图像/参数的异步SPI事务靠主循环持续推进，删除每圈1ms的强制等待。
 * review_camera_poll_rate.py：40MHz SPI、每20ms模拟1.5ms视觉处理时，
 * 原1ms轮询/100ms上传=8.6fps；100us轮询/66ms上传=14.2fps（软件模型，非实车保证）。
 * 设0=全速服务；不改变5ms轮速/IMU、10ms按键、相机50Hz和视觉按新帧执行的周期。 */
#ifndef APP_MAIN_LOOP_DELAY_MS
#define APP_MAIN_LOOP_DELAY_MS   (0u)
#endif

#define WIFI_COMMAND_POLL_MS    (10u) // 空闲事务间非阻塞读取模块RX，指令与上传共用SPI
#define WIFI_LOG_PERIOD_MS       (1000)
#define WIFI_RETRY_MS            (5000)
#define WIFI_TX_FAILURE_LIMIT       (3u)   // 连续事务失败后向app报告离线，停车才重新入网
#define WIFI_FAILURE_BACKOFF_MS     (1000u)
#define WIFI_STATUS_PERIOD_MS       (3000u) // 低频日志区分停车/主循环进度/SPI故障；0=关闭
#define WIFI_HOST_TIMEOUT_MS        (8000u) // 首次上位机ALIVE后启用端到端检测
#define WIFI_HOST_RECOVERY_MS       (5000u) // 先异步重建socket；无回包再停车重入网
#define VIS_CROSS_CONFIRM_FRAMES    (2u)
#define VIS_CROSS_HOLD_FRAMES       (4u) // 50Hz下最多保持80ms，随后清除旧补线
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
/* SSID/密码在此配置，IP/三路端口与发送节拍见 wifi.h；app_init入网，app_poll异步收发。 */
#define WIFI_SSID               "ASC_Circuit"                               // 当前电脑连接的WiFi
#define WIFI_PASSWORD           "1145141919810"                             // 用户提供的WiFi密码；运行日志不输出

/* 图像目标100ms启动一帧；快照+分片DMA异步推进，不等待无线发送完成。
 * 实际上位机帧率受主循环推进与丢包影响，视觉仍按摄像头新帧计算。 */

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
 * 转弯半径 R = v/ω：150cm/s 时 120°/s → 72cm（理想无滑移计算）；240°/s → 36cm；360°/s → 24cm
 * 建议调参顺序：KP1（普通偏差的转向力度）→ OMEGA_MAX（急弯能否转过来）→ GYRO_KD（摆头）
 *              → KP2（大偏差补强）→ KD（入弯超调）→ 仍不够再动速度环 SPEED_KP/KI */
#define CTL_OMEGA_KP1_DEFAULT      (1.60f)  // 线性比例 (deg/s)/px。普通偏差强度；过大可能摆动，数值需实车迭代
#define CTL_OMEGA_KP2_DEFAULT      (0.012f) // 二次项 (deg/s)/px²，所有非零e均参与；偏差大时相对线性项更强
#define CTL_OMEGA_KD_DEFAULT       (0.30f)  // 微分 (deg/s)/(px/帧)：每帧差分，提前响应快速偏差；也会放大视觉噪声
#define CTL_OMEGA_MAX_DPS_DEFAULT  (240.0f) // 自转限幅 (deg/s)：最大允许指令，理想半径见R公式，实车还受滑移/轮速响应影响
#define CTL_GYRO_KD_DEFAULT        (0.30f)  // 陀螺阻尼 0~1：抑制摆头；过大转向变钝。符号由 IMU_MOUNT_YAW_SIGN 保证
#define CTL_PREVIEW_ROW_DEFAULT   (60u)   // 固定预瞄中心行20..100；用户试跑选择60，取值50..70
#define CTL_DEADBAND_PX_DEFAULT    (2.0f)   // 偏差死区 (px)：直道防抖
#define CTL_LATERAL_GAIN_DEFAULT   (0.10f)  // (cm/s)/px：小幅横移贴线，50px→5cm/s；先试0.10，过冲时减小

/* 弯道速度规划：固定前瞻不变，用未叠加陀螺阻尼的视觉转向需求分级。
 * 只降低车体vx，omega/vy及四轮PID继续运行；0可回退直弯同速。 */
#ifndef CTL_CURVE_SPEED_ENABLE
#define CTL_CURVE_SPEED_ENABLE     (1)     // 仅视觉循迹模式启用，独立轮速调试不分弯道
#endif
#define CTL_CURVE_NORMAL_CMS       (120.0f) // 普通弯规划上限且不超过cruise_cms；过渡按减速率
#define CTL_CURVE_TIGHT_CMS        (100.0f) // 紧弯规划上限；进入后保持到稳定出弯
#define CTL_CURVE_ENTER_DPS        (70.0f)  // 普通弯：视觉转向需求绝对值达到此阈值
#define CTL_CURVE_TIGHT_DPS        (140.0f) // 紧弯：视觉转向需求绝对值达到此阈值
#define CTL_CURVE_CONFIRM_FRAMES   (2u)    // 连续新帧确认降速等级，单帧D项尖峰不立即分弯
#define CTL_CURVE_EXIT_DPS         (45.0f) // 出弯视觉需求阈值，低于进入阈值形成滞回
#define CTL_CURVE_EXIT_ERR_PX      (12.0f) // 出弯像素偏差绝对值需低于此值
#define CTL_CURVE_EXIT_GYRO_DPS    (40.0f) // 出弯实际角速度绝对值需低于此值
#define CTL_CURVE_EXIT_STABLE_MS   (200u)  // 上述条件连续成立后允许恢复直道速度
#define CTL_CURVE_FRAME_MAX_AGE_MS (100u)  // 旧视觉帧超过此年龄，不计稳定时间/不继续加速
#define CTL_CURVE_DECEL_CMS2       (250.0f) // 目标vx下降速率；每5ms最多下降1.25cm/s
#define CTL_CURVE_ACCEL_CMS2       (100.0f) // 出弯恢复速率；每5ms最多上升0.50cm/s


#define CONTROL_GYRO_KD_DEFAULT (CTL_GYRO_KD_DEFAULT) // 兼容旧名，等价于 CTL_GYRO_KD_DEFAULT

/*********************************************************************************************************************
*                                            四、速度环参数（增量式 PID）
* Target/Actual 单位统一为 cm/s。轮速 → cm/s 的换算见 encoder.c
*********************************************************************************************************************/
/* 待实车调参的初始PI值。Ki为每5ms一步的离散积分系数，不再额外乘周期。
 * 巡航150cm/s；500ms斜坡首拍目标1.5cm/s，静止时FL约0.3315%、FR/RR约0.3015%、RL约0.2715%。
 * 四轮按前后组起调，之后根据目标/实际/占空比曲线分别调整。 */
/* 2026-10-12：排除起步0.8秒及末0.5秒后FL/RR平均欠速约9.3/7.4cm/s，
 * 只小幅提高FL 0.20→0.22、RR 0.18→0.20；FR/RL、Ki及视觉/陀螺参数保持。待实车验证。 */
#define SPEED_KP_FL             (0.22f)
#define SPEED_KI_FL             (0.0010f)
#define SPEED_KD_FL             (0.0f)
#define SPEED_KP_FR             (0.20f)
#define SPEED_KI_FR             (0.0010f)
#define SPEED_KD_FR             (0.0f)
#define SPEED_KP_RL             (0.18f)
#define SPEED_KI_RL             (0.0010f)
#define SPEED_KD_RL             (0.0f)
#define SPEED_KP_RR             (0.20f)
#define SPEED_KI_RR             (0.0010f)
#define SPEED_KD_RR             (0.0f)
/* 误差附加项（并非按目标速度标定的独立前馈）：仅Target>Actual时叠加 duty = Kff*(Target-Actual)，用来减小加速滞后。
 * 0=关闭；0.02~0.05 起步更跟手；过大会在弯道加速段顶到占空比上限。 */
#define SPEED_FF                (0.0f)

/*********************************************************************************************************************
*                                            五、固定前进速度
*********************************************************************************************************************/
#define CAR_TARGET_SPEED_CMS    (150.0f) // 直道巡航上限cm/s；弯道按CTL_CURVE_*规划
#define CAR_START_RAMP_MS       (500u)   // K1后0→巡航起步包络；弯中不超过限速，K2立即清PWM

/* 注意：视觉算法参数（前瞻行、Otsu 间隔、滤波行数、斑马线/出界阈值等）
* 固定前瞻默认值集中在本文件；其余视觉细节见 code/vision/vision.h 与 vision.c，以保证 vision.c 与
* summer-smartcar 原文件保持接近逐行一致，便于后续对照原工程调参。
* 上车最常用的两个入口：
*       CTL_PREVIEW_ROW_DEFAULT（本文件，固定前瞻中心行）
*       VIS_FILTER_BOTTOM_ROWS（底部连通滤波行数，影响视觉耗时） */

/* UART1只保留启动及事件日志备份；运行参数通过UDP上传。 */

/*********************************************************************************************************************
*                                            九、保护阈值
*********************************************************************************************************************/
#define STALL_TARGET_MIN_CMS    (60.0f)                                     // 覆盖100cm/s弯速；更低启动/慢速阶段不计堵转
#define STALL_ACTUAL_MAX_CMS    (25.0f )                                    // 堵转判定：四轮实际速度均低于此值
#define STALL_FRAMES            (15   )                                     // 堵转判定：连续帧数（50Hz 下 15 帧 = 0.3s）
#define LOST_STOP_FRAMES        (3    )                                     // 出界连续帧数 → 停车
#define ZEBRA_STOP_RISES        (2    )                                     // 斑马线上升沿次数 → 停车

#endif
