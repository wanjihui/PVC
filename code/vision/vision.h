/*********************************************************************************************************************
* 文件名称          vision.h
* 适用平台          RT1064DVL6A
* 描述              总钻风 MT9V03X 图像处理（188×120 灰度）
*
* 移植说明（MM32F327X-G8P → RT1064DVL6A）：
* 算法本体以 summersmartcar 为基础；麦轮版本另外修订单边路宽缓存、
* 中线贴边截断、前瞻缺行误差估计和十字保持。平台差异包括：
*       1) 定时/耗时统计接口（原工程用 TIM8，本工程用 GPT1 的 TIMER_US 模式）
*       2) 移除了全部 IPS200 屏幕显示代码（本工程无屏，改用 WiFi SPI 上传图像
*          与边界/中线到逐飞助手看）
*
* 坐标语义：y=0 是图像上侧/最远处，y=119 是下侧/最近处
*********************************************************************************************************************/

#ifndef VISION_H
#define VISION_H

#include "zf_common_headfile.h"
#include "config.h"

/* ========== 图像尺寸 ========== */
#define pho_h                   MT9V03X_H                                   // 120 行
#define pho_w                   MT9V03X_W                                   // 188 列
#define pho_center_x            (pho_w / 2)                                 // 94，图像水平中心
#define pho_w_min               (0)                                         // 最左列
#define pho_w_max               (pho_w - 1)                                 // 187，最右列
#define BORDER_INVALID          (255)                                       // 边界无效标记（不与 0~187 合法列号冲突）

/* ========== 可调视觉参数（上车标定入口） ========== */
#define VIS_PREVIEW_ROW_DEFAULT CTL_PREVIEW_ROW_DEFAULT                                        // 前瞻基准行 asc_far 默认值
#define VIS_SWEEP_OFFSET        (8 )                                        // 边界跟随起点向内偏移
#define VIS_FILTER_BOTTOM_ROWS  (110)                                       // 底部连通滤波处理行数（越大越慢）
#define VIS_OTSU_INTERVAL       (10)                                        // 大津法每 N 帧重算一次
#define VIS_BORDER_SMOOTH_JUMP  (8 )                                        // 边界 3 点平滑时的跳变保护阈值(px)
#define VIS_TRACE_ROWS          (pho_h)                                     // 轨迹导出的有效行数（供上位机显示边界/中线）
#define VIS_CENTER_EDGE_MARGIN  (2)                                         // 中线距画面边缘<=2px时停止向远处生成
#define VIS_CENTER_SLOPE_ROWS   (5)                                         // 贴边点向近处最多5行估计趋势
#define VIS_CENTER_SLOPE_MAX    (8.0f)                                      // 缺行外推斜率上限(px/行)
#define VIS_CENTER_ERR_SCALE_MAX (2.5f)                                     // 继承summer缺行放大上限；外推err也限于此倍图像半宽

/* ========== 轨迹导出（给 WiFi 图像上传用） ==========
* 本工程没有屏幕，调参靠"图像 + 边界/中线"上传到逐飞助手上位机看。
* 算法层只负责把结果填进下面三个坐标数组，**不调用任何无线/传输函数**，
* 保持 vision.c 与传输方式解耦（换 UART/WiFi/USB 都不用改视觉代码）。
* 约定：Y 全部用行号 0..119；X 用列号 0..187；无效点用 VIS_TRACE_NONE 标记。 */
#define VIS_TRACE_NONE          (0xFF)                                      // 轨迹点无效标记
extern uint8 vis_trace_x_left[VIS_TRACE_ROWS];                              // 左边界列号
extern uint8 vis_trace_y_left[VIS_TRACE_ROWS];                              // 左边界行号
extern uint8 vis_trace_x_center[VIS_TRACE_ROWS];                            // 中线列号
extern uint8 vis_trace_y_center[VIS_TRACE_ROWS];                            // 中线上界行号
extern uint8 vis_trace_x_right[VIS_TRACE_ROWS];                             // 右边界列号
extern uint8 vis_trace_y_right[VIS_TRACE_ROWS];                             // 右边界行号

/* 把当前帧的边界/中线导出到上面的数组（vis_deal 内部自动调用，无需手工触发） */
void vis_export_trace(void);

/* ========== 赛道识别结果 ========== */
typedef uint8 border_line[pho_h];                                           // 类型别名：120个uint8组成的数组，下标是行，值是列

typedef struct
{
    border_line l_border;                                                   // 左边界
    border_line r_border;                                                   // 右边界
    border_line center_line;                                                // 双边中点/真实路宽单边外推/白重心；贴边以上255
    uint8 l_border_exist[pho_h];                                            // 1=真实扫描或十字补线；0=普通搜索假边/缺边，补线需另看repair_flags
    uint8 r_border_exist[pho_h];                                            // 右墙同上，不能单凭exist=1就把该行拿来标定真实路宽
    float err;                                                              // 中线偏离图像中心像素均值，>0 偏右
} VisionResult_t;

extern VisionResult_t g_vis;                                               // 声明vision.c里定义的全局结果，app/control/wifi可读取

/* 同一帧快照用于算法和上传；仅主循环调用，不在采集中断里处理图像。
 * MCAM format=3：原始灰度w*h + 每行4字节[left,right,center,repair_flags] + 12字节标注。
 * 标注：[取值远行,取值近行,拐点有效mask,拐点状态,左上x/y,右上x/y,左下x/y,右下x/y]。
 * repair_flags bit0/1表示左右补线；mask bit0..3对应四拐点；255表示无效坐标。
 * 拐点状态：0无、1候选、2确认、3短暂保持；坐标来自本帧实际识别/保持结果。 */
#define VISION_MONITOR_METADATA_SIZE 12u                                    // 帧尾标注12字节：取值远/近行+拐点mask/状态+四拐点坐标
#define VISION_MONITOR_SIZE (MT9V03X_IMAGE_SIZE + 4u * pho_h + VISION_MONITOR_METADATA_SIZE) // 灰度图+每行4字节叠线+标注的总长度
uint8 vision_capture_frame(void);                                          // 返回1取到新帧/0无新帧，不在此函数内执行算法
void vision_monitor_payload(uint8 *destination);                            // destination指向至少VISION_MONITOR_SIZE字节的可写缓冲
extern volatile uint32 vis_interval_us, vis_processed_frames;               // 已处理帧的开始间隔(us)/数量，不是上传到电脑的帧率
extern volatile uint32 vis_last_event;                                     // 最新完成帧事件码，与下面VisionEvent枚举对应

/* ========== 全局量 ========== */
extern uint8  asc_far;                                                      // 固定前瞻窗口中心行，实际取值中心±10
extern volatile uint8_t  vis_frame_ready;                                   // 新帧已搜线完成（供上行传输层判断数据是否新鲜）
extern volatile int16    asc_range_dbg;                                     // 诊断：ASC 窗口中心行
extern volatile uint32   vis_time_us;                                       // vis_deal 单次耗时(µs)
extern volatile uint8_t  oob_black_rows_dbg;                                // 诊断：底部窗口内全黑行数（0~20）
extern volatile uint8_t  zebra_dbg_detected;                                // 诊断：本帧 zebra_detected（0/1）
extern volatile uint8_t  zebra_dbg_valid_rows;                              // 诊断：本帧斑马线有效采样行数（0~3）
extern volatile uint8_t  zebra_dbg_confirm;                                 // 诊断：本帧斑马线确认计数
extern volatile uint16_t vis_t_bin_us;                                      // 分段耗时：二值化
extern volatile uint16_t vis_t_filter_us;                                   // 分段耗时：底部连通滤波
extern volatile uint16_t vis_t_sweep_us;                                    // 分段耗时：逐行搜线
extern volatile uint16_t vis_t_cross_us;                                    // 分段耗时：十字检测/补线
extern volatile uint16_t vis_t_center_us;                                   // 分段耗时：中线/偏差

/* ========== 二值化图像缓存 ========== */
extern uint8 Image_Used[pho_h][pho_w];                                      // Otsu 二值化结果，0=黑 255=白

/* ========== 视觉事件（供主循环做停车决策） ========== */
typedef enum
{
    VISION_EVT_NONE  = 0,                                                   // 本帧没有新停车事件，不代表每一行边界都识别成功
    VISION_EVT_LOST  = 1,                                                   // 本帧出界
    VISION_EVT_ZEBRA = 2,                                                   // 本帧斑马线上升沿
} VisionEvent;

/* ========== 函数声明 ========== */
void vision_init(void);                                                     // 初始化 GPT1 微秒计时（上电必须调用一次）
VisionEvent vis_deal(void);                                                 // 搜线，返回本帧事件
void cross_reset(void);                                                     // 发车时重置十字/斑马线/路宽/截断状态，无冷却
void vision_reset(void);                                                    // 发车时重置全部视觉状态（十字+斑马线+出界窗口）

/* 说明：原工程里还有 vis_draw() / vis_bin_draw() 两个屏幕显示函数，
* 本工程无屏（用 WiFi SPI 上传图像到逐飞助手看），已整体移除。 */

#endif
