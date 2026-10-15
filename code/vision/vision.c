
#include "vision.h"
#include "config.h"


/* 阅读约定：y=0在画面顶部(远处)，y=119在底部(近处)，x向右增大。
 * static文件级变量只供本文件使用；未显式赋值的全局/静态变量上电自动为0。
 * volatile要求编译器实际读取/写入内存，不代表加锁或保证多个字段是同一时刻的快照。 */
VisionResult_t g_vis;                        // 当前处理结果：左右边界、逐行中线、边界有效标志、像素偏差err
static uint8 vision_gray[pho_h][pho_w];      // 已取出的原始灰度帧；算法和图像上传共用它，防止灰度与叠线错帧
static uint8 vision_repair_flags[pho_h];     // 每行补线来源：bit0=1表示左补线，bit1=1表示右
static uint8 vision_sample_far = VIS_PREVIEW_ROW_DEFAULT - 10u;  // 前瞻上边界
static uint8 vision_sample_near = VIS_PREVIEW_ROW_DEFAULT + 10u; // 前瞻下边界
static uint8 vision_corners_in_frame;        // 本帧拐点标志位
static void vision_monitor_metadata(uint8 *destination); // 函数声明，函数体在文件末尾；填前瞻位置和实际/保持拐点,给上位机
static uint32 vision_previous_start;         // 上一次vis_deal开始处理的间戳
volatile uint32 vis_interval_us, vis_processed_frames, vis_last_event;
// 分别为处理间隔、已处理帧数、最新帧事件(0无/1出界/2斑马线上升沿)

uint8 vision_capture_frame(void) // 本函数只取一帧图：返回1=成功；0=无新图，主循环不重复处理旧图  
{
    uint32 csi_enabled = NVIC_GetEnableIRQ(CSI_IRQn); // 记住CSI原来是否开中断，结束后按原状态恢复
    uint8 captured = 0;                             // 默认没有取到新帧
    NVIC_DisableIRQ(CSI_IRQn);                      // 暂停摄像头完成回调，避免复制途中完成标志/缓冲引用发生变化
    __DSB(); __ISB();                               // 确保关中断操作生效后才继续；不关闭PIT电机/IMU节拍
    /* CSI回调不回收当前完成缓冲；短暂复制期间PIT采样/按键中断仍运行。 */
    if (mt9v03x_finish_flag)                        // 摄像头驱动已通知一帧采集完成，才消费这份图
    {
        mt9v03x_finish_flag = 0;                    // 帧消费标志位
        memcpy(vision_gray, mt9v03x_image, sizeof(vision_gray)); // 复制188×120=22560字节
        captured = 1;                              // 返回新帧就绪标志位
    }
    if (csi_enabled) NVIC_EnableIRQ(CSI_IRQn);       // 原来关闭就保持关闭，避免擅自打开CSI中断
    return captured;     
}                         

void vision_monitor_payload(uint8 *destination) // 写图像负载到缓冲
{
    uint16 y;
    memcpy(destination, vision_gray, sizeof(vision_gray)); // 放原始灰度图
    destination += sizeof(vision_gray);                   // 指针后移22560字节，放逐行叠线坐标
    for (y = 0; y < pho_h; y++)
    {
        uint8 flags = vision_repair_flags[y];             // 两个位可同时置1：该行左右都经过十字补线
        uint8 l = g_vis.l_border[y], r = g_vis.r_border[y], c = g_vis.center_line[y];
        /* 普通单边丢失的内部搜索外推不属于十字补线，不显示虚构的紫色边界。 */
        if (!g_vis.l_border_exist[y] && !(flags & 1u)) l = BORDER_INVALID;
        if (!g_vis.r_border_exist[y] && !(flags & 2u)) r = BORDER_INVALID;
        *destination++ = l < pho_w ? l : BORDER_INVALID;  // 每行第1字节左边界；无效255则让上位机跳过
        *destination++ = r < pho_w ? r : BORDER_INVALID;  // 第2字节右边界；*p++表示先写当前位置，再把指针向后移1字节
        *destination++ = c < pho_w ? c : BORDER_INVALID;  // 第3字节中线；贴边截断以上的255保持不显示
        *destination++ = flags;                          // 第4字节补线位图；颜色由上位机根据它决定
    }
    vision_monitor_metadata(destination);                // 最后追加12字节前瞻/拐点标注
}

volatile uint8_t vis_frame_ready;          // 新帧处理完成标志位
volatile int16  asc_range_dbg = 0;          // 诊断：实际窗口中心行，夹到20..100；图像包通过far/near传前瞻位置
volatile uint8_t zebra_dbg_detected   = 0;  /* 诊断：本帧 zebra_detected（0/1） */
volatile uint8_t zebra_dbg_valid_rows  = 0;  /* 诊断：本帧斑马线有效采样行数（0~3） */
volatile uint8_t zebra_dbg_confirm     = 0;  /* 诊断：当前斑马线确认/单行预确认计数；确认门限由ZEBRA_CONFIRM_FRAMES决定 */

/* 分段耗时诊断（us），供 VOFA/调参定位视觉瓶颈 */
volatile uint32   vis_time_us      = 0;    /* vis_deal 单帧总耗时(us)，GPT1 微秒计时 */
volatile uint16_t vis_t_bin_us     = 0;     // 整图二值化耗时(us)，Otsu重算耗时仅计入vis_time_us总耗时
volatile uint16_t vis_t_filter_us  = 0;     // 底部连通滤波耗时(us)
volatile uint16_t vis_t_sweep_us   = 0;     // 边界扫描及扫描内斜入判据耗时(us)
volatile uint16_t vis_t_cross_us   = 0;     // 十字确认/保持/补线耗时(us)，不执行该段时接近0
volatile uint16_t vis_t_center_us  = 0;     // 中线生成、贴边截断及前瞻误差计算耗时(us)

 uint8 asc_far = VIS_PREVIEW_ROW_DEFAULT;             // 固定前瞻中心行，默认60；实际采样范围中心±10
static int16 last_valid_width = 60;            // 当前扫描参考宽度：真实双边测量，或底行单边的可见宽度下界
static int   last_real_width  = 60;            // 最近帧最靠近车身的真实双边宽度；不能被远处窄路宽覆盖
static uint8 learned_row_width[pho_h];        // 同一图像行最近一次真实双边路宽；停车/发车保留，不能拿远处窄宽度覆盖近行
static uint8 sweep_row_width[pho_h];          // 本帧各行外推宽度估计，保存裁剪之前的依据；单边下界不等于精确完整宽度
static int16 center_cut_row = -1;             // 本帧中线首次贴边行；-1表示未贴边
static int16 center_cut_col;                  // 贴边处未裁剪的中心，可超出0..187
static float center_cut_slope;               // 向远处外推的像素/行，仅用于err，不伪造上方黄线
static uint8 raw_l_border[pho_h];              // 十字补线前的原始左边界（标定用）
static uint8 raw_r_border[pho_h];              // 十字补线前的原始右边界（标定用）
static uint8 raw_l_exist[pho_h];               // 十字补线前的原始左边界 real 标志（0/1）
static uint8 raw_r_exist[pho_h];               // 十字补线前的原始右边界 real 标志（0/1）
#define SWEEP_OFFSET  VIS_SWEEP_OFFSET                          // 上行搜索起点向内侧偏移量（参考工程值）

/* ================================================================
 * 十字识别与补线 — 编译期常量（四拐点候选版）
 * ================================================================ */
#define CROSS_WHITE_ZONE_MIN           3    /* 连续白区行数阈值（≥3 判为十字候选。实车远处横道二值化后只占3行，5行会漏检）*/
#define CROSS_WHITE_ZONE_FAR_ROW_MIN   31   /* 白区远侧行号必须 >=31，防止白区伸到太远/顶部 */
#define CROSS_DN_ROW_OFFSET            5    /* 下拐点起始行 = near_row + 5 */
#define CROSS_UP_ROW_OFFSET            3    /* 上拐点起始行 = far_row  - 3 */
#define CROSS_DN_SNAP_ROW            115    /* 下拐点行 > 此值 → 用屏幕底角 */
#define CROSS_ANCHOR_SEARCH_MAX       20    /* 拐点从起始行向外最多再搜 20 行 */
#define CROSS_ANCHOR_STABLE_GAP        3    /* 稳定判据：用相隔 3 行的两个边界点比较 */
#define CROSS_ANCHOR_STABLE_COL        8    /* 相隔 3 行列号变化 ≤ 8px 才算稳定（≈2.7px/行）。
                                             * 4px 过苛：正常弯道边沿 3 行可移动 6~9px，会把锚点推到很远；
                                             * 横道恢复斜坡实测 10~15px/3行，8px 仍能把它排除。 */
#define CROSS_CORNER_JUMP_COL          6    /* 四拐点校验：拐点与横道之间必须出现 ≥6px 跳变或 INVALID 行 */
#define CROSS_CONFIRM_FRAMES           VIS_CROSS_CONFIRM_FRAMES
#define CROSS_HOLD_MISSED_FRAMES       VIS_CROSS_HOLD_FRAMES

/* 斜入耦合扫线 */
#define CROSS_OBLIQUE_LOST_MIN         7u   /* 同一侧连续丢边达到7行才允许启动斜入下拐点搜索（行数，不是帧数） */
#define CROSS_OBLIQUE_CONFIRM_ROWS     3u   /* 找到上拐点候选后，还需再向上扫过3行稳定段才确认 */
#define CROSS_OBLIQUE_LOST_SIDE_MAX_REAL_ROWS 10u  /* 斜入段丢线侧允许的真实边界行数 */
/* 斜入单边丢重扫起点：左墙有角(右丢)用168，右墙有角(左丢)用20 */
#define CROSS_OBLIQUE_RESET_COL_LEFT_REAL   168u
#define CROSS_OBLIQUE_RESET_COL_RIGHT_REAL  20u
#define CROSS_OBLIQUE_RESET_COL(real_side) \
    ((real_side) == 1 ? CROSS_OBLIQUE_RESET_COL_LEFT_REAL \
                      : CROSS_OBLIQUE_RESET_COL_RIGHT_REAL)
#define CROSS_EDGE_LOST_L              (-12)  /* 左墙未扫到时的哨兵：小于合法列0，与真实列作差可看出左墙"消失"的方向 */
#define CROSS_EDGE_LOST_R              200    /* 右墙未扫到时的哨兵：大于最大列187，方向与左墙哨兵相反 */

/* 标定阈值（供斜入撕裂辅助函数使用；放宽版：下拐点更容易触发） */
#define CROSS_STABLE                   10  /* 稳定段判据：相邻行有角侧列号变化超过10px就认为仍在过渡 */
#define CROSS_JUMP1                    1   /* 相隔1行的撕裂差值门限1px，必须联合稳定段和另外两次比较，不能单独确认拐点 */
#define CROSS_JUMP2                    2   /* 与相隔2/3行的坐标差比较，门限2px；左/右墙按各自符号判断 */

/* 嵌入式 abs 宏，避免依赖 stdlib */
#define CROSS_ABS(x)  ((x) < 0 ? -(x) : (x))


/* ================================================================
 * 斑马线检测 — 编译期常量
 * ================================================================ */
#define ZEBRA_SAMPLE_ROWS              3    /* 底部采样行数 */
#define ZEBRA_SAMPLE_ROW_STEP          5    /* 采样行间隔 */
#define ZEBRA_VALID_ROWS_MIN           2    /* 最少有效采样行数（至少2行，且必须竖向对齐） */
#define ZEBRA_WHITE_RATIO_MIN         25    /* 白像素占比下限(%) */
#define ZEBRA_WHITE_RATIO_MAX         75    /* 白像素占比上限(%) */
#define ZEBRA_TRANSITIONS_MIN         10    /* 最少黑白跳变次数 */
#define ZEBRA_RUN_WIDTH_MIN            4    /* 连续同色段最小宽度 */
#define ZEBRA_BLACK_RUNS_MIN           5    /* 最少黑色段数 */
#define ZEBRA_WHITE_RUNS_MIN           5    /* 最少白色段数 */
#define ZEBRA_FILTER_RADIUS            2    /* 中值滤波半径(±2=5像素窗口) */
#define ZEBRA_HOLD_MISSED_FRAMES       2    /* 漏检保持帧数 */
#define ZEBRA_CONFIRM_FRAMES           1    /* 连续确认帧数：2→1。Round25 数据证实终点只出现 1 帧有效斑马线（valid_rows=2, confirm=1），2 帧会漏检；坡道误触发靠竖向对齐+有效行数抑制 */
#define ZEBRA_TRACK_MARGIN             3    /* 赛道窗口向外扩3列 */
/* 竖向条纹一致性：真斑马线是竖条纹，119/114/109 三行的黑白跳变位置应基本对齐。
 * 阈值噪声/坡道灰度抖动产生的"假条纹"各行动作随机，用跳变列对齐率剔除。 */
#define ZEBRA_TRANS_COLS_MAX          40    /* 每行最多保存的跳变列数 */
#define ZEBRA_ALIGN_TOL_COL            3    /* 相邻采样行跳变列允许错位(px) */
#define ZEBRA_ALIGN_MIN_RATIO         60    /* 跳变列对齐率下限(%) */

/* 十字状态机 */
typedef enum {
    CROSS_STATE_NONE = 0,
    CROSS_STATE_CANDIDATE,
    CROSS_STATE_DETECTED,
} cross_state_enum;

/* 十字静态变量 */
static cross_state_enum cross_state = CROSS_STATE_NONE; // 十字状态：无/候选/确认；只有确认状态才执行补线
static bool   cross_corners_valid;                      // 当前存的锚点是否可用，斜入可以只有有角侧上下两个点
static uint8  cross_confirm_count;                      // 连续有效候选帧数，达到2帧确认十字
static uint8  cross_missed_count;                       // 确认后连续漏检的帧数，最多4帧沿用旧锚点

/* sweep 写入的候选（每帧开头复位）*/
static bool  cross_cand_present;         // 本帧扫描是否出现双边丢失白区形成的正入候选；不等于已确认十字
static uint8 cross_near_row;             /* 下行号候选（近侧，行大）*/
static uint8 cross_far_row;              /* 上行号候选（远侧，行小）*/

/* cross_detect 算出的四拐点 */
static uint8 cross_up_l_col, cross_up_l_row;   /* 左上(远) */
static uint8 cross_up_r_col, cross_up_r_row;   /* 右上(远) */
static uint8 cross_dn_l_col, cross_dn_l_row;   /* 左下(近) */
static uint8 cross_dn_r_col, cross_dn_r_row;   /* 右下(近) */

/* 斜入状态（手动重构版） */
typedef enum {
    OBLIQUE_IDLE = 0,                    // 未锁定下拐点；单边丢线统计和下拐点搜索在这个分支完成
    OBLIQUE_UPPER_SEARCH = 2                 // 已找到下拐点并锁定有角侧，正在向远处寻找上拐点
} ObliqueState;

static ObliqueState oblique_state;       // 本帧扫描的斜入搜索阶段；不是跨帧十字确认状态
static uint8 oblique_lost_side;          /* 1=左丢，2=右丢 */
static uint8 oblique_real_side;          /* 1=左墙有角，2=右墙有角；触发后固定 */
static uint8 oblique_lost_run;           // 连续同侧丢边行数；达到7行才尝试斜入下拐点搜索，不是7张图
static uint8 oblique_dn_row, oblique_dn_col; // 已找到的有角侧下拐点(y,x)，靠近车身
static uint8 oblique_up_row, oblique_up_col; // 已确认的有角侧上拐点(y,x)，远离车身
static uint8 oblique_pending_up_row, oblique_pending_up_col; // 待确认上拐点，继续扫上方稳定段后才采用
static uint8 oblique_pending_confirm;    // 上拐点待确认进度；0表示当前没有候选
static bool  oblique_valid;              // 本帧斜入锚点已可交给cross_detect做跨帧确认
static bool  cross_oblique_used;         /* 本帧 cross_detect 是否实际采用了斜入 */
static uint8 cross_oblique_real_side;    /* 与保持拐点一起保存，独立于每帧清零的扫线状态 */

/* 斑马线静态变量 */
static bool   zebra_detected;            // 当前检测/短时保持的斑马线状态；不是每个白黑段都算终点
static uint8  zebra_missed_count;        // 斑马线漏检保持计数，也用于单行预确认容忍一次间隔
static uint8  zebra_confirm_count;      /* 连续有效帧计数（发车时清零） */
static bool   zebra_prev;               /* 上一帧检测状态 */
static uint8  oob_track_l = pho_w / 4;  /* OOB赛道窗口保持值：上一帧底边界失效时沿用 */
static uint8  oob_track_r = pho_w * 3 / 4; // 出界统计窗口右端；首次默认47..141，随后按近端真实双边更新
static uint8  zebra_track_l = pho_w / 4; /* 斑马线赛道窗口保持值：上一帧底边界失效时沿用（发车时清零） */
static uint8  zebra_track_r = pho_w * 3 / 4; // 斑马线统计窗口右端；缺边时保留最近一次可靠窗口

/* 斑马线诊断输出（随遥测上传）：main/control 层只读 */
volatile uint8_t oob_black_rows_dbg = 0; /* 本帧出界检查中底部窗口内全黑的行数（0~20） */

/* ================================================================
 * Otsu 二值化图像缓存
 *   0=黑（背景）, 255=白（赛道）
 * ================================================================ */
uint8 Image_Used[pho_h][pho_w];

/* ================================================================
 * 大津法 (Otsu) — 全屏 下采样4x4，安全钳 20~235
 * 移植自 loongson_learn，double → float
 * ================================================================ */
static int compute_otsu(void)
{
    unsigned long hist[256] = {0}; // 直方图，hist[g]表示抽样像素中灰度值g出现几次，初始256桶全部为0
    /* 全屏统计（行0~119），下采样4x4抗干扰 */
    for (int y = 0; y < pho_h; y += 4) // 每隔4行采样，降低阈值计算量；这里遍历整个高度，不只看前瞻窗口
        for (int x = 0; x < pho_w; x += 4) // 每隔4列采样，188×120图得到47×30个样本
            hist[vision_gray[y][x]]++; // 读原始灰度0..255，增加对应灰度桶计数

    unsigned long total = (unsigned long)((pho_h + 3) / 4) * ((pho_w + 3) / 4); // 实际抽样点总数；加3再除4是向上取整
    if (total == 0) return -1;

    unsigned long sum_all = 0; // 全部样本的灰度加权和，不是像素数量
    for (int i = 0; i < 256; i++) sum_all += (unsigned long)i * hist[i]; // 灰度值×出现次数求和，供前景均值计算使用

    unsigned long wB = 0, sumB = 0; // 当前阈值以下背景像素数wB及背景灰度和sumB
    float max_between = -1.0f; // 已找到的最大类间方差，初值-1保证首个有效候选被采纳
    int best_th = 0; // 对应最大方差的二值化阈值

    for (int t = 0; t < 256; t++) // 依次尝试每个灰度作为阈值t，比较黑白两类的可分程度
    {
        wB += hist[t]; // 把灰度t这一桶并入背景类，背景包含0..t
        if (wB == 0) continue; // 背景还没有像素，不能除以wB，换下一个阈值
        unsigned long wF = total - wB; // 余下t+1..255的样本归前景(亮赛道候选)
        if (wF == 0) break; // 前景已经空，后续更高阈值也不会有前景，结束遍历
        sumB += (unsigned long)t * hist[t]; // 同步累加背景类灰度和
        float mB = (float)sumB / wB; // 背景平均灰度，强制转float避免整数除法
        float mF = (float)(sum_all - sumB) / wF; // 前景平均灰度，用总和减背景和得到前景和
        float diff = mB - mF; // 两类平均灰度差；平方后只看分离程度，不看正负
        float between = (float)wB * (float)wF * diff * diff; // 类间方差的比例量，越大表示当前阈值把两类分得越开
        if (between > max_between) { max_between = between; best_th = t; } // 只在分离程度更好时更新阈值；相同值保留较早阈值
    }
    if (best_th < 20)  best_th = 20;
    if (best_th > 235) best_th = 235;
    return best_th; // 返回20..235以内阈值，由vis_deal再做跨帧平滑
}



/* ================================================================
 * 边界3点滑动平均：消除单行噪声，保持趋势
 * 只平滑三个坐标都非255的连续段；0是合法坐标，不能把0当丢线
 * ================================================================ */
static void border_smooth(border_line border)
{
    for (int y = 1; y < pho_h - 1; y++) // 第0/119行没有完整三点邻域，保持原值；其余行按远到近原地处理
    {
        uint8 l = border[y - 1]; // 变量l是上方相邻行的列号，此处不是赛道左边界
        uint8 m = border[y]; // 当前行的列号
        uint8 r = border[y + 1]; // 下方相邻行的列号，此处不是赛道右边界
        if (l == BORDER_INVALID || m == BORDER_INVALID || r == BORDER_INVALID) continue; // 有缺行就不平滑，也不跨越缺行连接两段线

        int sum = (int)l + (int)m + (int)r;
        uint8 v = (uint8)((sum + 1) / 3);  // +1 四舍五入

        // 变化超过 8px → 不信任此次平滑，保留原值
        int diff = (int)v - (int)m;
        if (diff < 0) diff = -diff;
        if (diff > VIS_BORDER_SMOOTH_JUMP) continue;  // 跳变太大，跳过不改

        border[y] = v; // 直接改原数组；下一个y会读到本次修改后的border[y]
    }
}

/* ================================================================
 * pho_center — 中线生成 + 偏差计算
 *
 * 1. border_smooth(g_vis.l_border/g_vis.r_border) — 平滑左右边界
 * 2. g_vis.center_line[]: 双边→中点, 单边→本帧逐行完整路宽外推, 都缺→白重心
 *    首次贴边即截断；上方保持INVALID，不拼接远处其他道路
 * 3. border_smooth(g_vis.center_line) — 平滑中线
 * 4. 固定asc_far±10，21行，权重1→5；截断缺行仅外推err，不画虚拟中线
 *
 * 调参: asc_far(固定前瞻中心行，默认60)，VIS_CENTER_*为缺行可靠性约束
 * ================================================================ */

/* ================================================================
 * longest_white_col — 最长白列（正确版）
 *
 * 中心区域按列采样，统计每列"从底行向上的连续白像素高度"，
 * 高度最大的列作为扫线种子。等价于参考工程 otsu_longest_white_col()。
 * 它是搜索起点的启发式，并不保证就是赛道几何中心；连接到赛道的背景白仍可能干扰。
 * ================================================================ */
#define LONGEST_COL_SAMPLE_STEP  5   // 最长白列采样步长：中部每5列测一根，兼顾速度与种子位置精度

static int longest_white_col(void)
{
    int best_col = pho_center_x; // 没有更好白柱时默认种子x=94
    int best_len = 0; // 已找到的最长底部连续白柱长度

    for (int x = pho_w / 4; x <= pho_w * 3 / 4; x += LONGEST_COL_SAMPLE_STEP) // 只在中部47..141列范围每5列测一根白柱，避免边角亮物体直接成为种子
    {
        int len = 0; // 当前采样列从底部连续为白的行数
        for (int y = pho_h - 1; y >= 0; y--) // 从119向0数白柱，遇到黑即结束这根柱，不累计不连续白点
        {
            if (Image_Used[y][x] == 255) len++;
            else break;                 /* 遇到黑像素，该列白段到顶 */
        }
        if (len > best_len) { best_len = len; best_col = x; } // 仅采用更长的白柱；长度相同时保留较早列
    }
    return best_col; // 这个列用于搜左右边界，不直接当作前瞻中线
}

/* ================================================================
 * 斜入辅助 — 边界哨兵读取
 * ================================================================ */
static int16 cross_edge_l(uint8 y)
{
    /* 斜入判据必须使用“本帧真实扫到的原始边界”。
     * 当前行在 sweep 中先保存 raw 再执行状态机，g_vis 此时还未写入当前行，
     * 若读 g_vis 会把当前行当成丢线，导致上拐点永远找不到。 */
    return (raw_l_exist[y] && raw_l_border[y] != BORDER_INVALID) // 本行有真实左墙且列号非255才算扫到
         ? (int16)raw_l_border[y]                                // 真实列号转int16，便于和负哨兵作差
         : CROSS_EDGE_LOST_L;                                    // 未扫到→-12，表示左墙"在很左/不存在"
}

static int16 cross_edge_r(uint8 y) // 右墙版本：同样只读本帧raw真实边界
{
    return (raw_r_exist[y] && raw_r_border[y] != BORDER_INVALID) // 条件同左墙：exist=1且列号有效
         ? (int16)raw_r_border[y]                                // 真实右墙列号
         : CROSS_EDGE_LOST_R;                                    // 未扫到→200，表示右墙"在很右/不存在"
}

/* ================================================================
 * 斜入辅助 — 判断单个候选行是否为有角侧下拐点
 * ================================================================ */
static bool cross_oblique_dn_at(uint8 real_side, uint8 y,
                                uint8 *dn_row, uint8 *dn_col)
{
    /* 越界保护：下方稳定需要 y+1..y+4，上方撕裂需要 y-1..y-3 */
    if (y + 4 >= pho_h || y < 3) return false;

    if (real_side == 1)  /* 左墙 LD */
    {
        int16 v0 = cross_edge_l(y); // 候选行(当前行)的左墙列号
        if (v0 == CROSS_EDGE_LOST_L) return false; // 候选行本身丢线，不可能是有角侧下拐点
        /* 上侧三行必须是已经真实扫到的边界，不能被未扫行的哨兵误判成“撕裂” */
        if (cross_edge_l(y-1) == CROSS_EDGE_LOST_L ||
            cross_edge_l(y-2) == CROSS_EDGE_LOST_L ||
            cross_edge_l(y-3) == CROSS_EDGE_LOST_L) return false; // 上方任一行丢线就无法确认撕裂
        if (CROSS_ABS(cross_edge_l(y+1) - cross_edge_l(y+2)) <= CROSS_STABLE && // 下方4行相邻差≤10；此稳定条件本身没有逐行排除丢线哨兵
            CROSS_ABS(cross_edge_l(y+2) - cross_edge_l(y+3)) <= CROSS_STABLE &&
            CROSS_ABS(cross_edge_l(y+3) - cross_edge_l(y+4)) <= CROSS_STABLE &&
            v0 - cross_edge_l(y-1) >= CROSS_JUMP1 && // 候选行比上方1行更靠右，越过拐点后墙开始错开
            v0 - cross_edge_l(y-2) >= CROSS_JUMP2 && // 上方第2行列号至少比候选小2px，即在候选左侧
            v0 - cross_edge_l(y-3) >= CROSS_JUMP2)   // 上方第3行也在候选左侧至少2px，不只比较相邻一行
        {
            *dn_row = y;         // 输出下拐点行（近侧，行号较大）
            *dn_col = (uint8)v0; // 输出该行有角侧列号
            return true;
        }
    }
    else  /* 右墙 RD */
    {
        int16 v0 = cross_edge_r(y); // 候选行的右墙列号
        if (v0 == CROSS_EDGE_LOST_R) return false; // 候选行丢线
        if (cross_edge_r(y-1) == CROSS_EDGE_LOST_R ||
            cross_edge_r(y-2) == CROSS_EDGE_LOST_R ||
            cross_edge_r(y-3) == CROSS_EDGE_LOST_R) return false; // 上方行必须都已真实扫到
        if (CROSS_ABS(cross_edge_r(y+1) - cross_edge_r(y+2)) <= CROSS_STABLE && // 下方4行相邻差≤10；与左墙稳定性比较对称
            CROSS_ABS(cross_edge_r(y+2) - cross_edge_r(y+3)) <= CROSS_STABLE &&
            CROSS_ABS(cross_edge_r(y+3) - cross_edge_r(y+4)) <= CROSS_STABLE &&
            v0 - cross_edge_r(y-1) <= -CROSS_JUMP1 && // 右墙方向相反：候选行比上方行更靠左
            v0 - cross_edge_r(y-2) <= -CROSS_JUMP2 && // 上方第2行列号至少比候选大2px，即在候选右侧
            v0 - cross_edge_r(y-3) <= -CROSS_JUMP2)   // 上方第3行也在候选右侧至少2px，和左墙差值符号相反
        {
            *dn_row = y;         // 输出下拐点行（近侧）
            *dn_col = (uint8)v0; // 输出该行右墙列号
            return true;
        }
    }
    return false; // 所有分支都不满足，当前行不是有角侧下拐点
}

/* ================================================================
 * 斜入辅助 — 找有角侧下拐点（从底部向上扫到 current_y）
 * ================================================================ */
static bool cross_find_oblique_dn(uint8 real_side, uint8 current_y,
                                  uint8 *dn_row, uint8 *dn_col)
{
    for (uint8 y = pho_h - 1; y >= current_y; y--) // 从近处(119)向远扫到current_y，首个命中就是最靠近车身的下拐点
    {
        if (cross_oblique_dn_at(real_side, y, dn_row, dn_col))
            return true;                            // 命中即写出dn_row/dn_col并结束
    }
    return false;                                   // 已扫描的119..current_y范围内，没有符合判据的下拐点
}

/* 上拐点撕裂触发：只使用当前行和已扫过的下方行 */
static bool cross_oblique_up_tear(uint8 real_side, uint8 y)
{
    /* 越界保护：需要 y+1..y+3 */
    if (y + 3 >= pho_h) return false;

    if (real_side == 1)  /* 左墙 LU：当前行 > 下一行 */
    {
        int16 v0 = cross_edge_l(y); // 当前行(较远)的左墙列号
        if (v0 == CROSS_EDGE_LOST_L) return false; // 当前行丢线，无法谈撕裂
        return (v0 - cross_edge_l(y+1) >= CROSS_JUMP1 && // 比下方1行已错开
                v0 - cross_edge_l(y+2) >= CROSS_JUMP2 && // 比下方2行错开更多
                v0 - cross_edge_l(y+3) >= CROSS_JUMP2);  // 上拐点撕裂候选成立，之后还需检查候选上方稳定段
    }
    else  /* 右墙 RU：当前行 < 下一行 */
    {
        int16 v0 = cross_edge_r(y); // 当前行(较远)的右墙列号
        if (v0 == CROSS_EDGE_LOST_R) return false; // 当前行丢线
        return (v0 - cross_edge_r(y+1) <= -CROSS_JUMP1 && // 右墙方向相反：当前行更靠左
                v0 - cross_edge_r(y+2) <= -CROSS_JUMP2 &&
                v0 - cross_edge_r(y+3) <= -CROSS_JUMP2);
    }
}

/* 上拐点候选确认：已向上扫过若干行后检查上方稳定 */
static bool cross_oblique_up_stable(uint8 real_side,
                                    uint8 up_row, uint8 current_y)
{
    /* 至少需要向上扫过 CROSS_OBLIQUE_CONFIRM_ROWS 行 */
    int urow = (int)up_row; // 上拐点候选行；后续行差明确按有符号int比较
    if (urow < (int)CROSS_OBLIQUE_CONFIRM_ROWS
        || (int)current_y > urow - (int)CROSS_OBLIQUE_CONFIRM_ROWS) return false; // 还未向上扫够3行证据，暂不确认

    if (real_side == 1)
    {
        if (CROSS_ABS(cross_edge_l((uint8)(urow-1)) - cross_edge_l((uint8)(urow-2))) > CROSS_STABLE)
            return false; // 候选正上方两行必须平稳，否则仍在过渡段
        if ((int)current_y <= urow - (int)CROSS_OBLIQUE_CONFIRM_ROWS &&
            CROSS_ABS(cross_edge_l((uint8)(urow-2)) - cross_edge_l((uint8)(urow-3))) > CROSS_STABLE)
            return false; // 已扫到候选上方第3行，继续比较第2/3行的列差
        if ((int)current_y <= urow - (int)CROSS_OBLIQUE_CONFIRM_ROWS - 1 &&
            CROSS_ABS(cross_edge_l((uint8)(urow-3)) - cross_edge_l((uint8)(urow-4))) > CROSS_STABLE)
            return false; // 再多扫1行则校验更上一对，逐级确认稳定段
        return true; // 左墙上方稳定性通过，扫描主流程还会检查丢线侧连续性
    }
    else
    {
        if (CROSS_ABS(cross_edge_r((uint8)(urow-1)) - cross_edge_r((uint8)(urow-2))) > CROSS_STABLE)
            return false; // 右墙版本：判据与左墙对称
        if ((int)current_y <= urow - (int)CROSS_OBLIQUE_CONFIRM_ROWS &&
            CROSS_ABS(cross_edge_r((uint8)(urow-2)) - cross_edge_r((uint8)(urow-3))) > CROSS_STABLE)
            return false; // 已扫到候选上方第3行，检查右墙第2/3行的列差
        if ((int)current_y <= urow - (int)CROSS_OBLIQUE_CONFIRM_ROWS - 1 &&
            CROSS_ABS(cross_edge_r((uint8)(urow-3)) - cross_edge_r((uint8)(urow-4))) > CROSS_STABLE)
            return false; // 逐级校验更上方一对
        return true; // 右墙上方稳定性通过，尚需联合丢线侧连续性条件
    }
}

/* 丢线侧连续性校验：斜入上下拐点之间，丢线侧不应出现大量真实边界。
 * 若出现过多 real 行，说明只是弯道单边丢失，不是斜入十字。 */
static bool cross_oblique_lost_side_continuous(uint8 real_side,
                                               uint8 dn_row, uint8 up_row)
{
    uint8 lost_side = (real_side == 1) ? 2 : 1; // 有角侧是左(1)则丢线侧是右(2)，反之亦然
    int   real_cnt  = 0;                        // 丢线侧却扫到真实边界的行数

    for (int y = (int)dn_row; y >= (int)up_row; y--) // 从下拐点(近)向上扫到上拐点(远)；转int便于递减
    {
        bool real = (lost_side == 1)       // 丢线侧是左墙时查左墙raw标志
                  ? (raw_l_exist[y] != 0)
                  : (raw_r_exist[y] != 0); // 丢线侧是右墙时查右墙raw标志
        if (real)
        {
            real_cnt++; // 丢线侧出现真实墙，斜入假设变弱
            if (real_cnt > (int)CROSS_OBLIQUE_LOST_SIDE_MAX_REAL_ROWS)
                return false; // 真实行超过10行，判为普通弯道单边丢失而非斜入十字
        }
    }
    return true; // 丢线侧基本连续无墙，符合斜入十字
}

/* ================================================================
 * sweep_boundaries — 逐行扫描（边界跟随版，对齐参考工程 Track_sweep.c）
 *
 * 1. longest_white_col() 找种子列（赛道中心）
 * 2. 底行：从种子向左右扩白(白→黑黑)得底行边界
 * 3. 逐行向上：起点=上一行边界向内偏移 SWEEP_OFFSET，沿边界法向
 *    走到"连续两个黑像素"才停（白→黑黑确认，抗单像素噪声）
 * 4. 单边丢: last_valid_width 外推；双边丢: 黑区=真丢线 / 白区=十字横道
 *
 * 直接填充 g_vis.l_border[] / g_vis.r_border[] / exist[]。
 * 无效标记统一用 BORDER_INVALID(255)，与 pho_center()/border_smooth() 一致。
 * 返回: 1=真丢线（上方全标无效） 0=正常
 * ================================================================ */
/* 起点落在黑区时，按summer稳定版的“局部→扩大范围”重新寻找真实跳变。
 * 固定seed也可能在黑区，因此最后允许全行恢复；选离预测墙最近的跳变。
 * 只有黑黑白白/白白黑黑才算墙，画面边缘不伪装成真实边界。 */
static int sweep_recover_edge(int y, int predicted, bool left)
{
    int range = last_valid_width / 4; // 局部搜索半径随已知路宽变化
    if (range < 3) range = 3;
    if (range > 25) range = 25;
    for (int pass = 0; pass < 3; pass++)
    {
        int radius = pass == 0 ? range : (pass == 1 ? range * 3 : pho_w);
        int lo = predicted - radius, hi = predicted + radius;
        if (lo < 2) lo = 2;
        if (hi > pho_w_max - 2) hi = pho_w_max - 2;
        int best = BORDER_INVALID, best_distance = pho_w + 1;
        for (int x = lo; x <= hi; x++)
        {
            bool transition = left
                ? (Image_Used[y][x-2] == 0 && Image_Used[y][x-1] == 0 &&
                   Image_Used[y][x] == 255 && Image_Used[y][x+1] == 255)
                : (Image_Used[y][x-1] == 255 && Image_Used[y][x] == 255 &&
                   Image_Used[y][x+1] == 0 && Image_Used[y][x+2] == 0);
            int distance = CROSS_ABS(x - predicted);
            if (transition && distance < best_distance) { best = x; best_distance = distance; }
        }
        if (best != BORDER_INVALID) return best; // 找到真实墙才结束，不返回白重心作为边界
    }
    return BORDER_INVALID;
}

/* 单边缺失：优先用同一行学习的路宽；可见白段是完整路宽的下界。
 * 例如左墙x=70、白色一直延伸到187，完整宽度不可能只有20px。
 * 必须确认该墙与画面边缘属于同一白段，不能把远处另一白块算进去。 */
static int sweep_single_width(int y, int edge, bool left)
{
    int width = learned_row_width[y] ? learned_row_width[y] : last_valid_width;
    int end = left ? pho_w_max : 0, step = left ? 1 : -1;
    bool reaches_edge = Image_Used[y][end] == 255;
    int black_run = 0;
    for (int x = edge; reaches_edge && x != end; x += step)
    {
        black_run = Image_Used[y][x] == 0 ? black_run + 1 : 0;
        if (black_run >= 2) reaches_edge = false; // 两黑隔开，说明不能以画面边缘作为可见宽度下界
    }
    int visible_width = CROSS_ABS(end - edge);
    if (reaches_edge && width < visible_width) width = visible_width;
    return width; // 不反写真实宽度缓存；单边画面只能提供下界，不能完成路宽标定
}

static int sweep_boundaries(void)
{
    cross_cand_present = false;          /* 每帧复位候选（必须在任何 return 之前）*/
    bool frame_width_saved = false; // 本帧是否已保存最靠近车身的真实路宽；保存一次后不再被远端窄路宽覆盖
    memset(sweep_row_width, 0, sizeof(sweep_row_width)); // 清上帧逐行宽度，0表示本帧还没写该行

    oblique_state           = OBLIQUE_IDLE;
    oblique_valid           = false;
    oblique_lost_run        = 0;
    oblique_lost_side       = 0;
    oblique_real_side       = 0;
    oblique_pending_confirm = 0;
    oblique_dn_row          = 0;
    oblique_dn_col          = 0;
    oblique_up_row          = 0;
    oblique_up_col          = 0;
    oblique_pending_up_row  = 0;
    oblique_pending_up_col  = 0;
    /* 注意：cross_oblique_used 不在每帧开头清空，
     * 以便 DETECTED 漏检保持期间沿用上一帧的补线模式 */

    /* 原始判据数组先全部清空，sweep 只填真正扫到的行 */
    for (int y = 0; y < pho_h; y++)
    {
        raw_l_border[y] = BORDER_INVALID;
        raw_r_border[y] = BORDER_INVALID;
        raw_l_exist[y]  = 0;
        raw_r_exist[y]  = 0;
    }

    /* ---- 最长白列作种子（替代"底行最长白段"）---- */
    int y_bot = pho_h - 1; // 底行119，从最近的道路开始追踪
    int seed  = longest_white_col(); // 找底部连续白柱最长的一列作为本帧搜索种子

    /* 中间采样列可能全部落在黑区；底行回到summer稳定版的最长白段。
     * 只在seed不白时兜底，避免把黑种子向两侧扩张成虚构路宽。 */
    if (Image_Used[y_bot][seed] != 255)
    {
        int run_start = -1, best_start = -1, best_length = 0;
        for (int col = 0; col <= pho_w; col++)
        {
            if (col < pho_w && Image_Used[y_bot][col] == 255) { if (run_start < 0) run_start = col; }
            else if (run_start >= 0)
            {
                if (col - run_start > best_length) { best_start = run_start; best_length = col - run_start; }
                run_start = -1;
            }
        }
        if (best_length < 3) return 1;
        seed = best_start + best_length / 2;
    }

    /* ---- 底行：从种子列向左右扩白(白→黑黑) ---- */
    int best_s = seed; // 从种子向左扩张得到的底行左端列
    while (best_s > 2
        && !(Image_Used[y_bot][best_s - 1] == 0 && Image_Used[y_bot][best_s - 2] == 0))
        best_s--;
    int best_e = seed; // 从种子向右扩张得到的底行右端列
    while (best_e < pho_w_max - 2
        && !(Image_Used[y_bot][best_e + 1] == 0 && Image_Used[y_bot][best_e + 2] == 0))
        best_e++;

    if (best_e - best_s < 3) return 1;          /* 底行无赛道 */

    int prev_l = best_s; // 近一行左边界位置，作为当前行搜索起点的依据
    int prev_r = best_e; // 近一行右边界位置
    int l_exist = (best_s > 2);                 /* 贴边(≤2)视为伪边界 */
    int r_exist = (best_e < pho_w_max - 2); // 右端不贴画面边缘才算真实右墙，贴边表示墙可能已出视野

    /* summer的近→远路宽传播：只有真实双边才能更新，截断的白段不是完整路宽。
     * 底行单边时沿用上一帧近端真实宽度，不把远端缓存或可见白段当完整宽度。 */
    last_valid_width = (int16)last_real_width; // 先以最近帧的近端真实宽度兜底，底行双边真实时再覆盖
    if (l_exist && r_exist) // 底行左右墙都真实可见，才能用左右列差标定完整路宽
    {
        last_valid_width = best_e - best_s;
        if (last_valid_width < 20) last_valid_width = 20;
        if (last_valid_width > 160) last_valid_width = 160;
        last_real_width = last_valid_width;
        frame_width_saved = true;
        if (best_e - best_s >= 20 && best_e - best_s <= 160) learned_row_width[y_bot] = (uint8)(best_e - best_s);
    }
    else if (l_exist || r_exist) last_valid_width = (int16)sweep_single_width(y_bot, l_exist ? best_s : best_e, l_exist != 0);
    sweep_row_width[y_bot] = (uint8)last_valid_width; // 保存底行使用的完整宽度，供后面pho_center单边求半宽

    g_vis.l_border[y_bot] = (uint8)best_s;
    g_vis.r_border[y_bot] = (uint8)best_e;
    g_vis.l_border_exist[y_bot] = (uint8)l_exist;
    g_vis.r_border_exist[y_bot] = (uint8)r_exist;

    /* 保存底行原始边界（未外推） */
    raw_l_border[y_bot] = l_exist ? (uint8)best_s : BORDER_INVALID;
    raw_r_border[y_bot] = r_exist ? (uint8)best_e : BORDER_INVALID;
    raw_l_exist[y_bot]  = (uint8)l_exist;
    raw_r_exist[y_bot]  = (uint8)r_exist;

    /* ---- 向上逐行扫描（边界跟随）---- */
    int consec_lost = 0; // 连续双边丢失且中部为黑的行数，达到5行停止继续向上搜
    uint8 white_consec = 0;              /* 连续白区行计数 */
    uint8 near_row = 0;                  /* 近侧候选行 */
    bool  force_seed_after_cross = false; /* 白区候选后先从seed重找，但不能抹掉已找到的单边 */
    bool oblique_dn_initial_searched = false; /* 斜入下拐点是否做过一次全量搜索 */

    for (int y = y_bot - 1; y >= 5; y--) // 扫描118..5行，y越小越远；顶上0..4行不做普通边界跟随
    {
        int new_l = BORDER_INVALID, new_r = BORDER_INVALID; // 本行新边界坐标；找不到保持255，不能沿用上帧坐标
        int l_real = 0, r_real = 0; // 本行是否实际找到墙，与后续搜索外推坐标分开记录
        int x; // 当前横向搜索列，先找左墙，再复用来找右墙

        /* 正常行保持原来的向内起点+扩白；起点不白时搜索真实跳变，不能退回黑seed。 */
        x = force_seed_after_cross ? seed : prev_l + SWEEP_OFFSET;
        if (x < 2) x = 2;
        if (x > pho_w_max - 2) x = pho_w_max - 2;
        if (Image_Used[y][x] == 255)
        {
            while (x > 2 && !(Image_Used[y][x-1] == 0 && Image_Used[y][x-2] == 0)) x--;
            if (x > 2 && Image_Used[y][x] == 255) new_l = x;
        }
        else new_l = sweep_recover_edge(y, prev_l, true);
        l_real = new_l != BORDER_INVALID;

        x = force_seed_after_cross ? seed : prev_r - SWEEP_OFFSET;
        if (x < 2) x = 2;
        if (x > pho_w_max - 2) x = pho_w_max - 2;
        if (Image_Used[y][x] == 255)
        {
            while (x < pho_w_max - 2 && !(Image_Used[y][x+1] == 0 && Image_Used[y][x+2] == 0)) x++;
            if (x < pho_w_max - 2 && Image_Used[y][x] == 255) new_r = x;
        }
        else new_r = sweep_recover_edge(y, prev_r, false);
        r_real = new_r != BORDER_INVALID;
        int row_width = last_valid_width; // 与全局近端参考分开；单边下界不能污染下一行真实标定

        /* 保存“真正扫到的原始边界”，供串口判据使用（未被外推/标空污染） */
        raw_l_border[y] = l_real ? (uint8)new_l : BORDER_INVALID;
        raw_r_border[y] = r_real ? (uint8)new_r : BORDER_INVALID;
        raw_l_exist[y]  = (uint8)l_real;
        raw_r_exist[y]  = (uint8)r_real;

        /* --- 白区出口：本行有效(至少一边找到)、上一行仍在白区 → 提交候选 --- */
        if (white_consec > 0 && (l_real || r_real)) // 近处曾连续缺双边但有白，本行恢复至少一墙，检查是否到横道远端
        {
            if (white_consec >= CROSS_WHITE_ZONE_MIN
                && (uint8)y >= CROSS_WHITE_ZONE_FAR_ROW_MIN)
            {
                cross_cand_present = true;
                cross_near_row = near_row;
                cross_far_row  = (uint8)y;
                force_seed_after_cross = true;   /* 横道候选后优先seed重找，恢复的真实单边仍保留 */
            }
            white_consec = 0;
        }

        /* ===== 耦合扫线：斜入判断直接复用本行刚保存的raw边界 =====
         * 顺序：搜本行左右墙→保存raw→累计丢边/找角点→决定下一行prev_l/prev_r。
         * 斜入进入UPPER_SEARCH时还会改变后续搜索起点，因此不是扫完图后的独立检查。
         * 首次回找下拐点只遍历已经保存的边界数组，不重新扫描整幅图的像素。 */

        /* 正入候选出现时保留斜入作为兜底；cross_detect 仍正入优先，
         * 正入失败后才使用斜入。 */
        {
            /* 只有还没找到下拐点时才需要计数 */
            if (oblique_state == OBLIQUE_IDLE) // 没有锁定下拐点时才累计单侧丢边，已锁定时不随意换侧
            {
                /* 单边丢线计数（oblique_lost_side: 1=左丢，2=右丢） */
                if (l_real && !r_real) {
                    /* 右丢 */
                    if (oblique_lost_side == 2) oblique_lost_run++;
                    else { oblique_lost_side = 2; oblique_lost_run = 1; }
                }
                else if (!l_real && r_real) {
                    /* 左丢 */
                    if (oblique_lost_side == 1) oblique_lost_run++;
                    else { oblique_lost_side = 1; oblique_lost_run = 1; }
                }
                else {
                    oblique_lost_side = 0;
                    oblique_lost_run  = 0;
                    oblique_dn_initial_searched = false;
                }

                /* 触发下拐点搜索：
                 * 第一次做一次全量搜索（覆盖已经扫过的底部区域），
                 * 之后每行只检查新满足数据条件的一行（y+3），不再从119重扫。 */
                if (!oblique_valid
                    && oblique_lost_run >= CROSS_OBLIQUE_LOST_MIN)
                {
                    uint8 real_side = (oblique_lost_side == 1) ? 2 : 1; // 左丢意味着右墙是真实侧；右丢意味着左墙是真实侧
                    bool found = false;

                    if (!oblique_dn_initial_searched)
                    {
                        oblique_dn_initial_searched = true;
                        found = cross_find_oblique_dn(
                                    real_side, (uint8)y,
                                    &oblique_dn_row, &oblique_dn_col);
                    }
                    else
                    {
                        uint8 cand = (uint8)((int)y + 3); // 当前刚扫到y，候选y+3所需的上方三行至此都已扫描
                        found = cross_oblique_dn_at(
                                    real_side, cand,
                                    &oblique_dn_row, &oblique_dn_col);
                    }

                    if (found)
                    {
                        oblique_real_side = real_side;   /* 触发后固定，防止中途换边 */
                        oblique_state = OBLIQUE_UPPER_SEARCH;
                        oblique_pending_confirm = 0;
                        oblique_valid = false;

                        /* 不再逐行标空；只记录下拐点，后续正常扫 + 每行结束按有角侧固定列重置 */
                    }
                }
            }

            /* 上拐点搜索 */
            if (oblique_state == OBLIQUE_UPPER_SEARCH)
            {
                uint8 real_side = oblique_real_side;

                if (oblique_pending_confirm == 0 // 没有待确认上拐点时，才尝试本行撕裂作为新候选；条件下一行继续
                    && cross_oblique_up_tear(real_side, (uint8)y))
                {
                    oblique_pending_up_row = (uint8)y;
                    oblique_pending_up_col = (real_side == 1)
                                           ? (uint8)cross_edge_l((uint8)y)
                                           : (uint8)cross_edge_r((uint8)y);
                    oblique_pending_confirm = 1;
                }
                else if (oblique_pending_confirm > 0)
                {
                    if (cross_oblique_up_stable(real_side,
                                                oblique_pending_up_row,
                                                (uint8)y))
                    {
                        if (cross_oblique_lost_side_continuous(
                                real_side,
                                oblique_dn_row,
                                oblique_pending_up_row))
                        {
                            oblique_up_row = oblique_pending_up_row;
                            oblique_up_col = oblique_pending_up_col;
                            oblique_valid  = true;
                            oblique_state  = OBLIQUE_IDLE;

                            /* 这里只提交候选，不擦掉真实墙；cross_detect跨帧确认后由cross_repair覆盖。
                             * 未确认的第一帧仍可用真实单边循迹，不能先造出一整段丢线。 */
                            /* 当前行在上拐点上方，不标空；交给正常处理 */
                        }
                        else
                        {
                            /* 丢线侧在上下拐点之间出现过多真实边界
                             * → 判定不是斜入，丢弃本次斜入候选 */
                            oblique_state           = OBLIQUE_IDLE;
                            oblique_valid           = false;
                            oblique_pending_confirm = 0;
                            oblique_dn_row          = 0;
                            oblique_dn_col          = 0;
                            oblique_up_row          = 0;
                            oblique_up_col          = 0;
                            oblique_pending_up_row  = 0;
                            oblique_pending_up_col  = 0;
                        }
                    }
                    else if (oblique_pending_confirm >= CROSS_OBLIQUE_CONFIRM_ROWS)
                    {
                        oblique_pending_confirm = 0;
                    }
                    else
                    {
                        oblique_pending_confirm++;
                    }
                }

                /* 扫到顶仍没有真实上拐点：证据不足，取消本帧候选。
                 * 不再虚构(187,0)/(0,0)，也不清空普通单边弯道的真实边界。 */
                if (oblique_state == OBLIQUE_UPPER_SEARCH && y == 5)
                {
                    oblique_valid = false;
                    oblique_state = OBLIQUE_IDLE;
                    oblique_pending_confirm = 0;
                }
            }
        }

        /* --- 处理边界状态 --- */
        if (l_real && r_real && new_l < new_r) // 双墙实际可见且左右顺序合理，才能更新路宽
        {
            /* 双边真 → 更新路宽，并解除“十字后强制 seed” */
            force_seed_after_cross = false;
            last_valid_width = new_r - new_l;
            if (last_valid_width < 20)  last_valid_width = 20;
            if (last_valid_width > 160) last_valid_width = 160;
            if (!frame_width_saved) // 本帧首次遇到真实双墙才保存跨帧近端路宽
            {
                last_real_width = last_valid_width; // 本帧最靠近车身的真实双边，仅保存一次
                frame_width_saved = true;
            }
            row_width = last_valid_width;
            if (!cross_cand_present && !oblique_valid && oblique_state == OBLIQUE_IDLE
                && new_r - new_l >= 20 && new_r - new_l <= 160)
                learned_row_width[y] = (uint8)(new_r - new_l); // 只用实际双墙标定对应行，不学习十字候选宽白带
            consec_lost = 0;
        }
        else if (l_real && !r_real) // 单左墙：用真实左墙加完整路宽生成搜索假右边，不把假边标成真实
        {
            /* 白区候选尚未确认时，也保留本行真实墙；确认后的补线另行覆盖。 */
            row_width = sweep_single_width(y, new_l, true); // 同行缓存+可见白段下界，避免黄线贴左墙
            new_r = new_l + row_width;
            if (new_r > pho_w_max) new_r = pho_w_max;
            consec_lost = 0;
        }
        else if (!l_real && r_real) // 单右墙：用真实右墙减完整路宽生成搜索假左边
        {
            /* 白区候选尚未确认时，也保留本行真实墙；确认后的补线另行覆盖。 */
            row_width = sweep_single_width(y, new_r, false); // 右墙镜像；不使用被远行缩小的半宽
            new_l = new_r - row_width;
            if (new_l < 0) new_l = 0;
            consec_lost = 0;
        }
        else
        {
            /* 双边都丢：区分黑区(真丢线) / 白区(十字横道) */
            int is_black = 1; // 先假设中部为黑，找到任何采样白点就改为非黑
            for (int cx = pho_w / 4; cx <= pho_w * 3 / 4; cx += LONGEST_COL_SAMPLE_STEP) // 只抽查中部列区分全黑丢线与白横道，不是整行白像素统计
                if (Image_Used[y][cx] == 255) { is_black = 0; break; }

            if (is_black)
            {
                white_consec = 0;      /* 黑区打断白区连续性 */
                if (++consec_lost >= 5) // 先累计再判断；连续5个黑缺行才截断边界扫描
                {
                    /* 连续5行全黑 → 停止，上方全标无效 */
                    for (int ry = y; ry >= 0; ry--)
                    {
                        g_vis.l_border[ry] = BORDER_INVALID;
                        g_vis.r_border[ry] = BORDER_INVALID;
                        g_vis.l_border_exist[ry] = 0;
                        g_vis.r_border_exist[ry] = 0;
                    }
                    return 1;
                }
            }
            else
            {
                consec_lost = 0;       /* 白区：十字横道，不判丢线 */
                if (white_consec == 0)
                {
                    /* 刚进入白区：近侧候选 = 上一行(y+1)。
                     * 不再提前校验该行边界是否有效：near_row 只是下拐点
                     * 的起始行，后面 cross_detect() 会按真实边界取舍。 */
                    near_row = (uint8)(y + 1);
                }
                white_consec++; // 连续双边缺失但中部仍有白的行数，用于正入候选
                /* 连续3行双丢+白区：立即判为正入，不需要等单边重新出现。
                 * 斜入保留作正入失败后的兜底。
                 * 白区过高/伸到太远（y < 31）时取消，避免弯道/出界误判。 */
                if (cross_cand_present
                    && (uint8)y < CROSS_WHITE_ZONE_FAR_ROW_MIN)
                {
                    cross_cand_present = false;
                    force_seed_after_cross = false;
                }
                else if (!cross_cand_present
                    && white_consec >= CROSS_WHITE_ZONE_MIN
                    && (uint8)y >= CROSS_WHITE_ZONE_FAR_ROW_MIN)
                {
                    cross_cand_present = true;
                    cross_near_row     = near_row;
                    cross_far_row      = (uint8)y;
                    force_seed_after_cross = true;
                    /* 注意：这里不再清空斜入，保留斜入作为正入失败后的兜底 */
                }
            }

            g_vis.l_border[y] = BORDER_INVALID;
            g_vis.r_border[y] = BORDER_INVALID;
            g_vis.l_border_exist[y] = 0;
            g_vis.r_border_exist[y] = 0;
            if (oblique_state == OBLIQUE_UPPER_SEARCH && !cross_cand_present)
            {
                /* 斜入上拐点搜索期间：不回到 seed，按有角侧固定重扫列；
                 * 左墙有角(右丢)→168，右墙有角(左丢)→20 */
                prev_l = CROSS_OBLIQUE_RESET_COL(oblique_real_side);
                prev_r = CROSS_OBLIQUE_RESET_COL(oblique_real_side);
            }
            else
            {
                prev_l = seed;             /* 下一行搜索起点重置回种子列 */
                prev_r = seed;
            }
            continue;
        }

        /* 写入本行 */
        sweep_row_width[y] = (uint8)row_width; // 保存当前行用过的未裁剪完整宽度；即使假边已夹0/187，半宽仍完整
        g_vis.l_border[y] = (uint8)new_l;
        g_vis.r_border[y] = (uint8)new_r;
        g_vis.l_border_exist[y] = (uint8)l_real; // 假左边有坐标但exist=0，十字补线还未在这里执行
        g_vis.r_border_exist[y] = (uint8)r_real; // 真实/假右边的来源同样保留

        /* 斜入上拐点搜索期间：下一行强制从有角侧对应的固定列重扫。
         * 左墙有角(右丢)→168，右墙有角(左丢)→20，不再回 seed。
         * 正入候选存在时仍以正入的 seed 逻辑优先，斜入仅作兜底。 */
        if (oblique_state == OBLIQUE_UPPER_SEARCH && !cross_cand_present)
        {
            prev_l = CROSS_OBLIQUE_RESET_COL(oblique_real_side);
            prev_r = CROSS_OBLIQUE_RESET_COL(oblique_real_side);
        }
        else
        {
            prev_l = new_l;
            prev_r = new_r;
        }
    }

    return 0; // 扫到正常上限后结束；返回值不是VisionEvent事件码
}



/* 贴边点附近已存在的近端中线估计向远处的趋势。
 * 不跨INVALID行，不允许趋势反向把预测拉回画面；只有一个点则保持贴边误差。 */
static float center_outward_slope(int row, int col)
{
    int ref_row = row; // 先把贴边行本身当参考；找不到近端点时保持斜率0
    for (int y = row + 1; y < pho_h && y <= row + VIS_CENTER_SLOPE_ROWS; y++) // 向下(靠近车身)最多找5行已生成的中心点，而不是去读尚未生成的远端中线
    {
        if (g_vis.center_line[y] == BORDER_INVALID) break; // 近端趋势只能用连续有效段，遇缺行停止，不能跨段拟合
        ref_row = y; // 记录这段中最远离贴边行的近端参考点
    }
    if (ref_row == row) return 0.0f; // 只有贴边点，没有其他参考点，无法算趋势；外推保持同方向偏差
    float slope = ((float)col - (float)g_vis.center_line[ref_row]) / (float)(ref_row - row); // 向远处每跨一行中心变化多少px；例如左出界时通常为负
    if ((col <= VIS_CENTER_EDGE_MARGIN && slope > 0.0f) ||
        (col >= pho_w_max - VIS_CENTER_EDGE_MARGIN && slope < 0.0f)) slope = 0.0f;
    if (slope > VIS_CENTER_SLOPE_MAX) slope = VIS_CENTER_SLOPE_MAX;
    if (slope < -VIS_CENTER_SLOPE_MAX) slope = -VIS_CENTER_SLOPE_MAX;
    return slope; // 用于估计缺行err，不把估计列写回center_line，也不是车辆yaw角
}

/* 本行全部滤波后白像素的重心，与summer的双边缺失兜底口径一致。
 * 至少3点才有效；它是可见白区中心，不是已经标定的物理赛道中心。 */
static bool vision_white_center(int y, int *center)
{
    int sum = 0, count = 0;
    for (int x = 0; x < pho_w; x++)
        if (Image_Used[y][x] == 255) { sum += x; count++; }
    if (count < 3) return false;
    *center = (sum + count / 2) / count;
    return true;
}

static void pho_center(void)
{
    border_smooth(g_vis.l_border); // 先去左右墙单行抖动；十字修复已在前一步完成
    border_smooth(g_vis.r_border);
    memset(g_vis.center_line, BORDER_INVALID, sizeof(g_vis.center_line)); // 每帧全部中线先置255，后续提前break时上方自然保持无效
    center_cut_row = -1; // 贴边位置是本帧状态，不能沿用上一帧截断结果
    center_cut_col = 0;
    center_cut_slope = 0.0f;

    for (int y = pho_h - 1; y >= 0; y--) // 从近到远生成中线，保证遇贴边时已得到近端趋势点
    {
        int l = (int)g_vis.l_border[y], r = (int)g_vis.r_border[y], c = 0; // 本行左右列与待计算中心；c用int以容纳出画面的负数或大于187的值
        bool have_center = true; // 用独立标志判有没有中心，避免int外推值255被误当uint8无效标记
        bool l_valid = l != BORDER_INVALID && (g_vis.l_border_exist[y] || (vision_repair_flags[y] & 1u)); // 左坐标存在且来源是真墙/十字补线才参与；普通搜索假边被排除
        bool r_valid = r != BORDER_INVALID && (g_vis.r_border_exist[y] || (vision_repair_flags[y] & 2u)); // &2检查bit1右补线；不能写成检查bit0
        /* 与summer同源的逐行路宽：扫描时保存，避免pho_center再次使用上一帧远端宽度。
         * 普通搜索假边仍不算真边；先推完整半宽，再裁剪，不能对裁剪假边求中点。 */
        int width = sweep_row_width[y] ? sweep_row_width[y] : last_real_width; // 优先本帧扫该行用的完整宽度；该行未保存宽度才用跨帧近端缓存
        if (l_valid && r_valid) c = (l + r) / 2; // 双边真实/补线仍按中点，不能用全行重心覆盖正常双边
        else
        {
            int white_center = 0;
            bool white_valid = vision_white_center(y, &white_center);
            if (l_valid)
            {
                c = l + width / 2; // 先按完整路宽估计向右外推，保留未裁剪坐标
                if (white_valid && c < white_center) c = white_center; // 外推尚未到可见白中心，按用户要求取白中心
            }
            else if (r_valid)
            {
                c = r - width / 2;
                if (white_valid && c > white_center) c = white_center; // 右单边镜像：更远是更靠左，不能也写max
            }
            else if (white_valid) c = white_center; // 两墙都缺：summer白像素重心兜底
            else have_center = false;
        }
        if (!have_center) continue; // 只跳过本行，上方仍可继续生成；不是已确认的贴边截断
        /* 必须在平滑、夹到画面内之前记录原始中心。int值255可以是外推坐标，
         * 不可与uint8的INVALID标记混淆。截断只影响中线，十字/边界已在前面处理完成。 */
        if (c <= VIS_CENTER_EDGE_MARGIN || c >= pho_w_max - VIS_CENTER_EDGE_MARGIN) // 中心到左右画面边缘2px以内，或已超出画面，都进入贴边处理
        {
            /* 无真实/补线边界的少量边缘亮点不能成为整窗外推锚点。 */
            if (!l_valid && !r_valid) continue; // 孤立边缘白重心没有墙支撑，不拿它当整窗外推锚点
            center_cut_row = (int16)y;
            center_cut_col = (int16)c; // 保存未裁剪中心；例如真实推到220，不能只记显示列187
            center_cut_slope = center_outward_slope(y, c); // 从已经生成的近端5行中心估计离开画面的趋势
            g_vis.center_line[y] = (uint8)(c <= VIS_CENTER_EDGE_MARGIN ? 0 : pho_w_max); // 本行黄线显示在左/右边缘，之后更远行保持255
            break; // 上方全部保持255，不再重新接到远处另一段弯道
        }
        g_vis.center_line[y] = (uint8)c; // 未贴边且有可靠中心才写入本行正常黄线
    }
    border_smooth(g_vis.center_line); // 最后平滑中线；255缺行使平滑无法跨过贴边截断继续画线

    /* 固定前瞻，不引入自适应前瞻。闭区间共21行；近到远权重1→5。 */
    int y_center = (int)asc_far; // 在线前瞻参数表示窗口中心行，而不是最远取值行
    if (y_center < 20) y_center = 20;
    if (y_center > 100) y_center = 100;
    int y_near = y_center + 10, y_far = y_center - 10; // 默认60对应近70/远50，包含两端共21行
    asc_range_dbg = (int16)y_center;
    vision_sample_far = (uint8)y_far;
    vision_sample_near = (uint8)y_near;
    float total_dev = 0.0f, total_w = 0.0f; // 偏差乘权重的总和/有效权重总和，最后相除求平均
    float w = 1.0f, w_step = 4.0f / (float)(y_near - y_far); // 近行权重1，向远每行加0.2，到最远行为5
    int valid_cnt = 0; // 实际参与err的行数，包含贴边缺行估计，不只是画出来的黄线点数
    float err_limit = (float)pho_center_x * VIS_CENTER_ERR_SCALE_MAX; // 94×2.5=235px，是视觉误差约束，不是电机占空比限幅
    for (int y = y_near; y >= y_far; y--) // 遍历前瞻窗口，从近到远按各行应有权重取值
    {
        int c = g_vis.center_line[y];
        float dev = 0.0f; // 本行中心相对x=94的偏差，>0目标在右，<0目标在左
        bool use = false; // 本行是否实际贡献误差；无效行不往总权重中灌0
        if (c != BORDER_INVALID &&
            ((c >= 3 && c <= pho_w_max - 3) || g_vis.l_border_exist[y] ||
             g_vis.r_border_exist[y] || vision_repair_flags[y]))
        {
            dev = (float)c - (float)pho_center_x; // 已有中心直接算偏差，不按车辆速度/轮距换算
            use = true;
        }
        else if (center_cut_row >= 0 && y < center_cut_row) // 只对本帧已确认贴边以上的缺行外推，不外推任意黑缺行
        {
            /* 截断缺行：按贴边点+最近5行趋势+距贴边点剩余行数估计偏差。
             * 不写center_line，也不读上方真假边界；整窗口越过贴边点时仍有同向err。
             * 上限沿用summer的2.5倍半屏宽，避免极少点产生无限纠偏。 */
            dev = (float)center_cut_col + center_cut_slope * (float)(center_cut_row - y)
                  - (float)pho_center_x;
            use = true;
        }
        if (use) // 真实中心或允许的缺行估计才计入总和
        {
            if (dev > err_limit) dev = err_limit;
            if (dev < -err_limit) dev = -err_limit;
            total_dev += dev * w; // 偏差加权，远端同样偏差的影响更大
            total_w += w; // 分母只加参与取值行的权重
            valid_cnt++; // 统计参与行数，供非贴边零星缺行覆盖率补偿
        }
        w += w_step; // 每前进到更远一行都加权，缺行也占据原本的权重位置
    }
    if (total_w > 0.0f) // 分母不为0才能更新err；整窗无信息时保留上一帧err
    {
        float err = total_dev / total_w; // 得到当前窗口的加权像素偏差
        /* 非贴边的零星缺行继续summer覆盖率补偿。贴边缺行已逐行估计，不重复放大。 */
        int theo_rows = y_near - y_far + 1; // 含首尾应有21行，+1避免原来按20行统计
        if (center_cut_row < 0 && valid_cnt < theo_rows) // 没发生贴边但窗口有零星缺行，保留summer覆盖率补偿；贴边估计不重复放大
        {
            float scale = (float)theo_rows / (float)valid_cnt; // 可用行越少，补偿越大；下方再限制最大2.5倍
            if (scale > VIS_CENTER_ERR_SCALE_MAX) scale = VIS_CENTER_ERR_SCALE_MAX;
            err *= scale;
        }
        if (err > err_limit) err = err_limit;
        if (err < -err_limit) err = -err_limit;
        g_vis.err = err; // 发布本帧视觉偏差，control_update随后读取它生成omega/vy
    }
    // 完全无中心且没有本帧贴边锚点：保留上帧err，出界停车仍由独立事件链判断。
}

/* ---- 旧 fallback_scan 已删除，sweep_boundaries() 内置兜底处理 ---- */

/* ================================================================
 * 十字识别 — 辅助函数
 * ================================================================ */
static uint8 cross_limit_u8(int32 value, uint8 lower, uint8 upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return (uint8)value;
}

/* 上拐点行基本有效：左右边界都有值且左 < 右（上拐点不查 exist）*/
static bool cross_anchor_row_valid(uint8 y)
{
    uint8 l = g_vis.l_border[y];
    uint8 r = g_vis.r_border[y];
    return (l != BORDER_INVALID && r != BORDER_INVALID && l < r); // 这里只要求坐标和左右顺序，不查exist，搜索外推坐标也可能满足
}

/* 下拐点行查找（用户既定逻辑）：
 * 从 near+5 开始向下找第一行“左右边界都真实存在(exist=1)且 l<r”的行；
 * 找到且行号 ≤115 就用该行左右边界；
 * 一路找不到（例如右下拐点出画面）到 115 都没找到 → 进入 snap：dn=119, L=0, R=187。 */
static bool cross_find_dn_both_exist(uint8 start, uint8 *row)
{
    for (uint8 y = start; y <= CROSS_DN_SNAP_ROW; y++) // start=near+5，向近车方向(行号增大)找，最多到115
    {
        if (g_vis.l_border_exist[y]                    // 要求exist=1；当前主流程在十字补线之前调用这个搜索
         && g_vis.r_border_exist[y]                    // 右墙同样要exist=1
         && g_vis.l_border[y] != BORDER_INVALID        // 排除无效255；合法屏幕边缘0/187不是255
         && g_vis.r_border[y] != BORDER_INVALID
         && g_vis.l_border[y] < g_vis.r_border[y])     // 左右顺序正确，防止错边行当选
        {
            *row = y;      // 输出找到的下拐点行
            return true;
        }
    }
    return false; // 到115都没有双边真实行，调用者改用屏幕底角snap
}

/* 上拐点稳定判据：y 和 y-3 两行（y-3 离横道更远）左右边界列号变化都 ≤ 8。
 * 与下拐点对称，防止 far_row 上缘同样存在逐行外扩/收缩的过渡行。 */
static bool cross_anchor_up_stable(uint8 y)
{
    if (y < (uint8)(5 + CROSS_ANCHOR_STABLE_GAP)) return false;  /* y-3 必须 ≥5（扫线下限）*/
    uint8 y_ref = (uint8)(y - CROSS_ANCHOR_STABLE_GAP); // 上拐点与再远3行比较，上方行号更小
    if (!cross_anchor_row_valid(y) || !cross_anchor_row_valid(y_ref)) return false;

    int16 dl = (int16)g_vis.l_border[y_ref] - (int16)g_vis.l_border[y];
    int16 dr = (int16)g_vis.r_border[y_ref] - (int16)g_vis.r_border[y];
    if (dl < 0) dl = -dl;
    if (dr < 0) dr = -dr;
    return (dl <= CROSS_ANCHOR_STABLE_COL && dr <= CROSS_ANCHOR_STABLE_COL); // 左右墙都变化不超过8px才认为进入稳定段
}

/* 从起始行向上找第一个稳定锚点行；找不到返回起始行 */
static uint8 cross_find_up_stable(uint8 start)
{
    uint8 row = start; // 默认返回起始行；未找到更稳定点不表示保证有真实锚点，调用者还要校验
    uint8 minv = (start >= (uint8)(5 + CROSS_ANCHOR_SEARCH_MAX))
               ? (uint8)(start - CROSS_ANCHOR_SEARCH_MAX) : 5;
    for (uint8 y = start; y >= minv; y--)
        if (cross_anchor_up_stable(y)) { row = y; break; }
    return row; // 后续cross_detect会读取该行左右坐标并检查有效性
}

/* 四拐点“稳定段+突变段”校验（借用 CSDN qq_53239103 的思路）：
 * up/dn 拐点之间必须存在 INVALID 行（横道断带）或 ≥6px 的边界跳变，
 * 否则认为只是弯道边沿渐变，不是十字拐点。 */
static bool cross_corner_has_band_gap(const uint8 line[pho_h], uint8 up_row, uint8 dn_row)
{
    bool  has_invalid = false; // 上下点中间是否出现边界断带
    bool  have_prev   = false; // 是否有可比较的前一有效点，断带后不能直接跨段比较跳变
    uint8 prev        = BORDER_INVALID;
    int16 max_jump    = 0; // 这段相邻有效边界的最大列跳变(px)

    for (uint8 y = up_row + 1; y < dn_row; y++)
    {
        uint8 v = line[y];
        if (v == BORDER_INVALID) { has_invalid = true; have_prev = false; continue; }
        if (have_prev)
        {
            int16 d = (int16)v - (int16)prev;
            if (d < 0) d = -d;
            if (d > max_jump) max_jump = d;
        }
        prev = v;
        have_prev = true;
    }
    return (has_invalid || max_jump >= CROSS_CORNER_JUMP_COL); // 有断带或跳变≥6px才有横道拐角证据，不把平缓弯墙直接当十字
}


/* (十字独立ROI扫边已删除：四拐点候选版直接复用 sweep 边界) */

/* (车道行有效性 + 全宽白带检测已删除：白区判定已并入 sweep_boundaries) */

/* (单侧拐点搜索 + 拐点对提取已删除：拐点改由 sweep 白区候选行直接得出) */

/* ================================================================
 * 十字补线 — 四拐点直线桥接（下拐点 ← 上拐点）
 * ================================================================ */
static void cross_fill_segment(border_line line, uint8 exist[pho_h],
                               uint8 c0, uint8 r0, uint8 c1, uint8 r1)
{
    if (r1 <= r0) return; // 必须给远行r0<近行r1；同行或顺序反向直接返回，当前代码不交换端点

    /* 桥接必须经过调用者指定的拐点。旧最小二乘读取附近旧边界，
     * 会忽略c0/c1，斜入时甚至把“缺边固定到屏幕边缘”拟合成另一条斜线。
     * 如需平滑应在识别锚点时处理，不能让补线偏离锚点。 */
    if (r0 >= pho_h || r1 >= pho_h || c0 >= pho_w || c1 >= pho_w) return;
    float k = (float)((int16)c1-(int16)c0)/(float)((int16)r1-(int16)r0); // 列差/行差得到直线斜率，先转有符号防uint8相减绕回
    float b = (float)c0-k*(float)r0; // 直线截距，使c=k*r+b通过第一个锚点

    for (uint16 r = r0; r <= r1; r++) // 从远到近补整段，包含上下两个端点
    {
        int32 c = (int32)(k * (float)r + b + 0.5f); // 当前行插值列，转整数前加0.5作四舍五入
        if (r == r0) c = c0; // 上端点强制精确经过原锚点，避免浮点舍入偏一列
        if (r == r1) c = c1; // 下端点同样保证精确
        line[r]  = (uint8)cross_limit_u8(c, 0, MT9V03X_W - 1);
        exist[r] = 1; // 补线允许后续中线使用，因此exist=1，但下面repair_flags会记录它不是实际墙
        /* 补线也会设置exist=1，所以独立保存补线来源，不能仅靠exist判断。 */
        if (line == g_vis.l_border) vision_repair_flags[r] |= 1u; // 左补线置bit0，|=保留本行已有右补线位
        if (line == g_vis.r_border) vision_repair_flags[r] |= 2u; // 右补线置bit1；两边都补时flags=3
    }
}

static void cross_repair(void) // 用已确认十字的拐点桥接被横道打断的边界；只改g_vis边界数组
{
    if (cross_state != CROSS_STATE_DETECTED || !cross_corners_valid)
        return; // 未确认十字或锚点不可用时保持sweep原始边界，不做任何补线

    if (cross_oblique_used)
    {
        /* 斜入：有角侧补真实上下拐点，丢线侧补到屏幕边缘 */
        if (cross_oblique_real_side == 1)   /* 使用与保持拐点一起保存的方向 */
        {
            cross_fill_segment(g_vis.l_border, g_vis.l_border_exist,
                               cross_up_l_col, cross_up_l_row,
                               cross_dn_l_col, cross_dn_l_row);

            /* 右墙丢线：从下拐点行补到屏幕最上方 y=0，列固定 187 */
            cross_fill_segment(g_vis.r_border, g_vis.r_border_exist,
                               (uint8)pho_w_max, 0,
                               (uint8)pho_w_max, cross_dn_l_row);
        }
        else                          /* 右墙有角 → 补右墙，左墙丢线补 x=0 */
        {
            cross_fill_segment(g_vis.r_border, g_vis.r_border_exist,
                               cross_up_r_col, cross_up_r_row,
                               cross_dn_r_col, cross_dn_r_row);

            /* 左墙丢线：从下拐点行补到屏幕最上方 y=0，列固定 0 */
            cross_fill_segment(g_vis.l_border, g_vis.l_border_exist,
                               0, 0,
                               0, cross_dn_r_row);
        }
        return; // 斜入方案已补完，不再执行下面的正入四拐点补线
    }

    /* 左墙：左上拐点(远) → 左下拐点(近)；只补两侧都有效的墙 */
    if (cross_up_l_col != BORDER_INVALID && cross_dn_l_col != BORDER_INVALID)
    {
        cross_fill_segment(g_vis.l_border, g_vis.l_border_exist,
                           cross_up_l_col, cross_up_l_row,
                           cross_dn_l_col, cross_dn_l_row);
    }

    /* 右墙：右上拐点(远) → 右下拐点(近)；只补两侧都有效的墙 */
    if (cross_up_r_col != BORDER_INVALID && cross_dn_r_col != BORDER_INVALID)
    {
        cross_fill_segment(g_vis.r_border, g_vis.r_border_exist,
                           cross_up_r_col, cross_up_r_row,
                           cross_dn_r_col, cross_dn_r_row);
    }
}

/* ================================================================
 * 十字检测主函数 — 候选 → 锚点校验 → 连续确认/短时保持（当前没有冷却状态）
 *
 * 下拐点：near+5 起向下找“双边都真实存在”的第一行；
 *         到 115 仍找不到 → snap 屏幕底角(119, 0, 187)。
 * 上拐点：far-3 起向上找稳定锚点行（搜不到保持 far-3）。
 * 正入失败后采用扫描得到的斜入候选；斜入候选直接来自本帧边界扫描，不再保留独立错边检测分支。
 * ================================================================ */
/* 保存正常十字候选的四拐点，供随后确认、cross_repair补线和图像标注。
 * 输入up_*为远侧(行号小)，dn_*为近侧(行号大)；左右上拐点可不同行，左右下拐点共用dn_row。
 * 本函数只存值，不校验顺序/有效性；当前正常候选调用前已校验四点，漏检保持时不调用它。 */
static void cross_set_corners(uint8 up_l, uint8 up_l_row,
                              uint8 up_r, uint8 up_r_row,
                              uint8 dn_l, uint8 dn_r, uint8 dn_row)
{
    cross_dn_l_col = dn_l; cross_dn_l_row = dn_row; // 左下拐点(近)：列、行
    cross_dn_r_col = dn_r; cross_dn_r_row = dn_row; // 右下拐点(近)：与左下共用同一行
    cross_up_l_col = up_l; cross_up_l_row = up_l_row; // 左上拐点(远)：列、行
    cross_up_r_col = up_r; cross_up_r_row = up_r_row; // 右上拐点(远)：行号可不同于左上
}

static void cross_detect(void)
{
    bool pair_ok = false; // 本帧是否通过至少一种十字锚点校验，先尝试正入，再尝试斜入

    if (cross_cand_present)
    {
        uint8 dn_start = (uint8)(cross_near_row + CROSS_DN_ROW_OFFSET);
        bool  dn_snap  = (dn_start > CROSS_DN_SNAP_ROW); // 下拐点搜索起点已超过115时直接用底角兜底

        uint8 up_start = (uint8)(cross_far_row - CROSS_UP_ROW_OFFSET);
        if (up_start < 5) up_start = 5; // 上拐点起点不能进入普通扫线未处理的0..4行

        uint8 dn_row = dn_start;
        uint8 dn_l, dn_r;
        bool  dn_found = false; // 有无找到实际双边都真实的下拐点行
        if (!dn_snap)
            dn_found = cross_find_dn_both_exist(dn_start, &dn_row);

        if (!dn_found) {
            /* 到 115 都没找到双边真实存在的行（例如右下拐点出画面）→ snap 屏幕底角 */
            dn_snap = true;
            dn_row = (uint8)(pho_h - 1);
            dn_l = 0; dn_r = (uint8)pho_w_max;
        } else {
            dn_l = g_vis.l_border[dn_row];
            dn_r = g_vis.r_border[dn_row];
        }

        uint8 up_row = cross_find_up_stable(up_start);
        uint8 up_l = g_vis.l_border[up_row];
        uint8 up_r = g_vis.r_border[up_row];

        /* 四拐点有效 + 左右墙都有断带/跳变（snap 底角不查断带）*/
        bool corners_ok = (dn_l != BORDER_INVALID && dn_r != BORDER_INVALID
                        && up_l != BORDER_INVALID && up_r != BORDER_INVALID
                        && dn_l < dn_r && up_l < up_r);
        bool gap_ok = (dn_snap
                    || (cross_corner_has_band_gap(g_vis.l_border, up_row, dn_row)
                     && cross_corner_has_band_gap(g_vis.r_border, up_row, dn_row)));

        if (corners_ok && gap_ok) // 坐标、左右顺序和断带条件同时成立，才采纳正入锚点
        {
            cross_set_corners(up_l, up_row, up_r, up_row, dn_l, dn_r, dn_row);
            pair_ok = true;
            cross_oblique_used = false;
        }
    }

    /* 正入失败后允许斜入兜底：只要斜入已经找到有角侧上下拐点，就使用 */
    if (!pair_ok && oblique_valid) // 正入不成立才用本帧扫描确认的斜入锚点，避免两套补线同时抢结果
    {
        if (oblique_real_side == 1)   /* 左墙有角 */
        {
            cross_up_l_col = oblique_up_col;
            cross_up_l_row = oblique_up_row;
            cross_dn_l_col = oblique_dn_col;
            cross_dn_l_row = oblique_dn_row;

            /* 丢线侧清空，防止残留旧值画出假线 */
            cross_up_r_col = BORDER_INVALID;
            cross_up_r_row = BORDER_INVALID;
            cross_dn_r_col = BORDER_INVALID;
            cross_dn_r_row = BORDER_INVALID;
        }
        else                          /* 右墙有角 */
        {
            cross_up_r_col = oblique_up_col;
            cross_up_r_row = oblique_up_row;
            cross_dn_r_col = oblique_dn_col;
            cross_dn_r_row = oblique_dn_row;

            /* 丢线侧清空，防止残留旧值画出假线 */
            cross_up_l_col = BORDER_INVALID;
            cross_up_l_row = BORDER_INVALID;
            cross_dn_l_col = BORDER_INVALID;
            cross_dn_l_row = BORDER_INVALID;
        }
        pair_ok = true;
        cross_oblique_used = true;
        cross_oblique_real_side = oblique_real_side;
    }

    if (pair_ok) // 本帧锚点可用，进入跨帧连续确认
    {
        cross_corners_valid = true; // 候选阶段也保存可显示锚点，但候选还不执行补线
        cross_missed_count  = 0; // 本帧识别成功，漏检保持计数重新从0开始
        if (cross_confirm_count < CROSS_CONFIRM_FRAMES)
            cross_confirm_count++; // 按新处理图像计数，不按5ms中断次数计数
        cross_state = (cross_confirm_count >= CROSS_CONFIRM_FRAMES)
                    ? CROSS_STATE_DETECTED : CROSS_STATE_CANDIDATE;
    }
    else if (cross_state == CROSS_STATE_DETECTED
             && cross_missed_count < CROSS_HOLD_MISSED_FRAMES)
    {
        cross_missed_count++; // 保持原锚点和有角侧方向，最多4个漏检帧，下一次再失败才清除
    }
    else
    {
        /* 离开十字 → 直接复位，不再冷却 */
        cross_state = CROSS_STATE_NONE;
        cross_corners_valid = false;
        cross_confirm_count = 0;
        cross_missed_count  = 0; // 退出十字后，漏检保持计数也清零
        cross_oblique_used  = false;

        /* 清空拐点，防止调试/轨迹导出残留上一帧值 */
        cross_up_l_col = BORDER_INVALID;
        cross_up_l_row = BORDER_INVALID;
        cross_up_r_col = BORDER_INVALID;
        cross_up_r_row = BORDER_INVALID;
        cross_dn_l_col = BORDER_INVALID;
        cross_dn_l_row = BORDER_INVALID;
        cross_dn_r_col = BORDER_INVALID;
        cross_dn_r_row = BORDER_INVALID;
    }
}

/* ================================================================
 * 十字复位 — 发车时清零状态
 * ================================================================ */
void cross_reset(void)
{
    cross_state          = CROSS_STATE_NONE; // 回到"无十字"，非候选/确认
    cross_corners_valid  = false;             // 旧锚点作废，防止补线沿用发车前的点
    vision_corners_in_frame = 0;              // 本帧不允许上传拐点
    cross_confirm_count  = 0;                 // 连续确认帧计数清零
    cross_missed_count   = 0;                 // 漏检保持计数清零
    cross_cand_present   = false;             // 本帧扫描候选清零

    /* 清空拐点，防止复位后残留旧值 */
    cross_up_l_col = BORDER_INVALID;
    cross_up_l_row = BORDER_INVALID;
    cross_up_r_col = BORDER_INVALID;
    cross_up_r_row = BORDER_INVALID;
    cross_dn_l_col = BORDER_INVALID;
    cross_dn_l_row = BORDER_INVALID;
    cross_dn_r_col = BORDER_INVALID;
    cross_dn_r_row = BORDER_INVALID;
    oblique_state           = OBLIQUE_IDLE; // 斜入搜索阶段复位，重新累计丢边行数
    oblique_valid           = false;        // 本帧无斜入锚点
    oblique_lost_run        = 0;            // 连续丢边行数清零
    oblique_lost_side       = 0;            // 丢线侧未定
    oblique_real_side       = 0;            // 有角侧未定
    oblique_pending_confirm = 0;            // 上拐点待确认进度清零
    oblique_dn_row          = 0;            // 下拐点(近)行/列
    oblique_dn_col          = 0;
    oblique_up_row          = 0;            // 上拐点(远)行/列
    oblique_up_col          = 0;
    oblique_pending_up_row  = 0;            // 待确认上拐点行/列
    oblique_pending_up_col  = 0;
    cross_oblique_used      = false;        // 本帧不使用斜入补线
    cross_oblique_real_side = 0;            // 保持的有角侧方向清零
    zebra_prev           = false;   /* 上一帧状态置假，配合zebra_detected防首帧假上升沿 */
    zebra_detected       = false;   /* 防斑马线上发车：首帧即产生一次虚假ZEBRA上升沿 */
    zebra_missed_count   = 0;       /* 斑马线漏检保持计数清零 */
    zebra_confirm_count  = 0;       /* 斑马线连续确认计数清零 */
    oob_black_rows_dbg   = 0;       /* 出界诊断行数清零 */
    oob_track_l          = pho_w / 4;      /* 出界统计窗口左端回到默认47 */
    oob_track_r          = pho_w * 3 / 4;  /* 出界统计窗口右端回到默认141 */
    zebra_track_l        = pho_w / 4;      /* 斑马线统计窗口左端回到默认47 */
    zebra_track_r        = pho_w * 3 / 4;  /* 斑马线统计窗口右端回到默认141 */
    last_valid_width     = 60;      /* 逐行扫描外推用的默认路宽(px) */
    last_real_width      = 60;      /* 最靠近车身的真实路宽默认值 */
    memset(sweep_row_width, 0, sizeof(sweep_row_width)); // 本帧逐行宽度缓存清0
    center_cut_row = -1;            /* -1表示本帧中线尚未贴边 */
    center_cut_col = 0;             /* 贴边处中心列复位 */
    center_cut_slope = 0.0f;        /* 缺行外推斜率复位 */
}

/* ================================================================
 * 视觉整体复位（发车时调用）
 *
 * cross_reset() 已经覆盖了十字状态、斑马线状态、出界窗口与路宽外推基准，
 * 本函数作为统一入口供 control_state_launch() 调用，语义更清晰；
 * 后续若新增别的视觉状态，只需在这里补充，不必改控制层。
 * ================================================================ */
void vision_reset(void)
{
    cross_reset();
}


/* ================================================================
 * 斑马线检测 — 5像素多数表决滤波
 *
 *   对横向 ±ZEBRA_FILTER_RADIUS 窗口做多数表决，
 *   消除单/双像素噪点后再判断该列是否为白。
 * ================================================================ */
static bool zebra_pixel_is_white(uint8 row, uint16 col)
{
    uint16 start = (col > ZEBRA_FILTER_RADIUS)
                 ? col - ZEBRA_FILTER_RADIUS : 0;
    uint16 end   = col + ZEBRA_FILTER_RADIUS;
    uint8  white = 0, total = 0; // 邻域内白点数和实际点数；图像左右边缘窗口会缩短

    if (end >= MT9V03X_W) end = MT9V03X_W - 1;

    for (uint16 c = start; c <= end; c++) // 采样当前列左右各2列，共最多5个二值像素
    {
        if (Image_Used[row][c] == 255) white++;
        total++;
    }
    return (white * 2 >= total + 1);  /* 严格超过半数才判白：white > total/2，等价 white*2>=total+1；偶数窗口(边缘total=4)要3票，ceil(n/2)=2票不准确 */
}

/* ================================================================
 * 斑马线行扫描 — 只在“赛道窗口”[col_l,col_r] 内检查单行
 *
 *   条件：白像素占比 25%~75%、跳变 ≥10、
 *         黑白连续段各 ≥5（段宽 ≥4）
 *   同时把每个跳变发生的列号保存下来，供 zebra_detect() 做
 *   “竖向条纹对齐”检查：真斑马线的条纹在 119/114/109 三行
 *   上跳变位置应基本一致；阈值噪声/坡道灰度抖动产生的假条纹
 *   各行动作随机，会被对齐检查剔除。
 * ================================================================ */
typedef struct {
    bool  valid;
    uint8 transitions;
    uint8 trans_col[ZEBRA_TRANS_COLS_MAX];  /* 每次黑白跳变的首列 */
} zebra_row_info_t;

static void zebra_row_scan(uint8 row, uint8 col_l, uint8 col_r,
                           zebra_row_info_t *info)
{
    info->valid       = false; // 先置无效，条件全部满足才在函数末尾改为有效
    info->transitions = 0;

    if (col_l >= col_r || (uint16)(col_r - col_l + 1) < 30) return;

    uint16 span = (uint16)(col_r - col_l + 1); // 统计窗口宽度，含左右两个端点

    bool   last_white  = zebra_pixel_is_white(row, col_l); // 第一列经过多数表决后的颜色，作为第一个连续段起点
    uint16 white_count = last_white ? 1 : 0; // 第一列是白就先计1个，后续从第二列开始扫描
    uint16 run_width   = 1; // 当前同色连续段长度，初始已包含第一列
    uint8  transitions = 0; // 黑白颜色切换次数，并非黑段数量
    uint8  black_runs  = 0; // 宽度至少4px的黑段数，小噪声黑点不算有效黑条
    uint8  white_runs  = 0; // 宽度至少4px的白段数

    for (uint16 col = (uint16)col_l + 1; col <= col_r; col++)
    {
        bool cur = zebra_pixel_is_white(row, col);
        if (cur) white_count++;

        if (cur == last_white) // 颜色没变，当前连续段延长；没有新跳变
        {
            run_width++;
            continue;
        }

        /* 颜色跳变：记录首列，同时统计上一种颜色的有效段 */
        if (transitions < ZEBRA_TRANS_COLS_MAX)
            info->trans_col[transitions] = (uint8)col;
        transitions++;
        if (run_width >= ZEBRA_RUN_WIDTH_MIN)
        {
            if (last_white) white_runs++; else black_runs++;
        }
        last_white = cur; // 发生跳变后，以当前颜色开始下一段
        run_width = 1; // 新段只包含当前列，长度重新计1
    }

    /* 收尾最后一段 */
    if (run_width >= ZEBRA_RUN_WIDTH_MIN)
    {
        if (last_white) white_runs++; else black_runs++;
    }

    info->transitions = transitions; // 保存总跳变次数；跳变列数组最多只存前40个，后续对齐会夹到40
    info->valid =
           ((uint32)white_count * 100 >= (uint32)span * ZEBRA_WHITE_RATIO_MIN)
        && ((uint32)white_count * 100 <= (uint32)span * ZEBRA_WHITE_RATIO_MAX)
        && (transitions   >= ZEBRA_TRANSITIONS_MIN)
        && (black_runs    >= ZEBRA_BLACK_RUNS_MIN)
        && (white_runs    >= ZEBRA_WHITE_RUNS_MIN);
}

/* 相邻两行跳变列对齐率：a 中每个跳变列在 b 中找 ±ZEBRA_ALIGN_TOL_COL
 * 内的对应列，命中数 / min(两行跳变数) >= ZEBRA_ALIGN_MIN_RATIO。 */
static bool zebra_rows_aligned(const zebra_row_info_t *a,
                               const zebra_row_info_t *b)
{
    if (!a->valid || !b->valid) return false;

    /* 只比较已保存的跳变列（最多 ZEBRA_TRANS_COLS_MAX 个） */
    uint8 na = a->transitions;
    if (na > ZEBRA_TRANS_COLS_MAX) na = ZEBRA_TRANS_COLS_MAX;
    uint8 nb = b->transitions;
    if (nb > ZEBRA_TRANS_COLS_MAX) nb = ZEBRA_TRANS_COLS_MAX;

    uint8 denom = na; // 用两行较小跳变数作对齐率分母，减少另一行多几条噪声的影响
    if (nb < denom) denom = nb;
    if (denom == 0) return false; // 没有跳变不能形成条纹，也避免比例计算分母为0

    uint8 match = 0; // 第一行跳变在第二行±3列内找到对应的个数
    for (uint8 i = 0; i < na; i++)
    {
        int16 ca = (int16)a->trans_col[i];
        for (uint8 j = 0; j < nb; j++)
        {
            int16 cb = (int16)b->trans_col[j];
            int16 d  = ca - cb;
            if (d < 0) d = -d;
            if (d <= ZEBRA_ALIGN_TOL_COL)
            {
                match++;
                break;
            }
        }
    }
    return ((uint16)match * 100 >= (uint16)denom * ZEBRA_ALIGN_MIN_RATIO);
}


/* ================================================================
 * 斑马线检测 — 底部3条采样线中至少2条有效 + 竖向条纹对齐 + 确认/短时保持
 *
 *   采样行：119, 114, 109（最底行向上每5行一条）
 *   赛道窗口：用上一帧底部真实边界 ± margin；斑马线期间保持最近窗口。
 *   确认机制：连续 ZEBRA_CONFIRM_FRAMES 帧有效才置位；
 *            漏检≤2帧时维持上一帧状态。
 * ================================================================ */
static void zebra_detect(void)
{


    /* 上一帧底部边界真实时更新赛道窗口；斑马线/丢线时保持最近有效窗口 */
    if (g_vis.l_border[pho_h - 1] != BORDER_INVALID
        && g_vis.r_border[pho_h - 1] != BORDER_INVALID
        && g_vis.l_border_exist[pho_h - 1]
        && g_vis.r_border_exist[pho_h - 1])
    {
        uint8 l = g_vis.l_border[pho_h - 1];
        uint8 r = g_vis.r_border[pho_h - 1];
        if (l < r && r - l >= 20)
        {
            zebra_track_l = (l >= ZEBRA_TRACK_MARGIN) ? (uint8)(l - ZEBRA_TRACK_MARGIN) : 0;
            zebra_track_r = (r <= pho_w_max - ZEBRA_TRACK_MARGIN)
                    ? (uint8)(r + ZEBRA_TRACK_MARGIN) : pho_w_max;
        }
    }

    zebra_row_info_t rows[ZEBRA_SAMPLE_ROWS]; // 分别保存119/114/109三行扫描结果和跳变列
    uint8 valid_rows = 0; // 本帧符合白黑比例、跳变、段数要求的采样行数
    for (uint8 s = 0; s < ZEBRA_SAMPLE_ROWS; s++)
    {
        uint8 row = MT9V03X_H - 1 - s * ZEBRA_SAMPLE_ROW_STEP; // s=0/1/2分别映射119/114/109，不是从顶行检测
        zebra_row_scan(row, zebra_track_l, zebra_track_r, &rows[s]);
        if (rows[s].valid) valid_rows++;
    }

    /* 有效行≥2后，再要求“竖向条纹对齐”：任取两行有效行组成的
     * 跳变列对齐对，命中对数要 >= 有效行数-1。
     * 3行全有效 → 3对中至少2对对齐；2行有效 → 这一对必须对齐。
     * 坡道/阈值噪声各行动作随机，对齐率低，不会进入确认计数。 */
    bool aligned = false; // 尚未通过多行条纹对齐，单行好看也不能直接判终点
    if (valid_rows >= ZEBRA_VALID_ROWS_MIN)
    {
        uint8 pairs_ok = 0; // 三行之间满足对齐的有效行对数量
        for (uint8 i = 0; i < ZEBRA_SAMPLE_ROWS; i++)
        {
            if (!rows[i].valid) continue;
            for (uint8 j = i + 1; j < ZEBRA_SAMPLE_ROWS; j++)
            {
                if (!rows[j].valid) continue;
                if (zebra_rows_aligned(&rows[i], &rows[j])) pairs_ok++;
            }
        }
        aligned = (pairs_ok + 1 >= valid_rows);   /* pairs_ok >= valid_rows-1 */
    }

    if (aligned)
    {
        uint8 confirm_need = (uint8)ZEBRA_CONFIRM_FRAMES; // 当前宏为1；必须先通过至少2行有效和对齐，才累计确认
        if (zebra_confirm_count < confirm_need) zebra_confirm_count++;
        zebra_detected      = (zebra_confirm_count >= confirm_need); // 达到确认门限才进入检测态，随后vis_deal只在首次上升沿发ZEBRA事件
        zebra_missed_count  = 0;
    }
    else if (valid_rows == 1 && !zebra_detected)
    {
        /* 单行有效只做预确认：计数置1，但本帧不置zebra_detected。
         * 后续必须至少2行有效且跳变列对齐，才走上面的aligned分支。
         * 最多容忍一个额外单行间隔，再出现单行就清零。
         * 当前确认门限为1；这段仍保留用于单行候选的状态维护，不能理解成必须两帧确认。 */
        if (zebra_confirm_count == 0)
        {
            zebra_confirm_count = 1;    /* 预确认，等待至少2行有效且竖向对齐的帧 */
        }
        else if (zebra_missed_count == 0)
        {
            zebra_missed_count = 1;     /* 容忍一个单行间隔帧 */
        }
        else
        {
            zebra_detected      = false;
            zebra_missed_count  = 0;
            zebra_confirm_count = 0;
        }
    }
    else if (zebra_detected && zebra_missed_count < ZEBRA_HOLD_MISSED_FRAMES)
    {
        zebra_missed_count++;
    }
    else
    {
        zebra_detected      = false;
        zebra_missed_count  = 0;
        zebra_confirm_count = 0;
    }

    /* 诊断输出：供 VOFA 判断终点是否真的看到斑马线 */
    zebra_dbg_detected   = zebra_detected ? 1 : 0;
    zebra_dbg_valid_rows = valid_rows;
    zebra_dbg_confirm    = zebra_confirm_count;

}


/* ================================================================
 * 底部连通滤波：只保留与图像底部连通的白像素（赛道），清除环境白
 *
 * 算法：自底向上逐行传播连通标记
 *   1. 底部行：所有白像素初始标记为连通
 *   2. 向上逐行：膨胀上一行的连通标记（±2列），当前行只保留
 *      与膨胀标记重叠的白像素
 *   3. 同排传播对称两趟：Pass1 左→右（±2膨胀+同排左链，只标记不擦除）
 *      Pass2 右→左补同排右链，Pass3 统一擦除 —— 消除单趟"右无限
 *      左有限"造成的左侧啃边偏差（2026-08-13）
 *
 * 优势：±2列膨胀容忍小间断，白墙/反光/场边杂物不与底部连通 → 被滤除
 * ================================================================ */
#define FILTER_BOTTOM_ROWS VIS_FILTER_BOTTOM_ROWS  /* Round69: 120→110，去除顶部10行远距离滤波，确保Otsu重算帧也不超20ms */
static void filter_bottom_connected(void)
{
    uint8 connected[pho_w]; // 当前行188列连通标志：1能沿白区接到图像底部，0还未接到
    uint8 prev[pho_w]; // 近一行的连通副本，避免本行更新connected影响读取近行结果

    /* 底部行：所有白像素初始标记为连通 */
    for (int x = 0; x < pho_w; x++)
        connected[x] = (Image_Used[pho_h - 1][x] == 255);

    /* 向上逐行传播（仅底部 FILTER_BOTTOM_ROWS 行，降低耗时） */
    int y_min = pho_h - FILTER_BOTTOM_ROWS; // 默认120-110=10，处理119..10的底部区域，0..9不滤
    if (y_min < 0) y_min = 0;
    for (int y = pho_h - 2; y >= y_min; y--) // 从118向远处传播；底行119已作为初始连通种子
    {
        /* 保存上一行连通标记（合并循环在读connected的同时写它，必须用副本） */
        for (int x = 0; x < pho_w; x++)
            prev[x] = connected[x]; // 本行开始前保存近一行各列的状态，之后connected可重新写

        /* Pass 1: 左→右 标记（±2列膨胀 + 同排左链），只标记不擦除 */
        for (int x = 0; x < pho_w; x++)
        {
            if (Image_Used[y][x] == 255)
            {
                /* 检查上方连通：prev[x] ±2 列 或 左边已连通 */
                if (prev[x]
                    || (x > 0        && prev[x - 1])
                    || (x > 1        && prev[x - 2])
                    || (x < pho_w - 1 && prev[x + 1])
                    || (x < pho_w - 2 && prev[x + 2])
                    || (x > 0        && connected[x - 1]))
                {
                    connected[x] = 1;
                }
                else
                {
                    connected[x] = 0;
                }
            }
            else
            {
                connected[x] = 0;
            }
        }

        /* Pass 2: 右→左 补同排右链（对称化：白段右半能吃的连通，左半同样能吃） */
        for (int x = pho_w - 2; x >= 0; x--)
        {
            if (Image_Used[y][x] == 255 && !connected[x] && connected[x + 1])
                connected[x] = 1;
        }

        /* Pass 3: 统一擦除：白像素仍未连通 → 抹掉 */
        for (int x = 0; x < pho_w; x++)
        {
            if (Image_Used[y][x] == 255 && !connected[x])
                Image_Used[y][x] = 0;
        }
    }
}

/* ================================================================
 * 出界检测 —— 在底部连通滤波前，检查原始二值图的底部 20 行
 *
 *   必须在 filter_bottom_connected() 之前调用：滤波算法以底行为
 *   “种子”，底行一旦全黑，会连锁把整幅图的白都擦掉；如果等滤波
 *   后再看 118/117 行，它们必定也已经黑了，无法区分“坡道近处单
 *   行阴影”和“真出界”。
 *   老规则只看 y=119 一行全黑：上/下坡道时近处坡面会变暗，单是
 *   底行被判黑就连续 3 帧触发 LOST 停车（实车 reason=2）。因此
 *   改成统计 y=100..119 这 20 行、上一帧赛道窗口 ±3 列内的白像
 *   素总量：坡道上只有近处几行黑，上方仍有大量白；真出界时车底
 *   这一整带都没有赛道，白像素会接近 0。
 *   斑马线优先在 vis_deal 的事件决策处处理，这里只扫描原始图。
 * ================================================================ */
#define OOB_CHECK_ROWS         20    /* 检查 y=100..119 共 20 行 */
#define OOB_MIN_WHITE_PIXELS   40    /* 窗口内白像素低于 40 个才算 LOST */
#define OOB_WINDOW_MARGIN      3

static int out_of_bounds_check(void)
{
    uint8 l = g_vis.l_border[pho_h - 1]; // 当前新帧尚未扫线，读到的是上一帧底部左墙
    uint8 r = g_vis.r_border[pho_h - 1]; // 上一帧底部右墙，用于限定本帧原始二值图检查区域
    bool  win_ok = (l != BORDER_INVALID && r != BORDER_INVALID
                    && g_vis.l_border_exist[pho_h - 1]
                    && g_vis.r_border_exist[pho_h - 1]
                    && l < r && (r - l) >= 20);

    uint8 wl, wr; // 真正用于统计的窗口左右端，含端点
    if (win_ok)
    {
        wl = (l >= OOB_WINDOW_MARGIN) ? (uint8)(l - OOB_WINDOW_MARGIN) : 0;
        wr = (r <= pho_w_max - OOB_WINDOW_MARGIN)
           ? (uint8)(r + OOB_WINDOW_MARGIN) : pho_w_max;
        oob_track_l = wl;            /* 更新保持窗口，后续帧底边界失效时沿用 */
        oob_track_r = wr;
    }
    else
    {
        wl = oob_track_l;            /* 上一帧底边界失效（坡道/LOST期间）→ 沿用最近有效窗口 */
        wr = oob_track_r;
    }

    uint8  black_rows  = 0; // 底部20行中窗口完全没有白点的行数，仅作诊断
    uint16 white_total = 0; // 整个底部检查带的白点总数，实际LOST判据用它而不是black_rows
    for (uint8 y = pho_h - OOB_CHECK_ROWS; y < pho_h; y++) // 检查100..119，不用远处图像决定近处是否出界
    {
        bool   has_white = false; // 本行窗口是否至少出现一个白点
        uint16 row_white = 0; // 本行白点数，最后加入20行总数
        for (uint16 x = wl; x <= wr; x++)
        {
            if (Image_Used[y][x] == 255)
            {
                has_white = true;
                row_white++;
            }
        }
        if (!has_white) black_rows++;
        white_total += row_white;
    }

    oob_black_rows_dbg = black_rows;
    return (white_total < OOB_MIN_WHITE_PIXELS); // 少于40个白像素返回1(出界预检)，app再累计LOST帧决定停车
}

/* ================================================================
 * 搜线主函数
 *
 * 流程：
 *   1. Otsu 自适应阈值 + 一次性二值化 → Image_Used[][]
 *   2. 原始二值图出界预检 → 底部连通滤波（出界必须先查，防止滤波擦图误判）
 *   3. 斑马线检测 + 事件判定 → 上报事件（停车决策由app/control做，斑马线优先）
 *   4. sweep_boundaries → 逐行扫描边界
 *   5. cross_detect + cross_repair → 十字补线（覆盖被横道干扰的边界）
 *   6. pho_center → 中线 + 偏差
 *
 * 返回：本帧事件（VISION_EVT_NONE / LOST / ZEBRA），不再直接停车
 * ================================================================ */
/* ================================================================
 * 视觉模块初始化（RT1064 适配新增）
 *
 * 原 summer 工程的耗时统计用 MM32 的 TIM_8，上电后由定时器初始化统一打开；
 * 本工程改用 RT1064 的 GPT1（GPT2 已被 zf_driver_delay 的 system_delay 占用），
 * 需要显式初始化为微秒模式，否则 timer_get 读到的是外设时钟计数值，
 * 分段耗时通道会全线错乱。
 * ================================================================ */
void vision_init(void)
{
    memset(learned_row_width, 0, sizeof(learned_row_width)); // 冷启动清同行路宽；K1的cross_reset不清，保留停车时学到的宽度
    timer_init(GPT_TIM_1, TIMER_US);                                        // 微秒计时，最大 0xFFFFFFFF
    timer_start(GPT_TIM_1);                                               // 原库初始化后停止；显式启动才有真实耗时
    timer_clear(GPT_TIM_1);
    vis_interval_us = vis_processed_frames = vis_last_event = 0;
    vision_previous_start = 0;
}

VisionEvent vis_deal(void)
{
    uint32 t_frame = timer_get(GPT_TIM_1);                                  // 本帧总耗时起点
    vis_interval_us = vis_processed_frames ? t_frame - vision_previous_start : 0; // 首帧间隔置0；以后是相邻两次开始处理间隔，unsigned差兼容计时回绕
    vision_previous_start = t_frame; // 更新下次计算间隔使用的起点，不是记录本帧结束时间
    memset(vision_repair_flags, 0, sizeof(vision_repair_flags)); // 清上一帧补线来源；本帧cross_repair需要的话会重新置位

    /* ---- Otsu 自适应阈值（每10帧重算，EMA滤波抑制跳变；优化耗时）---- */
    static int cached_th = -1; // 跨帧保留的二值化阈值，-1表示尚未做过Otsu，先用128
    static int frame_since_otsu = 0; // 距离上次重算阈值的处理帧计数；局部static不会每次调用都清0
    frame_since_otsu++; // 只有处理新图像才加1，不按主循环空转次数增加
    if (frame_since_otsu >= VIS_OTSU_INTERVAL) // 默认每10个处理帧重算，减小Otsu计算开销和阈值抖动
    {
        int th = compute_otsu();
        if (th > 0)
        {
            if (cached_th < 0)
                cached_th = th;
            else
                cached_th = (int)(0.3f * (float)th + 0.7f * (float)cached_th); // 新阈值占30%，旧值占70%，平滑光照小幅变化，不是自适应前瞻
        }
        frame_since_otsu = 0;
    }

    /* ---- 一次性二值化整张图 ---- */
    int th = (cached_th >= 0) ? cached_th : 128; // 首次Otsu之前使用128，此后用最近的平滑阈值
    uint16 t_stage = timer_get(GPT_TIM_1); // 分段起点只保留低16位，最终耗时也用uint16保存，单段范围0..65535us
    for (int y = 0; y < pho_h; y++)
        for (int x = 0; x < pho_w; x++)
            Image_Used[y][x] = (vision_gray[y][x] > th) ? 255 : 0; // 原始灰度大于阈值记白255，否则黑0；原始vision_gray不被改写
    vis_t_bin_us = (uint16)(timer_get(GPT_TIM_1) - t_stage);

    /* ---- 出界预检：必须在底部连通滤波之前，用原始二值图的
     *      y=100..119 赛道窗口判断。滤波会以底行为种子向上擦除，
     *      底行黑会把上方白全部擦掉，滤波后再查会失真。 ---- */
    int raw_oob = out_of_bounds_check(); // 在滤波擦掉亮点之前查近处白像素，先记结果，斑马线优先级随后再决定事件

    /* ---- 底部连通滤波（对称两趟版，仅处理底部 FILTER_BOTTOM_ROWS 行优化耗时）---- */
    t_stage = timer_get(GPT_TIM_1);
    filter_bottom_connected();
    vis_t_filter_us = (uint16)(timer_get(GPT_TIM_1) - t_stage);

    /* ---- 斑马线检测（先于出界判定，斑马线不出界）---- */
    zebra_detect(); // 此时读本帧滤波图，但窗口依据仍是尚未清除的上一帧真实底边界

    /* ---- 事件上报（停车决策交给 main.c 主循环）----
     * 斑马线上升沿 → VISION_EVT_ZEBRA；底部20行窗口白像素过少 → LOST。
     * 视觉层不再直接停车、也不再提前 return，保证本帧 err 正常更新。 */
    VisionEvent evt = VISION_EVT_NONE; // 默认本帧无新事件，视觉层不直接操作电机
    if (zebra_detected)
    {
        if (!zebra_prev) evt = VISION_EVT_ZEBRA;   /* 上升沿 → ZEBRA */
        /* zebra 保持帧也强制不出界（斑马线优先），不报 LOST */
    }
    else if (raw_oob)
    {
        evt = VISION_EVT_LOST;
    }
    zebra_prev = zebra_detected; // 保存本次斑马线状态，下一帧据此判断是否出现新的上升沿

    /* ---- 清空边界数组 ---- */
    for (int y = 0; y < pho_h; y++)
    {
        g_vis.l_border[y] = BORDER_INVALID;
        g_vis.r_border[y] = BORDER_INVALID;
        g_vis.l_border_exist[y] = 0;
        g_vis.r_border_exist[y] = 0;
    }

    /* ---- 逐行扫描边界（替代种子生长 + pho_border + 补全）---- */
    t_stage = timer_get(GPT_TIM_1);
    sweep_boundaries(); // 本帧搜左右边界/临时十字候选；这里不以扫描函数返回值直接判LOST
    vis_t_sweep_us = (uint16)(timer_get(GPT_TIM_1) - t_stage);

    /* 原始边界已在 sweep_boundaries() 内逐行保存，这里不再覆盖 */

    /* ---- 十字检测 + 补线（覆盖被横道干扰的边界行）----
     * 预检：没有十字候选/斜入候选时跳过整段十字逻辑，优化耗时。 */
    t_stage = timer_get(GPT_TIM_1);
    vision_corners_in_frame = 0; // 不把未执行十字检测时的旧静态拐点画在新帧上
    if (cross_cand_present || oblique_valid || cross_state != CROSS_STATE_NONE) {
        cross_detect(); // 对本帧候选进行锚点验证、跨帧确认或旧锚点漏检保持
        cross_repair(); // 只有已确认十字且锚点可用时才真正改边界数组
        vision_corners_in_frame = cross_corners_valid;
    }
    vis_t_cross_us = (uint16)(timer_get(GPT_TIM_1) - t_stage);

    /* ---- 中线 + 偏差 ---- */
    t_stage = timer_get(GPT_TIM_1);
    pho_center(); // 使用修复后边界生成中线、贴边截断及最终像素偏差
    vis_t_center_us = (uint16)(timer_get(GPT_TIM_1) - t_stage);

    vis_frame_ready = 1;
    vis_export_trace();                                                     // 导出边界/中线坐标，供 WiFi 上传给上位机显示
    vis_time_us = timer_get(GPT_TIM_1) - t_frame;                           // 本帧总耗时(us)
    vis_processed_frames++; // 计数已完成算法的图像帧，不是上传数量或摄像头驱动采集总数
    vis_last_event = (uint32)evt; // 发布最新处理完成帧的事件码，wifi参数包可读取它
    return evt; // 交给app处理发车状态/停车判定，不在vision里直接清PWM
}

/* ================================================================
 * 轨迹导出 —— 把本帧的左右边界与中线整理成"可直接上传"的坐标数组
 *
 * 为什么要单独存一份而不是让 main.c 直接读 g_vis：
 *   1) 上位机的 XY_BOUNDARY 协议要的是 (x 数组, y 数组) 成对数据，
 *      且每个数组长度 = 点数；g_vis 是按行索引的，需要转换。
 *   2) 无屏工程调参全靠这张图，导出逻辑集中在这里，main.c 保持干净。
 *   3) 算法层不调用任何传输函数，换 WiFi/UART/USB 都不用改 vision.c。
 *
 * 约定：y 从图像底部向上（先近后远）逐行取；
 *      无效点 X 置 VIS_TRACE_NONE(0xFF)，Y 仍填行号（上位机会忽略该点）。
 * ================================================================ */
uint8 vis_trace_x_left[VIS_TRACE_ROWS];
uint8 vis_trace_y_left[VIS_TRACE_ROWS];
uint8 vis_trace_x_center[VIS_TRACE_ROWS];
uint8 vis_trace_y_center[VIS_TRACE_ROWS];
uint8 vis_trace_x_right[VIS_TRACE_ROWS];
uint8 vis_trace_y_right[VIS_TRACE_ROWS];

void vis_export_trace(void)
{
    int n = 0; // 导出数组的序号；它和图像行号y方向相反

    for (int y = pho_h - 1; y >= 0 && n < VIS_TRACE_ROWS; y--) // 先导出119行，再118行直到0；每个X都配套保留实际Y
    {
        vis_trace_y_left[n]   = (uint8)y;
        vis_trace_y_center[n] = (uint8)y;
        vis_trace_y_right[n]  = (uint8)y;

        vis_trace_x_left[n]   = (g_vis.l_border[y] != BORDER_INVALID)
                              ? g_vis.l_border[y] : (uint8)VIS_TRACE_NONE;
        vis_trace_x_right[n]  = (g_vis.r_border[y] != BORDER_INVALID)
                              ? g_vis.r_border[y] : (uint8)VIS_TRACE_NONE;
        vis_trace_x_center[n] = (g_vis.center_line[y] != BORDER_INVALID)
                              ? g_vis.center_line[y] : (uint8)VIS_TRACE_NONE;
        n++; // 写完一行的左右/中线共6个数组后，导出序号再加1
    }

    /* 行数不足时把剩余位置标记为无效，避免上位机画出脏数据 */
    for (; n < VIS_TRACE_ROWS; n++)
    {
        vis_trace_x_left[n]   = (uint8)VIS_TRACE_NONE;
        vis_trace_x_center[n] = (uint8)VIS_TRACE_NONE;
        vis_trace_x_right[n]  = (uint8)VIS_TRACE_NONE;
        vis_trace_y_left[n]   = (uint8)VIS_TRACE_NONE;
        vis_trace_y_center[n] = (uint8)VIS_TRACE_NONE;
        vis_trace_y_right[n]  = (uint8)VIS_TRACE_NONE;
    }
}

/* 与原始灰度、边界共用同一帧快照；在线改变前瞻后，绿线随下一帧实际取值更新。 */
static void vision_monitor_metadata(uint8 *destination)
{
    uint8 i;
    const uint8 corners[8] = {cross_up_l_col, cross_up_l_row, cross_up_r_col, cross_up_r_row,
                             cross_dn_l_col, cross_dn_l_row, cross_dn_r_col, cross_dn_r_row};
    memset(destination, BORDER_INVALID, VISION_MONITOR_METADATA_SIZE); // 先将12字节置255，未用坐标自然无效；mask/state接着单独置0
    destination[0] = vision_sample_far; // 第0字节取值远行；上位机用它画上方绿线
    destination[1] = vision_sample_near; // 第1字节取值近行；上位机用它画下方绿线
    destination[2] = destination[3] = 0; // 第2字节4拐点有效位图，第3字节无/候选/确认/保持状态
    if (!vision_corners_in_frame || !cross_corners_valid) return; // 没允许上传本帧锚点时，前瞻仍传，拐点mask保持0，避免绘旧点
    for (i = 0; i < 4u; i++) // 顺序左上、右上、左下、右下，每点占(x,y)两个字节
    {
        if (corners[2u*i] >= pho_w || corners[2u*i+1u] >= pho_h) continue; // 跳过无效255或越界角点，不让上位机误画到图像外
        destination[2] |= (uint8)(1u << i); // 把该角点对应bit置1；1u<<i依次为1/2/4/8，可多点同时有效
        destination[4u+2u*i] = corners[2u*i]; // 写该角点x，坐标区从第4字节开始
        destination[5u+2u*i] = corners[2u*i+1u]; // 写该角点y，与上一字节组成一对
    }
    if (destination[2]) destination[3] = cross_missed_count ? 3u : (uint8)cross_state; // 有有效角点时，漏检保持显示3；否则沿用候选1/确认2
}
