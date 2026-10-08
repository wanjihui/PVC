
#include "vision.h"
#include "config.h"


VisionResult_t g_vis;                        // 赛道识别结果（边界+中线+偏差）
static uint8 vision_gray[pho_h][pho_w];      // 当前已消费帧，算法与上传共用，防止叠线错帧
static uint8 vision_repair_flags[pho_h];
static uint32 vision_previous_start;
volatile uint32 vis_interval_us, vis_processed_frames, vis_last_event;

uint8 vision_capture_frame(void)
{
    uint32 csi_enabled = NVIC_GetEnableIRQ(CSI_IRQn);
    uint8 captured = 0;
    NVIC_DisableIRQ(CSI_IRQn);
    __DSB(); __ISB();
    /* CSI回调不回收当前完成缓冲；短暂复制期间PIT采样/按键中断仍运行。 */
    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        memcpy(vision_gray, mt9v03x_image, sizeof(vision_gray));
        captured = 1;
    }
    if (csi_enabled) NVIC_EnableIRQ(CSI_IRQn);
    return captured;
}

void vision_monitor_payload(uint8 *destination)
{
    uint16 y;
    memcpy(destination, vision_gray, sizeof(vision_gray));
    destination += sizeof(vision_gray);
    for (y = 0; y < pho_h; y++)
    {
        uint8 flags = vision_repair_flags[y];
        uint8 l = g_vis.l_border[y], r = g_vis.r_border[y], c = g_vis.center_line[y];
        if (l < pho_w && !g_vis.l_border_exist[y]) flags |= 1u;
        if (r < pho_w && !g_vis.r_border_exist[y]) flags |= 2u;
        *destination++ = l < pho_w ? l : BORDER_INVALID;
        *destination++ = r < pho_w ? r : BORDER_INVALID;
        *destination++ = c < pho_w ? c : BORDER_INVALID;
        *destination++ = flags;
    }
}

volatile uint8_t vis_frame_ready;          // 新帧处理完成（供上行传输层判断数据是否新鲜）
volatile int16  asc_range_dbg = 0;          // 诊断：ASC窗口中心行（=clamp(asc_far,20,100)，随遥测上传）
volatile uint8_t zebra_dbg_detected   = 0;  /* 诊断：本帧 zebra_detected（0/1） */
volatile uint8_t zebra_dbg_valid_rows  = 0;  /* 诊断：本帧斑马线有效采样行数（0~3） */
volatile uint8_t zebra_dbg_confirm     = 0;  /* 诊断：本帧斑马线确认计数（0~2） */

/* 分段耗时诊断（us），供 VOFA/调参定位视觉瓶颈 */
volatile uint32   vis_time_us      = 0;    /* vis_deal 单帧总耗时(us)，GPT1 微秒计时 */
volatile uint16_t vis_t_bin_us     = 0;
volatile uint16_t vis_t_filter_us  = 0;
volatile uint16_t vis_t_sweep_us   = 0;
volatile uint16_t vis_t_cross_us   = 0;
volatile uint16_t vis_t_center_us  = 0;

 uint8 asc_far = VIS_PREVIEW_ROW_DEFAULT;             // ASC采样远行（顶部行），菜单可调，默认55（Round34）
static int16 last_valid_width = 60;            // 最近双边真实路宽（逐行扫描外推基准）
static int   last_real_width  = 60;            // 近一次双边真实宽度（单边外推基准，发车时清零）
static uint8 raw_l_border[pho_h];              // 十字补线前的原始左边界（标定用）
static uint8 raw_r_border[pho_h];              // 十字补线前的原始右边界（标定用）
static uint8 raw_l_exist[pho_h];               // 十字补线前的原始左边界 real 标志（0/1）
static uint8 raw_r_exist[pho_h];               // 十字补线前的原始右边界 real 标志（0/1）
#define SWEEP_OFFSET  8                          // 上行搜索起点向内侧偏移量（参考工程值）

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
#define CROSS_OBL_MIN_ROWS             8    /* 斜入十字：错边行数 ≥8 行才判定 */
#define CROSS_OBL_REF_MARGIN           3    /* 斜入判据：错边必须越过参考中线 3px 以上 */
#define CROSS_OBL_JUMP_COL            15    /* 斜入拐点：错边 1~2 行内跳变 >15px */
#define CROSS_OBL_ROW_MAX             60    /* 斜入检测只扫 y=20..60 的远/中区域 */
#define CROSS_OBL_DN_GAP               8    /* 斜入上拐点向下至少 8 行再找下拐点 */
#define CROSS_CONFIRM_FRAMES           2    /* 连续确认帧数 */
#define CROSS_HOLD_MISSED_FRAMES       2    /* 漏检保持帧数 */

/* 斜入耦合扫线（手动重构版） */
#define CROSS_USE_OBLIQUE_SWEEP        1
#define CROSS_OBLIQUE_LOST_MIN         7u
#define CROSS_OBLIQUE_CONFIRM_ROWS     3u
#define CROSS_OBLIQUE_LOST_SIDE_MAX_REAL_ROWS 10u  /* 斜入段丢线侧允许的真实边界行数 */
/* 斜入单边丢重扫起点：左墙有角(右丢)用168，右墙有角(左丢)用20 */
#define CROSS_OBLIQUE_RESET_COL_LEFT_REAL   168u
#define CROSS_OBLIQUE_RESET_COL_RIGHT_REAL  20u
#define CROSS_OBLIQUE_RESET_COL(real_side) \
    ((real_side) == 1 ? CROSS_OBLIQUE_RESET_COL_LEFT_REAL \
                      : CROSS_OBLIQUE_RESET_COL_RIGHT_REAL)
#define CROSS_EDGE_LOST_L              (-12)
#define CROSS_EDGE_LOST_R              200

/* 标定阈值（供斜入撕裂辅助函数使用；放宽版：下拐点更容易触发） */
#define CROSS_STABLE                   10
#define CROSS_JUMP1                    1
#define CROSS_JUMP2                    2

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
static cross_state_enum cross_state = CROSS_STATE_NONE;
static bool   cross_corners_valid;
static uint8  cross_confirm_count;
static uint8  cross_missed_count;

/* sweep 写入的候选（每帧开头复位）*/
static bool  cross_cand_present;
static uint8 cross_near_row;             /* 下行号候选（近侧，行大）*/
static uint8 cross_far_row;              /* 上行号候选（远侧，行小）*/

/* cross_detect 算出的四拐点 */
static uint8 cross_up_l_col, cross_up_l_row;   /* 左上(远) */
static uint8 cross_up_r_col, cross_up_r_row;   /* 右上(远) */
static uint8 cross_dn_l_col, cross_dn_l_row;   /* 左下(近) */
static uint8 cross_dn_r_col, cross_dn_r_row;   /* 右下(近) */

#if CROSS_USE_OBLIQUE_SWEEP
/* 斜入状态（手动重构版） */
typedef enum {
    OBLIQUE_IDLE = 0,
    OBLIQUE_LOWER_SEARCH,
    OBLIQUE_UPPER_SEARCH
} ObliqueState;

static ObliqueState oblique_state;
static uint8 oblique_lost_side;          /* 1=左丢，2=右丢 */
static uint8 oblique_real_side;          /* 1=左墙有角，2=右墙有角；触发后固定 */
static uint8 oblique_lost_run;
static uint8 oblique_dn_row, oblique_dn_col;
static uint8 oblique_up_row, oblique_up_col;
static uint8 oblique_pending_up_row, oblique_pending_up_col;
static uint8 oblique_pending_confirm;
static bool  oblique_valid;
static bool  cross_oblique_used;         /* 本帧 cross_detect 是否实际采用了斜入 */
#endif /* CROSS_USE_OBLIQUE_SWEEP */

/* 斑马线静态变量 */
static bool   zebra_detected;
static uint8  zebra_missed_count;
static uint8  zebra_confirm_count;      /* 连续有效帧计数（发车时清零） */
static bool   zebra_prev;               /* 上一帧检测状态 */
static uint8  oob_track_l = pho_w / 4;  /* OOB赛道窗口保持值：上一帧底边界失效时沿用 */
static uint8  oob_track_r = pho_w * 3 / 4;
static uint8  zebra_track_l = pho_w / 4; /* 斑马线赛道窗口保持值：上一帧底边界失效时沿用（发车时清零） */
static uint8  zebra_track_r = pho_w * 3 / 4;

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
    unsigned long hist[256] = {0};
    /* 全屏统计（行0~119），下采样4x4抗干扰 */
    for (int y = 0; y < pho_h; y += 4)
        for (int x = 0; x < pho_w; x += 4)
            hist[vision_gray[y][x]]++;

    unsigned long total = (unsigned long)((pho_h + 3) / 4) * ((pho_w + 3) / 4);
    if (total == 0) return -1;

    unsigned long sum_all = 0;
    for (int i = 0; i < 256; i++) sum_all += (unsigned long)i * hist[i];

    unsigned long wB = 0, sumB = 0;
    float max_between = -1.0f;
    int best_th = 0;

    for (int t = 0; t < 256; t++)
    {
        wB += hist[t];
        if (wB == 0) continue;
        unsigned long wF = total - wB;
        if (wF == 0) break;
        sumB += (unsigned long)t * hist[t];
        float mB = (float)sumB / wB;
        float mF = (float)(sum_all - sumB) / wF;
        float diff = mB - mF;
        float between = (float)wB * (float)wF * diff * diff;
        if (between > max_between) { max_between = between; best_th = t; }
    }
    if (best_th < 20)  best_th = 20;
    if (best_th > 235) best_th = 235;
    return best_th;
}



/* ================================================================
 * 边界3点滑动平均：消除单行噪声，保持趋势
 * 只平滑非零连续段，跳过缺失行
 * ================================================================ */
static void border_smooth(border_line border)
{
    for (int y = 1; y < pho_h - 1; y++)
    {
        uint8 l = border[y - 1];
        uint8 m = border[y];
        uint8 r = border[y + 1];
        if (l == BORDER_INVALID || m == BORDER_INVALID || r == BORDER_INVALID) continue;

        int sum = (int)l + (int)m + (int)r;
        uint8 v = (uint8)((sum + 1) / 3);  // +1 四舍五入

        // 变化超过 8px → 不信任此次平滑，保留原值
        int diff = (int)v - (int)m;
        if (diff < 0) diff = -diff;
        if (diff > 8) continue;  // 跳变太大，跳过不改

        border[y] = v;
    }
}

/* ================================================================
 * pho_center — 中线生成 + 偏差计算
 *
 * 1. border_smooth(g_vis.l_border/g_vis.r_border) — 平滑左右边界
 * 2. g_vis.center_line[]: 双边→中点(记录宽度), 单边→真实路宽外推, 都缺→白像素重心
 * 3. border_smooth(g_vis.center_line) — 平滑中线
 * 4. 窄窗口加权平均: asc_far±10, 权重1.0→5.0递增(远重近轻), 均值=err
 *
 * 调参: asc_far(预瞄中心行,菜单"preview",默认65); 单边→最新真实路宽外推
 * ================================================================ */

/* ================================================================
 * longest_white_col — 最长白列（正确版）
 *
 * 中心区域按列采样，统计每列"从底行向上的连续白像素高度"，
 * 高度最大的列 = 赛道中心。等价于参考工程 otsu_longest_white_col()。
 * 纵向连通性度量对十字横道/白路肩/反光/相邻车道天然免疫。
 * ================================================================ */
#define LONGEST_COL_SAMPLE_STEP  5

static int longest_white_col(void)
{
    int best_col = pho_center_x;
    int best_len = 0;

    for (int x = pho_w / 4; x <= pho_w * 3 / 4; x += LONGEST_COL_SAMPLE_STEP)
    {
        int len = 0;
        for (int y = pho_h - 1; y >= 0; y--)
        {
            if (Image_Used[y][x] == 255) len++;
            else break;                 /* 遇到黑像素，该列白段到顶 */
        }
        if (len > best_len) { best_len = len; best_col = x; }
    }
    return best_col;
}

#if CROSS_USE_OBLIQUE_SWEEP
/* ================================================================
 * 斜入辅助 — 边界哨兵读取
 * ================================================================ */
static int16 cross_edge_l(uint8 y)
{
    /* 斜入判据必须使用“本帧真实扫到的原始边界”。
     * 当前行在 sweep 中先保存 raw 再执行状态机，g_vis 此时还未写入当前行，
     * 若读 g_vis 会把当前行当成丢线，导致上拐点永远找不到。 */
    return (raw_l_exist[y] && raw_l_border[y] != BORDER_INVALID)
         ? (int16)raw_l_border[y] : CROSS_EDGE_LOST_L;
}

static int16 cross_edge_r(uint8 y)
{
    return (raw_r_exist[y] && raw_r_border[y] != BORDER_INVALID)
         ? (int16)raw_r_border[y] : CROSS_EDGE_LOST_R;
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
        int16 v0 = cross_edge_l(y);
        if (v0 == CROSS_EDGE_LOST_L) return false;
        /* 上侧三行必须是已经真实扫到的边界，不能被未扫行的哨兵误判成“撕裂” */
        if (cross_edge_l(y-1) == CROSS_EDGE_LOST_L ||
            cross_edge_l(y-2) == CROSS_EDGE_LOST_L ||
            cross_edge_l(y-3) == CROSS_EDGE_LOST_L) return false;
        if (CROSS_ABS(cross_edge_l(y+1) - cross_edge_l(y+2)) <= CROSS_STABLE &&
            CROSS_ABS(cross_edge_l(y+2) - cross_edge_l(y+3)) <= CROSS_STABLE &&
            CROSS_ABS(cross_edge_l(y+3) - cross_edge_l(y+4)) <= CROSS_STABLE &&
            v0 - cross_edge_l(y-1) >= CROSS_JUMP1 &&
            v0 - cross_edge_l(y-2) >= CROSS_JUMP2 &&
            v0 - cross_edge_l(y-3) >= CROSS_JUMP2)
        {
            *dn_row = y;
            *dn_col = (uint8)v0;
            return true;
        }
    }
    else  /* 右墙 RD */
    {
        int16 v0 = cross_edge_r(y);
        if (v0 == CROSS_EDGE_LOST_R) return false;
        if (cross_edge_r(y-1) == CROSS_EDGE_LOST_R ||
            cross_edge_r(y-2) == CROSS_EDGE_LOST_R ||
            cross_edge_r(y-3) == CROSS_EDGE_LOST_R) return false;
        if (CROSS_ABS(cross_edge_r(y+1) - cross_edge_r(y+2)) <= CROSS_STABLE &&
            CROSS_ABS(cross_edge_r(y+2) - cross_edge_r(y+3)) <= CROSS_STABLE &&
            CROSS_ABS(cross_edge_r(y+3) - cross_edge_r(y+4)) <= CROSS_STABLE &&
            v0 - cross_edge_r(y-1) <= -CROSS_JUMP1 &&
            v0 - cross_edge_r(y-2) <= -CROSS_JUMP2 &&
            v0 - cross_edge_r(y-3) <= -CROSS_JUMP2)
        {
            *dn_row = y;
            *dn_col = (uint8)v0;
            return true;
        }
    }
    return false;
}

/* ================================================================
 * 斜入辅助 — 找有角侧下拐点（从底部向上扫到 current_y）
 * ================================================================ */
static bool cross_find_oblique_dn(uint8 real_side, uint8 current_y,
                                  uint8 *dn_row, uint8 *dn_col)
{
    for (uint8 y = pho_h - 1; y >= current_y; y--)
    {
        if (cross_oblique_dn_at(real_side, y, dn_row, dn_col))
            return true;
    }
    return false;
}

/* 上拐点撕裂触发：只使用当前行和已扫过的下方行 */
static bool cross_oblique_up_tear(uint8 real_side, uint8 y)
{
    /* 越界保护：需要 y+1..y+3 */
    if (y + 3 >= pho_h) return false;

    if (real_side == 1)  /* 左墙 LU：当前行 > 下一行 */
    {
        int16 v0 = cross_edge_l(y);
        if (v0 == CROSS_EDGE_LOST_L) return false;
        return (v0 - cross_edge_l(y+1) >= CROSS_JUMP1 &&
                v0 - cross_edge_l(y+2) >= CROSS_JUMP2 &&
                v0 - cross_edge_l(y+3) >= CROSS_JUMP2);
    }
    else  /* 右墙 RU：当前行 < 下一行 */
    {
        int16 v0 = cross_edge_r(y);
        if (v0 == CROSS_EDGE_LOST_R) return false;
        return (v0 - cross_edge_r(y+1) <= -CROSS_JUMP1 &&
                v0 - cross_edge_r(y+2) <= -CROSS_JUMP2 &&
                v0 - cross_edge_r(y+3) <= -CROSS_JUMP2);
    }
}

/* 上拐点候选确认：已向上扫过若干行后检查上方稳定 */
static bool cross_oblique_up_stable(uint8 real_side,
                                    uint8 up_row, uint8 current_y)
{
    /* 至少需要向上扫过 CROSS_OBLIQUE_CONFIRM_ROWS 行 */
    int urow = (int)up_row;
    if (urow < (int)CROSS_OBLIQUE_CONFIRM_ROWS
        || (int)current_y > urow - (int)CROSS_OBLIQUE_CONFIRM_ROWS) return false;

    if (real_side == 1)
    {
        if (CROSS_ABS(cross_edge_l((uint8)(urow-1)) - cross_edge_l((uint8)(urow-2))) > CROSS_STABLE)
            return false;
        if ((int)current_y <= urow - (int)CROSS_OBLIQUE_CONFIRM_ROWS &&
            CROSS_ABS(cross_edge_l((uint8)(urow-2)) - cross_edge_l((uint8)(urow-3))) > CROSS_STABLE)
            return false;
        if ((int)current_y <= urow - (int)CROSS_OBLIQUE_CONFIRM_ROWS - 1 &&
            CROSS_ABS(cross_edge_l((uint8)(urow-3)) - cross_edge_l((uint8)(urow-4))) > CROSS_STABLE)
            return false;
        return true;
    }
    else
    {
        if (CROSS_ABS(cross_edge_r((uint8)(urow-1)) - cross_edge_r((uint8)(urow-2))) > CROSS_STABLE)
            return false;
        if ((int)current_y <= urow - (int)CROSS_OBLIQUE_CONFIRM_ROWS &&
            CROSS_ABS(cross_edge_r((uint8)(urow-2)) - cross_edge_r((uint8)(urow-3))) > CROSS_STABLE)
            return false;
        if ((int)current_y <= urow - (int)CROSS_OBLIQUE_CONFIRM_ROWS - 1 &&
            CROSS_ABS(cross_edge_r((uint8)(urow-3)) - cross_edge_r((uint8)(urow-4))) > CROSS_STABLE)
            return false;
        return true;
    }
}

/* 丢线侧连续性校验：斜入上下拐点之间，丢线侧不应出现大量真实边界。
 * 若出现过多 real 行，说明只是弯道单边丢失，不是斜入十字。 */
static bool cross_oblique_lost_side_continuous(uint8 real_side,
                                               uint8 dn_row, uint8 up_row)
{
    uint8 lost_side = (real_side == 1) ? 2 : 1;
    int   real_cnt  = 0;

    for (int y = (int)dn_row; y >= (int)up_row; y--)
    {
        bool real = (lost_side == 1)
                  ? (raw_l_exist[y] != 0)
                  : (raw_r_exist[y] != 0);
        if (real)
        {
            real_cnt++;
            if (real_cnt > (int)CROSS_OBLIQUE_LOST_SIDE_MAX_REAL_ROWS)
                return false;
        }
    }
    return true;
}
#endif /* CROSS_USE_OBLIQUE_SWEEP */

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
static int sweep_boundaries(void)
{
    cross_cand_present = false;          /* 每帧复位候选（必须在任何 return 之前）*/

#if CROSS_USE_OBLIQUE_SWEEP
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
#endif

    /* 原始判据数组先全部清空，sweep 只填真正扫到的行 */
    for (int y = 0; y < pho_h; y++)
    {
        raw_l_border[y] = BORDER_INVALID;
        raw_r_border[y] = BORDER_INVALID;
        raw_l_exist[y]  = 0;
        raw_r_exist[y]  = 0;
    }

    /* ---- 最长白列作种子（替代"底行最长白段"）---- */
    int y_bot = pho_h - 1;
    int seed  = longest_white_col();

    /* ---- 底行：从种子列向左右扩白(白→黑黑) ---- */
    int best_s = seed;
    while (best_s > 2
        && !(Image_Used[y_bot][best_s - 1] == 0 && Image_Used[y_bot][best_s - 2] == 0))
        best_s--;
    int best_e = seed;
    while (best_e < pho_w_max - 2
        && !(Image_Used[y_bot][best_e + 1] == 0 && Image_Used[y_bot][best_e + 2] == 0))
        best_e++;

    if (best_e - best_s < 3) return 1;          /* 底行无赛道 */

    int prev_l = best_s;
    int prev_r = best_e;
    int l_exist = (best_s > 2);                 /* 贴边(≤2)视为伪边界 */
    int r_exist = (best_e < pho_w_max - 2);

    last_valid_width = best_e - best_s;
    if (last_valid_width < 20)  last_valid_width = 20;
    if (last_valid_width > 160) last_valid_width = 160;

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
    int consec_lost = 0;
    uint8 white_consec = 0;              /* 连续白区行计数 */
    uint8 near_row = 0;                  /* 近侧候选行 */
    bool  force_seed_after_cross = false; /* 确认白区十字后，强制从 seed 重找双边，禁止单边外推 */
#if CROSS_USE_OBLIQUE_SWEEP
    bool oblique_dn_initial_searched = false; /* 斜入下拐点是否做过一次全量搜索 */
#endif

    for (int y = y_bot - 1; y >= 5; y--)
    {
        int new_l = BORDER_INVALID, new_r = BORDER_INVALID;
        int l_real = 0, r_real = 0;
        int x;

        /* --- 左边界: 起点=prev_l+SWEEP_OFFSET，向左走到"白→黑黑" --- */
        x = force_seed_after_cross ? seed : (prev_l + SWEEP_OFFSET);
        if (x > pho_w_max - 2) x = pho_w_max - 2;
        if (Image_Used[y][x] == 255)
        {
            while (x > 2
                && !(Image_Used[y][x - 1] == 0 && Image_Used[y][x - 2] == 0))
                x--;
        }
        else
        {
            /* 起点失效(上一行边界已过时) → 回退到种子列重走 */
            x = seed;
            while (x > 2
                && !(Image_Used[y][x - 1] == 0 && Image_Used[y][x - 2] == 0))
                x--;
        }
        if (x > 2 && Image_Used[y][x] == 255) { new_l = x; l_real = 1; }

        /* --- 右边界: 起点=prev_r-SWEEP_OFFSET，向右走到"白→黑黑" --- */
        x = force_seed_after_cross ? seed : (prev_r - SWEEP_OFFSET);
        if (x < 2) x = 2;
        if (Image_Used[y][x] == 255)
        {
            while (x < pho_w_max - 2
                && !(Image_Used[y][x + 1] == 0 && Image_Used[y][x + 2] == 0))
                x++;
        }
        else
        {
            x = seed;
            while (x < pho_w_max - 2
                && !(Image_Used[y][x + 1] == 0 && Image_Used[y][x + 2] == 0))
                x++;
        }
        if (x < pho_w_max - 2 && Image_Used[y][x] == 255) { new_r = x; r_real = 1; }

        /* 保存“真正扫到的原始边界”，供串口判据使用（未被外推/标空污染） */
        raw_l_border[y] = l_real ? (uint8)new_l : BORDER_INVALID;
        raw_r_border[y] = r_real ? (uint8)new_r : BORDER_INVALID;
        raw_l_exist[y]  = (uint8)l_real;
        raw_r_exist[y]  = (uint8)r_real;

        /* --- 白区出口：本行有效(至少一边找到)、上一行仍在白区 → 提交候选 --- */
        if (white_consec > 0 && (l_real || r_real))
        {
            if (white_consec >= CROSS_WHITE_ZONE_MIN
                && (uint8)y >= CROSS_WHITE_ZONE_FAR_ROW_MIN)
            {
                cross_cand_present = true;
                cross_near_row = near_row;
                cross_far_row  = (uint8)y;
                force_seed_after_cross = true;   /* 横道后强制从 seed 重找，禁止单边外推 */
            }
            white_consec = 0;
        }

#if CROSS_USE_OBLIQUE_SWEEP
        /* ===== 斜入状态机 ===== */

        /* 正入候选出现时保留斜入作为兜底；cross_detect 仍正入优先，
         * 正入失败后才使用斜入。 */
        {
            /* 只有还没找到下拐点时才需要计数 */
            if (oblique_state == OBLIQUE_IDLE)
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
                    uint8 real_side = (oblique_lost_side == 1) ? 2 : 1;
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
                        uint8 cand = (uint8)((int)y + 3);
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

                if (oblique_pending_confirm == 0
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

                            /* 正入存在时不在扫线过程中标空，避免破坏正入候选；
                             * 正入失败后 cross_detect 直接用斜入拐点补线。 */
                            if (!cross_cand_present)
                            {
                                /* 统一标空：下拐点行 → 上拐点行（含两端） */
                                for (int yy = (int)oblique_dn_row; yy >= (int)oblique_up_row; yy--) {
                                    g_vis.l_border[yy]     = BORDER_INVALID;
                                    g_vis.r_border[yy]     = BORDER_INVALID;
                                    g_vis.l_border_exist[yy] = 0;
                                    g_vis.r_border_exist[yy] = 0;
                                }
                            }
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

                /* 扫到顶仍无上拐点 → 兜底并统一标空 */
                if (oblique_state == OBLIQUE_UPPER_SEARCH && y == 5)
                {
                    if (oblique_real_side == 1) {   /* 左墙有角 */
                        oblique_up_row = 0;
                        oblique_up_col = 187;
                    } else {                        /* 右墙有角 */
                        oblique_up_row = 0;
                        oblique_up_col = 0;
                    }
                    oblique_valid = true;
                    oblique_state = OBLIQUE_IDLE;

                    if (!cross_cand_present)
                    {
                        /* 统一标空：下拐点行 → 兜底上拐点行（含当前 y=5） */
                        for (int yy = (int)oblique_dn_row; yy >= (int)oblique_up_row; yy--) {
                            g_vis.l_border[yy]     = BORDER_INVALID;
                            g_vis.r_border[yy]     = BORDER_INVALID;
                            g_vis.l_border_exist[yy] = 0;
                            g_vis.r_border_exist[yy] = 0;
                        }
                    }
                    continue;
                }
            }
        }
#endif

        /* --- 处理边界状态 --- */
        if (l_real && r_real && new_l < new_r)
        {
            /* 双边真 → 更新路宽，并解除“十字后强制 seed” */
            force_seed_after_cross = false;
            last_valid_width = new_r - new_l;
            if (last_valid_width < 20)  last_valid_width = 20;
            if (last_valid_width > 160) last_valid_width = 160;
            consec_lost = 0;
        }
        else if (l_real && !r_real)
        {
            if (force_seed_after_cross)
            {
                /* 十字后未恢复双边：禁止单边外推，标空并从 seed 继续 */
                g_vis.l_border[y] = BORDER_INVALID;
                g_vis.r_border[y] = BORDER_INVALID;
                g_vis.l_border_exist[y] = 0;
                g_vis.r_border_exist[y] = 0;
                prev_l = seed;
                prev_r = seed;
                continue;
            }
            /* 只丢右边 → 真实路宽外推（不再拉满屏）*/
            new_r = new_l + last_valid_width;
            if (new_r > pho_w_max) new_r = pho_w_max;
            consec_lost = 0;
        }
        else if (!l_real && r_real)
        {
            if (force_seed_after_cross)
            {
                /* 十字后未恢复双边：禁止单边外推，标空并从 seed 继续 */
                g_vis.l_border[y] = BORDER_INVALID;
                g_vis.r_border[y] = BORDER_INVALID;
                g_vis.l_border_exist[y] = 0;
                g_vis.r_border_exist[y] = 0;
                prev_l = seed;
                prev_r = seed;
                continue;
            }
            /* 只丢左边 → 真实路宽外推 */
            new_l = new_r - last_valid_width;
            if (new_l < 0) new_l = 0;
            consec_lost = 0;
        }
        else
        {
            /* 双边都丢：区分黑区(真丢线) / 白区(十字横道) */
            int is_black = 1;
            for (int cx = pho_w / 4; cx <= pho_w * 3 / 4; cx += LONGEST_COL_SAMPLE_STEP)
                if (Image_Used[y][cx] == 255) { is_black = 0; break; }

            if (is_black)
            {
                white_consec = 0;      /* 黑区打断白区连续性 */
                if (++consec_lost >= 5)
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
                white_consec++;
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
#if CROSS_USE_OBLIQUE_SWEEP
            if (oblique_state == OBLIQUE_UPPER_SEARCH && !cross_cand_present)
            {
                /* 斜入上拐点搜索期间：不回到 seed，按有角侧固定重扫列；
                 * 左墙有角(右丢)→168，右墙有角(左丢)→20 */
                prev_l = CROSS_OBLIQUE_RESET_COL(oblique_real_side);
                prev_r = CROSS_OBLIQUE_RESET_COL(oblique_real_side);
            }
            else
#endif
            {
                prev_l = seed;             /* 下一行搜索起点重置回种子列 */
                prev_r = seed;
            }
            continue;
        }

        /* 写入本行 */
        g_vis.l_border[y] = (uint8)new_l;
        g_vis.r_border[y] = (uint8)new_r;
        g_vis.l_border_exist[y] = (uint8)l_real;
        g_vis.r_border_exist[y] = (uint8)r_real;

#if CROSS_USE_OBLIQUE_SWEEP
        /* 斜入上拐点搜索期间：下一行强制从有角侧对应的固定列重扫。
         * 左墙有角(右丢)→168，右墙有角(左丢)→20，不再回 seed。
         * 正入候选存在时仍以正入的 seed 逻辑优先，斜入仅作兜底。 */
        if (oblique_state == OBLIQUE_UPPER_SEARCH && !cross_cand_present)
        {
            prev_l = CROSS_OBLIQUE_RESET_COL(oblique_real_side);
            prev_r = CROSS_OBLIQUE_RESET_COL(oblique_real_side);
        }
        else
#endif
        {
            prev_l = new_l;
            prev_r = new_r;
        }
    }

    return 0;
}



static void pho_center(void)
{
    // ---- 平滑边界 ----
    border_smooth(g_vis.l_border);
    border_smooth(g_vis.r_border);

    // ---- 生成 g_vis.center_line[]（轨迹导出 + err 共用） ----
    for (int y = pho_h - 1; y >= 0; y--)
    {
        int l = (int)g_vis.l_border[y];
        int r = (int)g_vis.r_border[y];
        int c;

        /* 双边真 → 中点（记录宽度为外推基准）；
         * 双边有值但至少一侧是估的（单边丢线）→ 同样取中点。
         *     sweep_boundaries 已经把丢失侧用 last_valid_width 外推并写入边界数组，
         *     TRACK 显示的两条黄线就是这两个值，所以中线必须用 (l+r)/2 才能落在黄线中央；
         *     若仍用 last_real_width（上一帧/上方行的旧宽度）外推，会与黄线不一致。
         * 只有一侧有值 → 最近真实路宽外推；都无 → 白像素重心 */
        if (l != BORDER_INVALID && r != BORDER_INVALID)
        {
            c = (l + r) / 2;
            if (g_vis.l_border_exist[y] && g_vis.r_border_exist[y])
            {
                last_real_width = r - l;
                if (last_real_width < 20)  last_real_width = 20;
                if (last_real_width > 160) last_real_width = 160;
            }
        }
        else if (l != BORDER_INVALID)
        {
            c = l + last_real_width / 2;
        }
        else if (r != BORDER_INVALID)
        {
            c = r - last_real_width / 2;
        }
        else
        {
            /* 双边都缺 → 白像素重心。找不到就是真没赛道，标 INVALID 不画绿线。
             * 底部连通滤波已清除环境白，白像素重心安全可靠。 */
            int sum = 0, cnt = 0;
            for (int x = 0; x < pho_w; x++)
            {
                if (Image_Used[y][x] == 255)
                { sum += x; cnt++; }
            }
            if (cnt >= 3)
                c = (sum + cnt / 2) / cnt;
            else
                c = BORDER_INVALID;
        }

        if (c == BORDER_INVALID)
        {
            g_vis.center_line[y] = BORDER_INVALID;
            continue;
        }

        if (c < pho_w_min) c = pho_w_min;
        if (c > pho_w_max) c = pho_w_max;
        g_vis.center_line[y] = (uint8)c;
    }

    // ---- 平滑中线（消除半宽偏移可能引入的不连续）----
    border_smooth(g_vis.center_line);

    // ---- 固定前瞻中心行 ±10，加权计算中线偏差 ----
#define WINDOW_HALF 10
    int y_center = (int)asc_far;
    if (y_center < 20)  y_center = 20;
    if (y_center > 100) y_center = 100;
    int y_near = y_center + WINDOW_HALF;   // e.g. 50+10=60
    int y_far  = y_center - WINDOW_HALF;   // e.g. 50-10=40
    asc_range_dbg = y_center;

    float total_dev = 0.0f, total_w = 0.0f;
    float w = 1.0f;
    float w_step = 4.0f / (float)(y_near - y_far);   // Round33: 远行权重 1→5，Omega中部大偏差不被近行平均掉
    int   valid_cnt = 0;

    for (int y = y_near; y >= y_far; y--)
    {
        uint8 c = g_vis.center_line[y];
        if (c != BORDER_INVALID && c >= 3 && c <= pho_w_max - 3)
        {
            total_dev += ((float)c - (float)pho_center_x) * w;
            total_w   += w;
            valid_cnt++;
        }
        w += w_step;
    }

    int theo_rows = y_near - y_far;
    if (total_w > 0.0f)
    {
        g_vis.err = total_dev / total_w;
        if (valid_cnt > 0 && valid_cnt < theo_rows)
        {
            float scale = (float)theo_rows / (float)valid_cnt;
            if (scale > 2.5f) scale = 2.5f;
            g_vis.err *= scale;
        }
    }
    // else: total_w==0 → err保留上帧不变
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
    return (l != BORDER_INVALID && r != BORDER_INVALID && l < r);
}

/* 下拐点行查找（用户既定逻辑）：
 * 从 near+5 开始向下找第一行“左右边界都真实存在(exist=1)且 l<r”的行；
 * 找到且行号 ≤115 就用该行左右边界；
 * 一路找不到（例如右下拐点出画面）到 115 都没找到 → 进入 snap：dn=119, L=0, R=187。 */
static bool cross_find_dn_both_exist(uint8 start, uint8 *row)
{
    for (uint8 y = start; y <= CROSS_DN_SNAP_ROW; y++)
    {
        if (g_vis.l_border_exist[y]
         && g_vis.r_border_exist[y]
         && g_vis.l_border[y] != BORDER_INVALID
         && g_vis.r_border[y] != BORDER_INVALID
         && g_vis.l_border[y] < g_vis.r_border[y])
        {
            *row = y;
            return true;
        }
    }
    return false;
}

/* 上拐点稳定判据：y 和 y-3 两行（y-3 离横道更远）左右边界列号变化都 ≤ 8。
 * 与下拐点对称，防止 far_row 上缘同样存在逐行外扩/收缩的过渡行。 */
static bool cross_anchor_up_stable(uint8 y)
{
    if (y < (uint8)(5 + CROSS_ANCHOR_STABLE_GAP)) return false;  /* y-3 必须 ≥5（扫线下限）*/
    uint8 y_ref = (uint8)(y - CROSS_ANCHOR_STABLE_GAP);
    if (!cross_anchor_row_valid(y) || !cross_anchor_row_valid(y_ref)) return false;

    int16 dl = (int16)g_vis.l_border[y_ref] - (int16)g_vis.l_border[y];
    int16 dr = (int16)g_vis.r_border[y_ref] - (int16)g_vis.r_border[y];
    if (dl < 0) dl = -dl;
    if (dr < 0) dr = -dr;
    return (dl <= CROSS_ANCHOR_STABLE_COL && dr <= CROSS_ANCHOR_STABLE_COL);
}

/* 从起始行向上找第一个稳定锚点行；找不到返回起始行 */
static uint8 cross_find_up_stable(uint8 start)
{
    uint8 row = start;
    uint8 minv = (start >= (uint8)(5 + CROSS_ANCHOR_SEARCH_MAX))
               ? (uint8)(start - CROSS_ANCHOR_SEARCH_MAX) : 5;
    for (uint8 y = start; y >= minv; y--)
        if (cross_anchor_up_stable(y)) { row = y; break; }
    return row;
}

/* 四拐点“稳定段+突变段”校验（借用 CSDN qq_53239103 的思路）：
 * up/dn 拐点之间必须存在 INVALID 行（横道断带）或 ≥6px 的边界跳变，
 * 否则认为只是弯道边沿渐变，不是十字拐点。 */
static bool cross_corner_has_band_gap(const uint8 line[pho_h], uint8 up_row, uint8 dn_row)
{
    bool  has_invalid = false;
    bool  have_prev   = false;
    uint8 prev        = BORDER_INVALID;
    int16 max_jump    = 0;

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
    return (has_invalid || max_jump >= CROSS_CORNER_JUMP_COL);
}

#if !CROSS_USE_OBLIQUE_SWEEP
/* 参考中线：优先上一帧 g_vis.center_line，缺省时用本行双边中点 */
static bool cross_get_ref_center(uint8 y, uint8 *ref)
{
    uint8 c = g_vis.center_line[y];
    if (c != BORDER_INVALID && c > 3) { *ref = c; return true; }

    uint8 l = g_vis.l_border[y];
    uint8 r = g_vis.r_border[y];
    if (l != BORDER_INVALID && r != BORDER_INVALID && l < r)
    {
        *ref = (uint8)(((int)l + (int)r) / 2);
        return true;
    }
    return false;
}

/* 斜入十字检测（CSDN qq_53239103 的“错边检测 + 换边”适配版）。
 * 正常白区候选失败后调用；只在远/中区域出现大量“边界跑到中线另一侧”
 * 并且该错边在 1~2 行内跳变 >15px 时才生成斜入候选。
 * 左右上拐点允许不在同一行（cross_repair 本来就分别存 up_l_row/up_r_row）。 */
static bool cross_detect_oblique(uint8 *up_l, uint8 *up_l_row,
                                 uint8 *up_r, uint8 *up_r_row,
                                 uint8 *dn_l, uint8 *dn_r, uint8 *dn_row)
{
    uint8 ref_bot = pho_center_x;
    for (uint8 y = pho_h - 1; y >= 80; y--)
    {
        if (cross_get_ref_center(y, &ref_bot)) break;
    }

    int left_wrong  = 0;
    int right_wrong = 0;

    for (uint8 y = 20; y <= CROSS_OBL_ROW_MAX; y++)
    {
        uint8 ref = ref_bot;
        (void)cross_get_ref_center(y, &ref);

        if (g_vis.r_border[y] != BORDER_INVALID
         && (int16)g_vis.r_border[y] + CROSS_OBL_REF_MARGIN < (int16)ref)
            left_wrong++;

        if (g_vis.l_border[y] != BORDER_INVALID
         && (int16)g_vis.l_border[y] > (int16)ref + CROSS_OBL_REF_MARGIN)
            right_wrong++;
    }

    if (left_wrong >= CROSS_OBL_MIN_ROWS && left_wrong > right_wrong)
    {
        /* 左斜入：上方“右边界”实际是左墙；在它向右跳变处恢复成左墙 */
        for (uint8 y = 22; y <= CROSS_OBL_ROW_MAX; y++)
        {
            uint8 r0 = g_vis.r_border[y];
            uint8 r1 = g_vis.r_border[y - 1];
            uint8 r2 = g_vis.r_border[y - 2];
            if (r0 == BORDER_INVALID || r1 == BORDER_INVALID || r2 == BORDER_INVALID) continue;
            if ((int16)r0 - (int16)r1 > CROSS_OBL_JUMP_COL
             && (int16)r0 - (int16)r2 > CROSS_OBL_JUMP_COL)
            {
                *up_l = r0;        /* 跳变后的“右边界”就是真正的左上拐点 */
                *up_l_row = y;

                /* 真正的右墙：从跳变行向下找第一个回到中线右侧的右边界 */
                uint8 y_r = y;
                uint8 r_ok = BORDER_INVALID;
                for (uint8 y2 = y; y2 < pho_h; y2++)
                {
                    uint8 ref2 = ref_bot;
                    (void)cross_get_ref_center(y2, &ref2);
                    if (g_vis.r_border[y2] != BORDER_INVALID
                     && (int16)g_vis.r_border[y2] > (int16)ref2 + CROSS_OBL_REF_MARGIN)
                    {
                        r_ok = g_vis.r_border[y2];
                        y_r = y2;
                        break;
                    }
                }
                if (r_ok == BORDER_INVALID || r_ok <= *up_l) return false;
                *up_r = r_ok;
                *up_r_row = y_r;

                uint8 start = (uint8)(y_r + CROSS_OBL_DN_GAP);
                uint8 d;
                if (cross_find_dn_both_exist(start, &d))
                {
                    *dn_row = d;
                    *dn_l = g_vis.l_border[d];
                    *dn_r = g_vis.r_border[d];
                }
                else
                {
                    *dn_row = (uint8)(pho_h - 1);
                    *dn_l = 0;
                    *dn_r = (uint8)pho_w_max;
                }
                return true;
            }
        }
    }
    else if (right_wrong >= CROSS_OBL_MIN_ROWS && right_wrong > left_wrong)
    {
        /* 右斜入：上方“左边界”实际是右墙；在它向左跳变处恢复成右墙 */
        for (uint8 y = 22; y <= CROSS_OBL_ROW_MAX; y++)
        {
            uint8 l0 = g_vis.l_border[y];
            uint8 l1 = g_vis.l_border[y - 1];
            uint8 l2 = g_vis.l_border[y - 2];
            if (l0 == BORDER_INVALID || l1 == BORDER_INVALID || l2 == BORDER_INVALID) continue;
            if ((int16)l1 - (int16)l0 > CROSS_OBL_JUMP_COL
             && (int16)l2 - (int16)l0 > CROSS_OBL_JUMP_COL)
            {
                *up_r = l0;        /* 跳变后的“左边界”就是真正的右上拐点 */
                *up_r_row = y;

                /* 真正的左墙：从跳变行向下找第一个回到中线左侧的左边界 */
                uint8 y_l = y;
                uint8 l_ok = BORDER_INVALID;
                for (uint8 y2 = y; y2 < pho_h; y2++)
                {
                    uint8 ref2 = ref_bot;
                    (void)cross_get_ref_center(y2, &ref2);
                    if (g_vis.l_border[y2] != BORDER_INVALID
                     && (int16)g_vis.l_border[y2] + CROSS_OBL_REF_MARGIN < (int16)ref2)
                    {
                        l_ok = g_vis.l_border[y2];
                        y_l = y2;
                        break;
                    }
                }
                if (l_ok == BORDER_INVALID || l_ok >= *up_r) return false;
                *up_l = l_ok;
                *up_l_row = y_l;

                uint8 start = (uint8)(y_l + CROSS_OBL_DN_GAP);
                uint8 d;
                if (cross_find_dn_both_exist(start, &d))
                {
                    *dn_row = d;
                    *dn_l = g_vis.l_border[d];
                    *dn_r = g_vis.r_border[d];
                }
                else
                {
                    *dn_row = (uint8)(pho_h - 1);
                    *dn_l = 0;
                    *dn_r = (uint8)pho_w_max;
                }
                return true;
            }
        }
    }

    return false;
}
#endif /* !CROSS_USE_OBLIQUE_SWEEP */

/* (十字独立ROI扫边已删除：四拐点候选版直接复用 sweep 边界) */

/* (车道行有效性 + 全宽白带检测已删除：白区判定已并入 sweep_boundaries) */

/* (单侧拐点搜索 + 拐点对提取已删除：拐点改由 sweep 白区候选行直接得出) */

/* ================================================================
 * 十字补线 — 四拐点直线桥接（下拐点 ← 上拐点）
 * ================================================================ */
static void cross_fill_segment(border_line line, uint8 exist[pho_h],
                               uint8 c0, uint8 r0, uint8 c1, uint8 r1)
{
    if (r1 <= r0) return;

    /* ---- 最小二乘拟合：上/下拐点各取 ±2 行共 10 个点 ----
     * 两点直线对单行锚点噪声太敏感（参考 CSDN qq_53239103 十字补线），
     * 这里用 x=row, y=col 做 y = k*x + b；点不够 4 个时回退两点直线。 */
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    int   n  = 0;
    int16 row;

    for (row = (int16)r0 - 2; row <= (int16)r0 + 2; row++)
    {
        if (row < 0 || row >= pho_h) continue;
        if (line[row] == BORDER_INVALID) continue;
        float x = (float)row;
        float y = (float)line[row];
        sx += x; sy += y; sxx += x * x; sxy += x * y; n++;
    }
    for (row = (int16)r1 - 2; row <= (int16)r1 + 2; row++)
    {
        if (row < 0 || row >= pho_h) continue;
        if (line[row] == BORDER_INVALID) continue;
        float x = (float)row;
        float y = (float)line[row];
        sx += x; sy += y; sxx += x * x; sxy += x * y; n++;
    }

    float k, b;
    bool  use_ls = false;
    if (n >= 4)
    {
        float den = (float)n * sxx - sx * sx;
        if (den > 0.01f || den < -0.01f)
        {
            k = ((float)n * sxy - sx * sy) / den;
            b = (sy - k * sx) / (float)n;
            use_ls = true;
        }
    }
    if (!use_ls)
    {
        k = (float)((int16)c1 - (int16)c0) / (float)((int16)r1 - (int16)r0);
        b = (float)c0 - k * (float)r0;
    }

    for (uint16 r = r0; r <= r1; r++)
    {
        int32 c = (int32)(k * (float)r + b);
        line[r]  = (uint8)cross_limit_u8(c, 0, MT9V03X_W - 1);
        exist[r] = 1;
        /* 补线也会设置exist=1，所以独立保存补线来源，不能仅靠exist判断。 */
        if (line == g_vis.l_border) vision_repair_flags[r] |= 1u;
        if (line == g_vis.r_border) vision_repair_flags[r] |= 2u;
    }
}

static void cross_repair(void)
{
    if (cross_state != CROSS_STATE_DETECTED || !cross_corners_valid)
        return;

#if CROSS_USE_OBLIQUE_SWEEP
    if (cross_oblique_used)
    {
        /* 斜入：有角侧补真实上下拐点，丢线侧补到屏幕边缘 */
        if (oblique_real_side == 1)   /* 左墙有角 → 补左墙，右墙丢线补 x=187 */
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
        return;
    }
#endif

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
 * 十字检测主函数 — 候选 → 四拐点 + 状态机（确认/保持/冷却）
 *
 * 下拐点：near+5 起向下找“双边都真实存在”的第一行；
 *         到 115 仍找不到 → snap 屏幕底角(119, 0, 187)。
 * 上拐点：far-3 起向上找稳定锚点行（搜不到保持 far-3）。
 * 正常候选失败后尝试斜入十字（错边检测 + 换边）。
 * ================================================================ */
static void cross_set_corners(uint8 up_l, uint8 up_l_row,
                              uint8 up_r, uint8 up_r_row,
                              uint8 dn_l, uint8 dn_r, uint8 dn_row)
{
    cross_dn_l_col = dn_l; cross_dn_l_row = dn_row;
    cross_dn_r_col = dn_r; cross_dn_r_row = dn_row;
    cross_up_l_col = up_l; cross_up_l_row = up_l_row;
    cross_up_r_col = up_r; cross_up_r_row = up_r_row;
}

static void cross_detect(void)
{
    bool pair_ok = false;

    if (cross_cand_present)
    {
        uint8 dn_start = (uint8)(cross_near_row + CROSS_DN_ROW_OFFSET);
        bool  dn_snap  = (dn_start > CROSS_DN_SNAP_ROW);

        uint8 up_start = (uint8)(cross_far_row - CROSS_UP_ROW_OFFSET);
        if (up_start < 5) up_start = 5;

        uint8 dn_row = dn_start;
        uint8 dn_l, dn_r;
        bool  dn_found = false;
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

        if (corners_ok && gap_ok)
        {
            cross_set_corners(up_l, up_row, up_r, up_row, dn_l, dn_r, dn_row);
            pair_ok = true;
            cross_oblique_used = false;
        }
    }

    /* 正入失败后允许斜入兜底：只要斜入已经找到有角侧上下拐点，就使用 */
#if CROSS_USE_OBLIQUE_SWEEP
    if (!pair_ok && oblique_valid)
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
    }
#else
    /* 开关关闭时：保留旧 cross_detect_oblique() 回退 */
    if (!pair_ok)
    {
        uint8 o_up_l, o_up_l_row, o_up_r, o_up_r_row, o_dn_l, o_dn_r, o_dn_row;
        if (cross_detect_oblique(&o_up_l, &o_up_l_row,
                                 &o_up_r, &o_up_r_row,
                                 &o_dn_l, &o_dn_r, &o_dn_row)
         && o_up_l != BORDER_INVALID && o_up_r != BORDER_INVALID
         && o_dn_l != BORDER_INVALID && o_dn_r != BORDER_INVALID
         && o_up_l < o_up_r && o_dn_l < o_dn_r)
        {
            cross_set_corners(o_up_l, o_up_l_row, o_up_r, o_up_r_row,
                              o_dn_l, o_dn_r, o_dn_row);
            pair_ok = true;
        }
    }
#endif

    if (pair_ok)
    {
        cross_corners_valid = true;
        cross_missed_count  = 0;
        if (cross_confirm_count < CROSS_CONFIRM_FRAMES)
            cross_confirm_count++;
        cross_state = (cross_confirm_count >= CROSS_CONFIRM_FRAMES)
                    ? CROSS_STATE_DETECTED : CROSS_STATE_CANDIDATE;
    }
    else if (cross_state == CROSS_STATE_DETECTED
             && cross_missed_count < CROSS_HOLD_MISSED_FRAMES)
    {
        cross_missed_count++;
    }
    else
    {
        /* 离开十字 → 直接复位，不再冷却 */
        cross_state = CROSS_STATE_NONE;
        cross_corners_valid = false;
        cross_confirm_count = 0;
        cross_missed_count  = 0;
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
    cross_state          = CROSS_STATE_NONE;
    cross_corners_valid  = false;
    cross_confirm_count  = 0;
    cross_missed_count   = 0;
    cross_cand_present   = false;

    /* 清空拐点，防止复位后残留旧值 */
    cross_up_l_col = BORDER_INVALID;
    cross_up_l_row = BORDER_INVALID;
    cross_up_r_col = BORDER_INVALID;
    cross_up_r_row = BORDER_INVALID;
    cross_dn_l_col = BORDER_INVALID;
    cross_dn_l_row = BORDER_INVALID;
    cross_dn_r_col = BORDER_INVALID;
    cross_dn_r_row = BORDER_INVALID;
#if CROSS_USE_OBLIQUE_SWEEP
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
    cross_oblique_used      = false;
#endif
    zebra_prev           = false;
    zebra_detected       = false;   /* 防斑马线上发车：首帧即产生一次虚假ZEBRA上升沿 */
    zebra_missed_count   = 0;
    zebra_confirm_count  = 0;
    oob_black_rows_dbg   = 0;
    oob_track_l          = pho_w / 4;
    oob_track_r          = pho_w * 3 / 4;
    zebra_track_l        = pho_w / 4;
    zebra_track_r        = pho_w * 3 / 4;
    last_valid_width     = 60;
    last_real_width      = 60;
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
    uint8  white = 0, total = 0;

    if (end >= MT9V03X_W) end = MT9V03X_W - 1;

    for (uint16 c = start; c <= end; c++)
    {
        if (Image_Used[row][c] == 255) white++;
        total++;
    }
    return (white * 2 >= total + 1);  /* 多数：≥ceil(n/2) */
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
    info->valid       = false;
    info->transitions = 0;

    if (col_l >= col_r || (uint16)(col_r - col_l + 1) < 30) return;

    uint16 span = (uint16)(col_r - col_l + 1);

    bool   last_white  = zebra_pixel_is_white(row, col_l);
    uint16 white_count = last_white ? 1 : 0;
    uint16 run_width   = 1;
    uint8  transitions = 0;
    uint8  black_runs  = 0;
    uint8  white_runs  = 0;

    for (uint16 col = (uint16)col_l + 1; col <= col_r; col++)
    {
        bool cur = zebra_pixel_is_white(row, col);
        if (cur) white_count++;

        if (cur == last_white)
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
        last_white = cur;
        run_width = 1;
    }

    /* 收尾最后一段 */
    if (run_width >= ZEBRA_RUN_WIDTH_MIN)
    {
        if (last_white) white_runs++; else black_runs++;
    }

    info->transitions = transitions;
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

    uint8 denom = na;
    if (nb < denom) denom = nb;
    if (denom == 0) return false;

    uint8 match = 0;
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
 * 斑马线检测 — 底部3条采样线全部有效 + 竖向条纹对齐 + 连续确认
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

    zebra_row_info_t rows[ZEBRA_SAMPLE_ROWS];
    uint8 valid_rows = 0;
    for (uint8 s = 0; s < ZEBRA_SAMPLE_ROWS; s++)
    {
        uint8 row = MT9V03X_H - 1 - s * ZEBRA_SAMPLE_ROW_STEP;
        zebra_row_scan(row, zebra_track_l, zebra_track_r, &rows[s]);
        if (rows[s].valid) valid_rows++;
    }

    /* 有效行≥2后，再要求“竖向条纹对齐”：任取两行有效行组成的
     * 跳变列对齐对，命中对数要 >= 有效行数-1。
     * 3行全有效 → 3对中至少2对对齐；2行有效 → 这一对必须对齐。
     * 坡道/阈值噪声各行动作随机，对齐率低，不会进入确认计数。 */
    bool aligned = false;
    if (valid_rows >= ZEBRA_VALID_ROWS_MIN)
    {
        uint8 pairs_ok = 0;
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
        uint8 confirm_need = (uint8)ZEBRA_CONFIRM_FRAMES;
        if (zebra_confirm_count < confirm_need) zebra_confirm_count++;
        zebra_detected      = (zebra_confirm_count >= confirm_need);
        zebra_missed_count  = 0;
    }
    else if (valid_rows == 1 && !zebra_detected)
    {
        /* 高速冲过终点时，斑马线首帧常常只有 1 行有效（valid_rows=1），
         * 紧接着一帧 3 行有效。若这一帧直接清零，后面那帧只能把确认
         * 计数加到 1，永远凑不满 2 帧确认。这里把“单行有效”当作预
         * 确认：确认计数置 1，下一帧必须全有效并竖向对齐，才能置位；
         * 最多容忍一个额外的单行帧，否则清零。单行帧本身永远不能
         * 单独产生 zebra_detected。 */
        if (zebra_confirm_count == 0)
        {
            zebra_confirm_count = 1;    /* pre-arm，等待下一帧全有效 */
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
#define FILTER_BOTTOM_ROWS 110  /* Round69: 120→110，去除顶部10行远距离滤波，确保Otsu重算帧也不超20ms */
static void filter_bottom_connected(void)
{
    uint8 connected[pho_w];
    uint8 prev[pho_w];

    /* 底部行：所有白像素初始标记为连通 */
    for (int x = 0; x < pho_w; x++)
        connected[x] = (Image_Used[pho_h - 1][x] == 255);

    /* 向上逐行传播（仅底部 FILTER_BOTTOM_ROWS 行，降低耗时） */
    int y_min = pho_h - FILTER_BOTTOM_ROWS;
    if (y_min < 0) y_min = 0;
    for (int y = pho_h - 2; y >= y_min; y--)
    {
        /* 保存上一行连通标记（合并循环在读connected的同时写它，必须用副本） */
        for (int x = 0; x < pho_w; x++)
            prev[x] = connected[x];

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
    uint8 l = g_vis.l_border[pho_h - 1];
    uint8 r = g_vis.r_border[pho_h - 1];
    bool  win_ok = (l != BORDER_INVALID && r != BORDER_INVALID
                    && g_vis.l_border_exist[pho_h - 1]
                    && g_vis.r_border_exist[pho_h - 1]
                    && l < r && (r - l) >= 20);

    uint8 wl, wr;
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

    uint8  black_rows  = 0;
    uint16 white_total = 0;
    for (uint8 y = pho_h - OOB_CHECK_ROWS; y < pho_h; y++)
    {
        bool   has_white = false;
        uint16 row_white = 0;
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
    return (white_total < OOB_MIN_WHITE_PIXELS);
}

/* ================================================================
 * 搜线主函数
 *
 * 流程：
 *   1. Otsu 自适应阈值 + 一次性二值化 → Image_Used[][]
 *   2. 底部连通滤波 → 只留与底部连通的白（对称两趟版）
 *   3. 斑马线检测 + 出界检测 → 上报事件（停车决策由 main.c 做）
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
    timer_init(GPT_TIM_1, TIMER_US);                                        // 微秒计时，最大 0xFFFFFFFF
    timer_start(GPT_TIM_1);                                               // 原库初始化后停止；显式启动才有真实耗时
    timer_clear(GPT_TIM_1);
    vis_interval_us = vis_processed_frames = vis_last_event = 0;
    vision_previous_start = 0;
}

VisionEvent vis_deal(void)
{
    uint32 t_frame = timer_get(GPT_TIM_1);                                  // 本帧总耗时起点
    vis_interval_us = vis_processed_frames ? t_frame - vision_previous_start : 0;
    vision_previous_start = t_frame;
    memset(vision_repair_flags, 0, sizeof(vision_repair_flags));

    /* ---- Otsu 自适应阈值（每10帧重算，EMA滤波抑制跳变；优化耗时）---- */
    static int cached_th = -1;
    static int frame_since_otsu = 0;
    frame_since_otsu++;
    if (frame_since_otsu >= 10)
    {
        int th = compute_otsu();
        if (th > 0)
        {
            if (cached_th < 0)
                cached_th = th;
            else
                cached_th = (int)(0.3f * (float)th + 0.7f * (float)cached_th);
        }
        frame_since_otsu = 0;
    }

    /* ---- 一次性二值化整张图 ---- */
    int th = (cached_th >= 0) ? cached_th : 128;
    uint16 t_stage = timer_get(GPT_TIM_1);
    for (int y = 0; y < pho_h; y++)
        for (int x = 0; x < pho_w; x++)
            Image_Used[y][x] = (vision_gray[y][x] > th) ? 255 : 0;
    vis_t_bin_us = (uint16)(timer_get(GPT_TIM_1) - t_stage);

    /* ---- 出界预检：必须在底部连通滤波之前，用原始二值图的
     *      y=100..119 赛道窗口判断。滤波会以底行为种子向上擦除，
     *      底行黑会把上方白全部擦掉，滤波后再查会失真。 ---- */
    int raw_oob = out_of_bounds_check();

    /* ---- 底部连通滤波（对称两趟版，仅处理底部 FILTER_BOTTOM_ROWS 行优化耗时）---- */
    t_stage = timer_get(GPT_TIM_1);
    filter_bottom_connected();
    vis_t_filter_us = (uint16)(timer_get(GPT_TIM_1) - t_stage);

    /* ---- 斑马线检测（先于出界判定，斑马线不出界）---- */
    zebra_detect();

    /* ---- 事件上报（停车决策交给 main.c 主循环）----
     * 斑马线上升沿 → VISION_EVT_ZEBRA；底部20行窗口白像素过少 → LOST。
     * 视觉层不再直接停车、也不再提前 return，保证本帧 err 正常更新。 */
    VisionEvent evt = VISION_EVT_NONE;
    if (zebra_detected)
    {
        if (!zebra_prev) evt = VISION_EVT_ZEBRA;   /* 上升沿 → ZEBRA */
        /* zebra 保持帧也强制不出界（斑马线优先），不报 LOST */
    }
    else if (raw_oob)
    {
        evt = VISION_EVT_LOST;
    }
    zebra_prev = zebra_detected;

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
    sweep_boundaries();
    vis_t_sweep_us = (uint16)(timer_get(GPT_TIM_1) - t_stage);

    /* 原始边界已在 sweep_boundaries() 内逐行保存，这里不再覆盖 */

    /* ---- 十字检测 + 补线（覆盖被横道干扰的边界行）----
     * 预检：没有十字候选/斜入候选时跳过整段十字逻辑，优化耗时。 */
    t_stage = timer_get(GPT_TIM_1);
    if (cross_cand_present || oblique_valid) {
        cross_detect();
        cross_repair();
    }
    vis_t_cross_us = (uint16)(timer_get(GPT_TIM_1) - t_stage);

    /* ---- 中线 + 偏差 ---- */
    t_stage = timer_get(GPT_TIM_1);
    pho_center();
    vis_t_center_us = (uint16)(timer_get(GPT_TIM_1) - t_stage);

    vis_frame_ready = 1;
    vis_export_trace();                                                     // 导出边界/中线坐标，供 WiFi 上传给上位机显示
    vis_time_us = timer_get(GPT_TIM_1) - t_frame;                           // 本帧总耗时(us)
    vis_processed_frames++;
    vis_last_event = (uint32)evt;
    return evt;
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
    int n = 0;

    for (int y = pho_h - 1; y >= 0 && n < VIS_TRACE_ROWS; y--)
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
        n++;
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
