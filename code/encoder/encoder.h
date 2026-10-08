/*********************************************************************************************************************
* 文件名称          encoder.h
* 适用平台          RT1064DVL6A
* 描述              四路正交编码器（QTIMER 原库一倍频：A上升沿计数、B判向）
*
* 硬件：编码器1 A=C0  B=C1     编码器2 A=C2  B=C24
*       编码器3 A=C3  B=C4     编码器4 A=C5  B=C25
*
* 采样：在 PIT_CH0 的 5ms 中断里读取自由计数器并取差 → 200Hz
* 单位：内部保存"每采样周期的脉冲数"，对外提供 cm/s 换算结果
*********************************************************************************************************************/

#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "zf_common_headfile.h"
#include "motor.h"

/* 编码器初始化（四路计数外设；PIT由main单独初始化） */
void  encoder_init(void);

/* 读取某轮原始脉冲计数（每 5ms 周期的脉冲数，带方向）
* 符号约定：与"车前进"同号（内部已应用 ENCODERx_INVERT） */
int16 encoder_get_pulse(wheel_index_enum wheel);

/* 读取某轮线速度，单位 cm/s */
float encoder_get_speed_cms(wheel_index_enum wheel);

/* 四轮线速度一次性取出 */
void  encoder_get_all_cms(float out[WHEEL_NUM]);

/* 里程：累计前进距离（cm），由四轮平均速度积分得到 */
float encoder_get_distance_cm(void);
void  encoder_reset_distance(void);

/* PIT 中断回调（由 isr.c 的 PIT_IRQHandler 调用，5ms 一次） */
void  encoder_pit_callback(void);

/* 标定用：每厘米有效计数，四轮共用；默认由1024线×1×68/30÷19.5cm计算。
 * 用encoder_snapshot()的累计计数差 / 独立测量距离(cm)核对。
 * 每车轮圈计数已包含齿轮与倍频，不再重复乘。 */
void  encoder_set_counts_per_cm(float counts_per_cm);
float encoder_get_counts_per_cm(void);

typedef struct
{
    int32 total[WHEEL_NUM];
    int32 raw_total[WHEEL_NUM];
    uint32 absolute_total[WHEEL_NUM];                                        // 累加每次采样净增量的绝对值，表示活动量
    uint32 sample_count;
    uint32 overflow_mask;                                                  // bit0~3累计量饱和，复位后清零
} encoder_snapshot_t;

void encoder_snapshot(encoder_snapshot_t *out);

#endif
