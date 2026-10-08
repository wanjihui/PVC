/*********************************************************************************************************************
* 文件名称          encoder.c
* 适用平台          RT1064DVL6A
* 描述              四路正交编码器（QTIMER 硬件正交解码）
*
* 定时采样：PIT_CH0 的 5ms 中断 → 200Hz，与速度环同频
* 单位换算：有效计数/周期 → cm/s，依赖每厘米计数 COUNTS_PER_CM
*********************************************************************************************************************/

#include "encoder.h"
#include "config.h"
#include <float.h>

/* 四路编码器索引与通道（顺序 FL, FR, RL, RR） */
static const encoder_index_enum   encoder_index[WHEEL_NUM] =
{
    ENCODER_1, ENCODER_2, ENCODER_3, ENCODER_4,
};
static const encoder_channel1_enum encoder_ch_a[WHEEL_NUM] =
{
    ENCODER_1_A, ENCODER_2_A, ENCODER_3_A, ENCODER_4_A,
};
static const encoder_channel2_enum encoder_ch_b[WHEEL_NUM] =
{
    ENCODER_1_B, ENCODER_2_B, ENCODER_3_B, ENCODER_4_B,
};
static const uint8 encoder_invert[WHEEL_NUM] =
{
    ENCODER_1_INVERT, ENCODER_2_INVERT, ENCODER_3_INVERT, ENCODER_4_INVERT,
};


#include "encoder_counter.h"
static volatile int32 encoder_pulse[WHEEL_NUM];
static volatile encoder_snapshot_t counts;
static uint16 previous_count[WHEEL_NUM];
static float counts_per_cm = ENCODER_COUNTS_PER_CM;
static float speed_scale = 1.0f / ((SPEED_LOOP_PERIOD_MS / 1000.0f) * ENCODER_COUNTS_PER_CM);
static volatile float distance_cm;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     脉冲/周期 → cm/s 换算
// 功能描述     换算系数预计算，每周期只做乘法；计数为整数，速度为float。
//              v(cm/s) = 计数增量 ÷ (周期秒数 × 每厘米计数)
//-------------------------------------------------------------------------------------------------------------------
static float pulse_to_cms(int32 pulse)
{
    return (float)pulse * speed_scale;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四路编码器初始化
// 功能描述     四路 QTIMER 硬件正交解码，PIT周期任务由应用层启动
// 使用示例     encoder_init();
//-------------------------------------------------------------------------------------------------------------------
void encoder_init(void)
{
    uint8 wheel;
    encoder_snapshot_t empty = {0};
    counts = empty;
    distance_cm = 0;
    for (wheel = 0; wheel < WHEEL_NUM; wheel++)
    {
        encoder_quad_init(encoder_index[wheel], encoder_ch_a[wheel], encoder_ch_b[wheel]);
        encoder_clear_count(encoder_index[wheel]);
        previous_count[wheel] = (uint16)encoder_get_count(encoder_index[wheel]);
        encoder_pulse[wheel] = 0;
    }
}

/* 硬件自由计数器取差：运行期不清零，避免读数与清零之间丢脉冲。 */
void encoder_pit_callback(void)
{
    uint8 wheel;
    float speed_sum = 0;
    for (wheel = 0; wheel < WHEEL_NUM; wheel++)
    {
        uint16 current = (uint16)encoder_get_count(encoder_index[wheel]);
        int32 raw = encoder_counter_delta(current, previous_count[wheel], 0);
        int32 corrected = encoder_invert[wheel] ? -raw : raw;
        uint32 overflow = 0;
        uint32 absolute = (uint32)(raw < 0 ? -raw : raw);
        previous_count[wheel] = current;
        encoder_pulse[wheel] = corrected;
        counts.total[wheel] = encoder_counter_add(counts.total[wheel], corrected, &overflow);
        counts.raw_total[wheel] = encoder_counter_add(counts.raw_total[wheel], raw, &overflow);
        if (counts.absolute_total[wheel] > UINT32_MAX - absolute)
        {
            counts.absolute_total[wheel] = UINT32_MAX;
            overflow = 1;
        }
        else counts.absolute_total[wheel] += absolute;
        if (overflow) counts.overflow_mask |= 1u << wheel;
        speed_sum += pulse_to_cms(corrected);
    }
    counts.sample_count++;
    distance_cm += speed_sum * 0.25f * (SPEED_LOOP_PERIOD_MS / 1000.0f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取某轮原始脉冲（每采样周期的脉冲数）
//-------------------------------------------------------------------------------------------------------------------
int16 encoder_get_pulse(wheel_index_enum wheel)
{
    int32 pulse;
    if (wheel >= WHEEL_NUM) return 0;
    pulse = encoder_pulse[wheel];
    return (int16)(pulse > 32767 ? 32767 : pulse < -32768 ? -32768 : pulse);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取某轮线速度（cm/s）
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_speed_cms(wheel_index_enum wheel)
{
    if (wheel >= WHEEL_NUM) return 0.0f;
    return pulse_to_cms(encoder_pulse[wheel]);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一次性取出四轮线速度（cm/s）
//-------------------------------------------------------------------------------------------------------------------
void encoder_get_all_cms(float out[WHEEL_NUM])
{
    uint8 i;
    if (out == NULL) return;
    for (i = 0; i < WHEEL_NUM; i++)
    {
        out[i] = pulse_to_cms(encoder_pulse[i]);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取累计前进里程（cm）
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_distance_cm(void)
{
    return distance_cm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     里程清零（发车前调用）
//-------------------------------------------------------------------------------------------------------------------
void encoder_reset_distance(void)
{
    distance_cm = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置每厘米有效计数（标定用，四轮共用）
// 参数说明     value    累计计数变化 / 独立测量距离(cm)，不要用低频显示的瞬时增量相加。
//-------------------------------------------------------------------------------------------------------------------
void encoder_set_counts_per_cm(float value)
{
    float scale;
    uint32 primask;
    if (!(value > 0.0f && value <= FLT_MAX)) return;
    scale = 1.0f / ((SPEED_LOOP_PERIOD_MS / 1000.0f) * value);
    if (!(scale > 0.0f && scale <= FLT_MAX)) return;
    primask = interrupt_global_disable();
    {
        counts_per_cm = value;
        speed_scale = scale;
    }
    interrupt_global_enable(primask);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取当前每厘米有效计数
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_counts_per_cm(void)
{
    return counts_per_cm;
}

void encoder_snapshot(encoder_snapshot_t *out)
{
    uint32 primask;
    zf_assert(out != NULL);
    primask = interrupt_global_disable();
    *out = counts;
    interrupt_global_enable(primask);
}
