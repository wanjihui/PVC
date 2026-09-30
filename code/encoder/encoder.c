/*********************************************************************************************************************
* 文件名称          encoder.c
* 适用平台          RT1064DVL6A
* 描述              四路正交编码器（QTIMER 硬件正交解码）
*
* 定时采样：PIT_CH0 的 5ms 中断 → 200Hz，与速度环同频
* 单位换算：脉冲/周期 → cm/s，依赖 PULSES_PER_METER 标定值
*********************************************************************************************************************/

#include "encoder.h"
#include "config.h"

/* 四路编码器索引与通道（顺序 FL, FR, RL, RR） */
#if ENCODER_ENABLE
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
#endif

/* 最近一次采样得到的脉冲数（每 SPEED_LOOP_PERIOD_MS 周期），符号已按"前进为正"归一 */
static volatile int16 encoder_pulse[WHEEL_NUM] = {0, 0, 0, 0};

/* 每米脉冲数：决定 cm/s 换算精度。默认值按常见配置估算，实车必须标定
* 标定方法：推车直线走 10m，累加 encoder_pulse 绝对值 / 10 */
static float pulses_per_meter = 2000.0f;

/* 累计前进里程（cm） */
static volatile float distance_cm = 0.0f;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     脉冲/周期 → cm/s 换算
// 功能描述     使用运行期标定值 pulses_per_meter，因此菜单改标定后立即生效
//              推导：v(cm/s) = 脉冲/周期 ÷ (周期秒数 × 每米脉冲数) × 100
//-------------------------------------------------------------------------------------------------------------------
static float pulse_to_cms(int16 pulse)
{
    float period_s = (float)SPEED_LOOP_PERIOD_MS / 1000.0f;
    return ((float)pulse / (period_s * pulses_per_meter)) * 100.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四路编码器初始化
// 功能描述     四路 QTIMER 硬件正交解码 + PIT_CH0 周期中断（5ms）
// 使用示例     encoder_init();
//-------------------------------------------------------------------------------------------------------------------
void encoder_init(void)
{
#if ENCODER_ENABLE
    uint8 i;

    for (i = 0; i < WHEEL_NUM; i++)
    {
        encoder_quad_init(encoder_index[i], encoder_ch_a[i], encoder_ch_b[i]);
        encoder_clear_count(encoder_index[i]);                              // 清零，避免上电残留
    }

    /* 周期中断由 main.c 统一初始化（encoder_init 只负责编码器本体），
    * 采集回调 encoder_pit_callback() 在 isr.c 的 PIT_IRQHandler 里被调用 */
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 中断回调：读取并清零四路编码器
// 功能描述     必须在 PIT_CH0 的 5ms 中断里调用
// 备注信息     encoder_get_count 读的是正交解码硬件计数器的累计值，
//              读完后立即清零，得到的就是"本周期脉冲数"
//-------------------------------------------------------------------------------------------------------------------
void encoder_pit_callback(void)
{
#if ENCODER_ENABLE
    uint8 i;
    int16 p;
    float sum_cms = 0.0f;
    float v;

    for (i = 0; i < WHEEL_NUM; i++)
    {
        p = encoder_get_count(encoder_index[i]);
        encoder_clear_count(encoder_index[i]);

        if (encoder_invert[i]) p = -p;                                      // 方向归一：前进为正

        encoder_pulse[i] = p;

        v = pulse_to_cms(p);                                        // 脉冲/周期 → cm/s
        sum_cms += v;
    }

    /* 里程积分：四轮平均速度 × 周期时长（cm） */
    distance_cm += (sum_cms * 0.25f) * (SPEED_LOOP_PERIOD_MS / 1000.0f);
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取某轮原始脉冲（每采样周期的脉冲数）
//-------------------------------------------------------------------------------------------------------------------
int16 encoder_get_pulse(wheel_index_enum wheel)
{
    if (wheel >= WHEEL_NUM) return 0;
    return encoder_pulse[wheel];
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
// 函数简介     设置每米脉冲数（标定用）
// 参数说明     pulses_per_meter    推车 10m 得到的累计脉冲数 / 10
//-------------------------------------------------------------------------------------------------------------------
void encoder_set_pulses_per_meter(float pulses_per_meter_val)
{
    if (pulses_per_meter_val > 1.0f)
    {
        pulses_per_meter = pulses_per_meter_val;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取当前每米脉冲数
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_pulses_per_meter(void)
{
    return pulses_per_meter;
}
