/* ICM20602统一采样：同一批六轴数据供姿态解算、控制反馈和上位机。 */
#include "imu.h"
#include "config.h"
#include "imu_attitude.h"
#if ICM20602_USE_SOFT_IIC
#error "This board uses SPI4 ICM20602."
#endif
#if IMU_CAL_SAMPLES < 2 || IMU_PERIOD_MS < 1 || IMU_PERIOD_MS > 256
#error "Invalid IMU calibration or sampling configuration."
#endif
#define IMU_DT ((float)IMU_PERIOD_MS / 1000.0f)
#define IMU_HPF_COEF (IMU_DT / (0.8f + IMU_DT))
#define IMU_GYRO_LPF_ALPHA 0.5f
static volatile imu_snapshot_t imu_state;
static float attitude_q[4], gyro_bias[3];
static float gyro_z_bias, gyro_z_dps;
static uint8 bias_tracking;
static uint32 previous_cycles;
static uint8 imu_read_register(uint8 reg)
{
    uint8 value;
    ICM20602_CS(0);
    value = spi_read_8bit_register(ICM20602_SPI, reg | ICM20602_SPI_R);
    ICM20602_CS(1);
    return value;
}

static void imu_write_register(uint8 reg, uint8 value)
{
    ICM20602_CS(0);
    spi_write_8bit_register(ICM20602_SPI, reg | ICM20602_SPI_W, value);
    ICM20602_CS(1);
}

uint8 imu_init(void)
{
    uint16 sample;
    uint8 axis, attempt, result = 1;
    uint8 id, divider, filter;
    float sum[3] = {0}, square_sum[3] = {0}, acc_sum[3] = {0};
    float acc[3], std_dps[3], angles[3], norm;
    imu_snapshot_t empty = {0};

    imu_state = empty;
    gyro_z_dps = 0;
    bias_tracking = 0;
    previous_cycles = 0;
    for (attempt = 0; attempt < 3; attempt++)
    {
        result = icm20602_init();
        if (result == 0) break;
        system_delay_ms(100);
    }
    if (result != 0) return 1;

    /* 库默认DIV=7对应125Hz。配置DIV=4对应200Hz，与5ms PIT一致。
     * CONFIG=3选择约41Hz陀螺仪低通；加速度库默认约44.8Hz。只改运行时寄存器，不改库。 */
    imu_write_register(ICM20602_CONFIG, 0x03);
    imu_write_register(ICM20602_SMPLRT_DIV, (uint8)(IMU_PERIOD_MS - 1));
    id = imu_read_register(ICM20602_WHO_AM_I);
    divider = imu_read_register(ICM20602_SMPLRT_DIV);
    filter = imu_read_register(ICM20602_CONFIG);
    if (id != 0x12 || divider != IMU_PERIOD_MS - 1 || filter != 0x03) return 4;
    system_delay_ms(100);                                                  // 等待传感器与低通输出稳定

    for (sample = 0; sample < IMU_CAL_SAMPLES; sample++)
    {
        float raw[3];
        icm20602_get_acc();
        icm20602_get_gyro();
        raw[0] = (float)icm20602_gyro_x;
        raw[1] = (float)icm20602_gyro_y;
        raw[2] = (float)icm20602_gyro_z;
        acc_sum[0] += icm20602_acc_transition(icm20602_acc_x);
        acc_sum[1] += icm20602_acc_transition(icm20602_acc_y);
        acc_sum[2] += icm20602_acc_transition(icm20602_acc_z);
        for (axis = 0; axis < 3; axis++)
        {
            sum[axis] += raw[axis];
            square_sum[axis] += raw[axis] * raw[axis];
        }
        system_delay_ms(IMU_PERIOD_MS);
    }
    for (axis = 0; axis < 3; axis++)
    {
        float variance;
        gyro_bias[axis] = sum[axis] / IMU_CAL_SAMPLES;
        acc[axis] = acc_sum[axis] / IMU_CAL_SAMPLES;
        variance = square_sum[axis] / IMU_CAL_SAMPLES - gyro_bias[axis] * gyro_bias[axis];
        std_dps[axis] = icm20602_gyro_transition(sqrtf(variance > 0.0f ? variance : 0.0f));
    }
    norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (norm < 0.75f || norm > 1.25f) return 2;
    /* 粗略运动筛查，不能替代人为保证静止（匀速旋转可能有很小的方差）。 */
    if (std_dps[0] > 3.0f || std_dps[1] > 3.0f || std_dps[2] > 3.0f) return 3;

    imu_attitude_reset(attitude_q, acc);
    imu_attitude_angles(attitude_q, angles);
    for (axis = 0; axis < 3; axis++)
    {
        imu_state.angles[axis] = angles[axis];
        imu_state.acc_g[axis] = acc[axis];
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    imu_state.ready = 1;
    gyro_z_bias = gyro_bias[2];
    return 0;
}

void imu_update(void)
{
    uint32 started = DWT->CYCCNT;
    uint32 cycles_per_us = system_clock / 1000000u;
    uint32 interval = IMU_PERIOD_MS * 1000u;
    uint32 cost;
    uint8 axis;
    float dt = IMU_DT;
    float control_yaw_rate;
    float acc[3], gyro[3], angles[3];

    if (imu_state.sample_count != 0)
    {
        interval = (started - previous_cycles) / cycles_per_us;
        if (interval > IMU_PERIOD_MS * 1500u) imu_state.late_samples++;
        /* 正常抖动用实际dt；长时间缺采样无法重建运动，计数提示，不用陈旧角速度积分长间隔。 */
        if (interval > 0 && interval <= IMU_PERIOD_MS * 10000u) dt = (float)interval * 0.000001f;
    }
    previous_cycles = started;
    icm20602_get_acc();
    icm20602_get_gyro();
    imu_state.raw[0] = icm20602_acc_x;
    imu_state.raw[1] = icm20602_acc_y;
    imu_state.raw[2] = icm20602_acc_z;
    imu_state.raw[3] = icm20602_gyro_x;
    imu_state.raw[4] = icm20602_gyro_y;
    imu_state.raw[5] = icm20602_gyro_z;
    for (axis = 0; axis < 3; axis++)
    {
        acc[axis] = icm20602_acc_transition(imu_state.raw[axis]);
        gyro[axis] = icm20602_gyro_transition((float)imu_state.raw[axis + 3] - gyro_bias[axis]);
    }
    /* 控制反馈沿用静止零偏跟踪；姿态和上位机六轴值沿用固定三轴校准，避免手动转车时吞掉航向。 */
    control_yaw_rate = icm20602_gyro_transition((float)icm20602_gyro_z - gyro_z_bias);
    if (bias_tracking) gyro_z_bias += IMU_HPF_COEF * ((float)icm20602_gyro_z - gyro_z_bias);
    gyro_z_dps += IMU_GYRO_LPF_ALPHA * (control_yaw_rate * IMU_MOUNT_YAW_SIGN - gyro_z_dps);
    imu_attitude_step(attitude_q, acc, gyro, dt, IMU_FUSION_KP);
    imu_attitude_angles(attitude_q, angles);
    for (axis = 0; axis < 3; axis++)
    {
        imu_state.angles[axis] = angles[axis];
        imu_state.acc_g[axis] = acc[axis];
        imu_state.gyro_dps[axis] = gyro[axis];
    }
    imu_state.interval_us = interval;
    imu_state.sample_count++;
    cost = (DWT->CYCCNT - started) / cycles_per_us;
    if (cost > imu_state.max_update_us) imu_state.max_update_us = cost;
}

void imu_snapshot(imu_snapshot_t *snapshot)
{
    uint32 primask;
    zf_assert(snapshot != NULL);
    primask = interrupt_global_disable();                                 // 仅复制快照，禁止在临界区打印或发WiFi
    *snapshot = imu_state;
    interrupt_global_enable(primask);
}
float imu_get_gyro_z_dps(void) { return gyro_z_dps; }
void imu_set_bias_tracking(uint8 enable) { bias_tracking = enable ? 1 : 0; }
