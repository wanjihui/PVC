#ifndef MCAR_IMU_H
#define MCAR_IMU_H
#include "zf_common_headfile.h"
/* 初始化返回0成功；1芯片失败、2加速度异常、3校准期间运动、4寄存器异常。 */
typedef struct
{
    uint8 ready;
    float angles[3];                                                        // roll/pitch/yaw，deg，传感器坐标
    float acc_g[3];
    float gyro_dps[3];                                                      // 三轴去零偏，未用车辆安装符号
    int16 raw[6];                                                           // ax/ay/az/gx/gy/gz
    uint32 sample_count;
    uint32 late_samples;                                                   // 间隔>1.5倍目标周期的次数
    uint32 max_update_us;                                                  // 一次采集+解算最大用时（PIT采样）
    uint32 interval_us;                                                    // 最新两次采样入口间隔
} imu_snapshot_t;
uint8 imu_init(void);
void imu_update(void); /* 正常模式仅PIT调用；IMU验证模式仅主循环定时调用，禁止两处并发。 */
void imu_snapshot(imu_snapshot_t *out);
float imu_get_gyro_z_dps(void);
void imu_set_bias_tracking(uint8 enable);
#endif
