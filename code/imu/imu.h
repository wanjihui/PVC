/*********************************************************************************************************************
* 文件名称          imu.h
* 适用平台          RT1064DVL6A
* 描述              ICM20602 六轴传感器（硬件 SPI4）
*
* 硬件：SCK=C23  MOSI=C22  MISO=C21  CS=C20（引脚宏在 libraries/zf_device/zf_device_icm20602.h）
* 用途：本车不用姿态角，只需 gyro_z（绕垂直轴角速度）做转向阻尼/自转闭环反馈
*********************************************************************************************************************/

#ifndef __IMU_H_
#define __IMU_H_

#include "zf_common_headfile.h"

/* 初始化（失败会返回非 0，调用方应重试或报错停在屏幕上） */
uint8 imu_init(void);

/* 采样一次并更新内部量（放在 200Hz 中断或 50Hz 主循环里均可，保证周期稳定） */
void  imu_update(void);

/* 陀螺仪 Z 轴角速度，单位 deg/s，已应用 IMU_MOUNT_YAW_SIGN 与零偏校准
* 符号约定：车头向左（逆时针）转时为正 */
float imu_get_gyro_z_dps(void);

/* 三轴原始值（调试用） */
int16 imu_get_gyro_raw_x(void);
int16 imu_get_gyro_raw_y(void);
int16 imu_get_gyro_raw_z(void);
int16 imu_get_acc_raw_z(void);

/* 零偏校准：上电后车必须静止，采样 N 次取平均作为零偏；返回实际采样次数 */
uint16 imu_calibrate_gyro(uint16 samples);

/* 停止时自动零偏跟踪（车辆静止时抑制零偏漂移），由 control 层调用 */
void  imu_set_bias_tracking(uint8 enable);

#endif
