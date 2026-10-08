/* 六轴测试用姿态内核。输入：加速度g、去零偏角速度deg/s、实际步长s。
 * 坐标遵循传感器右手XYZ；四元数表示传感器相对参考坐标的旋转。
 * 加速度只修正重力方向，yaw无绝对观测；动态加速时姿态仍可能受扰动。
 * 独立于硬件，供imu测试与电脑数值测试共同使用。 */
#ifndef MCAR_IMU_ATTITUDE_H
#define MCAR_IMU_ATTITUDE_H

#include <math.h>

#define IMU_ATTITUDE_RAD_PER_DEG (0.017453292519943295f)
#define IMU_ATTITUDE_DEG_PER_RAD (57.29577951308232f)

static inline void imu_attitude_angles(const float q[4], float angles[3])
{
    float sin_pitch = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    angles[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                      1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * IMU_ATTITUDE_DEG_PER_RAD;
    angles[1] = asinf(sin_pitch) * IMU_ATTITUDE_DEG_PER_RAD;
    angles[2] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * IMU_ATTITUDE_DEG_PER_RAD;
}

/* 用静止重力方向初始化倾角；参考yaw置0，保留真实roll/pitch而非强制归零。 */
static inline void imu_attitude_reset(float q[4], const float acc[3])
{
    float roll = atan2f(acc[1], acc[2]);
    float pitch = atan2f(-acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2]));
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
}

static inline void imu_attitude_step(float q[4], const float acc[3],
                                    const float gyro_dps[3], float dt, float kp)
{
    float gx = gyro_dps[0] * IMU_ATTITUDE_RAD_PER_DEG;
    float gy = gyro_dps[1] * IMU_ATTITUDE_RAD_PER_DEG;
    float gz = gyro_dps[2] * IMU_ATTITUDE_RAD_PER_DEG;
    float acc_norm_sq = acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2];
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float inv_norm;

    /* 模长在0.8~1.2g时才用重力纠偏；否则仅积分陀螺仪，避免明显平移加速度干扰。 */
    if (acc_norm_sq >= 0.64f && acc_norm_sq <= 1.44f)
    {
        float ax, ay, az, vx, vy, vz;
        inv_norm = 1.0f / sqrtf(acc_norm_sq);
        ax = acc[0] * inv_norm;
        ay = acc[1] * inv_norm;
        az = acc[2] * inv_norm;
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        gx += kp * (ay * vz - az * vy);
        gy += kp * (az * vx - ax * vz);
        gz += kp * (ax * vy - ay * vx);
    }

    q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    q[1] += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
    q[2] += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
    q[3] += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;
    inv_norm = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= inv_norm;
    q[1] *= inv_norm;
    q[2] *= inv_norm;
    q[3] *= inv_norm;
}

#endif
