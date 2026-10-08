/* 电脑上执行的姿态内核验证：使用解析解生成重力向量/角速度，不依赖传感器。
 * 编译示例：cl /TC /Icode/imu tests/test_imu_attitude.c
 * 测试与固件共用imu_attitude.h，覆盖角度单位、旋转方向、重力纠偏和无效加速度。 */
#include <stdio.h>
#include <stdlib.h>
#include "imu_attitude.h"

static unsigned int checks;

static void near_value(const char *label, float actual, float expected, float tolerance)
{
    checks++;
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance)
    {
        fprintf(stderr, "FAIL %s: actual %.6f expected %.6f tolerance %.6f\n",
                label, actual, expected, tolerance);
        exit(1);
    }
}

static void gravity(float roll_deg, float pitch_deg, float acc[3])
{
    float roll = roll_deg * IMU_ATTITUDE_RAD_PER_DEG;
    float pitch = pitch_deg * IMU_ATTITUDE_RAD_PER_DEG;
    acc[0] = -sinf(pitch);
    acc[1] = sinf(roll) * cosf(pitch);
    acc[2] = cosf(roll) * cosf(pitch);
}

int main(void)
{
    float q[4], angles[3], acc[3], zero[3] = {0};
    float gyro[3] = {0, 0, 90};
    unsigned int i;

    gravity(30, -20, acc);
    imu_attitude_reset(q, acc);
    imu_attitude_angles(q, angles);
    near_value("static initial roll", angles[0], 30, 0.001f);
    near_value("static initial pitch", angles[1], -20, 0.001f);
    near_value("initial yaw", angles[2], 0, 0.001f);
    for (i = 0; i < 1000; i++) imu_attitude_step(q, acc, zero, 0.005f, 2);
    imu_attitude_angles(q, angles);
    near_value("tilted board stays roll", angles[0], 30, 0.01f);
    near_value("tilted board stays pitch", angles[1], -20, 0.01f);

    gravity(0, 0, acc);
    imu_attitude_reset(q, acc);
    for (i = 0; i < 200; i++) imu_attitude_step(q, acc, gyro, 0.005f, 2);
    imu_attitude_angles(q, angles);
    near_value("positive Z 90deg/s for 1sec", angles[2], 90, 0.01f);
    near_value("yaw does not tilt roll", angles[0], 0, 0.001f);
    near_value("yaw does not tilt pitch", angles[1], 0, 0.001f);
    for (i = 0; i < 1000; i++) imu_attitude_step(q, acc, zero, 0.005f, 2);
    imu_attitude_angles(q, angles);
    near_value("gravity cannot reset yaw", angles[2], 90, 0.01f);

    imu_attitude_reset(q, acc);
    gyro[2] = -90;
    for (i = 0; i < 200; i++) imu_attitude_step(q, zero, gyro, 0.005f, 2);
    imu_attitude_angles(q, angles);
    near_value("zero acceleration safe gyro-only", angles[2], -90, 0.01f);

    gyro[2] = 60;
    imu_attitude_reset(q, acc);
    for (i = 0; i < 100; i++)
    {
        imu_attitude_step(q, acc, gyro, 0.003f, 2);
        imu_attitude_step(q, acc, gyro, 0.007f, 2);
    }
    imu_attitude_angles(q, angles);
    near_value("variable sample interval", angles[2], 60, 0.01f);

    /* 初值故意与真实重力方向不同，检查纠偏方向和收敛。 */
    imu_attitude_reset(q, acc);
    gravity(30, -20, acc);
    for (i = 0; i < 2000; i++) imu_attitude_step(q, acc, zero, 0.005f, 2);
    imu_attitude_angles(q, angles);
    near_value("gravity feedback roll converges", angles[0], 30, 0.01f);
    near_value("gravity feedback pitch converges", angles[1], -20, 0.01f);

    gravity(0, 0, acc);
    imu_attitude_reset(q, acc);
    acc[2] = 2;                                                           // 模长不在纠偏范围内
    gyro[0] = 45;
    gyro[2] = 0;
    for (i = 0; i < 200; i++) imu_attitude_step(q, acc, gyro, 0.005f, 2);
    imu_attitude_angles(q, angles);
    near_value("2g acceleration rejects gravity correction", angles[0], 45, 0.01f);

    /* 持续绕X转动，重力向量来自已知旋转的解析解。 */
    gravity(0, 0, acc);
    imu_attitude_reset(q, acc);
    gyro[0] = 90;
    for (i = 0; i < 100; i++)
    {
        gravity((float)i * 0.45f, 0, acc);
        imu_attitude_step(q, acc, gyro, 0.005f, 2);
    }
    imu_attitude_angles(q, angles);
    near_value("X rotation and moving gravity agree", angles[0], 45, 0.02f);

    /* 欧拉角速率取决于当前姿态：倒装roll=180度时，正gy使pitch降低。
     * 对照正装roll=0度，区分安装坐标效应与简单的角速度反号。 */
    gyro[0] = gyro[2] = 0;
    gyro[1] = 30;
    gravity(0, 0, acc);
    imu_attitude_reset(q, acc);
    for (i = 0; i < 200; i++) imu_attitude_step(q, zero, gyro, 0.005f, 2);
    imu_attitude_angles(q, angles);
    near_value("upright +gy gives positive pitch", angles[1], 30, 0.01f);
    gravity(180, 0, acc);
    imu_attitude_reset(q, acc);
    imu_attitude_angles(q, angles);
    near_value("inverted initial abs roll", fabsf(angles[0]), 180, 0.001f);
    for (i = 0; i < 200; i++) imu_attitude_step(q, zero, gyro, 0.005f, 2);
    imu_attitude_angles(q, angles);
    near_value("inverted +gy gives negative pitch", angles[1], -30, 0.01f);

    /* 正负90度俯仰附近保证反三角函数有限；此处欧拉角本来就有奇异性。 */
    gravity(0, 90, acc);
    imu_attitude_reset(q, acc);
    imu_attitude_angles(q, angles);
    near_value("pitch +90 finite", angles[1], 90, 0.03f);
    gravity(0, -90, acc);
    imu_attitude_reset(q, acc);
    imu_attitude_angles(q, angles);
    near_value("pitch -90 finite", angles[1], -90, 0.03f);
    near_value("normalized quaternion", q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3], 1, 0.00001f);
    printf("PASS: %u analytical attitude checks\n", checks);
    return 0;
}
