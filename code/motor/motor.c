/*********************************************************************************************************************
* 文件名称          motor.c
* 适用平台          RT1064DVL6A
* 描述              四路 DRV8701E 电机驱动 + 麦轮逆运动学
*
* 两个 DRV8701E 双路模块 = 四路 DIR + PWM，PWM 17kHz（与逐飞示例一致）
* 引脚定义全部来自 config.h，本文件不硬编码引脚号。
*********************************************************************************************************************/

#include "motor.h"
#include "config.h"

/* ---- 四轮引脚表：顺序与 wheel_index_enum 一致（FL, FR, RL, RR）---- */
static const gpio_pin_enum  motor_dir_pin[WHEEL_NUM] =
{
    MOTOR1_DIR_PIN,                                                         // 左前
    MOTOR2_DIR_PIN,                                                         // 右前
    MOTOR3_DIR_PIN,                                                         // 左后
    MOTOR4_DIR_PIN,                                                         // 右后
};

static const pwm_channel_enum motor_pwm_pin[WHEEL_NUM] =
{
    MOTOR1_PWM_PIN,                                                         // 左前
    MOTOR2_PWM_PIN,                                                         // 右前
    MOTOR3_PWM_PIN,                                                         // 左后
    MOTOR4_PWM_PIN,                                                         // 右后
};

/* 方向补偿表：1 = 该轮取反（电机线接反或麦轮镜像安装导致） */
static const uint8 motor_dir_invert[WHEEL_NUM] =
{
    MOTOR1_DIR_INVERT,
    MOTOR2_DIR_INVERT,
    MOTOR3_DIR_INVERT,
    MOTOR4_DIR_INVERT,
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四路电机初始化
// 功能描述     DIR 初始化为推挽输出、PWM 初始化为 17kHz 占空比 0
// 使用示例     motor_init();
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    uint8 i;

    for (i = 0; i < WHEEL_NUM; i++)
    {
        gpio_init(motor_dir_pin[i], GPO, GPIO_LOW, GPO_PUSH_PULL);          // 方向脚先拉低，避免上电瞬间乱转
        pwm_init(motor_pwm_pin[i], MOTOR_PWM_FREQ, 0);                      // 占空比 0，确认停车状态
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置单个轮子占空比
// 参数说明     wheel       轮子索引（WHEEL_FL / FR / RL / RR）
// 参数说明     duty        占空比，正值 = 车前进方向，范围 ±MOTOR_DUTY_MAX
// 返回参数     void
// 功能描述     内部完成：方向补偿 → 限幅 → DIR 电平 → PWM 占空比
//-------------------------------------------------------------------------------------------------------------------
void motor_set_wheel(wheel_index_enum wheel, int16 duty)
{
    int32 d;

    if (wheel >= WHEEL_NUM) return;

    d = (int32)duty;

    if (motor_dir_invert[wheel]) d = -d;                                    // 方向补偿

    if (d > MOTOR_DUTY_MAX)  d = MOTOR_DUTY_MAX;                            // 限幅
    if (d < -MOTOR_DUTY_MAX) d = -MOTOR_DUTY_MAX;

    if (d >= 0)
    {
        gpio_set_level(motor_dir_pin[wheel], GPIO_HIGH);                    // 正转方向
        pwm_set_duty(motor_pwm_pin[wheel], (uint32)d * (PWM_DUTY_MAX / 100));
    }
    else
    {
        gpio_set_level(motor_dir_pin[wheel], GPIO_LOW);                     // 反转方向
        pwm_set_duty(motor_pwm_pin[wheel], (uint32)(-d) * (PWM_DUTY_MAX / 100));
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四轮同时设置占空比
// 使用示例     motor_set_wheels(30, 30, 30, 30);      // 四轮同速前进
//-------------------------------------------------------------------------------------------------------------------
void motor_set_wheels(int16 duty_fl, int16 duty_fr, int16 duty_rl, int16 duty_rr)
{
    motor_set_wheel(WHEEL_FL, duty_fl);
    motor_set_wheel(WHEEL_FR, duty_fr);
    motor_set_wheel(WHEEL_RL, duty_rl);
    motor_set_wheel(WHEEL_RR, duty_rr);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四轮停车（PWM 归零）
//-------------------------------------------------------------------------------------------------------------------
void motor_stop(void)
{
    motor_set_wheels(0, 0, 0, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     麦轮逆运动学：车体速度 → 四轮线速度
// 参数说明     vx          前进速度 cm/s（正 = 前进）
// 参数说明     vy          横移速度 cm/s（正 = 向左平移）
// 参数说明     omega       自转角速度 deg/s（正 = 逆时针 / 车头向左）
// 参数说明     out         输出四轮线速度 cm/s，顺序 FL, FR, RL, RR
// 返回参数     void
// 功能描述     标准麦轮逆解：
//                  v_fl = vx - vy - ω·(L+W)/2
//                  v_fr = vx + vy + ω·(L+W)/2
//                  v_rl = vx + vy - ω·(L+W)/2
//                  v_rr = vx - vy + ω·(L+W)/2
//              其中 (L+W)/2 是轮心到车体中心的等效力臂；ω 由 deg/s 转 rad/s。
// 备注信息     麦轮辊子装配成 X 形时上式成立；若实车横移/自转方向相反，
//              用 config.h 的 MECANUM_Y_SIGN / MECANUM_W_SIGN 整体取反，
//              不要去改单个电机的方向补偿表。
//-------------------------------------------------------------------------------------------------------------------
void mecanum_inverse(float vx, float vy, float omega, float out[WHEEL_NUM])
{
    float half_diag;                                                        // (L+W)/2，单位 m
    float wz;                                                               // 角速度，单位 rad/s
    float rot;                                                              // ω·(L+W)/2，单位 cm/s

    half_diag = (CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_MM) * 0.5f / 1000.0f; // mm → m
    wz        = omega * 0.0174532925f;                                      // deg/s → rad/s
    rot       = wz * half_diag * 100.0f;                                    // m/s → cm/s

    vy = vy * MECANUM_Y_SIGN;
    rot = rot * MECANUM_W_SIGN;

    out[WHEEL_FL] = vx - vy - rot;
    out[WHEEL_FR] = vx + vy + rot;
    out[WHEEL_RL] = vx + vy - rot;
    out[WHEEL_RR] = vx - vy + rot;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     只前进（横移与自转为 0 的便捷封装）
//-------------------------------------------------------------------------------------------------------------------
void mecanum_drive(float vx, float out[WHEEL_NUM])
{
    mecanum_inverse(vx, 0.0f, 0.0f, out);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     原地自转（前进与横移为 0 的便捷封装）
//-------------------------------------------------------------------------------------------------------------------
void mecanum_spin(float omega, float out[WHEEL_NUM])
{
    mecanum_inverse(0.0f, 0.0f, omega, out);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取麦轮几何参数（供 display / 里程计算使用）
//-------------------------------------------------------------------------------------------------------------------
float motor_get_wheelbase_mm(void) { return CHASSIS_WHEELBASE_MM; }
float motor_get_track_mm(void)     { return CHASSIS_TRACK_MM;     }
float motor_get_wheel_dia_mm(void) { return CHASSIS_WHEEL_DIA_MM; }
