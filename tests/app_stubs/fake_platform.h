#ifndef MCAR_FAKE_PLATFORM_H
#define MCAR_FAKE_PLATFORM_H
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
typedef int16_t int16;
typedef int32_t int32;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef enum { WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR, WHEEL_NUM } wheel_index_enum;
typedef int encoder_index_enum;
typedef int encoder_channel1_enum;
typedef int encoder_channel2_enum;
typedef int gpio_pin_enum;
#define KEY_LIST {15,14,13,12}
#define KEY_RELEASE_LEVEL 1
uint8 gpio_get_level(gpio_pin_enum);
#define GPI 0
#define GPIO_HIGH 1
#define GPI_PULL_UP 1
#define KEY_MAX_SHOCK_PERIOD 10
#define KEY_LONG_PRESS_PERIOD 1000
void gpio_init(gpio_pin_enum, int, int, int);
#define ENCODER_1 1
#define ENCODER_2 0
#define ENCODER_3 2
#define ENCODER_4 3
#define ENCODER_1_A 2
#define ENCODER_1_B 3
#define ENCODER_2_A 0
#define ENCODER_2_B 1
#define ENCODER_3_A 4
#define ENCODER_3_B 5
#define ENCODER_4_A 6
#define ENCODER_4_B 7
#define ENCODER_1_INVERT 1
#define ENCODER_2_INVERT 0
#define ENCODER_3_INVERT 1
#define ENCODER_4_INVERT 0
#define zf_assert assert
void encoder_quad_init(int, int, int);
void encoder_clear_count(int);
int16 encoder_get_count(int);
uint32 interrupt_global_disable(void);
typedef enum { CAR_IDLE = 0, CAR_RUNNING = 2 } CarState;
typedef enum { VISION_EVT_NONE, VISION_EVT_ZEBRA, VISION_EVT_LOST } VisionEvent;
#ifndef APP_DRIVE_ENABLE
#define APP_DRIVE_ENABLE 1
#endif
#ifndef APP_VISION_TRACKING_ENABLE
#define APP_VISION_TRACKING_ENABLE 0
#endif
#define MOTOR_OPEN_LOOP_MAX 20
#define ENCODER_COUNTS_PER_CM (1024.0f * 1.0f * (68.0f / 30.0f) / 19.5f)
typedef enum { KEY_1, KEY_2, KEY_3, KEY_4, KEY_NUMBER } key_index_enum;
typedef enum { KEY_RELEASE, KEY_SHORT_PRESS, KEY_LONG_PRESS } key_state_enum;
key_state_enum key_get_state(key_index_enum key);
void key_clear_state(key_index_enum key);
void motor_set_wheel(wheel_index_enum wheel, int16 duty);
void encoder_get_all_cms(float out[WHEEL_NUM]);
#define ENCODER_CLOSED_LOOP_ENABLE 1
#define CAR_TARGET_SPEED_CMS 150.0f
#define CAR_START_RAMP_MS 500u
#ifndef APP_IMU_VERIFY_ENABLE
#define APP_IMU_VERIFY_ENABLE 0
#endif
#ifndef APP_IMU_VERIFY_TEXT_LOG_HZ
#define APP_IMU_VERIFY_TEXT_LOG_HZ 0
#endif
#define WIFI_DATA_PERIOD_MS 50u
#define APP_MAIN_LOOP_DELAY_MS 1u
extern float ctl_gyro_kd;
#define SYSTEM_CLOCK_600M 600
#define SPEED_LOOP_PERIOD_MS 5
#define IMU_PERIOD_MS 5
#define KEY_PERIOD_MS 10
#define KEY_SCAN_PERIOD_MS 10
#define KEY_STOP_DEBOUNCE_MS 20
void key_init(uint32);
#define WIFI_RETRY_MS 5000
#define WIFI_DATA_PORT "18081"
#define WIFI_LOG_PORT "18082"
#define WIFI_CAMERA_PORT "18083"
#define WIFI_UDP_TARGET_IP "127.0.0.1"
#define PIT_CH0 0
#define PIT_CH1 1
#define LOST_STOP_FRAMES 3
#define ZEBRA_STOP_RISES 2
#define STOP_REASON_ZEBRA 1
#define STOP_REASON_LOST 2
#define STOP_REASON_MANUAL 3
extern volatile CarState car_state;
extern float ctl_speed_target;
extern volatile uint8 stop_reason, mt9v03x_finish_flag;
extern char wifi_spi_ip_addr_port[];
void clock_init(int);
void debug_init(void);
void motor_init(void);
void interrupt_global_enable(int);
void system_delay_ms(int);
int wifi_log(const char *, ...);
uint8 imu_init(void);
void encoder_init(void);
uint8 mt9v03x_init(void);
void vision_init(void);
uint8 vision_capture_frame(void);
#define GPT_TIM_1 1
#define TIMER_US 1
uint32 timer_get(int timer);
void timer_init(int timer, int mode);
void timer_start(int timer);
void timer_clear(int timer);
void wifi_cancel_camera(void);
void wifi_cancel_data(void);
void wifi_record_service_cost(uint32 cost);
void control_init(void);
void control_state_launch(void);
void buttons_init(void);
void buttons_scan_isr(void);
void pit_ms_init(int, int);
uint8 wifi_init(void);
uint8 wifi_is_ready(void);
uint8 wifi_flush_log(void);
void encoder_pit_callback(void);
void imu_update(void);
typedef struct {
    uint8 ready;
    float angles[3], acc_g[3], gyro_dps[3];
    int16 raw[6];
    uint32 sample_count, late_samples, max_update_us, interval_us;
} imu_snapshot_t;
void imu_snapshot(imu_snapshot_t *snapshot);
float imu_get_gyro_z_dps(void);
extern volatile uint8 Speed_PID_Enable;
void Speed_PID_Crtl(void);
void control_tick_isr(void);
void key_scanner(void);
void buttons_poll(void);
VisionEvent vis_deal(void);
void control_state_stop(uint8);
void control_update(void);
void control_stall_watchdog(void);
void wifi_poll(uint32, uint8);
void wifi_poll_imu_verify(uint32);
void wifi_poll_log(uint32);
void app_init(void);
void app_poll(void);
void app_tick_isr(void);
void app_keys_isr(void);
#endif
