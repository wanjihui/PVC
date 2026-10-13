/* Link the real app.c against deterministic peripherals; no physical motor or UDP. */
#include "fake_platform.h"
#include "app.h"
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

volatile CarState car_state;
float ctl_speed_target=CAR_TARGET_SPEED_CMS;
float ctl_gyro_kd;
volatile uint8 stop_reason, mt9v03x_finish_flag;
char wifi_spi_ip_addr_port[] = "test-only";
static int scenario, encoder_calls, imu_calls, motor_calls, wifi_calls;
static int control_ticks;
static int key_calls, vision_calls, camera_calls, upload_calls, command;
static int pending_logs, log_packets, flush_calls, log_poll_calls;
static uint32 log_at;
static uint8 mock_online=1;
void clock_init(int clock) { (void)clock; }
void debug_init(void) {}
void motor_init(void) {}
void interrupt_global_enable(int state) { (void)state; }
void system_delay_ms(int delay) { (void)delay; }
int wifi_log(const char *fmt, ...) { (void)fmt; pending_logs++; return 0; }
uint8 imu_init(void) { return scenario == 1 ? 1 : 0; }
void encoder_init(void) {}
uint8 mt9v03x_init(void) { camera_calls++; return scenario == 2 ? 1 : 0; }
void vision_init(void) {}
uint8 vision_capture_frame(void) { uint8 ready = mt9v03x_finish_flag; mt9v03x_finish_flag = 0; return ready; }
uint32 timer_get(int timer) { static uint32 clock; (void)timer; return ++clock; }
void wifi_cancel_camera(void) {}
void wifi_cancel_data(void) {}
void wifi_record_service_cost(uint32 cost) { (void)cost; }
void control_init(void) { car_state = CAR_IDLE; }
void buttons_init(void) {}
void pit_ms_init(int channel, int period) { (void)channel; (void)period; }
uint8 wifi_init(void) { wifi_calls++; mock_online=1; return scenario == 3 && wifi_calls == 1 ? 2 : 0; }
uint8 wifi_is_ready(void) { return mock_online; }
uint8 wifi_flush_log(void)
{
    flush_calls++;
    if (scenario == 4 && flush_calls == 1) return 4; /* Retain failed startup submission. */
    if (pending_logs) { log_packets++; pending_logs = 0; }
    return 0;
}
void wifi_poll_log(uint32 now)
{
    log_poll_calls++;
    if (!pending_logs || (uint32)(now - log_at) < 1000u) return;
    log_at = now;
    (void)wifi_flush_log();
}
void encoder_pit_callback(void) { encoder_calls++; }
void imu_update(void) { imu_calls++; }
void control_tick_isr(void) { control_ticks++; assert(control_ticks == encoder_calls); }
void Speed_PID_Crtl(void) { motor_calls++; assert(motor_calls == control_ticks); }
void key_scanner(void) { key_calls++; }
void buttons_scan_isr(void)
{
    key_scanner();
    if (command == 2) { car_state = CAR_IDLE; command = 0; }
}
void buttons_poll(void)
{
    if (command == 1) car_state = CAR_RUNNING;
    if (command == 2) car_state = CAR_IDLE;
    command = 0;
}
VisionEvent vis_deal(void) { vision_calls++; return VISION_EVT_NONE; }
void control_state_stop(uint8 reason) { stop_reason = reason; car_state = CAR_IDLE; }
void control_update(void) {}
void control_stall_watchdog(void) {}
void wifi_poll(uint32 now, uint8 new_frame)
{ (void)now; if(scenario != 1) upload_calls++; (void)new_frame; }

int main(int argc, char **argv)
{
    int i, allowed, previous_wifi, previous_uploads, previous_logs, previous_log_polls, logs_waiting;
    scenario = argc > 1 ? atoi(argv[1]) : 0;
    app_init();
    allowed = APP_DRIVE_ENABLE && scenario != 1 && (!APP_VISION_TRACKING_ENABLE || scenario != 2);
    assert(car_state == CAR_IDLE); /* Never launch at power-on. */
    assert(flush_calls == (scenario == 3 ? 0 : 1));
    assert(log_packets == (scenario == 3 || scenario == 4 ? 0 : 1)); /* Init logs independent of K1/IMU success. */
    assert((pending_logs != 0) == (scenario == 3 || scenario == 4));
    assert(camera_calls == (APP_VISION_TRACKING_ENABLE ? (scenario == 2 ? 3 : 1) : 0));
    previous_wifi = wifi_calls;
    previous_log_polls = log_poll_calls;
    app_tick_isr();
    app_keys_isr();
    assert(encoder_calls == 1);
    assert(imu_calls == (scenario == 1 ? 0 : 1));
    assert(motor_calls == allowed && key_calls == allowed);
    assert(wifi_calls == previous_wifi && upload_calls == 0); /* No network in ISR. */
    assert(log_poll_calls == previous_log_polls);
    app_poll();
    assert(upload_calls == (scenario != 1 && scenario != 3)); /* Healthy parked IMU gets main-loop WiFi service. */
    assert(log_poll_calls == (scenario == 3 ? 0 : 1));
    previous_logs = log_packets;
    for (i = 0; i < 200; i++) app_tick_isr();
    app_poll();
    assert(upload_calls == 2*(scenario != 1 && scenario != 3));
    assert(log_packets == previous_logs + (scenario == 4 ? 1 : 0)); /* Startup retry while parked. */
    command = 1;
    mt9v03x_finish_flag = 1;
    app_poll();
    assert(car_state == (allowed ? CAR_RUNNING : CAR_IDLE));
    assert(vision_calls == (APP_VISION_TRACKING_ENABLE && scenario != 2 ? 1 : 0));
    if (APP_VISION_TRACKING_ENABLE && scenario != 2) assert(mt9v03x_finish_flag == 0);
    if (!APP_VISION_TRACKING_ENABLE) assert(mt9v03x_finish_flag == 1); /* No image consumption during speed tuning. */
    assert(upload_calls == 3*(scenario != 1 && scenario != 3));
    previous_uploads = upload_calls;
    if (scenario == 3 && allowed)
    {
        for (i = 0; i < 1001; i++) app_tick_isr();
        app_poll();
        assert(wifi_calls == 1); /* No blocking WiFi join while moving. */
        command = 2;
        app_keys_isr();
        assert(car_state == CAR_IDLE); /* K2 must stop before main returns from a blocked network call. */
        app_poll();
        assert(car_state == CAR_IDLE && wifi_calls == 2);
    }
    else if (allowed)
    {
        command = 2;
        app_keys_isr();
        assert(car_state == CAR_IDLE);
        app_poll();
        assert(car_state == CAR_IDLE);
    }
    assert(upload_calls == previous_uploads + allowed); /* K2 stops motion; parked IMU service continues. */
    previous_uploads = upload_calls;
    previous_logs = log_packets;
    logs_waiting = pending_logs != 0 && (scenario != 3 || wifi_calls > 1);
    for (i = 0; i < 200; i++) app_tick_isr();
    app_poll();
    assert(upload_calls == previous_uploads + (scenario != 1 && (scenario != 3 || wifi_calls > 1)));
    previous_uploads = upload_calls;
    assert(log_packets == previous_logs + logs_waiting); /* STOP/reconnect logs leave while parked. */
    previous_logs = log_packets;
    for (i = 0; i < 200; i++) app_tick_isr();
    app_poll();
    assert(log_packets == previous_logs); /* No repeated event logs after cache drains. */
    assert(upload_calls == previous_uploads + (scenario != 1 && (scenario != 3 || wifi_calls > 1)));
    previous_uploads = upload_calls;
    if (allowed)
    {
        command = 1;
        app_poll();
        assert(car_state == CAR_RUNNING && upload_calls == previous_uploads + 1);
        if (scenario == 5)
        {
            previous_wifi=wifi_calls;
            mock_online=0; // Actual WiFi layer reports repeated runtime SPI failures.
            app_poll();
            for(i=0;i<1001;i++)app_tick_isr();
            app_poll();
            assert(car_state==CAR_RUNNING && wifi_calls==previous_wifi);
            command=2;app_keys_isr();app_poll();
            assert(car_state==CAR_IDLE && wifi_calls==previous_wifi+1 && mock_online);
        }
    }
    puts("PASS: sensor ISR, K1/K2 motion control, parked IMU service, independent init/STOP logs, failed log retry, parked reconnect");
    return 0;
}
