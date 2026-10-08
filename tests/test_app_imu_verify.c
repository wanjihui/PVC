/* Actual app.c verification branch; peripherals are deterministic mocks. */
#include "fake_platform.h"
#include <string.h>
#include <stdlib.h>
volatile CarState car_state;
volatile uint8 Speed_PID_Enable;
char wifi_spi_ip_addr_port[]="mock";
static uint32 clock_us;
static uint8 clock_running;
static int scenario, motor_initializations, imu_initializations, imu_reads, wifi_initializations;
static int uploads, service_polls, init_logs, log_flushes;
void clock_init(int x) {(void)x;}
void debug_init(void) {}
void motor_init(void) {motor_initializations++;}
void interrupt_global_enable(int x) {(void)x;}
void system_delay_ms(int ms) {if(clock_running)clock_us+=(uint32)ms*1000u;}
int wifi_log(const char *format,...) {(void)format;init_logs++;return 0;}
uint8 imu_init(void) {imu_initializations++;return scenario==1 ? 1 : 0;}
void imu_update(void) {imu_reads++;assert(car_state==CAR_IDLE && !Speed_PID_Enable);}
void imu_snapshot(imu_snapshot_t *sample) {memset(sample,0,sizeof(*sample));}
float imu_get_gyro_z_dps(void) {return 0;}
void timer_init(int timer,int mode) {(void)timer;(void)mode;clock_us=0;clock_running=0;}
void timer_start(int timer) {(void)timer;clock_running=1;}
void timer_clear(int timer) {(void)timer;if(clock_running)clock_us=0;}
uint32 timer_get(int timer) {(void)timer;return clock_us;}
uint8 wifi_init(void) {wifi_initializations++;return scenario==2 && wifi_initializations==1 ? 2 : 0;}
uint8 wifi_flush_log(void) {log_flushes++;return 0;}
void wifi_poll_imu_verify(uint32 ms) {(void)ms;uploads++;assert(car_state==CAR_IDLE && !Speed_PID_Enable);}
void wifi_poll_log(uint32 ms) {(void)ms;service_polls++;}
void wifi_record_service_cost(uint32 us) {(void)us;}
int main(int argc,char **argv) {
    scenario=argc>1 ? atoi(argv[1]):0;
    app_init();
    assert(clock_running); // Missing timer_start reproduces frozen sampling/telemetry timestamps.
    assert(car_state==CAR_IDLE && !Speed_PID_Enable);
    assert(motor_initializations==1 && imu_initializations==1 && init_logs>=4);
    assert(log_flushes==(scenario!=2));
    app_tick_isr();app_keys_isr();assert(!imu_reads); // no second sampling path
    for(int n=0;n<1000;n++)app_poll();
    assert(imu_reads==(scenario==1 ? 0 : 199));
    assert(uploads==(scenario==2 ? 0 : 1000) && uploads==service_polls);
    if(scenario==2) {
        for(int n=0;n<4100;n++)app_poll();
        assert(wifi_initializations==2 && uploads>0);
    }
    assert(car_state==CAR_IDLE && !Speed_PID_Enable);
    puts("PASS: IMU-only actual app, main 5ms sampling, no PIT duplicate, idle auto WiFi, no motor enable, init failure and retry");
    return 0;
}
