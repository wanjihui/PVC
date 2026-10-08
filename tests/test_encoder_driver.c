#include "../code/encoder/encoder.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
static uint16 positions[WHEEL_NUM];
static int clears;
void encoder_quad_init(int unit, int a, int b) { (void)unit; (void)a; (void)b; }
void encoder_clear_count(int unit) { positions[unit] = 0; clears++; }
int16 encoder_get_count(int unit) { return (int16)positions[unit]; }
uint32 interrupt_global_disable(void) { return 0; }
void interrupt_global_enable(int state) { (void)state; }
int main(void)
{
    encoder_snapshot_t values;
    encoder_init();
    assert(clears == 4);
    positions[0] = 10;
    positions[1] = positions[2] = (uint16)-10;
    positions[3] = 10;
    encoder_pit_callback();
    encoder_snapshot(&values);
    assert(values.sample_count == 1);
    assert(values.total[0] == 10 && values.total[1] == 10 && values.total[2] == 10 && values.total[3] == 10);
    assert(encoder_get_pulse(WHEEL_FR) == 10);
    /* Independent physical reference: 1024 lines, x1, wheel/encoder gears 68/30, 19.5cm/rev. */
    assert(fabsf(encoder_get_counts_per_cm() - 119.02906f) < 0.0001f);
    assert(fabsf(encoder_get_speed_cms(WHEEL_FR) - 16.802628f) < 0.00001f);
    assert(fabsf(encoder_get_distance_cm() - 0.08401314f) < 0.000001f);
    assert(clears == 4); /* Sampling must not clear pulses that arrive after the read. */
    positions[0] += 3;
    positions[1] -= 3;
    positions[2] -= 3;
    positions[3] += 3;
    encoder_reset_distance(); /* Start reset must preserve lifetime telemetry counts. */
    encoder_pit_callback();
    encoder_snapshot(&values);
    assert(values.sample_count == 2);
    assert(values.total[0] == 13 && values.total[1] == 13 && values.total[2] == 13 && values.total[3] == 13);
    assert(clears == 4);
    assert(fabsf(encoder_get_distance_cm() - 0.025203944f) < 0.000001f);
    encoder_set_counts_per_cm(100.0f);
    assert(encoder_get_counts_per_cm() == 100.0f);
    assert(encoder_get_speed_cms(WHEEL_FL) == 6.0f);
    encoder_set_counts_per_cm(0.0f);
    encoder_set_counts_per_cm(-1.0f);
    encoder_set_counts_per_cm(NAN);
    encoder_set_counts_per_cm(INFINITY);
    assert(encoder_get_counts_per_cm() == 100.0f);
    /* The 16-bit peripheral wraps while software cumulative counts continue. */
    positions[0] = 32760; positions[1] = positions[2] = (uint16)-32760; positions[3] = 32760;
    encoder_pit_callback();
    positions[0] += 20; positions[1] -= 20; positions[2] -= 20; positions[3] += 20;
    encoder_pit_callback();
    encoder_snapshot(&values);
    assert(values.total[0] == 32780 && values.total[1] == 32780 && values.total[2] == 32780 && values.total[3] == 32780);
    assert(encoder_get_speed_cms(WHEEL_FL) == 40.0f);
    assert(clears == 4);
    /* Independent wiring fixture: peripheral 1 is physical FL (raw negative),
     * peripheral 0 is physical FR (raw positive). One wheel at a time. */
    {
        encoder_snapshot_t before = values;
        positions[1] -= 11;
        encoder_pit_callback();
        encoder_snapshot(&values);
        assert(values.total[WHEEL_FL] == before.total[WHEEL_FL] + 11);
        assert(values.total[WHEEL_FR] == before.total[WHEEL_FR]);
        assert(values.total[WHEEL_RL] == before.total[WHEEL_RL]);
        assert(values.total[WHEEL_RR] == before.total[WHEEL_RR]);
        before = values;
        positions[0] += 7;
        encoder_pit_callback();
        encoder_snapshot(&values);
        assert(values.total[WHEEL_FR] == before.total[WHEEL_FR] + 7);
        assert(values.total[WHEEL_FL] == before.total[WHEEL_FL]);
        assert(values.total[WHEEL_RL] == before.total[WHEEL_RL]);
        assert(values.total[WHEEL_RR] == before.total[WHEEL_RR]);
    }
    assert(clears == 4);
    puts("PASS: physical FL/FR mapping and sign, cm scale, physical reference, calibration, signed wrap, no destructive clear");
    return 0;
}
