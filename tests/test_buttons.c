/* Real key scanner + real buttons.c; simulate a blocked main loop. */
#include "fake_platform.h"
static uint8 levels[16];
static uint32 irq_mask;
static int starts, stops, pwm, running;
uint32 interrupt_global_disable(void) { uint32 old=irq_mask; irq_mask=1; return old; }
void interrupt_global_enable(int state) { irq_mask=(uint32)state; }
void gpio_init(gpio_pin_enum pin, int mode, int level, int pull)
{ (void)mode; (void)pull; levels[pin]=(uint8)level; }
uint8 gpio_get_level(gpio_pin_enum pin) { return levels[pin]; }
void control_state_launch(void)
{ assert(irq_mask==1); starts++; running=1; pwm=15; }
void control_state_stop(uint8 reason)
{ assert(reason==STOP_REASON_MANUAL); stops++; running=0; pwm=0; }
static void scan_many(int count) { while(count--) buttons_scan_isr(); }
static void press_k1(void)
{ levels[15]=0; scan_many(3); levels[15]=1; buttons_scan_isr(); }
int main(void)
{
    int previous_stops;
    buttons_init();
    /* Reproduce original library event expiry without application latch. */
    levels[14]=0; key_scanner();
    levels[14]=1; key_scanner();
    assert(key_get_state(KEY_2)==KEY_SHORT_PRESS);
    key_scanner();
    assert(key_get_state(KEY_2)==KEY_RELEASE);

    buttons_init();
    press_k1(); scan_many(200); /* Main unavailable for 2 seconds. */
    assert(starts==0);
    buttons_poll(); assert(starts==1 && running && pwm==15 && irq_mask==0);
    buttons_poll(); assert(starts==1);

    levels[14]=0; buttons_scan_isr(); assert(running);
    buttons_scan_isr(); assert(!running && pwm==0); /* K2 stops while main is still blocked. */
    previous_stops=stops;
    scan_many(200); assert(stops==previous_stops); /* Held K2 does not repeatedly reset. */
    press_k1(); buttons_poll(); assert(starts==1 && !running); /* K2 held blocks K1. */
    levels[14]=1; scan_many(200); buttons_poll();
    assert(!running && pwm==0);
    press_k1(); buttons_poll(); assert(starts==2 && running);

    /* Both events queued: K2 supersedes the earlier K1. */
    press_k1(); levels[14]=0; scan_many(2);
    levels[14]=1; scan_many(200); buttons_poll();
    assert(starts==2 && !running && pwm==0 && irq_mask==0);

    /* Very short press: release event also stops and persists until consumed. */
    press_k1(); buttons_poll(); assert(starts==3 && running);
    levels[14]=0; buttons_scan_isr(); assert(running);
    levels[14]=1; buttons_scan_isr(); assert(!running && pwm==0);
    scan_many(200); buttons_poll(); assert(starts==3 && !running);
    puts("PASS: real library 10ms event expiry reproduced; latched K1, ISR K2 stop with blocked main, held-key priority, restart, short K2");
    return 0;
}
