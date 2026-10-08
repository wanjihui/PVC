/* 现场按键：K1事件锁存后由主循环发车；K2在10ms扫描中断直接停车。
 * 库短按事件只保留一拍，必须在扫描后立即存入pending_events。
 * ISR不打印、不访问WiFi；状态日志仍由app主循环提交。 */
#include "buttons.h"
#include "config.h"
#include "control.h"

#define BUTTON_EVENT_START (1u)
#define BUTTON_EVENT_STOP  (2u)
static volatile uint8 pending_events;
static volatile uint16 stop_press_ms;
static uint8 stop_press_handled;
static const gpio_pin_enum button_pins[KEY_NUMBER] = KEY_LIST;

void buttons_init(void)
{
    pending_events = 0;
    stop_press_ms = 0;
    stop_press_handled = 0;
    key_init(KEY_SCAN_PERIOD_MS);
}

/* 10ms PIT：先保存库事件，再做K2按下消抖。普通按下连续20ms即清PWM，
 * 不等待松手或主循环/WiFi。若短按不足20ms但被库识别，松手事件仍可停车。 */
void buttons_scan_isr(void)
{
    uint8 i, start = 0, stop = 0;
    uint8 stop_down = (gpio_get_level(button_pins[KEY_2]) != KEY_RELEASE_LEVEL);
    key_state_enum state;
    key_scanner();
    for (i = 0; i < KEY_NUMBER; i++)
    {
        state = key_get_state((key_index_enum)i);
        if (state == KEY_RELEASE) continue;
        key_clear_state((key_index_enum)i);
        if (i == KEY_1 && state == KEY_SHORT_PRESS) start = 1; // K1短按松开后发车
        if (i == KEY_2 && !stop_press_handled) stop = 1;
    }
    if (stop_down)
    {
        if (stop_press_ms < KEY_STOP_DEBOUNCE_MS) stop_press_ms += KEY_SCAN_PERIOD_MS;
        if (stop_press_ms >= KEY_STOP_DEBOUNCE_MS && !stop_press_handled)
        {
            stop = 1;
            stop_press_handled = 1;
        }
        start = 0; // 按住K2时丢弃K1，不允许重新开速度环
        pending_events &= BUTTON_EVENT_STOP;
    }
    else
    {
        stop_press_ms = 0;
        stop_press_handled = 0;
    }
    if (stop)
    {
        pending_events = BUTTON_EVENT_STOP; // 丢弃先前待处理K1，停车优先
        control_state_stop(STOP_REASON_MANUAL); // 只清内存/目标/PWM，没有等待或通信
    }
    else if (start && !(pending_events & BUTTON_EVENT_STOP)) pending_events |= BUTTON_EVENT_START;
}

void buttons_poll(void)
{
    uint8 events;
    uint32 irq_state = interrupt_global_disable();
    events = pending_events;
    pending_events = 0;
    /* 取事件与提交发车在同一临界区，防止K2刚停完、旧K1又开启PID。
     * control_state_launch只复位内存/目标，不执行延时、SPI或网络通信。 */
    if (events & BUTTON_EVENT_STOP) control_state_stop(STOP_REASON_MANUAL);
    else if ((events & BUTTON_EVENT_START) && stop_press_ms == 0) control_state_launch();
    interrupt_global_enable(irq_state);
}
