/* 与固件共用计数内核，用已知正反向位置序列验证回绕与累计。 */
#include <stdio.h>
#include <stdlib.h>
#include "encoder_counter.h"

static unsigned int checks;

static void equal_value(const char *label, int64_t actual, int64_t expected)
{
    checks++;
    if (actual != expected)
    {
        fprintf(stderr, "FAIL %s: actual %lld expected %lld\n", label,
                (long long)actual, (long long)expected);
        exit(1);
    }
}

int main(void)
{
    uint32_t overflow = 0;
    int32_t total = 0;
    uint16_t previous = 0;
    unsigned int i;
    equal_value("stationary", encoder_counter_delta(100, 100, 0), 0);
    equal_value("forward", encoder_counter_delta(130, 100, 0), 30);
    equal_value("reverse", encoder_counter_delta(70, 100, 0), -30);
    equal_value("forward wrap", encoder_counter_delta(5, 65530, 0), 11);
    equal_value("reverse wrap", encoder_counter_delta(65530, 5, 0), -11);
    equal_value("signed bit forward", encoder_counter_delta(32770, 32760, 0), 10);
    equal_value("signed bit reverse", encoder_counter_delta(32760, 32770, 0), -10);
    equal_value("direction inversion", encoder_counter_delta(70, 100, 1), 30);
    equal_value("negative signed limit widened", encoder_counter_delta(32768, 0, 1), 32768);

    /* 真实位置始终前进，经过多个16位回绕；显示应保持累计100000而不是回到低16位。 */
    for (i = 1; i <= 1000; i++)
    {
        uint16_t current = (uint16_t)(i * 100u);
        total = encoder_counter_add(total, encoder_counter_delta(current, previous, 0), &overflow);
        previous = current;
    }
    equal_value("multiple wraps cumulative", total, 100000);
    equal_value("no overflow", overflow, 0);
    for (i = 1000; i > 0; i--)
    {
        uint16_t current = (uint16_t)((i - 1u) * 100u);
        total = encoder_counter_add(total, encoder_counter_delta(current, previous, 0), &overflow);
        previous = current;
    }
    equal_value("reverse returns to origin", total, 0);

    equal_value("positive saturation", encoder_counter_add(INT32_MAX - 3, 5, &overflow), INT32_MAX);
    equal_value("positive overflow reported", overflow, 1);
    overflow = 0;
    equal_value("negative saturation", encoder_counter_add(INT32_MIN + 3, -5, &overflow), INT32_MIN);
    equal_value("negative overflow reported", overflow, 1);
    printf("PASS: %u encoder counter checks\n", checks);
    return 0;
}
