/* 编码器测试计数内核：16位自由计数器差分，正确处理正反向回绕。
 * 前提：相邻采样间实际净位移少于32768个计数；5ms手转测试满足该条件。
 * 累计量饱和时设置标志，避免有符号整数溢出。独立于硬件，可在电脑验证。 */
#ifndef MCAR_ENCODER_COUNTER_H
#define MCAR_ENCODER_COUNTER_H

#include <stdint.h>
#include <limits.h>

static inline int32_t encoder_counter_delta(uint16_t current, uint16_t previous, int invert)
{
    uint16_t difference = (uint16_t)(current - previous);
    int32_t delta = difference >= 32768u ? (int32_t)difference - 65536 : (int32_t)difference;
    return invert ? -delta : delta;
}

static inline int32_t encoder_counter_add(int32_t total, int32_t delta, uint32_t *overflow)
{
    if (delta > 0 && total > INT32_MAX - delta)
    {
        *overflow = 1;
        return INT32_MAX;
    }
    if (delta < 0 && total < INT32_MIN - delta)
    {
        *overflow = 1;
        return INT32_MIN;
    }
    return total + delta;
}

#endif
