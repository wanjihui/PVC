/* PIT调用应用固定周期任务；CSI/UART等库中断入口保留。禁止在中断打印或发送WiFi。 */

#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"

#include "app.h"

void pit_handler_200hz(void) { app_tick_isr(); }
void pit_handler_100hz(void) { app_keys_isr(); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 周期中断服务函数
// 备注信息     CH0 与 CH1 共用一个中断向量，靠 pit_flag_get 区分
//-------------------------------------------------------------------------------------------------------------------
void PIT_IRQHandler(void)
{
    if (pit_flag_get(PIT_CH0))
    {
        pit_flag_clear(PIT_CH0);
        pit_handler_200hz();
    }

    if (pit_flag_get(PIT_CH1))
    {
        pit_handler_100hz();
        pit_flag_clear(PIT_CH1);
    }

    if (pit_flag_get(PIT_CH2))
    {
        pit_flag_clear(PIT_CH2);
    }

    if (pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}

//-------------------------------------------------------------------------------------------------------------------
// 摄像头 CSI 中断：SDK 内部会回调 mt9v03x 的场中断处理，最终置位 mt9v03x_finish_flag
//-------------------------------------------------------------------------------------------------------------------
void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();
    __DSB();                                                                // 数据同步隔离
}

//-------------------------------------------------------------------------------------------------------------------
// 调试串口（LPUART1，B12/B13）
//-------------------------------------------------------------------------------------------------------------------
void LPUART1_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
    #if DEBUG_UART_USE_INTERRUPT                                            // 如果开启 debug 串口中断
        debug_interrupr_handler();                                          // 数据进 debug 环形缓冲区
    #endif
    }

    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);                // 不允许删除
}

//-------------------------------------------------------------------------------------------------------------------
// 遥测串口（LPUART6，B2/B3）：本工程只发不收，仅清标志
//-------------------------------------------------------------------------------------------------------------------
void LPUART6_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        /* 预留：如果以后要接收上位机指令，在这里解析 */
    }

    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);                // 不允许删除
}
