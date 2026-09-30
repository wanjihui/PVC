/*********************************************************************************************************************
* 文件名称          isr.h
* 适用平台          RT1064DVL6A
* 描述              中断服务函数声明
*********************************************************************************************************************/

#ifndef __ISR_H_
#define __ISR_H_

#include "zf_common_headfile.h"

/* ---- 业务处理函数（由 PIT 中断调用） ---- */
void pit_handler_200hz(void);                                               // PIT_CH0：编码器 + 速度环 + IMU
void pit_handler_100hz(void);                                               // PIT_CH1：按键扫描

/* ---- 中断向量服务函数 ----
* 这些名字由启动文件 startup_MIMXRT1064.S 的向量表引用，必须与库里保持一致；
* 在这里显式声明是为了让"定义与声明成对"，也便于工具检查。 */
void PIT_IRQHandler(void);                                                  // 周期中断（CH0/CH1/CH2/CH3 共用）
void CSI_IRQHandler(void);                                                  // 摄像头 CSI 场中断
void LPUART1_IRQHandler(void);                                              // 调试串口
void LPUART6_IRQHandler(void);                                              // VOFA 遥测串口

#endif
