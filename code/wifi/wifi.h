#ifndef MCAR_WIFI_H
#define MCAR_WIFI_H
#include "zf_common_headfile.h"
#include "config.h"
/* 0成功；1模块/SPI失败，2入网失败，3UDP配置失败，4凭据异常，5异步DMA配置异常。 */
uint8 wifi_init(void);
void wifi_poll(uint32 now_ms, uint8 new_camera_frame); // 新图像与参数包独立上传；停车仍有IMU，目标/PWM归零
#if APP_IMU_VERIFY_ENABLE
/* IMU 符号验证模式专用：按 WIFI_DATA_PERIOD_MS 提交只含 IMU 的参数包，不依赖 car_state。
 * 参数包仍是 148 字节，前 36 字节与正常模式布局一致，上位机用同一解析器。 */
void wifi_poll_imu_verify(uint32 now_ms);
#endif
void wifi_cancel_camera(void); // 显式禁用摄像头才取消半帧，K1/K2不取消图像
void wifi_cancel_data(void);   // 显式停用通信才取消参数；K2不取消IMU
void wifi_record_service_cost(uint32 elapsed_us); // 主循环上一轮WiFi服务耗时/上电以来峰值
/* 无论停车/运行都调用：推进异步发送，包括参数/图像，停车也有IMU参数、图像和必要日志。 */
void wifi_poll_log(uint32 now_ms);
/* 主循环事件日志同时输出UART和8KB待发缓存；不得在中断调用。 */
int wifi_log(const char *format, ...);
uint8 wifi_flush_log(void);
/* wifi_flush_log返回0表示请求已接受；主循环继续poll才真正提交，不同步等待。 */
#endif
