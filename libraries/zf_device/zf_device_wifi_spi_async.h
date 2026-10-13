#ifndef MCAR_WIFI_SPI_ASYNC_H
#define MCAR_WIFI_SPI_ASYNC_H
#include "zf_common_typedef.h"

/* RT1064版：参考CYT4BB7_Air的提交/poll/DMA思想，不改变原同步API语义。
 * 仅主循环提交；DMA回调只释放CS和记录完成。状态机不调用延时或等待循环。 */
typedef enum
{
    WIFI_SPI_ASYNC_OK = 0,       // SPI提交完成且自动发包窗口结束，不等于电脑确认收到
    WIFI_SPI_ASYNC_BUSY = 1,     // 等待INT、DMA、模块应答或自动发包窗口；下轮继续poll
    WIFI_SPI_ASYNC_TIMEOUT = 2,  // 时间期限已到，事务已中止
    WIFI_SPI_ASYNC_PROTOCOL = 3, // 非法模块应答或长度
    WIFI_SPI_ASYNC_DMA = 4,      // DMA启动/执行失败
    WIFI_SPI_ASYNC_CANCELLED = 5 // 主动停车取消，下次先清理socket
} wifi_spi_async_result_enum;

/* 初始化DMA句柄与socket配置快照；原库入网/初始socket成功后调用。
 * ip/local_port/initial_port为已验证配置，返回0成功，1参数/通道占用异常。 */
uint8 wifi_spi_async_init(const char *ip, const char *local_port, const char *initial_port);
/* 复制一包到内部缓存并立即返回：0已接受，1忙/未就绪，2参数非法。
 * length=1..1424，port为电脑UDP目标端口，now_ms为持续更新的板上毫秒时间。
 * 接受不等于发送完成；必须poll到is_busy=0，再查询send_result。 */
uint8 wifi_spi_send_async(const uint8 *buffer, uint16 length, const char *port, uint32 now_ms);
uint8 wifi_spi_receive_async(uint32 now_ms); // 空闲时提交只读事务，不发UDP空包
uint32 wifi_spi_read_async_fifo(uint8 *buffer, uint32 capacity); // 仅FIFO，不同步SPI
void wifi_spi_send_poll(uint32 now_ms); // 消费完成应答后可直接启动后继DMA，每次最多启动一次，不原地等待
uint8 wifi_spi_is_busy(void); // 1有在途事务，0可提交
uint8 wifi_spi_send_result(void); // 最近事务的完成状态，取值见上方enum
void wifi_spi_send_cancel(void); // 取消在途事务；不发送新UDP包，不同步重连

void wifi_spi_refresh_socket(void); // 非阻塞：下一次发送强制重建UDP socket
#endif
