/* WiFi主循环通信：参数保留最新一包，图像分片逐包提交，日志成功后才出缓存。
 * 参考CYT4BB7_Air的提交/poll模式；运行期不使用逐飞同步发送或socket切换。
 * 初始化入网保留原库，传感器/电机/PIT不由通信模块调度。 */
#include "wifi.h"
#include "config.h"
#include "imu.h"
#include "encoder.h"
#include "pid.h"
#include "control.h"
#include "vision.h"
#include "zf_device_wifi_spi_async.h"
#include "monitor_protocol.h"
#include <stdarg.h>
#if WIFI_SPI_AUTO_CONNECT != 0
#error "Application manages UDP sockets; disable library auto-connect."
#endif

static uint8 monitor_ready_flag;
static char monitor_log_buffer[8192]; // 待发送文本，失败保留；启动日志不受K1限制
static uint32 monitor_log_length, monitor_dropped;
static uint8 monitor_image[VISION_MONITOR_SIZE]; // 在途图像快照，不被后续视觉覆盖
static uint8 monitor_packet[MONITOR_CAMERA_HEADER_SIZE + MONITOR_CAMERA_CHUNK_SIZE];
static uint8 data_packet[MONITOR_DATA_SIZE]; // 只保留最新参数，避免积压旧轮速
static uint8 camera_active, pending_camera, pending_data, log_requested;
static uint8 camera_burst_count;
static uint32 camera_burst_at;
static uint16 camera_index, tx_log_length;
static uint32 camera_crc, camera_frame_id, camera_started;
static uint32 data_at, image_at, log_at, sequence;
static uint32 service_last_us, service_peak_us, retry_at;
static uint8 backoff, last_error;
typedef enum { MONITOR_TX_NONE, MONITOR_TX_DATA, MONITOR_TX_CAMERA, MONITOR_TX_LOG } monitor_tx_kind_enum;
static monitor_tx_kind_enum tx_kind; // 单socket一次只允许一个完整UDP事务

void wifi_cancel_camera(void)
{
    camera_active = pending_camera = 0;
    /* 仅显式禁用摄像头时取消图像；K1/K2不调用本函数。 */
    if (tx_kind == MONITOR_TX_CAMERA)
    {
        wifi_spi_send_cancel();
        tx_kind = MONITOR_TX_NONE;
    }
}
void wifi_cancel_data(void)
{
    pending_data = 0;
    if (tx_kind == MONITOR_TX_DATA)
    {
        wifi_spi_send_cancel();
        tx_kind = MONITOR_TX_NONE;
    }
}
void wifi_record_service_cost(uint32 elapsed_us)
{
    service_last_us = elapsed_us;
    if (elapsed_us > service_peak_us) service_peak_us = elapsed_us;
}
int wifi_log(const char *format, ...)
{
    char text[512];
    int result;
    uint32 length;
    va_list args;
    va_start(args, format);
    result = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (result < 0) return result;
    length = (uint32)result;
    if (length >= sizeof(text))
    {
        length = sizeof(text) - 1u;
        text[length - 2u] = '\r'; text[length - 1u] = '\n';
        monitor_dropped += (uint32)result - length;
    }
    printf("%s", text); // UART备份仍是同步printf，只输出低频必要事件
    if (length <= sizeof(monitor_log_buffer) - monitor_log_length)
    {
        memcpy(monitor_log_buffer + monitor_log_length, text, length);
        monitor_log_length += length;
    }
    else monitor_dropped += length;
    return result;
}

/* 只请求提交，不在这里等待；app_init之后的主循环负责推进并补发全部缓存。 */
uint8 wifi_flush_log(void)
{
    if (!monitor_ready_flag) return 1;
    if (monitor_log_length || monitor_dropped) log_requested = 1;
    return 0;
}

uint8 wifi_init(void)
{
    const char *password = WIFI_PASSWORD;
    uint32 credentials_length = (uint32)strlen(WIFI_SSID) + 2u;
    wifi_spi_send_cancel();
    monitor_ready_flag = 0;
    tx_kind = MONITOR_TX_NONE;
    camera_active = pending_camera = pending_data = backoff = 0;
    if (password) credentials_length += (uint32)strlen(password) + 2u;
    if (WIFI_SSID[0] == '\0' || credentials_length >= 64u) return 4;
    /* 同步初始化只允许上电/停车重试；运行期不调用这些函数。 */
    if (wifi_spi_init(NULL, NULL)) return 1;
    if (wifi_spi_wifi_connect(WIFI_SSID, (char *)password)) return 2;
    if (wifi_spi_socket_connect("UDP", WIFI_UDP_TARGET_IP, WIFI_DATA_PORT, WIFI_UDP_LOCAL_PORT)) return 3;
    if (wifi_spi_async_init(WIFI_UDP_TARGET_IP, WIFI_UDP_LOCAL_PORT, WIFI_DATA_PORT)) return 5;
    monitor_ready_flag = 1;
    return 0;
}

/* 只准备参数/图像，不等待网络。单包被接受后驱动复制缓存，下一拍可继续更新参数。 */
void wifi_poll(uint32 now, uint8 new_camera_frame)
{
#if !APP_IMU_VERIFY_ENABLE
    imu_snapshot_t imu;
    encoder_snapshot_t enc;
    speed_pid_snapshot_t speed;
    float commands[4] = {0, 0, 0, 0};
    uint32 timing[6] = {0, 0, 0, 0, 0, 0};

#endif
    if (!monitor_ready_flag) return;
    if (new_camera_frame) pending_camera = 1;
#if APP_IMU_VERIFY_ENABLE
    wifi_poll_imu_verify(now); // 可选只采IMU入口，不读取未初始化编码器/PID。
#else
    if ((uint32)(now - data_at) >= WIFI_DATA_PERIOD_MS)
    {
        data_at = now;
        imu_snapshot(&imu);
        // IMU正常就上传；K1/K2仅控制运动。停车仍上传IMU/累计计数；速度环状态复位，目标和PWM为0。
        if (imu.ready)
        {
            encoder_snapshot(&enc);
            Speed_PID_Snapshot(&speed);
            memcpy(data_packet, imu.angles, 12);
            memcpy(data_packet + 12, imu.gyro_dps, 12);
            memcpy(data_packet + 24, imu.acc_g, 12);
            memcpy(data_packet + 36, enc.total, 16);
            monitor_put_u32(data_packet + 52, enc.sample_count);
            monitor_put_u32(data_packet + 56, sequence++); // 队列替换/发送失败也会产生seq缺号
            monitor_speed_fields(data_packet, speed.target, speed.actual, speed.duty);
#if APP_VISION_TRACKING_ENABLE
            commands[0] = g_vis.err; commands[1] = omega_dbg;
            commands[2] = speed_target_dbg; commands[3] = vy_dbg;
            timing[0] = vis_time_us; timing[1] = vis_interval_us;
            timing[2] = vis_processed_frames; timing[5] = vis_last_event;
#endif
            timing[3] = service_last_us; timing[4] = service_peak_us;
            monitor_vision_fields(data_packet, commands, timing);
            pending_data = 1;
        }
        else pending_data = 0; // 初始化失败只发日志，不用NaN伪造参数。
    }
#endif
    if (!camera_active && pending_camera && (uint32)(now - image_at) >= WIFI_IMAGE_PERIOD_MS)
    {
        image_at = now;
        pending_camera = 0;
        vision_monitor_payload(monitor_image);
        camera_crc = monitor_crc32(monitor_image, VISION_MONITOR_SIZE);
        camera_frame_id = vis_processed_frames;
        camera_index = 0;
        camera_burst_count = 0;
        camera_started = now;
        camera_active = 1;
    }
}

#if APP_IMU_VERIFY_ENABLE
/* IMU 符号验证：不依赖 car_state，只提交 angles/gyro_dps/acc_g 三个三元组。
 * 参数包仍是 148 字节且前 36 字节与正常模式完全一致，上位机用同一解析器即可。
 * 编码器/轮速/视觉字段填0；保留原有字段含义，采样计数使用imu.sample_count。 */
void wifi_poll_imu_verify(uint32 now)
{
    imu_snapshot_t imu;
    float commands[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float zero4[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint32 timing[6] = {0, 0, 0, 0, 0, 0};

    if (!monitor_ready_flag) return;
    if ((uint32)(now - data_at) < WIFI_DATA_PERIOD_MS) return;
    data_at = now;

    imu_snapshot(&imu);
    // 未就绪只保留初始化错误日志；不发送NaN充数。模块正常后自动上传，无K1/K2门控。
    if (!imu.ready) { pending_data = 0; return; }
    memcpy(data_packet, imu.angles, 12);        // 0   : roll/pitch/yaw (deg)
    memcpy(data_packet + 12, imu.gyro_dps, 12); // 12  : 三轴去零偏角速度 (deg/s)
    memcpy(data_packet + 24, imu.acc_g, 12);    // 24  : 三轴加速度 (g)
    memset(data_packet + 36, 0, 16);            // 36  : 编码器累计（验证模式未初始化，固定 0）
    monitor_put_u32(data_packet + 52, imu.sample_count);
    monitor_put_u32(data_packet + 56, sequence++);
    monitor_speed_fields(data_packet, zero4, zero4, zero4);
    timing[3] = service_last_us;
    timing[4] = service_peak_us;
    monitor_vision_fields(data_packet, commands, timing);
    pending_data = 1;
}
#endif

/* 推进一个DMA/模块应答步骤，再处理完成结果；不调用wait_idle或等待循环。
 * 从app_poll无论停车/运行都调用，保证初始化/STOP日志可以继续传输。 */
void wifi_poll_log(uint32 now)
{
    uint8 result, camera_priority;
    uint16 count, length;
    uint32 offset;
    if (!monitor_ready_flag) return;
    // 两种入口均独立上传IMU；停止电机不取消参数包、图像或必要日志。
    if (camera_active && (uint32)(now - camera_started) > 600u)
    {
        camera_active = 0;
        if (tx_kind == MONITOR_TX_CAMERA)
        {
            wifi_spi_send_cancel();
            tx_kind = MONITOR_TX_NONE;
        }
    }
    if (tx_kind != MONITOR_TX_NONE)
    {
        wifi_spi_send_poll(now);
        if (wifi_spi_is_busy()) return;
        result = wifi_spi_send_result();
        if (result != WIFI_SPI_ASYNC_OK)
        {
            if (tx_kind == MONITOR_TX_CAMERA) camera_active = 0;
            retry_at = now;
            backoff = 1; // 故障退避1秒；视觉/内环继续，不反复同步等待模块
            if (result != last_error) wifi_log("[ERR] UDP async result=%u kind=%u\r\n", (unsigned int)result, (unsigned int)tx_kind);
            last_error = result;
        }
        else
        {
            if (tx_kind == MONITOR_TX_LOG)
            {
                monitor_log_length -= tx_log_length; // SPI提交和自动发包窗口结束后才删除文本（UDP无电脑确认）
                memmove(monitor_log_buffer, monitor_log_buffer + tx_log_length, monitor_log_length);
                if (!monitor_log_length && monitor_dropped)
                {
                    uint32 dropped = monitor_dropped;
                    monitor_dropped = 0;
                    wifi_log("[WARN] log cache dropped %lu bytes\r\n", (unsigned long)dropped);
                }
                log_requested = (monitor_log_length != 0);
            }
            else if (tx_kind == MONITOR_TX_CAMERA)
            {
                count = (uint16)((VISION_MONITOR_SIZE + MONITOR_CAMERA_CHUNK_SIZE - 1u) / MONITOR_CAMERA_CHUNK_SIZE);
                if (camera_burst_count == 0) camera_burst_at = now;
                camera_burst_count++;
                if (++camera_index == count) camera_active = 0;
            }
            else if (tx_kind == MONITOR_TX_DATA) camera_burst_count = 0;
            if (last_error) wifi_log("[NET] UDP async recovered\r\n");
            last_error = 0;
        }
        tx_kind = MONITOR_TX_NONE;
    }
    if (backoff)
    {
        if ((uint32)(now - retry_at) < 1000u) return;
        backoff = 0;
    }
    if (!log_requested && monitor_log_length && (uint32)(now - log_at) >= WIFI_LOG_PERIOD_MS)
    {
        log_at = now;
        log_requested = 1;
    }
    /* 必要日志最高优先级。图像最多连续4片/首片完成后20ms，然后让出最新参数。
     * 避免持续更新参数时图像饿死，也避免每一片都来回重建socket。 */
    camera_priority = camera_active && (camera_burst_count == 0 ||
                       (camera_burst_count < WIFI_CAMERA_BURST_CHUNKS &&
                        (uint32)(now - camera_burst_at) < WIFI_CAMERA_BURST_MS));
    /* 一次只提交一包，保持参数最新候选、图像当前快照。
     * 接受只表示复制入驱动，下一圈poll启动DMA，不能立刻切换端口或复用事务。 */
    if (log_requested && monitor_log_length)
    {
        tx_log_length = (uint16)(monitor_log_length > 1000u ? 1000u : monitor_log_length);
        if (!wifi_spi_send_async((const uint8 *)monitor_log_buffer, tx_log_length, WIFI_LOG_PORT, now)) tx_kind = MONITOR_TX_LOG;
    }
    else if (pending_data && !camera_priority) // 参数与图像公平调度，保持148字节协议。
    {
        if (!wifi_spi_send_async(data_packet, sizeof(data_packet), WIFI_DATA_PORT, now))
        {
            pending_data = 0;
            tx_kind = MONITOR_TX_DATA;
        }
    }
    else if (camera_active)
    {
        if (camera_burst_count >= WIFI_CAMERA_BURST_CHUNKS ||
            (uint32)(now - camera_burst_at) >= WIFI_CAMERA_BURST_MS) camera_burst_count = 0;
        count = (uint16)((VISION_MONITOR_SIZE + MONITOR_CAMERA_CHUNK_SIZE - 1u) / MONITOR_CAMERA_CHUNK_SIZE);
        offset = (uint32)camera_index * MONITOR_CAMERA_CHUNK_SIZE;
        length = (uint16)((VISION_MONITOR_SIZE - offset) > MONITOR_CAMERA_CHUNK_SIZE ?
                           MONITOR_CAMERA_CHUNK_SIZE : (VISION_MONITOR_SIZE - offset));
        monitor_camera_overlay_header(monitor_packet, MT9V03X_W, MT9V03X_H, camera_frame_id,
                                      camera_index, count, length, camera_crc);
        memcpy(monitor_packet + MONITOR_CAMERA_HEADER_SIZE, monitor_image + offset, length);
        if (!wifi_spi_send_async(monitor_packet, MONITOR_CAMERA_HEADER_SIZE + length, WIFI_CAMERA_PORT, now)) tx_kind = MONITOR_TX_CAMERA;
    }
    // 新包已复制到驱动，立即检查INT启动DMA；不再空等下一圈主循环。
    if (tx_kind != MONITOR_TX_NONE) wifi_spi_send_poll(now);

}
