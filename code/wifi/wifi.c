/*********************************************************************************************************************
* 文件名称          wifi.c
* 适用平台          RT1064DVL6A
* 描述              高速 WiFi SPI 模块 —— 图像 + 边界/中线 + 示波器数据上传
*
* 上传内容分两类：
*   1) 图像：188×120 灰度图，叠加 3 条线（左边界 / 中线 / 右边界），
*      用 XY_BOUNDARY 协议，使上位机能在灰度图上画出算法提取的结果。
*   2) 示波器：8 通道 float，用于看速度环跟踪、偏差、陀螺仪等实时曲线。
*
* 数据来源全部是"只读快照"：
*      图像       → 发送前先 memcpy 到本地缓冲（E6_09 示例的做法），
*                   避免传输途中摄像头 DMA 写入造成图像撕裂。
*      边界/中线  → vision.c 的 vis_export_trace() 每帧填好的坐标数组。
*      示波器     → control.h / encoder.h 的只读诊断量。
*
* 为什么发送要降频：一帧 45120 字节经无线传输耗时百毫秒级，若每帧都发，
* 会把主循环（视觉 50Hz）拖到几 Hz。图像默认每 25 帧一次（约 2Hz），
* 示波器每 5 帧一次（约 10Hz）——曲线够实时、又不至于把视觉挤死。
* 注意这些发送都在主循环里，**200Hz 速度环中断不受任何影响**。
*********************************************************************************************************************/

#include "wifi.h"
#include "config.h"
#include "vision.h"
#include "control.h"
#include "encoder.h"
#include "pid.h"
#include "imu.h"

#if WIFI_SPI_ENABLE

/* 图像发送快照缓冲（22KB）
*
* 为什么必须有：camera_send() 发送的是**配置时存下的那个地址**里的内容，
* 而一帧 45120 字节经无线传输耗时百毫秒级，期间摄像头 DMA 还在以 50Hz 重写
* mt9v03x_image —— 直接发原图会得到几十帧竖向拼起来的"花图"
* （summer-smartcar 的 Round6 记录过这个坑）。
* 因此发送前先 memcpy 一份快照，传输期间新帧不会再污染它。 */
static uint8 wifi_image_copy[MT9V03X_H][MT9V03X_W];

/* 降频计数 */
static uint16 img_div_counter  = 0;
static uint16 data_div_counter = 0;

/* TCP 连接状态（wifi_spi 驱动内部维护，这里缓存一份便于查询） */
static uint8 wifi_connected = 0;

/* 示波器通道数（≤ SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT，默认 8） */
#define WIFI_SCOPE_CHANNELS     (8)

/* 数据降频：图像用 config.h 的 WIFI_IMG_SEND_DIV，示波器单独一个更快的节拍 */
#define WIFI_DATA_SEND_DIV      (5)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WiFi SPI 初始化并建立 TCP 连接
// 返回参数     0=成功 1=失败
// 功能描述     连接 config.h 里配置的 WiFi → 对目标 IP:PORT 建立 TCP 连接 → 初始化逐飞助手接口
// 备注信息     与 E6_09_wifi_spi_mt9v03x_demo 的流程一致；失败不阻塞主流程
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_init(void)
{
    uint16 retry = 0;

    /* ---- 1. 等待模块就绪并连接 WiFi ---- */
    while (wifi_spi_init(WIFI_SSID, WIFI_PASSWORD))
    {
        retry++;
        if (retry > 10)
        {
            printf("\r\nWiFi SPI init failed (检查模块供电/接线/SSID)");
            return 1;
        }
        system_delay_ms(200);
    }

    printf("\r\nWiFi module version: %s", wifi_spi_version);
    printf("\r\nWiFi module ip     : %s", wifi_spi_ip_addr_port);

    /* ---- 2. 建立 TCP 连接（逐飞助手设为 TCP Server，默认端口 8080）---- */
    if (1 != WIFI_SPI_AUTO_CONNECT)
    {
        retry = 0;
        while (wifi_spi_socket_connect("TCP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT))
        {
            retry++;
            if (retry > 10)
            {
                printf("\r\nTCP connect %s:%s failed (在 zf_device_wifi_spi.h 里改)",
                   WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT);
                return 1;
            }
            system_delay_ms(200);
        }
    }

    /* ---- 3. 逐飞助手接口初始化（图像与数据都走 WiFi SPI）---- */
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    /* ---- 4. 配置摄像头信息（必须先调，边界配置里有断言检查）----
    * image_addr 会被 camera_send() 真正使用，所以这里必须给**快照缓冲**的地址，
    * 而不是 mt9v03x_image（否则传输期间被 DMA 重写会出花图）。
    * 宽高也必须在这里给定：上位机据此决定边界坐标的数据宽度。 */
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                 wifi_image_copy,
                                                 MT9V03X_W,
                                                 MT9V03X_H);

    wifi_connected = 1;
    printf("\r\nWiFi ready: img every %d frames, scope every %d frames, %d scope channels",
           WIFI_IMG_SEND_DIV, WIFI_DATA_SEND_DIV, WIFI_SCOPE_CHANNELS);
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     上传示波器数据（8 通道）
// 功能描述     通道分配：
//               1-4  四轮实际速度 FL/FR/RL/RR   (cm/s)
//               5    err 视觉中线偏差            (px)
//               6    vx  前进速度目标           (cm/s)
//               7    ω   自转角速度指令         (deg/s)
//               8    gyro_z 陀螺仪 Z            (deg/s)
// 备注信息     想改通道含义就改下面的赋值，同时更新本注释与 README
//-------------------------------------------------------------------------------------------------------------------
void wifi_send_scope_now(void)
{
    if (!wifi_connected) return;

    seekfree_assistant_oscilloscope_data.channel_num = WIFI_SCOPE_CHANNELS;

    seekfree_assistant_oscilloscope_data.data[0] = Motor_PID[WHEEL_FL].Actual;
    seekfree_assistant_oscilloscope_data.data[1] = Motor_PID[WHEEL_FR].Actual;
    seekfree_assistant_oscilloscope_data.data[2] = Motor_PID[WHEEL_RL].Actual;
    seekfree_assistant_oscilloscope_data.data[3] = Motor_PID[WHEEL_RR].Actual;
    seekfree_assistant_oscilloscope_data.data[4] = g_vis.err;
    seekfree_assistant_oscilloscope_data.data[5] = speed_target_dbg;
    seekfree_assistant_oscilloscope_data.data[6] = omega_dbg;
    seekfree_assistant_oscilloscope_data.data[7] = gyro_z_dbg;

    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     上传一次图像（灰度图 + 左边界/中线/右边界）
// 功能描述     1) 先把当前帧灰度图快照到本地缓冲（防"花图"，必须做）
//              2) 用 vis_export_trace() 已填好的坐标数组配置 3 条边界
//              3) 交给逐飞助手发送（发送的是快照地址，见文件头说明）
//-------------------------------------------------------------------------------------------------------------------
void wifi_send_image_now(void)
{
    if (!wifi_connected) return;

    /* ---- 1. 快照一帧灰度图（必须在发送前完成）---- */
    memcpy(wifi_image_copy, mt9v03x_image, MT9V03X_IMAGE_SIZE);

    /* ---- 2. 配置 3 条边界（XY 坐标都发，能画出回弯）----
    * dot_num = 每帧点数 = 图像行数 VIS_TRACE_ROWS；
    * 无效点在 vision.c 里已置 VIS_TRACE_NONE，上位机会跳过该点。 */
#if WIFI_SEND_BOUNDARY
    {
        /* 中间那组 x/y 留给"中线"，左右两组是左右边界 */
        seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                     wifi_image_copy,
                                                     MT9V03X_W,
                                                     MT9V03X_H);
        seekfree_assistant_camera_boundary_config(XY_BOUNDARY,
                                                  VIS_TRACE_ROWS,
                                                  vis_trace_x_left,
                                                  vis_trace_x_center,
                                                  vis_trace_x_right,
                                                  vis_trace_y_left,
                                                  vis_trace_y_center,
                                                  vis_trace_y_right);
    }
#endif

    /* ---- 3. 发送 ---- */
    seekfree_assistant_camera_send();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主循环轮询：按降频节拍上传
// 功能描述     每帧调用一次即可；内部用两个独立计数器分别控制图像与示波器节拍
//-------------------------------------------------------------------------------------------------------------------
void wifi_poll(void)
{
    if (!wifi_connected) return;

    /* ---- 示波器：每 WIFI_DATA_SEND_DIV 帧 ---- */
    data_div_counter++;
    if (data_div_counter >= WIFI_DATA_SEND_DIV)
    {
        data_div_counter = 0;
        wifi_send_scope_now();
    }

    /* ---- 图像：每 WIFI_IMG_SEND_DIV 帧（0 = 关闭图像上传）---- */
#if (WIFI_IMG_SEND_DIV > 0)
    img_div_counter++;
    if (img_div_counter >= WIFI_IMG_SEND_DIV)
    {
        img_div_counter = 0;
        wifi_send_image_now();
    }
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询 WiFi TCP 连接状态
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_is_connected(void)
{
    return wifi_connected;
}
#endif /* WIFI_SPI_ENABLE：暂时停用整个业务模块，保留源码便于恢复。 */
