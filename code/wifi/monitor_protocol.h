#ifndef MCAR_MONITOR_PROTOCOL_H
#define MCAR_MONITOR_PROTOCOL_H
#include <stdint.h>
#include <string.h>

#define MONITOR_CAMERA_HEADER_SIZE 24u
#define MONITOR_CAMERA_CHUNK_SIZE 1400u
#define MONITOR_BASE_DATA_SIZE 60u
#define MONITOR_SPEED_DATA_SIZE 108u
#define MONITOR_DATA_SIZE 152u // 保留148字节前缀，偏移148追加uint32实际弯级0/1/2
#define MONITOR_MUX_HEADER_SIZE 8u // MUDP魔数4字节、版本1、类型1、负载长度2（小端）
#define MONITOR_MUX_DATA 1u // 内层保持原148字节参数及字段偏移
#define MONITOR_MUX_LOG 2u // 内层为原日志字节，允许一行跨多个包
#define MONITOR_MUX_CAMERA 3u // 内层为原MCAM分片，保留整帧CRC与元数据

static inline void monitor_speed_fields(uint8_t *packet, const float target[4],
                                        const float actual[4], const float duty[4])
{
    memcpy(packet + 60, target, 16);
    memcpy(packet + 76, actual, 16);
    memcpy(packet + 92, duty, 16);
}

static inline void monitor_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static inline void monitor_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
/* 封装单端口类型头；调用者保证p有8字节空间，length为随后负载字节数。 */
static inline void monitor_mux_header(uint8_t *p, uint8_t type, uint16_t length)
{
    memcpy(p,"MUDP",4);p[4]=1;p[5]=type;monitor_put_u16(p+6,length);
}
static inline uint32_t monitor_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xffffffffu, i;
    uint8_t bit;
    for (i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return crc ^ 0xffffffffu;
}
static inline void monitor_camera_header(uint8_t *p, uint16_t w, uint16_t h, uint32_t frame,
                                         uint16_t index, uint16_t count, uint16_t length, uint32_t crc)
{
    memcpy(p, "MCAM", 4); p[4] = 1; p[5] = 1;
    monitor_put_u16(p + 6, w); monitor_put_u16(p + 8, h);
    monitor_put_u32(p + 10, frame); monitor_put_u16(p + 14, index);
    monitor_put_u16(p + 16, count); monitor_put_u16(p + 18, length);
    monitor_put_u32(p + 20, crc);
}
static inline void monitor_camera_overlay_header(uint8_t *p, uint16_t w, uint16_t h, uint32_t frame,
                                         uint16_t index, uint16_t count, uint16_t length, uint32_t crc)
{
    monitor_camera_header(p, w, h, frame, index, count, length, crc);
    p[5] = 2; // 灰度图+逐行边界坐标；CRC覆盖两部分
}
static inline void monitor_vision_fields(uint8_t *p, const float commands[4],
                                         const uint32_t timing[6])
{
    uint8_t i;
    memcpy(p + 108, commands, 16); // err(px), omega(deg/s), vx(cm/s), vy(cm/s)
    for (i = 0; i < 6; i++) monitor_put_u32(p + 124 + 4u * i, timing[i]);
}
static inline void monitor_camera_debug_header(uint8_t *p, uint16_t w, uint16_t h, uint32_t frame,
                                        uint16_t index, uint16_t count, uint16_t length, uint32_t crc)
{
    monitor_camera_header(p, w, h, frame, index, count, length, crc);
    p[5] = 3; // 灰度+逐行坐标+12字节实际取值范围/拐点；CRC覆盖全部内容
}
#endif
