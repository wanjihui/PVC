#ifndef MCAR_MONITOR_PROTOCOL_H
#define MCAR_MONITOR_PROTOCOL_H
#include <stdint.h>
#include <string.h>

#define MONITOR_CAMERA_HEADER_SIZE 24u
#define MONITOR_CAMERA_CHUNK_SIZE 1400u
#define MONITOR_BASE_DATA_SIZE 60u
#define MONITOR_SPEED_DATA_SIZE 108u
#define MONITOR_DATA_SIZE 148u // 保留108字节前缀，追加视觉指令和实际处理/通信耗时

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
#endif
