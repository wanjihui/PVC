#define _CRT_SECURE_NO_WARNINGS
#include <assert.h>
#include <stdio.h>
#include "../code/wifi/monitor_protocol.h"

int main(int argc, char **argv)
{
    uint8_t header[24], image[188 * 120], packet[24 + 1400], speed[MONITOR_DATA_SIZE] = {0};
    const float target[4] = {150,150,150,150}, actual[4] = {145,153,140,152}, duty[4] = {18,17,20,19};
    const uint8_t expected[] = {0x4d,0x43,0x41,0x4d,1,1,0xbc,0,0x78,0,0x78,0x56,0x34,0x12,
                                0,0,0x11,0,0x78,5,0x26,0x39,0xf4,0xcb};
    uint32_t i, offset, crc;
    uint16_t index, size;
    FILE *stream;
    assert(monitor_crc32((const uint8_t *)"123456789",9) == 0xcbf43926u);
    assert(monitor_crc32((const uint8_t *)"",0) == 0);
    monitor_put_u32(speed + 52,300);
    monitor_put_u32(speed + 56,400);
    monitor_speed_fields(speed,target,actual,duty);
    assert(sizeof(speed) == 148);
    {
        const float commands[4] = {12, -15, 150, 0};
        const uint32_t timing[6] = {2345, 20000, 500, 3500, 9000, 0};
        monitor_vision_fields(speed, commands, timing);
        assert(memcmp(speed + 108, commands, 16) == 0);
        assert(speed[128] == 0x20 && speed[129] == 0x4e);
    }
    assert(memcmp(speed + 60,target,16) == 0 && memcmp(speed + 76,actual,16) == 0 && memcmp(speed + 92,duty,16) == 0);
    if (argc == 3)
    {
        stream = fopen(argv[2],"wb");
        assert(stream != NULL && fwrite(speed,1,sizeof(speed),stream) == sizeof(speed));
        fclose(stream);
    }
    monitor_camera_header(header,188,120,0x12345678u,0,17,1400,0xcbf43926u);
    assert(memcmp(header,expected,sizeof(header)) == 0);
    for (i = 0; i < sizeof(image); i++) image[i] = (uint8_t)i;
    crc = monitor_crc32(image,sizeof(image));
    if (argc >= 2)
    {
        stream = fopen(argv[1],"wb");
        assert(stream != NULL);
        for (index = 0; index < 17; index++)
        {
            offset = index * 1400u;
            size = (uint16_t)((sizeof(image) - offset) > 1400u ? 1400u : sizeof(image) - offset);
            monitor_camera_header(packet,188,120,0x12345678u,index,17,size,crc);
            memcpy(packet+24,image+offset,size);
            assert(fwrite(packet,1,24u+size,stream) == 24u+size);
        }
        fclose(stream);
    }
    puts("PASS: firmware CRC32, little-endian header, 17 camera chunks");
    return 0;
}
