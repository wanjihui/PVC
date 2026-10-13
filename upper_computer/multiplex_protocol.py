"""MUDP v1: one UDP socket, explicit data/log/camera type, unchanged inner payload."""
import struct

HEADER = struct.Struct('<4sBBH')
MAGIC = b'MUDP'
CHANNELS = {1:'data', 2:'log', 3:'camera'}
TYPES = {name:kind for kind,name in CHANNELS.items()}
MAX_PAYLOAD = 1424


def encode_packet(channel, payload):
    if channel not in TYPES:
        raise ValueError('未知数据类型')
    if not 0 < len(payload) <= MAX_PAYLOAD:
        raise ValueError('MUDP负载长度错误')
    return HEADER.pack(MAGIC, 1, TYPES[channel], len(payload)) + payload


def decode_packet(packet):
    if len(packet) < HEADER.size:
        raise ValueError('MUDP包头不完整')
    magic,version,kind,length = HEADER.unpack_from(packet)
    if magic != MAGIC or version != 1 or kind not in CHANNELS:
        raise ValueError('MUDP魔数、版本或类型错误')
    if not 0 < length <= MAX_PAYLOAD or len(packet) != HEADER.size + length:
        raise ValueError('MUDP负载长度错误')
    payload = packet[HEADER.size:]
    channel = CHANNELS[kind]
    if channel == 'camera' and not payload.startswith(b'MCAM'):
        raise ValueError('图像类型缺少MCAM包头')
    return channel,payload
