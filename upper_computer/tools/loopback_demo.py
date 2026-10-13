"""Explicit synthetic test source, only sent to local test ports 18081..18083."""
import math
from pathlib import Path
import socket
import struct
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from camera_protocol import encode_frame
from multiplex_protocol import encode_packet

with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
    for sequence in range(80):
        t = sequence/20
        angles = (20*math.sin(t),12*math.cos(t),t*15)
        data = struct.pack('<9f4i2I',*angles,0,0,15,0,0,1,-sequence,sequence,sequence*2,0,sequence*10,sequence)
        data += struct.pack('<12f',150,150,150,150,148,152,149,151,13,13,12,14)
        data += struct.pack('<4f6I',12,-15,150,0,2345,20000,sequence,3500,9000,0)
        sock.sendto(encode_packet('data',data),('127.0.0.1',18081))
        if sequence%20 == 0:
            message = f'[自测，非实车] 通信验证 seq={sequence}；中文日志正常\r\n'.encode('utf-8')
            sock.sendto(encode_packet('log',message[:2]),('127.0.0.1',18081))
            sock.sendto(encode_packet('log',message[2:]),('127.0.0.1',18081))
        if sequence%10 == 0:
            w,h = (160 if sequence == 70 else 188),120
            pixels = bytes(245 if abs(x-(w//2+int(20*math.sin(t))+(y-h//2)//5))>8 else 25
                           for y in range(h) for x in range(w))
            overlay = b'' if sequence < 40 else b''.join(
                bytes((w//2-35, w//2+35, w//2, 3 if 40<=y<80 else 0)) for y in range(h))
            far,near = (50,70) if sequence == 70 else (45,65)
            metadata = b'' if sequence < 60 else bytes((far,near,15,2,45,35,w-45,35,45,85,w-45,85))
            packets = encode_frame(pixels,w,h,sequence//10,overlay,metadata)
            for packet in reversed(packets):
                sock.sendto(encode_packet('camera',packet),('127.0.0.1',18081))
                time.sleep(.001)
        time.sleep(.05)
    print('Sent 80 synthetic track packets, 4 logs, 4 gray + 2 legacy overlay + 2 sampling/corner frames to LOCAL test ports only.')
