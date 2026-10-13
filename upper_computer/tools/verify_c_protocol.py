"""Cross-check the real firmware header/CRC writer against the Python receiver."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from camera_protocol import HEADER, encode_frame, CameraAssembler
from transport import Packet

raw = Path(sys.argv[1]).read_bytes()
image = bytes(i%256 for i in range(188*120))
expected = encode_frame(image,188,120,0x12345678)
assert raw == b''.join(expected), 'C / Python packet bytes differ'
assembler = CameraAssembler()
cursor = 0
packets = []
while cursor < len(raw):
    size = HEADER.unpack_from(raw,cursor)[8]
    packets.append(raw[cursor:cursor+HEADER.size+size])
    cursor += HEADER.size+size
for data in reversed(packets):
    frame = assembler.feed(Packet(123,('127.0.0.1',6666),data))
assert frame.pixels == image and frame.frame_id == 0x12345678
print('PASS: C firmware -> Python, 17 byte-identical packets, out-of-order full image')
