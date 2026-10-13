import socket
import struct
import time
import pytest
from packet_parser import FieldRule, DEFAULT_RULES, MONITOR_RULES, SPEED_RULES, TRACK_RULES, parse_packet
from transport import Packet, UdpReceiver
from debug_log import LineDecoder
from camera_protocol import CameraAssembler, encode_frame

def test_actual_firmware_track_extension():
    from pathlib import Path
    p = Path(__file__).resolve().parent / 'fixtures/speed_packet.bin'
    if not p.exists():
        pytest.skip('Run firmware native checks to generate fixture')
    values, errors = parse_packet(p.read_bytes(), TRACK_RULES)
    assert not errors
    assert values['vx_cmd_cms'] == 150 and values['omega_cmd_dps'] == -15
    assert values['vis_err_px'] == 12 and values['vis_time_us'] == 2345
    assert values['vis_interval_us'] == 20000 and values['wifi_peak_us'] == 9000

def test_overlay_preserves_raw_gray_and_marks_repairs_and_bmp():
    from camera_protocol import compose_rgb, frame_bmp, LEFT_COLOR, REPAIR_COLOR
    pixels = bytes([91]) * (188*120)
    overlay = bytes([10, 170, 94, 0])*60 + bytes([12, 168, 94, 3])*60
    packets = encode_frame(pixels, 188, 120, 10, overlay)
    assert len(packets) == 17 and len(packets[-1]) == 24+640
    assembler = CameraAssembler()
    for data in reversed(packets):
        frame = assembler.feed(packet(data), now=1)
    assert frame.pixels == pixels and frame.overlay == overlay
    rgb = compose_rgb(frame)
    offset = 3*(20*188+10)
    assert rgb[offset:offset+3] == bytes(LEFT_COLOR)
    assert rgb[:20*188*3] == bytes([91])*(20*188*3)
    assert rgb[100*188*3:] == bytes([91])*(20*188*3)
    assert rgb[3*(61*188+12):3*(61*188+12)+3] == bytes(REPAIR_COLOR)
    assert rgb[0:3] == bytes([91]*3)
    bmp = frame_bmp(frame)
    assert bmp[:2] == b'BM' and struct.unpack_from('<H', bmp, 28)[0] == 24
    assert len(bmp) == 54+564*120
    # Damage only metadata: CRC must reject the entire frame.
    damaged = packets.copy()
    damaged[-1] = damaged[-1][:-1] + bytes([damaged[-1][-1]^1])
    bad = CameraAssembler()
    for data in damaged:
        assert bad.feed(packet(data), now=1) is None
    assert bad.invalid == 1

def test_invalid_overlay_and_gaps_are_not_connected():
    from camera_protocol import Frame, compose_rgb, LEFT_COLOR
    with pytest.raises(ValueError):
        encode_frame(bytes(200), 100, 2, 0, bytes([100,255,255,0])*2)
    with pytest.raises(ValueError):
        encode_frame(bytes(200), 100, 2, 0, bytes([10,90,50,4])*2)
    overlay = bytearray([255,255,255,0]*120)
    overlay[4*20]=1;overlay[4*22]=8
    frame = Frame(10,120,0,bytes(1200),0,('test',1),bytes(overlay))
    rgb = compose_rgb(frame)
    assert rgb[21*30:22*30] == bytes(30)
    assert rgb[20*30+3:20*30+6] == bytes(LEFT_COLOR)

def test_sampling_guides_same_boundary_color_actual_corners_and_edge_crop():
    from camera_protocol import compose_rgb, LEFT_COLOR, RIGHT_COLOR, SAMPLE_COLOR, CORNER_COLOR, HEADER
    pixels = bytes([91])*(188*120)
    overlay = bytes([30,155,94,0])*120
    metadata = bytes((45,65,5,2,30,35,255,255,30,85,255,255))
    packets = encode_frame(pixels,188,120,10,overlay,metadata)
    assert HEADER.unpack_from(packets[0])[2] == 3
    assert len(packets) == 17 and len(packets[-1]) == 24+652
    assembler = CameraAssembler()
    for data in reversed(packets):
        frame = assembler.feed(packet(data),now=1)
    assert frame.pixels == pixels and frame.sample_rows == (45,65)
    assert frame.corners == ((0,30,35),(2,30,85))
    rgb = compose_rgb(frame)
    assert LEFT_COLOR == RIGHT_COLOR
    for y in (45,65):
        assert rgb[3*y*188:3*(y+1)*188] == bytes(SAMPLE_COLOR)*188
    for x,y in ((30,25),(155,25)):
        assert rgb[3*(y*188+x):3*(y*188+x)+3] == bytes(LEFT_COLOR)
    assert rgb[3*(35*188+30):3*(35*188+30)+3] == bytes(CORNER_COLOR)
    assert rgb[:20*188*3] == bytes([91])*(20*188*3)
    assert rgb[100*188*3:] == bytes([91])*(20*188*3)
    # Live preview changes must move guides without changing grayscale or line crop.
    moved = bytes((40,60))+metadata[2:]
    for data in encode_frame(pixels,188,120,11,overlay,moved):
        updated = assembler.feed(packet(data),now=1.1)
    assert updated.sample_rows == (40,60) and updated.pixels == pixels
    assert compose_rgb(updated)[3*40*188:3*41*188] == bytes(SAMPLE_COLOR)*188
    from camera_protocol import Frame, CANDIDATE_COLOR
    candidate = Frame(188,120,12,pixels,1,('test',1),overlay,metadata[:3]+bytes([1])+metadata[4:])
    rgb_candidate = compose_rgb(candidate)
    assert rgb_candidate[3*(35*188+30):3*(35*188+30)+3] == bytes(CANDIDATE_COLOR)

def test_malformed_camera_metadata_is_rejected_even_with_valid_crc():
    from camera_protocol import HEADER
    pixels = bytes(188*120)
    overlay = bytes((30,155,94,0))*120
    valid = bytes((45,65,0,0))+bytes([255])*8
    for metadata in (bytes((65,45))+valid[2:],valid[:2]+bytes((1,2,188,35))+valid[6:]):
        with pytest.raises(ValueError):
            encode_frame(pixels,188,120,0,overlay,metadata)
        payload = pixels+overlay+metadata
        import zlib
        count = (len(payload)+1399)//1400
        assembler = CameraAssembler()
        for index in range(count):
            chunk = payload[1400*index:1400*(index+1)]
            data = HEADER.pack(b'MCAM',1,3,188,120,0,index,count,len(chunk),zlib.crc32(payload))+chunk
            assert assembler.feed(packet(data),now=1) is None
        assert assembler.invalid == 1 and assembler.completed == 0

def test_actual_firmware_vision_datagrams_reassemble_same_frame():
    from pathlib import Path
    from camera_protocol import HEADER, compose_rgb, REPAIR_COLOR
    path = Path(__file__).resolve().parent / 'fixtures/vision_camera_packets.bin'
    if not path.exists():
        pytest.skip('Run review_vision_wifi.py for the actual C-generated fixture')
    data = path.read_bytes()
    assembler, cursor, frames = CameraAssembler(), 0, []
    while cursor < len(data):
        size = HEADER.unpack_from(data,cursor)[-2]
        frame = assembler.feed(packet(data[cursor:cursor+HEADER.size+size]),now=1)
        cursor += HEADER.size+size
        if frame:
            frames.append(frame)
    assert len(frames) == 1 and assembler.invalid == 0 and assembler.lost == 0
    frame = frames[0]
    assert frame.width == 188 and len(frame.overlay) == 480
    assert frame.sample_rows == (35,55) and len(frame.metadata) == 12
    assert frame.pixels == bytes(200 if 60<x<128 else 20 for y in range(120) for x in range(188))
    assert frame.overlay[4*25+3] & 1
    left = frame.overlay[4*25]
    rgb = compose_rgb(frame)
    offset = 3*(25*188+left)
    assert rgb[offset:offset+3] == bytes(REPAIR_COLOR)

def packet(data, stamp=123, peer=('127.0.0.1',6666)):
    return Packet(stamp,peer,data)

def test_original_12_byte_compatibility_and_rule_isolation():
    payload = struct.pack('<fff',12.5,-2.25,179)
    values, errors = parse_packet(payload,DEFAULT_RULES+[FieldRule('missing','float64',12)])
    assert values == dict(roll=12.5,pitch=-2.25,yaw=179)
    assert set(errors) == {'missing'}

def test_monitor_packet_signed_encoders_and_sequence():
    data = struct.pack('<9f4i2I',*range(9),-123,22,-99,0,300,400)
    values,errors = parse_packet(data,MONITOR_RULES)
    assert not errors
    assert values['enc_FL'] == -123 and values['enc_RL'] == -99
    assert values['samples'] == 300 and values['seq'] == 400

def test_nan_does_not_reach_gui():
    values,errors = parse_packet(struct.pack('<fff',float('nan'),1,2),DEFAULT_RULES)
    assert 'roll' in errors and 'roll' not in values and values['pitch'] == 1

def test_speed_loop_packet_keeps_base_fields_and_wheel_order():
    base = struct.pack('<9f4i2I',*range(9),11,22,33,44,300,400)
    data = base + struct.pack('<12f',150,150,150,150,145,153,140,152,18,17,20,19)
    values, errors = parse_packet(data,SPEED_RULES)
    assert not errors and values['enc_FL'] == 11 and values['seq'] == 400
    assert [values['target_'+w] for w in ('FL','FR','RL','RR')] == [150]*4
    assert [values['speed_'+w] for w in ('FL','FR','RL','RR')] == [145,153,140,152]
    assert [values['duty_'+w] for w in ('FL','FR','RL','RR')] == [18,17,20,19]
    assert parse_packet(data,MONITOR_RULES) == parse_packet(base,MONITOR_RULES)

def test_actual_firmware_speed_packet_parses_in_monitor():
    from pathlib import Path
    p = Path(__file__).resolve().parent / 'fixtures/speed_packet.bin'
    if not p.exists():
        pytest.skip('Run the firmware native framework checks for the C/Python integration fixture')
    values, errors = parse_packet(p.read_bytes(),SPEED_RULES)
    assert not errors
    assert values['samples'] == 300 and values['seq'] == 400
    assert [values['speed_'+w] for w in ('FL','FR','RL','RR')] == [145,153,140,152]
    assert [values['duty_'+w] for w in ('FL','FR','RL','RR')] == [18,17,20,19]

def test_split_utf8_crlf_and_independent_senders():
    decoder = LineDecoder()
    text = '测试成功\r\nsecond\n'.encode()
    assert decoder.feed(packet(text[:2])) == []
    lines = decoder.feed(packet(text[2:],124))
    assert [x.text for x in lines] == ['测试成功','second']
    assert lines[0].stamp == 123
    decoder.feed(packet(b'first'))
    other = decoder.feed(packet(b'other\n',peer=('127.0.0.2',6666)))
    assert other[0].text == 'other'
    assert decoder.flush()[0].text == 'first'

def test_out_of_order_camera_duplicate_crc_and_missing():
    image = bytes(i%256 for i in range(188*120))
    packets = encode_frame(image,188,120,0xffffffff)
    assembler = CameraAssembler()
    assert assembler.feed(packet(packets[-1]),now=1) is None
    assert assembler.feed(packet(packets[-1]),now=1) is None
    frame = None
    for data in reversed(packets[:-1]):
        frame = assembler.feed(packet(data),now=1)
    assert frame.pixels == image and assembler.duplicates == 1
    # uint32 sequence wrap is a new frame, while stale old frames cannot replace it.
    for data in encode_frame(image,188,120,0):
        new = assembler.feed(packet(data),now=1.1)
    assert new.frame_id == 0
    assert assembler.feed(packet(packets[0]),now=1.2) is None and assembler.stale == 1
    damaged = encode_frame(image,188,120,1)
    damaged[0] = damaged[0][:-1] + bytes([damaged[0][-1]^1])
    for data in damaged:
        assert assembler.feed(packet(data),now=1.3) is None
    assert assembler.invalid == 1
    assembler.feed(packet(encode_frame(image,188,120,2)[0]),now=2)
    assembler.expire(now=3)
    assert assembler.lost == 1

def test_bad_camera_headers_and_bounded_pending():
    assembler = CameraAssembler(max_pending=2)
    assert assembler.feed(packet(b'bad')) is None
    assert assembler.invalid == 1
    pixels = bytes(2000)
    for fid in range(5):
        assembler.feed(packet(encode_frame(pixels,100,20,fid)[0]),now=1)
    assert len(assembler.pending) == 2 and assembler.lost == 3

def test_real_udp_loopback_and_exclusive_port():
    probe = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    probe.bind(('127.0.0.1',0))
    port = probe.getsockname()[1]
    probe.close()
    receiver = UdpReceiver(capacity=2)
    receiver.start('127.0.0.1',port)
    competitor = UdpReceiver()
    try:
        with pytest.raises(OSError):
            competitor.start('127.0.0.1',port)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            for data in (b'one',b'two',b'three'):
                sender.sendto(data,('127.0.0.1',port))
        deadline = time.monotonic()+2
        while receiver.received < 3 and time.monotonic()<deadline:
            time.sleep(.01)
        assert receiver.received == 3 and receiver.dropped == 1
        assert [p.data for p in receiver.drain()] == [b'one',b'two']
    finally:
        receiver.stop()
    assert not receiver.running
