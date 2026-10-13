import socket
import struct
import time
from pathlib import Path
import pytest
from camera_protocol import CameraAssembler, encode_frame
from debug_log import LineDecoder
from multiplex_protocol import encode_packet, decode_packet
from packet_parser import TRACK_RULES, parse_packet
from transport import MultiplexReceiver, Packet


@pytest.mark.parametrize('channel,payload', [('data',b'\0'*148),('log','中文\r\n'.encode()),('camera',encode_frame(b'123456',3,2,7)[0])])
def test_roundtrip(channel,payload):
    assert decode_packet(encode_packet(channel,payload)) == (channel,payload)


@pytest.mark.parametrize('packet',[b'',b'MUDP',b'MUDP\2\1\1\0X',b'MUDP\1\4\1\0X',
                                  b'MUDP\1\1\0\0',b'MUDP\1\1\2\0X',
                                  b'MUDP\1\3\4\0FAIL',b'MUDP\1\2\1\0XY'])
def test_malformed_never_reaches_a_channel(packet):
    router = MultiplexReceiver()
    router.receiver.queue.put(Packet(1,('127.0.0.1',6666),packet))
    router.route()
    assert router.invalid==1
    assert all(c.received==0 and c.queue.empty() for c in router.channels.values())


def test_one_real_socket_routes_all_channels_and_releases_port():
    probe=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    probe.bind(('127.0.0.1',0));port=probe.getsockname()[1];probe.close()
    router=MultiplexReceiver();router.start('127.0.0.1',port)
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            for channel,payload in [('data',b'\0'*148),('log',b'hello\n'),('camera',encode_frame(b'123456',3,2,0)[0])]:
                sender.sendto(encode_packet(channel,payload),('127.0.0.1',port))
        deadline=time.monotonic()+2
        while router.receiver.received<3 and time.monotonic()<deadline:time.sleep(.01)
        router.route()
        assert router.receiver.received==3
        assert all(c.running and c.received==1 for c in router.channels.values())
        assert all(not hasattr(c,'_socket') for c in router.channels.values())
    finally:router.stop()
    assert all(not c.running for c in router.channels.values())
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:probe.bind(('127.0.0.1',port))


def test_actual_c_wire_data_logs_camera_and_guides():
    path=Path(__file__).resolve().parent/'fixtures/single_udp_wire.bin'
    assert path.exists(),'Missing bundled C-generated protocol fixture'
    raw=path.read_bytes();offset=0;counts=dict(data=0,log=0,camera=0)
    camera=CameraAssembler();decoder=LineDecoder();lines=[];frames=[]
    while offset<len(raw):
        length=struct.unpack_from('<H',raw,offset)[0];offset+=2
        packet=raw[offset:offset+length];offset+=length
        channel,payload=decode_packet(packet);counts[channel]+=1
        assert len(packet)<=1432
        if channel=='data':
            values,errors=parse_packet(payload,TRACK_RULES)
            assert not errors and 'seq' in values
        elif channel=='log':lines+=decoder.feed(Packet(time.time(),('127.0.0.1',6666),payload))
        else:
            frame=camera.feed(Packet(time.time(),('127.0.0.1',6666),payload))
            if frame:frames.append(frame)
    lines+=decoder.flush()
    assert all(n>0 for n in counts.values()) and camera.completed>=2
    assert any('[INIT]' in line.text for line in lines)
    assert frames[0].sample_rows==(35,55) and len(frames[0].metadata)==12


def test_shared_socket_app_callbacks_keep_logging_and_motion_peer(monkeypatch,tmp_path):
    import app
    from argparse import Namespace
    from test_app_model import GuiStub
    gui=GuiStub();monkeypatch.setattr(app,'STORE',tmp_path)
    monkeypatch.setattr(app,'dpg',gui);monkeypatch.setattr(app.Monitor,'_build',lambda self:None)
    m=app.Monitor(Namespace(test_ports=True));m.rules=TRACK_RULES
    m.history={r.name:app.deque(maxlen=1200) for r in TRACK_RULES}
    try:
        m.multiplex.receiver.queue.put(Packet(time.time(),('127.0.0.1',6666),encode_packet('data',b'\0'*148)))
        m.multiplex.receiver.queue.put(Packet(time.time(),('127.0.0.1',6666),encode_packet('log',b'[INIT] ready\r\n')))
        for p in encode_frame(b'123456',3,2,9):
            m.multiplex.receiver.queue.put(Packet(time.time(),('127.0.0.1',6666),encode_packet('camera',p)))
        m.stop() # must route/save queued packets even after listener stops
        assert m.data_records==1 and m.frame.frame_id==9
        assert '[INIT] ready' in m.log_path.read_text('utf-8')
        pulse=m.link.heartbeat(time.monotonic(),True)
        assert pulse and pulse[1]==('127.0.0.1',6666) and pulse[0].startswith(b'MCAR ALIVE')
    finally:m.log_file.close();m.data_file.close()
