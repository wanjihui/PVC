"""Exercise actual application callbacks with a GUI stub (no desktop input automation)."""
from argparse import Namespace
import csv
import queue
import struct
import time
import pytest
import app
from camera_protocol import Frame, encode_frame
from debug_log import LogLine
from packet_parser import MONITOR_RULES
from transport import Packet

class GuiStub:
    def __init__(self):
        self.values = dict(log_pause=False,log_filter='',log_timestamp=True,log_scroll=True,
                           log_encoding='utf-8',csv_path='',bind_ip='127.0.0.1',
                           data_port=18081,log_port=18082,camera_port=18083)
        self.texts = []
        self.colors = []
        self.scrolls = 0
    def get_value(self, name):
        return self.values[name]
    def set_value(self, name, value):
        self.values[name] = value
    def add_text(self, text, parent, color=None):
        self.texts.append(text)
        self.colors.append(color)
        return len(self.texts)
    def delete_item(self, item, children_only=False):
        if item == 'log_panel':
            self.texts.clear()
            self.colors.clear()
    def set_y_scroll(self, *args):
        self.scrolls += 1
    def get_y_scroll_max(self, name):
        return len(self.texts)

def model(monkeypatch,tmp_path):
    gui = GuiStub()
    monkeypatch.setattr(app,'STORE',tmp_path)
    monkeypatch.setattr(app,'dpg',gui)
    monkeypatch.setattr(app.Monitor,'_build',lambda self:None)
    monitor = app.Monitor(Namespace(test_ports=True,legacy_ports=True))
    monitor.rules = MONITOR_RULES
    monitor.history = {rule.name:app.deque(maxlen=1200) for rule in MONITOR_RULES}
    return monitor,gui

class ControlledReceiver:
    """Model socket success/failure so receiver orchestration is deterministic."""
    def __init__(self, error_on_start=None):
        self.queue = queue.Queue()
        self.running = False
        self.error_on_start = error_on_start
        self.starts = []
        self.stops = 0

    def start(self, ip, port):
        self.starts.append((ip,port))
        if self.error_on_start:
            raise self.error_on_start
        self.running = True

    def stop(self):
        self.running = False
        self.stops += 1

    def drain(self, limit):
        for _ in range(limit):
            try:
                yield self.queue.get_nowait()
            except queue.Empty:
                break

def controlled_receivers(monitor):
    monitor.receivers = {name:ControlledReceiver() for name in ('data','log','camera')}
    return monitor.receivers

def test_paused_display_keeps_receiving_and_saves_hidden_logs(monkeypatch,tmp_path):
    monitor,gui = model(monkeypatch,tmp_path)
    try:
        monitor.append_logs([LogLine(123,'test','first')])
        monitor.refresh_logs()
        assert len(gui.texts) == 1 and gui.scrolls == 1
        gui.values['log_pause'] = True
        monitor.receivers['log'].queue.put(Packet(124,('127.0.0.1',6666),'暂停期间仍接收\n'.encode()))
        monitor.receive()
        monitor.refresh_logs()
        assert len(gui.texts) == 1 and monitor.total_lines == 2
        monitor.save_log()
        saved = next(monitor.output.glob('导出日志_*.txt')).read_text('utf-8')
        assert '暂停期间仍接收' in saved and 'first' in saved
        gui.values['log_pause'] = False
        monitor.refresh_logs()
        assert any('暂停期间仍接收' in x for x in gui.texts)
        gui.values['log_scroll'] = False
        scrolls = gui.scrolls
        monitor.append_logs([LogLine(125,'test','no autoscroll')])
        monitor.refresh_logs()
        assert gui.scrolls == scrolls
        monitor.clear_logs()
        monitor.refresh_logs()
        assert not gui.texts and 'first' in monitor.log_path.read_text('utf-8')
    finally:
        monitor.log_file.close()
        monitor.data_file.close()

def test_clear_while_paused_removes_visible_rows_without_erasing_saved_logs(monkeypatch,tmp_path):
    monitor,gui = model(monkeypatch,tmp_path)
    try:
        monitor.append_logs([LogLine(time.time(),'test','visible before pause')])
        monitor.refresh_logs()
        assert gui.texts
        gui.values['log_pause'] = True
        monitor.append_logs([LogLine(time.time(),'test','received while paused')])
        monitor.clear_logs()
        monitor.refresh_logs()
        assert not gui.texts and not monitor._log_widgets
        assert gui.values['log_pause'] and not monitor._pending_log_scroll
        monitor.save_log()
        saved = next(monitor.output.glob('导出日志_*.txt')).read_text('utf-8')
        assert 'visible before pause' in saved and 'received while paused' in saved
        gui.values['log_pause'] = False
        monitor.refresh_logs()
        assert all('visible before pause' not in text and 'received while paused' not in text
                   for text in gui.texts)
        monitor.append_logs([LogLine(time.time(),'test','new after clear')])
        monitor.refresh_logs()
        assert any('new after clear' in text for text in gui.texts)
    finally:
        monitor.log_file.close()
        monitor.data_file.close()

def test_telemetry_csv_and_camera_save_uses_displayed_frame(monkeypatch,tmp_path):
    monitor,gui = model(monkeypatch,tmp_path)
    try:
        payload = struct.pack('<9f4i2I',*range(9),-20,30,-40,50,300,400)
        monitor.receivers['data'].queue.put(Packet(time.time(),('127.0.0.1',6666),payload))
        monitor.receive()
        monitor.save_csv()
        with next(monitor.output.glob('数据_*.csv')).open(encoding='utf-8-sig',newline='') as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 1 and rows[0]['enc_FL'] == '-20' and rows[0]['seq'] == '400'
        first = Frame(3,2,0,bytes([0,1,2,3,4,5]),123,('localhost',6666))
        monitor.shown_frame = first
        for packet in encode_frame(bytes([9]*6),3,2,1):
            monitor.receivers['camera'].queue.put(Packet(124,first.address,packet))
        monitor.receive()
        assert monitor.frame.frame_id == 1 and monitor.shown_frame.frame_id == 0
        monitor.save_frame()
        bmp = next(monitor.output.glob('总钻风_*.bmp')).read_bytes()
        assert bmp[:2] == b'BM' and struct.unpack_from('<ii',bmp,18) == (3,2)
        assert bmp[1078:] == bytes([3,4,5,0,0,1,2,0])  # bottom-up rows, padded to four bytes
    finally:
        monitor.log_file.close()
        monitor.data_file.close()

def test_combined_start_failure_preserves_existing_camera(monkeypatch,tmp_path):
    monitor,gui = model(monkeypatch,tmp_path)
    receivers = controlled_receivers(monitor)
    try:
        monitor.start_channel('camera')
        camera_decoder = monitor.camera
        receivers['log'].error_on_start = OSError('port occupied')
        with pytest.raises(OSError,match='port occupied'):
            monitor.start()
        assert receivers['camera'].running
        assert receivers['camera'].starts == [('127.0.0.1',18083)]
        assert receivers['camera'].stops == 0
        assert monitor.camera is camera_decoder
        assert not receivers['data'].running and receivers['data'].stops == 1
        assert monitor.bindings == {'camera':('127.0.0.1',18083)}
    finally:
        monitor.log_file.close()
        monitor.data_file.close()

def test_combined_start_rejects_changed_live_binding_without_stopping_it(monkeypatch,tmp_path):
    monitor,gui = model(monkeypatch,tmp_path)
    receivers = controlled_receivers(monitor)
    try:
        monitor.start_channel('camera')
        gui.values['camera_port'] = 19083
        with pytest.raises(ValueError,match='更改监听地址'):
            monitor.start()
        assert receivers['camera'].running and receivers['camera'].stops == 0
        assert not receivers['data'].running and not receivers['log'].running
        assert monitor.bindings == {'camera':('127.0.0.1',18083)}
    finally:
        monitor.log_file.close()
        monitor.data_file.close()

def test_independent_camera_stop_keeps_data_and_logs_receiving(monkeypatch,tmp_path):
    monitor,gui = model(monkeypatch,tmp_path)
    receivers = controlled_receivers(monitor)
    try:
        monitor.start()
        receivers['log'].queue.put(Packet(time.time(),('127.0.0.1',6666),'图像停止时日志仍接收\n'.encode()))
        monitor.stop_channel('camera')
        assert receivers['data'].running and receivers['log'].running
        assert not receivers['camera'].running
        assert set(monitor.bindings) == {'data','log'}
        monitor.receive()
        assert any(line.text == '图像停止时日志仍接收' for line in monitor.logs)
        monitor.start_channel('camera')
        assert all(receiver.running for receiver in receivers.values())
        assert receivers['data'].starts == [('127.0.0.1',18081)]
        assert receivers['log'].starts == [('127.0.0.1',18082)]
    finally:
        monitor.log_file.close()
        monitor.data_file.close()

def test_stop_drains_backlog_before_saving(monkeypatch,tmp_path):
    monitor,gui = model(monkeypatch,tmp_path)
    receivers = controlled_receivers(monitor)
    try:
        monitor.start()
        stamp = time.time()
        payload = struct.pack('<9f4i2I',*range(9),1,2,3,4,500,600)
        for index in range(650):
            receivers['data'].queue.put(Packet(stamp,('127.0.0.1',6666),payload))
            receivers['log'].queue.put(Packet(stamp,('127.0.0.1',6666),f'backlog {index}\n'.encode()))
        monitor.stop()
        assert monitor.data_records == 650
        assert len([line for line in monitor.logs if line.text.startswith('backlog ')]) == 650
        assert all(receiver.queue.empty() for receiver in receivers.values())
        monitor.save_csv()
        with next(monitor.output.glob('数据_*.csv')).open(encoding='utf-8-sig',newline='') as stream:
            assert len(list(csv.DictReader(stream))) == 650
        monitor.save_log()
        exported = next(monitor.output.glob('导出日志_*.txt')).read_text('utf-8')
        assert 'backlog 649' in exported
    finally:
        monitor.log_file.close()
        monitor.data_file.close()

def test_camera_cannot_reuse_a_live_data_port(monkeypatch,tmp_path):
    monitor,gui = model(monkeypatch,tmp_path)
    receivers = controlled_receivers(monitor)
    try:
        monitor._start_receiver('data')
        gui.values['camera_port'] = gui.values['data_port']
        with pytest.raises(ValueError,match='另一路接收'):
            monitor.start_channel('camera')
        assert receivers['data'].running and not receivers['camera'].running
        assert receivers['camera'].starts == []
        assert monitor.bindings == {'data':('127.0.0.1',18081)}
    finally:
        monitor.log_file.close()
        monitor.data_file.close()

def test_csv_requested_path_creates_parent_and_preserves_partial_records(monkeypatch,tmp_path):
    monitor,gui = model(monkeypatch,tmp_path)
    try:
        sender = ('127.0.0.1',6666)
        monitor.receivers['data'].queue.put(Packet(time.time(),sender,
            struct.pack('<9f4i2I',*range(9),-20,30,-40,50,300,400)))
        monitor.receivers['data'].queue.put(Packet(time.time(),sender,struct.pack('<3f',1.5,2.5,3.5)))
        monitor.receive()
        requested = tmp_path/'自定义目录'/'指定文件'
        gui.values['csv_path'] = '  '+str(requested)+'  '
        monitor.save_csv()
        actual = requested.with_suffix('.csv')
        assert actual.is_file()
        assert actual.read_bytes().startswith(b'\xef\xbb\xbf')
        with actual.open(encoding='utf-8-sig',newline='') as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 2
        assert rows[0]['enc_FL'] == '-20' and rows[0]['seq'] == '400'
        assert rows[1]['roll'] == '1.5' and rows[1]['enc_FL'] == ''
        assert rows[1]['source_ip'] == sender[0] and rows[1]['source_port'] == '6666'
        assert not list(monitor.output.glob('数据_*.csv'))
    finally:
        monitor.log_file.close()
        monitor.data_file.close()


def test_live_tuning_callbacks_and_split_logs_require_ack(monkeypatch,tmp_path):
    monitor,gui = model(monkeypatch,tmp_path)
    receivers = controlled_receivers(monitor)
    sent = []
    monitor.tuning.sender = lambda *args:sent.append(args)
    gui.values.update(remote_ip='127.0.0.1',remote_port=6666,tune_edit_omega_kp1=1.7,
                      tuning_file_path=str(tmp_path/'scheme.json'))
    from live_tuning import PARAMETERS
    for p in PARAMETERS:
        gui.values.setdefault('tune_edit_'+p.key,p.default)
    try:
        with pytest.raises(ValueError,match='日志监听'):
            monitor.apply_tuning('omega_kp1')
        assert sent == []
        receivers['log'].running = True
        monitor.apply_tuning('omega_kp1')
        rid = int(sent[0][0].split()[2])
        assert '等待' in gui.values['tune_state_omega_kp1']
        assert gui.values['tune_actual_omega_kp1'] == '未确认'
        response = f'[PARAM] {rid} OK omega_kp1 1.7\r\n'.encode()
        stamp = time.time()
        receivers['log'].queue.put(Packet(stamp,('127.0.0.2',6666),response))
        monitor.receive()
        assert not monitor.tuning.actual
        assert gui.values['remote_ip'] == '127.0.0.1'
        receivers['log'].queue.put(Packet(stamp,('127.0.0.1',6666),response[:12]))
        receivers['log'].queue.put(Packet(stamp,('127.0.0.1',6666),response[12:]))
        monitor.receive()
        monitor.refresh_tuning()
        assert gui.values['tune_actual_omega_kp1'] == '1.7'
        monitor.export_tuning()
        gui.values['tune_edit_omega_kp1'] = 1.8
        monitor.import_tuning()
        assert gui.values['tune_edit_omega_kp1'] == 1.7 and len(sent) == 1
        gui.values['remote_port'] = 7000
        monitor.apply_tuning('omega_kp1')
        gui.values['remote_port'] = 6666
        monitor.refresh_tuning()
        assert not monitor.tuning.pending and not monitor.tuning.actual
        assert '发送对象' in gui.values['tuning_status']
    finally:
        monitor.log_file.close()
        monitor.data_file.close()
