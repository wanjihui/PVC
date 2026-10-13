import json
import socket
import time
import pytest
from debug_log import LineDecoder
from live_tuning import BY_KEY, PARAMETERS, EDITABLE_PARAMETERS, EDITABLE_BY_KEY, SPEED_LOOP_KEYS, TuningClient, load_scheme, save_scheme
from transport import UdpReceiver, send_udp


def client():
    sent = []
    clock = [0.0]
    instance = TuningClient(lambda *args:sent.append(args),lambda:clock[0],lambda:100.0+clock[0])
    return instance,sent,clock


@pytest.mark.parametrize('key,value',[
    ('cruise_cms',201),('cruise_cms',-1),('gyro_kd','nan'),('ff','inf'),
    ('omega_kp1',True),('ramp_ms',503),('preview_row',55.5),('preview_row',19),('kp_FL',.501)])
def test_unsafe_values_do_not_send(key,value):
    c,sent,_ = client()
    with pytest.raises(ValueError):
        c.apply(key,value,'127.0.0.1',6666)
    assert not sent


def test_ack_matches_id_peer_key_value_and_time():
    c,sent,clock = client()
    rid = c.apply('omega_kp1',1.7,'127.0.0.1',6666)
    assert sent[0][0] == f'MCAR SET {rid} omega_kp1 1.7\n'.encode()
    assert c.actual == {}
    for text,peer,stamp in [
        (f'[PARAM] {rid+1} OK omega_kp1 1.7','127.0.0.1',100),
        (f'[PARAM] {rid} OK omega_kp1 1.7','127.0.0.2',100),
        (f'[PARAM] {rid} OK omega_kp2 1.7','127.0.0.1',100),
        (f'[PARAM] {rid} OK omega_kp1 1.8','127.0.0.1',100),
        (f'[PARAM] {rid} OK omega_kp1 1.7','127.0.0.1',99),
        (f'[PARAM] {rid} OK omega_kp1 nan','127.0.0.1',100),
    ]:
        assert not c.feed_line(text,peer,stamp)
        assert not c.actual
    assert c.feed_line(f'[PARAM] {rid} OK omega_kp1 1.70000005','127.0.0.1',100)
    assert c.actual['omega_kp1'] == 1.70000005 and not c.pending
    assert '已应用' in c.states['omega_kp1']


@pytest.mark.parametrize('key', ['ff','kd_FL','kd_FR','kd_RL','kd_RR'])
def test_float32_upper_bound_ack_without_relaxing_input(key):
    assert BY_KEY[key].validate_ack(.100000001) == .1
    with pytest.raises(ValueError):
        BY_KEY[key].validate(.100000001)


def test_retries_reuse_id_and_timeout_is_not_success():
    c,sent,clock = client()
    rid = c.apply('gyro_kd',.09,'127.0.0.1',6666)
    for tick in (1,2,3):
        clock[0] = tick
        c.tick()
    assert len(sent) == 3 and sent[0] == sent[1] == sent[2]
    assert not c.pending and not c.actual
    assert '超时' in c.states['gyro_kd']
    assert not c.feed_line(f'[PARAM] {rid} OK gyro_kd .09','127.0.0.1',103)


def test_superseded_ack_does_not_overwrite_new_value():
    c,_,clock = client()
    old = c.apply('omega_kp1',1.7,'127.0.0.1',6666)
    new = c.apply('omega_kp1',1.8,'127.0.0.1',6666)
    assert not c.feed_line(f'[PARAM] {old} OK omega_kp1 1.7','127.0.0.1',100)
    assert c.feed_line(f'[PARAM] {new} OK omega_kp1 1.8','127.0.0.1',100)
    c.feed_line('MCAR_APP built Oct 9 2026 00:01:00','127.0.0.1',100)
    assert not c.actual and '旧值' in c.states['omega_kp1']
    new = c.apply('omega_kp1',1.9,'127.0.0.1',6666)
    c.feed_line(f'[PARAM] {new} OK omega_kp1 1.9','127.0.0.1',100)
    clock[0] = 3.1
    c.tick()
    assert not c.actual and '超时' in c.message


def test_get_partial_done_and_wrong_format_do_not_claim_complete():
    c,_,clock = client()
    rid = c.read_all('127.0.0.1',6666)
    assert c.feed_line(f'[PARAM] {rid} OK cruise_cms 150','127.0.0.1',100)
    assert not c.feed_line(f'[PARAM] {rid} DONE','127.0.0.1',100)
    assert not c.actual and rid in c.pending
    for p in PARAMETERS:
        c.feed_line(f'[PARAM] {rid} OK {p.key} {p.default}','127.0.0.1',100)
    assert c.feed_line(f'[PARAM] {rid} DONE','127.0.0.1',100)
    assert len(c.actual) == len(PARAMETERS) and not c.pending


def test_error_and_peer_change_drop_pending():
    c,_,_ = client()
    rid = c.apply('ramp_ms',500,'127.0.0.1',6666)
    assert c.feed_line(f'[PARAM] {rid} ERR STOP_REQUIRED','127.0.0.1',100)
    assert not c.actual and 'STOP_REQUIRED' in c.states['ramp_ms']
    rid = c.apply('gyro_kd',.09,'127.0.0.1',6666)
    c.apply('omega_kp1',1.7,'127.0.0.2',6666)
    assert not c.feed_line(f'[PARAM] {rid} OK gyro_kd .09','127.0.0.1',100)


def test_scheme_round_trip_and_bad_import_never_send(tmp_path):
    c,sent,_ = client()
    path = tmp_path/'方案.json'
    values = {p.key:p.default for p in EDITABLE_PARAMETERS}
    values['omega_kp1'] = 1.75
    save_scheme(path,values)
    assert load_scheme(path) == values and not sent
    bad = json.loads(path.read_text('utf-8'))
    bad['values']['gyro_kd'] = float('nan')
    path.write_text(json.dumps(bad),encoding='utf-8')
    with pytest.raises(ValueError):
        load_scheme(path)
    assert not sent


def test_real_udp_split_lines_and_get_round_trip():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as module:
        module.bind(('127.0.0.1',0))
        module.settimeout(1)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1',0))
            log_port = probe.getsockname()[1]
        receiver = UdpReceiver()
        receiver.start('127.0.0.1',log_port)
        c = TuningClient(send_udp)
        decoder = LineDecoder()
        try:
            rid = c.read_all('127.0.0.1',module.getsockname()[1])
            payload,_ = module.recvfrom(4096)
            assert payload == f'MCAR GET {rid} ALL\n'.encode()
            reply = ''.join(f'[PARAM] {rid} OK {p.key} {p.default}\r\n' for p in PARAMETERS)+f'[PARAM] {rid} DONE\r\n'
            module.sendto(reply[:11].encode(),('127.0.0.1',log_port))
            module.sendto(reply[11:107].encode(),('127.0.0.1',log_port))
            module.sendto(reply[107:].encode(),('127.0.0.1',log_port))
            deadline = time.monotonic()+1
            while c.pending and time.monotonic() < deadline:
                for packet in receiver.drain(50):
                    for line in decoder.feed(packet):
                        c.feed_line(line.text,packet.address[0],line.stamp)
                time.sleep(.01)
            assert len(c.actual) == len(PARAMETERS) and not c.pending
            rid = c.apply('omega_kp1',1.7,'127.0.0.1',module.getsockname()[1])
            payload,_ = module.recvfrom(4096)
            assert payload == f'MCAR SET {rid} omega_kp1 1.7\n'.encode()
            module.sendto(f'[PARAM] {rid} OK omega_kp1 1.700000\r\n'.encode(),('127.0.0.1',log_port))
            deadline = time.monotonic()+1
            while c.pending and time.monotonic() < deadline:
                for packet in receiver.drain(50):
                    for line in decoder.feed(packet):
                        c.feed_line(line.text,packet.address[0],line.stamp)
                time.sleep(.01)
            assert c.actual['omega_kp1'] == 1.7
        finally:
            receiver.stop()


def test_legacy_get_ack_marks_new_fields_unsupported():
    from live_tuning import BASE_KEYS
    c,_,_ = client()
    rid = c.read_all('127.0.0.1',6666)
    for p in PARAMETERS:
        if p.key in BASE_KEYS:
            c.feed_line(f'[PARAM] {rid} OK {p.key} {p.default}','127.0.0.1',100)
    assert c.feed_line(f'[PARAM] {rid} DONE','127.0.0.1',100)
    assert len(c.actual) == 23 and 'curve_tight_cms' not in c.actual
    assert '固件未提供' in c.states['curve_tight_cms']


def test_legacy_scheme_migrates_local_values_without_transmission(tmp_path):
    from live_tuning import BASE_KEYS
    values = {p.key:p.default for p in PARAMETERS if p.key in BASE_KEYS}
    values['preview_row'] = 55
    path = tmp_path/'old.json'
    path.write_text(json.dumps({'format':'MCAR_TUNING_1','values':values}),encoding='utf-8')
    migrated = load_scheme(path)
    assert migrated['preview_row'] == 55 and migrated['curve_tight_cms'] == 100
    assert len(migrated) == 23 and not (migrated.keys() & SPEED_LOOP_KEYS)


@pytest.mark.parametrize('key,value',[('curve_enable',2),('curve_enable',.5),
    ('curve_confirm_frames',0),('curve_exit_stable_ms',203),('curve_accel_cms2',0)])
def test_curve_invalid_values_do_not_send(key,value):
    c,sent,_ = client()
    with pytest.raises(ValueError): c.apply(key,value,'127.0.0.1',6666)
    assert not sent


def test_curve_order_rejection_is_shown_and_scheme_order_checked(tmp_path):
    c,_,_ = client()
    rid = c.apply('curve_tight_cms',150,'127.0.0.1',6666)
    assert c.feed_line(f'[PARAM] {rid} ERR CURVE_ORDER','127.0.0.1',100)
    assert '紧弯速度' in c.states['curve_tight_cms'] and not c.actual
    values = {p.key:p.default for p in PARAMETERS}
    values['curve_tight_cms'] = 150
    with pytest.raises(ValueError): save_scheme(tmp_path/'bad.json',values)


@pytest.mark.parametrize('key',sorted(SPEED_LOOP_KEYS))
def test_removed_speed_parameters_cannot_be_sent(key):
    c,sent,_ = client()
    with pytest.raises(ValueError,match='不提供'): c.apply(key,0,'127.0.0.1',6666)
    assert not sent


def test_full_old_scheme_import_filters_motor_gains_and_new_export_omits_them(tmp_path):
    path = tmp_path/'old36.json'
    values = {p.key:p.default for p in PARAMETERS}
    values.update(preview_row=60,curve_normal_cms=140,curve_tight_cms=120)
    path.write_text(json.dumps({'format':'MCAR_TUNING_2','values':values}),encoding='utf-8')
    clean = load_scheme(path)
    assert clean.keys() == EDITABLE_BY_KEY.keys() and clean['curve_tight_cms'] == 120
    save_scheme(path,values)
    saved = json.loads(path.read_text('utf-8'))
    assert saved['format'] == 'MCAR_TUNING_3' and saved['values'] == clean
    assert load_scheme(path) == clean
