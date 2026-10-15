"""Unit tests for the 152-byte track curve_level extension.

Pure decode/status logic only: no GUI is built and no network socket is opened.
"""
import struct

import pytest

from app import merge_curve_level
from packet_parser import (CURVE_LEVEL_EXPIRY_SECONDS, CURVE_LEVEL_LABELS,
                           DEFAULT_RULES, MONITOR_RULES, SPEED_RULES, TRACK_RULES,
                           curve_status, decode_curve_level, parse_packet)


def track_packet(level=None, length=152):
    """A DATA payload with curve_level written little-endian at offset 148."""
    packet = bytearray(length)
    if level is not None:
        struct.pack_into('<I', packet, 148, level)
    return bytes(packet)


@pytest.mark.parametrize('level,label', [(0, '直道'), (1, '小弯'), (2, '紧弯')])
def test_valid_levels_decode_little_endian_and_display(level, label):
    packet = track_packet(level)
    assert packet[148] == level and packet[149:] == bytes(3)  # little-endian
    assert decode_curve_level(packet) == ('ok', level)
    status = curve_status('ok', level, age=0.0)
    assert status.state == 'ok' and status.label == label
    assert status.text == f'控制器状态：{label}（curve_level={level}）'


@pytest.mark.parametrize('length', [0, 12, 60, 108, 147, 148, 151])
def test_short_legacy_packets_are_unavailable_not_straight(length):
    packet = bytes(length)
    assert decode_curve_level(packet) == ('unavailable', None)
    status = curve_status('unavailable')
    assert status.state == 'unavailable' and status.level is None
    assert '不可用' in status.text and '直道' not in status.text


@pytest.mark.parametrize('level', [3, 4, 255, 0xFFFFFFFF])
def test_invalid_extension_is_unknown_not_straight(level):
    packet = track_packet(level)
    assert decode_curve_level(packet) == ('invalid', level)
    status = curve_status('invalid', level)
    assert status.state == 'invalid' and status.label == '未知'
    assert '未知' in status.text and '控制器状态：直道' not in status.text


def test_status_is_fresh_up_to_expiry_and_stale_after():
    assert curve_status('ok', 1, age=0.0).state == 'ok'
    # Exactly at the expiry boundary the state is still considered current.
    assert curve_status('ok', 1, age=CURVE_LEVEL_EXPIRY_SECONDS).state == 'ok'
    stale = curve_status('ok', 1, age=CURVE_LEVEL_EXPIRY_SECONDS + 0.001)
    assert stale.state == 'stale' and stale.label == '小弯'
    assert '过期' in stale.text and '小弯' in stale.text
    # No age at all (never timestamped) is not reported stale by itself.
    assert curve_status('ok', 2, age=None).state == 'ok'


def test_every_state_has_a_distinct_text_prefix():
    assert '不可用' in curve_status('unavailable').text
    assert '未知' in curve_status('invalid', 7).text
    assert '过期' in curve_status('ok', 0, age=5.0).text
    assert '直道' in curve_status('ok', 0).text


@pytest.mark.parametrize('length,rules', [(12, DEFAULT_RULES), (60, MONITOR_RULES),
                                          (108, SPEED_RULES), (148, TRACK_RULES)])
def test_legacy_layouts_still_parse_without_extension_errors(length, rules):
    values, errors = parse_packet(bytes(length), rules)
    assert not errors and values
    state, level = decode_curve_level(bytes(length))
    assert state == 'unavailable' and level is None


def test_curve_level_recorded_even_when_saved_rules_omit_it():
    packet = track_packet(2)
    values, errors = parse_packet(packet, DEFAULT_RULES)  # old 3-angle rule list
    assert 'curve_level' not in values and not errors
    state, level = merge_curve_level(values, packet)
    assert (state, level) == ('ok', 2)
    assert values['curve_level'] == 2  # recorded for CSV/history regardless of rules


def test_merge_does_not_touch_values_for_legacy_148_byte_packet():
    values = {}
    state, level = merge_curve_level(values, bytes(148))
    assert (state, level) == ('unavailable', None)
    assert 'curve_level' not in values


def test_invalid_value_is_recorded_raw_and_never_shown_as_straight():
    packet = track_packet(9)
    values = {}
    state, level = merge_curve_level(values, packet)
    assert state == 'invalid' and level == 9
    assert values['curve_level'] == 9
    assert curve_status(state, level).label != '直道'


def test_label_table_matches_firmware_contract():
    assert CURVE_LEVEL_LABELS == {0: '直道', 1: '小弯', 2: '紧弯'}


def test_new_extension_records_through_actual_app_receive_and_csv(monkeypatch, tmp_path):
    import json
    import time
    from argparse import Namespace
    import app
    from test_app_model import GuiStub
    from transport import Packet
    from multiplex_protocol import encode_packet
    gui = GuiStub()
    gui.values['csv_path'] = str(tmp_path / 'status.csv')
    monkeypatch.setattr(app, 'STORE', tmp_path)
    monkeypatch.setattr(app, 'dpg', gui)
    monkeypatch.setattr(app.Monitor, '_build', lambda self: None)
    monitor = app.Monitor(Namespace(test_ports=True))
    monitor.rules = list(DEFAULT_RULES)  # saved legacy rules must not hide controller status
    try:
        monitor.multiplex.receiver.queue.put(Packet(time.time(), ('127.0.0.1', 6666),
                                                    encode_packet('data', track_packet(2))))
        monitor.receive()
        assert monitor.curve_state == 'ok' and monitor.curve_level == 2
        assert monitor.history['curve_level'][-1][1] == 2
        monitor.save_csv()
        import csv
        with (tmp_path / 'status.csv').open(encoding='utf-8-sig', newline='') as stream:
            rows = list(csv.DictReader(stream))
        assert rows[0]['curve_level'] == '2'
        monitor.reset_curve_status()
        assert monitor.curve_state == 'unavailable' and monitor.curve_stamp == 0
    finally:
        monitor.log_file.close()
        monitor.data_file.close()
