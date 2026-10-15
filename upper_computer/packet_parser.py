"""Reconstructed from the original packet_parser bytecode; independent of the GUI."""
from dataclasses import dataclass
import math
import struct

TYPE_FORMATS = dict(float32='f', float64='d', int8='b', uint8='B', int16='h',
                    uint16='H', int32='i', uint32='I', int64='q', uint64='Q')

@dataclass(frozen=True)
class FieldRule:
    name: str
    data_type: str
    offset: int
    byte_order: str = 'little'

    @property
    def size(self):
        return struct.calcsize(TYPE_FORMATS[self.data_type])

def parse_packet(packet, rules):
    values, errors = {}, {}
    for rule in rules:
        name = rule.name.strip() or '未命名字段'
        try:
            if rule.offset < 0:
                raise ValueError('起始字节不能为负数')
            if rule.byte_order not in ('little', 'big'):
                raise ValueError('字节序只能为 little / big')
            fmt = TYPE_FORMATS[rule.data_type]
            if rule.offset + rule.size > len(packet):
                raise ValueError(f'包长 {len(packet)}，无法读取偏移 {rule.offset}')
            value = struct.unpack_from(('<' if rule.byte_order == 'little' else '>') + fmt,
                                       packet, rule.offset)[0]
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError('非有限数值 NaN / Inf')
            values[name] = value
        except (ValueError, KeyError, struct.error) as exc:
            errors[name] = str(exc)
    return values, errors

DEFAULT_RULES = [FieldRule(n, 'float32', i * 4) for i, n in enumerate(('roll', 'pitch', 'yaw'))]
MONITOR_RULES = DEFAULT_RULES + [FieldRule(n, 'float32', (i + 3) * 4)
    for i, n in enumerate(('gx_dps', 'gy_dps', 'gz_dps', 'ax_g', 'ay_g', 'az_g'))] + [
    FieldRule(n, 'int32', 36 + i * 4) for i, n in enumerate(('enc_FL', 'enc_FR', 'enc_RL', 'enc_RR'))] + [
    FieldRule('samples', 'uint32', 52), FieldRule('seq', 'uint32', 56)]

# 108-byte speed-loop packet retains the entire 60-byte IMU/encoder prefix.
SPEED_RULES = MONITOR_RULES + [
    FieldRule(f'{kind}_{wheel}', 'float32', base + i * 4)
    for kind, base in (('target', 60), ('speed', 76), ('duty', 92))
    for i, wheel in enumerate(('FL', 'FR', 'RL', 'RR'))]

# 148-byte track packet; every previous protocol remains readable through its prefix.
TRACK_RULES = SPEED_RULES + [FieldRule(name, 'float32', 108 + i*4)
    for i, name in enumerate(('vis_err_px', 'omega_cmd_dps', 'vx_cmd_cms', 'vy_cmd_cms'))] + [
    FieldRule(name, 'uint32', 124 + i*4) for i, name in enumerate((
        'vis_time_us', 'vis_interval_us', 'vis_frames', 'wifi_time_us', 'wifi_peak_us', 'vision_event'))]

# --- 152-byte track packet extension -----------------------------------------
# The firmware appends a little-endian uint32 curve_level at offset 148 while
# keeping the first 148 bytes byte-for-byte identical. It is the controller's
# confirmed/held curve state, decoded independently of the user's rule list so
# an old saved rule set cannot hide it, and a short legacy packet is reported
# as unavailable instead of being assumed to be a straight line.
CURVE_LEVEL_OFFSET = 148
CURVE_LEVEL_SIZE = 4
CURVE_LEVEL_EXPIRY_SECONDS = 1.0
CURVE_LEVEL_LABELS = {0: '直道', 1: '小弯', 2: '紧弯'}


def decode_curve_level(packet):
    """Decode the 152-byte track extension at offset 148.

    Returns ``(state, level)``:
      * ``('ok', 0|1|2)``          valid controller-confirmed state
      * ``('invalid', value)``     extension present but value is not 0/1/2
      * ``('unavailable', None)``  packet shorter than the 152-byte layout

    Short 12/60/108/148-byte packets yield ``unavailable``; an out-of-range
    value yields ``invalid`` and is never silently treated as straight.
    """
    if len(packet) < CURVE_LEVEL_OFFSET + CURVE_LEVEL_SIZE:
        return 'unavailable', None
    value = struct.unpack_from('<I', packet, CURVE_LEVEL_OFFSET)[0]
    if value in CURVE_LEVEL_LABELS:
        return 'ok', value
    return 'invalid', value


@dataclass(frozen=True)
class CurveStatus:
    state: str
    level: object
    label: str
    text: str


def curve_status(state, level=None, age=None, expiry=CURVE_LEVEL_EXPIRY_SECONDS):
    """Build display text for a decoded curve state.

    ``age`` is seconds since the DATA packet that carried the extension (wall
    clock). It is only consulted for a valid state; once it exceeds ``expiry``
    the status is reported as stale rather than currently held.
    """
    if state == 'ok' and level in CURVE_LEVEL_LABELS:
        label = CURVE_LEVEL_LABELS[level]
        if age is not None and age > expiry:
            return CurveStatus('stale', level, label,
                               f'控制器状态：{label}（状态过期，最后更新 {age:.1f} 秒前）')
        return CurveStatus('ok', level, label, f'控制器状态：{label}（curve_level={level}）')
    if state == 'invalid':
        shown = '未知' if level is None else str(level)
        return CurveStatus('invalid', level, '未知',
                           f'控制器状态：未知（curve_level={shown} 无效，未按直道处理）')
    return CurveStatus('unavailable', None, '',
                       '控制器状态：不可用（等待新固件 152 字节数据）')
