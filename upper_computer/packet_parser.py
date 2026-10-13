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
