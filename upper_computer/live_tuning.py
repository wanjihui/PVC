"""MCAR runtime tuning: finite, bounded values and explicit MCU acknowledgement.

No command here arms motors. Import/export affects local editor values only.
"""
from dataclasses import dataclass, field
import ipaddress
import json
import math
from pathlib import Path
import secrets
import time


@dataclass(frozen=True)
class Parameter:
    key: str
    label: str
    default: float
    minimum: float
    maximum: float
    help: str
    integer: bool = False
    step: int = 1

    def validate(self, value):
        if isinstance(value, bool):
            raise ValueError('参数不能为布尔值')
        try:
            value = float(value)
        except (ValueError, TypeError, OverflowError):
            raise ValueError('请输入有限数字') from None
        if not math.isfinite(value) or not self.minimum <= value <= self.maximum:
            raise ValueError(f'{self.label} 范围 {self.minimum:g}～{self.maximum:g}')
        if self.integer and (not value.is_integer() or int(value) % self.step):
            raise ValueError(f'{self.label} 必须为 {self.step} 的整数倍')
        return int(value) if self.integer else value

    def validate_ack(self, value):
        """MCU float32 %.9g can print a bound such as 0.1f as 0.100000001."""
        try:
            value = float(value)
        except (ValueError,TypeError,OverflowError):
            raise ValueError('无效车辆回执') from None
        if value < self.minimum and math.isclose(value,self.minimum,rel_tol=2e-7,abs_tol=1e-10):
            value = self.minimum
        if value > self.maximum and math.isclose(value,self.maximum,rel_tol=2e-7,abs_tol=1e-10):
            value = self.maximum
        return self.validate(value)


PARAMETERS = [
    Parameter('cruise_cms','直道速度上限 / cm/s',150,0,200,'弯道速度另行设置，且不超过此值；0不等于停车，停车按K2。'),
    Parameter('ramp_ms','起步时间 / ms',500,100,2000,'仅停车时修改；加长可减轻起步打滑。',True,5),
    Parameter('preview_row','固定前瞻行',60,20,100,'默认取值50～70行；小看远，大看近，绿线跟随实际帧更新。',True),
    Parameter('deadband_px','视觉死区 / px',2,0,10,'减小可修正小偏差；过小会追随图像噪声。'),
    Parameter('omega_kp1','视觉比例 Kp',1.6,0,3,'增大同一偏差的转向指令；过大左右摆动。'),
    Parameter('omega_kp2','视觉非线性 Kp2',.012,0,.03,'增大时，大偏差获得更强转向；所有非零偏差均参与。'),
    Parameter('omega_kd','视觉差分 Kd',.3,0,2,'按相邻图像偏差变化纠偏；过大对跳变敏感。'),
    Parameter('omega_max_dps','转向上限 / °/s',240,30,360,'提高允许更大轮速差；只在转向达到限制时生效。'),
    Parameter('gyro_kd','陀螺反馈系数',.3,0,1,'增大可抑制实际旋转；过大会削弱转向、使入弯迟缓。'),
    Parameter('lateral_gain','横移比例',.1,0,1,'少量横移辅助贴线；增大可能过冲，设0关闭。'),
] + [
    Parameter(f'{gain}_{wheel}',f'{wheel} 速度环 {gain.upper()}',default,0,maximum,help_text)
    for wheel in ('FL','FR','RL','RR')
    for gain,default,maximum,help_text in (
        ('kp',{'FL':.22,'FR':.20,'RL':.18,'RR':.20}[wheel],.5,'增大轮速误差的即时补偿；过大导致占空比振荡。'),
        ('ki',.001,.01,'累积补偿持续轮速误差；过大易过冲，先小幅递增。'),
        ('kd',0,.1,'抑制轮速误差变化；编码器噪声会放大，先保持 0。'))
] + [Parameter('ff','正误差附加增益',0,0,.1,'仅目标轮速大于实测时增加 ff×(目标－实测) 输出；先保持 0。')]
BASE_KEYS = {p.key for p in PARAMETERS}
PARAMETERS += [
    Parameter('curve_enable','弯道减速开关',1,0,1,'1开启、0直弯同速；仅停车修改，不改变发车状态。',True),
    Parameter('curve_normal_cms','普通弯速度 / cm/s',120,5,200,'普通弯前进上限，不超过直道速度；须不低于紧弯速度。'),
    Parameter('curve_tight_cms','紧弯速度 / cm/s',100,5,200,'紧弯前进上限；须不高于普通弯速度。'),
    Parameter('curve_enter_dps','普通弯进入需求 / °/s',70,1,720,'视觉转向需求达到此值开始确认；提高可减少普通弯降速。'),
    Parameter('curve_tight_dps','紧弯进入需求 / °/s',140,1,1440,'须大于普通弯阈值；提高可减少紧弯降速。'),
    Parameter('curve_confirm_frames','入弯确认帧数',2,1,10,'连续新图像确认；提高可过滤尖峰，也会推迟降速。',True),
    Parameter('curve_exit_dps','出弯视觉需求 / °/s',45,0,720,'低于普通弯进入阈值；满足此项及偏差/转速条件才累计稳定时间。'),
    Parameter('curve_exit_err_px','出弯偏差上限 / px',12,0,100,'放宽更早恢复速度；过宽可能还未贴线就加速。'),
    Parameter('curve_exit_gyro_dps','出弯实际转速上限 / °/s',40,0,360,'放宽可更早恢复速度，也可能在车身仍旋转时加速。'),
    Parameter('curve_exit_stable_ms','出弯稳定时间 / ms',200,20,2000,'三项条件连续成立的时间；5ms整数倍。',True,5),
    Parameter('curve_frame_max_age_ms','视觉最大有效年龄 / ms',100,20,500,'旧帧超过此值不加速、不累计出弯稳定时间；5ms整数倍。',True,5),
    Parameter('curve_decel_cms2','入弯减速率 / cm/s²',250,1,2000,'前进目标下降速率；过大容易急减速，实际减速仍取决于速度环。'),
    Parameter('curve_accel_cms2','出弯加速率 / cm/s²',100,1,1000,'稳定出弯后的前进目标恢复速率；起步另由起步时间控制。'),
]
BY_KEY = {p.key:p for p in PARAMETERS}
# Keep all firmware definitions for GET compatibility, but never expose/send wheel-loop edits.
SPEED_LOOP_KEYS = {f'{gain}_{wheel}' for gain in ('kp','ki','kd')
                   for wheel in ('FL','FR','RL','RR')} | {'ff'}
EDITABLE_PARAMETERS = [p for p in PARAMETERS if p.key not in SPEED_LOOP_KEYS]
EDITABLE_BY_KEY = {p.key:p for p in EDITABLE_PARAMETERS}


@dataclass
class Request:
    id: int
    payload: bytes
    target: tuple
    key: str | None
    value: float | None
    started_wall: float
    next_try: float
    attempts: int = 1
    values: dict = field(default_factory=dict)


class TuningClient:
    def __init__(self, sender, clock=time.monotonic, wall_clock=time.time):
        self.sender, self.clock, self.wall_clock = sender, clock, wall_clock
        self.pending = {}
        self.actual = {}
        self.states = {p.key:'未读取' for p in PARAMETERS}
        self.message = '先启动 01 的日志监听，再读取车辆参数。编辑和导入不会自动发送。'
        self.next_id = secrets.randbelow(2**30)
        self.peer = None
        self.last_seen = None
        self.connected = False

    @staticmethod
    def target(ip, port):
        address = ipaddress.ip_address(ip.strip())
        if address.version != 4 or address.is_unspecified or address.is_multicast:
            raise ValueError('请填写车辆模块 IPv4 地址')
        if isinstance(port,bool) or not isinstance(port,int) or not 1 <= port <= 65535:
            raise ValueError('命令端口必须为 1～65535 的整数')
        return str(address),port

    def invalidate(self, reason):
        for req in self.pending.values():
            for key in ([req.key] if req.key else BY_KEY):
                self.states[key] = '未确认：'+reason
        self.pending.clear()
        for key in self.actual:
            self.states[key] = '旧值：'+reason+'；请重新读取'
        self.actual.clear()
        self.connected = False
        self.message = reason+'；已确认值已失效，请重新读取。'

    def observe(self, ip):
        if self.peer == ip:
            self.last_seen = self.clock()
            self.connected = True

    def _begin(self, key, value, ip, port):
        target = self.target(ip,port)
        if self.peer is not None and self.peer != target[0]:
            self.invalidate('发送对象已切换')
        self.peer = target[0]
        if any(req.key is None for req in self.pending.values()) or (key is None and self.pending):
            raise ValueError('请等待当前读取或应用结束后再读取全部')
        for rid,req in list(self.pending.items()):
            if req.key == key:
                del self.pending[rid]  # Superseded ACK must not overwrite a newer edit.
        rid = self.next_id
        self.next_id = (rid+1) % 2**31
        command = f'MCAR GET {rid} ALL\n' if key is None else f'MCAR SET {rid} {key} {value:.9g}\n'
        payload = command.encode('ascii')
        self.sender(payload,*target)
        req = Request(rid,payload,target,key,value,self.wall_clock(),self.clock()+1)
        self.pending[rid] = req
        for affected in ([key] if key else BY_KEY):
            self.states[affected] = '等待车辆确认'
        self.message = '请求已发送，等待车辆确认；最多发送 3 次。'
        return rid

    def read_all(self, ip, port):
        return self._begin(None,None,ip,port)

    def apply(self, key, value, ip, port):
        if key not in EDITABLE_BY_KEY:
            raise ValueError('此参数不提供上位机调节')
        return self._begin(key,EDITABLE_BY_KEY[key].validate(value),ip,port)

    def feed_line(self, text, ip, stamp=None):
        self.observe(ip)
        if self.peer == ip and 'MCAR_APP built ' in text:
            self.invalidate('车辆已重新启动')
            self.last_seen = self.clock()
            return False
        words = text.split()
        if len(words) < 3 or words[0] != '[PARAM]':
            return False
        try:
            rid = int(words[1])
        except ValueError:
            return False
        req = self.pending.get(rid)
        if req is None or ip != req.target[0] or self.clock() >= req.next_try and req.attempts >= 3:
            return False
        if stamp is not None and stamp < req.started_wall:
            return False
        kind = words[2]
        if kind == 'ERR' and len(words) >= 4:
            reason = ' '.join(words[3:])
            friendly = {'STOP_FIRST':'请先按 K2 停车再修改', 'RANGE':'参数超出范围',
                        'INTEGER':'参数必须是整数', 'STEP_5MS':'起步时间必须是 5 ms 整数倍',
                        'UNKNOWN_KEY':'车辆固件不支持此参数', 'SYNTAX':'命令格式错误',
                        'NUMBER':'数字格式错误',
                        'CURVE_ORDER':'须满足紧弯速度≤普通弯速度，出弯需求<普通弯需求<紧弯需求'}.get(reason,reason)
            for key in ([req.key] if req.key else BY_KEY):
                self.states[key] = '拒绝：'+friendly
            del self.pending[rid]
            self.message = '车辆拒绝参数请求：'+friendly+'（'+reason+'）'
            return True
        if kind == 'DONE' and len(words) == 3 and req.key is None:
            legacy = req.values.keys() == BASE_KEYS
            if req.values.keys() != BY_KEY.keys() and not legacy:
                self.message = f'读取不完整：{len(req.values)}/{len(BY_KEY)}，仍等待回包或重试。'
                return False
            self.actual.update(req.values)
            for key in req.values:
                self.states[key] = '已读取（车辆确认）'
            if legacy:
                for key in BY_KEY.keys()-BASE_KEYS:
                    self.actual.pop(key,None)
                    self.states[key] = '固件未提供：请下载支持弯道调参的新版'
            del self.pending[rid]
            shown_count = len(req.values.keys() & EDITABLE_BY_KEY.keys())
            self.message = f'已读取 {shown_count} 项可调参数（车辆回传 {len(req.values)} 项）。'+('旧固件未提供弯道参数，编辑值不代表已应用。' if legacy else '')
            return True
        if kind != 'OK' or len(words) != 5 or words[3] not in BY_KEY:
            return False
        key = words[3]
        try:
            value = BY_KEY[key].validate_ack(words[4])
        except ValueError:
            return False
        if req.key is None:
            req.values[key] = value
            self.states[key] = '已收到，等待读取完成'
            return True
        if key != req.key or not math.isclose(value,req.value,rel_tol=1e-5,abs_tol=1e-7):
            return False
        self.actual[key] = value
        self.states[key] = '已应用（车辆确认）'
        self.message = f'{BY_KEY[key].label} 已应用：{value:g}。'
        del self.pending[rid]
        return True

    def tick(self):
        now = self.clock()
        if self.connected and self.last_seen is not None and now-self.last_seen > 3:
            self.invalidate('车辆数据连接超时')
        for rid,req in list(self.pending.items()):
            if now < req.next_try:
                continue
            if req.attempts >= 3:
                for key in ([req.key] if req.key else BY_KEY):
                    self.states[key] = '未确认：超时'
                self.message = '请求超时，不能确认车辆是否执行；请重新读取。'
                del self.pending[rid]
                continue
            try:
                self.sender(req.payload,*req.target)
            except OSError as exc:
                self.message = '重试发送失败：'+str(exc)
            req.attempts += 1
            req.next_try = now+1


def save_scheme(path, values):
    clean = {key:EDITABLE_BY_KEY[key].validate(value) for key,value in values.items() if key in EDITABLE_BY_KEY}
    if len(clean) != len(EDITABLE_BY_KEY):
        raise ValueError(f'方案必须包含全部 {len(EDITABLE_BY_KEY)} 项可调参数')
    validate_curve_scheme(clean)
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps({'format':'MCAR_TUNING_3','values':clean},ensure_ascii=False,indent=2),encoding='utf-8')
    temporary.replace(path)


def load_scheme(path):
    obj = json.loads(Path(path).read_text('utf-8'))
    if not isinstance(obj,dict) or obj.get('format') not in ('MCAR_TUNING_1','MCAR_TUNING_2','MCAR_TUNING_3') or not isinstance(obj.get('values'),dict):
        raise ValueError('不是 MCAR 参数方案')
    values = obj['values']
    if obj['format'] == 'MCAR_TUNING_1' and values.keys() == BASE_KEYS:
        values = {**{p.key:p.default for p in PARAMETERS if p.key not in BASE_KEYS},**values}
    expected = EDITABLE_BY_KEY.keys() if obj['format'] == 'MCAR_TUNING_3' else BY_KEY.keys()
    if values.keys() != expected:
        raise ValueError('方案参数与当前版本不一致')
    clean = {key:EDITABLE_BY_KEY[key].validate(value) for key,value in values.items() if key in EDITABLE_BY_KEY}
    validate_curve_scheme(clean)
    return clean


def validate_curve_scheme(values):
    if not (values['curve_tight_cms'] <= values['curve_normal_cms'] and
            values['curve_exit_dps'] < values['curve_enter_dps'] < values['curve_tight_dps']):
        raise ValueError('须满足紧弯速度≤普通弯速度，出弯需求<普通弯需求<紧弯需求')
