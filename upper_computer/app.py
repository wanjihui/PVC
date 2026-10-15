"""Mcar WiFi monitor. Maintained source reconstructed from the original program's bytecode.
Default: one MUDP v1 listener at 8081; legacy three-port mode remains selectable.
"""
import argparse
from collections import deque
from dataclasses import asdict
from datetime import datetime
import csv
import ctypes
import json
import os
from pathlib import Path
import shutil
import struct
import sys
import time
import traceback
import uuid

import dearpygui.dearpygui as dpg
from packet_parser import (DEFAULT_RULES, MONITOR_RULES, SPEED_RULES, TRACK_RULES, TYPE_FORMATS,
                           FieldRule, curve_status, decode_curve_level, parse_packet)
from transport import UdpReceiver, MultiplexReceiver, send_udp
from link_health import LinkHealth
from debug_log import LineDecoder, LogLine
from camera_protocol import CameraAssembler, frame_rgba, frame_bmp
from attitude_3d import draw_attitude
from quick_commands import FORMATS, ENDINGS, new_command, defaults as command_defaults, load_config, save_config, encode_command, validate_target
from live_tuning import EDITABLE_PARAMETERS, EDITABLE_BY_KEY, TuningClient, load_scheme, save_scheme

def storage_directory():
    # Prefer per-user storage; portable installs may run in a restricted profile.
    app_dir = Path(sys.executable).parent if getattr(sys,'frozen',False) else Path(__file__).parent
    candidates = [Path(os.environ.get('LOCALAPPDATA',str(Path.home())))/'McarWiFiMonitor_SingleUDP',
                  app_dir/'McarWiFiMonitorData_SingleUDP',Path.home()/'Documents'/'McarWiFiMonitor_SingleUDP']
    for candidate in candidates:
        try:
            candidate.mkdir(parents=True,exist_ok=True)
            # Windows tempfile retries PermissionError as a filename collision.
            # One exclusive probe avoids a long startup loop in a denied directory.
            probe = candidate / ('.write_probe_'+uuid.uuid4().hex)
            with probe.open('xb') as stream:
                stream.write(b'1')
            probe.unlink()
            return candidate
        except OSError:
            continue
    raise OSError('无法创建保存目录，请将程序放到可写的文件夹运行')

STORE = storage_directory()

def startup_rules(settings):
    """Default to the full track packet; migrate the old three-angle default once."""
    try:
        stored = settings.get('rules')
        rules = [FieldRule(**r) for r in stored] if stored else list(TRACK_RULES)
        if any(r.data_type not in TYPE_FORMATS or r.offset < 0 for r in rules):
            raise ValueError()
        if settings.get('rules_schema_version') != 2 and rules == DEFAULT_RULES:
            return list(TRACK_RULES)
        return rules
    except (ValueError,TypeError,KeyError):
        return list(TRACK_RULES)

def camera_display_rect(frame_width, frame_height, canvas_width, canvas_height, fit=True, zoom=4):
    """Size a complete frame for the view without changing camera pixels."""
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError('图像尺寸必须大于零')
    canvas_width,canvas_height = max(1,canvas_width),max(1,canvas_height)
    scale = min(canvas_width/frame_width,canvas_height/frame_height) if fit else max(1,zoom)
    width,height = frame_width*scale,frame_height*scale
    return width,height,max(0,(canvas_width-width)/2),max(0,(canvas_height-height)/2)

# Colour by display state so unavailable/invalid/stale stay visibly distinct.
CURVE_STATUS_COLORS = {'ok':(105,205,145),'stale':(255,190,75),
                       'invalid':(255,120,120),'unavailable':(142,150,164)}

def merge_curve_level(values, packet):
    """Record the 152-byte track extension even when the saved rules omit it.

    Returns ``(state, level)`` from :func:`decode_curve_level`; a short legacy
    packet leaves ``values`` untouched and reports ``unavailable``.
    """
    state, level = decode_curve_level(packet)
    if state != 'unavailable':
        values['curve_level'] = level
    return state, level

class Monitor:
    def __init__(self, args):
        STORE.mkdir(parents=True, exist_ok=True)
        self.output = STORE / 'captures'
        self.output.mkdir(exist_ok=True)
        try:
            self.settings = json.loads((STORE / 'settings.json').read_text('utf-8'))
        except (OSError, ValueError):
            self.settings = {}
        if not isinstance(self.settings,dict):
            self.settings = {}
        self.args = args
        self.quick_path = STORE/'quick_commands.json'
        self.quick_load_error = ''
        try:
            self.quick_config = load_config(self.quick_path)
        except FileNotFoundError:
            self.quick_config = command_defaults()
        except (OSError,ValueError) as exc:
            self.quick_config = command_defaults()
            self.quick_load_error = '命令列表读取失败：'+str(exc)+'；原文件保留，请检查或重新导入。'
        self.quick_rows = []
        self.quick_results = {}
        self._quick_save_due = 0
        self.tuning = TuningClient(send_udp)
        self.tuning_path = STORE/'tuning_scheme.json'
        self.tuning_edit = {p.key:p.default for p in EDITABLE_PARAMETERS}
        self.tuning_load_error = ''
        try:
            self.tuning_edit = load_scheme(self.tuning_path)
        except FileNotFoundError:
            pass
        except (OSError,ValueError) as exc:
            self.tuning_load_error = '本地方案读取失败：'+str(exc)+'；原文件保留。'
        self._tuning_signature = None
        self._observed_module = None
        self.single_port = not getattr(args,'legacy_ports',False) and self.settings.get('single_port',True)
        self.multiplex = MultiplexReceiver() if self.single_port else None
        self.receivers = self.multiplex.channels if self.multiplex else {n: UdpReceiver() for n in ('data','log','camera')}
        self.bindings = {}
        self.decoder = LineDecoder()
        self.camera = CameraAssembler()
        self.rules = []
        self.rule_rows = []
        self.latest = {}
        self.history = {}
        self.history_origin = None
        self.logs = deque(maxlen=10000)
        self.session = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.log_path = self.output / f'调试日志_{self.session}.txt'
        self.data_path = self.output / f'数据记录_{self.session}.jsonl'
        self.log_file = self.log_path.open('w',encoding='utf-8')
        self.data_file = self.data_path.open('w',encoding='utf-8')
        self.total_lines = self.data_records = self.parse_errors = 0
        self.frame = self.shown_frame = None
        self.data_times, self.camera_times = deque(), deque()
        self.last_data = self.last_camera = 0
        self.link = LinkHealth()
        self._receive_report_at = time.monotonic()
        self._last_refresh = self._last_flush = self._last_plot = 0
        self._log_dirty = True
        self._pending_log_scroll = False
        self._log_widgets = deque()
        self._visible_signature = None
        self._plot_names = []
        self._last_error = ''
        self._last_texture_size = (188,120)
        self.active_page = 'params'
        self.last_angles = (0.0,0.0,0.0)
        self.has_pose = False
        self.max_samples = 1200
        self._layout_size = None
        self._pose_signature = None
        self._camera_display_signature = None
        self.curve_state, self.curve_level, self.curve_stamp = 'unavailable', None, 0.0
        self._build()

    def reset_curve_status(self):
        """Forget the held curve state after a restart or receiver switch."""
        self.curve_state, self.curve_level, self.curve_stamp = 'unavailable', None, 0.0

    def status(self, text):
        dpg.set_value('status',text if len(text) <= 46 else text.split('：',1)[0]+'；详见运行日志。')
        self.append_logs([LogLine(time.time(),'上位机',text)])

    def append_logs(self, lines):
        for line in lines:
            if ':' in line.source:
                peer_ip = line.source.rsplit(':',1)[0]
                if '[INIT]' in line.text or 'MCAR_APP built ' in line.text or '[PARAM]' in line.text:
                    self.observe_module(peer_ip)
                self.tuning.feed_line(line.text,peer_ip,line.stamp)
                if (peer_ip == self._observed_module and
                        any(marker in line.text for marker in ('[NET]','[INIT]','MCAR_APP built','[PARAM]'))):
                    try:
                        self.link.received((peer_ip,int(line.source.rsplit(':',1)[1])),time.monotonic())
                    except ValueError:
                        pass
            self.log_file.write(datetime.fromtimestamp(line.stamp).isoformat(timespec='milliseconds') +
                                f' [{line.source}] {line.text}\n')
            self.logs.append(line)
            self.total_lines += 1
            self._log_dirty = True

    def guard(self, action):
        def callback(sender=None, app_data=None, user_data=None):
            try:
                action()
            except Exception as exc:
                if self.active_page == 'commands':
                    dpg.set_value('quick_status','操作失败：'+str(exc))
                if self.active_page == 'tuning':
                    dpg.set_value('tuning_status','操作失败：'+str(exc))
                self.status('操作失败：' + str(exc))
        return callback

    def _build(self):
        dpg.create_context()
        dpg.configure_app(manual_callback_management=True)
        font_path = Path('C:/Windows/Fonts/msyh.ttc')
        if font_path.exists():
            with dpg.font_registry():
                # Physical pixel size follows system DPI because the viewport is DPI aware.
                dpi = ctypes.windll.user32.GetDpiForSystem() if sys.platform == 'win32' else 96
                with dpg.font(str(font_path),round(20*max(1,dpi/96))) as font:
                    pass
                dpg.bind_font(font)
        with dpg.theme() as theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg,(35,35,35))
                dpg.add_theme_color(dpg.mvThemeCol_ChildBg,(35,35,35))
                dpg.add_theme_color(dpg.mvThemeCol_PopupBg,(40,40,40))
                dpg.add_theme_color(dpg.mvThemeCol_Border,(65,65,65))
                dpg.add_theme_color(dpg.mvThemeCol_Button,(48,49,53))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,(62,92,126))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,(37,108,194))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg,(48,49,53))
                dpg.add_theme_color(dpg.mvThemeCol_Header,(38,49,63))
                dpg.add_theme_color(dpg.mvThemeCol_CheckMark,(67,161,231))
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding,6,3)
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing,8,6)
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding,8,8)
        dpg.bind_theme(theme)
        self.nav_themes = {}
        for selected in (False,True):
            with dpg.theme() as nav_theme:
                with dpg.theme_component(dpg.mvButton):
                    dpg.add_theme_color(dpg.mvThemeCol_Button,(37,108,194) if selected else (38,46,58))
                    dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,(44,125,210))
            self.nav_themes[selected] = nav_theme
        with dpg.texture_registry(tag='texture_registry'):
            dpg.add_dynamic_texture(188,120,[.07,.1,.13,1]*22560,tag='camera_texture')
        with dpg.window(tag='main'):
            with dpg.group(horizontal=True):
                with dpg.child_window(tag='sidebar',width=260,height=-1):
                    dpg.add_spacer(height=12)
                    self.heading('WiFi-SPI / IMU')
                    dpg.add_text('UDP 实时上位机',color=(156,164,175))
                    dpg.add_spacer(height=24)
                    for page,label in (('params','01  调参'),('plot','02  实时绘图'),
                                       ('imu','03  IMU 姿态'),('camera','04  摄像头图像'),('commands','05  快捷命令'),
                                       ('tuning','06  实时调参')):
                        dpg.add_button(label=label,tag='nav_'+page,width=-1,height=65,
                                       callback=lambda s,a,p:self.show_page(p),user_data=page)
                        dpg.add_spacer(height=5)
                    dpg.add_spacer(height=15)
                    dpg.add_separator()
                    self.heading('连接状态')
                    dpg.add_text('未监听',tag='status',wrap=230)
                    dpg.add_text('接收帧率：0 fps',tag='sidebar_rate')
                    dpg.add_text('',tag='network_stats',wrap=230)
                    dpg.add_spacer(height=15)
                    dpg.add_text('切页不影响后台收包与记录。',wrap=230,color=(142,150,164))
                    if self.args.test_ports:
                        dpg.add_text('软件自测 / 非实车\n18081 / 18082 / 18083',wrap=230,color=(255,190,75))
                    dpg.add_spacer(height=15)
                    dpg.add_button(label='打开保存目录',width=-1,callback=self.guard(lambda:os.startfile(self.output)))
                with dpg.child_window(tag='content',width=-1,height=-1,border=False):
                    with dpg.child_window(tag='params_tab',width=-1,height=-1,border=False):
                        self.heading('调参 / 数据连接')
                        dpg.add_text('配置监听、发送指令与解析规则；调试文字集中显示在下方运行日志。',tag='params_intro',wrap=1300)
                        with dpg.group(horizontal=True):
                            with dpg.child_window(tag='udp_listen_panel',width=620,auto_resize_y=True,
                                                  no_scrollbar=True,no_scroll_with_mouse=True):
                                self.heading('UDP 监听 / 参数、日志与图像')
                                dpg.add_combo(['单端口 MUDP','旧版三端口'],tag='receive_mode',width=200,
                                              default_value='单端口 MUDP' if self.single_port else '旧版三端口',
                                              callback=self.guard(self.set_receive_mode))
                                with dpg.group(horizontal=True):
                                    dpg.add_input_text(default_value=self.settings.get('ip','0.0.0.0'),width=200,tag='bind_ip')
                                    dpg.add_text('本机监听 IP')
                                self.port_input('data','共用端口' if self.single_port else '参数端口')
                                self.port_input('log','日志端口')
                                with dpg.group(horizontal=True):
                                    dpg.add_button(label='开始监听（三路）',callback=self.guard(self.start))
                                    dpg.add_button(label='停止监听',callback=self.guard(self.stop))
                                dpg.add_text('单端口下三类数据共用8081，按包类型解析；0.0.0.0监听所有网卡。',tag='listen_note',color=(142,150,164),wrap=600)
                            with dpg.child_window(tag='udp_send_panel',width=-1,auto_resize_y=True,
                                                  no_scrollbar=True,no_scroll_with_mouse=True):
                                self.heading('UDP 指令发送')
                                dpg.add_input_text(label='模块 IP',tag='remote_ip',default_value=self.settings.get('remote_ip','192.168.50.41'),width=190)
                                dpg.add_input_int(label='端口',tag='remote_port',default_value=self.settings.get('remote_port',6666),width=140,min_value=1,max_value=65535,min_clamped=True,max_clamped=True)
                                dpg.add_input_text(tag='send_text',hint='文字或 AA 55 01',width=-1)
                                with dpg.group(horizontal=True):
                                    dpg.add_combo(['UTF-8 文字','十六进制'],default_value='UTF-8 文字',tag='send_mode',width=150)
                                    dpg.add_button(label='发送 UDP 指令',callback=self.guard(self.send))
                        with dpg.child_window(tag='image_connection',width=-1,auto_resize_y=True,
                                              no_scrollbar=True,no_scroll_with_mouse=True):
                            self.heading('图像传输 / 总钻风摄像头')
                            with dpg.group(horizontal=True):
                                self.port_input('camera','图像端口')
                                dpg.add_button(label='开始图像监听',tag='start_camera_button',callback=self.guard(lambda:self.start_channel('camera')))
                                dpg.add_button(label='停止图像监听',tag='stop_camera_button',callback=self.guard(lambda:self.stop_channel('camera')))
                            dpg.add_text('单端口模式：图像共用上方端口；图像启停按钮控制三类接收。旧版模式保留独立图像端口。',tag='image_protocol_note',color=(156,164,175),wrap=1200)
                            dpg.add_text('尚未接收图像',tag='image_connection_status',color=(142,150,164),wrap=1200)
                        with dpg.child_window(tag='rules_panel',height=215,width=-1):
                            with dpg.group(horizontal=True):
                                self.heading('二进制报文解析规则')
                                dpg.add_button(label='原版三角度',callback=self.guard(lambda:self.load_rules(DEFAULT_RULES)))
                                dpg.add_button(label='综合验证（IMU + 编码器）',callback=self.guard(lambda:self.load_rules(MONITOR_RULES)))
                                dpg.add_button(label='四轮速度环',callback=self.guard(lambda:self.load_rules(SPEED_RULES)))
                                dpg.add_button(label='视觉循迹',callback=self.guard(lambda:self.load_rules(TRACK_RULES)))
                                dpg.add_button(label='+ 新增字段',callback=self.guard(lambda:self.add_rule(FieldRule('field_'+str(len(self.rule_rows)),'float32',0))))
                            with dpg.table(tag='rules_table',header_row=True,resizable=True,borders_innerH=True,
                                           scrollY=True,height=-1,policy=dpg.mvTable_SizingStretchProp):
                                for label in ('字段名','数据类型','起始字节','字节序','操作'):
                                    dpg.add_table_column(label=label)
                        with dpg.child_window(tag='save_panel',height=95,width=-1):
                            self.heading('数据保存')
                            with dpg.group(horizontal=True):
                                dpg.add_input_text(tag='csv_path',hint='留空自动命名',width=650)
                                dpg.add_text('CSV 文件路径')
                                dpg.add_button(label='保存全部解析数据',callback=self.guard(self.save_csv))
                        with dpg.child_window(tag='running_logs',height=285,width=-1):
                            with dpg.group(horizontal=True):
                                self.heading('运行日志')
                                dpg.add_button(label='清空日志',callback=self.guard(self.clear_logs))
                                dpg.add_button(label='保存日志',callback=self.guard(self.save_log))
                                dpg.add_text('绿：接收',color=(105,205,145))
                                dpg.add_text('蓝：发送 / 状态',color=(88,170,238))
                                dpg.add_text('红：错误',color=(255,120,120))
                            with dpg.group(horizontal=True):
                                dpg.add_checkbox(label='时间戳',tag='log_timestamp',default_value=True,callback=lambda:self._dirty_log())
                                dpg.add_checkbox(label='自动滚动',tag='log_scroll',default_value=True)
                                dpg.add_checkbox(label='暂停显示',tag='log_pause',default_value=False,callback=lambda:self._dirty_log())
                                dpg.add_combo(['utf-8','gbk'],default_value='utf-8',tag='log_encoding',width=110,callback=self.guard(self.change_encoding))
                                dpg.add_input_text(tag='log_filter',hint='筛选文字',width=220,callback=lambda:self._dirty_log())
                            dpg.add_text('',tag='log_stats',color=(142,150,164))
                            with dpg.child_window(tag='log_panel',height=-1,width=-1,horizontal_scrollbar=True,border=False):
                                pass
                    with dpg.child_window(tag='plot_tab',width=-1,height=-1,border=False,show=False):
                        self.heading('实时绘图')
                        dpg.add_text('选中任意解析字段即可绘制；图像刷新与 UDP 接收分离，最多 15 次/秒。')
                        with dpg.group(horizontal=True):
                            with dpg.child_window(tag='plot_controls',width=290,height=760):
                                self.heading('实时解析值')
                                with dpg.table(tag='values_table',header_row=True,borders_innerV=True):
                                    dpg.add_table_column(label='字段')
                                    dpg.add_table_column(label='实时值')
                                dpg.add_separator()
                                self.heading('曲线字段')
                                with dpg.group(tag='plot_choices'):
                                    pass
                                dpg.add_spacer(height=20)
                                dpg.add_input_int(label='最大采样点',tag='max_samples',default_value=1200,width=130,min_value=50,max_value=100000,min_clamped=True,max_clamped=True,
                                                  callback=self.guard(self.resize_history))
                                dpg.add_checkbox(label='自动调整坐标轴',tag='fit_plot',default_value=True)
                                dpg.add_text('每字段保留最近采样点。',wrap=260,color=(142,150,164))
                            with dpg.plot(label='',height=760,width=-1,tag='plot'):
                                dpg.add_plot_legend()
                                dpg.add_plot_axis(dpg.mvXAxis,label='相对时间 / s',tag='plot_x')
                                dpg.add_plot_axis(dpg.mvYAxis,label='数值',tag='plot_y')
                    with dpg.child_window(tag='imu_tab',width=-1,height=-1,border=False,show=False):
                        self.heading('IMU 三维姿态')
                        dpg.add_text('根据 roll / pitch / yaw 欧拉角旋转机体模型；角度单位为度。X 向前，Y 向左，Z 向上。')
                        with dpg.group(horizontal=True):
                            for name,label in (('roll','ROLL / 横滚'),('pitch','PITCH / 俯仰'),('yaw','YAW / 航向')):
                                with dpg.child_window(tag='angle_card_'+name,width=280,height=95):
                                    dpg.add_text(label,color=(156,164,175))
                                    dpg.add_text('—',tag='angle_'+name,color=(67,161,231))
                        dpg.add_text('等待实测角度；下方显示零姿态模型预览。',tag='angles')
                        with dpg.group(horizontal=True):
                            dpg.add_text('模型视角',color=(142,150,164))
                            for label,yaw,elev in (('等轴',-55,30),('正视',0,0),('俯视',0,90)):
                                dpg.add_button(label=label,callback=lambda s,a,v:self.set_model_view(*v),user_data=(yaw,elev))
                            dpg.add_slider_float(label='缩放',tag='view_zoom',min_value=.4,max_value=2,default_value=1,width=190)
                            dpg.add_combo(['精细','简洁'],default_value='精细',tag='model_quality',width=110)
                        dpg.add_slider_float(tag='view_yaw',min_value=-180,max_value=180,default_value=-55,show=False)
                        dpg.add_slider_float(tag='view_elev',min_value=0,max_value=90,default_value=30,show=False)
                        dpg.add_drawlist(width=1100,height=630,tag='pose')
                    with dpg.child_window(tag='camera_tab',width=-1,height=-1,border=False,show=False):
                        self.heading('摄像头图像 / 总钻风')
                        dpg.add_text('原始灰度图；左右边界蓝、中线黄、十字补线紫；候选拐点橙、确认/保持拐点红。绿线标出本帧实际取值范围。',tag='camera_intro',wrap=1250)
                        with dpg.group(horizontal=True,tag='camera_toolbar'):
                            with dpg.group(horizontal=True):
                                dpg.add_checkbox(label='暂停显示',tag='camera_pause',default_value=False)
                                dpg.add_checkbox(label='适应窗口',tag='camera_fit',default_value=True,callback=self.guard(self.change_camera_view))
                                dpg.add_slider_int(label='手动倍率',tag='camera_zoom',default_value=4,min_value=1,max_value=8,width=180,enabled=False)
                            with dpg.group(horizontal=True):
                                dpg.add_button(label='保存当前画面',callback=self.guard(self.save_frame))
                                dpg.add_button(label='图像传输设置',callback=lambda:self.show_page('params'))
                        dpg.add_text('等待完整图像',tag='camera_stats',wrap=1250)
                        dpg.add_text(curve_status('unavailable').text,tag='camera_curve_status',
                                     color=CURVE_STATUS_COLORS['unavailable'],wrap=1250)
                        dpg.add_text('',tag='camera_display_info',color=(142,150,164),wrap=1250)
                        dpg.add_text('边界/中线/补线及拐点隐藏顶部和底部20行；取值绿线贯穿图像宽度。',tag='camera_annotation_info',color=(142,150,164),wrap=1250)
                        with dpg.child_window(tag='camera_canvas',height=-1,width=-1,horizontal_scrollbar=True,
                                              no_scrollbar=True,no_scroll_with_mouse=True):
                            dpg.add_image('camera_texture',tag='camera_image',width=752,height=480)
                    with dpg.child_window(tag='commands_tab',width=-1,height=-1,border=False,show=False):
                        self.heading('快捷命令 / 自定义条目')
                        dpg.add_text('按单片机协议填写命令；备注只供阅读。点击发送一次发送一个 UDP 包，不自动重发。',tag='quick_intro',wrap=1300)
                        with dpg.child_window(width=-1,auto_resize_y=True,no_scrollbar=True):
                            dpg.add_checkbox(label='沿用 01 调参的发送地址',tag='quick_inherit',default_value=self.quick_config['inherit_target'])
                            with dpg.group(horizontal=True):
                                dpg.add_input_text(label='模块 IP',tag='quick_ip',default_value=self.quick_config['target_ip'],width=190)
                                dpg.add_input_int(label='命令端口',tag='quick_port',default_value=self.quick_config['target_port'],width=150,min_value=1,max_value=65535,min_clamped=True,max_clamped=True)
                            dpg.add_text('',tag='quick_target',wrap=1300,color=(142,150,164))
                        with dpg.group(horizontal=True):
                            dpg.add_button(label='+ 新增条目',callback=self.guard(lambda:self.edit_quick_list('add')))
                            dpg.add_button(label='保存列表',callback=self.guard(self.save_quick_commands))
                            dpg.add_text('编辑后自动保存；发送结果不表示单片机已经执行。',color=(142,150,164))
                        with dpg.group(horizontal=True):
                            dpg.add_input_text(tag='quick_file_path',hint='导入/导出 JSON 路径；留空导出自动命名',width=650)
                            dpg.add_button(label='导入 JSON',callback=self.guard(self.import_quick_commands))
                            dpg.add_button(label='导出 JSON',callback=self.guard(self.export_quick_commands))
                        dpg.add_text('内容支持多行。文本可附加 LF/CRLF；HEX 和 Binary 发送原始字节。',tag='quick_help',wrap=1300)
                        dpg.add_text('Binary 示例：[{"type":"uint8","value":1},{"type":"float32","value":0.5}]',tag='quick_binary_help',wrap=1300,color=(142,150,164))
                        dpg.add_text(self.quick_load_error or '尚未发送命令。',tag='quick_status',wrap=1300)
                        with dpg.table(tag='quick_table',header_row=True,resizable=True,borders_innerH=True,borders_innerV=True,
                                       scrollY=True,height=-1,policy=dpg.mvTable_SizingStretchProp):
                            for label,weight in (('名称',1),('格式',1.3),('命令内容',2.5),('备注 / 命令意义',2),('快捷发送',1.3)):
                                dpg.add_table_column(label=label,init_width_or_weight=weight)
                        self.rebuild_quick_rows()
                    with dpg.child_window(tag='tuning_tab',width=-1,height=-1,border=False,show=False):
                        self.heading('实时调参 / 车辆运行参数')
                        dpg.add_text('沿用 01 的模块 IP 与命令端口；回包走当前模式的日志通道。编辑后逐项应用，只有车辆回包才显示已确认。',tag='tuning_intro',wrap=1300)
                        dpg.add_text('',tag='tuning_target',wrap=1300,color=(142,150,164))
                        with dpg.group(horizontal=True):
                            dpg.add_button(label='读取车辆全部参数',callback=self.guard(self.read_tuning))
                            dpg.add_button(label='确认值填入编辑栏',callback=self.guard(self.copy_tuning_actual))
                            dpg.add_button(label='本地默认值',callback=self.guard(self.default_tuning))
                        with dpg.group(horizontal=True):
                            dpg.add_input_text(tag='tuning_file_path',hint='导入/导出 JSON 路径；留空导出自动命名',width=650)
                            dpg.add_button(label='导入方案（仅编辑）',callback=self.guard(self.import_tuning))
                            dpg.add_button(label='保存本地方案',callback=self.guard(self.save_tuning))
                            dpg.add_button(label='导出方案',callback=self.guard(self.export_tuning))
                        dpg.add_text('参数复位恢复默认值。起步时间、弯道开关仅停车可改；紧弯速度≤普通弯速度，出弯需求<普通弯需求<紧弯需求。速度0仍可转向，停车按K2。',tag='tuning_note',wrap=1300,color=(255,190,75))
                        dpg.add_text(self.tuning_load_error or self.tuning.message,tag='tuning_status',wrap=1300)
                        with dpg.table(tag='tuning_table',header_row=True,resizable=True,borders_innerH=True,borders_innerV=True,
                                       scrollY=True,height=-1,policy=dpg.mvTable_SizingStretchProp):
                            for label,weight in (('参数 / 键名',1.4),('编辑值',.9),('车辆确认值',.9),('范围',.8),('作用 / 调整方向',3),('应用 / 状态',1.7)):
                                dpg.add_table_column(label=label,init_width_or_weight=weight)
                            for parameter in EDITABLE_PARAMETERS:
                                key = parameter.key
                                with dpg.table_row():
                                    dpg.add_text(parameter.label+'\n'+key,tag='tune_label_'+key,wrap=180)
                                    if parameter.integer:
                                        dpg.add_input_int(tag='tune_edit_'+key,default_value=int(self.tuning_edit[key]),width=-1,step=parameter.step)
                                    else:
                                        dpg.add_input_double(tag='tune_edit_'+key,default_value=self.tuning_edit[key],width=-1,format='%.8g',step=0)
                                    dpg.add_text('未确认',tag='tune_actual_'+key)
                                    dpg.add_text(f'{parameter.minimum:g}～{parameter.maximum:g}')
                                    dpg.add_text(parameter.help,tag='tune_help_'+key,wrap=400)
                                    with dpg.group():
                                        dpg.add_button(label='应用此项',tag='tune_apply_'+key,width=-1,callback=self.guard(lambda k=key:self.apply_tuning(k)))
                                        dpg.add_text('未读取',tag='tune_state_'+key,wrap=210)
        self.load_rules(startup_rules(self.settings))
        dpg.create_viewport(title='Mcar WiFi Monitor - SingleUDP',width=1800,height=1120,min_width=1250,min_height=820)
        dpg.setup_dearpygui()
        dpg.set_primary_window('main',True)
        self.show_page('params')
        # A model is available before the first UDP packet, including offline use.
        draw_attitude('pose',(0,0,0))

    @staticmethod
    def heading(text):
        dpg.add_text(text,color=(67,161,231))

    def port_input(self, name, label):
        i = list(self.receivers).index(name)
        try:
            port = 18081+i if self.args.test_ports else int(self.settings.get(name+'_port',8081+i))
            if not 1 <= port <= 65535:
                raise ValueError()
        except (ValueError,TypeError):
            port = 8081+i
        dpg.add_input_int(label=label,tag=name+'_port',default_value=port,width=140,
                          min_value=1,max_value=65535,min_clamped=True,max_clamped=True,
                          show=not self.single_port or name=='data')

    def show_page(self, page):
        page = 'params' if page == 'log' else page
        self.active_page = page
        for name in ('params','plot','imu','camera','commands','tuning'):
            dpg.configure_item(name+'_tab',show=name == page)
            dpg.bind_item_theme('nav_'+name,self.nav_themes[name == page])
        self._layout_size = None

    def set_model_view(self, yaw, elev):
        dpg.set_value('view_yaw',yaw)
        dpg.set_value('view_elev',elev)

    def resize_history(self):
        self.max_samples = max(50,min(100000,dpg.get_value('max_samples')))
        for name,samples in self.history.items():
            self.history[name] = deque(samples,maxlen=self.max_samples)

    def resize_layout(self):
        width,height = dpg.get_item_rect_size('content')
        if width < 100 or height < 100:
            return
        size = (round(width),round(height),self.active_page)
        if size == self._layout_size:
            return
        self._layout_size = size
        content_width = max(880,int(width)-32)
        dpg.configure_item('udp_listen_panel',width=(content_width-8)//2)
        dpg.configure_item('listen_note',wrap=(content_width-8)//2-20)
        dpg.configure_item('csv_path',width=max(230,content_width-450))
        dpg.configure_item('camera_toolbar',horizontal=content_width >= 1200)
        # Smaller windows scroll vertically rather than hiding settings or logs.
        dpg.configure_item('running_logs',height=max(265,int(height)-760))
        for tag in ('params_intro','camera_intro','camera_stats','camera_curve_status','camera_display_info','camera_annotation_info','image_protocol_note','image_connection_status'):
            dpg.configure_item(tag,wrap=content_width)
        for tag in ('quick_intro','quick_target','quick_help','quick_binary_help','quick_status'):
            dpg.configure_item(tag,wrap=content_width)
        dpg.configure_item('quick_file_path',width=max(220,content_width-400))
        for tag in ('tuning_intro','tuning_target','tuning_note','tuning_status'):
            dpg.configure_item(tag,wrap=content_width)
        dpg.configure_item('tuning_file_path',width=max(220,content_width-570))
        for parameter in EDITABLE_PARAMETERS:
            dpg.configure_item('tune_label_'+parameter.key,wrap=max(100,int(content_width*1.4/8.7)-24))
            dpg.configure_item('tune_help_'+parameter.key,wrap=max(140,int(content_width*3/8.7)-24))
            dpg.configure_item('tune_state_'+parameter.key,wrap=max(110,int(content_width*1.7/8.7)-24))
        dpg.configure_item('plot_controls',height=max(500,int(height)-95))
        dpg.configure_item('plot',height=max(500,int(height)-95))
        dpg.configure_item('pose',width=content_width,height=max(350,int(height)-250))

    def rebuild_quick_rows(self):
        dpg.delete_item('quick_table',children_only=True,slot=1)
        self.quick_rows = []
        for item in self.quick_config['commands']:
            cid = item['id']
            prefix = 'qc_'+cid+'_'
            with dpg.table_row(parent='quick_table'):
                dpg.add_input_text(tag=prefix+'name',default_value=item['name'],width=-1)
                with dpg.group():
                    dpg.add_combo(FORMATS,tag=prefix+'format',default_value=item['format'],width=-1)
                    dpg.add_text('行尾')
                    dpg.add_combo(ENDINGS,tag=prefix+'ending',default_value=item['ending'],width=-1)
                    dpg.add_text('字节序')
                    dpg.add_combo(['little','big'],tag=prefix+'byte_order',default_value=item['byte_order'],width=-1)
                dpg.add_input_text(tag=prefix+'content',default_value=item['content'],multiline=True,height=120,width=-1)
                dpg.add_input_text(tag=prefix+'note',default_value=item['note'],multiline=True,height=120,width=-1)
                with dpg.group():
                    dpg.add_button(label='发送',tag=prefix+'send',width=-1,callback=self.guard(lambda c=cid:self.send_quick_command(c)))
                    dpg.add_text(self.quick_results.get(cid,'未发送'),tag=prefix+'result',wrap=160)
                    with dpg.group(horizontal=True):
                        dpg.add_button(label='↑',callback=self.guard(lambda c=cid:self.edit_quick_list('up',c)))
                        dpg.add_button(label='↓',callback=self.guard(lambda c=cid:self.edit_quick_list('down',c)))
                    dpg.add_button(label='复制',width=-1,callback=self.guard(lambda c=cid:self.edit_quick_list('copy',c)))
                    dpg.add_button(label='删除',width=-1,callback=self.guard(lambda c=cid:self.edit_quick_list('delete',c)))
            self.quick_rows.append(cid)

    def observe_module(self, ip):
        """Adopt only the first recognized module; later edits stay user controlled."""
        if self._observed_module is None:
            if self.tuning.peer is not None and self.tuning.peer != ip:
                return
            try:
                TuningClient.target(ip,6666)
                dpg.set_value('remote_ip',ip)
            except (ValueError,KeyError):
                return
            self._observed_module = ip
            self.tuning.peer = ip
        self.tuning.observe(ip)

    def tuning_target(self):
        if not self.receivers['log'].running:
            raise ValueError('请先在 01 启动日志监听，才能接收车辆确认')
        return TuningClient.target(dpg.get_value('remote_ip'),dpg.get_value('remote_port'))

    def apply_tuning(self, key):
        if key not in EDITABLE_BY_KEY:
            raise ValueError('此参数不提供上位机调节')
        target = self.tuning_target()
        value = dpg.get_value('tune_edit_'+key)
        rid = self.tuning.apply(key,value,*target)
        self.status(f'实时调参 SET id={rid} {key}={value:g}；等待车辆确认。')
        self.refresh_tuning()

    def read_tuning(self):
        rid = self.tuning.read_all(*self.tuning_target())
        self.status(f'实时调参 GET id={rid}；等待车辆返回当前参数。')
        self.refresh_tuning()

    def copy_tuning_actual(self):
        if not self.tuning.actual:
            raise ValueError('尚未读取到车辆确认参数')
        for p in EDITABLE_PARAMETERS:
            if p.key in self.tuning.actual:
                dpg.set_value('tune_edit_'+p.key,self.tuning.actual[p.key])
        dpg.set_value('tuning_status','车辆已确认值已填入编辑栏；没有发送修改。')

    def default_tuning(self):
        for p in EDITABLE_PARAMETERS:
            dpg.set_value('tune_edit_'+p.key,p.default)
        dpg.set_value('tuning_status','本地默认值已填入编辑栏；没有发送修改。')

    def collect_tuning(self):
        return {p.key:p.validate(dpg.get_value('tune_edit_'+p.key)) for p in EDITABLE_PARAMETERS}

    def save_tuning(self):
        self.tuning_edit = self.collect_tuning()
        save_scheme(self.tuning_path,self.tuning_edit)
        self.tuning_load_error = ''
        dpg.set_value('tuning_status','本地方案已保存；没有发送修改。')

    def export_tuning(self):
        requested = dpg.get_value('tuning_file_path').strip()
        target = Path(requested) if requested else self.output/f'调参方案_{datetime.now():%Y%m%d_%H%M%S_%f}.json'
        save_scheme(target,self.collect_tuning())
        dpg.set_value('tuning_file_path',str(target))
        dpg.set_value('tuning_status','本地方案已导出：'+str(target)+'；没有发送修改。')

    def import_tuning(self):
        requested = dpg.get_value('tuning_file_path').strip()
        if not requested:
            raise ValueError('请填写要导入的 JSON 文件路径')
        values = load_scheme(requested)
        for key,value in values.items():
            dpg.set_value('tune_edit_'+key,value)
        dpg.set_value('tuning_status','方案已导入编辑栏；逐项应用后等待车辆确认。')

    def refresh_tuning(self):
        target = (dpg.get_value('remote_ip').strip(),dpg.get_value('remote_port'))
        if self.tuning.peer is not None and (target[0] != self.tuning.peer or any(r.target != target for r in self.tuning.pending.values())):
            self.tuning.invalidate('发送对象已切换')
            self.tuning.peer = target[0]
        self.tuning.tick()
        signature = (target,tuple(self.tuning.actual.items()),tuple(self.tuning.states.items()),self.tuning.message)
        if signature == self._tuning_signature:
            return
        self._tuning_signature = signature
        dpg.set_value('tuning_target',f'车辆目标：{target[0]}:{target[1]}；接收端口 {dpg.get_value("log_port")}；仅应用到本次运行。')
        dpg.set_value('tuning_status',self.tuning.message)
        for p in EDITABLE_PARAMETERS:
            value = self.tuning.actual.get(p.key)
            dpg.set_value('tune_actual_'+p.key,f'{value:g}' if value is not None else '未确认')
            dpg.set_value('tune_state_'+p.key,self.tuning.states[p.key])

    def sync_quick_commands(self):
        inherit = dpg.get_value('quick_inherit')
        dpg.configure_item('quick_ip',enabled=not inherit)
        dpg.configure_item('quick_port',enabled=not inherit)
        if inherit:
            dpg.set_value('quick_ip',dpg.get_value('remote_ip'))
            dpg.set_value('quick_port',dpg.get_value('remote_port'))
        ip,port = dpg.get_value('quick_ip'),dpg.get_value('quick_port')
        dpg.set_value('quick_target',f'发送目标：{ip or "请填写模块 IP"}:{port}；这是模块命令端口，非电脑数据监听端口。')
        commands = []
        for cid in self.quick_rows:
            prefix = 'qc_'+cid+'_'
            item = dict(id=cid,**{field:dpg.get_value(prefix+field) for field in ('name','format','content','note','ending','byte_order')})
            text_mode = item['format'] not in ('HEX','Binary')
            if not text_mode:
                item['ending'] = 'None'
                dpg.set_value(prefix+'ending','None')
            dpg.configure_item(prefix+'ending',enabled=text_mode)
            dpg.configure_item(prefix+'byte_order',enabled=item['format']=='Binary')
            try:
                button_width = dpg.get_item_rect_size(prefix+'send')[0]
                if button_width>0:
                    dpg.configure_item(prefix+'result',wrap=max(60,int(button_width)))
            except KeyError:
                pass  # The first, unrendered frame does not yet have a rectangle.
            commands.append(item)
        value = dict(version=1,inherit_target=inherit,target_ip=ip,target_port=port,commands=commands)
        if value != self.quick_config:
            self.quick_config = value
            self._quick_save_due = time.monotonic()+.6

    def save_quick_commands(self):
        self.sync_quick_commands()
        save_config(self.quick_path,self.quick_config)
        self.quick_load_error = ''
        self._quick_save_due = 0
        dpg.set_value('quick_status','命令列表已保存：'+str(self.quick_path))

    def edit_quick_list(self, action, cid=None):
        self.sync_quick_commands()
        items = self.quick_config['commands']
        if action == 'add':
            items.append(new_command('命令 '+str(len(items)+1)))
        else:
            index = next(i for i,item in enumerate(items) if item['id']==cid)
            if action == 'delete':
                items.pop(index)
                self.quick_results.pop(cid,None)
            elif action == 'copy':
                copied = dict(items[index],id=uuid.uuid4().hex,name=items[index]['name']+' 副本')
                items.insert(index+1,copied)
            elif action in ('up','down'):
                other = index+(-1 if action=='up' else 1)
                if 0 <= other < len(items):
                    items[index],items[other] = items[other],items[index]
        self.rebuild_quick_rows()
        self._quick_save_due = time.monotonic()+.6

    def send_quick_command(self, cid):
        self.sync_quick_commands()
        item = next(item for item in self.quick_config['commands'] if item['id']==cid)
        prefix = 'qc_'+cid+'_'
        try:
            payload = encode_command(item)
            target = validate_target(self.quick_config['target_ip'],self.quick_config['target_port'])
            sent = send_udp(payload,*target)
        except Exception as exc:
            self.quick_results[cid] = '发送失败：'+str(exc)
            dpg.set_value(prefix+'result',self.quick_results[cid])
            dpg.set_value('quick_status',self.quick_results[cid])
            raise
        self.quick_results[cid] = f'已发送 {sent}B\n未确认执行'
        dpg.set_value(prefix+'result',self.quick_results[cid])
        preview = payload[:48].hex(' ').upper()+(' …' if len(payload)>48 else '')
        message = f'快捷命令 [{item["name"] or "未命名"}] → {target[0]}:{target[1]}，{sent} 字节；未确认执行。报文：{preview}'
        dpg.set_value('quick_status',message)
        self.status(message)

    def export_quick_commands(self):
        self.save_quick_commands()
        requested = dpg.get_value('quick_file_path').strip()
        target = Path(requested).expanduser() if requested else self.output/f'快捷命令_{datetime.now():%Y%m%d_%H%M%S_%f}.json'
        if target.suffix.lower() != '.json':
            target = target.with_suffix('.json')
        save_config(target,self.quick_config)
        dpg.set_value('quick_file_path',str(target))
        dpg.set_value('quick_status','命令列表已导出：'+str(target))

    def import_quick_commands(self):
        requested = dpg.get_value('quick_file_path').strip()
        if not requested:
            raise ValueError('请先填写要导入的 JSON 文件路径')
        value = load_config(Path(requested).expanduser())
        # Keep the previous editable list recoverable when replacing it by import.
        self.sync_quick_commands()
        save_config(STORE/f'quick_commands_before_import_{datetime.now():%Y%m%d_%H%M%S_%f}.json',self.quick_config)
        self.quick_config = value
        self.quick_results.clear()
        dpg.set_value('quick_inherit',value['inherit_target'])
        dpg.set_value('quick_ip',value['target_ip'])
        dpg.set_value('quick_port',value['target_port'])
        self.rebuild_quick_rows()
        self.save_quick_commands()
        dpg.set_value('quick_status',f'已导入 {len(value["commands"])} 个条目；原列表已备份。')

    def change_camera_view(self):
        fit = dpg.get_value('camera_fit')
        dpg.configure_item('camera_zoom',enabled=not fit)
        dpg.configure_item('camera_canvas',no_scrollbar=fit,no_scroll_with_mouse=fit)
        if fit:
            dpg.set_x_scroll('camera_canvas',0)
            dpg.set_y_scroll('camera_canvas',0)
        self._camera_display_signature = None

    def resize_camera_display(self):
        canvas_width,canvas_height = dpg.get_item_rect_size('camera_canvas')
        if canvas_width < 32 or canvas_height < 32:
            return
        displayed = self.shown_frame
        frame_width,frame_height = (displayed.width,displayed.height) if displayed else self._last_texture_size
        fit,zoom = dpg.get_value('camera_fit'),dpg.get_value('camera_zoom')
        signature = (frame_width,frame_height,canvas_width,canvas_height,fit,zoom)
        if signature == self._camera_display_signature:
            return
        # Child borders use eight pixels of window padding on each side.
        width,height,x,y = camera_display_rect(frame_width,frame_height,canvas_width-16,canvas_height-16,fit,zoom)
        dpg.configure_item('camera_image',width=max(1,round(width)),height=max(1,round(height)),pos=(round(x+8),round(y+8)))
        mode = '适应窗口' if fit else '手动倍率'
        prefix = '原图' if displayed else '等待图像 / 预留画面'
        dpg.set_value('camera_display_info',f'{mode}：{prefix} {frame_width}×{frame_height} → 显示 {round(width)}×{round(height)}（约 {width/frame_width:.2f} 倍）')
        self._camera_display_signature = signature

    def _dirty_log(self):
        self._log_dirty = True

    def add_rule(self, rule):
        row = dpg.generate_uuid()
        with dpg.table_row(parent='rules_table',tag=row):
            dpg.add_input_text(default_value=rule.name,tag=f'n{row}',width=-1)
            dpg.add_combo(list(TYPE_FORMATS),default_value=rule.data_type,tag=f't{row}',width=-1)
            dpg.add_input_int(default_value=rule.offset,tag=f'o{row}',width=-1,min_value=0,min_clamped=True)
            dpg.add_combo(['little','big'],default_value=rule.byte_order,tag=f'b{row}',width=-1)
            dpg.add_button(label='删除',callback=self.guard(lambda:self.delete_rule(row)))
        self.rule_rows.append(row)

    def delete_rule(self, row):
        self.rule_rows.remove(row)
        dpg.delete_item(row)

    def load_rules(self, rules):
        for row in list(self.rule_rows):
            self.delete_rule(row)
        for rule in rules:
            self.add_rule(rule)

    def collect_rules(self):
        rules, used = [], set()
        for row in self.rule_rows:
            name = dpg.get_value(f'n{row}').strip()
            if name and name not in used:
                rules.append(FieldRule(name,dpg.get_value(f't{row}'),dpg.get_value(f'o{row}'),dpg.get_value(f'b{row}')))
                used.add(name)
        if rules != self.rules:
            self.rules = rules
            self.latest.clear()
            self.history.clear()
            self.history_origin = None
            dpg.delete_item('plot_choices',children_only=True)
            dpg.delete_item('values_table',children_only=True,slot=1)
            for name in self._plot_names:
                dpg.delete_item('series_'+name)
            self._plot_names = []
            for rule in rules:
                self.history[rule.name] = deque(maxlen=self.max_samples)
                dpg.add_checkbox(label=rule.name,tag='check_'+rule.name,default_value=rule.name in ('roll','pitch','yaw'),parent='plot_choices')
                dpg.add_line_series([],[],label=rule.name,tag='series_'+rule.name,parent='plot_y')
                with dpg.table_row(parent='values_table'):
                    dpg.add_text(rule.name)
                    dpg.add_text('—',tag='value_'+rule.name)
                self._plot_names.append(rule.name)
            # Recorded independently of the selected rules, so history stays
            # appendable even when an old saved rule list omits curve_level.
            self.history.setdefault('curve_level',deque(maxlen=self.max_samples))
        return rules

    def start(self):
        if self.single_port:
            self._start_shared()
            self.status(f'单端口接收已启动：{self.multiplex.address[1]}，参数 / 日志 / 图像按类型分流')
            return
        ports = [dpg.get_value(n+'_port') for n in self.receivers]
        if len(set(ports)) != 3:
            raise ValueError('数据、日志、图像端口不能重复')
        started = []
        try:
            for name in self.receivers:
                if self.receivers[name].running:
                    expected = (dpg.get_value('bind_ip').strip(),dpg.get_value(name+'_port'))
                    if self.bindings.get(name) != expected:
                        raise ValueError('更改监听地址或端口前，请先停止对应接收')
                else:
                    self._start_receiver(name)
                    started.append(name)
        except Exception:
            # A failed combined start must not interrupt an existing independent receiver.
            for name in started:
                self.receivers[name].stop()
                self.bindings.pop(name,None)
            raise
        self.status(f'接收已启动：数据 {ports[0]} / 日志 {ports[1]} / 图像 {ports[2]}')

    def _start_receiver(self, name):
        receiver = self.receivers[name]
        self.reset_curve_status()
        address = (dpg.get_value('bind_ip').strip(),dpg.get_value(name+'_port'))
        if any(other != name and self.receivers[other].running and bound[1] == address[1]
               for other,bound in self.bindings.items()):
            raise ValueError('该端口已被另一路接收使用')
        list(receiver.drain(4096))
        if name == 'log':
            self.append_logs(self.decoder.flush())
            self.decoder = LineDecoder(dpg.get_value('log_encoding'))
        elif name == 'camera':
            self.camera = CameraAssembler()
        receiver.start(*address)
        self.bindings[name] = address

    def _start_shared(self):
        address = (dpg.get_value('bind_ip').strip(),dpg.get_value('data_port'))
        if self.multiplex.receiver.running:
            if self.multiplex.address != address:
                raise ValueError('更改监听地址或端口前，请先停止监听')
            return
        self.receive()
        self.append_logs(self.decoder.flush())
        self.decoder = LineDecoder(dpg.get_value('log_encoding'))
        self.camera = CameraAssembler()
        self.reset_curve_status()
        self.multiplex.start(*address)
        self.bindings = {name:address for name in self.receivers}

    def set_receive_mode(self):
        requested = dpg.get_value('receive_mode')=='单端口 MUDP'
        if requested == self.single_port:
            return
        if any(r.running for r in self.receivers.values()):
            dpg.set_value('receive_mode','单端口 MUDP' if self.single_port else '旧版三端口')
            raise ValueError('先停止监听，再切换接收模式')
        self.receive()
        self.reset_curve_status()
        self.append_logs(self.decoder.flush())
        self.single_port = requested
        self.multiplex = MultiplexReceiver() if requested else None
        self.receivers = self.multiplex.channels if self.multiplex else {n:UdpReceiver() for n in ('data','log','camera')}
        self.bindings.clear()
        for name in ('log','camera'):
            dpg.configure_item(name+'_port',show=not requested)
        dpg.configure_item('data_port',label='共用端口' if requested else '参数端口')
        self.tuning.invalidate('接收模式已切换')
        self.status('接收模式已切换；请核对车端协议后开始监听')

    def start_channel(self, name):
        if self.single_port:
            self.start()
            return
        if self.receivers[name].running:
            self.status('图像已经在接收；更改端口前请先停止图像监听。')
            return
        self._start_receiver(name)
        self.status(f'图像接收已启动：端口 {self.bindings[name][1]}')

    def stop_channel(self, name):
        if self.single_port:
            self.stop()
            return
        self.receivers[name].stop()
        self.bindings.pop(name,None)
        self.receive()
        while not self.receivers[name].queue.empty():
            self.receive()
        self.status('图像接收已停止；参数和日志继续接收。')

    def stop(self):
        for receiver in self.receivers.values():
            receiver.stop()
        self.bindings.clear()
        self.receive() # 单端口还可能有尚未分流的原始UDP包，停止后同样保存
        # Process the last queued records before flushing partial text.
        while any(not receiver.queue.empty() for receiver in self.receivers.values()):
            self.receive()
        self.reset_curve_status()
        self.append_logs(self.decoder.flush())
        self.tuning.invalidate('上位机已停止监听')
        self.status('接收已停止。已收到的数据和日志仍保存在本地。')
        self.log_file.flush()
        self.data_file.flush()

    def change_encoding(self):
        self.append_logs(self.decoder.flush())
        self.decoder = LineDecoder(dpg.get_value('log_encoding'))
        self.status('日志解码已切换为 '+dpg.get_value('log_encoding'))

    def clear_logs(self):
        self.logs.clear()
        dpg.delete_item('log_panel',children_only=True)
        self._log_widgets.clear()
        self._pending_log_scroll = False
        self._log_dirty = True
        self._visible_signature = None

    def save_log(self):
        self.log_file.flush()
        target = self.output / ('导出日志_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.txt')
        shutil.copyfile(self.log_path,target)
        self.status('日志已保存：'+str(target))

    def save_csv(self):
        self.data_file.flush()
        names = []
        with self.data_path.open(encoding='utf-8') as stream:
            for line in stream:
                for key in json.loads(line)['values']:
                    if key not in names:
                        names.append(key)
        if not names:
            raise ValueError('尚未收到可解析的数据')
        requested = dpg.get_value('csv_path').strip()
        target = Path(requested).expanduser() if requested else self.output / ('数据_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.csv')
        if target.suffix.lower() != '.csv':
            target = target.with_suffix('.csv')
        target.parent.mkdir(parents=True,exist_ok=True)
        with target.open('w',encoding='utf-8-sig',newline='') as output, self.data_path.open(encoding='utf-8') as stream:
            writer = csv.DictWriter(output,fieldnames=['timestamp','source_ip','source_port']+names)
            writer.writeheader()
            for line in stream:
                record = json.loads(line)
                writer.writerow(dict(timestamp=record['timestamp'],source_ip=record['address'][0],
                                     source_port=record['address'][1],**record['values']))
        self.status('CSV 已保存：'+str(target))

    def save_frame(self):
        frame = self.shown_frame
        if not frame:
            raise ValueError('尚未显示完整图像')
        target = self.output / f'总钻风_{datetime.now():%Y%m%d_%H%M%S_%f}_frame{frame.frame_id}.bmp'
        target.write_bytes(frame_bmp(frame))
        self.status('图像已保存：'+str(target))

    def send(self):
        text = dpg.get_value('send_text')
        data = bytes.fromhex(text) if dpg.get_value('send_mode') == '十六进制' else text.encode('utf-8')
        sent = send_udp(data,dpg.get_value('remote_ip').strip(),dpg.get_value('remote_port'))
        self.status(f'已发送 {sent} 字节（UDP 不保证对端收到）')

    def receive(self):
        if self.multiplex:
            self.multiplex.route()
        for packet in self.receivers['data'].drain(300):
            values, errors = parse_packet(packet.data,self.rules)
            # Read the controller curve state straight from the DATA packet,
            # independently of whichever rules are selected.
            self.curve_state, self.curve_level = merge_curve_level(values, packet.data)
            self.curve_stamp = packet.stamp
            for name in errors:
                self.latest.pop(name,None)
            self.latest.update(values)
            self.parse_errors += bool(errors)
            self.last_data = packet.stamp
            self.data_times.append(packet.stamp)
            if values:
                self.observe_module(packet.address[0])
                if packet.address[0] == self._observed_module:
                    self.link.received(packet.address,time.monotonic())
                if self.history_origin is None:
                    self.history_origin = packet.stamp
                self.data_file.write(json.dumps(dict(timestamp=datetime.fromtimestamp(packet.stamp).isoformat(timespec='milliseconds'),
                                                    address=packet.address,values=values),ensure_ascii=False)+'\n')
                self.data_records += 1
                for name,value in values.items():
                    self.history.setdefault(name,deque(maxlen=self.max_samples)).append((packet.stamp,value))
            if errors:
                message = '解析错误：'+'；'.join(f'{k}: {v}' for k,v in errors.items())
                if message != self._last_error:
                    self.status(message)
                self._last_error = message
            else:
                self._last_error = ''
        for packet in self.receivers['log'].drain(500):
            self.append_logs(self.decoder.feed(packet))
        self.append_logs(self.decoder.expire())
        for packet in self.receivers['camera'].drain(1200):
            frame = self.camera.feed(packet)
            if frame:
                self.frame = frame
                if packet.address[0] == self._observed_module:
                    self.link.received(packet.address,time.monotonic())
                self.last_camera = frame.stamp
                self.camera_times.append(frame.stamp)
        self.camera.expire()

    def refresh_logs(self):
        if self._pending_log_scroll and not dpg.get_value('log_pause'):
            if dpg.get_value('log_scroll'):
                # Content height is known after the preceding render, including the new rows.
                dpg.set_y_scroll('log_panel',dpg.get_y_scroll_max('log_panel'))
            self._pending_log_scroll = False
        if dpg.get_value('log_pause') or not self._log_dirty:
            return
        query = dpg.get_value('log_filter')
        timestamp = dpg.get_value('log_timestamp')
        lines = [line.format(timestamp) for line in self.logs if query.casefold() in line.text.casefold()][-1200:]
        signature = (query,timestamp)
        # Small batches append; filter/clear/resume/retention rebuilds are bounded to 1200 lines.
        old = [s for _,s in self._log_widgets]
        if signature == self._visible_signature and old and len(lines) >= len(old) and lines[:len(old)] == old:
            new_lines = lines[len(old):]
        else:
            dpg.delete_item('log_panel',children_only=True)
            self._log_widgets.clear()
            new_lines = lines
        for line in new_lines:
            error = any(word in line for word in ('操作失败','接收故障','解析错误','ERROR','[ERR]'))
            color = (255,120,120) if error else ((88,170,238) if '[上位机]' in line else (105,205,145))
            tag = dpg.add_text(line,parent='log_panel',color=color)
            self._log_widgets.append((tag,line))
        self._visible_signature = signature
        if dpg.get_value('log_scroll'):
            dpg.set_y_scroll('log_panel',dpg.get_y_scroll_max('log_panel'))
            self._pending_log_scroll = True
        self._log_dirty = False

    def refresh(self):
        now = time.monotonic()
        if now-self._last_refresh < 1/30:
            return
        self._last_refresh = now
        self.resize_layout()
        self.sync_quick_commands()
        if self._quick_save_due and not self.quick_load_error and now >= self._quick_save_due:
            try:
                self.save_quick_commands()
            except OSError as exc:
                self._quick_save_due = 0
                dpg.set_value('quick_status','自动保存失败：'+str(exc)+'；可点击保存列表重试。')
        self.collect_rules()
        self.receive()
        pulse = self.link.heartbeat(now,any(r.running for r in self.receivers.values()))
        if pulse:
            payload,peer = pulse
            try:
                send_udp(payload,*peer)
                self.link.sent += 1
            except OSError as exc:
                self.link.send_errors += 1
                if self.link.send_errors == 1:
                    self.status('接收心跳发送失败：'+str(exc))
        if any(r.running for r in self.receivers.values()) and now-self._receive_report_at >= 3:
            self._receive_report_at = now
            ages = ', '.join(f'{n}={r.received}/{r.dropped}/{int(r.running)}'
                             for n,r in self.receivers.items())
            self.append_logs([LogLine(time.time(),'上位机',
                f'[RX] 包数/队列丢弃/监听 {ages}; 完整图像={self.camera.completed}, '
                f'CRC无效={self.camera.invalid}, 未完整={self.camera.lost}; '
                f'参数距今={time.time()-self.last_data:.1f}s; 心跳={self.link.sent}/{self.link.send_errors}' +
                (f'; 单socket包={self.multiplex.receiver.received}, 原始队列丢弃={self.multiplex.receiver.dropped}, 类型包无效={self.multiplex.invalid}' if self.multiplex else ''))])
        self.refresh_tuning()
        wall = time.time()
        for times in (self.data_times,self.camera_times):
            while times and wall-times[0]>1:
                times.popleft()
        stats = '\n'.join(f'{label}：{r.received} 包\n丢弃 {r.dropped} / '+('未监听' if not r.running else '监听中')
                           for (name,r),label in zip(self.receivers.items(),('数据','日志','图像')))
        dpg.set_value('network_stats',stats)
        dpg.set_value('sidebar_rate',f'接收帧率：{len(self.data_times)} fps')
        for name in self._plot_names:
            dpg.set_value('value_'+name,f'{self.latest[name]:.5g}' if name in self.latest else '—')
        age = f'{wall-self.last_data:.1f} 秒前' if self.last_data else '尚未收到'
        dpg.set_value('log_stats',f'共 {self.total_lines} 行 / 参数 {self.data_records} 条 / 解析错误 {self.parse_errors} 包；最近参数：{age}；暂停时继续记录。')
        camera_receiver = self.receivers['camera']
        dpg.set_value('image_connection_status',f'{"监听中" if camera_receiver.running else "未监听"} / 完整图像 {self.camera.completed} 帧 / 接收 {len(self.camera_times)} fps；画面见左侧 04。')
        for name,receiver in self.receivers.items():
            if receiver.error and receiver.error != getattr(receiver,'reported_error',''):
                receiver.reported_error = receiver.error
                self.status(name+' 接收故障：'+receiver.error)
        active = self.active_page
        if active == 'params':
            self.refresh_logs()
        if active == 'plot' and now-self._last_plot > 1/15:
            self._last_plot = now
            for name in self._plot_names:
                samples = self.history[name]
                show = dpg.get_value('check_'+name)
                dpg.configure_item('series_'+name,show=show)
                if show and samples:
                    start = self.history_origin
                    dpg.set_value('series_'+name,[[t-start for t,v in samples],[v for t,v in samples]])
            if dpg.get_value('fit_plot'):
                dpg.fit_axis_data('plot_x')
                dpg.fit_axis_data('plot_y')
        if active == 'imu':
            if all(n in self.latest for n in ('roll','pitch','yaw')):
                self.last_angles = tuple(self.latest[n] for n in ('roll','pitch','yaw'))
                self.has_pose = True
                dpg.set_value('angles',f'角度来自接收数据 / 最近更新：{age}；yaw 为相对航向。')
                for name,value in zip(('roll','pitch','yaw'),self.last_angles):
                    dpg.set_value('angle_'+name,f'{value:+.2f}°')
            else:
                dpg.set_value('angles','当前角度无效，保留上次模型姿态。' if self.has_pose else '等待实测 roll / pitch / yaw；模型为零姿态预览。')
                for name in ('roll','pitch','yaw'):
                    dpg.set_value('angle_'+name,'—')
            config = dpg.get_item_configuration('pose')
            signature = (self.last_angles,dpg.get_value('view_yaw'),dpg.get_value('view_elev'),
                         dpg.get_value('view_zoom'),dpg.get_value('model_quality'),config['width'],config['height'])
            if signature != self._pose_signature:
                draw_attitude('pose',self.last_angles,signature[1],signature[2],signature[3],
                              quality='high' if signature[4] == '精细' else 'low')
                self._pose_signature = signature
        if active == 'camera':
            frame = self.frame
            if frame and not dpg.get_value('camera_pause') and frame is not self.shown_frame:
                size = (frame.width,frame.height)
                if size != self._last_texture_size:
                    # Image must be deleted before its texture; DearPyGui keeps a reference.
                    parent = dpg.get_item_parent('camera_image')
                    dpg.delete_item('camera_image')
                    dpg.delete_item('camera_texture')
                    dpg.add_dynamic_texture(*size,[0,0,0,1]*(size[0]*size[1]),tag='camera_texture',parent='texture_registry')
                    dpg.add_image('camera_texture',tag='camera_image',parent=parent)
                    self._last_texture_size = size
                    self._camera_display_signature = None
                dpg.set_value('camera_texture',frame_rgba(frame))
                self.shown_frame = frame
            self.resize_camera_display()
            details = f'{frame.width}×{frame.height} / 帧 {frame.frame_id} / {len(self.camera_times)} fps / {wall-frame.stamp:.1f} 秒前' if frame else '等待完整图像'
            dpg.set_value('camera_stats',f'{details}；完整 {self.camera.completed} 帧，未完整/超时 {self.camera.lost}，无效/CRC {self.camera.invalid}，重复包 {self.camera.duplicates}，旧帧 {self.camera.stale}')
            # Staleness uses the DATA packet stamp only; camera frames never refresh it.
            curve_age = (wall-self.curve_stamp) if self.curve_stamp else None
            curve = curve_status(self.curve_state,self.curve_level,curve_age)
            dpg.set_value('camera_curve_status',curve.text)
            dpg.configure_item('camera_curve_status',color=CURVE_STATUS_COLORS[curve.state])
            annotation = '等待本帧实际取值范围和拐点'
            displayed = self.shown_frame
            if displayed:
                if displayed.metadata:
                    far,near = displayed.sample_rows
                    state = ('无','候选','确认','短暂保持')[displayed.metadata[3]]
                    annotation = f'本帧前瞻中心 {(far+near)//2}；取值行 {far}～{near}（含端点）；拐点 {len(displayed.corners)} 个 / {state}；叠线第20～{displayed.height-21}行。'
                else:
                    annotation = '旧图像协议：可显示灰度与边界；更新车端固件后显示实际取值绿线和拐点。'
            dpg.set_value('camera_annotation_info',annotation)
        if now-self._last_flush > 1:
            self.log_file.flush()
            self.data_file.flush()
            self._last_flush = now

    def save_settings(self):
        settings = dict(ip=dpg.get_value('bind_ip'),rules=[asdict(r) for r in self.rules],
                        remote_ip=dpg.get_value('remote_ip'),remote_port=dpg.get_value('remote_port'),single_port=self.single_port,
                        rules_schema_version=2)
        for name in self.receivers:
            settings[name+'_port'] = 8081+list(self.receivers).index(name) if self.args.test_ports else dpg.get_value(name+'_port')
        temporary = STORE/'settings.tmp'
        temporary.write_text(json.dumps(settings,ensure_ascii=False,indent=2),encoding='utf-8')
        temporary.replace(STORE/'settings.json')

    def run(self):
        dpg.show_viewport()
        started = time.monotonic()
        if getattr(self.args,'listen_test',False):
            self.load_rules(TRACK_RULES)
            self.collect_rules()
            self.start()
        if getattr(self.args,'page',None):
            self.show_page(self.args.page)
        try:
            while dpg.is_dearpygui_running():
                jobs = dpg.get_callback_queue()
                if jobs:
                    dpg.run_callbacks(jobs)
                self.refresh()
                dpg.render_dearpygui_frame()
                if getattr(self.args,'startup_report',None):
                    Path(self.args.startup_report).write_text(json.dumps(dict(
                        started=True,frozen=bool(getattr(sys,'frozen',False)),pages=6,
                        idle_pose_items=sum(len(children) for children in dpg.get_item_children('pose').values()),
                        data_port=dpg.get_value('data_port'),log_port=dpg.get_value('log_port'),
                        camera_port=dpg.get_value('camera_port'),storage=str(STORE))),encoding='utf-8')
                    self.args.startup_report = None
                if getattr(self.args,'exit_after',0) and time.monotonic()-started >= self.args.exit_after:
                    if getattr(self.args,'test_report',None):
                        if self.shown_frame:
                            self.save_frame()
                        self.save_log()
                        if self.data_records:
                            self.save_csv()
                        Path(self.args.test_report).write_text(json.dumps(dict(
                            records=self.data_records,lines=self.total_lines,frames=self.camera.completed,
                            active_page=self.active_page,
                            shown_frame=self.shown_frame.frame_id if self.shown_frame else None,
                            shown_width=self.shown_frame.width if self.shown_frame else None,
                            shown_overlay_bytes=len(self.shown_frame.overlay) if self.shown_frame else 0,
                            shown_metadata_bytes=len(self.shown_frame.metadata) if self.shown_frame else 0,
                            shown_sample_rows=self.shown_frame.sample_rows if self.shown_frame else None,
                            shown_corner_count=len(self.shown_frame.corners) if self.shown_frame else 0,
                            displayed_log_rows=len(self._log_widgets),
                            pose_items=sum(len(children) for children in dpg.get_item_children('pose').values()),
                            plotted_points=len(dpg.get_value('series_roll')[0]) if 'roll' in self.history else 0,
                            source_is_synthetic_loopback=True,output=str(self.output))),encoding='utf-8')
                    break
        finally:
            self.stop()
            try:
                self.sync_quick_commands()
                if self._quick_save_due and not self.quick_load_error:
                    self.save_quick_commands()
            except OSError as exc:
                self.append_logs([LogLine(time.time(),'上位机','关闭时命令列表保存失败：'+str(exc))])
            self.save_settings()
            self.log_file.close()
            self.data_file.close()
            dpg.destroy_context()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test-ports',action='store_true',help='Use 18081..18083 for isolated loopback verification')
    parser.add_argument('--legacy-ports',action='store_true',help='Select old raw UDP ports 8081/8082/8083')
    parser.add_argument('--startup-report',help=argparse.SUPPRESS)
    parser.add_argument('--listen-test',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--page',choices=['params','plot','imu','log','camera','commands','tuning'],help=argparse.SUPPRESS)
    parser.add_argument('--exit-after',type=float,default=0,help=argparse.SUPPRESS)
    parser.add_argument('--test-report',help=argparse.SUPPRESS)
    args = parser.parse_args()
    if (args.listen_test or args.test_report) and not args.test_ports:
        parser.error('Automated loopback verification requires --test-ports')
    if sys.platform == 'win32':
        try:
            ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
        except (AttributeError,OSError):
            pass
    try:
        Monitor(args).run()
    except Exception:
        STORE.mkdir(parents=True,exist_ok=True)
        path = STORE/'error.log'
        path.write_text(traceback.format_exc(),encoding='utf-8')
        if sys.platform == 'win32':
            ctypes.windll.user32.MessageBoxW(0,'启动或运行出错，详情已保存：\n'+str(path),'Mcar WiFi Monitor',0)
        raise

if __name__ == '__main__':
    main()
