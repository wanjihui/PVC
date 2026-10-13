"""Native GUI plus real loopback commands/ACKs, entirely separate from car ports."""
from argparse import Namespace
import ctypes
import json
from pathlib import Path
import socket
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
import app
import dearpygui.dearpygui as dpg
from live_tuning import PARAMETERS, EDITABLE_PARAMETERS, SPEED_LOOP_KEYS


def render(monitor, seconds=.12):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        monitor._last_refresh = 0
        monitor.refresh()
        dpg.render_dearpygui_frame()
        time.sleep(.005)


def command_payload(module):
    # The app legitimately sends ALIVE after a recognised reply; inspect tuning commands only.
    deadline = time.monotonic()+.6
    while time.monotonic()<deadline:
        payload,_ = module.recvfrom(4096)
        if not payload.startswith(b'MCAR ALIVE '): return payload
    raise socket.timeout()


def main():
    ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
    runtime = ROOT/'tools'/'runtime_temp'
    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='tuning_native_',dir=runtime) as directory, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as module:
        app.STORE = Path(directory)
        module.bind(('127.0.0.1',0))
        module.settimeout(.12)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1',0))
            log_port = probe.getsockname()[1]
        monitor = app.Monitor(Namespace(test_ports=True,legacy_ports=True))
        try:
            dpg.show_viewport()
            dpg.set_value('bind_ip','127.0.0.1')
            dpg.set_value('log_port',log_port)
            dpg.set_value('remote_ip','127.0.0.1')
            dpg.set_value('remote_port',module.getsockname()[1])
            monitor._start_receiver('log')
            monitor.show_page('tuning')
            render(monitor)
            assert dpg.is_item_shown('tuning_tab')
            assert len(dpg.get_item_children('tuning_table',1)) == len(EDITABLE_PARAMETERS)
            assert all(not dpg.does_item_exist('tune_edit_'+key) for key in SPEED_LOOP_KEYS)
            monitor.read_tuning()
            payload = command_payload(module)
            rid = int(payload.split()[2])
            assert payload == f'MCAR GET {rid} ALL\n'.encode()
            reply = ''.join(f'[PARAM] {rid} OK {p.key} {p.default}\r\n' for p in PARAMETERS)+f'[PARAM] {rid} DONE\r\n'
            module.sendto(reply[:31].encode(),('127.0.0.1',log_port))
            module.sendto(reply[31:].encode(),('127.0.0.1',log_port))
            render(monitor)
            assert dpg.get_value('tune_actual_omega_kp1') == '1.6'
            assert '已读取' in dpg.get_value('tune_state_omega_kp1')
            monitor.copy_tuning_actual() # full 36-field reply must not access deleted GUI items
            dpg.set_value('tune_edit_omega_kp1',1.7)
            tag = 'tune_apply_omega_kp1'
            dpg.get_item_callback(tag)(tag,None,None)
            payload = command_payload(module)
            rid = int(payload.split()[2])
            assert payload == f'MCAR SET {rid} omega_kp1 1.7\n'.encode()
            assert dpg.get_value('tune_actual_omega_kp1') == '1.6'
            module.sendto(f'[PARAM] {rid} OK omega_kp1 1.700000\r\n'.encode(),('127.0.0.1',log_port))
            render(monitor)
            assert dpg.get_value('tune_actual_omega_kp1') == '1.7'
            dpg.set_value('tune_edit_omega_kp1',5)
            dpg.get_item_callback(tag)(tag,None,None)
            with_error = dpg.get_value('tuning_status')
            assert '范围' in with_error
            try:
                command_payload(module)
                raise AssertionError('Invalid edit was sent')
            except socket.timeout:
                pass
            dpg.set_value('tune_edit_omega_kp1',1.7)
            scheme = Path(directory)/'方案.json'
            dpg.set_value('tuning_file_path',str(scheme))
            monitor.export_tuning()
            monitor.default_tuning()
            assert dpg.get_value('tune_edit_omega_kp1') == 1.6
            monitor.import_tuning()
            assert dpg.get_value('tune_edit_omega_kp1') == 1.7
            try:
                command_payload(module)
                raise AssertionError('Local import sent a command')
            except socket.timeout:
                pass
            model_items = sum(len(x) for x in dpg.get_item_children('pose').values())
            assert model_items >= 700
            for page in ('params','plot','imu','camera','commands','tuning'):
                monitor.show_page(page)
                render(monitor,.03)
                assert dpg.is_item_shown(page+'_tab')
            for width,height in ((1800,1120),(1250,820)):
                dpg.configure_viewport(0,width=width,height=height)
                render(monitor,.06)
                assert dpg.get_item_state('tune_apply_cruise_cms')['visible']
                help_state = dpg.get_item_state('tune_help_cruise_cms')
                apply_state = dpg.get_item_state('tune_apply_cruise_cms')
                assert help_state['rect_max'][0] <= apply_state['rect_min'][0]+2
            report = dict(pages=6,parameters=len(EDITABLE_PARAMETERS),firmware_fields=len(PARAMETERS),speed_fields_removed=True,
                          real_loopback=True,partial_multiline_ack=True,
                          ack_required=True,invalid_not_sent=True,scheme_import_no_send=True,
                          model_items=model_items,car_contacted=False)
            (ROOT/'tools'/'live_tuning_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            print(json.dumps(report))
        finally:
            monitor.stop()
            monitor.log_file.close()
            monitor.data_file.close()
            dpg.destroy_context()


if __name__ == '__main__':
    main()
