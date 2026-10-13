"""Native GUI callbacks and real UDP loopback; never send to a car."""
from argparse import Namespace
import ctypes
import json
from pathlib import Path
import socket
import struct
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
import app
import dearpygui.dearpygui as dpg
from quick_commands import load_config

def render(monitor):
    for _ in range(10):
        monitor._last_refresh=0
        monitor.refresh()
        dpg.render_dearpygui_frame()

def invoke(tag):
    dpg.get_item_callback(tag)(tag,None,None)

def main():
    ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
    runtime=ROOT/'tools'/'runtime_temp'
    runtime.mkdir(exist_ok=True)
    results=[]
    with tempfile.TemporaryDirectory(prefix='quick_native_',dir=runtime) as directory, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as server:
        server.bind(('127.0.0.1',0))
        server.settimeout(.25)
        app.STORE=Path(directory)
        monitor=app.Monitor(Namespace(test_ports=True))
        try:
            dpg.show_viewport()
            monitor.show_page('commands')
            dpg.set_value('remote_ip','127.0.0.1')
            dpg.set_value('remote_port',server.getsockname()[1])
            render(monitor)
            cid=monitor.quick_rows[0]
            prefix='qc_'+cid+'_'
            dpg.set_value(prefix+'content','GET_STATE')
            dpg.set_value(prefix+'note','备注只在电脑显示')
            dpg.set_value(prefix+'ending','LF')
            invoke(prefix+'send')
            assert server.recvfrom(65535)[0]==b'GET_STATE\n'
            assert '未确认执行' in dpg.get_value(prefix+'result')
            monitor.edit_quick_list('copy',cid)
            copied=monitor.quick_rows[1]
            dpg.set_value('qc_'+copied+'_format','HEX')
            dpg.set_value('qc_'+copied+'_content','AA 55 00 FF')
            invoke('qc_'+copied+'_send')
            assert server.recvfrom(65535)[0]==b'\xaa\x55\x00\xff'
            dpg.set_value('qc_'+copied+'_format','Binary')
            dpg.set_value('qc_'+copied+'_content','[{"type":"uint16","value":4660},{"type":"float32","value":0.5}]')
            invoke('qc_'+copied+'_send')
            assert server.recvfrom(65535)[0]==struct.pack('<Hf',4660,.5)
            dpg.set_value('qc_'+copied+'_format','JSON')
            dpg.set_value('qc_'+copied+'_content','{"cmd":"STATE"}')
            invoke('qc_'+copied+'_send')
            assert server.recvfrom(65535)[0]==b'{"cmd":"STATE"}'
            dpg.set_value('qc_'+copied+'_format','GBK')
            dpg.set_value('qc_'+copied+'_content','查询状态')
            invoke('qc_'+copied+'_send')
            assert server.recvfrom(65535)[0]=='查询状态'.encode('gbk')
            dpg.set_value('qc_'+copied+'_format','HEX')
            dpg.set_value('qc_'+copied+'_content','ZZ')
            invoke('qc_'+copied+'_send')
            assert '发送失败' in dpg.get_value('qc_'+copied+'_result')
            try:
                server.recvfrom(65535)
                raise AssertionError('Invalid command was sent')
            except socket.timeout:
                pass
            monitor.edit_quick_list('up',copied)
            assert monitor.quick_rows==[copied,cid]
            monitor.edit_quick_list('delete',copied)
            dpg.set_value(prefix+'note','保存后重新打开仍存在')
            monitor.sync_quick_commands()
            monitor._quick_save_due=time.monotonic()-1
            render(monitor)
            assert load_config(monitor.quick_path)['commands'][0]['note']=='保存后重新打开仍存在'
            monitor.export_quick_commands()
            exported=dpg.get_value('quick_file_path')
            monitor.edit_quick_list('add')
            monitor.import_quick_commands()
            assert len(monitor.quick_rows)==1
            assert list(Path(directory).glob('quick_commands_before_import_*.json'))
            dpg.set_value('quick_file_path',str(Path(directory)/'bad.json'))
            Path(dpg.get_value('quick_file_path')).write_text('{"version":99}',encoding='utf-8')
            try:
                monitor.import_quick_commands()
                raise AssertionError('Bad JSON was imported')
            except ValueError:
                pass
            assert len(monitor.quick_rows)==1
            monitor.save_quick_commands()
            for width,height in ((1800,1120),(1250,820)):
                dpg.configure_viewport(0,width=width,height=height)
                render(monitor)
                assert dpg.get_item_state(prefix+'send')['visible']
                results.append(dict(viewport=[width,height],page=list(dpg.get_item_rect_size('commands_tab')),
                                    send_button=list(dpg.get_item_rect_size(prefix+'send'))))
        finally:
            monitor.log_file.close();monitor.data_file.close();dpg.destroy_context()
        restored=app.Monitor(Namespace(test_ports=True))
        try:
            assert restored.quick_config['commands'][0]['content']=='GET_STATE'
            assert restored.quick_config['commands'][0]['note']=='保存后重新打开仍存在'
        finally:
            restored.log_file.close();restored.data_file.close();dpg.destroy_context()
    report=dict(pages=6,udp_packets=5,invalid_not_sent=True,restored=True,auto_saved=True,
                imported=True,car_contacted=False,layouts=results)
    (ROOT/'tools'/'quick_commands_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('PASS: native quick command callbacks, loopback bytes, validation, persistence, import/export and two sizes')
    print(json.dumps(report,ensure_ascii=False))

if __name__=='__main__':
    main()
