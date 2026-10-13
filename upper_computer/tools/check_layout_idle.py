from argparse import Namespace
import ctypes
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from app import Monitor
ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
args = Namespace(test_ports=True, listen_test=False, page='imu', startup_report=str(Path(__file__).with_name('layout_idle_startup.json')), exit_after=2, test_report=str(Path(__file__).with_name('layout_idle_report.json')))
Monitor(args).run()
startup = json.loads(Path(args.startup_report or Path(__file__).with_name('layout_idle_startup.json')).read_text('utf-8'))
result = json.loads(Path(args.test_report).read_text('utf-8'))
assert startup['pages'] == 6 and startup['idle_pose_items'] >= 700, startup
assert result['records'] == 0 and result['frames'] == 0 and result['pose_items'] >= 700, result
print('PASS: offline native IMU page, original aircraft visible before any data arrives')
