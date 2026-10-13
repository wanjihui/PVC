"""End-to-end verification of our packaged application, using loopback test ports only."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
release_file = ROOT/'tools'/'release_path.txt'
release = Path(release_file.read_text('utf-8-sig').strip()) if release_file.exists() else ROOT/'dist'
single = len(sys.argv)>2 and sys.argv[2]=='single'
exe = release/'WiFiSPI_Monitor_SingleUDP.exe' if single else release/'WiFiSPI_Monitor_SingleUDP_Portable'/'WiFiSPI_Monitor_SingleUDP_Portable.exe'
page = sys.argv[1] if len(sys.argv)>1 else 'camera'
suffix = '_single' if single else ''
report = ROOT/'tools'/f'packaged_{page}{suffix}_report.json'
startup = ROOT/'tools'/f'packaged_{page}{suffix}_startup.json'
# Probe that these dedicated test ports are free; never touch an existing receiver.
probes = []
try:
    for port in (18081,18082,18083):
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        probes.append(sock)
        if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
        sock.bind(('127.0.0.1',port))
finally:
    for sock in probes:
        sock.close()
for file in (report,startup):
    if file.exists():
        file.unlink()
environment = os.environ.copy()
runtime = ROOT/'tools'/'runtime_temp'
runtime.mkdir(exist_ok=True)
environment['TEMP'] = environment['TMP'] = str(runtime)
process = subprocess.Popen([str(exe),'--test-ports','--listen-test','--page',page,
                            '--startup-report',str(startup),'--exit-after','9',
                            '--test-report',str(report)],cwd=ROOT,env=environment,
                            creationflags=subprocess.CREATE_NO_WINDOW)
try:
    deadline = time.monotonic()+20
    while not startup.exists() and process.poll() is None and time.monotonic()<deadline:
        time.sleep(.1)
    assert startup.exists(), 'Packaged application did not complete its first render'
    first_render = json.loads(startup.read_text('utf-8'))
    assert first_render['pages'] == 6 and first_render['idle_pose_items'] >= 10, first_render
    subprocess.run([sys.executable,str(ROOT/'tools'/'loopback_demo.py')],cwd=ROOT,check=True)
    process.wait(timeout=20)
    assert process.returncode == 0, f'Native application exit code {process.returncode}'
    result = json.loads(report.read_text('utf-8'))
    assert result['records'] == 80 and result['frames'] == 8, result
    assert result['active_page'] == ('params' if page == 'log' else page), result
    if page == 'camera':
        assert result['shown_frame'] == 7 and result['shown_width'] == 160, result
        assert result['shown_overlay_bytes'] == 480, result
        assert result['shown_metadata_bytes'] == 12 and result['shown_sample_rows'] == [50,70], result
        assert result['shown_corner_count'] == 4, result
        import csv
        saved = sorted(Path(result['output']).glob('数据_*.csv'))[-1]
        with saved.open(encoding='utf-8-sig',newline='') as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 80 and float(rows[-1]['vx_cmd_cms']) == 150
        assert int(rows[-1]['vis_interval_us']) == 20000
    if page in ('log','params'):
        assert result['displayed_log_rows'] >= 5, result
    if page == 'imu':
        assert result['pose_items'] >= 10, result
    if page == 'plot':
        assert result['plotted_points'] == 80, result
    print('PASS: packaged',page,'page, native rendering, 80 real loopback datagrams, 8 complete frames, log/CSV export')
    print(json.dumps(result,ensure_ascii=False))
finally:
    if process.poll() is None:
        process.kill()
        process.wait(timeout=5)
