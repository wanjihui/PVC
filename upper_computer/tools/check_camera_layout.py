"""Verify actual DearPyGui layout; no desktop input and no physical-device network.

Run with the project venv Python from the project root. The synthetic frame is
injected directly into the application model, and no UDP receiver is started.
"""
from argparse import Namespace
import ctypes
import json
import math
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

if sys.platform == 'win32':
    ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))

import dearpygui.dearpygui as dpg
import app
from camera_protocol import Frame


def render(monitor, count=16):
    for _ in range(count):
        monitor._last_refresh = 0
        monitor.refresh()
        dpg.render_dearpygui_frame()


def visible_bottom(tag):
    """Find the lowest currently shown content in a child window, including groups."""
    bottom = 0
    for children in dpg.get_item_children(tag).values():
        for child in children:
            if not dpg.get_item_configuration(child).get('show', True):
                continue
            state = dpg.get_item_state(child)
            if 'rect_min' in state and 'rect_size' in state:
                bottom = max(bottom, state['rect_min'][1] + state['rect_size'][1])
            if dpg.get_item_type(child) == 'mvAppItemType::mvGroup':
                bottom = max(bottom, visible_bottom(child))
    return bottom


def window_origin(tag):
    """Child windows omit rect_min; a leaf records both absolute and local coords."""
    pending = list(dpg.get_item_children(tag, 1))
    while pending:
        child = pending.pop(0)
        state = dpg.get_item_state(child)
        if 'rect_min' in state and 'pos' in state:
            return [state['rect_min'][axis] - state['pos'][axis] for axis in (0, 1)]
        pending.extend(dpg.get_item_children(child, 1))
    raise AssertionError(f'No rendered leaf found to locate {tag}')


def panel_metrics(tag):
    minimum = window_origin(tag)
    size = dpg.get_item_rect_size(tag)
    result = dict(size=list(size), scroll_y=dpg.get_y_scroll_max(tag),
                  scroll_x=dpg.get_x_scroll_max(tag),
                  last_content_bottom=visible_bottom(tag),
                  panel_bottom=minimum[1] + size[1])
    assert result['scroll_y'] <= 1, (tag, result)
    assert result['last_content_bottom'] <= result['panel_bottom'] - 5, (tag, result)
    return result


def camera_metrics():
    canvas_min = window_origin('camera_canvas')
    canvas_size = dpg.get_item_rect_size('camera_canvas')
    image_min = dpg.get_item_rect_min('camera_image')
    image_size = dpg.get_item_rect_size('camera_image')
    left = image_min[0] - canvas_min[0]
    top = image_min[1] - canvas_min[1]
    right = canvas_size[0] - left - image_size[0]
    bottom = canvas_size[1] - top - image_size[1]
    result = dict(canvas=list(canvas_size), image=list(image_size),
                  margins=[left, top, right, bottom],
                  scroll_x=dpg.get_x_scroll_max('camera_canvas'),
                  scroll_y=dpg.get_y_scroll_max('camera_canvas'))
    assert all(margin >= 5 for margin in result['margins']), result
    assert abs(left - right) <= 3 and abs(top - bottom) <= 3, result
    assert min(left + right, top + bottom) <= 20, result
    assert math.isclose(image_size[0] / image_size[1], 188 / 120, abs_tol=.004), result
    assert result['scroll_x'] <= 1 and result['scroll_y'] <= 1, result
    return result


def verify_size(width, height, directory):
    app.STORE = Path(directory)
    monitor = app.Monitor(Namespace(test_ports=True))
    try:
        dpg.configure_viewport(0, width=width, height=height)
        dpg.show_viewport()
        monitor.show_page('params')
        render(monitor)
        settings = {tag: panel_metrics(tag) for tag in
                    ('udp_listen_panel', 'udp_send_panel', 'image_connection')}
        monitor.frame = Frame(188, 120, 7, bytes([128] * 22560), time.time(), ('127.0.0.1', 18083))
        monitor.show_page('camera')
        render(monitor)
        camera = camera_metrics()
        actual = [dpg.get_viewport_width(), dpg.get_viewport_height()]
        assert monitor.shown_frame is monitor.frame
        assert monitor.shown_frame.pixels == bytes([128] * 22560)
        # Resizing while paused must still resize the displayed frame without
        # replacing it with the more recent received frame.
        dpg.set_value('camera_pause', True)
        frozen = monitor.shown_frame
        monitor.frame = Frame(188, 120, 8, bytes([64] * 22560), time.time(), ('127.0.0.1', 18083))
        dpg.configure_viewport(0, width=width + 80, height=height + 40)
        render(monitor)
        paused_camera = camera_metrics()
        assert monitor.shown_frame is frozen
        return dict(requested=[width, height], actual=actual,
                    paused_resized_actual=[dpg.get_viewport_width(), dpg.get_viewport_height()],
                    settings=settings, camera=camera, paused_resized_camera=paused_camera,
                    receivers_started=any(receiver.running for receiver in monitor.receivers.values()))
    finally:
        for receiver in monitor.receivers.values():
            receiver.stop()
        monitor.log_file.close()
        monitor.data_file.close()
        dpg.destroy_context()


def main():
    runtime = ROOT / 'tools' / 'runtime_temp'
    runtime.mkdir(exist_ok=True)
    results = []
    with tempfile.TemporaryDirectory(prefix='layout_native_', dir=runtime) as directory:
        for width, height in ((1800, 1120), (1250, 820)):
            results.append(verify_size(width, height, directory))
    report = ROOT / 'tools' / 'camera_layout_report.json'
    report.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
    print('PASS: native layout at two viewport sizes; settings have no internal vertical scroll;'
          ' fitted image preserves aspect ratio, fills one dimension and remains centered while paused.')
    print(json.dumps(results, ensure_ascii=False))


if __name__ == '__main__':
    main()
