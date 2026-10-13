"""Check original assets, Euler conventions, and actual DPG drawing calls."""
import math

import dearpygui.dearpygui as dpg
import pytest

from aircraft_model import load_aircraft_mesh
from attitude_3d import VIEW_PRESETS, build_scene, draw_attitude, rotate


@pytest.mark.parametrize("quality", ("high", "low"))
def test_original_mesh_is_solid_and_normalized(quality):
    mesh = load_aircraft_mesh(quality)
    assert len(mesh.vertices) > 100
    assert len(mesh.faces) > 100
    assert max(vertex[1] for vertex in mesh.vertices) - min(vertex[1] for vertex in mesh.vertices) == pytest.approx(215)
    assert all(sum(value * value for value in face.normal) == pytest.approx(1) for face in mesh.faces)
    assert load_aircraft_mesh(quality) is mesh


def test_right_hand_body_rotation_and_zyx_order():
    assert rotate((1, 0, 0), 0, 0, math.pi / 2) == pytest.approx((0, 1, 0), abs=1e-12)
    assert rotate((1, 0, 0), 0, math.pi / 2, 0) == pytest.approx((0, 0, -1), abs=1e-12)
    assert rotate((0, 1, 0), math.pi / 2, 0, 0) == pytest.approx((0, 0, 1), abs=1e-12)
    assert rotate((1, 0, 0), 0, math.pi / 2, math.pi / 2) == pytest.approx((0, 0, -1), abs=1e-12)


@pytest.mark.parametrize("preset", VIEW_PRESETS.values())
def test_all_views_have_visible_faces_and_finite_projection(preset):
    scene = build_scene(1000, 700, None, *preset)
    assert len(scene.faces) > 50
    assert all(math.isfinite(value) for point in scene.points for value in point)
    assert scene.angles == (0, 0, 0)


def test_projection_scales_and_moves_with_angles():
    initial = build_scene(800, 600, (0, 0, 0))
    bigger = build_scene(1600, 1200, (0, 0, 0))
    rotated = build_scene(800, 600, (20, -15, 80))
    assert bigger.scale == pytest.approx(initial.scale * 2)
    assert bigger.points[0] == pytest.approx(tuple(value * 2 for value in initial.points[0]))
    assert rotated.points != initial.points
    assert build_scene(800, 600, (float("nan"), float("inf"), 10)).angles == (0, 0, 10)


def test_native_dpg_draws_zero_pose_and_resized_top_view():
    dpg.create_context()
    try:
        with dpg.window():
            dpg.add_drawlist(width=900, height=600, tag="aircraft_test")
        for angles, view, size in (((0, 0, 0), (-55, 30), (900, 600)),
                                  ((30, -20, 70), (0, 90), (600, 400))):
            draw_attitude("aircraft_test", angles, *view, width=size[0], height=size[1])
            children = dpg.get_item_children("aircraft_test", 2)
            assert len(children) > 100
            assert sum("drawpolygon" in dpg.get_item_info(item)["type"].lower() for item in children) > 50
    finally:
        dpg.destroy_context()
