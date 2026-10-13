"""Restore the original aircraft scene using Rz(yaw) Ry(pitch) Rx(roll).

The OBJ files are the original assets. Projection, shading, depth sorting, and
color regions are restored from its bytecode, with exact top-view support and
resizing added. Drawing calls must run on the Dear PyGui GUI thread.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import dearpygui.dearpygui as dpg
from aircraft_model import load_aircraft_mesh

Point3 = tuple[float, float, float]
Point2 = tuple[float, float]
VIEW_PRESETS = {"isometric": (-55.0, 30.0), "front": (0.0, 0.0), "top": (0.0, 90.0)}


def rotate(point: Sequence[float], roll: float, pitch: float, yaw: float) -> Point3:
    """Rotate a body point; Euler angles here are in radians."""
    x, y, z = point
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return (cy * cp * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z,
            sy * cp * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z,
            -sp * x + cp * sr * y + cp * cr * z)


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def _unit(point: Point3) -> Point3:
    norm = math.sqrt(_dot(point, point))
    return tuple(value / norm for value in point)


@dataclass(frozen=True)
class Scene:
    points: tuple[Point2, ...]
    faces: tuple[tuple[float, tuple[int, ...], tuple[int, int, int, int]], ...]
    camera: Point3
    right: Point3
    up: Point3
    center: Point2
    scale: float
    angles: tuple[float, float, float]

    def project(self, point: Point3) -> Point2:
        return (self.center[0] + _dot(point, self.right) * self.scale,
                self.center[1] - _dot(point, self.up) * self.scale)


def _safe_angles(angles: Sequence[float] | None) -> tuple[float, float, float]:
    if angles is None:
        return (0.0, 0.0, 0.0)
    values = tuple(float(value) for value in angles)
    if len(values) != 3:
        raise ValueError("The aircraft requires roll, pitch, and yaw.")
    # Do not send NaN or infinity coordinates from a malformed packet to native code.
    return tuple(value if math.isfinite(value) else 0.0 for value in values)


def build_scene(width: float, height: float, angles: Sequence[float] | None,
                view_yaw: float = -55, view_elev: float = 30, zoom: float = 1,
                quality: str = "high") -> Scene:
    """Pure geometry preparation, independently testable without a viewport."""
    width, height = max(1.0, float(width)), max(1.0, float(height))
    angles = _safe_angles(angles)
    radians = tuple(map(math.radians, angles))
    azimuth, elevation = math.radians(view_yaw), math.radians(view_elev)
    camera = (math.cos(elevation) * math.cos(azimuth),
              math.cos(elevation) * math.sin(azimuth), math.sin(elevation))
    # Stable even at exactly +/-90 degrees elevation.
    right = (-math.sin(azimuth), math.cos(azimuth), 0.0)
    up = (-math.sin(elevation) * math.cos(azimuth),
          -math.sin(elevation) * math.sin(azimuth), math.cos(elevation))
    scale = min(width / 420.0, height / 350.0) * 1.5 * max(0.1, min(5.0, float(zoom)))
    center = (width / 2.0, height * 0.54)
    mesh = load_aircraft_mesh(quality)
    transformed = tuple(rotate(point, *radians) for point in mesh.vertices)
    projected = tuple((center[0] + _dot(point, right) * scale,
                       center[1] - _dot(point, up) * scale) for point in transformed)
    light = _unit((0.4, -0.4, 0.82))
    faces = []
    for face in mesh.faces:
        normal = rotate(face.normal, *radians)
        if _dot(normal, camera) <= 0.02:
            continue
        x, y, z = face.center
        if x > 65:
            base = (116, 155, 184)
        elif z > 15 and -25 < x < 45 and abs(y) < 27:
            base = (49, 103, 143)
        elif abs(y) > 82:
            base = (66, 147, 192)
        elif z < -14:
            base = (96, 125, 151)
        else:
            base = (168, 196, 216)
        brightness = 0.52 + 0.48 * max(0.0, _dot(normal, light))
        color = tuple(round(channel * brightness) for channel in base) + (255,)
        depth = _dot(rotate(face.center, *radians), camera)
        faces.append((depth, face.indices, color))
    faces.sort(key=lambda face: face[0])
    return Scene(projected, tuple(faces), camera, right, up, center, scale, angles)


def _dimensions(tag, width, height) -> tuple[float, float]:
    """Handle the first unrendered frame, where get_item_rect_size returns zero."""
    config = dpg.get_item_configuration(tag)
    try:
        actual = dpg.get_item_rect_size(tag)
    except (SystemError, RuntimeError):
        actual = (0, 0)
    result = []
    for axis, key in enumerate(("width", "height")):
        explicit = (width, height)[axis]
        value = explicit if explicit is not None else config.get(key, 0)
        if value is None or value <= 0:
            value = actual[axis] if actual[axis] > 0 else (800, 500)[axis]
        result.append(max(1, float(value)))
    return tuple(result)


def draw_attitude(tag, angles: Sequence[float] | None = (0, 0, 0),
                  view_yaw: float = -55, view_elev: float = 30, zoom: float = 1,
                  *, width: float | None = None, height: float | None = None,
                  quality: str = "high", ui_scale: float = 1) -> None:
    """Render a visible zero pose before the first IMU packet arrives."""
    width, height = _dimensions(tag, width, height)
    scene = build_scene(width, height, angles, view_yaw, view_elev, zoom, quality)
    ui_scale = max(0.65, min(2.0, float(ui_scale)))
    dpg.delete_item(tag, children_only=True)
    background = (13, 20, 31, 255)
    dpg.draw_rectangle((0, 0), (width, height), color=background, fill=background, parent=tag)
    for step in range(-360, 361, 60):
        for a, b in (((-360, step, -105), (360, step, -105)),
                     ((step, -360, -105), (step, 360, -105))):
            dpg.draw_line(scene.project(a), scene.project(b), color=(31, 49, 66), parent=tag)
    for a, b, color in (((-360, 0, -105), (360, 0, -105), (83, 60, 66)),
                         ((0, -360, -105), (0, 360, -105), (51, 89, 68))):
        dpg.draw_line(scene.project(a), scene.project(b), color=color, thickness=1.3, parent=tag)
    for _, indices, color in scene.faces:
        dpg.draw_polygon([scene.points[index] for index in indices], color=color,
                         fill=color, thickness=1.0, parent=tag)
    radians = tuple(map(math.radians, scene.angles))
    # Body convention: X forward, Y left, Z up, matching this car's IMU mount.
    for point, color, label in (((88, 0, 0), (255, 185, 78, 255), "FRONT"),
                                 ((0, 108, 0), (244, 87, 87, 255), "L"),
                                 ((0, -108, 0), (90, 228, 130, 255), "R")):
        x, y = scene.project(rotate(point, *radians))
        dpg.draw_circle((x, y), 4 * ui_scale, color=color, fill=color, parent=tag)
        dpg.draw_text((x + 9 * ui_scale, y - 8 * ui_scale), label,
                      color=color, size=15 * ui_scale, parent=tag)
    origin = (width - 68 * ui_scale, height - 72 * ui_scale)
    dpg.draw_circle(origin, 38 * ui_scale, color=(56, 77, 98),
                    fill=(18, 29, 43, 235), parent=tag)
    for direction, color, label in (((34, 0, 0), (245, 104, 104), "X"),
                                     ((0, 34, 0), (94, 223, 135), "Y"),
                                     ((0, 0, 34), (113, 165, 255), "Z")):
        direction = rotate(direction, *radians)
        tip = (origin[0] + _dot(direction, scene.right) * ui_scale,
               origin[1] - _dot(direction, scene.up) * ui_scale)
        dpg.draw_line(origin, tip, color=color, thickness=2 * ui_scale, parent=tag)
        dpg.draw_circle(tip, 3 * ui_scale, color=color, fill=color, parent=tag)
        dpg.draw_text((tip[0] + 5 * ui_scale, tip[1] - 7 * ui_scale), label,
                      color=color, size=14 * ui_scale, parent=tag)
    dpg.draw_text((18 * ui_scale, 16 * ui_scale), "IMU  /  ATTITUDE",
                  color=(204, 226, 243), size=17 * ui_scale, parent=tag)
    dpg.draw_text((18 * ui_scale, 43 * ui_scale), "ZYX Euler  /  degrees",
                  color=(115, 147, 173), size=13 * ui_scale, parent=tag)
    roll, pitch, yaw = scene.angles
    dpg.draw_text((18 * ui_scale, height - 28 * ui_scale),
                  f"ROLL {roll:+06.1f} deg    PITCH {pitch:+06.1f} deg    YAW {yaw:+06.1f} deg",
                  color=(177, 205, 226), size=14 * ui_scale, parent=tag)


def draw_aircraft_attitude(drawlist_tag, roll_deg, pitch_deg, yaw_deg,
                           view_yaw=-55, view_elev=30, zoom=1, ui_scale=1,
                           quality="high") -> None:
    """Compatibility entry point matching the original module."""
    draw_attitude(drawlist_tag, (roll_deg, pitch_deg, yaw_deg), view_yaw, view_elev,
                  zoom, quality=quality, ui_scale=ui_scale)
