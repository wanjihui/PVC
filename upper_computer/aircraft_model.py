"""Load the original aircraft OBJ assets extracted from the user's monitor.

Coordinate conversion and the 215-unit wingspan follow its original bytecode.
Parsing is cached, so drawing never reopens an OBJ file.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
from pathlib import Path
import sys

Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class MeshFace:
    indices: tuple[int, ...]
    center: Point3
    normal: Point3


@dataclass(frozen=True)
class AircraftMesh:
    vertices: tuple[Point3, ...]
    faces: tuple[MeshFace, ...]


def resource_path(name: str) -> Path:
    """Support source runs and PyInstaller's one-file resource directory."""
    root = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    path = root / "model_assets" / name
    return path if path.is_file() else root / name


def _cross(a: Point3, b: Point3) -> Point3:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


@lru_cache(maxsize=2)
def load_aircraft_mesh(quality: str = "high") -> AircraftMesh:
    if quality not in ("high", "low"):
        raise ValueError(f"Unsupported aircraft quality: {quality}")
    raw_vertices: list[Point3] = []
    raw_faces: list[tuple[int, ...]] = []
    path = resource_path(f"aircraft_{quality}.obj")
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if line.startswith("v "):
                raw_vertices.append(tuple(map(float, line.split()[1:4])))
            elif line.startswith("f "):
                indices = tuple(int(token.split("/", 1)[0]) for token in line.split()[1:])
                indices = tuple(index - 1 if index > 0 else len(raw_vertices) + index
                                for index in indices)
                if len(indices) >= 3:
                    raw_faces.append(indices)
    if not raw_vertices or not raw_faces:
        raise ValueError(f"No valid aircraft vertices or faces: {path}")
    minima = [min(vertex[axis] for vertex in raw_vertices) for axis in range(3)]
    maxima = [max(vertex[axis] for vertex in raw_vertices) for axis in range(3)]
    middle = [(a + b) / 2 for a, b in zip(minima, maxima)]
    scale = 215.0 / (maxima[0] - minima[0])
    # OBJ axes: X across the wings, Y upwards, Z towards the nose.
    vertices = tuple(((point[2] - middle[2]) * scale,
                      (point[0] - middle[0]) * scale,
                      (point[1] - middle[1]) * scale) for point in raw_vertices)
    faces = []
    for indices in raw_faces:
        if any(index < 0 or index >= len(vertices) for index in indices):
            raise ValueError(f"Invalid OBJ face index: {path}")
        a, b, c = (vertices[index] for index in indices[:3])
        ab = tuple(b[axis] - a[axis] for axis in range(3))
        ac = tuple(c[axis] - a[axis] for axis in range(3))
        normal = _cross(ab, ac)
        norm = math.sqrt(sum(value * value for value in normal))
        if norm < 1e-8:
            continue
        normal = tuple(value / norm for value in normal)
        center = tuple(sum(vertices[index][axis] for index in indices) / len(indices)
                       for axis in range(3))
        faces.append(MeshFace(indices, center, normal))
    return AircraftMesh(vertices, tuple(faces))
