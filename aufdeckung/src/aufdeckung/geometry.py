"""Geometric primitives: cage polygon, obstacle shapes, ray casting and distances.

All functions are vectorised with numpy where it matters (lidar, rasterising).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]

_EPS = 1e-9


@dataclass(frozen=True)
class Box:
    """Upright box: centre (x, y), footprint sx * sy, rotated by ``yaw`` (rad), from z0 to z0+height."""

    x: float
    y: float
    sx: float
    sy: float
    height: float
    yaw: float = 0.0
    z0: float = 0.0
    kind: str = "building"
    name: str = ""

    def __post_init__(self) -> None:
        if self.sx <= 0 or self.sy <= 0 or self.height <= 0 or self.z0 < 0:
            raise ValueError(f"invalid box dimensions for {self.name or self.kind}")

    @property
    def top(self) -> float:
        return self.z0 + self.height

    @property
    def bounding_radius(self) -> float:
        return 0.5 * math.hypot(self.sx, self.sy)


@dataclass(frozen=True)
class Cylinder:
    """Upright cylinder: centre (x, y), radius, from z0 to z0+height."""

    x: float
    y: float
    radius: float
    height: float
    z0: float = 0.0
    kind: str = "pole"
    name: str = ""

    def __post_init__(self) -> None:
        if self.radius <= 0 or self.height <= 0 or self.z0 < 0:
            raise ValueError(f"invalid cylinder dimensions for {self.name or self.kind}")

    @property
    def top(self) -> float:
        return self.z0 + self.height

    @property
    def bounding_radius(self) -> float:
        return self.radius


Shape = Box | Cylinder


# --------------------------------------------------------------------------- polygons


def regular_polygon(diameter: float, n_sides: int) -> FloatArray:
    """Vertices (n, 2) of a regular polygon whose vertices lie on a circle of ``diameter``."""
    angles = np.linspace(0.0, 2.0 * math.pi, n_sides, endpoint=False) + math.pi / n_sides
    r = diameter / 2.0
    return np.stack([r * np.cos(angles), r * np.sin(angles)], axis=1)


def points_in_polygon(px: FloatArray, py: FloatArray, poly: FloatArray) -> BoolArray:
    """Even-odd rule point-in-polygon for arrays of points."""
    inside = np.zeros(np.shape(px), dtype=bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        crosses = (y1 > py) != (y2 > py)
        with np.errstate(divide="ignore", invalid="ignore"):
            x_at = (x2 - x1) * (py - y1) / (y2 - y1 + _EPS) + x1
        inside ^= crosses & (px < x_at)
    return inside


def distance_to_polygon_edges(px: FloatArray, py: FloatArray, poly: FloatArray) -> FloatArray:
    """Unsigned distance from points to the polygon boundary."""
    best = np.full(np.shape(px), np.inf)
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        t = np.clip(((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy), 0.0, 1.0)
        best = np.minimum(best, np.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


def inside_with_margin(px: FloatArray, py: FloatArray, poly: FloatArray, margin: float) -> BoolArray:
    """True where a point is inside ``poly`` and at least ``margin`` from its boundary."""
    result: BoolArray = points_in_polygon(px, py, poly) & (
        distance_to_polygon_edges(px, py, poly) >= margin
    )
    return result


# --------------------------------------------------------------------------- shapes


def _to_local(shape: Box, px: FloatArray, py: FloatArray) -> tuple[FloatArray, FloatArray]:
    c, s = math.cos(shape.yaw), math.sin(shape.yaw)
    dx, dy = px - shape.x, py - shape.y
    return c * dx + s * dy, -s * dx + c * dy


def footprint_mask(shape: Shape, px: FloatArray, py: FloatArray, pad: float = 0.0) -> BoolArray:
    """True where (px, py) lies inside the shape footprint grown by ``pad``."""
    if isinstance(shape, Box):
        lx, ly = _to_local(shape, px, py)
        mask: BoolArray = (np.abs(lx) <= shape.sx / 2 + pad) & (np.abs(ly) <= shape.sy / 2 + pad)
        return mask
    cyl: BoolArray = np.hypot(px - shape.x, py - shape.y) <= shape.radius + pad
    return cyl


def distance_to_shape(shape: Shape, p: FloatArray) -> float:
    """Euclidean distance from point ``p`` (3,) to the solid shape (0 inside)."""
    px, py, pz = float(p[0]), float(p[1]), float(p[2])
    dz = max(shape.z0 - pz, 0.0, pz - shape.top)
    if isinstance(shape, Box):
        c, s = math.cos(shape.yaw), math.sin(shape.yaw)
        dx0, dy0 = px - shape.x, py - shape.y
        lx, ly = c * dx0 + s * dy0, -s * dx0 + c * dy0
        dx = max(abs(lx) - shape.sx / 2, 0.0)
        dy = max(abs(ly) - shape.sy / 2, 0.0)
        return math.sqrt(dx * dx + dy * dy + dz * dz)
    dr = max(math.hypot(px - shape.x, py - shape.y) - shape.radius, 0.0)
    return math.hypot(dr, dz)


def ray_cast(origin: FloatArray, dirs: FloatArray, shapes: list[Shape], max_range: float) -> FloatArray:
    """Distance along each unit ray in ``dirs`` (N, 3) to the first hit (ground z=0 included).

    Returns ``inf`` for rays that hit nothing within ``max_range``.
    """
    ox, oy, oz = (float(v) for v in origin)
    t_best = np.full(len(dirs), np.inf)
    dz = dirs[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        t_ground = np.where(dz < -_EPS, -oz / dz, np.inf)
    t_best = np.minimum(t_best, np.where(t_ground >= 0, t_ground, np.inf))
    near = [s for s in shapes if math.hypot(s.x - ox, s.y - oy) - s.bounding_radius <= max_range]
    boxes = [s for s in near if isinstance(s, Box)]
    if boxes:
        t_best = np.minimum(t_best, _ray_boxes(origin, dirs, boxes))
    for shape in near:
        if isinstance(shape, Cylinder):
            t_best = np.minimum(t_best, _ray_cyl(origin, dirs, shape))
    return np.where(t_best <= max_range, t_best, np.inf)


def _ray_boxes(origin: FloatArray, dirs: FloatArray, boxes: list[Box]) -> FloatArray:
    """Slab test of all rays against all boxes at once; returns the nearest hit per ray."""
    yaw = np.array([b.yaw for b in boxes])
    c, s = np.cos(yaw), np.sin(yaw)  # (M,)
    ox = origin[0] - np.array([b.x for b in boxes])
    oy = origin[1] - np.array([b.y for b in boxes])
    o = np.stack([c * ox + s * oy, -s * ox + c * oy, np.full(len(boxes), origin[2])], axis=1)  # (M,3)
    dx, dy, dz = dirs[:, 0:1], dirs[:, 1:2], dirs[:, 2:3]  # (N,1)
    d = np.stack([c * dx + s * dy, -s * dx + c * dy, np.broadcast_to(dz, (len(dirs), len(boxes)))],
                 axis=2)  # (N,M,3)
    lo = np.array([[-b.sx / 2, -b.sy / 2, b.z0] for b in boxes])
    hi = np.array([[b.sx / 2, b.sy / 2, b.top] for b in boxes])
    parallel = np.abs(d) < _EPS
    safe = np.where(parallel, 1.0, d)
    t1 = (lo - o) / safe
    t2 = (hi - o) / safe
    inside_slab = (o >= lo) & (o <= hi)  # (M,3)
    tmin_axes = np.where(parallel, np.where(inside_slab, -np.inf, np.inf), np.minimum(t1, t2))
    tmax_axes = np.where(parallel, np.where(inside_slab, np.inf, -np.inf), np.maximum(t1, t2))
    tmin = tmin_axes.max(axis=2)
    tmax = tmax_axes.min(axis=2)
    hit = (tmax >= np.maximum(tmin, 0.0)) & np.isfinite(tmax)
    t_hit = np.where(hit, np.maximum(tmin, 0.0), np.inf)
    result: FloatArray = t_hit.min(axis=1)
    return result


def _ray_box(origin: FloatArray, dirs: FloatArray, box: Box) -> FloatArray:
    c, s = math.cos(box.yaw), math.sin(box.yaw)
    ox, oy = origin[0] - box.x, origin[1] - box.y
    o = np.array([c * ox + s * oy, -s * ox + c * oy, origin[2]])
    d = np.stack(
        [c * dirs[:, 0] + s * dirs[:, 1], -s * dirs[:, 0] + c * dirs[:, 1], dirs[:, 2]], axis=1
    )
    lo = np.array([-box.sx / 2, -box.sy / 2, box.z0])
    hi = np.array([box.sx / 2, box.sy / 2, box.top])
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / d
        t1 = (lo - o) * inv
        t2 = (hi - o) * inv
    parallel = np.abs(d) < _EPS
    inside_slab = (o >= lo) & (o <= hi)
    tmin_axes = np.where(parallel, np.where(inside_slab, -np.inf, np.inf), np.minimum(t1, t2))
    tmax_axes = np.where(parallel, np.where(inside_slab, np.inf, -np.inf), np.maximum(t1, t2))
    tmin = np.max(tmin_axes, axis=1)
    tmax = np.min(tmax_axes, axis=1)
    hit = (tmax >= np.maximum(tmin, 0.0)) & np.isfinite(tmax)
    t_hit = np.where(tmin >= 0, tmin, 0.0)
    return np.where(hit, t_hit, np.inf)


def _ray_cyl(origin: FloatArray, dirs: FloatArray, cyl: Cylinder) -> FloatArray:
    ox, oy, oz = origin[0] - cyl.x, origin[1] - cyl.y, origin[2]
    dx, dy, dz = dirs[:, 0], dirs[:, 1], dirs[:, 2]
    result: FloatArray = np.full(len(dirs), np.inf)
    if ox * ox + oy * oy <= cyl.radius**2 and cyl.z0 <= oz <= cyl.top:
        return np.zeros(len(dirs))
    a = dx * dx + dy * dy
    b = 2 * (ox * dx + oy * dy)
    cc = ox * ox + oy * oy - cyl.radius**2
    disc = b * b - 4 * a * cc
    ok = (a > _EPS) & (disc >= 0)
    with np.errstate(divide="ignore", invalid="ignore"):
        t_side = (-b - np.sqrt(np.where(ok, disc, 0.0))) / (2 * a)
    z_side = oz + t_side * dz
    side_hit = ok & (t_side >= 0) & (z_side >= cyl.z0) & (z_side <= cyl.top)
    result = np.where(side_hit, t_side, result)
    for zc in (cyl.z0, cyl.top):
        with np.errstate(divide="ignore", invalid="ignore"):
            t_cap = np.where(np.abs(dz) > _EPS, (zc - oz) / dz, np.inf)
        finite = np.isfinite(t_cap)
        t_safe = np.where(finite, t_cap, 0.0)
        cx, cy = ox + t_safe * dx, oy + t_safe * dy
        cap_hit = (t_cap >= 0) & finite & (cx * cx + cy * cy <= cyl.radius**2)
        result = np.where(cap_hit & (t_cap < result), t_cap, result)
    return result
