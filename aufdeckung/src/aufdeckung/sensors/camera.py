"""Nadir camera model: which surface cells are imaged from a given pose.

Used three ways: by the simulator (true map -> ground-truth coverage), by the
mission controller (belief map -> what we think we have seen) and by the
planner (belief map -> predicted coverage of candidate viewpoints).
"""

from __future__ import annotations

import math

import numpy as np

from aufdeckung.config import CameraConfig
from aufdeckung.geometry import BoolArray, FloatArray
from aufdeckung.mapping.grid import GridSpec, IntArray


def visible_cells(
    pos: tuple[float, float, float] | FloatArray,
    height: FloatArray,
    spec: GridSpec,
    camera: CameraConfig,
) -> tuple[IntArray, IntArray]:
    """Indices (iy, ix) of surface cells inside the camera footprint with a clear line of sight."""
    x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
    hx, hy = camera.half_extents(min(z, camera.max_detect_agl))
    ix0 = max(math.floor((x - hx - spec.xmin) / spec.res), 0)
    ix1 = min(math.floor((x + hx - spec.xmin) / spec.res), spec.nx - 1)
    iy0 = max(math.floor((y - hy - spec.ymin) / spec.res), 0)
    iy1 = min(math.floor((y + hy - spec.ymin) / spec.res), spec.ny - 1)
    empty = np.zeros(0, dtype=np.int64)
    if ix0 > ix1 or iy0 > iy1:
        return empty, empty
    iy, ix = np.mgrid[iy0 : iy1 + 1, ix0 : ix1 + 1]
    iy, ix = iy.ravel().astype(np.int64), ix.ravel().astype(np.int64)
    cx = spec.xmin + (ix + 0.5) * spec.res
    cy = spec.ymin + (iy + 0.5) * spec.res
    hs = height[iy, ix]
    agl = z - hs
    tan_h, tan_v = math.tan(camera.hfov / 2), math.tan(camera.vfov / 2)
    ok = (
        (agl >= camera.min_agl)
        & (agl <= camera.max_detect_agl)
        & (np.abs(cx - x) <= agl * tan_h)
        & (np.abs(cy - y) <= agl * tan_v)
    )
    iy, ix, cx, cy, hs = iy[ok], ix[ok], cx[ok], cy[ok], hs[ok]
    if len(iy) == 0:
        return empty, empty
    dist_xy = np.hypot(cx - x, cy - y)
    k = max(math.ceil(float(dist_xy.max()) / (spec.res / 2)), 1)
    f = (np.arange(1, k + 1) / (k + 1))[None, :]
    sx = x + (cx[:, None] - x) * f
    sy = y + (cy[:, None] - y) * f
    sz = z + (hs[:, None] - z) * f
    siy, six = spec.indices(sx, sy)
    siy = np.clip(siy, 0, spec.ny - 1)
    six = np.clip(six, 0, spec.nx - 1)
    own = (siy == iy[:, None]) & (six == ix[:, None])
    blocked = ((height[siy, six] > sz + 1e-3) & ~own).any(axis=1)
    return iy[~blocked], ix[~blocked]


def visible_mask(
    pos: tuple[float, float, float] | FloatArray,
    height: FloatArray,
    spec: GridSpec,
    camera: CameraConfig,
) -> BoolArray:
    mask = np.zeros(spec.shape, dtype=bool)
    iy, ix = visible_cells(pos, height, spec, camera)
    mask[iy, ix] = True
    return mask
