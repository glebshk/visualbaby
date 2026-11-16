"""Regular 2D grid over the cage and the 2.5D height map built on it."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from aufdeckung.config import ArenaConfig
from aufdeckung.geometry import (
    BoolArray,
    FloatArray,
    Shape,
    footprint_mask,
    inside_with_margin,
    points_in_polygon,
    regular_polygon,
)

IntArray = NDArray[np.int64]
UNSEEN_TOP_MARGIN = 2.0
CARVE_MAX_RANGE = 12.0  # beyond this, beams spread too far apart to vouch for thin obstacles
CARVE_END_EXCLUSION = 0.75  # ignore beam samples this close (xy) to the beam's own hit point


@dataclass(frozen=True)
class GridSpec:
    """Square cells of size ``res``; cell (iy, ix) has centre (xmin + (ix+.5) res, ymin + (iy+.5) res)."""

    xmin: float
    ymin: float
    res: float
    nx: int
    ny: int

    @classmethod
    def for_arena(cls, arena: ArenaConfig, res: float) -> GridSpec:
        r = arena.diameter / 2
        n = math.ceil(2 * r / res)
        return cls(-n * res / 2, -n * res / 2, res, n, n)

    @property
    def shape(self) -> tuple[int, int]:
        return self.ny, self.nx

    def centers(self) -> tuple[FloatArray, FloatArray]:
        xs = self.xmin + (np.arange(self.nx) + 0.5) * self.res
        ys = self.ymin + (np.arange(self.ny) + 0.5) * self.res
        gx, gy = np.meshgrid(xs, ys)
        return gx, gy

    def index(self, x: float, y: float) -> tuple[int, int]:
        return math.floor((y - self.ymin) / self.res), math.floor((x - self.xmin) / self.res)

    def indices(self, x: FloatArray, y: FloatArray) -> tuple[IntArray, IntArray]:
        iy = np.floor((y - self.ymin) / self.res).astype(np.int64)
        ix = np.floor((x - self.xmin) / self.res).astype(np.int64)
        return iy, ix

    def valid(self, iy: IntArray, ix: IntArray) -> BoolArray:
        ok: BoolArray = (iy >= 0) & (iy < self.ny) & (ix >= 0) & (ix < self.nx)
        return ok

    def contains(self, x: float, y: float) -> bool:
        iy, ix = self.index(x, y)
        return 0 <= iy < self.ny and 0 <= ix < self.nx

    def center(self, iy: int, ix: int) -> tuple[float, float]:
        return self.xmin + (ix + 0.5) * self.res, self.ymin + (iy + 0.5) * self.res


def cage_mask(spec: GridSpec, arena: ArenaConfig, margin: float = 0.0) -> BoolArray:
    """Cells whose centre is inside the cage (and ``margin`` away from the net)."""
    gx, gy = spec.centers()
    poly = regular_polygon(arena.diameter, arena.n_sides)
    if margin <= 0:
        return points_in_polygon(gx, gy, poly)
    return inside_with_margin(gx, gy, poly, margin)


def rasterize(shapes: tuple[Shape, ...] | list[Shape], spec: GridSpec) -> FloatArray:
    """Height map of the tallest obstacle top touching each cell (0 = ground)."""
    gx, gy = spec.centers()
    height: FloatArray = np.zeros(spec.shape)
    pad = spec.res / 2
    for shape in shapes:
        mask = footprint_mask(shape, gx, gy, pad=pad)
        height = np.where(mask, np.maximum(height, shape.top), height)
    return height


def disk_offsets(radius_cells: float) -> list[tuple[int, int]]:
    r = math.ceil(radius_cells)
    return [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1)
            if dx * dx + dy * dy <= radius_cells * radius_cells + 1e-9]


def max_filter_disk(values: FloatArray, radius_cells: float, fill: float = 0.0) -> FloatArray:
    """Maximum of ``values`` over a disk neighbourhood (cells outside the grid count as ``fill``)."""
    r = math.ceil(radius_cells)
    padded = np.pad(values, r, constant_values=fill)
    out = np.full(values.shape, -np.inf)
    ny, nx = values.shape
    for dy, dx in disk_offsets(radius_cells):
        out = np.maximum(out, padded[r + dy : r + dy + ny, r + dx : r + dx + nx])
    return out


def dilate(mask: BoolArray, radius_cells: float) -> BoolArray:
    result: BoolArray = max_filter_disk(mask.astype(np.float64), radius_cells) > 0.5
    return result


class HeightMap:
    """Belief 2.5D map: per-cell maximum observed surface height plus an 'observed' flag.

    In explore mode the map starts empty and is filled from lidar hits. A prior
    (e.g. the organisers' layout plan) can be loaded with :meth:`from_prior`.
    """

    def __init__(self, spec: GridSpec, arena: ArenaConfig) -> None:
        self.spec = spec
        self.arena = arena
        self.height: FloatArray = np.zeros(spec.shape)
        self.hit_height: FloatArray = np.zeros(spec.shape)  # highest real lidar hit, no margin
        self.observed: BoolArray = np.zeros(spec.shape, dtype=bool)
        self.version = 0

    @classmethod
    def from_prior(cls, spec: GridSpec, arena: ArenaConfig, shapes: tuple[Shape, ...] | list[Shape]
                   ) -> HeightMap:
        hm = cls(spec, arena)
        hm.height = rasterize(shapes, spec)
        hm.hit_height = hm.height.copy()
        hm.observed[:] = True
        return hm

    def integrate_hits(self, points: FloatArray, sensor_z: float, top_beam: BoolArray | None = None,
                       origin: tuple[float, float, float] | FloatArray | None = None) -> int:
        """Fuse lidar hit points (N, 3) in world frame. Returns the number of cells that grew.

        Hits from the highest beam at or above the sensor height mean the obstacle
        may continue upward out of view, so the cell is raised by ``UNSEEN_TOP_MARGIN``.
        Later scans from higher up raise it further if needed.

        With ``origin`` (the sensor position), beams also carve free space: a beam that passes
        over a cell proves the cell is lower there, which removes that guessed margin again
        (never below a real hit). Without it, a roof edge seen from just below its top stays
        2 m too tall for the whole flight and the roof can never be imaged.
        """
        if len(points) == 0:
            return 0
        iy, ix = self.spec.indices(points[:, 0], points[:, 1])
        ok = self.spec.valid(iy, ix)
        raw_z = np.where(points[:, 2] < 0.15, 0.0, np.clip(points[:, 2], 0.0, self.arena.height))
        if origin is not None:
            self._carve(np.asarray(origin, dtype=float), points, iy, ix, ok)
        np.maximum.at(self.hit_height, (iy[ok], ix[ok]), raw_z[ok])
        z = np.clip(points[:, 2], 0.0, self.arena.height)
        if top_beam is not None:
            z = np.where(top_beam & (points[:, 2] >= sensor_z - 0.05),
                         np.minimum(z + UNSEEN_TOP_MARGIN, self.arena.height), z)
        z = np.where(z < 0.15, 0.0, z)  # ground noise
        iy, ix, z = iy[ok], ix[ok], z[ok]
        before = self.height.copy()
        np.maximum.at(self.height, (iy, ix), z)
        self.observed[iy, ix] = True
        grown = int(np.count_nonzero(self.height > before + 0.05))
        if grown or len(iy):
            self.version += 1
        return grown

    def _carve(self, origin: FloatArray, points: FloatArray, hit_iy: IntArray, hit_ix: IntArray,
               hit_ok: BoolArray) -> None:
        """Lower margin-inflated cells that beams of this scan demonstrably passed over."""
        inflated = self.height > self.hit_height + 0.05
        if not inflated.any():
            return
        # A beam can pass through a cell beside a thin pole that other beams of the same scan hit.
        # So within one cell of this scan's hits, only beams passing above the highest of those
        # hits count as evidence (a roof edge hit at 8.3 m, overflown by a beam at 8.6 m).
        hit_now = np.full(self.spec.shape, -np.inf)
        np.maximum.at(hit_now, (hit_iy[hit_ok], hit_ix[hit_ok]), points[hit_ok, 2])
        protect_below = max_filter_disk(hit_now, 1.5, fill=-np.inf) + 0.1
        candidates = inflated
        z_floor = float(np.min(self.hit_height[candidates]))
        vec = points - origin[None, :]
        length = np.linalg.norm(vec, axis=1)
        horiz = np.hypot(vec[:, 0], vec[:, 1])
        step = self.spec.res / 2
        k = max(math.ceil(min(float(np.max(length)), CARVE_MAX_RANGE) / step), 1)
        f = (np.arange(1, k + 1) * step)[None, :] / np.maximum(length, 1e-9)[:, None]  # (N, k)
        keep = (f < 1.0) & ((1.0 - f) * horiz[:, None] > CARVE_END_EXCLUSION)
        sx = origin[0] + vec[:, 0:1] * f
        sy = origin[1] + vec[:, 1:2] * f
        sz = origin[2] + vec[:, 2:3] * f
        keep &= sz > z_floor
        if not keep.any():
            return
        sx, sy, sz = sx[keep], sy[keep], sz[keep]
        siy, six = self.spec.indices(sx, sy)
        valid = self.spec.valid(siy, six)
        siy, six, sz = siy[valid], six[valid], sz[valid]
        sel = candidates[siy, six] & (sz < self.height[siy, six]) & (sz > protect_below[siy, six])
        if not sel.any():
            return
        cap = np.full(self.spec.shape, np.inf)
        np.minimum.at(cap, (siy[sel], six[sel]), sz[sel])
        lowered = np.isfinite(cap)
        self.height[lowered] = np.maximum(self.hit_height[lowered],
                                          np.minimum(self.height[lowered], cap[lowered]))
