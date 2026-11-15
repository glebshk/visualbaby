"""Flight map: the minimum safe altitude for every cell (the multi-altitude core).

``required_alt[cell]`` = tallest surface within ``clearance`` (horizontally)
plus ``vertical_clearance``, never below ``min_alt``. A cell is flyable when
that altitude is within ``max_alt`` and the cell keeps ``net_margin`` from the
cage net. Rain zones are a separate *soft* block: the drone may cross them only
to escape when one switches on around it.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np

from aufdeckung.config import Config
from aufdeckung.geometry import BoolArray, FloatArray
from aufdeckung.mapping.grid import GridSpec, HeightMap, cage_mask, max_filter_disk
from aufdeckung.scenario.model import RainZone


@dataclass
class FlightMap:
    spec: GridSpec
    required_alt: FloatArray  # inf where never flyable
    hard_ok: BoolArray  # flyable ignoring rain
    flyable: BoolArray  # flyable now
    cruise_alt: float
    min_alt: float
    max_alt: float

    def cell(self, x: float, y: float) -> tuple[int, int]:
        iy, ix = self.spec.index(x, y)
        return min(max(iy, 0), self.spec.ny - 1), min(max(ix, 0), self.spec.nx - 1)

    def transit_alt(self, iy: int, ix: int) -> float:
        """Altitude used when travelling: cruise altitude, raised where obstacles demand it."""
        return quantize_alt(max(float(self.required_alt[iy, ix]), self.cruise_alt))

    def is_flyable(self, x: float, y: float) -> bool:
        if not self.spec.contains(x, y):
            return False
        iy, ix = self.spec.index(x, y)
        return bool(self.flyable[iy, ix])

    def segment_profile(self, a: tuple[float, float], b: tuple[float, float], soft: bool = False
                        ) -> tuple[bool, float]:
        """(all cells along a->b flyable, max required altitude along a->b)."""
        n = max(int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.spec.res / 2))), 1)
        f = np.arange(n + 1) / n
        iy = np.floor((a[1] + (b[1] - a[1]) * f - self.spec.ymin) / self.spec.res).astype(np.int64)
        ix = np.floor((a[0] + (b[0] - a[0]) * f - self.spec.xmin) / self.spec.res).astype(np.int64)
        if iy.min() < 0 or ix.min() < 0 or iy.max() >= self.spec.ny or ix.max() >= self.spec.nx:
            return False, math.inf
        mask = self.hard_ok if soft else self.flyable
        ok = bool(mask[iy, ix].all())
        return ok, float(self.required_alt[iy, ix].max())


ALT_STEP = 0.25


def quantize_alt(z: float) -> float:
    """Round an altitude up to the next ``ALT_STEP`` so tiny climbs are never merged away."""
    if not math.isfinite(z):
        return z
    return math.ceil(z / ALT_STEP - 1e-9) * ALT_STEP


def build_flight_map(
    hm: HeightMap,
    cfg: Config,
    rain: Iterable[RainZone] = (),
    assume_unknown_below_cruise: bool = True,
) -> FlightMap:
    spec = hm.spec
    fl = cfg.flight
    height = hm.height
    if assume_unknown_below_cruise:
        # Unseen space is assumed clear only up to cruise level: the drone never
        # descends below cruise altitude over cells the lidar has not observed.
        height = np.where(hm.observed, height, np.maximum(height, fl.cruise_alt - fl.vertical_clearance))
    req = max_filter_disk(height, fl.clearance / spec.res) + fl.vertical_clearance
    req = np.maximum(req, fl.min_alt)
    inside = cage_mask(spec, cfg.arena, margin=cfg.arena.net_margin)
    hard_ok = inside & (req <= fl.max_alt + 1e-9)
    required_alt = np.where(hard_ok, req, np.inf)
    flyable = hard_ok & ~rain_mask(spec, rain, fl.clearance)
    return FlightMap(spec, required_alt, hard_ok, flyable, fl.cruise_alt, fl.min_alt, fl.max_alt)


def rain_mask(spec: GridSpec, rain: Iterable[RainZone], clearance: float) -> BoolArray:
    gx, gy = spec.centers()
    mask = np.zeros(spec.shape, dtype=bool)
    for zone in rain:
        mask |= np.hypot(gx - zone.x, gy - zone.y) <= zone.radius + clearance
    return mask
