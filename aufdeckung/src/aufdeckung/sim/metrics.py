"""Ground-truth evaluation helpers."""

from __future__ import annotations

import math

import numpy as np

from aufdeckung.config import Config
from aufdeckung.geometry import BoolArray, distance_to_polygon_edges, regular_polygon
from aufdeckung.mapping.grid import GridSpec, HeightMap, cage_mask
from aufdeckung.planning.flightmap import build_flight_map
from aufdeckung.planning.paths import reachable
from aufdeckung.scenario.model import Scenario
from aufdeckung.sensors.camera import visible_cells


def observable_mask(scenario: Scenario, cfg: Config, step: float = 1.0) -> BoolArray:
    """Cells inside the cage that some safe viewpoint reachable from the start can image.

    This is the oracle denominator for coverage: cells under tree crowns, inside
    ceiling-high buildings or in boxed-in courtyards are excluded.
    """
    spec = GridSpec.for_arena(cfg.arena, cfg.planner.grid_res)
    hm = HeightMap.from_prior(spec, cfg.arena, scenario.obstacles)
    fm = build_flight_map(hm, cfg, (), assume_unknown_below_cruise=False)
    reach = reachable(fm.flyable, fm.cell(*scenario.start))
    stride = max(round(step / spec.res), 1)
    lattice = np.zeros(spec.shape, dtype=bool)
    lattice[::stride, ::stride] = True
    seen = np.zeros(spec.shape, dtype=bool)
    for iy, ix in np.argwhere(lattice & reach):
        x, y = spec.center(int(iy), int(ix))
        req = float(fm.required_alt[iy, ix])
        for z in {max(req, cfg.flight.cruise_alt), max(req, cfg.flight.min_alt)}:
            vy, vx = visible_cells((x, y, z), hm.height, spec, cfg.camera)
            seen[vy, vx] = True
    result: BoolArray = seen & cage_mask(spec, cfg.arena)
    return result


def net_distance(x: float, y: float, cfg: Config) -> float:
    poly = regular_polygon(cfg.arena.diameter, cfg.arena.n_sides)
    return float(distance_to_polygon_edges(np.array([x]), np.array([y]), poly)[0])


def summarize(values: list[float]) -> dict[str, float]:
    if not values:
        return {"mean": math.nan, "min": math.nan, "max": math.nan}
    arr = np.array(values, dtype=float)
    return {"mean": float(arr.mean()), "min": float(arr.min()), "max": float(arr.max())}
