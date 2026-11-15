"""Shared assertions for planner safety invariants."""

from __future__ import annotations

import math

from aufdeckung.config import Config
from aufdeckung.planning.coverage_planner import Waypoint
from aufdeckung.planning.flightmap import FlightMap


def assert_route_safe(wps: list[Waypoint], start: tuple[float, float, float], fm: FlightMap,
                      cfg: Config, allow_escape: bool = False) -> None:
    """Every horizontal leg stays in flyable cells and at/above the required altitude."""
    prev = start
    for i, w in enumerate(wps):
        assert w.z <= cfg.flight.max_alt + 1e-6, f"wp {i} above max_alt: {w}"
        horiz = math.hypot(w.x - prev[0], w.y - prev[1])
        if horiz > 1e-6:
            assert abs(w.z - prev[2]) < 1e-6 or i == 0, f"leg {i} is not level: {prev} -> {w}"
            soft = allow_escape and w.kind == "escape"
            ok, peak = fm.segment_profile((prev[0], prev[1]), (w.x, w.y), soft=soft)
            assert ok, f"leg {i} leaves the flyable area: {prev} -> {w}"
            assert peak <= w.z + 1e-6, f"leg {i} too low: needs {peak:.2f} m, flies {w.z:.2f} m"
        else:
            assert w.z >= fm.required_alt[fm.cell(w.x, w.y)] - 1e-6 or w.kind == "land"
        prev = w.xyz
