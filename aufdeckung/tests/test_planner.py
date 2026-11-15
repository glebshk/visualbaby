from __future__ import annotations

import numpy as np
import pytest
from helpers import assert_route_safe

from aufdeckung.config import Config
from aufdeckung.geometry import Box
from aufdeckung.mapping.grid import GridSpec, HeightMap, cage_mask
from aufdeckung.planning.coverage_planner import (
    PlanningError,
    Waypoint,
    estimate_time,
    plan_coverage,
    plan_transit,
    to_waypoints,
)
from aufdeckung.planning.flightmap import build_flight_map
from aufdeckung.scenario.generator import PRESETS, generate
from aufdeckung.scenario.model import RainZone


def _prior(cfg: Config, spec: GridSpec, shapes, rain=()):  # type: ignore[no-untyped-def]
    hm = HeightMap.from_prior(spec, cfg.arena, shapes)
    return hm, build_flight_map(hm, cfg, rain, assume_unknown_below_cruise=False)


def test_empty_arena_full_predicted_coverage(cfg: Config, spec: GridSpec) -> None:
    hm, fm = _prior(cfg, spec, [])
    need = cage_mask(spec, cfg.arena)
    plan = plan_coverage((0.0, -18.0, 0.0), hm, fm, need, cfg)
    assert plan.predicted[need].mean() > 0.99
    assert plan.est_time < cfg.mission.bonus_window, "open arena fits the 3 minute bonus window"
    assert_route_safe(plan.waypoints, (0.0, -18.0, 0.0), fm, cfg)
    assert plan.waypoints[0].x == pytest.approx(0.0) and plan.waypoints[0].y == pytest.approx(-18.0)


@pytest.mark.parametrize("preset", sorted(PRESETS))
def test_routes_respect_required_altitude(preset: str, cfg: Config, spec: GridSpec) -> None:
    for seed in range(3):
        sc = generate(preset, seed)
        hm, fm = _prior(cfg, spec, sc.obstacles)
        start = (sc.start[0], sc.start[1], 0.0)
        plan = plan_coverage(start, hm, fm, cage_mask(spec, cfg.arena), cfg)
        assert plan.waypoints
        assert_route_safe(plan.waypoints, start, fm, cfg)


def test_multi_altitude_climbs_over_low_building(cfg: Config, spec: GridSpec) -> None:
    hm, fm = _prior(cfg, spec, [Box(0.0, 0.0, 10.0, 10.0, 7.5)])
    plan = plan_coverage((0.0, -18.0, 0.0), hm, fm, cage_mask(spec, cfg.arena), cfg)
    over = [w for w in plan.waypoints if abs(w.x) < 4 and abs(w.y) < 4]
    assert over, "the roof is part of the area to image"
    assert min(w.z for w in over) >= 7.5 + cfg.flight.vertical_clearance - 1e-6
    assert min(w.z for w in plan.waypoints if w.z > 1) <= cfg.flight.cruise_alt + 1e-6


def test_low_viewpoints_used_in_narrow_alleys(cfg: Config, spec: GridSpec) -> None:
    # 6 m wide alley between ceiling-high walls: only a narrow strip keeps 1.5 m clearance.
    alley = [Box(-5.5, 0.0, 5.0, 30.0, 13.0, name="w"), Box(5.5, 0.0, 5.0, 30.0, 13.0, name="e")]
    hm, fm = _prior(cfg, spec, alley)
    assert fm.flyable[fm.cell(0.0, 5.0)]
    need = np.zeros(spec.shape, dtype=bool)
    for y in (5.0, -5.0):
        iy, ix = spec.index(0.0, y)
        need[iy - 1 : iy + 2, ix - 1 : ix + 2] = True
    for axes in (("x",), ("y",), ("x", "y")):
        plan = plan_coverage((0.0, -18.0, 0.0), hm, fm, need, cfg, axes=axes)
        assert plan.predicted[need].all(), axes
    assert_route_safe(plan.waypoints, (0.0, -18.0, 0.0), fm, cfg)


def test_too_narrow_alley_is_not_entered(cfg: Config, spec: GridSpec) -> None:
    alley = [Box(-4.5, 0.0, 5.0, 30.0, 13.0, name="w"), Box(4.5, 0.0, 5.0, 30.0, 13.0, name="e")]
    hm, fm = _prior(cfg, spec, alley)
    assert not fm.flyable[fm.cell(0.0, 5.0)]
    plan = plan_coverage((0.0, -18.0, 0.0), hm, fm, cage_mask(spec, cfg.arena), cfg)
    assert_route_safe(plan.waypoints, (0.0, -18.0, 0.0), fm, cfg)


def test_nothing_needed(cfg: Config, spec: GridSpec) -> None:
    hm, fm = _prior(cfg, spec, [])
    plan = plan_coverage((0.0, 0.0, 5.0), hm, fm, np.zeros(spec.shape, dtype=bool), cfg)
    assert plan.waypoints == [] and plan.est_time == 0.0


def test_escape_from_rain(cfg: Config, spec: GridSpec) -> None:
    hm, fm = _prior(cfg, spec, [], (RainZone(0.0, 0.0, 3.0),))
    plan = plan_coverage((0.0, 0.0, 8.0), hm, fm, cage_mask(spec, cfg.arena), cfg)
    assert plan.waypoints[0].kind == "escape"
    first_dry = next(w for w in plan.waypoints if w.kind != "escape")
    assert np.hypot(first_dry.x, first_dry.y) > 3.0
    assert_route_safe(plan.waypoints, (0.0, 0.0, 8.0), fm, cfg, allow_escape=True)


def test_trapped_raises(cfg: Config, spec: GridSpec) -> None:
    ring = [Box(0.0, 6.0, 14.0, 2.0, 13.0, name="n"), Box(0.0, -6.0, 14.0, 2.0, 13.0, name="s"),
            Box(6.0, 0.0, 2.0, 14.0, 13.0, name="e"), Box(-6.0, 0.0, 2.0, 14.0, 13.0, name="w")]
    _hm, fm = _prior(cfg, spec, ring)
    with pytest.raises(PlanningError):
        plan_transit((0.0, 0.0, 8.0), (0.0, -18.0), fm)


def test_plan_transit_home(cfg: Config, spec: GridSpec) -> None:
    _hm, fm = _prior(cfg, spec, [Box(0.0, -8.0, 16.0, 3.0, 13.0)])
    wps = plan_transit((0.0, 5.0, 8.0), (0.0, -18.0), fm)
    assert (wps[-1].x, wps[-1].y) == (0.0, -18.0)
    assert_route_safe(wps, (0.0, 5.0, 8.0), fm, cfg)


def test_to_waypoints_climbs_before_and_descends_after(cfg: Config, spec: GridSpec) -> None:
    _hm, fm = _prior(cfg, spec, [Box(0.0, 0.0, 4.0, 4.0, 7.0)])
    wps = to_waypoints([(-8.0, 0.0, 8.0, "a"), (8.0, 0.0, 8.0, "b")], fm)
    assert [w.kind for w in wps] == ["a", "climb", "b", "b"]
    assert wps[1].z == pytest.approx(8.5) and wps[-1].z == 8.0
    assert to_waypoints([], fm) == []


def test_estimate_time(cfg: Config) -> None:
    wps = [Waypoint(0.0, 0.0, 8.0), Waypoint(30.0, 0.0, 8.0)]
    t = estimate_time(wps, (0.0, 0.0, 0.0), cfg.flight)
    assert t > 30.0 / cfg.flight.max_speed + 8.0 / cfg.flight.max_climb
