from __future__ import annotations

import numpy as np
import pytest

from aufdeckung.config import Config
from aufdeckung.planning.mission import MissionController, Phase
from aufdeckung.scenario.model import RainZone, Scenario
from aufdeckung.sim.runner import run_mission


def test_requires_prior_when_not_exploring(cfg: Config) -> None:
    with pytest.raises(ValueError):
        MissionController(cfg.with_overrides({"mission": {"explore": False}}), (0.0, -18.0))


def test_first_setpoint_is_vertical_takeoff(cfg: Config) -> None:
    ctrl = MissionController(cfg, (0.0, -18.0))
    sp = ctrl.update(0.0, np.array([0.0, -18.0, 0.0]))
    assert sp.phase == Phase.TAKEOFF
    assert (sp.x, sp.y) == pytest.approx((0.0, -18.0), abs=0.3)
    assert sp.z >= cfg.flight.min_alt


def test_open_arena_mission_lifecycle(cfg: Config) -> None:
    result = run_mission(Scenario("open"), cfg)
    m = result.metrics
    phases = [e.detail for e in result.controller.events if e.kind == "phase"]
    assert phases[0] == "takeoff -> cover"
    assert phases[-1].endswith("-> done")
    assert m["completed"] and m["landed_home"]
    assert m["coverage"] > 0.98
    assert m["coverage_bonus"] > 0.95, "an open arena is covered inside the 3 minute bonus window"
    assert m["collisions"] == 0


def test_time_budget_forces_return(cfg: Config, simple_scenario: Scenario) -> None:
    short = cfg.with_overrides({"mission": {"time_budget": 70.0, "return_reserve": 10.0}})
    result = run_mission(simple_scenario, short)
    kinds = [e.kind for e in result.controller.events]
    assert "time_budget" in kinds
    assert result.metrics["landed_home"]
    assert result.metrics["mission_time"] <= 70.0 + 5.0


def test_rain_over_route_is_avoided_and_revisited(cfg: Config) -> None:
    sc = Scenario("rain", rain_zones=(RainZone(0.0, 0.0, 4.0, t_on=10.0, t_off=150.0),))
    result = run_mission(sc, cfg)
    m = result.metrics
    assert m["rain_exposure_s"] <= 5.0, "drone leaves a rain zone that switches on around it"
    assert m["coverage"] > 0.97, "the rain area is imaged after the rain stops"
    assert m["collisions"] == 0


def test_rain_switching_on_over_drone_triggers_escape(cfg: Config) -> None:
    sc = Scenario("rain_start", rain_zones=(RainZone(0.0, -12.0, 3.0, t_on=6.0, t_off=60.0),))
    result = run_mission(sc, cfg)
    assert result.metrics["rain_exposure_s"] <= 5.0
    assert result.metrics["landed_home"]


def test_hold_is_left_as_soon_as_a_way_home_exists(cfg: Config) -> None:
    # Regression (first Gazebo flight): after a failed return the drone hovered in HOLD forever.
    ctrl = MissionController(cfg, (0.0, -18.0))
    ctrl.phase = Phase.HOLD
    sp = ctrl.update(20.0, np.array([0.0, -10.0, 8.0]))
    assert ctrl.phase in (Phase.RETURN, Phase.LAND)
    assert (sp.x, sp.y) != (0.0, -10.0) or sp.z != 8.0  # it is heading somewhere, not holding
    assert any(e.kind == "phase" and "hold ->" in e.detail for e in ctrl.events)


def test_phantoms_at_flight_height_around_home_keep_home_flyable(cfg: Config) -> None:
    # Regression (Gazebo flights): misplaced facade hits at flight height near the take-off pad got
    # the +2 m top-beam margin, the home cell became "not flyable", and the drone hovered in HOLD.
    home = (0.0, -18.0)
    ctrl = MissionController(cfg, home)
    ring = np.array([[home[0] + r * np.cos(a), home[1] + r * np.sin(a), 8.1]
                     for r in (0.6, 1.2, 1.8) for a in np.linspace(0, 2 * np.pi, 24, endpoint=False)])
    ctrl.update(0.0, np.array([home[0], home[1], 0.0]))
    ctrl.update(2.0, np.array([home[0], home[1], 8.0]), (ring, np.ones(len(ring), dtype=bool)))
    ctrl.update(3.5, np.array([home[0], home[1], 8.0]))  # rebuild the flight map
    iy, ix = ctrl.fm.cell(*home)
    assert ctrl.fm.hard_ok[iy, ix]
    assert ctrl.phase != Phase.HOLD
