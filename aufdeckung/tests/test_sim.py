from __future__ import annotations

import math

import numpy as np
import pytest

from aufdeckung.config import Config
from aufdeckung.scenario.generator import PRESETS, generate
from aufdeckung.scenario.model import Scenario
from aufdeckung.sim.benchmark import Criteria, format_summary, run_benchmark
from aufdeckung.sim.drone import KinematicDrone
from aufdeckung.sim.metrics import observable_mask, summarize
from aufdeckung.sim.runner import run_mission


def test_drone_respects_limits(cfg: Config) -> None:
    d = KinematicDrone(cfg.flight, (0.0, 0.0, 0.0))
    speeds = []
    for _ in range(400):
        d.step((30.0, 0.0, 8.0), 0.05)
        speeds.append(float(np.hypot(*d.vel[:2])))
        assert d.vel[2] <= cfg.flight.max_climb + 1e-9
    assert max(speeds) <= cfg.flight.max_speed + 1e-9
    assert np.linalg.norm(d.pos - np.array([30.0, 0.0, 8.0])) < cfg.flight.waypoint_tolerance


def test_drone_does_not_sink_below_ground(cfg: Config) -> None:
    d = KinematicDrone(cfg.flight, (0.0, 0.0, 0.5))
    for _ in range(100):
        d.step((0.0, 0.0, -5.0), 0.1)
    assert d.pos[2] == 0.0


def test_observable_excludes_hidden_cells(cfg: Config, simple_scenario: Scenario) -> None:
    obs = observable_mask(simple_scenario, cfg)
    from aufdeckung.mapping.grid import GridSpec

    spec = GridSpec.for_arena(cfg.arena, cfg.planner.grid_res)
    assert not obs[spec.index(8.0, 2.0)], "inside the ceiling-high tower"
    assert obs[spec.index(-8.0, 4.0)], "roof of the low building"
    assert obs[spec.index(0.0, 0.0)]


def test_simple_scenario_both_modes(cfg: Config, simple_scenario: Scenario) -> None:
    for prior in (False, True):
        m = run_mission(simple_scenario, cfg, use_prior=prior).metrics
        assert m["collisions"] == 0
        assert m["min_clearance"] >= Criteria().min_clearance
        assert m["coverage"] >= 0.95
        assert m["targets_found"] == m["targets_observable"] == 3
        assert m["landed_home"]
        assert m["mission_time"] <= cfg.mission.time_budget


def test_run_is_reproducible(cfg: Config, simple_scenario: Scenario) -> None:
    a = run_mission(simple_scenario, cfg).metrics
    b = run_mission(simple_scenario, cfg).metrics
    assert a == b


def test_reports_have_small_georeference_error(cfg: Config, simple_scenario: Scenario) -> None:
    result = run_mission(simple_scenario, cfg)
    assert all(r.error < 2.0 for r in result.reports)
    assert all(math.isfinite(r.t_found) for r in result.reports)


def test_summarize() -> None:
    assert summarize([1.0, 3.0]) == {"mean": 2.0, "min": 1.0, "max": 3.0}
    assert math.isnan(summarize([])["mean"])


def test_benchmark_report(cfg: Config) -> None:
    report = run_benchmark(["street"], [0], cfg, workers=1)
    assert report["per_preset"]["street"]["runs"] == 1
    text = format_summary(report)
    assert "street" in text and ("PASS" in text or "FAIL" in text)


def test_criteria_failures(cfg: Config) -> None:
    m = {"collisions": 1, "min_clearance": 0.2, "coverage_time_budget": 0.5, "completed": False,
         "landed_home": False, "mission_time": 700.0}
    assert len(Criteria().failures(m, cfg)) == 5


@pytest.mark.slow
@pytest.mark.parametrize("preset", sorted(PRESETS))
def test_random_layouts_meet_acceptance_criteria(preset: str, cfg: Config) -> None:
    for seed in (100, 101):
        m = run_mission(generate(preset, seed), cfg).metrics
        assert not Criteria().failures(m, cfg), (preset, seed, m)
