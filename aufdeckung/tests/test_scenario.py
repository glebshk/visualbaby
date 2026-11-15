from __future__ import annotations

import math
from pathlib import Path

import pytest

from aufdeckung.config import ArenaConfig, Config
from aufdeckung.geometry import Box, Cylinder
from aufdeckung.scenario.generator import PRESETS, generate
from aufdeckung.scenario.io import load_scenario, save_scenario, scenario_from_dict, scenario_to_dict
from aufdeckung.scenario.model import RainZone, Scenario, ScenarioError, Target


@pytest.mark.parametrize("preset", sorted(PRESETS))
def test_presets_generate_valid_layouts(preset: str, cfg: Config) -> None:
    for seed in range(8):
        sc = generate(preset, seed, cfg.arena)
        assert sc.problems(cfg.arena) == []
        assert sc.obstacles, "every preset places obstacles"
        assert 6 <= len(sc.targets) <= 14
        assert sum(t.cls == "special" for t in sc.targets) == 1


def test_generation_is_deterministic() -> None:
    assert generate("mixed", 5) == generate("mixed", 5)
    assert generate("mixed", 5) != generate("mixed", 6)
    assert generate("mixed", 5) != generate("dense_urban", 5)


def test_rain_can_be_disabled() -> None:
    assert all(not generate("mixed", s, rain=False).rain_zones for s in range(10))


def test_some_layouts_need_altitude_changes_or_detours() -> None:
    tall = [o for s in range(10) for o in generate("dense_urban", s).obstacles
            if o.kind == "building" and o.top > 8.5]
    assert tall, "dense_urban must contain buildings too tall to fly over"


def test_unknown_preset() -> None:
    with pytest.raises(KeyError):
        generate("moon", 0)


def test_validation_catches_rule_violations(cfg: Config) -> None:
    bad = Scenario(
        name="bad",
        obstacles=(Box(30.0, 0.0, 2, 2, 3, name="outside"), Box(0.0, -17.0, 2, 2, 3, name="on_pad"),
                   Box(0.0, 0.0, 2, 2, 20.0, name="too_tall")),
        targets=(Target(0.0, 0.0, "class1", "inside_tall"), Target(40.0, 0.0, "class1", "far")),
        rain_zones=(RainZone(0.0, -16.0, 2.0),),
    )
    problems = "\n".join(bad.problems(cfg.arena))
    for needle in ("outside extends outside", "on_pad blocks the start pad", "too_tall is taller",
                   "inside_tall is inside", "far is outside", "rain zone #0"):
        assert needle in problems
    with pytest.raises(ScenarioError):
        bad.validate(cfg.arena)


def test_start_outside_cage(cfg: Config) -> None:
    assert any("start" in p for p in Scenario("s", start=(0.0, 22.0)).problems(cfg.arena))


def test_target_and_rain_validation() -> None:
    with pytest.raises(ScenarioError):
        Target(0, 0, "class9")
    with pytest.raises(ScenarioError):
        Target(0, 0, "class1", size=0)
    with pytest.raises(ScenarioError):
        RainZone(0, 0, 1.0, t_on=5.0, t_off=5.0)
    with pytest.raises(ScenarioError):
        RainZone(0, 0, 0.0)
    zone = RainZone(0, 0, 1.0, 10.0, 20.0)
    assert not zone.active(9.9) and zone.active(10.0) and not zone.active(20.0)


def test_yaml_roundtrip(tmp_path: Path) -> None:
    sc = generate("urban_ring", 3)
    sc = Scenario(**{**sc.__dict__, "rain_zones": (*sc.rain_zones, RainZone(5.0, 5.0, 2.0, 10.0))})
    path = tmp_path / "sc.yaml"
    save_scenario(sc, path)
    loaded = load_scenario(path)
    assert loaded == sc
    assert math.isinf(loaded.rain_zones[-1].t_off)


def test_dict_roundtrip_keeps_shape_types() -> None:
    sc = Scenario("x", obstacles=(Box(1, 2, 3, 4, 5, yaw=0.5, name="b"),
                                  Cylinder(1, 2, 0.5, 3, z0=1.0, name="c")))
    again = scenario_from_dict(scenario_to_dict(sc))
    assert again.obstacles == sc.obstacles


@pytest.mark.parametrize("data", [
    {"obstacles": [{"type": "pyramid", "x": 0, "y": 0, "height": 1}]},
    {"obstacles": [{"type": "box", "x": 0, "y": 0}]},
    {"targets": [{"x": "a", "y": 0}]},
])
def test_malformed_data(data: dict[str, object]) -> None:
    with pytest.raises(ScenarioError):
        scenario_from_dict(data)


def test_load_rejects_non_mapping(tmp_path: Path) -> None:
    p = tmp_path / "list.yaml"
    p.write_text("- 1\n- 2\n")
    with pytest.raises(ScenarioError):
        load_scenario(p)


def test_custom_arena_size() -> None:
    small = ArenaConfig(diameter=30.0)
    sc = generate("sparse", 0, small)
    assert sc.problems(small) == []
