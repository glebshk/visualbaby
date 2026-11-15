from __future__ import annotations

import math

import pytest

from aufdeckung.config import (
    ArenaConfig,
    CameraConfig,
    Config,
    FlightConfig,
    LidarConfig,
    MissionConfig,
    PlannerConfig,
    SimConfig,
)


def test_defaults_follow_regelwerk(cfg: Config) -> None:
    assert cfg.arena.diameter == 45.0
    assert cfg.arena.height == 13.0
    assert cfg.flight.max_alt == 10.0
    assert cfg.flight.max_speed <= 10.0
    assert cfg.mission.time_budget == 600.0
    assert cfg.mission.bonus_window == 180.0


def test_camera_footprint(cfg: Config) -> None:
    hx, hy = cfg.camera.half_extents(8.0)
    assert hx == pytest.approx(8.0 * math.tan(1.204 / 2))
    assert hy < hx
    assert cfg.camera.vfov < cfg.camera.hfov


@pytest.mark.parametrize(
    "factory",
    [
        lambda: FlightConfig(max_speed=11.0),
        lambda: FlightConfig(cruise_alt=11.0),
        lambda: FlightConfig(min_alt=9.0, cruise_alt=8.0),
        lambda: FlightConfig(clearance=0.3, drone_radius=0.35),
        lambda: FlightConfig(max_climb=0.0),
        lambda: ArenaConfig(diameter=-1.0),
        lambda: ArenaConfig(n_sides=2),
        lambda: ArenaConfig(net_margin=30.0),
        lambda: CameraConfig(hfov=4.0),
        lambda: CameraConfig(min_agl=11.0),
        lambda: LidarConfig(h_samples=0),
        lambda: LidarConfig(v_min_deg=20.0, v_max_deg=10.0),
        lambda: LidarConfig(min_range=40.0),
        lambda: PlannerConfig(overlap=1.0),
        lambda: PlannerConfig(grid_res=0.0),
        lambda: MissionConfig(time_budget=0.0),
        lambda: SimConfig(dt=0.0),
        lambda: Config(arena=ArenaConfig(height=9.0)),
    ],
)
def test_invalid_configs_raise(factory: object) -> None:
    with pytest.raises(ValueError):
        factory()  # type: ignore[operator]


def test_with_overrides_nested(cfg: Config) -> None:
    new = cfg.with_overrides({"flight": {"cruise_alt": 7.0}, "mission": {"explore": False}})
    assert new.flight.cruise_alt == 7.0
    assert new.mission.explore is False
    assert cfg.flight.cruise_alt == 8.0  # original untouched


def test_with_overrides_rejects_unknown_key(cfg: Config) -> None:
    with pytest.raises(KeyError):
        cfg.with_overrides({"flight": {"warp_speed": 1}})


def test_with_overrides_validates(cfg: Config) -> None:
    with pytest.raises(ValueError):
        cfg.with_overrides({"flight": {"max_speed": 25.0}})
