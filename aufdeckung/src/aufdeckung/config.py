"""Configuration dataclasses. Defaults follow the Drone Challenge Brunswick Regelwerk 2027."""

from __future__ import annotations

import math
from dataclasses import dataclass, field, fields, is_dataclass, replace
from typing import Any

# Regelwerk 2027, section 5.1 / 5.3 / 6.5
CAGE_DIAMETER_M = 45.0
CAGE_HEIGHT_M = 13.0
RECOMMENDED_ALTITUDE_LIMIT_M = 10.0
MAX_ALLOWED_SPEED_MS = 10.0
TASK1_DURATION_S = 600.0
TASK1_BONUS_WINDOW_S = 180.0
DETECTION_RADIUS_M = 1.0  # "Durchmesser von 2 Metern"


@dataclass(frozen=True)
class ArenaConfig:
    """Cage geometry (regular polygon approximating the DLR net cage)."""

    diameter: float = CAGE_DIAMETER_M
    height: float = CAGE_HEIGHT_M
    n_sides: int = 16
    net_margin: float = 1.5
    origin_lat: float = 52.3110  # placeholder, replace with the surveyed cage centre
    origin_lon: float = 10.5600
    origin_alt: float = 80.0

    def __post_init__(self) -> None:
        if self.diameter <= 0 or self.height <= 0:
            raise ValueError("arena diameter and height must be positive")
        if self.n_sides < 3:
            raise ValueError("arena needs at least 3 sides")
        if not 0 <= self.net_margin < self.diameter / 2:
            raise ValueError("net_margin must be in [0, radius)")


@dataclass(frozen=True)
class CameraConfig:
    """Nadir camera. Defaults match the OakD-Lite IMX214 model in simulation/OakD-Lite_down."""

    hfov: float = 1.204
    aspect: float = 4.0 / 3.0
    min_agl: float = 1.0
    max_detect_agl: float = 10.0
    rate_hz: float = 5.0

    @property
    def vfov(self) -> float:
        return 2.0 * math.atan(math.tan(self.hfov / 2.0) / self.aspect)

    def half_extents(self, agl: float) -> tuple[float, float]:
        """Half footprint (along x, along y) on a surface ``agl`` metres below the camera."""
        return agl * math.tan(self.hfov / 2.0), agl * math.tan(self.vfov / 2.0)

    def __post_init__(self) -> None:
        if not 0 < self.hfov < math.pi:
            raise ValueError("hfov must be in (0, pi)")
        if self.aspect <= 0 or self.rate_hz <= 0:
            raise ValueError("aspect and rate_hz must be positive")
        if not 0 <= self.min_agl < self.max_detect_agl:
            raise ValueError("need 0 <= min_agl < max_detect_agl")


@dataclass(frozen=True)
class LidarConfig:
    """Spinning 3D lidar, tilted down so it sees both walls and the ground."""

    h_samples: int = 360
    v_min_deg: float = -60.0
    v_max_deg: float = 10.0
    v_samples: int = 16
    min_range: float = 0.2
    max_range: float = 30.0
    noise_std: float = 0.02
    rate_hz: float = 2.0

    def __post_init__(self) -> None:
        if self.h_samples < 1 or self.v_samples < 1:
            raise ValueError("lidar needs at least one sample per axis")
        if not self.v_min_deg <= self.v_max_deg:
            raise ValueError("v_min_deg must be <= v_max_deg")
        if not 0 <= self.min_range < self.max_range:
            raise ValueError("need 0 <= min_range < max_range")


@dataclass(frozen=True)
class FlightConfig:
    """Vehicle limits and safety margins."""

    cruise_alt: float = 8.0
    min_alt: float = 4.0
    max_alt: float = RECOMMENDED_ALTITUDE_LIMIT_M
    max_speed: float = 3.0
    max_climb: float = 1.5
    max_accel: float = 2.0
    clearance: float = 1.5
    vertical_clearance: float = 1.5
    drone_radius: float = 0.35
    waypoint_tolerance: float = 0.3

    def __post_init__(self) -> None:
        if not 0 < self.min_alt <= self.cruise_alt <= self.max_alt:
            raise ValueError("need 0 < min_alt <= cruise_alt <= max_alt")
        if not 0 < self.max_speed <= MAX_ALLOWED_SPEED_MS:
            raise ValueError(f"max_speed must be in (0, {MAX_ALLOWED_SPEED_MS}] m/s (Regelwerk 5.3)")
        if self.max_climb <= 0 or self.max_accel <= 0:
            raise ValueError("max_climb and max_accel must be positive")
        if self.clearance <= self.drone_radius:
            raise ValueError("clearance must exceed the drone radius")
        if self.vertical_clearance <= 0 or self.waypoint_tolerance <= 0:
            raise ValueError("vertical_clearance and waypoint_tolerance must be positive")


@dataclass(frozen=True)
class PlannerConfig:
    """Coverage planner tuning."""

    grid_res: float = 0.5
    overlap: float = 0.2
    sample_step: float = 1.0
    gapfill_step: float = 1.5
    gapfill_min_gain: int = 4
    replan_min_interval: float = 1.0

    def __post_init__(self) -> None:
        if self.grid_res <= 0 or self.sample_step <= 0 or self.gapfill_step <= 0:
            raise ValueError("resolutions and steps must be positive")
        if not 0 <= self.overlap < 1:
            raise ValueError("overlap must be in [0, 1)")


@dataclass(frozen=True)
class MissionConfig:
    """Mission timing (Flugaufgabe 1)."""

    time_budget: float = TASK1_DURATION_S
    bonus_window: float = TASK1_BONUS_WINDOW_S
    return_reserve: float = 30.0
    explore: bool = True

    def __post_init__(self) -> None:
        if self.time_budget <= 0 or self.return_reserve < 0:
            raise ValueError("time_budget must be positive and return_reserve >= 0")


@dataclass(frozen=True)
class SimConfig:
    """Headless simulator settings."""

    dt: float = 0.1
    gnss_std: float = 0.02
    wind_std: float = 0.15
    detection_error_std: float = 0.25
    seed: int = 0

    def __post_init__(self) -> None:
        if self.dt <= 0:
            raise ValueError("dt must be positive")


@dataclass(frozen=True)
class Config:
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    flight: FlightConfig = field(default_factory=FlightConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
    sim: SimConfig = field(default_factory=SimConfig)

    def __post_init__(self) -> None:
        if self.flight.max_alt >= self.arena.height:
            raise ValueError("max_alt must stay below the cage ceiling")

    def with_overrides(self, overrides: dict[str, Any]) -> Config:
        """Return a copy with nested overrides, e.g. ``{"flight": {"max_speed": 4.0}}``."""
        return _apply(self, overrides)


def _apply(obj: Any, overrides: dict[str, Any]) -> Any:
    names = {f.name for f in fields(obj)}
    changes: dict[str, Any] = {}
    for key, value in overrides.items():
        if key not in names:
            raise KeyError(f"unknown config key {key!r} for {type(obj).__name__}")
        current = getattr(obj, key)
        if is_dataclass(current) and isinstance(value, dict):
            changes[key] = _apply(current, value)
        else:
            changes[key] = value
    return replace(obj, **changes)
