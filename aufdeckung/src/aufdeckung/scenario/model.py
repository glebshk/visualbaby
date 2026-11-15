"""Scenario data model and validation."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from aufdeckung.config import ArenaConfig
from aufdeckung.geometry import (
    Box,
    Shape,
    distance_to_shape,
    footprint_mask,
    inside_with_margin,
    points_in_polygon,
    regular_polygon,
)

TARGET_CLASSES = ("class1", "class2", "special")
START_CLEAR_RADIUS = 3.0


class ScenarioError(ValueError):
    """Raised when a scenario violates the arena rules."""


@dataclass(frozen=True)
class Target:
    """Object to find (Flugaufgabe 1). Lies on the ground; small enough to fly over."""

    x: float
    y: float
    cls: str = "class2"
    name: str = ""
    size: float = 0.5

    def __post_init__(self) -> None:
        if self.cls not in TARGET_CLASSES:
            raise ScenarioError(f"unknown target class {self.cls!r}, expected one of {TARGET_CLASSES}")
        if self.size <= 0:
            raise ScenarioError("target size must be positive")


@dataclass(frozen=True)
class RainZone:
    """Vertical rain curtain (no-fly cylinder, ground to ceiling) active in [t_on, t_off)."""

    x: float
    y: float
    radius: float
    t_on: float = 0.0
    t_off: float = math.inf

    def __post_init__(self) -> None:
        if self.radius <= 0:
            raise ScenarioError("rain zone radius must be positive")
        if not self.t_on < self.t_off:
            raise ScenarioError("rain zone needs t_on < t_off")

    def active(self, t: float) -> bool:
        return self.t_on <= t < self.t_off


@dataclass(frozen=True)
class Scenario:
    name: str
    obstacles: tuple[Shape, ...] = ()
    targets: tuple[Target, ...] = ()
    rain_zones: tuple[RainZone, ...] = ()
    start: tuple[float, float] = (0.0, -18.0)
    preset: str = "custom"
    seed: int | None = None
    ground: str = "grass"
    notes: dict[str, str] = field(default_factory=dict)

    def active_rain(self, t: float) -> tuple[RainZone, ...]:
        return tuple(z for z in self.rain_zones if z.active(t))

    def validate(self, arena: ArenaConfig) -> None:
        """Raise :class:`ScenarioError` listing every rule violation."""
        errors = self.problems(arena)
        if errors:
            raise ScenarioError(f"scenario {self.name!r} is invalid:\n  - " + "\n  - ".join(errors))

    def problems(self, arena: ArenaConfig) -> list[str]:
        poly = regular_polygon(arena.diameter, arena.n_sides)
        errors: list[str] = []
        sx, sy = self.start
        if not inside_with_margin(np.array([sx]), np.array([sy]), poly, arena.net_margin)[0]:
            errors.append(f"start {self.start} is outside the cage or within the net margin")
        for i, obs in enumerate(self.obstacles):
            label = obs.name or f"{obs.kind}#{i}"
            if obs.top > arena.height + 1e-6:
                errors.append(f"{label} is taller than the cage ({obs.top:.1f} m)")
            px, py = _outline(obs)
            if not points_in_polygon(px, py, poly).all():
                errors.append(f"{label} extends outside the cage")
            if distance_to_shape(obs, np.array([sx, sy, obs.z0])) < START_CLEAR_RADIUS:
                errors.append(f"{label} blocks the start pad")
        for i, tgt in enumerate(self.targets):
            label = tgt.name or f"target#{i}"
            if not points_in_polygon(np.array([tgt.x]), np.array([tgt.y]), poly)[0]:
                errors.append(f"{label} is outside the cage")
            for obs in self.obstacles:
                if footprint_mask(obs, np.array([tgt.x]), np.array([tgt.y]))[0]:
                    errors.append(f"{label} is inside or under {obs.name or obs.kind}")
                    break
        for i, zone in enumerate(self.rain_zones):
            if math.hypot(zone.x - sx, zone.y - sy) < zone.radius + START_CLEAR_RADIUS:
                errors.append(f"rain zone #{i} covers the start pad")
        return errors


def _outline(obs: Shape, n: int = 16) -> tuple[np.ndarray, np.ndarray]:
    if isinstance(obs, Box):
        c, s = math.cos(obs.yaw), math.sin(obs.yaw)
        lx = np.array([-1, 1, 1, -1]) * obs.sx / 2
        ly = np.array([-1, -1, 1, 1]) * obs.sy / 2
        return obs.x + c * lx - s * ly, obs.y + s * lx + c * ly
    ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return obs.x + obs.radius * np.cos(ang), obs.y + obs.radius * np.sin(ang)
