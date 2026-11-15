"""Seeded random arena layouts.

The real layout is only known on competition day, so the planner is validated
across many random layouts drawn from these presets. Every layout is fully
reproducible from ``(preset, seed)``.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from aufdeckung.config import ArenaConfig
from aufdeckung.geometry import Box, Cylinder, Shape, inside_with_margin, regular_polygon
from aufdeckung.scenario.model import START_CLEAR_RADIUS, RainZone, Scenario, Target

GROUNDS = ("grass", "asphalt", "gravel", "sand")


@dataclass
class _Layout:
    arena: ArenaConfig
    rng: np.random.Generator
    start: tuple[float, float]
    shapes: list[Shape] = field(default_factory=list)
    discs: list[tuple[float, float, float]] = field(default_factory=list)
    counter: dict[str, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.poly = regular_polygon(self.arena.diameter, self.arena.n_sides)
        self.inradius = self.arena.diameter / 2 * math.cos(math.pi / self.arena.n_sides)

    def label(self, kind: str) -> str:
        self.counter[kind] = self.counter.get(kind, 0) + 1
        return f"{kind}_{self.counter[kind]}"

    def free(self, x: float, y: float, r: float, gap: float, wall_gap: float = 0.3) -> bool:
        if not inside_with_margin(np.array([x]), np.array([y]), self.poly, r + wall_gap)[0]:
            return False
        if math.hypot(x - self.start[0], y - self.start[1]) < r + START_CLEAR_RADIUS + 0.5:
            return False
        return all(math.hypot(x - ox, y - oy) >= r + orad + gap for ox, oy, orad in self.discs)

    def random_point(self, r: float, radius_max: float | None = None) -> tuple[float, float]:
        rmax = (radius_max if radius_max is not None else self.inradius) - r
        rho = math.sqrt(self.rng.uniform(0, 1)) * max(rmax, 0.0)
        phi = self.rng.uniform(0, 2 * math.pi)
        return rho * math.cos(phi), rho * math.sin(phi)

    def place(
        self,
        make: Callable[[float, float], list[Shape]],
        radius: float,
        gap: float,
        tries: int = 60,
        point: Callable[[], tuple[float, float]] | None = None,
    ) -> bool:
        for _ in range(tries):
            x, y = point() if point else self.random_point(radius)
            if self.free(x, y, radius, gap):
                self.shapes.extend(make(x, y))
                self.discs.append((x, y, radius))
                return True
        return False

    # ----------------------------------------------------------------- catalogue

    def building(self, w: float, d: float, h: float, gap: float = 3.0,
                 yaw: float | None = None, point: Callable[[], tuple[float, float]] | None = None,
                 ) -> bool:
        yaw = self.rng.uniform(0, math.pi) if yaw is None else yaw
        h = min(h, self.arena.height)
        name = self.label("building" if h < self.arena.height else "tower")

        def make(x: float, y: float) -> list[Shape]:
            return [Box(x, y, w, d, h, yaw=yaw, kind="building", name=name)]

        return self.place(make, 0.5 * math.hypot(w, d), gap, point=point)

    def vehicle(self, gap: float = 1.0, point: Callable[[], tuple[float, float]] | None = None,
                yaw: float | None = None) -> bool:
        kind, (w, d, h) = [("car", (4.3, 1.8, 1.5)), ("van", (5.2, 2.0, 2.3)),
                           ("truck", (7.5, 2.5, 3.3))][int(self.rng.choice(3, p=[0.65, 0.25, 0.1]))]
        yaw = self.rng.uniform(0, math.pi) if yaw is None else yaw
        name = self.label(kind)
        return self.place(lambda x, y: [Box(x, y, w, d, h, yaw=yaw, kind="vehicle", name=name)],
                          0.5 * math.hypot(w, d), gap, point=point)

    def tree(self, gap: float = 1.0, point: Callable[[], tuple[float, float]] | None = None) -> bool:
        trunk_h = self.rng.uniform(1.8, 3.0)
        crown_r = self.rng.uniform(1.0, 2.2)
        crown_h = self.rng.uniform(2.0, 4.5)
        name = self.label("tree")

        def make(x: float, y: float) -> list[Shape]:
            return [Cylinder(x, y, self.rng.uniform(0.12, 0.25), trunk_h, kind="tree", name=name + "_trunk"),
                    Cylinder(x, y, crown_r, crown_h, z0=trunk_h, kind="tree", name=name + "_crown")]

        return self.place(make, crown_r, gap, point=point)

    def pole(self, gap: float = 0.8, point: Callable[[], tuple[float, float]] | None = None) -> bool:
        kind, r, h = [("lamp", 0.1, self.rng.uniform(4.0, 6.0)),
                      ("sign", 0.05, self.rng.uniform(2.2, 3.0))][int(self.rng.integers(2))]
        name = self.label(kind)
        return self.place(lambda x, y: [Cylinder(x, y, r, h, kind="pole", name=name)], 0.3, gap,
                          point=point)

    def street_furniture(self, gap: float = 0.8) -> bool:
        kind, (w, d, h) = [("bench", (1.8, 0.6, 0.9)), ("bin", (0.6, 0.6, 1.1)),
                           ("bus_stop", (4.0, 1.5, 2.6)), ("container", (6.0, 2.4, 2.6)),
                           ("fence", (float(self.rng.uniform(4, 9)), 0.15, float(self.rng.uniform(1, 2)))),
                           ("hedge", (float(self.rng.uniform(2, 6)), 0.8, 1.2))][int(self.rng.integers(6))]
        yaw = self.rng.uniform(0, math.pi)
        name = self.label(kind)
        return self.place(lambda x, y: [Box(x, y, w, d, h, yaw=yaw, kind="furniture", name=name)],
                          0.5 * math.hypot(w, d), gap)

    def feature(self, x: float = 0.0, y: float = 0.0) -> bool:
        """Central eye-catcher (e.g. the waterfall in Abb. 3)."""
        w, d, h = self.rng.uniform(2.5, 4.0), self.rng.uniform(0.8, 1.5), self.rng.uniform(5.0, 9.0)
        name = self.label("feature")
        return self.place(lambda px, py: [Box(px, py, w, d, h, yaw=self.rng.uniform(0, math.pi),
                                              kind="feature", name=name)],
                          0.5 * math.hypot(w, d), 2.0, tries=5, point=lambda: (x, y))


# --------------------------------------------------------------------------- presets


def _fixed(p: tuple[float, float]) -> Callable[[], tuple[float, float]]:
    """Placement callback that always proposes the same point."""
    return lambda: p


def _height(rng: np.random.Generator, arena: ArenaConfig, p_tall: float) -> float:
    if rng.uniform() < p_tall:
        return float(rng.choice([rng.uniform(9.0, 12.0), arena.height]))
    return float(rng.uniform(3.0, 8.5))


def _sparse(lay: _Layout) -> None:
    rng = lay.rng
    for _ in range(int(rng.integers(2, 5))):
        lay.building(rng.uniform(4, 8), rng.uniform(4, 8), rng.uniform(3, 7), gap=4.0)
    for _ in range(int(rng.integers(4, 9))):
        lay.tree()
    for _ in range(int(rng.integers(2, 5))):
        lay.vehicle()
    for _ in range(int(rng.integers(2, 6))):
        lay.pole()


def _urban_ring(lay: _Layout) -> None:
    rng = lay.rng
    n = int(rng.integers(6, 10))
    offset = rng.uniform(0, 2 * math.pi)
    for i in range(n):
        phi = offset + 2 * math.pi * i / n + rng.uniform(-0.15, 0.15)
        rho = rng.uniform(13.0, 16.5)
        lay.building(rng.uniform(4, 7), rng.uniform(4, 7), _height(rng, lay.arena, 0.45), gap=2.5,
                     yaw=phi, point=_fixed((rho * math.cos(phi), rho * math.sin(phi))))
    lay.feature(rng.uniform(-2, 2), rng.uniform(-2, 2))
    for _ in range(int(rng.integers(4, 8))):
        lay.tree(point=lambda: lay.random_point(1.5, 11.0))
    for _ in range(int(rng.integers(2, 5))):
        lay.vehicle(point=lambda: lay.random_point(2.5, 11.0))
    for _ in range(int(rng.integers(2, 5))):
        lay.street_furniture()


def _dense_urban(lay: _Layout) -> None:
    rng = lay.rng
    for _ in range(int(rng.integers(10, 17))):
        lay.building(rng.uniform(3.5, 8), rng.uniform(3.5, 8), _height(rng, lay.arena, 0.35),
                     gap=float(rng.uniform(2.5, 5.0)))
    for _ in range(int(rng.integers(3, 7))):
        lay.vehicle()
    for _ in range(int(rng.integers(3, 7))):
        lay.pole()
    for _ in range(int(rng.integers(1, 4))):
        lay.tree()


def _center_tower(lay: _Layout) -> None:
    rng = lay.rng
    size = rng.uniform(7, 12)
    lay.building(size, size * rng.uniform(0.7, 1.3), lay.arena.height, gap=3.0,
                 point=lambda: (rng.uniform(-2, 2), rng.uniform(-2, 2)))
    _sparse(lay)


def _street(lay: _Layout) -> None:
    rng = lay.rng
    yaw = rng.uniform(0, math.pi / 2)
    c, s = math.cos(yaw), math.sin(yaw)
    half_street = rng.uniform(3.0, 4.5)

    def rot(u: float, v: float) -> tuple[float, float]:
        return c * u - s * v, s * u + c * v

    for qu in (-1, 1):
        for qv in (-1, 1):
            for _ in range(int(rng.integers(1, 3))):
                w, d = rng.uniform(4, 8), rng.uniform(4, 8)
                u = qu * (half_street + 1.0 + w / 2 + rng.uniform(0, 8))
                v = qv * (half_street + 1.0 + d / 2 + rng.uniform(0, 8))
                lay.building(w, d, _height(rng, lay.arena, 0.3), gap=2.5, yaw=yaw,
                             point=_fixed(rot(u, v)))
    for _ in range(int(rng.integers(3, 8))):
        along = rng.uniform(-18, 18)
        side = rng.choice([-1.0, 1.0]) * (half_street - 1.2)
        horizontal = bool(rng.integers(2))
        u, v = (along, side) if horizontal else (side, along)
        lay.vehicle(point=_fixed(rot(u, v)), yaw=yaw if horizontal else yaw + math.pi / 2)
    for _ in range(int(rng.integers(4, 9))):
        along = rng.uniform(-18, 18)
        side = rng.choice([-1.0, 1.0]) * (half_street + 0.5)
        u, v = (along, side) if rng.integers(2) else (side, along)
        if rng.uniform() < 0.5:
            lay.pole(point=_fixed(rot(u, v)))
        else:
            lay.tree(point=_fixed(rot(u, v)))


def _mixed(lay: _Layout) -> None:
    rng = lay.rng
    for _ in range(int(rng.integers(4, 11))):
        lay.building(rng.uniform(3.5, 9), rng.uniform(3.5, 9), _height(rng, lay.arena, 0.3),
                     gap=float(rng.uniform(2.5, 6.0)))
    for _ in range(int(rng.integers(2, 7))):
        lay.tree()
    for _ in range(int(rng.integers(1, 6))):
        lay.vehicle()
    for _ in range(int(rng.integers(1, 6))):
        lay.pole()
    for _ in range(int(rng.integers(0, 4))):
        lay.street_furniture()


PRESETS: dict[str, Callable[[_Layout], None]] = {
    "sparse": _sparse,
    "urban_ring": _urban_ring,
    "dense_urban": _dense_urban,
    "center_tower": _center_tower,
    "street": _street,
    "mixed": _mixed,
}


def _targets(lay: _Layout, n: int) -> list[Target]:
    rng = lay.rng
    targets: list[Target] = []
    classes = ["special"] + [str(rng.choice(["class1", "class2"], p=[0.4, 0.6])) for _ in range(n - 1)]
    for i, cls in enumerate(classes):
        for _ in range(200):
            x, y = lay.random_point(0.5)
            if not inside_with_margin(np.array([x]), np.array([y]), lay.poly, 1.0)[0]:
                continue
            if any(_covers(sh, x, y, 0.4) for sh in lay.shapes):
                continue
            if any(math.hypot(x - t.x, y - t.y) < 2.5 for t in targets):
                continue
            if math.hypot(x - lay.start[0], y - lay.start[1]) < START_CLEAR_RADIUS:
                continue
            size = 0.6 if cls == "special" else float(rng.uniform(0.3, 0.8))
            targets.append(Target(_r(x, 2), _r(y, 2), cls=cls, name=f"{cls}_{i}", size=_r(size, 2)))
            break
    return targets


def _covers(shape: Shape, x: float, y: float, pad: float) -> bool:
    from aufdeckung.geometry import footprint_mask

    return bool(footprint_mask(shape, np.array([x]), np.array([y]), pad=pad)[0])


def _rain(lay: _Layout, n: int) -> list[RainZone]:
    rng = lay.rng
    zones: list[RainZone] = []
    for _ in range(n):
        for _ in range(100):
            r = float(rng.uniform(2.0, 4.0))
            x, y = lay.random_point(r)
            if math.hypot(x - lay.start[0], y - lay.start[1]) < r + START_CLEAR_RADIUS + 1.0:
                continue
            t_on = float(rng.uniform(20.0, 300.0))
            zones.append(RainZone(_r(x, 2), _r(y, 2), _r(r, 2), _r(t_on, 1),
                                  _r(t_on + float(rng.uniform(60.0, 150.0)), 1)))
            break
    return zones


def generate(
    preset: str,
    seed: int,
    arena: ArenaConfig | None = None,
    n_targets: tuple[int, int] = (6, 14),
    rain: bool = True,
) -> Scenario:
    """Generate a valid random scenario for ``preset`` and ``seed``."""
    if preset not in PRESETS:
        raise KeyError(f"unknown preset {preset!r}; choose from {sorted(PRESETS)}")
    arena = arena or ArenaConfig()
    rng = np.random.default_rng([seed, sorted(PRESETS).index(preset)])
    start = (0.0, -float(arena.diameter / 2 * math.cos(math.pi / arena.n_sides) - arena.net_margin - 2.5))
    lay = _Layout(arena=arena, rng=rng, start=start)
    PRESETS[preset](lay)
    targets = _targets(lay, int(rng.integers(n_targets[0], n_targets[1] + 1)))
    zones = _rain(lay, int(rng.integers(0, 3))) if rain else []
    shapes = tuple(_round_shape(s) for s in lay.shapes)
    scenario = Scenario(
        name=f"{preset}_{seed}",
        obstacles=shapes,
        targets=tuple(targets),
        rain_zones=tuple(zones),
        start=start,
        preset=preset,
        seed=seed,
        ground=str(rng.choice(GROUNDS)),
    )
    scenario.validate(arena)
    return scenario


def _r(v: float, nd: int = 3) -> float:
    return round(float(v), nd)


def _round_shape(s: Shape) -> Shape:
    """Plain python floats, rounded, so scenarios serialise cleanly and compare exactly."""
    if isinstance(s, Box):
        return Box(_r(s.x), _r(s.y), _r(s.sx), _r(s.sy), _r(s.height), yaw=_r(s.yaw, 4), z0=_r(s.z0),
                   kind=s.kind, name=s.name)
    return Cylinder(_r(s.x), _r(s.y), _r(s.radius), _r(s.height), z0=_r(s.z0), kind=s.kind,
                    name=s.name)
