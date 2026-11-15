from __future__ import annotations

import numpy as np

from aufdeckung.config import Config
from aufdeckung.geometry import Box
from aufdeckung.mapping.grid import GridSpec, HeightMap
from aufdeckung.planning.flightmap import build_flight_map
from aufdeckung.planning.paths import astar, reachable, smooth


def test_astar_straight_and_around_wall() -> None:
    free = np.ones((20, 20), dtype=bool)
    path = astar(free, (0, 0), (0, 19))
    assert path is not None and len(path) == 20
    free[:15, 10] = False
    path = astar(free, (0, 0), (0, 19))
    assert path is not None
    assert all(free[c] for c in path)
    assert any(c[0] >= 15 for c in path)


def test_astar_no_path_and_blocked_endpoints() -> None:
    free = np.ones((10, 10), dtype=bool)
    free[:, 5] = False
    assert astar(free, (0, 0), (0, 9)) is None
    assert astar(free, (0, 5), (0, 9)) is None
    assert astar(free, (3, 3), (3, 3)) == [(3, 3)]


def test_no_corner_cutting() -> None:
    free = np.ones((3, 3), dtype=bool)
    free[0, 1] = False
    free[1, 0] = False
    assert astar(free, (0, 0), (1, 1)) is None
    assert not reachable(free, (0, 0))[1, 1]


def test_reachable_component() -> None:
    free = np.ones((10, 10), dtype=bool)
    free[:, 5] = False
    r = reachable(free, (0, 0))
    assert r[:, :5].all() and not r[:, 5:].any()
    assert not reachable(free, (0, 5)).any()
    assert not reachable(free, (-1, 0)).any()


def test_smooth_shortcuts_without_climbing(cfg: Config, spec: GridSpec) -> None:
    hm = HeightMap.from_prior(spec, cfg.arena, [Box(0.0, 0.0, 3.0, 3.0, 8.0)])
    fm = build_flight_map(hm, cfg, (), assume_unknown_below_cruise=False)
    straight = [(-10.0 + i, -6.0) for i in range(21)]
    out = smooth(fm, straight)
    assert out == [straight[0], straight[-1]]
    over = [(-10.0 + i, 0.0) for i in range(21)]
    out = smooth(fm, over)
    assert len(out) > 2, "must not shortcut across a building that needs a higher altitude"
    assert smooth(fm, over[:2]) == over[:2]
