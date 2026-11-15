from __future__ import annotations

import numpy as np
import pytest

from aufdeckung.config import Config
from aufdeckung.geometry import Box
from aufdeckung.mapping.coverage import CoverageMap
from aufdeckung.mapping.grid import (
    UNSEEN_TOP_MARGIN,
    GridSpec,
    HeightMap,
    cage_mask,
    dilate,
    max_filter_disk,
    rasterize,
)


def test_grid_indices(spec: GridSpec) -> None:
    assert spec.shape == (90, 90)
    iy, ix = spec.index(0.1, 0.1)
    assert spec.center(iy, ix) == pytest.approx((0.25, 0.25))
    assert not spec.contains(100.0, 0.0)
    iy_a, ix_a = spec.indices(np.array([-22.4, 22.4]), np.array([0.0, 0.0]))
    assert spec.valid(iy_a, ix_a).all()


def test_cage_mask(spec: GridSpec, cfg: Config) -> None:
    full = cage_mask(spec, cfg.arena)
    inner = cage_mask(spec, cfg.arena, margin=1.5)
    assert inner.sum() < full.sum()
    area = full.sum() * spec.res**2
    assert area == pytest.approx(1567, rel=0.03)  # regular 16-gon, circumradius 22.5 m


def test_rasterize_takes_tallest(spec: GridSpec) -> None:
    h = rasterize([Box(0, 0, 2, 2, 3.0), Box(0, 0, 1, 1, 7.0)], spec)
    assert h.max() == 7.0
    assert h[spec.index(1.2, 1.2)] == 3.0
    assert h[spec.index(5.0, 5.0)] == 0.0


def test_max_filter_and_dilate() -> None:
    v = np.zeros((9, 9))
    v[4, 4] = 5.0
    out = max_filter_disk(v, 2.0)
    assert out[4, 6] == 5.0 and out[4, 7] == 0.0 and out[6, 6] == 0.0
    assert dilate(v > 0, 1.0).sum() == 5


def test_heightmap_integrates_hits(spec: GridSpec, cfg: Config) -> None:
    hm = HeightMap(spec, cfg.arena)
    pts = np.array([[1.0, 1.0, 4.0], [1.0, 1.0, 2.0], [3.0, 3.0, 0.05], [80.0, 0.0, 3.0]])
    grown = hm.integrate_hits(pts, sensor_z=8.0)
    assert grown == 1
    assert hm.height[spec.index(1.0, 1.0)] == 4.0
    assert hm.height[spec.index(3.0, 3.0)] == 0.0  # ground noise suppressed
    assert hm.observed[spec.index(3.0, 3.0)]
    assert hm.observed.sum() == 2


def test_top_beam_hits_raise_unseen_tops(spec: GridSpec, cfg: Config) -> None:
    hm = HeightMap(spec, cfg.arena)
    pts = np.array([[5.0, 0.0, 8.5], [6.0, 0.0, 8.5]])
    hm.integrate_hits(pts, sensor_z=8.0, top_beam=np.array([True, False]))
    assert hm.height[spec.index(5.0, 0.0)] == pytest.approx(8.5 + UNSEEN_TOP_MARGIN)
    assert hm.height[spec.index(6.0, 0.0)] == pytest.approx(8.5)


def test_empty_hits(spec: GridSpec, cfg: Config) -> None:
    assert HeightMap(spec, cfg.arena).integrate_hits(np.zeros((0, 3)), 5.0) == 0


def test_coverage_map(spec: GridSpec) -> None:
    cov = CoverageMap(spec)
    a = np.zeros(spec.shape, dtype=bool)
    a[0, :10] = True
    assert cov.mark(a, 1.0) == 10
    assert cov.mark(a, 2.0) == 0
    assert cov.first_seen[0, 0] == 1.0
    of = np.zeros(spec.shape, dtype=bool)
    of[0, :20] = True
    assert cov.fraction(of) == 0.5
    assert cov.fraction_at(of, 0.5) == 0.0
    assert cov.fraction(np.zeros(spec.shape, dtype=bool)) == 1.0
