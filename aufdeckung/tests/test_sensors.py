from __future__ import annotations

import math

import numpy as np
import pytest

from aufdeckung.config import Config, LidarConfig
from aufdeckung.geometry import Box
from aufdeckung.mapping.grid import GridSpec, rasterize
from aufdeckung.sensors.camera import visible_cells, visible_mask
from aufdeckung.sensors.lidar import LidarSensor


def test_open_field_footprint(cfg: Config, spec: GridSpec) -> None:
    h = np.zeros(spec.shape)
    mask = visible_mask((0.0, 0.0, 8.0), h, spec, cfg.camera)
    hx, hy = cfg.camera.half_extents(8.0)
    expected = (2 * hx) * (2 * hy) / spec.res**2
    assert mask.sum() == pytest.approx(expected, rel=0.1)


def test_wall_occludes_cells_behind(cfg: Config, spec: GridSpec) -> None:
    h = rasterize([Box(2.0, 0.0, 0.5, 10.0, 7.0)], spec)
    mask = visible_mask((0.0, 0.0, 8.0), h, spec, cfg.camera)
    assert mask[spec.index(1.0, 0.0)]
    assert not mask[spec.index(4.0, 0.0)], "ground right behind a 7 m wall is hidden"
    assert mask[spec.index(-4.0, 0.0)]


def test_roof_is_imaged(cfg: Config, spec: GridSpec) -> None:
    h = rasterize([Box(0.0, 0.0, 4.0, 4.0, 4.0)], spec)
    mask = visible_mask((0.0, 0.0, 8.0), h, spec, cfg.camera)
    assert mask[spec.index(0.5, 0.5)]
    assert mask.sum() < visible_mask((0.0, 0.0, 8.0), np.zeros(spec.shape), spec, cfg.camera).sum()


def test_too_close_or_too_high(cfg: Config, spec: GridSpec) -> None:
    h = np.zeros(spec.shape)
    assert len(visible_cells((0.0, 0.0, 0.5), h, spec, cfg.camera)[0]) == 0
    assert len(visible_cells((0.0, 0.0, 12.0), h, spec, cfg.camera)[0]) == 0
    assert len(visible_cells((300.0, 0.0, 8.0), h, spec, cfg.camera)[0]) == 0


def test_lidar_sees_wall_and_ground(cfg: Config) -> None:
    sensor = LidarSensor(LidarConfig(noise_std=0.0))
    wall = Box(10.0, 0.0, 1.0, 20.0, 12.0)
    pts, top = sensor.scan(np.array([0.0, 0.0, 5.0]), [wall])
    assert len(pts) > 0
    ahead = pts[np.abs(pts[:, 1]) < 1.0]
    assert ahead[:, 0].max() == pytest.approx(9.5, abs=1e-6)
    ground = pts[np.abs(pts[:, 2]) < 1e-6]
    assert len(ground) > 0
    assert top.any()
    assert (pts[top][:, 2] > 5.0).all(), "top beam points up (+10 deg)"


def test_lidar_noise_and_range(cfg: Config) -> None:
    sensor = LidarSensor(LidarConfig(noise_std=0.05, max_range=6.0))
    pts, _ = sensor.scan(np.array([0.0, 0.0, 5.0]), [], np.random.default_rng(0))
    r = np.linalg.norm(pts - np.array([0.0, 0.0, 5.0]), axis=1)
    assert r.max() <= 6.0 + 0.3
    assert math.isclose(np.abs(pts[:, 2]).max(), 0.0, abs_tol=0.3)
