from __future__ import annotations

import math

import numpy as np
import pytest

from aufdeckung.geometry import (
    Box,
    Cylinder,
    _ray_box,
    _ray_boxes,
    distance_to_polygon_edges,
    distance_to_shape,
    footprint_mask,
    inside_with_margin,
    points_in_polygon,
    ray_cast,
    regular_polygon,
)


def _dir(*v: float) -> np.ndarray:
    d = np.array([v], dtype=float)
    return d / np.linalg.norm(d)


def test_polygon_contains_and_margin() -> None:
    poly = regular_polygon(45.0, 16)
    assert len(poly) == 16
    px, py = np.array([0.0, 22.6, 21.0]), np.array([0.0, 0.0, 0.0])
    assert points_in_polygon(px, py, poly).tolist() == [True, False, True]
    assert inside_with_margin(px, py, poly, 1.5).tolist() == [True, False, False]
    inradius = 22.5 * math.cos(math.pi / 16)
    assert distance_to_polygon_edges(np.array([0.0]), np.array([0.0]), poly)[0] == pytest.approx(inradius)


def test_shape_validation() -> None:
    with pytest.raises(ValueError):
        Box(0, 0, 0, 1, 1)
    with pytest.raises(ValueError):
        Cylinder(0, 0, 1, 1, z0=-1)


def test_footprint_mask_rotated_box() -> None:
    b = Box(0.0, 0.0, 4.0, 1.0, 2.0, yaw=math.pi / 2)
    assert footprint_mask(b, np.array([0.0]), np.array([1.9]))[0]
    assert not footprint_mask(b, np.array([1.9]), np.array([0.0]))[0]
    assert footprint_mask(b, np.array([0.6]), np.array([0.0]), pad=0.2)[0]


def test_distance_to_shape() -> None:
    b = Box(0.0, 0.0, 2.0, 2.0, 3.0)
    assert distance_to_shape(b, np.array([0.0, 0.0, 1.0])) == 0.0
    assert distance_to_shape(b, np.array([3.0, 0.0, 1.0])) == pytest.approx(2.0)
    assert distance_to_shape(b, np.array([0.0, 0.0, 5.0])) == pytest.approx(2.0)
    c = Cylinder(0.0, 0.0, 1.0, 2.0, z0=1.0)
    assert distance_to_shape(c, np.array([0.0, 0.0, 0.0])) == pytest.approx(1.0)
    assert distance_to_shape(c, np.array([4.0, 0.0, 1.5])) == pytest.approx(3.0)


def test_ray_hits_ground() -> None:
    t = ray_cast(np.array([0.0, 0.0, 5.0]), _dir(0, 0, -1), [], 30.0)
    assert t[0] == pytest.approx(5.0)
    assert math.isinf(ray_cast(np.array([0.0, 0.0, 5.0]), _dir(1, 0, 0.1), [], 30.0)[0])


def test_ray_box_axis_aligned_and_rotated() -> None:
    o = np.array([0.0, 0.0, 1.0])
    box = Box(5.0, 0.0, 2.0, 2.0, 3.0)
    assert ray_cast(o, _dir(1, 0, 0), [box], 30.0)[0] == pytest.approx(4.0)
    rot = Box(5.0, 0.0, 2.0, 2.0, 3.0, yaw=math.pi / 4)
    assert ray_cast(o, _dir(1, 0, 0), [rot], 30.0)[0] == pytest.approx(5.0 - math.sqrt(2), abs=1e-6)


def test_ray_over_box_misses_and_max_range() -> None:
    o = np.array([0.0, 0.0, 5.0])
    box = Box(5.0, 0.0, 2.0, 2.0, 3.0)
    assert math.isinf(ray_cast(o, _dir(1, 0, 0), [box], 30.0)[0])
    assert math.isinf(ray_cast(np.array([0.0, 0.0, 1.0]), _dir(1, 0, 0), [box], 3.0)[0])


def test_ray_parallel_to_face_and_from_inside() -> None:
    box = Box(5.0, 0.0, 2.0, 2.0, 3.0)
    along_face = ray_cast(np.array([0.0, 1.0, 1.0]), _dir(1, 0, 0), [box], 30.0)[0]
    assert along_face == pytest.approx(4.0)
    assert _ray_box(np.array([5.0, 0.0, 1.0]), _dir(1, 0, 0), box)[0] == 0.0


def test_ray_cylinder_side_and_cap() -> None:
    cyl = Cylinder(5.0, 0.0, 1.0, 2.0)
    assert ray_cast(np.array([0.0, 0.0, 1.0]), _dir(1, 0, 0), [cyl], 30.0)[0] == pytest.approx(4.0)
    assert ray_cast(np.array([5.0, 0.0, 6.0]), _dir(0, 0, -1), [cyl], 30.0)[0] == pytest.approx(4.0)
    crown = Cylinder(0.0, 0.0, 1.0, 1.0, z0=3.0)
    t = ray_cast(np.array([0.0, 0.0, 1.0]), _dir(0, 0, 1), [crown], 30.0)[0]
    assert t == pytest.approx(2.0)


def test_batched_boxes_match_single() -> None:
    rng = np.random.default_rng(1)
    dirs = rng.normal(size=(500, 3))
    dirs /= np.linalg.norm(dirs, axis=1)[:, None]
    boxes = [Box(*rng.uniform(-8, 8, 2), *rng.uniform(1, 4, 2), rng.uniform(1, 10),
                 yaw=float(rng.uniform(0, 3))) for _ in range(12)]
    o = np.array([0.3, -0.2, 4.0])
    single = np.min([_ray_box(o, dirs, b) for b in boxes], axis=0)
    np.testing.assert_allclose(_ray_boxes(o, dirs, boxes), single)
