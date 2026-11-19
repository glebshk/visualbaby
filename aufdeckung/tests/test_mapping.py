from __future__ import annotations

import numpy as np
import pytest

from aufdeckung.config import Config
from aufdeckung.geometry import Box, Cylinder
from aufdeckung.mapping.coverage import CoverageMap
from aufdeckung.mapping.grid import (
    TOP_MARGIN_RANGE_M,
    UNSEEN_TOP_MARGIN,
    GridSpec,
    HeightMap,
    cage_mask,
    dilate,
    max_filter_disk,
    rasterize,
)
from aufdeckung.sensors.lidar import LidarSensor


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



def test_top_beam_margin_only_near_the_sensor(spec: GridSpec, cfg: Config) -> None:
    # Regression (Gazebo flight 6): far top-beam hits are where attitude errors move points most; a net
    # hit misplaced 1.8 m inwards got the +2 m margin and became a 10 m wall that trapped the drone.
    # A far wall is seen again from closer, where the margin applies, before the drone gets there.
    hm = HeightMap(spec, cfg.arena)
    near = [TOP_MARGIN_RANGE_M - 1.0, 0.0, 8.2]
    far = [0.0, TOP_MARGIN_RANGE_M + 4.0, 8.2]
    hm.integrate_hits(np.array([near, far]), sensor_z=8.0, top_beam=np.array([True, True]),
                      origin=(0.0, 0.0, 8.0))
    assert hm.height[spec.index(near[0], near[1])] == pytest.approx(8.2 + UNSEEN_TOP_MARGIN)
    assert hm.height[spec.index(far[0], far[1])] == pytest.approx(8.2)

def _scan_from(hm: HeightMap, cfg: Config, shapes: list, positions: list[tuple[float, float, float]],
               carve: bool = True) -> None:  # type: ignore[type-arg]
    lidar = LidarSensor(cfg.lidar)
    for pos in positions:
        p = np.array(pos)
        pts, top = lidar.scan(p, shapes)
        hm.integrate_hits(pts, sensor_z=pos[2], top_beam=top, origin=pos if carve else None)


def test_carving_removes_margin_on_roof_just_above_the_drone(spec: GridSpec, cfg: Config) -> None:
    # Regression (benchmark street_13): an 8.3 m roof seen from 8 m cruise altitude was stored
    # as 10.3 m for good, so its 1.5 m clearance made the roof unflyable and it was never imaged.
    # Only beams that hit something carry free-space evidence; in a street the beams passing over
    # the roof edge end on the taller building behind it.
    roof = [Box(8.0, 0.0, 6.0, 6.0, 8.3, name="house"), Box(8.0, 9.0, 16.0, 4.0, 12.0, name="behind")]
    # Along the street at the 1.5 m minimum clearance from the wall (face at y = -3), as in street_13.
    # Later the drone sees the same edge from the far side of the street: beams now pass over it.
    path = [(x, -4.6, 7.9) for x in (0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0)]
    path += [(x, y, 8.0) for y in (-7.5, -8.5) for x in (2.0, 5.0, 8.0, 11.0, 14.0)]
    stuck = HeightMap(spec, cfg.arena)
    _scan_from(stuck, cfg, roof, path, carve=False)
    carved = HeightMap(spec, cfg.arena)
    _scan_from(carved, cfg, roof, path)
    region = (slice(*[spec.index(0.0, -3.0)[0], spec.index(0.0, 3.0)[0] + 1]),
              slice(*[spec.index(5.0, 0.0)[1], spec.index(11.0, 0.0)[1] + 1]))
    assert stuck.height[region].max() > 9.5  # the problem being fixed
    assert carved.height[region].max() < 8.3 + 0.75
    assert carved.height[region].max() >= 8.3 - 0.1  # never below what the lidar really hit


@pytest.mark.parametrize("shape", [Cylinder(6.0, 0.0, 0.15, 12.0, name="pole"),
                                   Box(6.0, 0.0, 0.3, 6.0, 12.0, name="wall")])
def test_carving_keeps_margin_on_obstacles_taller_than_the_beams(spec: GridSpec, cfg: Config,
                                                                 shape: Box | Cylinder) -> None:
    # Safety: beams pass beside a thin pole or over nothing at all near a tall wall, but the top
    # is above every beam. The guessed margin must survive, or the planner would overfly it.
    # The backdrop makes beams passing beside the pole return, i.e. bring (misleading) evidence.
    backdrop = Box(14.0, 0.0, 1.0, 30.0, 12.5, name="backdrop")
    hm = HeightMap(spec, cfg.arena)
    near_only = HeightMap(spec, cfg.arena)
    # Near views, then far views: at 18-22 m the 1 deg beams are ~35 cm apart and can miss the pole.
    near = [(x, y, 8.0) for x in (-4.0, 0.0, 2.0) for y in (-6.0, -2.0, 0.0, 2.0, 6.0)]
    far = [(x, y, 8.0) for x in (-16.0, -14.0, -12.0) for y in (-3.0, -1.1, 0.4, 1.7, 3.3)]
    _scan_from(hm, cfg, [shape, backdrop], near + far)
    # The margin applies to hits within TOP_MARGIN_RANGE_M of the drone (two "near" views are farther).
    _scan_from(near_only, cfg, [shape, backdrop],
               [v for v in near if np.hypot(v[0] - 6.0, v[1]) < TOP_MARGIN_RANGE_M])
    iy, ix = spec.index(6.0, 0.0)
    around = (slice(iy - 1, iy + 2), slice(ix - 1, ix + 2))
    # The top is above every beam, so the guessed margin above the highest real hit seen from close
    # up must stay, and no later view may carve it away. Far top-beam hits count as they are.
    # The map is capped at the cage ceiling.
    expected = min(near_only.hit_height[around].max() + UNSEEN_TOP_MARGIN, cfg.arena.height)
    assert hm.height[around].max() >= expected - 0.1
    assert hm.height[around].max() >= hm.hit_height[around].max() - 0.05


def _pole_with_margin(spec: GridSpec, cfg: Config, x: float) -> tuple[HeightMap, tuple[int, int]]:
    """A pole cell whose top-beam hit at 8.5 m was raised by the margin (top unseen)."""
    hm = HeightMap(spec, cfg.arena)
    hm.integrate_hits(np.array([[x, 0.0, 8.5]]), sensor_z=8.0, top_beam=np.array([True]))
    cell = spec.index(x, 0.0)
    assert hm.height[cell] == pytest.approx(8.5 + UNSEEN_TOP_MARGIN)
    return hm, cell


def test_carving_ignores_beams_below_this_scans_hits(spec: GridSpec, cfg: Config) -> None:
    # Beam A hits the pole at 8.9 m; beam B slips through the same cell beside the pole at ~8.8 m
    # and ends on a wall behind. B is no evidence that the pole ends below 8.8 m.
    hm, cell = _pole_with_margin(spec, cfg, 6.0)
    origin = (0.0, 0.0, 8.0)
    hit_pole = [6.0, 0.05, 8.9]
    slips_by = [12.0, 0.2, 8.0 + 12.0 * (0.8 / 6.0)]  # passes x = 6 at z = 8.8
    hm.integrate_hits(np.array([hit_pole, slips_by]), sensor_z=8.0, origin=origin)
    assert hm.height[cell] >= 8.9 + 1.0


def test_carving_ignores_far_beams(spec: GridSpec, cfg: Config) -> None:
    # 20 m away, 1 deg beams are 35 cm apart: one can pass a 30 cm pole without any beam hitting it.
    hm, cell = _pole_with_margin(spec, cfg, 0.0)
    origin = (-20.0, 0.0, 8.0)
    passes = [10.0, 0.0, 8.0 + 30.0 * (1.0 / 20.0)]  # passes x = 0 at z = 9.0
    hm.integrate_hits(np.array([passes]), sensor_z=8.0, origin=origin)
    assert hm.height[cell] == pytest.approx(8.5 + UNSEEN_TOP_MARGIN)


def test_carving_uses_beams_above_this_scans_hits(spec: GridSpec, cfg: Config) -> None:
    # The useful case: a beam clears the edge at 8.7 m while the edge itself was hit at 8.3 m.
    hm, cell = _pole_with_margin(spec, cfg, 6.0)
    origin = (0.0, 0.0, 8.0)
    over = [12.0, 0.0, 8.0 + 12.0 * (0.7 / 6.0)]  # passes x = 6 at z = 8.7
    hm.integrate_hits(np.array([over]), sensor_z=8.0, origin=origin)
    assert hm.height[cell] == pytest.approx(8.7, abs=0.15)


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


def test_known_free_space_caps_phantoms_but_keeps_real_lower_hits(spec: GridSpec, cfg: Config) -> None:
    hm = HeightMap(spec, cfg.arena)
    hm.mark_free(0.0, 0.0, 7.4, 0.35)  # the drone hovered here at 8 m
    cell = spec.index(0.0, 0.0)
    # A top-beam phantom at flight height (would become 10 m with the margin) and a real roof below.
    hm.integrate_hits(np.array([[0.0, 0.0, 8.05]]), sensor_z=8.0, top_beam=np.array([True]))
    assert hm.height[cell] == pytest.approx(7.4)
    hm.integrate_hits(np.array([[0.1, 0.1, 5.0]]), sensor_z=8.0)
    assert hm.height[cell] == pytest.approx(7.4)  # capped, and the 5 m roof is below the cap
    other = spec.index(3.0, 0.0)
    hm.integrate_hits(np.array([[3.0, 0.0, 5.0]]), sensor_z=8.0)
    assert hm.height[other] == pytest.approx(5.0)  # outside known free space nothing changes


def test_mark_free_lowers_what_was_already_mapped(spec: GridSpec, cfg: Config) -> None:
    hm = HeightMap(spec, cfg.arena)
    hm.integrate_hits(np.array([[0.0, -18.0, 9.0]]), sensor_z=8.0, top_beam=np.array([True]))
    hm.mark_free(0.0, -18.0, 0.0, 2.5)  # take-off pad
    assert hm.height[spec.index(0.0, -18.0)] == 0.0 and hm.hit_height[spec.index(0.0, -18.0)] == 0.0


def test_a_single_scan_cannot_raise_a_cell_when_confirmation_is_on(spec: GridSpec, cfg: Config) -> None:
    # Gazebo flight 6: lidar data was clean (~0 % phantoms per scan), yet at 0.1 % over a whole
    # flight thousands of stray top-beam points (+2 m margin) blocked cells and trapped the planner.
    hm = HeightMap(spec, cfg.arena, confirm_scans=2)
    cell = spec.index(5.0, 0.0)
    stray = np.array([[5.0, 0.0, 8.4]])
    hm.integrate_hits(stray, sensor_z=8.0, top_beam=np.array([True]))
    assert hm.height[cell] == 0.0  # one scan: not believed
    for _ in range(6):  # later scans see nothing there: the proposal expires
        hm.integrate_hits(np.array([[-5.0, 0.0, 0.0]]), sensor_z=8.0)
    hm.integrate_hits(stray, sensor_z=8.0, top_beam=np.array([True]))
    assert hm.height[cell] == 0.0  # too long ago to count as confirmation
    wall = np.array([[5.0, 0.0, 6.0], [5.1, 0.1, 5.5]])
    hm.integrate_hits(wall, sensor_z=8.0)
    hm.integrate_hits(wall, sensor_z=8.0)
    assert hm.height[cell] == pytest.approx(6.0)  # two scans agree: mapped


def test_confirmation_takes_the_height_both_scans_support(spec: GridSpec, cfg: Config) -> None:
    hm = HeightMap(spec, cfg.arena, confirm_scans=2)
    cell = spec.index(5.0, 0.0)
    hm.integrate_hits(np.array([[5.0, 0.0, 9.0]]), sensor_z=8.0)
    hm.integrate_hits(np.array([[5.0, 0.0, 6.0]]), sensor_z=8.0)
    assert hm.height[cell] == pytest.approx(6.0)  # 9 m was seen only once
    hm.integrate_hits(np.array([[5.0, 0.0, 9.0]]), sensor_z=8.0)
    assert hm.height[cell] == pytest.approx(6.0)  # the 6 m proposal is not 9 m support
    hm.integrate_hits(np.array([[5.0, 0.0, 9.0]]), sensor_z=8.0)
    assert hm.height[cell] == pytest.approx(9.0)


def test_unmapped_hits_carve_but_do_not_raise_cells(spec: GridSpec, cfg: Config) -> None:
    # Hits that are too far away or on the net are not mapped, but their beams still passed over the
    # cells in between: they lower a guessed margin like any other beam.
    hm, cell = _pole_with_margin(spec, cfg, 6.0)
    over = [12.0, 0.0, 8.0 + 12.0 * (0.7 / 6.0)]  # passes x = 6 at z = 8.7
    hm.integrate_hits(np.array([over]), sensor_z=8.0, origin=(0.0, 0.0, 8.0), mapped=np.array([False]))
    assert hm.height[cell] == pytest.approx(8.7, abs=0.15)
    assert hm.height[spec.index(12.0, 0.0)] == 0.0
    assert not hm.observed[spec.index(12.0, 0.0)]
