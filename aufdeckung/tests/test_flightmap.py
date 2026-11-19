from __future__ import annotations

import math

import numpy as np
import pytest

from aufdeckung.config import Config
from aufdeckung.geometry import Box
from aufdeckung.mapping.grid import GridSpec, HeightMap
from aufdeckung.planning.flightmap import build_flight_map, quantize_alt
from aufdeckung.scenario.model import RainZone


def _fm(cfg: Config, spec: GridSpec, shapes: list[Box], rain: tuple[RainZone, ...] = ()):  # type: ignore[no-untyped-def]
    return build_flight_map(HeightMap.from_prior(spec, cfg.arena, shapes), cfg, rain,
                            assume_unknown_below_cruise=False)


def test_low_building_is_flown_over_higher(cfg: Config, spec: GridSpec) -> None:
    fm = _fm(cfg, spec, [Box(0.0, 0.0, 4.0, 4.0, 7.0)])
    cell = fm.cell(0.0, 0.0)
    assert fm.flyable[cell]
    assert fm.required_alt[cell] == pytest.approx(7.0 + cfg.flight.vertical_clearance)
    assert fm.transit_alt(*cell) == pytest.approx(8.5)
    near = fm.cell(2.0 + cfg.flight.clearance - 0.3, 0.0)
    assert fm.required_alt[near] == pytest.approx(8.5), "clearance ring also needs the climb"
    assert fm.required_alt[fm.cell(8.0, 0.0)] == cfg.flight.min_alt


def test_tall_building_blocks_with_clearance(cfg: Config, spec: GridSpec) -> None:
    fm = _fm(cfg, spec, [Box(0.0, 0.0, 4.0, 4.0, 13.0)])
    assert not fm.flyable[fm.cell(0.0, 0.0)]
    assert not fm.flyable[fm.cell(2.0 + cfg.flight.clearance - 0.3, 0.0)]
    assert fm.flyable[fm.cell(2.0 + cfg.flight.clearance + 0.6, 0.0)]
    assert math.isinf(fm.required_alt[fm.cell(0.0, 0.0)])


def test_net_margin(cfg: Config, spec: GridSpec) -> None:
    fm = _fm(cfg, spec, [])
    inradius = 22.5 * math.cos(math.pi / 16)
    margin = cfg.arena.net_margin
    assert not fm.flyable[fm.cell(inradius - margin + 0.5, 0.0)]
    assert fm.flyable[fm.cell(inradius - margin - 0.5, 0.0)]


def test_unknown_space_requires_cruise(cfg: Config, spec: GridSpec) -> None:
    hm = HeightMap(spec, cfg.arena)
    hm.observed[spec.index(0.0, 0.0)] = True
    fm = build_flight_map(hm, cfg)
    assert fm.required_alt[fm.cell(10.0, 10.0)] == pytest.approx(cfg.flight.cruise_alt)
    hm.observed[:] = True
    assert build_flight_map(hm, cfg).required_alt[fm.cell(10.0, 10.0)] == cfg.flight.min_alt


def test_rain_is_soft_block(cfg: Config, spec: GridSpec) -> None:
    fm = _fm(cfg, spec, [], (RainZone(5.0, 5.0, 2.0),))
    c = fm.cell(5.0, 5.0)
    assert fm.hard_ok[c] and not fm.flyable[c]
    assert not fm.is_flyable(5.0, 5.0)
    assert not fm.is_flyable(500.0, 5.0)


def test_segment_profile(cfg: Config, spec: GridSpec) -> None:
    fm = _fm(cfg, spec, [Box(0.0, 0.0, 2.0, 2.0, 6.0)])
    ok, peak = fm.segment_profile((-8.0, 0.0), (8.0, 0.0))
    assert ok and peak == pytest.approx(7.5)
    ok, peak = fm.segment_profile((-8.0, 0.0), (80.0, 0.0))
    assert not ok and math.isinf(peak)


@pytest.mark.parametrize("z,q", [(8.0, 8.0), (8.01, 8.25), (8.24, 8.25), (math.inf, math.inf)])
def test_quantize(z: float, q: float) -> None:
    assert quantize_alt(z) == q


def test_quantize_never_lowers() -> None:
    zs = np.linspace(0, 10, 997)
    assert all(quantize_alt(float(z)) >= z for z in zs)
