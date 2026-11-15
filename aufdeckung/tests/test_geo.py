from __future__ import annotations

import pytest

from aufdeckung.geo import GeoOrigin, enu_to_ned, enu_yaw_to_ned_deg, ned_to_enu


def test_roundtrip_wgs84() -> None:
    o = GeoOrigin(52.311, 10.56, 80.0)
    lat, lon, alt = o.enu_to_wgs84(12.3, -20.5, 4.0)
    x, y, z = o.wgs84_to_enu(lat, lon, alt)
    assert (x, y, z) == pytest.approx((12.3, -20.5, 4.0), abs=1e-6)


def test_scale_is_plausible() -> None:
    o = GeoOrigin(52.311, 10.56)
    assert o.m_per_deg_lat == pytest.approx(111_270, rel=2e-3)
    assert o.m_per_deg_lon == pytest.approx(68_100, rel=5e-3)
    lat, lon, _ = o.enu_to_wgs84(0.0, 1.0)
    assert lat > o.lat and lon == o.lon


def test_wgs84_without_alt() -> None:
    assert GeoOrigin(0.0, 0.0, 5.0).wgs84_to_enu(0.0, 0.0)[2] == 0.0


def test_invalid_origin() -> None:
    with pytest.raises(ValueError):
        GeoOrigin(95.0, 0.0)


def test_enu_ned() -> None:
    assert enu_to_ned(1.0, 2.0, 3.0) == (2.0, 1.0, -3.0)
    assert ned_to_enu(*enu_to_ned(1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)


@pytest.mark.parametrize("enu,ned", [(0.0, 90.0), (1.5707963267948966, 0.0), (3.141592653589793, 270.0)])
def test_yaw(enu: float, ned: float) -> None:
    assert enu_yaw_to_ned_deg(enu) == pytest.approx(ned)
