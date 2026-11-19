from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from aufdeckung.config import Config
from aufdeckung.export.lagebild import detection_message
from aufdeckung.export.sdf import scenario_to_sdf, write_world
from aufdeckung.export.waypoints import to_ned_setpoints, to_qgc_plan, write_json
from aufdeckung.geo import GeoOrigin
from aufdeckung.planning.coverage_planner import Waypoint
from aufdeckung.scenario.generator import generate


def test_sdf_world_structure(cfg: Config, tmp_path: Path) -> None:
    sc = generate("urban_ring", 4)
    path = write_world(sc, cfg.arena, tmp_path / "w.sdf", world_name="testworld")
    root = ET.parse(path).getroot()
    world = root.find("world")
    assert world is not None and world.get("name") == "testworld"
    names = [m.get("name") for m in world.findall("model")]
    expected = 1 + 2 * cfg.arena.n_sides + 1 + len(sc.obstacles) + len(sc.targets) + len(sc.rain_zones)
    assert len(names) == expected
    assert len(set(names)) == len(names), "model names must be unique for Gazebo"
    plugins = {p.get("filename") for p in world.findall("plugin")}
    assert {"gz-sim-navsat-system", "gz-sim-sensors-system", "gz-sim-imu-system"} <= plugins
    lat = world.find("spherical_coordinates/latitude_deg")
    assert lat is not None and float(lat.text or 0) == cfg.arena.origin_lat


def test_sdf_poses_match_scenario(cfg: Config) -> None:
    sc = generate("sparse", 2)
    root = ET.fromstring(scenario_to_sdf(sc, cfg.arena))
    by_name = {m.get("name"): m for m in root.iter("model")}
    for obs in sc.obstacles:
        pose = by_name[obs.name].findtext("pose")
        assert pose is not None
        x, y, z = (float(v) for v in pose.split()[:3])
        assert (x, y, z) == pytest.approx((obs.x, obs.y, obs.z0 + obs.height / 2), abs=1e-3)
    rain = [m for name, m in by_name.items() if name and name.startswith("rain_zone")]
    assert all(m.find("link/collision") is None for m in rain), "rain is visual only"


def test_ned_setpoints() -> None:
    sp = to_ned_setpoints([Waypoint(3.0, -14.0, 8.0, "sweep")], home=(0.0, -18.0))
    assert sp == [{"x": 4.0, "y": 3.0, "z": 8.0, "w": 90.0, "kind": "sweep"}]


def test_qgc_plan(cfg: Config, tmp_path: Path) -> None:
    wps = [Waypoint(0.0, -18.0, 8.0), Waypoint(10.0, 0.0, 9.0)]
    plan = to_qgc_plan(wps, cfg.arena, cfg.flight, (0.0, -18.0))
    items = plan["mission"]["items"]
    assert [i["command"] for i in items] == [16, 16]
    o = GeoOrigin(cfg.arena.origin_lat, cfg.arena.origin_lon)
    x, y, _ = o.wgs84_to_enu(items[1]["params"][4], items[1]["params"][5])
    assert (x, y) == pytest.approx((10.0, 0.0), abs=0.01)
    assert items[1]["params"][6] == 9.0
    path = write_json(plan, tmp_path / "sub" / "m.plan")
    assert json.loads(path.read_text())["fileType"] == "Plan"


def test_lagebild_message(cfg: Config) -> None:
    msg = detection_message("class1", 5.0, -3.0, cfg.arena, 42.0, confidence=0.9, object_id="a")
    o = GeoOrigin(cfg.arena.origin_lat, cfg.arena.origin_lon)
    x, y, _ = o.wgs84_to_enu(msg["position"]["lat"], msg["position"]["lon"])
    assert (x, y) == pytest.approx((5.0, -3.0), abs=0.01)
    assert msg["class"] == "class1" and msg["mission_time_s"] == 42.0
    json.dumps(msg)
    with pytest.raises(ValueError):
        detection_message("class1", 0, 0, cfg.arena, 0, confidence=1.5)


def test_world_magnetic_field_matches_the_cage_location(cfg: Config) -> None:
    # Regression (Gazebo flight): the world used PX4's example field (~15 deg east of north) while
    # PX4 corrects headings with Braunschweig's ~4.6 deg declination. The ~10 deg heading error
    # rotated every lidar point about the drone: phantom halos around buildings, worse with range.
    import math

    root = ET.fromstring(scenario_to_sdf(generate("sparse", 0, cfg.arena), cfg.arena))
    east, north, up = (float(v) for v in root.find("world/magnetic_field").text.split())
    assert math.degrees(math.atan2(east, north)) == pytest.approx(4.6, abs=0.5)
    assert up < 0  # field points down in the northern hemisphere
    assert math.degrees(math.atan2(-up, math.hypot(east, north))) == pytest.approx(67.9, abs=1.0)
