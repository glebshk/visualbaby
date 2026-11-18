"""Pure-python tests (no ROS needed): python -m pytest src/aufdeckung_ros/test"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aufdeckung_ros.conversions import (
    cage_enu_to_setpoint,
    cloud_xyz,
    heading_to_enu_yaw,
    lidar_to_world,
    px4_ned_to_cage_enu,
    quat_to_matrix,
    tilt_deg,
)

HOME = (0.0, -18.0)


def test_pose_and_setpoint_roundtrip():
    x, y, z = px4_ned_to_cage_enu(4.0, 3.0, -8.0, HOME)
    assert (x, y, z) == (3.0, -14.0, 8.0)
    assert cage_enu_to_setpoint(x, y, z, HOME, 90.0) == (4.0, 3.0, 8.0, 90.0)


@pytest.mark.parametrize("heading,yaw", [(0.0, math.pi / 2), (90.0, 0.0), (180.0, -math.pi / 2)])
def test_heading(heading, yaw):
    assert heading_to_enu_yaw(heading) == pytest.approx(yaw)


def test_lidar_rotation_and_top_beam():
    pts = np.array([[10.0, 0.0, 0.0], [10.0, 0.0, 10.0 * math.tan(math.radians(10.0))], [np.nan, 0, 0]])
    world, top = lidar_to_world(pts, (1.0, 2.0, 8.0), math.pi / 2, v_max_deg=10.0, beam_step_deg=4.67)
    assert world.shape == (2, 3)
    np.testing.assert_allclose(world[0], [1.0, 12.0, 8.0], atol=1e-9)
    assert top.tolist() == [False, True]


def test_lidar_drops_hits_on_the_drone_itself():
    # Regression (first Gazebo flight): hits on the drone's own motors/props, 0.25-0.4 m from the
    # lidar, became an obstacle at flight altitude and the mission went to HOLD right after take-off.
    motor = [0.174, 0.174, -0.09]  # x500 motor relative to the lidar
    pts = np.array([motor, [0.3, -0.2, -0.05], [5.0, 0.0, -2.0]])
    world, _ = lidar_to_world(pts, (0.0, -18.0, 8.0), 0.0, v_max_deg=10.0, beam_step_deg=4.67)
    assert world.shape == (1, 3)
    np.testing.assert_allclose(world[0], [5.0, -18.0, 6.0], atol=1e-9)


def test_cloud_xyz_accepts_gazebo_lidar_layout():
    # Regression: Gazebo's cloud has x, y, z (float32) plus intensity (float64) and ring (uint16);
    # read_points_numpy refused it and the mission node died on the first scan.
    dtype = np.dtype([("x", np.float32), ("y", np.float32), ("z", np.float32),
                      ("intensity", np.float64), ("ring", np.uint16)])
    cloud = np.array([(1.0, 2.0, 3.0, 0.5, 7), (4.0, 5.0, np.inf, 0.1, 8)], dtype=dtype)
    xyz = cloud_xyz(cloud[["x", "y", "z"]])
    assert xyz.shape == (2, 3) and xyz.dtype == float
    assert xyz[0].tolist() == [1.0, 2.0, 3.0] and np.isinf(xyz[1, 2])
    assert cloud_xyz(cloud).shape == (2, 3)  # also when the extra fields are still present


def test_cloud_xyz_accepts_tuples_and_empty_clouds():
    assert cloud_xyz([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]).tolist() == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    assert cloud_xyz([]).shape == (0, 3)
    empty = np.zeros(0, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
    assert cloud_xyz(empty).shape == (0, 3)


def _q(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    """PX4-style attitude quaternion (w, x, y, z) for body FRD -> NED, ZYX Euler angles."""
    r, p, y = (math.radians(a) / 2 for a in (roll_deg, pitch_deg, yaw_deg))
    return (math.cos(r) * math.cos(p) * math.cos(y) + math.sin(r) * math.sin(p) * math.sin(y),
            math.sin(r) * math.cos(p) * math.cos(y) - math.cos(r) * math.sin(p) * math.sin(y),
            math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.cos(p) * math.sin(y),
            math.cos(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y))


def _sensor_view(world_rel_enu, q):
    """What the lidar (FLU, on a body with attitude q) measures for a world offset (ENU)."""
    e, n, u = world_rel_enu
    frd = quat_to_matrix(q).T @ np.array([n, e, -u])
    return np.array([frd[0], -frd[1], -frd[2]])


def test_yaw_only_attitude_matches_the_yaw_path():
    pts = np.array([[5.0, 1.0, -2.0], [-3.0, 4.0, 1.0], [0.5, -7.0, -6.0]])
    heading = 30.0  # deg from north, clockwise
    a, _ = lidar_to_world(pts, (1.0, 2.0, 8.0), heading_to_enu_yaw(heading), 10.0, 4.67)
    b, _ = lidar_to_world(pts, (1.0, 2.0, 8.0), 0.0, 10.0, 4.67, attitude_ned=_q(yaw_deg=heading))
    np.testing.assert_allclose(a, b, atol=1e-9)


@pytest.mark.parametrize("roll,pitch", [(0.0, 12.0), (0.0, -12.0), (15.0, 0.0), (-8.0, 10.0)])
def test_tilted_drone_maps_ground_as_ground(roll, pitch):
    # Regression (Gazebo flight): ignoring the 10-20 deg tilt while accelerating turned flat ground
    # 15-25 m away into 3-5 m phantom walls; the drone kept rerouting until home looked blocked.
    q = _q(roll, pitch, yaw_deg=90.0)  # heading east
    ground = [(dx, dy, -8.0) for dx in (15.0, 25.0) for dy in (-6.0, 0.0, 6.0)]
    pts = np.array([_sensor_view(g, q) for g in ground])
    world, _ = lidar_to_world(pts, (0.0, -18.0, 8.0), 0.0, 10.0, 4.67, attitude_ned=q)
    np.testing.assert_allclose(world[:, 2], 0.0, atol=1e-9)
    np.testing.assert_allclose(world[:, :2], [[g[0], g[1] - 18.0] for g in ground], atol=1e-9)
    yaw_only, _ = lidar_to_world(pts, (0.0, -18.0, 8.0), heading_to_enu_yaw(90.0), 10.0, 4.67)
    assert np.abs(yaw_only[:, 2]).max() > 1.0  # the old conversion: ground 1-5 m too high or low


def test_tilt_deg():
    assert tilt_deg(_q()) == pytest.approx(0.0, abs=1e-6)
    assert tilt_deg(_q(pitch_deg=10.0, yaw_deg=123.0)) == pytest.approx(10.0)
    assert tilt_deg(_q(roll_deg=-20.0)) == pytest.approx(20.0)
