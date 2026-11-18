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
