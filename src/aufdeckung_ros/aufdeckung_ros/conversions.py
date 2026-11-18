"""Frame conversions between PX4 (local NED, origin = home) and the cage ENU frame.

Kept free of ROS imports so it is unit-tested without a ROS installation.
"""

from __future__ import annotations

import math

import numpy as np


def px4_ned_to_cage_enu(n: float, e: float, d: float, home: tuple[float, float]
                        ) -> tuple[float, float, float]:
    """PX4 local NED (relative to home) -> cage ENU (origin at the cage centre)."""
    return home[0] + e, home[1] + n, -d


def cage_enu_to_setpoint(x: float, y: float, z: float, home: tuple[float, float], yaw_deg: float
                         ) -> tuple[float, float, float, float]:
    """Cage ENU -> ``ros_msg/Vector4`` for ``/drone/in/offboard/pos/ned``.

    ``px4_to_ros2`` expects x = north, y = east, z = altitude *up* (it negates it
    into NED down) and w = yaw in degrees.
    """
    return y - home[1], x - home[0], z, yaw_deg


def heading_to_enu_yaw(heading_deg: float) -> float:
    """PX4 heading (deg, 0 = north, clockwise) -> ENU yaw (rad, 0 = east, counter-clockwise)."""
    return math.radians(90.0 - heading_deg)


def cloud_xyz(points: object) -> np.ndarray:
    """(N, 3) float array of x, y, z from what ``point_cloud2.read_points`` returns.

    Gazebo's lidar cloud also carries intensity and ring fields of other types, which
    ``read_points_numpy`` refuses (it needs one datatype for all fields), so the node reads a
    structured array instead. Older ROS versions return a list of tuples; both are accepted.
    """
    if isinstance(points, np.ndarray) and points.dtype.names:
        xyz = np.column_stack([points[name].astype(float).ravel() for name in ("x", "y", "z")])
    else:
        xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=float)  # type: ignore[attr-defined]
    return xyz.reshape(-1, 3)


# The lidar sits 12 cm above the x500's body; its motors and propellers are 0.25-0.4 m away and
# inside its field of view. Those self-hits looked like an obstacle at flight altitude right where
# the drone is, so its own cell and the take-off area became unflyable (mission went to HOLD).
SELF_FILTER_RADIUS_M = 0.6
MAX_RANGE_MARGIN_M = 0.5  # returns this close to the range limit count as "no return"


def quat_to_matrix(q: tuple[float, float, float, float]) -> np.ndarray:
    """Rotation matrix of a unit quaternion (w, x, y, z), Hamilton convention."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def tilt_deg(q_ned: tuple[float, float, float, float]) -> float:
    """Angle between the body's down axis and the world's down axis (roll and pitch combined)."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(quat_to_matrix(q_ned)[2, 2])))))


def lidar_to_world(points_sensor: np.ndarray, drone_enu: tuple[float, float, float], yaw_enu: float,
                   v_max_deg: float, beam_step_deg: float, min_range: float = SELF_FILTER_RADIUS_M,
                   attitude_ned: tuple[float, float, float, float] | None = None,
                   max_range: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Rotate sensor-frame points (x forward, y left, z up) into the cage frame.

    ``attitude_ned`` is PX4's attitude quaternion (w, x, y, z): the rotation from the body frame
    (forward-right-down) to local NED. With it, roll and pitch are taken into account; without
    it only ``yaw_enu`` is. Roll and pitch matter: a multicopter tilts 10-20 deg when it
    accelerates, and ignoring 10 deg puts a ground hit 20 m away 3.5 m too high, a phantom wall.

    Points closer than ``min_range`` (the drone itself), non-finite points and points at the
    sensor's ``max_range`` are dropped: a beam that hits nothing may be reported at the range
    limit instead of as infinite, and those would draw a ring of 30 m-away "walls" every scan.
    Returns world points and a mask of points that came from the highest beam.
    """
    pts = np.asarray(points_sensor, dtype=float).reshape(-1, 3)
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    rng = np.linalg.norm(pts, axis=1)
    keep = rng >= min_range
    if max_range is not None:
        keep &= rng < max_range - MAX_RANGE_MARGIN_M
    pts = pts[keep]
    world = np.empty_like(pts)
    if attitude_ned is not None:
        frd = pts * np.array([1.0, -1.0, -1.0])  # sensor FLU -> body FRD
        ned = frd @ quat_to_matrix(attitude_ned).T
        world[:, 0] = drone_enu[0] + ned[:, 1]  # east
        world[:, 1] = drone_enu[1] + ned[:, 0]  # north
        world[:, 2] = drone_enu[2] - ned[:, 2]  # up
    else:
        c, s = math.cos(yaw_enu), math.sin(yaw_enu)
        world[:, 0] = drone_enu[0] + c * pts[:, 0] - s * pts[:, 1]
        world[:, 1] = drone_enu[1] + s * pts[:, 0] + c * pts[:, 1]
        world[:, 2] = drone_enu[2] + pts[:, 2]
    # Beam identity is a property of the sensor, so the top beam is found in the sensor frame.
    elevation = np.degrees(np.arctan2(pts[:, 2], np.hypot(pts[:, 0], pts[:, 1])))
    top = elevation >= v_max_deg - beam_step_deg / 2
    return world, top
