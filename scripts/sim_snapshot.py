#!/usr/bin/env python3
"""Snapshot of what the mission node sees, for debugging a Gazebo run (run inside the container).

    source install/setup.bash && python3 scripts/sim_snapshot.py

Prints statistics of one raw lidar message (where are the points, how many hit nothing?), the
drone's tilt, and the mission node's height map, and saves the map as out/height_map.png.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

CAGE_HEIGHT_M = 13.0  # /aufdeckung/height_map encodes 0..100 as 0..cage height
OUT = Path(__file__).resolve().parents[1] / "out" / "height_map.png"


class Snapshot(Node):
    def __init__(self) -> None:
        super().__init__("aufdeckung_snapshot")
        self.cloud: PointCloud2 | None = None
        self.grid: OccupancyGrid | None = None
        self.att: QuaternionStamped | None = None
        self.create_subscription(PointCloud2, "/lidar/points", self._set("cloud"), 2)
        self.create_subscription(OccupancyGrid, "/aufdeckung/height_map", self._set("grid"), 1)
        self.create_subscription(QuaternionStamped, "/drone/out/attitude", self._set("att"), 5)

    def _set(self, name: str):  # type: ignore[no-untyped-def]
        def cb(msg) -> None:  # type: ignore[no-untyped-def]
            setattr(self, name, msg)
        return cb


def lidar_report(msg: PointCloud2) -> None:
    fields = ", ".join(f"{f.name}(type {f.datatype})" for f in msg.fields)
    print(f"  frame '{msg.header.frame_id}', {msg.height} x {msg.width} points, fields: {fields}")
    raw = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
    xyz = np.column_stack([np.asarray(raw[n], dtype=float).ravel() for n in ("x", "y", "z")])
    finite = np.isfinite(xyz).all(axis=1)
    rng = np.linalg.norm(xyz[finite], axis=1)
    n = len(xyz)
    print(f"  total {n}, finite {int(finite.sum())}, inf/nan (no return) {n - int(finite.sum())}")
    if rng.size == 0:
        return
    bins = [0.0, 0.6, 5.0, 15.0, 29.5, math.inf]
    labels = ["< 0.6 m (the drone itself)", "0.6-5 m", "5-15 m", "15-29.5 m", ">= 29.5 m (range limit!)"]
    counts, _ = np.histogram(rng, bins=bins)
    for label, c in zip(labels, counts):
        print(f"    {label:28s} {int(c):6d}")
    print(f"  range max {rng.max():.2f} m; sensor-frame z from {xyz[finite, 2].min():.2f} "
          f"to {xyz[finite, 2].max():.2f} m")


def tilt_report(att: QuaternionStamped | None) -> None:
    if att is None:
        print("  NO attitude on /drone/out/attitude: lidar is not tilt-corrected (rebuild px4_to_ros2)")
        return
    q = att.quaternion
    r22 = 1 - 2 * (q.x * q.x + q.y * q.y)
    print(f"  tilt {math.degrees(math.acos(max(-1.0, min(1.0, r22)))):.1f} deg (roll and pitch combined)")


def map_report(grid: OccupancyGrid) -> None:
    w, h, res = grid.info.width, grid.info.height, grid.info.resolution
    hm = np.asarray(grid.data, dtype=float).reshape(h, w) / 100.0 * CAGE_HEIGHT_M
    x0, y0 = grid.info.origin.position.x, grid.info.origin.position.y
    print(f"  {w} x {h} cells of {res} m; cells above 0.5 m: {int((hm > 0.5).sum())}, "
          f"above 6.5 m: {int((hm > 6.5).sum())}, above 8.5 m (blocks 10 m flight): {int((hm > 8.5).sum())}")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (matplotlib missing: no image)")
        return
    OUT.parent.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 6))
    im = ax.imshow(hm, origin="lower", cmap="viridis", vmin=0, vmax=CAGE_HEIGHT_M,
                   extent=(x0, x0 + w * res, y0, y0 + h * res))
    fig.colorbar(im, ax=ax, label="believed height [m]")
    ax.contour(np.linspace(x0, x0 + w * res, w), np.linspace(y0, y0 + h * res, h), hm, levels=[8.5],
               colors="red", linewidths=0.8)
    ax.set_title("mission node height map (red: too tall to overfly at 10 m)")
    ax.set_xlabel("x east [m]")
    ax.set_ylabel("y north [m]")
    fig.savefig(OUT, dpi=110, bbox_inches="tight")
    print(f"  saved {OUT}")


def main() -> int:
    rclpy.init()
    node = Snapshot()
    deadline = time.time() + 8.0
    while time.time() < deadline and None in (node.cloud, node.grid):
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.spin_once(node, timeout_sec=0.5)
    print("== lidar (one raw message)")
    if node.cloud is None:
        print("  no message on /lidar/points within 8 s")
    else:
        lidar_report(node.cloud)
    print("== tilt")
    tilt_report(node.att)
    print("== height map (mission node)")
    if node.grid is None:
        print("  no message on /aufdeckung/height_map (mission node not running?)")
    else:
        map_report(node.grid)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
