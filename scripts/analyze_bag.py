#!/usr/bin/env python3
"""Find phantom obstacles in a recorded Gazebo flight by comparing every lidar point with the truth.

    python3 scripts/analyze_bag.py bags/flight1 [--scenario simulation/worlds/dense_urban_3.yaml]

Needs no ROS (pip install rosbags). For every lidar scan it takes the drone pose, attitude and
heading at that moment, converts the points exactly like the mission node, and checks each one
against the true arena: a point clearly above the true height at its place is a phantom. It then
tries alternative explanations (rotated or mirrored sensor, yaw-only conversion, pose arriving
early/late) and reports which one leaves the fewest phantoms. Prints a short report and saves
out/phantoms.png.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / "aufdeckung" / "src"), str(REPO / "src" / "aufdeckung_ros")]

from aufdeckung.config import Config  # noqa: E402
from aufdeckung.mapping.grid import GridSpec, cage_mask, max_filter_disk, rasterize  # noqa: E402
from aufdeckung.scenario.io import load_scenario  # noqa: E402
from aufdeckung_ros.conversions import (  # noqa: E402
    heading_to_enu_yaw,
    lidar_to_world,
    px4_ned_to_cage_enu,
)

LIDAR_ABOVE_BASE_M = 0.12  # lidar_link pose in simulation/x500_aufdeckung/model.sdf
PHANTOM_MARGIN_M = 0.5
_PC2_TYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def cloud_to_xyz(msg) -> np.ndarray:  # type: ignore[no-untyped-def]
    """(N, 3) sensor-frame points from a PointCloud2, without sensor_msgs_py."""
    order = ">" if msg.is_bigendian else "<"
    fields = {f.name: f for f in msg.fields}
    dtype = np.dtype({"names": ["x", "y", "z"],
                      "formats": [order + _PC2_TYPES[fields[n].datatype] for n in ("x", "y", "z")],
                      "offsets": [fields[n].offset for n in ("x", "y", "z")],
                      "itemsize": msg.point_step})
    arr = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    return np.column_stack([arr[n].astype(float) for n in ("x", "y", "z")])


def read_bag(path: Path) -> dict[str, list[tuple[float, object]]]:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    wanted = {"/lidar/points", "/drone/out/pos/ned", "/drone/out/attitude", "/drone/out/heading"}
    out: dict[str, list[tuple[float, object]]] = {t: [] for t in wanted}
    with AnyReader([path], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        conns = [c for c in reader.connections if c.topic in wanted]
        for conn, t_ns, raw in reader.messages(connections=conns):
            out[conn.topic].append((t_ns * 1e-9, reader.deserialize(raw, conn.msgtype)))
    return out


def _latest(times: np.ndarray, t: float) -> int | None:
    i = int(np.searchsorted(times, t, side="right")) - 1
    return i if i >= 0 else None


def scan_inputs(bag: dict, home: tuple[float, float], lag: float):  # type: ignore[no-untyped-def]
    """Yield (t, sensor points, drone ENU, yaw, attitude or None) per lidar scan."""
    pose_t = np.array([t for t, _ in bag["/drone/out/pos/ned"]])
    att_t = np.array([t for t, _ in bag["/drone/out/attitude"]])
    head_t = np.array([t for t, _ in bag["/drone/out/heading"]])
    for t, cloud in bag["/lidar/points"]:
        ip = _latest(pose_t, t + lag)
        if ip is None:
            continue
        p = bag["/drone/out/pos/ned"][ip][1].pose.position
        x, y, z = px4_ned_to_cage_enu(p.x, p.y, p.z, home)
        ih = _latest(head_t, t + lag)
        yaw = heading_to_enu_yaw(bag["/drone/out/heading"][ih][1].data) if ih is not None else 0.0
        ia = _latest(att_t, t + lag)
        att = None
        if ia is not None:
            q = bag["/drone/out/attitude"][ia][1].quaternion
            att = (q.w, q.x, q.y, q.z)
        yield t, cloud_to_xyz(cloud), (x, y, z + LIDAR_ABOVE_BASE_M), yaw, att


def _rot_z(pts: np.ndarray, deg: float, mirror: bool = False) -> np.ndarray:
    out = pts.copy()
    if mirror:
        out[:, 1] = -out[:, 1]
    if deg:
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        x, y = out[:, 0].copy(), out[:, 1].copy()
        with np.errstate(invalid="ignore"):  # "no return" points are inf and dropped later anyway
            out[:, 0], out[:, 1] = c * x - s * y, s * x + c * y
    return out


VARIANTS = {
    "as implemented (attitude)": dict(rot=0.0, mirror=False, attitude=True),
    "yaw only, no tilt": dict(rot=0.0, mirror=False, attitude=False),
    "sensor rotated +90 deg": dict(rot=90.0, mirror=False, attitude=True),
    "sensor rotated -90 deg": dict(rot=-90.0, mirror=False, attitude=True),
    "sensor rotated 180 deg": dict(rot=180.0, mirror=False, attitude=True),
    "sensor y mirrored": dict(rot=0.0, mirror=True, attitude=True),
}


class Truth:
    def __init__(self, scenario_path: Path, cfg: Config) -> None:
        self.sc = load_scenario(scenario_path)
        self.spec = GridSpec.for_arena(cfg.arena, cfg.planner.grid_res)
        true_h = rasterize(self.sc.obstacles, self.spec)
        # One cell of slack: a hit on a wall face may fall into the neighbouring cell.
        self.allowed = max_filter_disk(true_h, 1.5) + PHANTOM_MARGIN_M
        self.inside = cage_mask(self.spec, cfg.arena, margin=1.0)
        self.true_h = true_h

    def phantoms(self, world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(mask of points that count, mask of phantoms among all points)."""
        iy, ix = self.spec.indices(world[:, 0], world[:, 1])
        ok = self.spec.valid(iy, ix)
        counted = np.zeros(len(world), dtype=bool)
        phantom = np.zeros(len(world), dtype=bool)
        iy, ix = np.clip(iy, 0, self.spec.ny - 1), np.clip(ix, 0, self.spec.nx - 1)
        counted = ok & self.inside[iy, ix] & (world[:, 2] < 12.5)
        phantom = counted & (world[:, 2] > self.allowed[iy, ix])
        return counted, phantom


def convert(pts, pos, yaw, att, cfg, rot=0.0, mirror=False, attitude=True):  # type: ignore[no-untyped-def]
    beam_step = (cfg.lidar.v_max_deg - cfg.lidar.v_min_deg) / max(cfg.lidar.v_samples - 1, 1)
    world, _ = lidar_to_world(_rot_z(pts, rot, mirror), pos, yaw, cfg.lidar.v_max_deg, beam_step,
                              attitude_ned=att if attitude else None, max_range=cfg.lidar.max_range)
    return world


def tilt_of(att) -> float:  # type: ignore[no-untyped-def]
    if att is None:
        return float("nan")
    w, x, y, z = att
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def analyze(bag: dict, truth: Truth, cfg: Config, home: tuple[float, float], plot: Path | None) -> dict:
    n_scans = len(bag["/lidar/points"])
    print(f"messages: lidar {n_scans}, pose {len(bag['/drone/out/pos/ned'])}, "
          f"attitude {len(bag['/drone/out/attitude'])}, heading {len(bag['/drone/out/heading'])}")
    if not bag["/drone/out/attitude"]:
        print("  NOTE: no attitude in the bag (recorded before px4_to_ros2 published it)")
    rows = []
    for t, pts, pos, yaw, att in scan_inputs(bag, home, 0.0):
        world = convert(pts, pos, yaw, att, cfg)
        counted, phantom = truth.phantoms(world)
        sensor = pts[np.isfinite(pts).all(axis=1)]
        rows.append(dict(t=t, pos=pos, tilt=tilt_of(att), n=int(counted.sum()), ph=int(phantom.sum()),
                         world=world, phantom=phantom, raw=len(pts), finite=len(sensor),
                         at_limit=int((np.linalg.norm(sensor, axis=1) >= cfg.lidar.max_range - 0.5).sum())))
    if not rows:
        print("no lidar scan with a pose before it")
        return {}
    t0 = rows[0]["t"]
    tot_n = sum(r["n"] for r in rows)
    tot_ph = sum(r["ph"] for r in rows)
    raw, fin, lim = (sum(r[k] for r in rows) for k in ("raw", "finite", "at_limit"))
    print(f"raw points {raw}: finite {fin}, no return {raw - fin}, at the range limit {lim}")
    print(f"PHANTOMS (as implemented): {tot_ph} of {tot_n} counted points = {100 * tot_ph / max(tot_n, 1):.2f} %")

    # Where are they: distance from the drone, beam elevation in the sensor, drone tilt.
    all_w = np.concatenate([r["world"] for r in rows])
    all_ph = np.concatenate([r["phantom"] for r in rows])
    all_pos = np.concatenate([np.repeat(np.array([r["pos"]]), len(r["world"]), axis=0) for r in rows])
    all_tilt = np.concatenate([np.full(len(r["world"]), r["tilt"]) for r in rows])
    d = np.linalg.norm(all_w - all_pos, axis=1)
    print("phantom share by distance from the drone:")
    for lo, hi in [(0, 3), (3, 8), (8, 15), (15, 22), (22, 31)]:
        m = (d >= lo) & (d < hi)
        if m.any():
            print(f"    {lo:2d}-{hi:2d} m: {100 * all_ph[m].mean():6.2f} % of {int(m.sum())}")
    if np.isfinite(all_tilt).any():
        print("phantom share by drone tilt:")
        for lo, hi in [(0, 3), (3, 8), (8, 15), (15, 90)]:
            m = (all_tilt >= lo) & (all_tilt < hi)
            if m.any():
                print(f"    {lo:2d}-{hi:2d} deg: {100 * all_ph[m].mean():6.2f} % of {int(m.sum())}")
    dz = all_w[all_ph, 2] - truth.true_h[truth.spec.indices(all_w[all_ph, 0], all_w[all_ph, 1])]
    if dz.size:
        print(f"phantoms are {np.median(dz):.2f} m (median) above the true surface")

    print("alternative explanations (fewer phantoms = more likely the truth):")
    results = {}
    for name, v in VARIANTS.items():
        n = ph = 0
        for _t, pts, pos, yaw, att in scan_inputs(bag, home, 0.0):
            c, p = truth.phantoms(convert(pts, pos, yaw, att, cfg, **v))
            n += int(c.sum())
            ph += int(p.sum())
        results[name] = 100 * ph / max(n, 1)
        print(f"    {name:28s} {results[name]:6.2f} %")
    print("pose timing (positive = pose taken later than the scan):")
    for lag in (-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3):
        n = ph = 0
        for _t, pts, pos, yaw, att in scan_inputs(bag, home, lag):
            c, p = truth.phantoms(convert(pts, pos, yaw, att, cfg))
            n += int(c.sum())
            ph += int(p.sum())
        print(f"    {lag:+.1f} s: {100 * ph / max(n, 1):6.2f} %")

    worst = sorted(rows, key=lambda r: -r["ph"])[:5]
    print("worst scans (time since first scan, drone position, tilt, phantoms):")
    for r in worst:
        x, y, z = r["pos"]
        print(f"    t={r['t'] - t0:6.1f}s  pos=({x:6.1f},{y:6.1f},{z:5.1f})  tilt={r['tilt']:5.1f} deg  {r['ph']}")
    if plot is not None:
        _plot(truth, all_w[all_ph], np.array([r["pos"] for r in rows]), plot)
    return results


def _plot(truth: Truth, phantoms: np.ndarray, path_xy: np.ndarray, out: Path) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    s = truth.spec
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.imshow(truth.true_h, origin="lower", cmap="Greys", extent=(s.xmin, s.xmin + s.nx * s.res,
              s.ymin, s.ymin + s.ny * s.res))
    if len(phantoms):
        sc = ax.scatter(phantoms[:, 0], phantoms[:, 1], c=phantoms[:, 2], s=2, cmap="autumn")
        fig.colorbar(sc, ax=ax, label="phantom height [m]")
    ax.plot(path_xy[:, 0], path_xy[:, 1], "c-", lw=1, label="drone path")
    ax.legend(loc="upper right")
    ax.set_title("phantom lidar points (grey: true obstacles)")
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print(f"saved {out}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag", type=Path)
    ap.add_argument("--scenario", type=Path, default=None,
                    help="arena YAML (default: the newest simulation/worlds/*.yaml)")
    ap.add_argument("--plot", type=Path, default=REPO / "out" / "phantoms.png")
    args = ap.parse_args()
    scenario = args.scenario or max((REPO / "simulation" / "worlds").glob("*.yaml"),
                                    key=lambda p: p.stat().st_mtime)
    cfg = Config()
    truth = Truth(scenario, cfg)
    home = (float(truth.sc.start[0]), float(truth.sc.start[1]))
    print(f"bag {args.bag}, arena {scenario.name}, home {home}")
    analyze(read_bag(args.bag), truth, cfg, home, args.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
