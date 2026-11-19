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
        from aufdeckung.geometry import Box
        targets = [Box(t.x, t.y, t.size, t.size, max(t.size * 0.6, 0.15), kind="target")
                   for t in self.sc.targets]  # exported to Gazebo as small boxes
        true_h = rasterize(list(self.sc.obstacles) + targets, self.spec)
        # One cell of slack: a hit on a wall face may fall into the neighbouring cell.
        self.allowed = max_filter_disk(true_h, 1.5) + PHANTOM_MARGIN_M
        self.inside = cage_mask(self.spec, cfg.arena, margin=2.5)  # net and poles are not counted
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


def convert(pts, pos, yaw, att, cfg, rot=0.0, mirror=False, attitude=True, max_range=None):  # type: ignore[no-untyped-def]
    beam_step = (cfg.lidar.v_max_deg - cfg.lidar.v_min_deg) / max(cfg.lidar.v_samples - 1, 1)
    world, _ = lidar_to_world(_rot_z(pts, rot, mirror), pos, yaw, cfg.lidar.v_max_deg, beam_step,
                              attitude_ned=att if attitude else None,
                              max_range=max_range or cfg.lidar.max_range)
    return world


def _shifted(world: np.ndarray, pos, yaw_err_deg: float, dx: float, dy: float) -> np.ndarray:  # type: ignore[no-untyped-def]
    """Undo a heading error (rotate about the drone) and a position error (shift)."""
    out = world.copy()
    if yaw_err_deg:
        c, s = math.cos(math.radians(yaw_err_deg)), math.sin(math.radians(yaw_err_deg))
        rx, ry = world[:, 0] - pos[0], world[:, 1] - pos[1]
        out[:, 0], out[:, 1] = pos[0] + c * rx - s * ry, pos[1] + s * rx + c * ry
    out[:, 0] += dx
    out[:, 1] += dy
    return out


def _rate(bag, truth, cfg, home, yaw_err=0.0, dx=0.0, dy=0.0, every=4, lag=0.0,  # type: ignore[no-untyped-def]
          by_tilt=False):
    n = ph = 0
    tilt_bins = {b: [0, 0] for b in ((0, 3), (3, 8), (8, 15), (15, 90))}
    for k, (_t, pts, pos, yaw, att) in enumerate(scan_inputs(bag, home, lag)):
        if k % every:
            continue
        c, p = truth.phantoms(_shifted(convert(pts, pos, yaw, att, cfg), pos, yaw_err, dx, dy))
        n += int(c.sum())
        ph += int(p.sum())
        if by_tilt:
            tl = tilt_of(att)
            for (lo, hi), acc in tilt_bins.items():
                if lo <= tl < hi:
                    acc[0] += int(c.sum())
                    acc[1] += int(p.sum())
    if by_tilt:
        return {b: 100 * v[1] / max(v[0], 1) for b, v in tilt_bins.items() if v[0]}  # type: ignore[return-value]
    return 100 * ph / max(n, 1)


def latency_report(bag: dict) -> None:
    """How late lidar scans arrive (bag time minus the sensor's own stamp; only the spread matters,
    the offset is sim time vs wall clock)."""
    lid = bag["/lidar/points"]
    if not lid:
        return
    d = np.array([t - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9) for t, m in lid])
    d -= d.min()
    p5, p50, p95, mx = np.percentile(d, [5, 50, 95, 100])
    print(f"lidar arrival jitter (relative to its own stamp): p5 {p5:.3f} s, median {p50:.3f} s, "
          f"p95 {p95:.3f} s, max {mx:.3f} s")
    att = bag["/drone/out/attitude"]
    if att and att[0][1].header.stamp.sec:
        da = np.array([t - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9) for t, m in att])
        print(f"attitude transport delay: median {np.median(da) * 1000:.0f} ms")


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
    latency_report(bag)
    # The mission node's mapping gates (mapping_max_tilt_deg, mapping_max_range), as it maps now.
    n = ph = skipped = 0
    for _t, pts, pos, yaw, att in scan_inputs(bag, home, 0.0):
        if att is not None and tilt_of(att) > 12.0:
            skipped += 1
            continue
        c, p = truth.phantoms(convert(pts, pos, yaw, att, cfg, max_range=20.0))
        n += int(c.sum())
        ph += int(p.sum())
    print(f"WITH THE NODE'S GATES (tilt <= 12 deg, range < 20 m): {100 * ph / max(n, 1):.2f} % phantoms, "
          f"{skipped} of {len(bag['/lidar/points'])} scans skipped")

    print("heading error sweep (rotate points about the drone; the minimum is the real error):")
    coarse = {e: _rate(bag, truth, cfg, home, yaw_err=e) for e in range(-20, 21, 2)}
    best = min(coarse, key=lambda e: (round(coarse[e], 2), abs(e)))
    fine = {best + d / 2: _rate(bag, truth, cfg, home, yaw_err=best + d / 2) for d in range(-3, 4)}
    best_yaw = min(fine, key=lambda e: (round(fine[e], 2), abs(e)))
    print("    " + "  ".join(f"{e:+d}:{r:.1f}%" for e, r in coarse.items()))
    print(f"    BEST heading correction {best_yaw:+.1f} deg -> {fine[best_yaw]:.2f} % phantoms "
          f"(0 deg: {coarse[0]:.2f} %)")
    print(f"position error sweep at {best_yaw:+.1f} deg (dx, dy in m):")
    grid = {(dx, dy): _rate(bag, truth, cfg, home, yaw_err=best_yaw, dx=dx, dy=dy)
            for dx in (-1.0, -0.5, 0.0, 0.5, 1.0) for dy in (-1.0, -0.5, 0.0, 0.5, 1.0)}
    (bx, by) = min(grid, key=lambda k: (round(grid[k], 2), abs(k[0]) + abs(k[1])))
    print(f"    BEST shift dx={bx:+.1f} dy={by:+.1f} -> {grid[(bx, by)]:.2f} % (no shift: {grid[(0.0, 0.0)]:.2f} %)")
    print(f"timing sweep at {best_yaw:+.1f} deg (lag < 0: use pose/attitude from BEFORE the scan arrived):")
    lags = [round(-1.5 + 0.1 * i, 1) for i in range(21)]
    by_lag = {lag: _rate(bag, truth, cfg, home, yaw_err=best_yaw, dx=bx, dy=by, lag=lag) for lag in lags}
    best_lag = min(by_lag, key=lambda k: (round(by_lag[k], 2), abs(k)))
    print("    " + "  ".join(f"{k:+.1f}:{v:.1f}%" for k, v in by_lag.items()))
    print(f"    BEST lag {best_lag:+.1f} s -> {by_lag[best_lag]:.2f} % (lag 0: {by_lag[0.0]:.2f} %)")
    print("phantom share by tilt, after these corrections (flat = tilt correction works):")
    for (lo, hi), v in _rate(bag, truth, cfg, home, yaw_err=best_yaw, dx=bx, dy=by, lag=best_lag,
                             every=1, by_tilt=True).items():
        print(f"    {lo:2d}-{hi:2d} deg: {v:6.2f} %")

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


BLOCK_HEIGHT_M = 8.5  # a cell this tall makes its 1.5 m surroundings unflyable at the 10 m limit


def replay_blame(bag: dict, truth: Truth, cfg: Config, home: tuple[float, float], plot: Path | None) -> None:
    """Replay the scans through the mission node's own mapping and say which scan, which beam and
    which drone state wrongly made cells too tall to fly over, above all around home."""
    from aufdeckung.mapping.grid import HeightMap
    from aufdeckung.planning.flightmap import build_flight_map
    from aufdeckung_ros.conversions import tilt_deg

    beam_step = (cfg.lidar.v_max_deg - cfg.lidar.v_min_deg) / max(cfg.lidar.v_samples - 1, 1)
    spec = truth.spec
    hm = HeightMap(spec, cfg.arena)
    blame: dict[tuple[int, int], dict] = {}
    t0 = bag["/lidar/points"][0][0] if bag["/lidar/points"] else 0.0
    for t, pts, pos, yaw, att in scan_inputs(bag, home, 0.0):
        if att is not None and tilt_deg(att) > 12.0:  # the node's gates
            continue
        base = (pos[0], pos[1], pos[2] - LIDAR_ABOVE_BASE_M)  # the node uses the drone position
        world, top = lidar_to_world(pts, base, yaw, cfg.lidar.v_max_deg, beam_step, attitude_ned=att,
                                    max_range=20.0)
        before = hm.height.copy()
        hm.integrate_hits(world, base[2], top, origin=base)
        crossed = (hm.height >= BLOCK_HEIGHT_M) & (before < BLOCK_HEIGHT_M) & (truth.allowed < BLOCK_HEIGHT_M)
        if not crossed.any():
            continue
        iy, ix = spec.indices(world[:, 0], world[:, 1])
        ok = spec.valid(iy, ix)
        sens = pts[np.isfinite(pts).all(axis=1)]
        sens = sens[np.linalg.norm(sens, axis=1) >= 0.6]
        rng_all = np.linalg.norm(sens, axis=1)
        sens = sens[rng_all < 19.5]
        for cy, cx in zip(*np.nonzero(crossed)):
            m = ok & (iy == cy) & (ix == cx) & (world[:, 2] >= BLOCK_HEIGHT_M - 2.5)
            if not m.any() or len(sens) != len(world):
                src = None
            else:
                k = int(np.argmax(np.where(m, world[:, 2], -np.inf)))
                v = sens[k]
                src = dict(range=float(np.linalg.norm(v)), elev=float(np.degrees(np.arctan2(v[2], np.hypot(v[0], v[1])))),
                           az=float(np.degrees(np.arctan2(v[1], v[0]))), top=bool(top[k]), z=float(world[k, 2]))
            blame[(int(cy), int(cx))] = dict(t=t - t0, pos=base, tilt=tilt_of(att), src=src,
                                            height=float(hm.height[cy, cx]), true=float(truth.true_h[cy, cx]))
    fm = build_flight_map(hm, cfg, (), assume_unknown_below_cruise=True)
    hy, hx = fm.cell(*home)
    print("== replay through the mission node's mapping (gates on)")
    print(f"    wrongly blocking cells (believed >= {BLOCK_HEIGHT_M} m, truth lower): {len(blame)}")
    print(f"    home cell flyable at the end: {bool(fm.hard_ok[hy, hx])} "
          f"(required altitude {fm.required_alt[hy, hx]:.2f} m, limit {cfg.flight.max_alt} m)")
    r_cells = cfg.flight.clearance / spec.res + 1
    near = sorted(((cy - hy) ** 2 + (cx - hx) ** 2, c, b) for c, b in blame.items()
                  if (c[0] - hy) ** 2 + (c[1] - hx) ** 2 <= (r_cells + 4) ** 2)
    print(f"    of them within {cfg.flight.clearance + 2:.1f} m of home: {len(near)}")
    show = [b for _d, _c, b in near[:8]] or [b for b in sorted(blame.values(), key=lambda b: b["t"])[:8]]
    print("    culprits (time, cell believed/true height, drone pos, tilt, the beam that raised it):")
    for b in show:
        x, y, z = b["pos"]
        src = b["src"]
        beam = (f"range {src['range']:5.2f} m, elev {src['elev']:+6.1f} deg, az {src['az']:+7.1f} deg, "
                f"{'TOP beam (+2 m margin)' if src['top'] else 'not top beam'}, hit z {src['z']:.2f}") if src else "?"
        print(f"      t={b['t']:6.1f}s  {b['height']:5.2f}/{b['true']:4.2f} m  pos=({x:5.1f},{y:6.1f},{z:4.1f})  "
              f"tilt {b['tilt']:4.1f}  {beam}")
    if plot is not None:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            return
        out = plot.with_name("replay_map.png")
        ext = (spec.xmin, spec.xmin + spec.nx * spec.res, spec.ymin, spec.ymin + spec.ny * spec.res)
        fig, ax = plt.subplots(figsize=(7, 6))
        im = ax.imshow(hm.height, origin="lower", extent=ext, vmin=0, vmax=13)
        fig.colorbar(im, ax=ax, label="believed height [m]")
        ax.contour(np.linspace(ext[0], ext[1], spec.nx), np.linspace(ext[2], ext[3], spec.ny), truth.true_h,
                   levels=[0.5], colors="white", linewidths=0.6)
        if blame:
            cs = np.array([spec.center(*c) for c in blame])
            ax.scatter(cs[:, 0], cs[:, 1], s=6, c="red", label="wrongly blocking")
        ax.plot([home[0]], [home[1]], "w*", ms=12, label="home")
        ax.legend(loc="upper right")
        ax.set_title("map from replaying the bag (white: true obstacle outlines)")
        fig.savefig(out, dpi=110, bbox_inches="tight")
        print(f"    saved {out}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag", type=Path)
    ap.add_argument("--scenario", type=Path, default=None,
                    help="arena YAML (default: the newest simulation/worlds/*.yaml)")
    ap.add_argument("--plot", type=Path, default=REPO / "out" / "phantoms.png")
    ap.add_argument("--replay-only", action="store_true", help="only the mapping replay (fast)")
    args = ap.parse_args()
    scenario = args.scenario or max((REPO / "simulation" / "worlds").glob("*.yaml"),
                                    key=lambda p: p.stat().st_mtime)
    cfg = Config()
    truth = Truth(scenario, cfg)
    home = (float(truth.sc.start[0]), float(truth.sc.start[1]))
    print(f"bag {args.bag}, arena {scenario.name}, home {home}")
    bag = read_bag(args.bag)
    replay_blame(bag, truth, cfg, home, args.plot)
    if not args.replay_only:
        analyze(bag, truth, cfg, home, args.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
