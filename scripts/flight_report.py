#!/usr/bin/env python3
"""Mission outcome of a recorded Gazebo flight: what did the camera cover, how safe was it?

    python3 scripts/flight_report.py bags/flight6 [--scenario simulation/worlds/dense_urban_3.yaml]

Needs no ROS (pip install rosbags). Takes the path the drone really flew (/drone/out/pos/ned) and
computes, with the same camera model and ground truth as the headless benchmark, which cells of the
cage the downward camera imaged: coverage of the observable area (overall and within the 3-minute
bonus window), targets imaged, mission time, closest approach to any obstacle and to the net, and
whether it landed at home. Saves out/coverage.png. Like the benchmark it assumes that whatever is in
the camera footprint is seen; the image processing itself is not part of this check.
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
from aufdeckung.geometry import distance_to_shape  # noqa: E402
from aufdeckung.mapping.coverage import CoverageMap  # noqa: E402
from aufdeckung.mapping.grid import GridSpec, rasterize  # noqa: E402
from aufdeckung.scenario.io import load_scenario  # noqa: E402
from aufdeckung.sensors.camera import visible_cells  # noqa: E402
from aufdeckung.sim.metrics import net_distance, observable_mask  # noqa: E402
from aufdeckung_ros.conversions import px4_ned_to_cage_enu  # noqa: E402

CAMERA_BELOW_BASE_M = 0.1  # OakD-Lite pose in simulation/x500_aufdeckung/model.sdf
AIRBORNE_M = 0.5


def read_poses(path: Path) -> list[tuple[float, object]]:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    out: list[tuple[float, object]] = []
    with AnyReader([path], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        conns = [c for c in reader.connections if c.topic == "/drone/out/pos/ned"]
        for conn, t_ns, raw in reader.messages(connections=conns):
            out.append((t_ns * 1e-9, reader.deserialize(raw, conn.msgtype)))
    return out


def report(poses: list[tuple[float, np.ndarray]], scenario_path: Path, cfg: Config, plot: Path | None) -> dict:
    sc = load_scenario(scenario_path)
    spec = GridSpec.for_arena(cfg.arena, cfg.planner.grid_res)
    true_h = rasterize(sc.obstacles, spec)
    observable = observable_mask(sc, cfg)
    cov = CoverageMap(spec)
    airborne = [(t, p) for t, p in poses if p[2] > AIRBORNE_M]
    if not airborne:
        print("the drone never got airborne in this recording")
        return {}
    t_takeoff = airborne[0][0]
    t_landed = next((t for t, p in poses if t > airborne[-1][0] and p[2] < 0.3), None)
    next_cam = -math.inf
    min_clear, closest, min_net, max_speed = math.inf, "", math.inf, 0.0
    prev = None
    for t, p in poses:
        if prev is not None and t > prev[0]:
            max_speed = max(max_speed, float(np.linalg.norm(p[:2] - prev[1][:2]) / (t - prev[0])))
        prev = (t, p)
        if p[2] <= AIRBORNE_M:
            continue
        if t >= next_cam:
            next_cam = t + 1.0 / cfg.camera.rate_hz
            cam = (p[0], p[1], p[2] - CAMERA_BELOW_BASE_M)
            iy, ix = visible_cells(cam, true_h, spec, cfg.camera)
            mask = np.zeros(spec.shape, dtype=bool)
            mask[iy, ix] = True
            cov.mark(mask, t - t_takeoff)
        for obs in sc.obstacles:
            if math.hypot(obs.x - p[0], obs.y - p[1]) - obs.bounding_radius > 5.0:
                continue
            d = distance_to_shape(obs, p)
            if d < min_clear:
                min_clear, closest = d, obs.name or obs.kind
        min_net = min(min_net, net_distance(p[0], p[1], cfg))

    bonus, budget = cfg.mission.bonus_window, cfg.mission.time_budget
    seen = [tg for tg in sc.targets if cov.covered[spec.index(tg.x, tg.y)]]
    seen_bonus = [tg for tg in sc.targets if cov.first_seen[spec.index(tg.x, tg.y)] <= bonus]
    end = poses[-1][1]
    landed_home = t_landed is not None and math.hypot(end[0] - sc.start[0], end[1] - sc.start[1]) < 1.0
    res = dict(coverage=cov.fraction(observable), coverage_bonus=cov.fraction_at(observable, bonus),
               coverage_budget=cov.fraction_at(observable, budget), min_clearance=min_clear,
               min_net=min_net, landed_home=landed_home)
    print(f"arena {scenario_path.name}: {int(observable.sum())} observable cells, {len(sc.targets)} targets")
    flight = (t_landed or poses[-1][0]) - t_takeoff
    print(f"flight time (take-off to {'landing' if t_landed else 'end of recording'}): {flight:.0f} s, "
          f"max ground speed {max_speed:.1f} m/s")
    print(f"COVERAGE of the observable area: {100 * res['coverage']:.1f} %   "
          f"(within {bonus:.0f} s: {100 * res['coverage_bonus']:.1f} %, within {budget:.0f} s: "
          f"{100 * res['coverage_budget']:.1f} %)   - benchmark criterion: >= 95 % within {budget:.0f} s")
    print(f"targets imaged: {len(seen)} of {len(sc.targets)} (within {bonus:.0f} s: {len(seen_bonus)})")
    missed = [tg.name or f"({tg.x:.1f}, {tg.y:.1f})" for tg in sc.targets if tg not in seen]
    if missed:
        print(f"    not imaged: {', '.join(missed)}")
    print(f"closest approach: {min_clear:.2f} m to {closest} (criterion >= 0.75 m, drone radius "
          f"{cfg.flight.drone_radius} m), {min_net:.2f} m to the net")
    print(f"landed at home: {landed_home}" + ("" if t_landed else " (no landing in the recording)"))
    if plot is not None:
        _plot(cov, observable, true_h, spec, np.array([p for _t, p in airborne]), sc, plot)
    return res


def _plot(cov: CoverageMap, observable: np.ndarray, true_h: np.ndarray, spec: GridSpec, path: np.ndarray,
          sc, out: Path) -> None:  # type: ignore[no-untyped-def]
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    ext = (spec.xmin, spec.xmin + spec.nx * spec.res, spec.ymin, spec.ymin + spec.ny * spec.res)
    img = np.full(spec.shape, np.nan)
    img[observable] = 0.0
    img[observable & cov.covered] = 1.0
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.imshow(np.where(true_h > 0.3, 1.0, np.nan), origin="lower", extent=ext, cmap="Greys", vmin=0, vmax=2)
    ax.imshow(img, origin="lower", extent=ext, cmap="RdYlGn", vmin=0, vmax=1, alpha=0.6)
    ax.plot(path[:, 0], path[:, 1], "b-", lw=0.7, label="flown path")
    for tg in sc.targets:
        ax.plot(tg.x, tg.y, "m^", ms=5)
    ax.plot(sc.start[0], sc.start[1], "k*", ms=12, label="home")
    ax.legend(loc="upper right")
    ax.set_title("camera coverage (green imaged, red missed; grey obstacles; triangles targets)")
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print(f"saved {out}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag", type=Path)
    ap.add_argument("--scenario", type=Path, default=None,
                    help="arena YAML (default: the newest simulation/worlds/*.yaml)")
    ap.add_argument("--plot", type=Path, default=REPO / "out" / "coverage.png")
    args = ap.parse_args()
    scenario = args.scenario or max((REPO / "simulation" / "worlds").glob("*.yaml"), key=lambda p: p.stat().st_mtime)
    sc = load_scenario(scenario)
    home = (float(sc.start[0]), float(sc.start[1]))
    poses = []
    for t, msg in read_poses(args.bag):
        q = msg.pose.position
        poses.append((t, np.array(px4_ned_to_cage_enu(q.x, q.y, q.z, home))))
    if not poses:
        print("no /drone/out/pos/ned in the recording")
        return 1
    report(poses, scenario, Config(), args.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
