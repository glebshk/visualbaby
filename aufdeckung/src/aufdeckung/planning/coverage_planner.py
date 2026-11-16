"""Aufdeckung coverage planner (multi-altitude).

1. Lay out lawnmower rows (both axes are tried, the cheaper one wins). Row
   spacing comes from the camera footprint at cruise altitude and the overlap.
2. Sample each row; keep samples that are reachable and would image at least
   one still-unseen cell. Each sample flies at the cell's *transit altitude*:
   cruise altitude, raised locally where obstacles below demand it.
3. Order the row segments greedily (nearest first, by path length), joining
   them with A* paths that are smoothed without climbing higher than needed.
4. Gap filling: predict what the route will image (with occlusion), then add
   extra viewpoints - also at low altitude where the map is known - with a
   greedy set cover until no candidate adds enough new cells.
5. Expand to 3D waypoints: climbs happen before a leg, descents after it, so
   every horizontal leg is flown at or above the required altitude.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np

from aufdeckung.config import Config, FlightConfig
from aufdeckung.geometry import BoolArray, FloatArray
from aufdeckung.mapping.grid import HeightMap, IntArray, dilate
from aufdeckung.planning.flightmap import FlightMap, quantize_alt
from aufdeckung.planning.paths import Cell, astar, reachable, smooth
from aufdeckung.sensors.camera import visible_cells


class PlanningError(RuntimeError):
    """No safe plan exists (e.g. the drone is boxed in)."""


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    kind: str = "transit"

    @property
    def xyz(self) -> tuple[float, float, float]:
        return self.x, self.y, self.z


@dataclass
class CoveragePlan:
    waypoints: list[Waypoint]
    predicted: BoolArray
    axis: str
    need_cells: int
    reachable_need_cells: int
    est_time: float
    hopeless: BoolArray | None = None  # need cells no reachable viewpoint can image


@dataclass
class _Sample:
    x: float
    y: float
    z: float
    vis: tuple[IntArray, IntArray]


_Pt = tuple[float, float, float, str]


def plan_coverage(
    start: tuple[float, float, float],
    hm: HeightMap,
    fm: FlightMap,
    need: BoolArray,
    cfg: Config,
    axes: tuple[str, ...] = ("x", "y"),
    rescue: FlightMap | None = None,
    anchor: tuple[float, float] | None = None,
) -> CoveragePlan:
    """Plan a route from ``start`` that images as many ``need`` cells as possible.

    ``rescue`` is a flight map with reduced clearances used only to get back into the
    area connected to ``anchor`` (home) when the map has closed in around the drone.
    """
    spec = fm.spec
    entry_pts, entry = _entry(start, fm, rescue, anchor)
    reach = reachable(fm.flyable, entry)
    if not reach.any():
        raise PlanningError("no flyable cell reachable from the current position")
    sx, sy = spec.center(*entry)
    need_count = int(np.count_nonzero(need))
    if need_count == 0:
        return CoveragePlan([], np.zeros(spec.shape, bool), "-", 0, 0, 0.0)

    # Prefer the axis that images more (in 1 % steps), then the shorter route.
    best: tuple[tuple[float, float], list[_Pt], BoolArray, str] | None = None
    for axis in axes:
        rows = _row_segments(axis, fm, hm, reach, need, cfg)
        route, predicted = _order(rows, (sx, sy), fm, spec)
        gained = float(np.count_nonzero(predicted & need)) / need_count
        key = (-round(gained, 2), _route_length(route, (sx, sy)))
        if best is None or key < best[0]:
            best = (key, route, predicted, axis)
    assert best is not None
    _, route, predicted, axis = best

    remaining = need & ~predicted
    views, seeable = _gap_fill(remaining, fm, hm, reach, cfg)
    last = (route[-1][0], route[-1][1]) if route else (sx, sy)
    view_route, view_pred = _order([[v] for v in views], last, fm, spec, kind="view")
    route += view_route
    predicted |= view_pred

    pts: list[_Pt] = [*entry_pts, (sx, sy, fm.transit_alt(*entry), "transit"), *route]
    waypoints = to_waypoints([(start[0], start[1], start[2], "start"), *pts], fm)[1:]
    reach_need = _observable_from(reach, need, fm, hm, cfg)
    return CoveragePlan(waypoints, predicted, axis, need_count, reach_need,
                        estimate_time(waypoints, start, cfg.flight), remaining & ~seeable)


def plan_transit(start: tuple[float, float, float], goal: tuple[float, float], fm: FlightMap,
                 rescue: FlightMap | None = None) -> list[Waypoint]:
    """Safe path to ``goal`` (x, y) at transit altitude, ending above the goal."""
    entry_pts, entry = _entry(start, fm, rescue, goal)
    goal_cell = fm.cell(*goal)
    free = fm.flyable.copy()
    free[goal_cell] = free[goal_cell] or bool(fm.hard_ok[goal_cell])
    cells = astar(free, entry, goal_cell)
    if cells is None and rescue is not None and not entry_pts:
        entry_pts, entry = _rescue(start, fm, rescue, goal)
        cells = astar(free, entry, goal_cell)
    if cells is None:
        raise PlanningError(f"goal {goal} is not reachable")
    xy = smooth(fm, [fm.spec.center(*c) for c in cells])
    xy[-1] = (goal[0], goal[1])
    pts: list[_Pt] = [*entry_pts]
    for x, y in xy:
        pts.append((x, y, fm.transit_alt(*fm.cell(x, y)), "transit"))
    return to_waypoints([(start[0], start[1], start[2], "start"), *pts], fm)[1:]


# --------------------------------------------------------------------------- steps


def _entry(start: tuple[float, float, float], fm: FlightMap, rescue: FlightMap | None = None,
           anchor: tuple[float, float] | None = None) -> tuple[list[_Pt], Cell]:
    """Points needed to reach a flyable cell (escaping a rain zone if necessary) and that cell."""
    cell = fm.cell(start[0], start[1])
    if fm.flyable[cell]:
        if rescue is not None and anchor is not None and _cut_off(fm, cell, anchor):
            # Flyable here, but the map has closed in: this pocket no longer connects to home.
            # Planning from inside it would see almost nothing and write the rest off as hopeless.
            try:
                return _rescue(start, fm, rescue, anchor)
            except PlanningError:
                pass  # no way out even with reduced margins: work with the pocket
        return [], cell
    hard = fm.hard_ok
    if not hard[cell] and rescue is not None and anchor is not None:
        return _rescue(start, fm, rescue, anchor)
    if not hard[cell]:
        gx, gy = fm.spec.centers()
        d = np.where(hard, np.hypot(gx - start[0], gy - start[1]), np.inf)
        if not np.isfinite(d).any():
            raise PlanningError("no flyable cell in the map")
        cell = (int(np.unravel_index(np.argmin(d), d.shape)[0]),
                int(np.unravel_index(np.argmin(d), d.shape)[1]))
        if fm.flyable[cell]:
            x, y = fm.spec.center(*cell)
            return [(x, y, fm.transit_alt(*cell), "escape")], cell
    # Inside a rain zone: shortest path over hard-flyable cells to the nearest flyable cell.
    gx, gy = fm.spec.centers()
    d = np.where(fm.flyable & reachable(hard, cell), np.hypot(gx - start[0], gy - start[1]), np.inf)
    if not np.isfinite(d).any():
        raise PlanningError("trapped: no rain-free flyable cell reachable")
    flat = np.argsort(d, axis=None)[:5]
    for idx in flat:
        target = (int(idx // fm.spec.nx), int(idx % fm.spec.nx))
        path = astar(hard, cell, target)
        if path is not None:
            xy = smooth(fm, [fm.spec.center(*c) for c in path], soft=True)
            return [(x, y, fm.transit_alt(*fm.cell(x, y)), "escape") for x, y in xy], target
    raise PlanningError("trapped: no escape path from rain zone")


def _cut_off(fm: FlightMap, cell: Cell, anchor: tuple[float, float]) -> bool:
    """True if ``cell`` cannot reach ``anchor`` on the rain-free flight map.

    Rain is ignored on purpose: an active rain zone between the drone and home is temporary
    and must not trigger a rescue path through it.
    """
    home = fm.cell(*anchor)
    if not fm.hard_ok[home] or not fm.hard_ok[cell]:
        return False
    return not bool(reachable(fm.hard_ok, cell)[home])


def _rescue(start: tuple[float, float, float], fm: FlightMap, rescue: FlightMap,
            anchor: tuple[float, float]) -> tuple[list[_Pt], Cell]:
    """Path on the reduced-clearance map into the flyable area that is connected to ``anchor``."""
    anchor_cell = fm.cell(*anchor)
    home_area = reachable(fm.flyable if fm.flyable[anchor_cell] else fm.hard_ok, anchor_cell)
    if not home_area.any():
        raise PlanningError("home area is not flyable")
    cell = rescue.cell(start[0], start[1])
    gx, gy = fm.spec.centers()
    if not rescue.hard_ok[cell]:
        d0 = np.where(rescue.hard_ok, np.hypot(gx - start[0], gy - start[1]), np.inf)
        if not np.isfinite(d0).any():
            raise PlanningError("no flyable cell in the rescue map")
        idx = int(np.argmin(d0))
        cell = (idx // fm.spec.nx, idx % fm.spec.nx)
    targets = home_area & reachable(rescue.hard_ok, cell)
    d = np.where(targets, np.hypot(gx - start[0], gy - start[1]), np.inf)
    for idx in np.argsort(d, axis=None)[:5]:
        if not np.isfinite(d.flat[idx]):
            break
        target = (int(idx // fm.spec.nx), int(idx % fm.spec.nx))
        path = astar(rescue.hard_ok, cell, target)
        if path is not None:
            xy = smooth(rescue, [rescue.spec.center(*c) for c in path], soft=True)
            pts: list[_Pt] = [(x, y, max(rescue.transit_alt(*rescue.cell(x, y)), start[2]), "escape")
                              for x, y in xy]
            return pts, target
    raise PlanningError("trapped: no rescue path back to the home area")


def _row_segments(axis: str, fm: FlightMap, hm: HeightMap, reach: BoolArray, need: BoolArray,
                  cfg: Config) -> list[list[_Sample]]:
    _spec, cam = fm.spec, cfg.camera
    hx, hy = cam.half_extents(fm.cruise_alt)
    across = 2 * (hy if axis == "x" else hx) * (1 - cfg.planner.overlap)
    radius = cfg.arena.diameter / 2
    n_rows = max(math.ceil(2 * radius / across), 1)
    offsets = (np.arange(n_rows) - (n_rows - 1) / 2) * across
    along = np.arange(-radius, radius + 1e-9, cfg.planner.sample_step)
    segments: list[list[_Sample]] = []
    for k, off in enumerate(offsets):
        samples = along if k % 2 == 0 else along[::-1]
        current: list[_Sample] = []
        for a in samples:
            x, y = (a, off) if axis == "x" else (off, a)
            sample = _sample(x, y, fm, hm, reach, need, cfg)
            if sample is not None and current:
                ok, _ = fm.segment_profile((current[-1].x, current[-1].y), (x, y))
                if not ok:
                    segments.append(current)
                    current = []
            if sample is None:
                if current:
                    segments.append(current)
                current = []
            else:
                current.append(sample)
        if current:
            segments.append(current)
    return segments


def _sample(x: float, y: float, fm: FlightMap, hm: HeightMap, reach: BoolArray, need: BoolArray,
            cfg: Config) -> _Sample | None:
    if not fm.spec.contains(x, y):
        return None
    cell = fm.spec.index(x, y)
    if not reach[cell]:
        return None
    z = fm.transit_alt(*cell)
    iy, ix = visible_cells((x, y, z), hm.height, fm.spec, cfg.camera)
    if not need[iy, ix].any():
        return None
    return _Sample(x, y, z, (iy, ix))


def _order(groups: list[list[_Sample]], start: tuple[float, float], fm: FlightMap, spec: object,
           kind: str = "sweep", k_eval: int = 6) -> tuple[list[_Pt], BoolArray]:
    """Greedy nearest-first ordering of sample groups; returns route points and predicted coverage."""
    predicted = np.zeros(fm.spec.shape, dtype=bool)
    route: list[_Pt] = []
    pos = start
    todo = [g for g in groups if g]
    while todo:
        ends: list[tuple[float, int, bool]] = []
        for gi, g in enumerate(todo):
            for rev in (False, True) if len(g) > 1 else (False,):
                head = g[-1] if rev else g[0]
                ends.append((math.hypot(head.x - pos[0], head.y - pos[1]), gi, rev))
        ends.sort()
        best: tuple[float, int, bool, list[tuple[float, float]]] | None = None
        for evaluated, (lower_bound, gi, rev) in enumerate(ends):
            # Straight-line distance is a lower bound on path length: stop once it
            # cannot beat the best path found so far.
            if best is not None and (lower_bound >= best[0] - 1e-9 or evaluated >= k_eval):
                break
            head = todo[gi][-1] if rev else todo[gi][0]
            path = _connect(pos, (head.x, head.y), fm)
            if path is None:
                continue
            length = _polyline_length(path)
            if best is None or length < best[0]:
                best = (length, gi, rev, path)
        if best is None:
            # Remaining groups cannot be reached from here (map changed); drop them.
            break
        _, gi, rev, path = best
        group = todo.pop(gi)
        if rev:
            group = group[::-1]
        for x, y in path[1:-1]:
            route.append((x, y, fm.transit_alt(*fm.cell(x, y)), "transit"))
        for s in group:
            route.append((s.x, s.y, s.z, kind))
            predicted[s.vis] = True
        pos = (group[-1].x, group[-1].y)
    return route, predicted


def _connect(a: tuple[float, float], b: tuple[float, float], fm: FlightMap
             ) -> list[tuple[float, float]] | None:
    ok, peak = fm.segment_profile(a, b)
    za = fm.transit_alt(*fm.cell(*a))
    zb = fm.transit_alt(*fm.cell(*b))
    if ok and peak <= max(za, zb) + 1e-6:
        return [a, b]
    cells = astar(fm.flyable, fm.cell(*a), fm.cell(*b))
    if cells is None:
        return None
    pts = [a] + [fm.spec.center(*c) for c in cells[1:-1]] + [b]
    return smooth(fm, pts)


def _gap_fill(remaining: BoolArray, fm: FlightMap, hm: HeightMap, reach: BoolArray, cfg: Config
              ) -> tuple[list[_Sample], BoolArray]:
    """Greedy set cover of ``remaining``; also returns which remaining cells any candidate sees."""
    seeable = np.zeros(fm.spec.shape, dtype=bool)
    if not remaining.any():
        return [], seeable
    spec = fm.spec
    step = max(round(cfg.planner.gapfill_step / spec.res), 1)
    hx, hy = cfg.camera.half_extents(fm.max_alt)
    near = dilate(remaining, math.hypot(hx, hy) / spec.res)
    lattice = np.zeros(spec.shape, dtype=bool)
    lattice[step // 2 :: step, step // 2 :: step] = True
    cand_cells = np.argwhere(lattice & reach & near)
    candidates: list[_Sample] = []
    for iy, ix in cand_cells:
        x, y = spec.center(int(iy), int(ix))
        req = float(fm.required_alt[iy, ix])
        for z in sorted({quantize_alt(max(req, fm.cruise_alt)), quantize_alt(max(req, fm.min_alt))}):
            vis = visible_cells((x, y, z), hm.height, spec, cfg.camera)
            if remaining[vis].any():
                candidates.append(_Sample(x, y, z, vis))
                seeable[vis] = True
    chosen: list[_Sample] = []
    remaining = remaining.copy()
    while candidates and len(chosen) < 60:
        gains = [int(np.count_nonzero(remaining[c.vis])) for c in candidates]
        bi = int(np.argmax(gains))
        if gains[bi] < cfg.planner.gapfill_min_gain:
            break
        pick = candidates.pop(bi)
        chosen.append(pick)
        remaining[pick.vis] = False
    return chosen, seeable


def _observable_from(reach: BoolArray, need: BoolArray, fm: FlightMap, hm: HeightMap, cfg: Config
                     ) -> int:
    """Rough count of ``need`` cells observable from the reachable area (for reporting)."""
    near = dilate(reach, cfg.camera.half_extents(fm.max_alt)[0] / fm.spec.res)
    return int(np.count_nonzero(need & near))


# --------------------------------------------------------------------------- 3D expansion


def to_waypoints(points: list[_Pt], fm: FlightMap) -> list[Waypoint]:
    """Insert climb/descend points so that every horizontal leg clears the terrain below it."""
    if not points:
        return []
    out = [Waypoint(*points[0])]
    for p, q in itertools.pairwise(points):
        soft = q[3] == "escape" or p[3] == "escape"
        _, peak = fm.segment_profile((p[0], p[1]), (q[0], q[1]), soft=soft)
        z_seg = quantize_alt(max(p[2], q[2], peak if math.isfinite(peak) else max(p[2], q[2])))
        if z_seg > out[-1].z + 1e-6:
            out.append(Waypoint(p[0], p[1], z_seg, "climb"))
        out.append(Waypoint(q[0], q[1], z_seg, q[3]))
        if q[2] < z_seg - 1e-6:
            out.append(Waypoint(q[0], q[1], q[2], q[3]))
    return _simplify(out)


def _simplify(wps: list[Waypoint], tol: float = 0.05) -> list[Waypoint]:
    if len(wps) <= 2:
        return wps
    out = [wps[0]]
    for i in range(1, len(wps) - 1):
        a, b, c = np.array(out[-1].xyz), np.array(wps[i].xyz), np.array(wps[i + 1].xyz)
        if np.linalg.norm(b - a) < 1e-6:
            continue
        ac = c - a
        denom = float(np.dot(ac, ac))
        if denom > 1e-12:
            t = float(np.clip(np.dot(b - a, ac) / denom, 0.0, 1.0))
            if np.linalg.norm(a + t * ac - b) < tol and 0.0 < t < 1.0:
                continue
        out.append(wps[i])
    if np.linalg.norm(np.array(wps[-1].xyz) - np.array(out[-1].xyz)) > 1e-6:
        out.append(wps[-1])
    return out


def estimate_time(wps: list[Waypoint], start: tuple[float, float, float], fl: FlightConfig) -> float:
    total = 0.0
    prev = start
    for w in wps:
        horiz = math.hypot(w.x - prev[0], w.y - prev[1]) / fl.max_speed
        vert = abs(w.z - prev[2]) / fl.max_climb
        total += max(horiz, vert) + fl.max_speed / fl.max_accel * 0.5
        prev = w.xyz
    return total


def _route_length(route: list[_Pt], start: tuple[float, float]) -> float:
    pts = [start] + [(p[0], p[1]) for p in route]
    return _polyline_length(pts)


def _polyline_length(pts: list[tuple[float, float]] | FloatArray) -> float:
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in itertools.pairwise(pts)))
