"""Grid path search (A* with octile heuristic) and line-of-sight smoothing."""

from __future__ import annotations

import heapq
import math
from collections import deque

import numpy as np

from aufdeckung.geometry import BoolArray
from aufdeckung.planning.flightmap import FlightMap

Cell = tuple[int, int]
_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2)]


def reachable(free: BoolArray, start: Cell) -> BoolArray:
    """Cells 8-connected to ``start`` through ``free`` cells (no corner cutting)."""
    seen = np.zeros(free.shape, dtype=bool)
    ny, nx = free.shape
    if not (0 <= start[0] < ny and 0 <= start[1] < nx) or not free[start]:
        return seen
    seen[start] = True
    queue = deque([start])
    while queue:
        cy, cx = queue.popleft()
        for dy, dx, _ in _NEIGHBOURS:
            ny_, nx_ = cy + dy, cx + dx
            if not (0 <= ny_ < ny and 0 <= nx_ < nx) or seen[ny_, nx_] or not free[ny_, nx_]:
                continue
            if dy and dx and not (free[cy + dy, cx] and free[cy, cx + dx]):
                continue
            seen[ny_, nx_] = True
            queue.append((ny_, nx_))
    return seen


def astar(free: BoolArray, start: Cell, goal: Cell) -> list[Cell] | None:
    """Shortest 8-connected path from ``start`` to ``goal`` (inclusive) or None."""
    ny, nx = free.shape
    if not free[start] or not free[goal]:
        return None
    if start == goal:
        return [start]

    def h(c: Cell) -> float:
        dy, dx = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return (dx + dy) + (_SQRT2 - 2) * min(dx, dy)

    g = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    heap = [(h(start), 0.0, start)]
    closed: set[Cell] = set()
    while heap:
        _, gc, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            return path[::-1]
        closed.add(cur)
        cy, cx = cur
        for dy, dx, w in _NEIGHBOURS:
            nb = (cy + dy, cx + dx)
            if not (0 <= nb[0] < ny and 0 <= nb[1] < nx) or not free[nb] or nb in closed:
                continue
            if dy and dx and not (free[cy + dy, cx] and free[cy, cx + dx]):
                continue
            ng = gc + w
            if ng < g.get(nb, math.inf):
                g[nb] = ng
                parent[nb] = cur
                heapq.heappush(heap, (ng + h(nb), ng, nb))
    return None


def smooth(fm: FlightMap, path: list[tuple[float, float]], soft: bool = False
           ) -> list[tuple[float, float]]:
    """Greedy forward line-of-sight shortcutting that never flies higher than the endpoints need."""
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = i + 1
        zi = max(fm.cruise_alt, _req(fm, path[i]))
        while j + 1 < len(path):
            ok, peak = fm.segment_profile(path[i], path[j + 1], soft=soft)
            zj = max(fm.cruise_alt, _req(fm, path[j + 1]))
            if not (ok and peak <= max(zi, zj) + 1e-6):
                break
            j += 1
        out.append(path[j])
        i = j
    return out


def _req(fm: FlightMap, p: tuple[float, float]) -> float:
    iy, ix = fm.cell(*p)
    return float(fm.required_alt[iy, ix])
