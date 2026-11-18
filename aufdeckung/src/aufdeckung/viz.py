"""Top-down plots of scenarios, plans and simulation runs (needs the ``viz`` extra)."""

from __future__ import annotations

import math
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from aufdeckung.config import Config
from aufdeckung.geometry import Box, Cylinder, regular_polygon
from aufdeckung.scenario.model import Scenario

if TYPE_CHECKING:
    from aufdeckung.planning.coverage_planner import Waypoint
    from aufdeckung.sim.runner import RunResult

KIND_COLORS = {"building": "#8d8478", "vehicle": "#2b4a9b", "tree": "#2e7d32", "pole": "#444444",
               "furniture": "#7a5c3e", "feature": "#64b5f6"}
TARGET_COLORS = {"class1": "#d32f2f", "class2": "#1565c0", "special": "#ff8f00"}


def _plt() -> Any:
    import warnings

    import matplotlib

    matplotlib.use("Agg")
    with warnings.catch_warnings():
        # In the ROS image, Ubuntu's old mpl_toolkits sits next to the pip matplotlib, so the 3D
        # projection cannot load. The plots here are 2D, so that warning is noise.
        warnings.filterwarnings("ignore", message="Unable to import Axes3D")
        import matplotlib.pyplot as plt

    return plt


def draw_scenario(ax: Any, sc: Scenario, cfg: Config) -> None:
    from matplotlib.patches import Circle, Polygon

    poly = regular_polygon(cfg.arena.diameter, cfg.arena.n_sides)
    ax.add_patch(Polygon(poly, closed=True, fill=False, ec="black", lw=1.5))
    for o in sorted(sc.obstacles, key=lambda s: s.top):
        color = KIND_COLORS.get(o.kind, "#999999")
        alpha = 0.35 + 0.6 * min(o.top / cfg.arena.height, 1.0)
        if isinstance(o, Box):
            c, s = math.cos(o.yaw), math.sin(o.yaw)
            lx = np.array([-1, 1, 1, -1]) * o.sx / 2
            ly = np.array([-1, -1, 1, 1]) * o.sy / 2
            pts = np.stack([o.x + c * lx - s * ly, o.y + s * lx + c * ly], axis=1)
            ax.add_patch(Polygon(pts, closed=True, fc=color, ec="black", lw=0.4, alpha=alpha))
            if o.kind == "building":
                ax.text(o.x, o.y, f"{o.top:.0f}", ha="center", va="center", fontsize=6)
        elif isinstance(o, Cylinder):
            ax.add_patch(Circle((o.x, o.y), o.radius, fc=color, ec="black", lw=0.3, alpha=alpha))
    for z in sc.rain_zones:
        ax.add_patch(Circle((z.x, z.y), z.radius, fc="#64b5f6", ec="#1e88e5", alpha=0.25, ls="--"))
        ax.text(z.x, z.y, f"rain\n{z.t_on:.0f}-{z.t_off:.0f}s", ha="center", va="center", fontsize=6)
    for t in sc.targets:
        ax.plot(t.x, t.y, marker="*" if t.cls == "special" else "s", ms=7,
                color=TARGET_COLORS[t.cls], mec="white", mew=0.5, ls="none")
    ax.plot(*sc.start, marker="^", color="black", ms=8, ls="none")
    r = cfg.arena.diameter / 2 + 1
    ax.set_xlim(-r, r)
    ax.set_ylim(-r, r)
    ax.set_aspect("equal")
    ax.set_xlabel("x east [m]")
    ax.set_ylabel("y north [m]")


def plot_plan(sc: Scenario, cfg: Config, waypoints: list[Waypoint], path: str | Path) -> Path:
    plt = _plt()
    fig, ax = plt.subplots(figsize=(7, 7))
    draw_scenario(ax, sc, cfg)
    xs = [sc.start[0]] + [w.x for w in waypoints]
    ys = [sc.start[1]] + [w.y for w in waypoints]
    zs = [0.0] + [w.z for w in waypoints]
    ax.plot(xs, ys, color="#555555", lw=0.6)
    sca = ax.scatter(xs, ys, c=zs, cmap="viridis", s=10, vmin=cfg.flight.min_alt,
                     vmax=cfg.flight.max_alt, zorder=5)
    fig.colorbar(sca, ax=ax, label="flight altitude [m]", shrink=0.8)
    ax.set_title(f"{sc.name}: planned route ({len(waypoints)} waypoints)")
    return _save(fig, path, plt)


def plot_run(result: RunResult, cfg: Config, path: str | Path) -> Path:
    plt = _plt()
    sc, m = result.scenario, result.metrics
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    ax = axes[0]
    draw_scenario(ax, sc, cfg)
    tr = result.trajectory
    if len(tr):
        sca = ax.scatter(tr[:, 1], tr[:, 2], c=tr[:, 3], cmap="viridis", s=2, vmin=0,
                         vmax=cfg.flight.max_alt, zorder=5)
        fig.colorbar(sca, ax=ax, label="altitude [m]", shrink=0.8)
    for rep in result.reports:
        ax.plot(*rep.reported_xy, marker="x", color="black", ms=6)
    ax.set_title(f"{sc.name} ({m['mode']}): flown path")

    ax = axes[1]
    spec = result.controller.spec
    first = np.where(result.observable, result.coverage.first_seen, np.nan)
    extent = (spec.xmin, spec.xmin + spec.nx * spec.res, spec.ymin, spec.ymin + spec.ny * spec.res)
    img = ax.imshow(first, origin="lower", extent=extent, cmap="magma_r", vmin=0,
                    vmax=cfg.mission.time_budget)
    missed = result.observable & ~result.coverage.covered
    my, mx = np.nonzero(missed)
    ax.scatter(spec.xmin + (mx + 0.5) * spec.res, spec.ymin + (my + 0.5) * spec.res, s=2, c="cyan",
               label="observable but missed")
    fig.colorbar(img, ax=ax, label="first imaged at [s]", shrink=0.8)
    ax.set_aspect("equal")
    ax.set_title(f"coverage {m['coverage']:.1%} (at {cfg.mission.bonus_window:.0f}s: "
                 f"{m['coverage_bonus']:.1%}), targets {m['targets_found']}/{m['targets']}, "
                 f"collisions {m['collisions']}")
    ax.legend(loc="lower right", fontsize=7)
    return _save(fig, path, plt)


def _save(fig: Any, path: str | Path, plt: Any) -> Path:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out
