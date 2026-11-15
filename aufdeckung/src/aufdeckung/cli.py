"""Command line interface: ``python -m aufdeckung <command>``."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import yaml

from aufdeckung.config import Config
from aufdeckung.scenario.generator import PRESETS, generate
from aufdeckung.scenario.io import load_scenario, save_scenario
from aufdeckung.scenario.model import Scenario


def _config(path: str | None) -> Config:
    cfg = Config()
    if path:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        cfg = cfg.with_overrides(data)
    return cfg


def _scenario(args: argparse.Namespace, cfg: Config) -> Scenario:
    if getattr(args, "scenario", None):
        sc = load_scenario(args.scenario)
        sc.validate(cfg.arena)
        return sc
    return generate(args.preset, args.seed, cfg.arena)


def _add_scenario_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--scenario", help="scenario YAML file (overrides --preset/--seed)")
    p.add_argument("--preset", default="mixed", choices=sorted(PRESETS))
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--config", help="YAML file with config overrides, e.g. {flight: {cruise_alt: 7}}")


def cmd_generate(args: argparse.Namespace) -> int:
    cfg = _config(args.config)
    sc = _scenario(args, cfg)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    save_scenario(sc, out / f"{sc.name}.yaml")
    print(f"scenario: {out / (sc.name + '.yaml')}")
    if args.sdf:
        from aufdeckung.export.sdf import write_world

        print(f"world:    {write_world(sc, cfg.arena, out / f'{sc.name}.sdf', args.world_name)}")
    if args.plot:
        from aufdeckung.viz import _plt, _save, draw_scenario

        plt = _plt()
        fig, ax = plt.subplots(figsize=(7, 7))
        draw_scenario(ax, sc, cfg)
        ax.set_title(sc.name)
        print(f"plot:     {_save(fig, out / f'{sc.name}.png', plt)}")
    return 0


def cmd_plan(args: argparse.Namespace) -> int:
    """Offline plan on a known layout (e.g. the organisers' plan), exported for PX4/QGC."""
    from aufdeckung.export.waypoints import to_ned_setpoints, to_qgc_plan, write_json
    from aufdeckung.mapping.grid import GridSpec, HeightMap, cage_mask
    from aufdeckung.planning.coverage_planner import plan_coverage
    from aufdeckung.planning.flightmap import build_flight_map

    cfg = _config(args.config)
    sc = _scenario(args, cfg)
    spec = GridSpec.for_arena(cfg.arena, cfg.planner.grid_res)
    hm = HeightMap.from_prior(spec, cfg.arena, sc.obstacles)
    fm = build_flight_map(hm, cfg, (), assume_unknown_below_cruise=False)
    plan = plan_coverage((sc.start[0], sc.start[1], 0.0), hm, fm, cage_mask(spec, cfg.arena), cfg)
    out = Path(args.out_dir)
    write_json(to_ned_setpoints(plan.waypoints, sc.start), out / f"{sc.name}_setpoints_ned.json")
    write_json(to_qgc_plan(plan.waypoints, cfg.arena, cfg.flight, sc.start), out / f"{sc.name}.plan")
    print(f"{len(plan.waypoints)} waypoints, sweep axis {plan.axis}, est. {plan.est_time:.0f} s, "
          f"altitudes {min(w.z for w in plan.waypoints):.2f}-{max(w.z for w in plan.waypoints):.2f} m")
    print(f"written to {out}/")
    if args.plot:
        from aufdeckung.viz import plot_plan

        print(f"plot: {plot_plan(sc, cfg, plan.waypoints, out / f'{sc.name}_plan.png')}")
    return 0


def cmd_simulate(args: argparse.Namespace) -> int:
    from aufdeckung.sim.runner import run_mission

    cfg = _config(args.config)
    sc = _scenario(args, cfg)
    result = run_mission(sc, cfg, use_prior=args.prior)
    print(json.dumps(result.metrics, indent=2, default=_jsonable))
    if args.events:
        for e in result.controller.events:
            print(f"{e.t:8.1f}s  {e.kind:<14} {e.detail}")
    if args.plot:
        from aufdeckung.viz import plot_run

        print(f"plot: {plot_run(result, cfg, Path(args.out_dir) / f'{sc.name}_run.png')}")
    return 0 if result.metrics["collisions"] == 0 else 1


def cmd_benchmark(args: argparse.Namespace) -> int:
    from aufdeckung.sim.benchmark import format_summary, run_benchmark

    cfg = _config(args.config)
    presets = sorted(PRESETS) if args.presets == "all" else args.presets.split(",")
    seeds = list(range(args.first_seed, args.first_seed + args.seeds))
    report = run_benchmark(presets, seeds, cfg, prior=args.prior, workers=args.workers)
    print(format_summary(report))
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps(report, indent=2, default=_jsonable), encoding="utf-8")
        print(f"report: {args.json}")
    return 0 if report["passed"] else 1


def _jsonable(o: Any) -> Any:
    if hasattr(o, "item"):
        return o.item()
    return str(o)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="aufdeckung", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("generate", help="generate a random arena (YAML, optional SDF world and plot)")
    _add_scenario_args(p)
    p.add_argument("--out-dir", default="out")
    p.add_argument("--sdf", action="store_true", help="also write a Gazebo world")
    p.add_argument("--world-name", default="aufdeckung")
    p.add_argument("--plot", action="store_true")
    p.set_defaults(func=cmd_generate)

    p = sub.add_parser("plan", help="offline coverage plan on a known layout, exported for PX4/QGC")
    _add_scenario_args(p)
    p.add_argument("--out-dir", default="out")
    p.add_argument("--plot", action="store_true")
    p.set_defaults(func=cmd_plan)

    p = sub.add_parser("simulate", help="closed-loop headless simulation of one arena")
    _add_scenario_args(p)
    p.add_argument("--prior", action="store_true", help="give the planner the true layout up front")
    p.add_argument("--plot", action="store_true")
    p.add_argument("--events", action="store_true", help="print the mission event log")
    p.add_argument("--out-dir", default="out")
    p.set_defaults(func=cmd_simulate)

    p = sub.add_parser("benchmark", help="many random arenas per preset, checked against criteria")
    p.add_argument("--presets", default="all", help="'all' or comma list of " + ",".join(sorted(PRESETS)))
    p.add_argument("--seeds", type=int, default=10)
    p.add_argument("--first-seed", type=int, default=0)
    p.add_argument("--prior", action="store_true")
    p.add_argument("--workers", type=int, default=None)
    p.add_argument("--json", help="write the full report to this file")
    p.add_argument("--config")
    p.set_defaults(func=cmd_benchmark)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    code: int = args.func(args)
    return code


if __name__ == "__main__":
    sys.exit(main())
