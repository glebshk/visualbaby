"""Randomised benchmark: many layouts per preset, pass/fail against acceptance criteria."""

from __future__ import annotations

import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from typing import Any

from aufdeckung.config import Config
from aufdeckung.scenario.generator import generate
from aufdeckung.sim.runner import run_mission


@dataclass(frozen=True)
class Criteria:
    min_coverage: float = 0.95
    min_clearance: float = 0.75
    max_collisions: int = 0

    def failures(self, m: dict[str, Any], cfg: Config) -> list[str]:
        out: list[str] = []
        if m["collisions"] > self.max_collisions:
            out.append(f"{m['collisions']} collision(s)")
        if m["min_clearance"] is not None and m["min_clearance"] < self.min_clearance:
            out.append(f"clearance {m['min_clearance']:.2f} m")
        if m["coverage_time_budget"] < self.min_coverage:
            out.append(f"coverage {m['coverage_time_budget']:.1%} within budget")
        if not (m["completed"] and m["landed_home"]):
            out.append("did not land at home")
        if m["mission_time"] > cfg.mission.time_budget:
            out.append(f"mission took {m['mission_time']:.0f} s")
        return out


def _job(args: tuple[str, int, Config, bool]) -> dict[str, Any]:
    preset, seed, cfg, prior = args
    sc = generate(preset, seed, cfg.arena)
    metrics = run_mission(sc, cfg, use_prior=prior).metrics
    metrics["preset"] = preset
    metrics["seed"] = seed
    return metrics


def run_benchmark(
    presets: list[str],
    seeds: list[int],
    cfg: Config,
    prior: bool = False,
    workers: int | None = None,
    criteria: Criteria | None = None,
) -> dict[str, Any]:
    criteria = criteria or Criteria()
    jobs = [(p, s, cfg, prior) for p in presets for s in seeds]
    workers = workers or max((os.cpu_count() or 1), 1)
    if workers > 1 and len(jobs) > 1:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            results = list(pool.map(_job, jobs))
    else:
        results = [_job(j) for j in jobs]
    for m in results:
        m["failures"] = criteria.failures(m, cfg)
    per_preset: dict[str, dict[str, Any]] = {}
    for p in presets:
        rows = [m for m in results if m["preset"] == p]
        per_preset[p] = {
            "runs": len(rows),
            "passed": sum(1 for m in rows if not m["failures"]),
            "coverage_mean": _mean([m["coverage"] for m in rows]),
            "coverage_min": min((m["coverage"] for m in rows), default=float("nan")),
            "coverage_bonus_mean": _mean([m["coverage_bonus"] for m in rows]),
            "targets_found": sum(m["targets_found"] for m in rows),
            "targets_observable": sum(m["targets_observable"] for m in rows),
            "collisions": sum(m["collisions"] for m in rows),
            "min_clearance": min((m["min_clearance"] for m in rows if m["min_clearance"] is not None),
                                 default=float("nan")),
            "mission_time_mean": _mean([m["mission_time"] for m in rows]),
        }
    return {
        "mode": "prior" if prior else "explore",
        "criteria": criteria.__dict__,
        "passed": all(not m["failures"] for m in results),
        "per_preset": per_preset,
        "runs": results,
    }


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else float("nan")


def format_summary(report: dict[str, Any]) -> str:
    lines = [f"mode: {report['mode']}   overall: {'PASS' if report['passed'] else 'FAIL'}",
             f"{'preset':<14}{'pass':>8}{'cov mean':>10}{'cov min':>9}{'cov@180s':>10}"
             f"{'targets':>10}{'coll':>6}{'min clr':>9}{'time':>7}"]
    for p, s in report["per_preset"].items():
        lines.append(
            f"{p:<14}{s['passed']:>4}/{s['runs']:<3}{s['coverage_mean']:>10.1%}{s['coverage_min']:>9.1%}"
            f"{s['coverage_bonus_mean']:>10.1%}{s['targets_found']:>5}/{s['targets_observable']:<4}"
            f"{s['collisions']:>6}{s['min_clearance']:>9.2f}{s['mission_time_mean']:>7.0f}"
        )
    failed = [m for m in report["runs"] if m["failures"]]
    for m in failed:
        lines.append(f"  FAIL {m['scenario']}: {'; '.join(m['failures'])}")
    return "\n".join(lines)
