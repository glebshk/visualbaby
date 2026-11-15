"""YAML (de)serialisation of scenarios."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from aufdeckung.geometry import Box, Cylinder, Shape
from aufdeckung.scenario.model import RainZone, Scenario, ScenarioError, Target


def _num(v: float) -> float:
    return float(v)


def scenario_to_dict(sc: Scenario) -> dict[str, Any]:
    """Plain-python dict (YAML/JSON safe; numpy scalars are converted)."""
    obstacles: list[dict[str, Any]] = []
    for o in sc.obstacles:
        if isinstance(o, Box):
            obstacles.append(
                {"type": "box", "x": _num(o.x), "y": _num(o.y), "sx": _num(o.sx), "sy": _num(o.sy),
                 "height": _num(o.height), "yaw": _num(o.yaw), "z0": _num(o.z0), "kind": o.kind,
                 "name": o.name}
            )
        else:
            obstacles.append(
                {"type": "cylinder", "x": _num(o.x), "y": _num(o.y), "radius": _num(o.radius),
                 "height": _num(o.height), "z0": _num(o.z0), "kind": o.kind, "name": o.name}
            )
    return {
        "name": sc.name,
        "preset": sc.preset,
        "seed": sc.seed,
        "ground": sc.ground,
        "start": [_num(sc.start[0]), _num(sc.start[1])],
        "obstacles": obstacles,
        "targets": [
            {"x": _num(t.x), "y": _num(t.y), "cls": t.cls, "name": t.name, "size": _num(t.size)}
            for t in sc.targets
        ],
        "rain_zones": [
            {"x": _num(z.x), "y": _num(z.y), "radius": _num(z.radius), "t_on": _num(z.t_on),
             "t_off": None if math.isinf(z.t_off) else _num(z.t_off)}
            for z in sc.rain_zones
        ],
        "notes": dict(sc.notes),
    }


def scenario_from_dict(data: dict[str, Any]) -> Scenario:
    try:
        obstacles: list[Shape] = []
        for o in data.get("obstacles", []) or []:
            kind = o.get("type", "box")
            common: dict[str, Any] = {"x": float(o["x"]), "y": float(o["y"]), "height": float(o["height"]),
                      "z0": float(o.get("z0", 0.0)), "name": str(o.get("name", ""))}
            if kind == "box":
                obstacles.append(Box(sx=float(o["sx"]), sy=float(o["sy"]), yaw=float(o.get("yaw", 0.0)),
                                     kind=str(o.get("kind", "building")), **common))
            elif kind == "cylinder":
                obstacles.append(Cylinder(radius=float(o["radius"]), kind=str(o.get("kind", "pole")),
                                          **common))
            else:
                raise ScenarioError(f"unknown obstacle type {kind!r}")
        targets = tuple(
            Target(x=float(t["x"]), y=float(t["y"]), cls=str(t.get("cls", "class2")),
                   name=str(t.get("name", "")), size=float(t.get("size", 0.5)))
            for t in data.get("targets", []) or []
        )
        rain = tuple(
            RainZone(x=float(z["x"]), y=float(z["y"]), radius=float(z["radius"]),
                     t_on=float(z.get("t_on", 0.0)),
                     t_off=math.inf if z.get("t_off") is None else float(z["t_off"]))
            for z in data.get("rain_zones", []) or []
        )
        start = data.get("start", [0.0, -18.0])
        seed = data.get("seed")
        return Scenario(
            name=str(data.get("name", "unnamed")),
            obstacles=tuple(obstacles),
            targets=targets,
            rain_zones=rain,
            start=(float(start[0]), float(start[1])),
            preset=str(data.get("preset", "custom")),
            seed=None if seed is None else int(seed),
            ground=str(data.get("ground", "grass")),
            notes={str(k): str(v) for k, v in (data.get("notes") or {}).items()},
        )
    except (KeyError, TypeError, ValueError) as exc:
        if isinstance(exc, ScenarioError):
            raise
        raise ScenarioError(f"malformed scenario data: {exc}") from exc


def save_scenario(sc: Scenario, path: str | Path) -> None:
    Path(path).write_text(yaml.safe_dump(scenario_to_dict(sc), sort_keys=False), encoding="utf-8")


def load_scenario(path: str | Path) -> Scenario:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ScenarioError(f"{path}: expected a mapping at top level")
    return scenario_from_dict(data)
