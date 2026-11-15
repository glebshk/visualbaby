from __future__ import annotations

import json
from pathlib import Path

import pytest

from aufdeckung.cli import main


def test_generate_and_reload(tmp_path: Path) -> None:
    assert main(["generate", "--preset", "street", "--seed", "1", "--sdf", "--plot",
                 "--out-dir", str(tmp_path)]) == 0
    for suffix in (".yaml", ".sdf", ".png"):
        assert (tmp_path / f"street_1{suffix}").exists()
    assert main(["plan", "--scenario", str(tmp_path / "street_1.yaml"), "--plot",
                 "--out-dir", str(tmp_path)]) == 0
    setpoints = json.loads((tmp_path / "street_1_setpoints_ned.json").read_text())
    assert setpoints and all({"x", "y", "z", "w"} <= s.keys() for s in setpoints)
    assert (tmp_path / "street_1.plan").exists()


def test_simulate_with_config_override(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    cfg = tmp_path / "cfg.yaml"
    cfg.write_text("flight:\n  max_speed: 4.0\n")
    assert main(["simulate", "--preset", "street", "--seed", "0", "--config", str(cfg),
                 "--plot", "--events", "--out-dir", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert '"collisions": 0' in out and "phase" in out
    assert (tmp_path / "street_0_run.png").exists()


def test_benchmark_cli(tmp_path: Path) -> None:
    report = tmp_path / "r.json"
    code = main(["benchmark", "--presets", "street", "--seeds", "1", "--workers", "1",
                 "--json", str(report)])
    data = json.loads(report.read_text())
    assert code == (0 if data["passed"] else 1)


def test_bad_arguments() -> None:
    with pytest.raises(SystemExit):
        main(["generate", "--preset", "moon"])
