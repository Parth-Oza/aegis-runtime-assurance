from __future__ import annotations

import csv
import json
from dataclasses import replace
from pathlib import Path

import pytest

from aegis_rt import RuntimeAssuranceSupervisor
from aegis_rt.calibration import (
    empirical_collision_probability,
    fit_calibration,
    load_calibration,
    write_calibration,
)
from aegis_rt.scenario import load_scenario

SCENARIO = Path(__file__).parents[1] / "scenarios" / "warehouse_crossing.json"


def _write_dataset(path: Path) -> None:
    rows = [
        ("track-a", 1.0, 1.0, 1.0, 1.1, 1.0),
        ("track-b", 1.0, 2.0, 1.0, 2.2, 1.0),
        ("track-c", 1.0, 3.0, 1.0, 3.4, 1.0),
        ("track-a", 2.0, 2.0, 2.0, 2.3, 2.4),
        ("track-b", 2.0, 4.0, 2.0, 4.6, 2.0),
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "track_id",
            "horizon_s",
            "predicted_x_m",
            "predicted_y_m",
            "observed_x_m",
            "observed_y_m",
        ])
        writer.writerows(rows)


def test_fit_round_trip_and_coverage_report(tmp_path: Path) -> None:
    dataset = tmp_path / "residuals.csv"
    artifact = tmp_path / "calibration.json"
    _write_dataset(dataset)

    fitted = fit_calibration(dataset, target_coverage=0.8, source="synthetic-test")
    loaded = load_calibration(write_calibration(fitted, artifact))

    assert loaded == fitted
    assert loaded.sample_count == 5
    assert loaded.source == "synthetic-test"
    # Small per-horizon bins make nearest-rank coverage discrete; report the gap.
    assert loaded.empirical_coverage == pytest.approx(1.0)
    assert loaded.calibration_error == pytest.approx(0.2)
    assert json.loads(artifact.read_text())["schema_version"] == 1


def test_empirical_risk_is_deterministic_and_monotonic(tmp_path: Path) -> None:
    dataset = tmp_path / "residuals.csv"
    _write_dataset(dataset)
    calibration = fit_calibration(dataset, target_coverage=0.8)

    near = empirical_collision_probability(0.15, 1.0, calibration)
    far = empirical_collision_probability(0.5, 1.0, calibration)

    assert near > far
    assert far > 0.0
    assert near == empirical_collision_probability(0.15, 1.0, calibration)


def test_supervisor_can_compare_empirical_and_baseline_risk(tmp_path: Path) -> None:
    dataset = tmp_path / "residuals.csv"
    _write_dataset(dataset)
    calibration = fit_calibration(dataset, target_coverage=0.8)
    scenario = load_scenario(SCENARIO)

    baseline = RuntimeAssuranceSupervisor().evaluate(scenario)
    empirical = RuntimeAssuranceSupervisor().evaluate(
        replace(scenario, calibration=calibration)
    )

    assert baseline.commanded_evaluation.risk_samples[0].uncertainty_model == "radial_gaussian"
    assert empirical.commanded_evaluation.risk_samples[0].uncertainty_model == "empirical_residual"
    assert baseline.commanded_evaluation.max_collision_probability != empirical.commanded_evaluation.max_collision_probability


def test_scenario_loads_relative_calibration_artifact(tmp_path: Path) -> None:
    dataset = tmp_path / "residuals.csv"
    artifact = tmp_path / "calibration.json"
    _write_dataset(dataset)
    write_calibration(fit_calibration(dataset), artifact)
    payload = json.loads(SCENARIO.read_text(encoding="utf-8"))
    payload["uncertainty_calibration"] = "calibration.json"
    scenario_path = tmp_path / "scenario.json"
    scenario_path.write_text(json.dumps(payload), encoding="utf-8")

    scenario = load_scenario(scenario_path)

    assert scenario.calibration is not None
    assert scenario.calibration.sample_count == 5


def test_invalid_dataset_is_rejected(tmp_path: Path) -> None:
    dataset = tmp_path / "bad.csv"
    dataset.write_text("track_id,horizon_s\na,1.0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="missing columns"):
        fit_calibration(dataset)
