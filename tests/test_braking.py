from __future__ import annotations

import csv
from pathlib import Path

import pytest

from aegis_rt.braking import (
    DriftLimits,
    compare_braking_calibrations,
    fit_braking_calibration,
    load_braking_calibration,
    review_braking_calibration,
    safety_config_from_braking_artifact,
    write_braking_calibration,
)


def _write_run(path: Path, *, slow: bool = False) -> None:
    speeds = [2.0, 2.0, 1.9, 1.4, 0.8, 0.2, 0.0]
    if slow:
        speeds = [2.0, 2.0, 2.0, 1.9, 1.6, 1.2, 0.8, 0.4, 0.0]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "run_id",
            "time_s",
            "command_speed_mps",
            "odometry_speed_mps",
            "stop_event",
        ])
        for run_index in range(4):
            for index, speed in enumerate(speeds):
                writer.writerow([
                    f"run-{run_index}",
                    index * 0.1,
                    2.0 if index == 0 else 0.0,
                    speed,
                    1 if index == 1 else 0,
                ])


def test_fit_round_trip_requires_review_before_runtime_use(tmp_path: Path) -> None:
    dataset = tmp_path / "braking.csv"
    artifact = tmp_path / "braking.json"
    _write_run(dataset)

    fitted = fit_braking_calibration(dataset, target_coverage=0.8)
    loaded = load_braking_calibration(write_braking_calibration(fitted, artifact))

    assert loaded == fitted
    assert loaded.run_count == 4
    assert loaded.reaction_time_s == pytest.approx(0.1)
    assert loaded.max_braking_mps2 > 0.0
    assert loaded.stopping_distance_m > 0.0
    with pytest.raises(ValueError, match="reviewed"):
        safety_config_from_braking_artifact(loaded)

    reviewed = review_braking_calibration(
        loaded,
        reviewer="safety-reviewer",
        evidence="review/BRK-42",
        reviewed_at="2026-10-18T00:00:00+00:00",
    )
    config = safety_config_from_braking_artifact(reviewed)
    assert config.reaction_time_s == reviewed.reaction_time_s
    assert config.max_braking_mps2 == reviewed.max_braking_mps2


def test_drift_limits_reject_degraded_envelope(tmp_path: Path) -> None:
    baseline_path = tmp_path / "baseline.csv"
    degraded_path = tmp_path / "degraded.csv"
    _write_run(baseline_path)
    _write_run(degraded_path, slow=True)
    baseline = fit_braking_calibration(baseline_path, target_coverage=0.8)
    degraded = fit_braking_calibration(degraded_path, target_coverage=0.8)

    report = compare_braking_calibrations(
        degraded,
        baseline,
        DriftLimits(
            max_reaction_time_increase_s=0.05,
            max_deceleration_decrease_fraction=0.05,
            max_stopping_distance_increase_m=0.05,
        ),
    )

    assert not report.passed
    assert "reaction_time" in report.violations
    assert "deceleration" in report.violations
    assert "stopping_distance" in report.violations


def test_invalid_or_incomplete_run_is_rejected(tmp_path: Path) -> None:
    dataset = tmp_path / "bad.csv"
    dataset.write_text(
        "run_id,time_s,command_speed_mps,odometry_speed_mps,stop_event\n"
        "run-1,0.0,1.0,1.0,0\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="exactly one stop_event"):
        fit_braking_calibration(dataset)
