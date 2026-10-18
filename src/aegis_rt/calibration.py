from __future__ import annotations

import csv
import json
from dataclasses import asdict
from math import ceil, hypot, isfinite
from pathlib import Path

from .models import EmpiricalCalibration, ResidualBin

REQUIRED_COLUMNS = {
    "track_id",
    "horizon_s",
    "predicted_x_m",
    "predicted_y_m",
    "observed_x_m",
    "observed_y_m",
}


def _nearest_rank(values: tuple[float, ...], probability: float) -> float:
    index = max(0, min(len(values) - 1, ceil(probability * len(values)) - 1))
    return values[index]


def fit_calibration(
    path: str | Path,
    *,
    target_coverage: float = 0.95,
    source: str | None = None,
) -> EmpiricalCalibration:
    if not 0.0 < target_coverage < 1.0:
        raise ValueError("target_coverage must be between zero and one")
    dataset = Path(path)
    by_horizon: dict[float, list[float]] = {}
    with dataset.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = REQUIRED_COLUMNS - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"dataset is missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            try:
                horizon = float(row["horizon_s"])
                residual = hypot(
                    float(row["observed_x_m"]) - float(row["predicted_x_m"]),
                    float(row["observed_y_m"]) - float(row["predicted_y_m"]),
                )
            except (TypeError, ValueError) as error:
                raise ValueError(f"invalid numeric value on line {line_number}") from error
            if horizon < 0.0 or not isfinite(horizon) or not isfinite(residual):
                raise ValueError(f"non-finite or negative value on line {line_number}")
            by_horizon.setdefault(horizon, []).append(residual)
    if not by_horizon:
        raise ValueError("dataset must contain at least one trajectory residual")

    bins: list[ResidualBin] = []
    covered = 0
    sample_count = 0
    for horizon, residuals in sorted(by_horizon.items()):
        ordered = tuple(sorted(residuals))
        quantile = _nearest_rank(ordered, target_coverage)
        covered += sum(value <= quantile for value in ordered)
        sample_count += len(ordered)
        bins.append(
            ResidualBin(
                horizon_s=horizon,
                residuals_m=ordered,
                quantile_m=quantile,
            )
        )
    empirical_coverage = covered / sample_count
    return EmpiricalCalibration(
        schema_version=1,
        source=source or dataset.name,
        target_coverage=target_coverage,
        empirical_coverage=empirical_coverage,
        calibration_error=abs(empirical_coverage - target_coverage),
        sample_count=sample_count,
        bins=tuple(bins),
    )


def write_calibration(calibration: EmpiricalCalibration, path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(asdict(calibration), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return target


def load_calibration(path: str | Path) -> EmpiricalCalibration:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported calibration schema_version")
    bins = tuple(
        ResidualBin(
            horizon_s=float(item["horizon_s"]),
            residuals_m=tuple(float(value) for value in item["residuals_m"]),
            quantile_m=float(item["quantile_m"]),
        )
        for item in payload["bins"]
    )
    if not bins or any(not item.residuals_m for item in bins):
        raise ValueError("calibration must contain non-empty residual bins")
    return EmpiricalCalibration(
        schema_version=1,
        source=str(payload["source"]),
        target_coverage=float(payload["target_coverage"]),
        empirical_coverage=float(payload["empirical_coverage"]),
        calibration_error=float(payload["calibration_error"]),
        sample_count=int(payload["sample_count"]),
        bins=bins,
    )


def empirical_collision_probability(
    clearance_m: float,
    horizon_s: float,
    calibration: EmpiricalCalibration,
) -> float:
    """Estimate collision risk from the nearest recorded residual-horizon bin."""
    if clearance_m <= 0.0:
        return 1.0
    residual_bin = min(
        calibration.bins,
        key=lambda item: (abs(item.horizon_s - horizon_s), item.horizon_s),
    )
    exceedances = sum(value >= clearance_m for value in residual_bin.residuals_m)
    # Add-one smoothing prevents unsupported zero-risk claims with finite data.
    return (exceedances + 1) / (len(residual_bin.residuals_m) + 1)
