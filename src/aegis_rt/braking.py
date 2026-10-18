from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
from math import ceil, isfinite
from pathlib import Path
from typing import Any

from .models import SafetyConfig

REQUIRED_COLUMNS = {
    "run_id",
    "time_s",
    "command_speed_mps",
    "odometry_speed_mps",
    "stop_event",
}


@dataclass(frozen=True)
class BrakingSample:
    run_id: str
    time_s: float
    command_speed_mps: float
    odometry_speed_mps: float
    stop_event: bool


@dataclass(frozen=True)
class BrakingRunMetrics:
    run_id: str
    initial_speed_mps: float
    reaction_delay_s: float
    mean_deceleration_mps2: float
    stopping_distance_m: float
    stop_time_s: float


@dataclass(frozen=True)
class DriftReport:
    passed: bool
    reaction_time_increase_s: float
    deceleration_decrease_fraction: float
    stopping_distance_increase_m: float
    violations: tuple[str, ...]


@dataclass(frozen=True)
class BrakingCalibration:
    schema_version: int
    source: str
    target_coverage: float
    run_count: int
    reaction_time_s: float
    max_braking_mps2: float
    stopping_distance_m: float
    runs: tuple[BrakingRunMetrics, ...]
    safety_config: dict[str, float]
    review_status: str = "unreviewed"
    reviewed_by: str | None = None
    reviewed_at: str | None = None
    review_evidence: str | None = None
    drift: DriftReport | None = None


@dataclass(frozen=True)
class DriftLimits:
    max_reaction_time_increase_s: float = 0.05
    max_deceleration_decrease_fraction: float = 0.1
    max_stopping_distance_increase_m: float = 0.15


def _parse_bool(value: str, line_number: int) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no"}:
        return False
    raise ValueError(f"invalid stop_event on line {line_number}")


def read_braking_samples(path: str | Path) -> tuple[BrakingSample, ...]:
    source = Path(path)
    samples: list[BrakingSample] = []
    with source.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = REQUIRED_COLUMNS - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"dataset is missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            try:
                sample = BrakingSample(
                    run_id=str(row["run_id"]).strip(),
                    time_s=float(row["time_s"]),
                    command_speed_mps=float(row["command_speed_mps"]),
                    odometry_speed_mps=float(row["odometry_speed_mps"]),
                    stop_event=_parse_bool(row["stop_event"], line_number),
                )
            except (TypeError, ValueError) as error:
                if isinstance(error, ValueError) and "stop_event" in str(error):
                    raise
                raise ValueError(f"invalid value on line {line_number}") from error
            numeric = (
                sample.time_s,
                sample.command_speed_mps,
                sample.odometry_speed_mps,
            )
            if not sample.run_id or any(not isfinite(value) or value < 0.0 for value in numeric):
                raise ValueError(f"non-finite, negative, or empty value on line {line_number}")
            samples.append(sample)
    if not samples:
        raise ValueError("dataset must contain at least one braking sample")
    return tuple(samples)


def _nearest_rank(values: tuple[float, ...], probability: float) -> float:
    ordered = tuple(sorted(values))
    index = max(0, min(len(ordered) - 1, ceil(probability * len(ordered)) - 1))
    return ordered[index]


def _measure_run(
    run_id: str,
    samples: tuple[BrakingSample, ...],
    *,
    reaction_speed_drop_mps: float,
    stopped_speed_mps: float,
) -> BrakingRunMetrics:
    ordered = tuple(sorted(samples, key=lambda item: item.time_s))
    if any(right.time_s <= left.time_s for left, right in zip(ordered, ordered[1:])):
        raise ValueError(f"run {run_id} timestamps must be strictly increasing")
    stop_indices = [index for index, sample in enumerate(ordered) if sample.stop_event]
    if len(stop_indices) != 1:
        raise ValueError(f"run {run_id} must contain exactly one stop_event")
    stop_index = stop_indices[0]
    initial_speed = ordered[stop_index].odometry_speed_mps
    reaction_index = next(
        (
            index
            for index in range(stop_index + 1, len(ordered))
            if ordered[index].odometry_speed_mps
            <= initial_speed - reaction_speed_drop_mps
        ),
        None,
    )
    if reaction_index is None:
        raise ValueError(f"run {run_id} never shows a measurable braking response")
    stopped_index = next(
        (
            index
            for index in range(reaction_index, len(ordered))
            if ordered[index].odometry_speed_mps <= stopped_speed_mps
        ),
        None,
    )
    if stopped_index is None or stopped_index == reaction_index:
        raise ValueError(f"run {run_id} does not contain a resolved stop")
    reaction_delay = ordered[reaction_index].time_s - ordered[stop_index].time_s
    braking_duration = ordered[stopped_index].time_s - ordered[reaction_index].time_s
    deceleration = (
        ordered[reaction_index].odometry_speed_mps
        - ordered[stopped_index].odometry_speed_mps
    ) / braking_duration
    distance = 0.0
    for left, right in zip(
        ordered[stop_index:stopped_index],
        ordered[stop_index + 1 : stopped_index + 1],
    ):
        elapsed = right.time_s - left.time_s
        distance += 0.5 * (
            left.odometry_speed_mps + right.odometry_speed_mps
        ) * elapsed
    return BrakingRunMetrics(
        run_id=run_id,
        initial_speed_mps=initial_speed,
        reaction_delay_s=reaction_delay,
        mean_deceleration_mps2=deceleration,
        stopping_distance_m=distance,
        stop_time_s=ordered[stopped_index].time_s - ordered[stop_index].time_s,
    )


def compare_braking_calibrations(
    current: BrakingCalibration,
    baseline: BrakingCalibration,
    limits: DriftLimits | None = None,
) -> DriftReport:
    limits = limits or DriftLimits()
    reaction_increase = current.reaction_time_s - baseline.reaction_time_s
    deceleration_decrease = max(
        0.0,
        (baseline.max_braking_mps2 - current.max_braking_mps2)
        / baseline.max_braking_mps2,
    )
    distance_increase = current.stopping_distance_m - baseline.stopping_distance_m
    violations: list[str] = []
    if reaction_increase > limits.max_reaction_time_increase_s:
        violations.append("reaction_time")
    if deceleration_decrease > limits.max_deceleration_decrease_fraction:
        violations.append("deceleration")
    if distance_increase > limits.max_stopping_distance_increase_m:
        violations.append("stopping_distance")
    return DriftReport(
        passed=not violations,
        reaction_time_increase_s=reaction_increase,
        deceleration_decrease_fraction=deceleration_decrease,
        stopping_distance_increase_m=distance_increase,
        violations=tuple(violations),
    )


def fit_braking_calibration(
    path: str | Path,
    *,
    target_coverage: float = 0.95,
    source: str | None = None,
    base_config: SafetyConfig | None = None,
    reaction_speed_drop_mps: float = 0.05,
    stopped_speed_mps: float = 0.03,
    baseline: BrakingCalibration | None = None,
    drift_limits: DriftLimits | None = None,
) -> BrakingCalibration:
    if not 0.5 <= target_coverage < 1.0:
        raise ValueError("target_coverage must be at least 0.5 and below one")
    samples = read_braking_samples(path)
    grouped: dict[str, list[BrakingSample]] = {}
    for sample in samples:
        grouped.setdefault(sample.run_id, []).append(sample)
    runs = tuple(
        _measure_run(
            run_id,
            tuple(run_samples),
            reaction_speed_drop_mps=reaction_speed_drop_mps,
            stopped_speed_mps=stopped_speed_mps,
        )
        for run_id, run_samples in sorted(grouped.items())
    )
    reaction_time = _nearest_rank(
        tuple(run.reaction_delay_s for run in runs),
        target_coverage,
    )
    braking = _nearest_rank(
        tuple(run.mean_deceleration_mps2 for run in runs),
        1.0 - target_coverage,
    )
    stopping_distance = _nearest_rank(
        tuple(run.stopping_distance_m for run in runs),
        target_coverage,
    )
    safety = replace(
        base_config or SafetyConfig(),
        reaction_time_s=reaction_time,
        max_braking_mps2=braking,
    )
    calibration = BrakingCalibration(
        schema_version=1,
        source=source or Path(path).name,
        target_coverage=target_coverage,
        run_count=len(runs),
        reaction_time_s=reaction_time,
        max_braking_mps2=braking,
        stopping_distance_m=stopping_distance,
        runs=runs,
        safety_config=asdict(safety),
    )
    if baseline is not None:
        calibration = replace(
            calibration,
            drift=compare_braking_calibrations(
                calibration,
                baseline,
                drift_limits,
            ),
        )
    return calibration


def write_braking_calibration(
    calibration: BrakingCalibration,
    path: str | Path,
) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(asdict(calibration), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return target


def _metrics(payload: dict[str, Any]) -> tuple[BrakingRunMetrics, ...]:
    return tuple(BrakingRunMetrics(**item) for item in payload["runs"])


def load_braking_calibration(path: str | Path) -> BrakingCalibration:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported braking calibration schema_version")
    drift_payload = payload.get("drift")
    drift = (
        DriftReport(
            passed=bool(drift_payload["passed"]),
            reaction_time_increase_s=float(drift_payload["reaction_time_increase_s"]),
            deceleration_decrease_fraction=float(
                drift_payload["deceleration_decrease_fraction"]
            ),
            stopping_distance_increase_m=float(
                drift_payload["stopping_distance_increase_m"]
            ),
            violations=tuple(str(value) for value in drift_payload["violations"]),
        )
        if drift_payload
        else None
    )
    return BrakingCalibration(
        schema_version=1,
        source=str(payload["source"]),
        target_coverage=float(payload["target_coverage"]),
        run_count=int(payload["run_count"]),
        reaction_time_s=float(payload["reaction_time_s"]),
        max_braking_mps2=float(payload["max_braking_mps2"]),
        stopping_distance_m=float(payload["stopping_distance_m"]),
        runs=_metrics(payload),
        safety_config={key: float(value) for key, value in payload["safety_config"].items()},
        review_status=str(payload.get("review_status", "unreviewed")),
        reviewed_by=payload.get("reviewed_by"),
        reviewed_at=payload.get("reviewed_at"),
        review_evidence=payload.get("review_evidence"),
        drift=drift,
    )


def review_braking_calibration(
    calibration: BrakingCalibration,
    *,
    reviewer: str,
    evidence: str,
    reviewed_at: str | None = None,
) -> BrakingCalibration:
    if not reviewer.strip() or not evidence.strip():
        raise ValueError("reviewer and evidence are required")
    timestamp = reviewed_at or datetime.now(timezone.utc).isoformat()
    return replace(
        calibration,
        review_status="reviewed",
        reviewed_by=reviewer.strip(),
        reviewed_at=timestamp,
        review_evidence=evidence.strip(),
    )


def safety_config_from_braking_artifact(
    calibration: BrakingCalibration,
    *,
    require_reviewed: bool = True,
) -> SafetyConfig:
    if require_reviewed and calibration.review_status != "reviewed":
        raise ValueError("braking calibration must be reviewed before runtime use")
    if calibration.drift is not None and not calibration.drift.passed:
        raise ValueError("braking calibration exceeds envelope drift limits")
    return SafetyConfig(**calibration.safety_config)
