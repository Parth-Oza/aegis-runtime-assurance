from __future__ import annotations

import argparse
import json
import statistics
import time
from dataclasses import asdict, replace
from pathlib import Path

from .assurance import RuntimeAssuranceSupervisor
from .braking import (
    DriftLimits,
    fit_braking_calibration,
    load_braking_calibration,
    review_braking_calibration,
    write_braking_calibration,
)
from .calibration import fit_calibration, load_calibration, write_calibration
from .evidence import write_html, write_json
from .scenario import load_scenario


def _run(args: argparse.Namespace) -> int:
    scenario = load_scenario(args.scenario)
    decision = RuntimeAssuranceSupervisor().evaluate(scenario)
    output = Path(args.output)
    write_json(decision, output / "decision.json")
    write_html(decision, output / "evidence.html")
    print(json.dumps({
        "scenario": decision.scenario_name,
        "intervened": decision.intervened,
        "risk_before": round(decision.commanded_evaluation.max_collision_probability, 6),
        "risk_after": round(decision.applied_evaluation.max_collision_probability, 6),
        "candidates_evaluated": decision.candidates_evaluated,
        "evidence": str((output / "evidence.html").resolve()),
    }, indent=2))
    return 0


def _benchmark(args: argparse.Namespace) -> int:
    scenario = load_scenario(args.scenario)
    names = ("lattice", "cbf") if args.controller == "both" else (args.controller,)
    results: dict[str, dict[str, float | int | str]] = {}
    for name in names:
        supervisor = RuntimeAssuranceSupervisor(fallback_controller=name)
        durations_ms: list[float] = []
        for _ in range(args.warmup):
            supervisor.evaluate(scenario)
        decision = None
        for _ in range(args.runs):
            started = time.perf_counter_ns()
            decision = supervisor.evaluate(scenario)
            durations_ms.append((time.perf_counter_ns() - started) / 1_000_000)
        ordered = sorted(durations_ms)
        p95_index = min(len(ordered) - 1, int(0.95 * len(ordered)))
        assert decision is not None
        results[name] = {
            "runs": args.runs,
            "median_ms": round(statistics.median(ordered), 4),
            "p95_ms": round(ordered[p95_index], 4),
            "max_ms": round(max(ordered), 4),
            "feasibility": decision.fallback_status,
            "command_deviation": round(decision.command_deviation, 6),
        }
    print(json.dumps({"controllers": results}, indent=2))
    return 0


def _calibrate(args: argparse.Namespace) -> int:
    calibration = fit_calibration(
        args.dataset,
        target_coverage=args.coverage,
        source=args.source,
    )
    target = write_calibration(calibration, args.output)
    print(json.dumps({
        "artifact": str(target.resolve()),
        "sample_count": calibration.sample_count,
        "target_coverage": calibration.target_coverage,
        "empirical_coverage": calibration.empirical_coverage,
        "calibration_error": calibration.calibration_error,
        "horizon_bins": len(calibration.bins),
    }, indent=2))
    return 0


def _compare_predictors(args: argparse.Namespace) -> int:
    scenario = load_scenario(args.scenario)
    baseline = RuntimeAssuranceSupervisor().evaluate(replace(scenario, calibration=None))
    calibration = load_calibration(args.calibration)
    empirical = RuntimeAssuranceSupervisor().evaluate(
        replace(scenario, calibration=calibration)
    )
    print(json.dumps({
        "scenario": scenario.name,
        "baseline": {
            "model": "radial_gaussian",
            "commanded_risk": baseline.commanded_evaluation.max_collision_probability,
            "applied_risk": baseline.applied_evaluation.max_collision_probability,
        },
        "calibrated": {
            "model": "empirical_residual",
            "source": calibration.source,
            "commanded_risk": empirical.commanded_evaluation.max_collision_probability,
            "applied_risk": empirical.applied_evaluation.max_collision_probability,
        },
    }, indent=2))
    return 0


def _calibrate_braking(args: argparse.Namespace) -> int:
    baseline = load_braking_calibration(args.baseline) if args.baseline else None
    if baseline is not None and baseline.review_status != "reviewed":
        raise ValueError("baseline braking artifact must be reviewed")
    limits = DriftLimits(
        max_reaction_time_increase_s=args.max_reaction_drift,
        max_deceleration_decrease_fraction=args.max_deceleration_drift,
        max_stopping_distance_increase_m=args.max_distance_drift,
    )
    calibration = fit_braking_calibration(
        args.dataset,
        target_coverage=args.coverage,
        source=args.source,
        baseline=baseline,
        drift_limits=limits,
    )
    target = write_braking_calibration(calibration, args.output)
    print(json.dumps({
        "artifact": str(target.resolve()),
        "run_count": calibration.run_count,
        "reaction_time_s": calibration.reaction_time_s,
        "max_braking_mps2": calibration.max_braking_mps2,
        "stopping_distance_m": calibration.stopping_distance_m,
        "review_status": calibration.review_status,
        "drift": asdict(calibration.drift) if calibration.drift else None,
    }, indent=2))
    return 2 if calibration.drift is not None and not calibration.drift.passed else 0


def _review_braking(args: argparse.Namespace) -> int:
    calibration = load_braking_calibration(args.artifact)
    reviewed = review_braking_calibration(
        calibration,
        reviewer=args.reviewer,
        evidence=args.evidence,
    )
    target = write_braking_calibration(reviewed, args.output or args.artifact)
    print(json.dumps({
        "artifact": str(target.resolve()),
        "review_status": reviewed.review_status,
        "reviewed_by": reviewed.reviewed_by,
        "reviewed_at": reviewed.reviewed_at,
        "review_evidence": reviewed.review_evidence,
    }, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Aegis robot runtime assurance")
    subparsers = parser.add_subparsers(dest="command", required=True)
    run = subparsers.add_parser("run", help="Evaluate a scenario and write evidence")
    run.add_argument("scenario")
    run.add_argument("--output", default="artifacts")
    run.set_defaults(handler=_run)
    benchmark = subparsers.add_parser("benchmark", help="Measure supervisor latency")
    benchmark.add_argument("scenario")
    benchmark.add_argument("--runs", type=int, default=1000)
    benchmark.add_argument("--warmup", type=int, default=50)
    benchmark.add_argument(
        "--controller",
        choices=("lattice", "cbf", "both"),
        default="both",
        help="Fallback controller to benchmark (default: both)",
    )
    benchmark.set_defaults(handler=_benchmark)
    calibrate = subparsers.add_parser(
        "calibrate",
        help="Fit a deterministic empirical residual calibration artifact",
    )
    calibrate.add_argument("dataset")
    calibrate.add_argument("--output", required=True)
    calibrate.add_argument("--coverage", type=float, default=0.95)
    calibrate.add_argument("--source", help="Human-readable dataset provenance")
    calibrate.set_defaults(handler=_calibrate)
    compare = subparsers.add_parser(
        "compare-predictors",
        help="Compare baseline and empirical risk estimates",
    )
    compare.add_argument("scenario")
    compare.add_argument("calibration")
    compare.set_defaults(handler=_compare_predictors)
    braking = subparsers.add_parser(
        "calibrate-braking",
        help="Estimate a conservative braking envelope from recorded runs",
    )
    braking.add_argument("dataset")
    braking.add_argument("--output", required=True)
    braking.add_argument("--coverage", type=float, default=0.95)
    braking.add_argument("--source", help="Human-readable dataset provenance")
    braking.add_argument("--baseline", help="Reviewed baseline artifact for drift checks")
    braking.add_argument("--max-reaction-drift", type=float, default=0.05)
    braking.add_argument("--max-deceleration-drift", type=float, default=0.1)
    braking.add_argument("--max-distance-drift", type=float, default=0.15)
    braking.set_defaults(handler=_calibrate_braking)
    review = subparsers.add_parser(
        "review-braking",
        help="Record human review and evidence on a braking artifact",
    )
    review.add_argument("artifact")
    review.add_argument("--reviewer", required=True)
    review.add_argument("--evidence", required=True)
    review.add_argument("--output")
    review.set_defaults(handler=_review_braking)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())

