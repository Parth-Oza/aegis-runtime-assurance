from __future__ import annotations

import argparse
import json
import statistics
import time
from dataclasses import replace
from pathlib import Path

from .assurance import RuntimeAssuranceSupervisor
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
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())

