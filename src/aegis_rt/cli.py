from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

from .assurance import RuntimeAssuranceSupervisor
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
    supervisor = RuntimeAssuranceSupervisor()
    durations_ms: list[float] = []
    for _ in range(args.warmup):
        supervisor.evaluate(scenario)
    for _ in range(args.runs):
        started = time.perf_counter_ns()
        supervisor.evaluate(scenario)
        durations_ms.append((time.perf_counter_ns() - started) / 1_000_000)
    ordered = sorted(durations_ms)
    p95_index = min(len(ordered) - 1, int(0.95 * len(ordered)))
    print(json.dumps({
        "runs": args.runs,
        "median_ms": round(statistics.median(ordered), 4),
        "p95_ms": round(ordered[p95_index], 4),
        "max_ms": round(max(ordered), 4),
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
    benchmark.set_defaults(handler=_benchmark)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())

