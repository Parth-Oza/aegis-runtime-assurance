from __future__ import annotations

from .models import AssuranceDecision, CandidateEvaluation, Control, Scenario
from .predictor import evaluate_control


class RuntimeAssuranceSupervisor:
    """Simplex-style supervisor around an arbitrary upstream planner command."""

    def evaluate(self, scenario: Scenario) -> AssuranceDecision:
        commanded = evaluate_control(
            scenario.ego,
            scenario.commanded_control,
            scenario.obstacles,
            scenario.config,
        )
        if self._is_safe(commanded, scenario):
            return AssuranceDecision(
                scenario_name=scenario.name,
                commanded_control=scenario.commanded_control,
                applied_control=scenario.commanded_control,
                intervened=False,
                reason="Planner command remains inside the probabilistic safety envelope.",
                commanded_evaluation=commanded,
                applied_evaluation=commanded,
                candidates_evaluated=1,
                counterfactual=self._counterfactual(scenario.commanded_control, commanded, commanded),
            )

        evaluations = [
            evaluate_control(scenario.ego, candidate, scenario.obstacles, scenario.config)
            for candidate in self._candidate_controls(scenario)
        ]
        feasible = [evaluation for evaluation in evaluations if self._is_safe(evaluation, scenario)]
        if feasible:
            applied = min(feasible, key=lambda item: self._cost(item, scenario))
            reason = (
                "Command rejected: predicted collision probability exceeds the risk budget; "
                "least-deviating safe control selected."
            )
        else:
            applied = min(
                evaluations,
                key=lambda item: (
                    item.max_collision_probability,
                    item.tail_risk,
                    -item.minimum_clearance_m,
                ),
            )
            reason = (
                "No candidate fully satisfies the risk budget; minimum-risk emergency control selected."
            )

        return AssuranceDecision(
            scenario_name=scenario.name,
            commanded_control=scenario.commanded_control,
            applied_control=applied.control,
            intervened=True,
            reason=reason,
            commanded_evaluation=commanded,
            applied_evaluation=applied,
            candidates_evaluated=len(evaluations) + 1,
            counterfactual=self._counterfactual(scenario.commanded_control, commanded, applied),
        )

    @staticmethod
    def _is_safe(evaluation: CandidateEvaluation, scenario: Scenario) -> bool:
        return (
            evaluation.max_collision_probability <= scenario.config.risk_budget
            and evaluation.minimum_clearance_m >= 0.0
        )

    @staticmethod
    def _candidate_controls(scenario: Scenario) -> tuple[Control, ...]:
        cfg = scenario.config
        brake_levels = (
            -cfg.max_braking_mps2,
            -0.75 * cfg.max_braking_mps2,
            -0.5 * cfg.max_braking_mps2,
            -0.25 * cfg.max_braking_mps2,
            0.0,
        )
        yaw_levels = (
            -cfg.max_yaw_rate_rps,
            -0.5 * cfg.max_yaw_rate_rps,
            0.0,
            0.5 * cfg.max_yaw_rate_rps,
            cfg.max_yaw_rate_rps,
        )
        return tuple(Control(acceleration, yaw_rate) for acceleration in brake_levels for yaw_rate in yaw_levels)

    @staticmethod
    def _cost(evaluation: CandidateEvaluation, scenario: Scenario) -> float:
        command = scenario.commanded_control
        cfg = scenario.config
        acceleration_delta = abs(evaluation.control.acceleration_mps2 - command.acceleration_mps2)
        yaw_delta = abs(evaluation.control.yaw_rate_rps - command.yaw_rate_rps)
        normalized_change = acceleration_delta / cfg.max_braking_mps2 + yaw_delta / cfg.max_yaw_rate_rps
        lost_progress = max(0.0, scenario.ego.speed_mps - evaluation.trajectory[-1].state.speed_mps)
        return normalized_change + 0.15 * lost_progress + 5.0 * evaluation.tail_risk

    @staticmethod
    def _counterfactual(
        commanded_control: Control,
        commanded: CandidateEvaluation,
        applied: CandidateEvaluation,
    ) -> dict[str, float | str | None]:
        risk_reduction = commanded.max_collision_probability - applied.max_collision_probability
        return {
            "claim": "Minimum-cost safe alternative found by deterministic control search.",
            "acceleration_delta_mps2": applied.control.acceleration_mps2 - commanded_control.acceleration_mps2,
            "yaw_rate_delta_rps": applied.control.yaw_rate_rps - commanded_control.yaw_rate_rps,
            "risk_before": commanded.max_collision_probability,
            "risk_after": applied.max_collision_probability,
            "absolute_risk_reduction": risk_reduction,
            "earliest_risk_time_before_s": commanded.earliest_risk_time_s,
            "earliest_risk_time_after_s": applied.earliest_risk_time_s,
        }

