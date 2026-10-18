from __future__ import annotations

from .fallbacks import command_deviation, controller_for, is_safe
from .models import AssuranceDecision, CandidateEvaluation, Control, Scenario
from .predictor import evaluate_control


class RuntimeAssuranceSupervisor:
    """Simplex-style supervisor around an arbitrary upstream planner command."""

    def __init__(self, fallback_controller: str = "lattice") -> None:
        self._fallback = controller_for(fallback_controller)

    def evaluate(self, scenario: Scenario) -> AssuranceDecision:
        commanded = evaluate_control(
            scenario.ego,
            scenario.commanded_control,
            scenario.obstacles,
            scenario.config,
            scenario.calibration,
        )
        if is_safe(commanded, scenario):
            return AssuranceDecision(
                scenario_name=scenario.name,
                commanded_control=scenario.commanded_control,
                applied_control=scenario.commanded_control,
                intervened=False,
                reason="Planner command remains inside the probabilistic safety envelope.",
                commanded_evaluation=commanded,
                applied_evaluation=commanded,
                candidates_evaluated=1,
                fallback_controller="planner",
                fallback_status="not_required",
                command_deviation=0.0,
                counterfactual=self._counterfactual(scenario.commanded_control, commanded, commanded),
            )

        fallback = self._fallback.select(scenario)
        applied = fallback.evaluation
        if fallback.status == "feasible":
            reason = (
                "Command rejected: predicted collision probability exceeds the risk budget; "
                f"{self._fallback.name} fallback selected a feasible control."
            )
        else:
            reason = (
                f"{self._fallback.name} fallback is explicitly infeasible; "
                "minimum-risk emergency control selected."
            )

        return AssuranceDecision(
            scenario_name=scenario.name,
            commanded_control=scenario.commanded_control,
            applied_control=applied.control,
            intervened=True,
            reason=reason,
            commanded_evaluation=commanded,
            applied_evaluation=applied,
            candidates_evaluated=fallback.candidates_evaluated + 1,
            fallback_controller=self._fallback.name,
            fallback_status=fallback.status,
            command_deviation=command_deviation(applied.control, scenario),
            counterfactual=self._counterfactual(scenario.commanded_control, commanded, applied),
        )

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

