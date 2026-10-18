from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Protocol

from .models import CandidateEvaluation, Control, Scenario
from .predictor import evaluate_control

FallbackStatus = Literal["feasible", "infeasible"]


@dataclass(frozen=True)
class FallbackResult:
    evaluation: CandidateEvaluation
    candidates_evaluated: int
    status: FallbackStatus


class FallbackController(Protocol):
    name: str

    def select(self, scenario: Scenario) -> FallbackResult: ...


def command_deviation(control: Control, scenario: Scenario) -> float:
    """Return normalized L1 deviation from the upstream command."""
    command = scenario.commanded_control
    config = scenario.config
    acceleration = abs(control.acceleration_mps2 - command.acceleration_mps2)
    yaw = abs(control.yaw_rate_rps - command.yaw_rate_rps)
    return acceleration / config.max_braking_mps2 + yaw / config.max_yaw_rate_rps


def is_safe(evaluation: CandidateEvaluation, scenario: Scenario) -> bool:
    return (
        evaluation.max_collision_probability <= scenario.config.risk_budget
        and evaluation.minimum_clearance_m >= 0.0
    )


def _selection_cost(evaluation: CandidateEvaluation, scenario: Scenario) -> float:
    lost_progress = max(
        0.0,
        scenario.ego.speed_mps - evaluation.trajectory[-1].state.speed_mps,
    )
    return command_deviation(evaluation.control, scenario) + 0.15 * lost_progress + 5.0 * evaluation.tail_risk


def _minimum_risk(evaluations: tuple[CandidateEvaluation, ...]) -> CandidateEvaluation:
    return min(
        evaluations,
        key=lambda item: (
            item.max_collision_probability,
            item.tail_risk,
            -item.minimum_clearance_m,
        ),
    )


class LatticeFallbackController:
    name = "lattice"

    def select(self, scenario: Scenario) -> FallbackResult:
        config = scenario.config
        brake_levels = (
            -config.max_braking_mps2,
            -0.75 * config.max_braking_mps2,
            -0.5 * config.max_braking_mps2,
            -0.25 * config.max_braking_mps2,
            0.0,
        )
        yaw_levels = (
            -config.max_yaw_rate_rps,
            -0.5 * config.max_yaw_rate_rps,
            0.0,
            0.5 * config.max_yaw_rate_rps,
            config.max_yaw_rate_rps,
        )
        evaluations = tuple(
            evaluate_control(
                scenario.ego,
                Control(acceleration, yaw_rate),
                scenario.obstacles,
                config,
            )
            for acceleration in brake_levels
            for yaw_rate in yaw_levels
        )
        feasible = tuple(item for item in evaluations if is_safe(item, scenario))
        if feasible:
            return FallbackResult(
                evaluation=min(feasible, key=lambda item: _selection_cost(item, scenario)),
                candidates_evaluated=len(evaluations),
                status="feasible",
            )
        return FallbackResult(
            evaluation=_minimum_risk(evaluations),
            candidates_evaluated=len(evaluations),
            status="infeasible",
        )


class ControlBarrierFallbackController:
    """Deterministic sampled solver for a discrete exponential CBF constraint."""

    name = "cbf"

    def __init__(self, alpha: float = 4.0) -> None:
        if alpha <= 0.0:
            raise ValueError("alpha must be positive")
        self.alpha = alpha

    def select(self, scenario: Scenario) -> FallbackResult:
        config = scenario.config
        acceleration_levels = tuple(
            -config.max_braking_mps2 * index / 10 for index in range(10, -1, -1)
        )
        yaw_levels = tuple(
            -config.max_yaw_rate_rps + 2 * config.max_yaw_rate_rps * index / 16
            for index in range(17)
        )
        evaluations = tuple(
            evaluate_control(
                scenario.ego,
                Control(acceleration, yaw_rate),
                scenario.obstacles,
                config,
            )
            for acceleration in acceleration_levels
            for yaw_rate in yaw_levels
        )
        barrier_feasible = tuple(
            item
            for item in evaluations
            if self._satisfies_barrier(item, scenario) and is_safe(item, scenario)
        )
        if barrier_feasible:
            return FallbackResult(
                evaluation=min(
                    barrier_feasible,
                    key=lambda item: _selection_cost(item, scenario),
                ),
                candidates_evaluated=len(evaluations),
                status="feasible",
            )
        return FallbackResult(
            evaluation=_minimum_risk(evaluations),
            candidates_evaluated=len(evaluations),
            status="infeasible",
        )

    def _satisfies_barrier(
        self,
        evaluation: CandidateEvaluation,
        scenario: Scenario,
    ) -> bool:
        config = scenario.config
        contraction = max(0.0, 1.0 - self.alpha * config.step_s)
        for obstacle in scenario.obstacles:
            radius = config.robot_radius_m + obstacle.radius_m + config.safety_margin_m
            previous_h: float | None = None
            for point in evaluation.trajectory:
                obstacle_position = obstacle.position + obstacle.velocity * point.time_s
                relative = point.state.position - obstacle_position
                barrier = relative.dot(relative) - radius**2
                if previous_h is not None and barrier + 1e-9 < contraction * previous_h:
                    return False
                previous_h = barrier
        return True


def controller_for(name: str) -> FallbackController:
    if name == "lattice":
        return LatticeFallbackController()
    if name == "cbf":
        return ControlBarrierFallbackController()
    raise ValueError(f"unsupported fallback controller: {name}")
