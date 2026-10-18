from __future__ import annotations

from dataclasses import dataclass, field

from .assurance import RuntimeAssuranceSupervisor
from .models import (
    AssuranceDecision,
    Control,
    Obstacle,
    RobotState,
    SafetyConfig,
    Scenario,
)


@dataclass(frozen=True)
class VelocityCommand:
    linear_mps: float
    angular_rps: float


@dataclass(frozen=True)
class CommandGuardConfig:
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    stale_timeout_s: float = 0.5
    command_response_s: float = 0.25
    fallback_controller: str = "lattice"

    def __post_init__(self) -> None:
        if self.stale_timeout_s <= 0.0 or self.command_response_s <= 0.0:
            raise ValueError("guard timing parameters must be positive")


@dataclass(frozen=True)
class GuardResult:
    timestamp_ns: int
    applied: VelocityCommand
    intervened: bool
    reason: str
    stale_inputs: tuple[str, ...]
    decision: AssuranceDecision | None


class CommandGuard:
    """Timestamped, fail-closed adapter around the runtime supervisor."""

    def __init__(self, config: CommandGuardConfig | None = None) -> None:
        self.config = config or CommandGuardConfig()
        self._supervisor = RuntimeAssuranceSupervisor(
            fallback_controller=self.config.fallback_controller
        )
        self._state: tuple[RobotState, int] | None = None
        self._command: tuple[VelocityCommand, int] | None = None
        self._obstacles: tuple[tuple[Obstacle, ...], int] | None = None

    def update_state(self, state: RobotState, timestamp_ns: int) -> None:
        self._state = (state, timestamp_ns)

    def update_command(self, command: VelocityCommand, timestamp_ns: int) -> None:
        self._command = (command, timestamp_ns)

    def update_obstacles(
        self,
        obstacles: tuple[Obstacle, ...],
        timestamp_ns: int,
    ) -> None:
        self._obstacles = (obstacles, timestamp_ns)

    def evaluate(self, timestamp_ns: int) -> GuardResult:
        stale = self._stale_inputs(timestamp_ns)
        if stale:
            return GuardResult(
                timestamp_ns=timestamp_ns,
                applied=VelocityCommand(0.0, 0.0),
                intervened=True,
                reason=f"Fail-closed stop: stale or missing inputs: {', '.join(stale)}.",
                stale_inputs=stale,
                decision=None,
            )

        assert self._state is not None
        assert self._command is not None
        assert self._obstacles is not None
        state = self._state[0]
        command = self._command[0]
        safety = self.config.safety
        acceleration = max(
            -safety.max_braking_mps2,
            min(
                safety.max_braking_mps2,
                (command.linear_mps - state.speed_mps) / self.config.command_response_s,
            ),
        )
        yaw_rate = max(
            -safety.max_yaw_rate_rps,
            min(safety.max_yaw_rate_rps, command.angular_rps),
        )
        scenario = Scenario(
            name="ros2-command-guard",
            ego=state,
            commanded_control=Control(acceleration, yaw_rate),
            obstacles=self._obstacles[0],
            config=safety,
        )
        decision = self._supervisor.evaluate(scenario)
        applied_linear = max(
            0.0,
            min(
                safety.max_speed_mps,
                state.speed_mps
                + decision.applied_control.acceleration_mps2
                * self.config.command_response_s,
            ),
        )
        return GuardResult(
            timestamp_ns=timestamp_ns,
            applied=VelocityCommand(
                linear_mps=applied_linear,
                angular_rps=decision.applied_control.yaw_rate_rps,
            ),
            intervened=decision.intervened,
            reason=decision.reason,
            stale_inputs=(),
            decision=decision,
        )

    def _stale_inputs(self, timestamp_ns: int) -> tuple[str, ...]:
        limit_ns = int(self.config.stale_timeout_s * 1_000_000_000)
        values = (
            ("robot_state", self._state),
            ("velocity_command", self._command),
            ("tracked_objects", self._obstacles),
        )
        return tuple(
            name
            for name, value in values
            if value is None
            or timestamp_ns < value[1]
            or timestamp_ns - value[1] > limit_ns
        )
