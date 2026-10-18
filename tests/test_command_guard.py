from __future__ import annotations

from aegis_rt.command_guard import CommandGuard, CommandGuardConfig, VelocityCommand
from aegis_rt.models import Obstacle, RobotState, SafetyConfig, Vec2


def test_guard_fails_closed_when_inputs_are_missing_or_stale() -> None:
    guard = CommandGuard(CommandGuardConfig(stale_timeout_s=0.1))

    missing = guard.evaluate(1_000_000_000)
    assert missing.applied == VelocityCommand(0.0, 0.0)
    assert missing.stale_inputs == (
        "robot_state",
        "velocity_command",
        "tracked_objects",
    )

    guard.update_state(RobotState(Vec2(0.0, 0.0), 0.0, 1.0), 1_000_000_000)
    guard.update_command(VelocityCommand(1.0, 0.0), 1_000_000_000)
    guard.update_obstacles((), 1_000_000_000)
    stale = guard.evaluate(1_200_000_000)
    assert stale.intervened
    assert set(stale.stale_inputs) == {
        "robot_state",
        "velocity_command",
        "tracked_objects",
    }


def test_guard_preserves_safe_twist_semantics() -> None:
    guard = CommandGuard(
        CommandGuardConfig(
            safety=SafetyConfig(horizon_s=1.0),
            command_response_s=0.25,
        )
    )
    timestamp = 1_000_000_000
    guard.update_state(RobotState(Vec2(0.0, 0.0), 0.0, 1.0), timestamp)
    guard.update_command(VelocityCommand(1.2, 0.1), timestamp)
    guard.update_obstacles((), timestamp)

    result = guard.evaluate(timestamp)

    assert not result.intervened
    assert result.applied == VelocityCommand(1.2, 0.1)
    assert result.decision is not None


def test_guard_intervenes_on_collision_course() -> None:
    guard = CommandGuard(CommandGuardConfig())
    timestamp = 1_000_000_000
    guard.update_state(RobotState(Vec2(0.0, 0.0), 0.0, 2.0), timestamp)
    guard.update_command(VelocityCommand(2.0, 0.0), timestamp)
    guard.update_obstacles(
        (
            Obstacle(
                obstacle_id="forklift",
                position=Vec2(6.0, -3.0),
                velocity=Vec2(0.0, 1.0),
                radius_m=0.75,
                position_sigma_m=0.12,
                acceleration_sigma_mps2=0.12,
            ),
        ),
        timestamp,
    )

    result = guard.evaluate(timestamp)

    assert result.intervened
    assert result.decision is not None
    assert result.decision.applied_evaluation.max_collision_probability <= 0.05
    assert result.applied != VelocityCommand(2.0, 0.0)
