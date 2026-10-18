from __future__ import annotations

from math import cos, erf, pi, sin, sqrt

from .calibration import empirical_collision_probability
from .models import (
    CandidateEvaluation,
    Control,
    EmpiricalCalibration,
    Obstacle,
    RiskSample,
    RobotState,
    SafetyConfig,
    TrajectoryPoint,
    Vec2,
)


def rollout(state: RobotState, control: Control, config: SafetyConfig) -> tuple[TrajectoryPoint, ...]:
    """Roll out a bounded unicycle model with a reaction-time delay."""
    points: list[TrajectoryPoint] = [TrajectoryPoint(0.0, state)]
    current = state
    steps = int(round(config.horizon_s / config.step_s))
    for index in range(1, steps + 1):
        time_s = index * config.step_s
        active = time_s > config.reaction_time_s
        acceleration = control.acceleration_mps2 if active else 0.0
        yaw_rate = control.yaw_rate_rps if active else 0.0
        speed = min(config.max_speed_mps, max(0.0, current.speed_mps + acceleration * config.step_s))
        heading = current.heading_rad + yaw_rate * config.step_s
        position = current.position + Vec2(cos(heading), sin(heading)) * (speed * config.step_s)
        current = RobotState(position=position, heading_rad=heading, speed_mps=speed)
        points.append(TrajectoryPoint(time_s=time_s, state=current))
    return tuple(points)


def _normal_cdf(value: float) -> float:
    return 0.5 * (1.0 + erf(value / sqrt(2.0)))


def _obstacle_at(obstacle: Obstacle, time_s: float) -> Vec2:
    return obstacle.position + obstacle.velocity * time_s


def collision_probability(
    separation_m: float,
    combined_radius_m: float,
    uncertainty_sigma_m: float,
) -> float:
    """Conservative radial Gaussian approximation of overlap probability."""
    sigma = max(uncertainty_sigma_m, 1e-6)
    probability = _normal_cdf((combined_radius_m - separation_m) / sigma)
    return min(1.0, max(0.0, probability))


def evaluate_control(
    state: RobotState,
    control: Control,
    obstacles: tuple[Obstacle, ...],
    config: SafetyConfig,
    calibration: EmpiricalCalibration | None = None,
) -> CandidateEvaluation:
    trajectory = rollout(state, control, config)
    samples: list[RiskSample] = []
    for point in trajectory:
        for obstacle in obstacles:
            obstacle_position = _obstacle_at(obstacle, point.time_s)
            separation = (point.state.position - obstacle_position).norm()
            combined_radius = config.robot_radius_m + obstacle.radius_m + config.safety_margin_m
            growth = 0.5 * obstacle.acceleration_sigma_mps2 * point.time_s**2
            sigma = sqrt(
                config.localization_sigma_m**2
                + obstacle.position_sigma_m**2
                + growth**2
            )
            if calibration is None:
                probability = collision_probability(separation, combined_radius, sigma)
                uncertainty_model = "radial_gaussian"
            else:
                probability = empirical_collision_probability(
                    separation - combined_radius,
                    point.time_s,
                    calibration,
                )
                uncertainty_model = "empirical_residual"
            samples.append(
                RiskSample(
                    time_s=point.time_s,
                    obstacle_id=obstacle.obstacle_id,
                    separation_m=separation,
                    clearance_m=separation - combined_radius,
                    uncertainty_sigma_m=sigma,
                    collision_probability=probability,
                    uncertainty_model=uncertainty_model,
                )
            )

    ordered = sorted((sample.collision_probability for sample in samples), reverse=True)
    tail_count = max(1, int(len(ordered) * 0.05)) if ordered else 1
    tail_risk = sum(ordered[:tail_count]) / tail_count if ordered else 0.0
    risky_times = [sample.time_s for sample in samples if sample.collision_probability > config.risk_budget]
    return CandidateEvaluation(
        control=control,
        max_collision_probability=max(ordered, default=0.0),
        tail_risk=tail_risk,
        minimum_clearance_m=min((sample.clearance_m for sample in samples), default=float("inf")),
        earliest_risk_time_s=min(risky_times) if risky_times else None,
        trajectory=trajectory,
        risk_samples=tuple(samples),
    )

