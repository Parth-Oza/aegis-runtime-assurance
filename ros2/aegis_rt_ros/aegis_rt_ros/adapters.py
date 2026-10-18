from __future__ import annotations

from math import atan2, hypot
from typing import Any

from aegis_rt.command_guard import VelocityCommand
from aegis_rt.models import Obstacle, RobotState, Vec2


def odometry_to_state(message: Any) -> RobotState:
    orientation = message.pose.pose.orientation
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    linear = message.twist.twist.linear
    return RobotState(
        position=Vec2(message.pose.pose.position.x, message.pose.pose.position.y),
        heading_rad=atan2(sin_yaw, cos_yaw),
        speed_mps=hypot(linear.x, linear.y),
    )


def twist_to_command(message: Any) -> VelocityCommand:
    return VelocityCommand(
        linear_mps=max(0.0, float(message.linear.x)),
        angular_rps=float(message.angular.z),
    )


class DetectionTracker:
    """Convert Detection3DArray messages and estimate planar velocity by ID."""

    def __init__(
        self,
        *,
        default_radius_m: float = 0.45,
        position_sigma_m: float = 0.15,
        acceleration_sigma_mps2: float = 0.2,
    ) -> None:
        self.default_radius_m = default_radius_m
        self.position_sigma_m = position_sigma_m
        self.acceleration_sigma_mps2 = acceleration_sigma_mps2
        self._previous: dict[str, tuple[Vec2, int]] = {}

    def update(self, message: Any, timestamp_ns: int) -> tuple[Obstacle, ...]:
        obstacles: list[Obstacle] = []
        current: dict[str, tuple[Vec2, int]] = {}
        for index, detection in enumerate(message.detections):
            obstacle_id = str(detection.id or f"detection-{index}")
            center = detection.bbox.center.position
            position = Vec2(float(center.x), float(center.y))
            previous = self._previous.get(obstacle_id)
            velocity = Vec2(0.0, 0.0)
            if previous is not None and timestamp_ns > previous[1]:
                elapsed_s = (timestamp_ns - previous[1]) / 1_000_000_000
                velocity = (position - previous[0]) * (1.0 / elapsed_s)
            diameter = max(float(detection.bbox.size.x), float(detection.bbox.size.y))
            radius = diameter / 2.0 if diameter > 0.0 else self.default_radius_m
            obstacles.append(
                Obstacle(
                    obstacle_id=obstacle_id,
                    position=position,
                    velocity=velocity,
                    radius_m=radius,
                    position_sigma_m=self.position_sigma_m,
                    acceleration_sigma_mps2=self.acceleration_sigma_mps2,
                )
            )
            current[obstacle_id] = (position, timestamp_ns)
        self._previous = current
        return tuple(obstacles)
