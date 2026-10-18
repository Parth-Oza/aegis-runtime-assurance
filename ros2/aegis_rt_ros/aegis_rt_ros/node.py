from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

from aegis_rt.command_guard import CommandGuard, CommandGuardConfig, GuardResult
from aegis_rt.models import SafetyConfig

from .adapters import DetectionTracker, odometry_to_state, twist_to_command


class CommandGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("aegis_command_guard")
        self._declare_parameters()
        safety = SafetyConfig(
            horizon_s=self._float("horizon_s"),
            step_s=self._float("step_s"),
            risk_budget=self._float("risk_budget"),
            robot_radius_m=self._float("robot_radius_m"),
            safety_margin_m=self._float("safety_margin_m"),
            localization_sigma_m=self._float("localization_sigma_m"),
            max_speed_mps=self._float("max_speed_mps"),
            max_braking_mps2=self._float("max_braking_mps2"),
            max_yaw_rate_rps=self._float("max_yaw_rate_rps"),
            reaction_time_s=self._float("reaction_time_s"),
        )
        self._guard = CommandGuard(
            CommandGuardConfig(
                safety=safety,
                stale_timeout_s=self._float("stale_timeout_s"),
                command_response_s=self._float("command_response_s"),
                fallback_controller=self._string("fallback_controller"),
            )
        )
        self._tracker = DetectionTracker(
            default_radius_m=self._float("default_obstacle_radius_m"),
            position_sigma_m=self._float("default_obstacle_sigma_m"),
            acceleration_sigma_mps2=self._float("default_acceleration_sigma_mps2"),
        )
        self._applied = self.create_publisher(
            Twist,
            self._string("applied_command_topic"),
            10,
        )
        self._diagnostics = self.create_publisher(
            DiagnosticArray,
            self._string("diagnostics_topic"),
            10,
        )
        self.create_subscription(
            Odometry,
            self._string("state_topic"),
            self._on_state,
            10,
        )
        self.create_subscription(
            Twist,
            self._string("command_topic"),
            self._on_command,
            10,
        )
        self.create_subscription(
            Detection3DArray,
            self._string("tracked_objects_topic"),
            self._on_objects,
            10,
        )
        self.create_timer(1.0 / self._float("evaluation_rate_hz"), self._evaluate)

    def _declare_parameters(self) -> None:
        values = {
            "state_topic": "/odom",
            "command_topic": "/cmd_vel_raw",
            "tracked_objects_topic": "/tracked_objects",
            "applied_command_topic": "/cmd_vel",
            "diagnostics_topic": "/aegis/diagnostics",
            "fallback_controller": "lattice",
            "evaluation_rate_hz": 20.0,
            "stale_timeout_s": 0.5,
            "command_response_s": 0.25,
            "default_obstacle_radius_m": 0.45,
            "default_obstacle_sigma_m": 0.15,
            "default_acceleration_sigma_mps2": 0.2,
            "horizon_s": 5.0,
            "step_s": 0.1,
            "risk_budget": 0.05,
            "robot_radius_m": 0.45,
            "safety_margin_m": 0.25,
            "localization_sigma_m": 0.08,
            "max_speed_mps": 4.0,
            "max_braking_mps2": 2.5,
            "max_yaw_rate_rps": 1.2,
            "reaction_time_s": 0.2,
        }
        for name, value in values.items():
            self.declare_parameter(name, value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _on_state(self, message: Odometry) -> None:
        self._guard.update_state(odometry_to_state(message), self._now_ns())

    def _on_command(self, message: Twist) -> None:
        self._guard.update_command(twist_to_command(message), self._now_ns())

    def _on_objects(self, message: Detection3DArray) -> None:
        now_ns = self._now_ns()
        self._guard.update_obstacles(self._tracker.update(message, now_ns), now_ns)

    def _evaluate(self) -> None:
        result = self._guard.evaluate(self._now_ns())
        command = Twist()
        command.linear.x = result.applied.linear_mps
        command.angular.z = result.applied.angular_rps
        self._applied.publish(command)
        self._diagnostics.publish(self._diagnostic_message(result))

    def _diagnostic_message(self, result: GuardResult) -> DiagnosticArray:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "aegis_runtime_assurance/command_guard"
        status.hardware_id = "software"
        status.level = (
            DiagnosticStatus.ERROR
            if result.stale_inputs
            else DiagnosticStatus.WARN
            if result.intervened
            else DiagnosticStatus.OK
        )
        status.message = result.reason
        values = {
            "decision_timestamp_ns": str(result.timestamp_ns),
            "intervened": str(result.intervened).lower(),
            "stale_inputs": ",".join(result.stale_inputs),
            "applied_linear_mps": f"{result.applied.linear_mps:.6f}",
            "applied_angular_rps": f"{result.applied.angular_rps:.6f}",
        }
        if result.decision is not None:
            values.update({
                "risk_before": f"{result.decision.commanded_evaluation.max_collision_probability:.9f}",
                "risk_after": f"{result.decision.applied_evaluation.max_collision_probability:.9f}",
                "fallback_controller": result.decision.fallback_controller,
                "fallback_status": result.decision.fallback_status,
            })
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        message.status = [status]
        return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CommandGuardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
