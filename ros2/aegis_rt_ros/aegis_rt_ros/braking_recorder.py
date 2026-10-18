from __future__ import annotations

import csv
from math import hypot
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class BrakingRecorderNode(Node):
    """Record command, odometry speed, and stop-command edges as CSV."""

    def __init__(self) -> None:
        super().__init__("aegis_braking_recorder")
        self.declare_parameter("command_topic", "/cmd_vel")
        self.declare_parameter("state_topic", "/odom")
        self.declare_parameter("output_path", "braking_run.csv")
        self.declare_parameter("run_id", "run-001")
        self.declare_parameter("stop_threshold_mps", 0.03)
        self._command_speed = 0.0
        self._stop_event_pending = False
        self._threshold = float(self.get_parameter("stop_threshold_mps").value)
        output = Path(str(self.get_parameter("output_path").value))
        output.parent.mkdir(parents=True, exist_ok=True)
        new_file = not output.exists() or output.stat().st_size == 0
        self._stream = output.open("a", newline="", encoding="utf-8")
        self._writer = csv.writer(self._stream)
        if new_file:
            self._writer.writerow([
                "run_id",
                "time_s",
                "command_speed_mps",
                "odometry_speed_mps",
                "stop_event",
            ])
            self._stream.flush()
        self.create_subscription(
            Twist,
            str(self.get_parameter("command_topic").value),
            self._on_command,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("state_topic").value),
            self._on_odometry,
            10,
        )

    def _on_command(self, message: Twist) -> None:
        next_speed = max(0.0, float(message.linear.x))
        self._stop_event_pending = self._stop_event_pending or (
            self._command_speed > self._threshold
            and next_speed <= self._threshold
        )
        self._command_speed = next_speed

    def _on_odometry(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        measured_speed = hypot(linear.x, linear.y)
        time_s = self.get_clock().now().nanoseconds / 1_000_000_000
        self._writer.writerow([
            str(self.get_parameter("run_id").value),
            f"{time_s:.9f}",
            f"{self._command_speed:.9f}",
            f"{measured_speed:.9f}",
            int(self._stop_event_pending),
        ])
        self._stop_event_pending = False
        self._stream.flush()

    def destroy_node(self) -> bool:
        self._stream.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BrakingRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
