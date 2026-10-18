from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest
from vision_msgs.msg import Detection3D, Detection3DArray

from aegis_rt_ros.adapters import DetectionTracker, odometry_to_state, twist_to_command


def test_odometry_and_twist_adapters() -> None:
    odometry = Odometry()
    odometry.pose.pose.position.x = 1.0
    odometry.pose.pose.position.y = 2.0
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = 3.0
    odometry.twist.twist.linear.y = 4.0
    command = Twist()
    command.linear.x = 1.5
    command.angular.z = -0.2

    state = odometry_to_state(odometry)
    velocity = twist_to_command(command)

    assert state.position.x == 1.0
    assert state.position.y == 2.0
    assert state.speed_mps == 5.0
    assert velocity.linear_mps == 1.5
    assert velocity.angular_rps == -0.2


def test_detection_tracker_estimates_velocity_by_stable_id() -> None:
    tracker = DetectionTracker()
    first = Detection3DArray()
    detection = Detection3D()
    detection.id = "forklift"
    detection.bbox.center.position.x = 1.0
    detection.bbox.size.x = 1.2
    detection.bbox.size.y = 0.8
    first.detections = [detection]
    tracker.update(first, 1_000_000_000)

    second = Detection3DArray()
    moved = Detection3D()
    moved.id = "forklift"
    moved.bbox.center.position.x = 2.0
    moved.bbox.size.x = 1.2
    moved.bbox.size.y = 0.8
    second.detections = [moved]
    obstacle = tracker.update(second, 2_000_000_000)[0]

    assert obstacle.velocity.x == pytest.approx(1.0)
    assert obstacle.velocity.y == pytest.approx(0.0)
    assert obstacle.radius_m == pytest.approx(0.6)
