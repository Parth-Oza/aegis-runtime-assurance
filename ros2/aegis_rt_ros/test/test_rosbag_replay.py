from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from vision_msgs.msg import Detection3D, Detection3DArray

from aegis_rt.command_guard import CommandGuard
from aegis_rt_ros.adapters import DetectionTracker, odometry_to_state, twist_to_command


def _write_bag(path: Path) -> None:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = (
        ("/odom", "nav_msgs/msg/Odometry"),
        ("/cmd_vel_raw", "geometry_msgs/msg/Twist"),
        ("/tracked_objects", "vision_msgs/msg/Detection3DArray"),
    )
    for name, message_type in topics:
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0,
                name=name,
                type=message_type,
                serialization_format="cdr",
            )
        )
    timestamp = 1_000_000_000
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = 2.0
    command = Twist()
    command.linear.x = 2.0
    detections = Detection3DArray()
    obstacle = Detection3D()
    obstacle.id = "blocked-lane"
    obstacle.bbox.center.position.x = 3.0
    obstacle.bbox.size.x = 1.0
    obstacle.bbox.size.y = 1.0
    detections.detections = [obstacle]
    writer.write("/odom", serialize_message(odometry), timestamp)
    writer.write("/cmd_vel_raw", serialize_message(command), timestamp)
    writer.write("/tracked_objects", serialize_message(detections), timestamp)


def test_rosbag_replay_drives_guard_decision(tmp_path: Path) -> None:
    bag = tmp_path / "guard_inputs"
    _write_bag(bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: get_message(item.type)
        for item in reader.get_all_topics_and_types()
    }
    guard = CommandGuard()
    tracker = DetectionTracker()
    last_timestamp = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        message = deserialize_message(data, topic_types[topic])
        last_timestamp = timestamp
        if topic == "/odom":
            guard.update_state(odometry_to_state(message), timestamp)
        elif topic == "/cmd_vel_raw":
            guard.update_command(twist_to_command(message), timestamp)
        elif topic == "/tracked_objects":
            guard.update_obstacles(tracker.update(message, timestamp), timestamp)

    result = guard.evaluate(last_timestamp)

    assert not result.stale_inputs
    assert result.intervened
    assert result.decision is not None
