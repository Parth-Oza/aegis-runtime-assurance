from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = (
        ("state_topic", "/odom"),
        ("command_topic", "/cmd_vel_raw"),
        ("tracked_objects_topic", "/tracked_objects"),
        ("applied_command_topic", "/cmd_vel"),
        ("diagnostics_topic", "/aegis/diagnostics"),
    )
    declarations = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in arguments
    ]
    guard = Node(
        package="aegis_rt_ros",
        executable="command_guard",
        name="aegis_command_guard",
        output="screen",
        parameters=[
            {
                name: LaunchConfiguration(name)
                for name, _ in arguments
            }
        ],
    )
    return LaunchDescription([*declarations, guard])
