# ROS 2 command guard

`aegis_rt_ros` places the Aegis supervisor between an upstream velocity command and a robot base.

## Interfaces

| Direction | Default topic | Type | Purpose |
|---|---|---|---|
| Input | `/odom` | `nav_msgs/msg/Odometry` | Planar robot pose and measured speed |
| Input | `/cmd_vel_raw` | `geometry_msgs/msg/Twist` | Untrusted upstream velocity command |
| Input | `/tracked_objects` | `vision_msgs/msg/Detection3DArray` | Stable IDs, positions, and bounding boxes |
| Output | `/cmd_vel` | `geometry_msgs/msg/Twist` | Allowed or safety-modified command |
| Output | `/aegis/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Timestamped decision evidence |

Object velocity is estimated deterministically from successive detections with the same `Detection3D.id`. Bounding-box diameter supplies the collision radius. Tracker position and acceleration uncertainty are configurable parameters.

A `Twist` target is converted into bounded acceleration over `command_response_s`; the applied acceleration is converted back into a velocity target over the same interval. Only planar `linear.x` and `angular.z` are used.

## Build and launch

The package targets ROS 2 Jazzy and is also tested independently from the dependency-free Python core.

```bash
python3 -m pip install -e .
mkdir -p /tmp/aegis_ws/src
ln -s "$PWD/ros2/aegis_rt_ros" /tmp/aegis_ws/src/aegis_rt_ros
cd /tmp/aegis_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select aegis_rt_ros
source install/setup.bash
ros2 launch aegis_rt_ros command_guard.launch.py
```

Remap topics through launch arguments:

```bash
ros2 launch aegis_rt_ros command_guard.launch.py \
  state_topic:=/robot/odom \
  command_topic:=/planner/cmd_vel \
  tracked_objects_topic:=/perception/tracks \
  applied_command_topic:=/base/cmd_vel
```

Safety limits—including `risk_budget`, `max_speed_mps`, `max_braking_mps2`, `max_yaw_rate_rps`, `reaction_time_s`, footprint geometry, and prediction horizon—are declared ROS parameters. Override them with a parameter file or `ros2 run ... --ros-args -p name:=value`.

## Stale-input behavior

The guard uses local ROS receipt time for state, command, and tracked-object freshness. If any stream is missing, older than `stale_timeout_s`, or timestamped in the future relative to the evaluation clock, the node fails closed: it publishes a zero `Twist` and an `ERROR` diagnostic naming every stale input. A valid intervention produces `WARN`; an allowed command produces `OK`.

Each diagnostic carries the ROS header timestamp plus `decision_timestamp_ns`, intervention state, applied command, risks before and after, fallback controller, and fallback feasibility.

## Integration evidence

The ROS package tests write odometry, command, and tracked-object messages into a real SQLite3 rosbag through `rosbag2_py`, replay the serialized CDR messages through the adapters, and assert that the guard intervenes. CI builds and tests the package against ROS 2 Jazzy using the ROS Tooling action.
