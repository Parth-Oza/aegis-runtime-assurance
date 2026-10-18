# Hardware-in-the-loop braking validation

This workflow measures the delay and deceleration assumptions used by runtime assurance. A generated artifact is deliberately **unreviewed** until a named human reviewer links the run evidence. The checked-in CSV is synthetic and only tests the analysis path.

## Safety protocol

Do not run this procedure around people or ordinary production traffic.

1. Use a closed, level test lane with more free distance than the robot's worst-case uncontrolled coast and braking distance.
2. Inspect brakes, wheels, battery state, payload retention, odometry, and the physical emergency stop before each session.
3. Assign a test lead and a separate spotter holding the emergency stop. Establish abort words and lane boundaries.
4. Start at the lowest supported speed with no payload. Increase speed and payload in reviewed increments; never jump directly to the deployment maximum.
5. Keep the recorder off the real-time command path. Stop the session after any control, localization, communication, traction, or mechanical anomaly.
6. Preserve the raw rosbag, generated CSV, robot configuration, floor condition, payload, firmware versions, and incident notes as review evidence.

This repository does not claim that synthetic data or an unattended run is hardware validation.

## Record runs

Build the ROS package as described in [the ROS 2 guide](ROS2_COMMAND_GUARD.md), then start the recorder with a unique run ID:

```bash
ros2 run aegis_rt_ros braking_recorder --ros-args \
  -p command_topic:=/base/cmd_vel \
  -p state_topic:=/odom \
  -p output_path:=/evidence/braking.csv \
  -p run_id:=speed-0.5-payload-0-run-01
```

The recorder appends:

```text
run_id,time_s,command_speed_mps,odometry_speed_mps,stop_event
```

`stop_event=1` marks the first odometry sample after the command crosses the configured stop threshold. Capture a rosbag in parallel so every CSV row can be traced back to signed raw inputs.

For each speed/payload/floor condition, collect repeated clean stops. Discarding a run requires a documented reason; do not remove unfavorable valid trials.

## Fit the envelope

```bash
python -m aegis_rt calibrate-braking /evidence/braking.csv \
  --output /evidence/braking-envelope.json \
  --coverage 0.95 \
  --source "robot R17 / firmware 4.2 / dry concrete / 40 kg"
```

For every run, the fitter reports command-to-response delay, mean deceleration after response, trapezoidal stopping distance, and total stop time. The artifact uses a high quantile for reaction delay and stopping distance and a low quantile for deceleration, then exports those conservative values in a complete `SafetyConfig` object.

## Review before use

The runtime loader rejects unreviewed artifacts. After checking the rosbag, run selection, configuration, conditions, plots, and derived envelope, a human reviewer can attach their identity and evidence reference:

```bash
python -m aegis_rt review-braking /evidence/braking-envelope.json \
  --reviewer "Safety Review Board / ticket BRK-42" \
  --evidence "sha256:...; s3://robot-evidence/BRK-42/"
```

The review command records UTC time but does not certify the component or replace the organization's safety process.

## Regression limits

Compare every new campaign to the reviewed baseline:

```bash
python -m aegis_rt calibrate-braking /evidence/new-braking.csv \
  --output /evidence/new-envelope.json \
  --baseline /evidence/reviewed-baseline.json \
  --max-reaction-drift 0.05 \
  --max-deceleration-drift 0.10 \
  --max-distance-drift 0.15
```

The command exits with status 2 when reaction delay grows too much, conservative deceleration falls too far, or stopping distance increases beyond its limit. An artifact with failed drift cannot be loaded for runtime use, even if someone later adds review metadata.

## Repository evidence boundary

`datasets/synthetic_braking_runs.csv` exercises the deterministic parser, estimator, artifact, review gate, and drift gate. It is not hardware evidence. Closing the hardware-validation issue requires supervised physical runs and an independently reviewed artifact from the target robot.
