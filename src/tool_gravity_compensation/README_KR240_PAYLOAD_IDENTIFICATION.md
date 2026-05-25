# KR240 SRI Payload Identification

This package identifies payload mass and center of mass from static
`/sri_ft_sensor/wrench` samples. The workflow is intentionally manual:
MoveIt planning and execution are handled by the operator, while this package
records only stable wrench windows and computes tare-subtracted payload
properties.

## Offline Test

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tool_gravity_compensation kr240_offline_payload_test.launch
```

Use the RViz panel named `Payload Identification`.

1. Set mode to `Offline fake`.
2. Confirm the safety checklist.
3. Move the fake robot to at least six different static orientations.
4. Click `Record TOOL_ONLY` for the tooling-only profile.
5. Click `Record TOOL_PLUS_PAYLOAD` for the tooling plus payload profile.
6. Click `Compute Payload`.

## Real KR240 + SRI Launch

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tool_gravity_compensation kr240_real_payload_identification.launch \
  robot_ip:=172.31.1.147 eki_port:=54600 sensor_ip:=192.168.0.108 publish_rate:=200
```

This real launch defaults to `allow_trajectory_execution:=false` and
`use_rviz:=false`. Enable real MoveIt execution only after a separate hardware
safety review:

```bash
roslaunch tool_gravity_compensation kr240_real_payload_identification.launch \
  allow_trajectory_execution:=true use_rviz:=true
```

The SRI driver uses TCP port `4008` and publishes:

- `/sri_ft_sensor/raw_wrench`
- `/sri_ft_sensor/wrench`
- `sri_force` for compatibility

`/sri_ft_sensor/wrench` is the only input used by payload identification.

## Panel Parameters

- `Sensor mount tool0 -> sri_ft_sensor`: measured translation in meters and
  roll/pitch/yaw in radians. Click `Apply mount`, then `Check TF`.
- `ROS filter enabled`, `Moving average window`: runtime SRI ROS-side filter.
- `Min stable samples`, `Max force stddev N`, `Max torque stddev Nm`,
  `Sample timeout s`: stability gate before a sample is accepted.
- `Robot IP`, `EKI port`, `SRI IP`, `SRI rate Hz`: stored on the ROS parameter
  server for launch/operator visibility.
- SRI torque units/signs are configured at launch with `torque_scale` and
  `sign_correction`. Keep `torque_scale:=1.0` for Nm; use `0.001` if the
  sensor outputs Nmm.

## Safety Rules

- Software safety checks do not replace hardware safety.
- Keep the teach pendant active, emergency stop reachable, and the safety area
  clear.
- Use low-speed mode for real robot tests.
- This package refuses automatic motion execution in the sampling server.
- `/step_control` requires a manual confirmation flag, so command-line callers
  cannot record samples without explicitly declaring operator confirmation.
- Use `Clear Profiles` whenever tooling, payload, sensor mount, or filter
  settings change.

## Result Quality Gates

- At least three observations are required; six or more distinct static
  orientations are recommended.
- Near-singular orientation sets are rejected by least-squares rank/condition
  checks.
- Force and torque residuals are checked during `/compute_payload` before a
  payload result is reported as successful.
- Tare-subtracted payload mass must be at least `0.05 kg` by default.
- `residual_error` reports the largest profile residual used for the final
  payload result.
