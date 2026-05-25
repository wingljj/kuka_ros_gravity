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

## Safety Rules

- Software safety checks do not replace hardware safety.
- Keep the teach pendant active, emergency stop reachable, and the safety area
  clear.
- Use low-speed mode for real robot tests.
- This package refuses automatic motion execution in the sampling server.
- Use `Clear Profiles` whenever tooling, payload, sensor mount, or filter
  settings change.
