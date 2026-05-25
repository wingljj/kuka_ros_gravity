# KR240 SRI Payload Identification

ROS1 Noetic workspace for KUKA KR240 R3330 payload mass and center-of-mass
identification with an SRI six-axis force/torque sensor.

The project is deliberately conservative for real hardware. Offline testing is
the default path. Real robot trajectory execution is disabled by default and
must be enabled explicitly only after the cell is physically safe.

## Main Packages

- `src/kuka_kr240_support`: ROS1 KR240 R3330 xacro and support files.
- `src/kr340_moveit`: KR240 MoveIt configuration. The package name is kept for
  compatibility with the existing workspace.
- `src/kuka_eki_hw_interface`: KUKA EKI hardware interface.
- `src/sri_forcesensor_ros-master`: SRI TCP driver and ROS-side wrench filter.
- `src/tool_gravity_compensation`: RViz panel, sampling server, tare subtraction,
  and payload identification.

## Build

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Offline Test

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tool_gravity_compensation kr240_offline_payload_test.launch
```

Use RViz panel `Payload Identification`:

1. Set mode to `Offline fake`.
2. Apply/check the `tool0 -> sri_ft_sensor` transform.
3. Confirm the safety checklist.
4. Move the fake robot to at least six distinct static orientations.
5. Record `TOOL_ONLY`, then record `TOOL_PLUS_PAYLOAD`.
6. Compute payload mass, weight, and CoM in `sri_ft_sensor`.

## Real KR240 + SRI

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tool_gravity_compensation kr240_real_payload_identification.launch \
  robot_ip:=172.31.1.147 eki_port:=54600 \
  sensor_ip:=192.168.0.108 publish_rate:=200
```

Real execution safeguards:

- `allow_trajectory_execution` defaults to `false`.
- `use_rviz` defaults to `false` for real launch files.
- The sampling server refuses automatic motion requests.
- `/step_control` requires an explicit manual confirmation flag.
- Hardware safety still rules: teach pendant, low-speed mode, emergency stop,
  safety perimeter, and trained operator supervision are mandatory.

Only after a separate safety review should real MoveIt execution be enabled:

```bash
roslaunch tool_gravity_compensation kr240_real_payload_identification.launch \
  allow_trajectory_execution:=true use_rviz:=true
```

## SRI Data Chain

The SRI driver is `sriforcesensor` from `src/sri_forcesensor_ros-master`.
Expected sensor output is:

```text
(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)
```

Published topics:

- `/sri_ft_sensor/raw_wrench`
- `/sri_ft_sensor/wrench`
- `sri_force` compatibility/debug topic

Payload identification consumes only `/sri_ft_sensor/wrench`.

## Verification

```bash
source /opt/ros/noetic/setup.bash
catkin_make
catkin_make run_tests_sriforcesensor run_tests_tool_gravity_compensation
```

Useful launch checks:

```bash
roslaunch tool_gravity_compensation kr240_offline_payload_test.launch --ros-args
roslaunch tool_gravity_compensation kr240_real_payload_identification.launch --ros-args
roslaunch kr340_moveit kr240_eki_moveit_planning_execution.launch --ros-args
```

Detailed operator documentation:
`src/tool_gravity_compensation/README_KR240_PAYLOAD_IDENTIFICATION.md`
