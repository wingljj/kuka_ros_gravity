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

## Dataset Replay Regression

The repository-level `src/datasets/payload_test_dataset.json` file can be
replayed through the ROS sampling and compute services:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make payload_dataset_validation_report
```

The replay covers nominal, biased, rotated sensor, farther COM, and 150 kg
payload cases. The heavy payload case verifies that least-squares solving still
uses the original physical matrix while the condition-number gate uses a
column-normalized matrix.

`payload_dataset_validation_report` runs the ROS replay and then prints the
detailed Markdown table in the current terminal. Each row compares:

- `Theory`: value from `src/datasets/payload_test_dataset.json`.
- `Actual`: value returned by the ROS `/compute_payload` service.
- `Abs Error`: `abs(Actual - Theory)`.
- `Tolerance`: the regression threshold for the synthetic dataset.
- `Result`: `PASS` when `Abs Error <= Tolerance`.

The report contains five dataset cases and seven metrics per case:
`tool_mass_kg`, `payload_mass_kg`, `payload_weight_n`,
`payload_com_x_m`, `payload_com_y_m`, `payload_com_z_m`, and
`residual_error`. The residual theory value is `0` for this noise-free
synthetic dataset; it is a numerical regression gate, not a recommended
field-test sensor-noise threshold.

The run also writes:

```text
build/test_results/tool_gravity_compensation/payload_dataset_comparison.md
build/test_results/tool_gravity_compensation/payload_dataset_comparison.csv
```

Pass criteria:

```bash
catkin_test_results build/test_results/tool_gravity_compensation
```

should report `0 errors, 0 failures`, and the Markdown report should show
`Failed: 0` with every metric row marked `PASS`.

If you run the lower-level rostest target directly, ROS captures most test
logging, so the terminal output is shorter. Use this fallback to inspect the
same detailed theory-vs-actual report:

```bash
catkin_make run_tests_tool_gravity_compensation_rostest_test_payload_dataset_replay.test
catkin_test_results build/test_results/tool_gravity_compensation
sed -n '1,120p' build/test_results/tool_gravity_compensation/payload_dataset_comparison.md
```

For the whole package regression suite:

```bash
catkin_make run_tests_tool_gravity_compensation
catkin_test_results build/test_results/tool_gravity_compensation
```

Condition-number normalization is validated by the 150 kg replay case and the
C++ heavy-payload regression. The public payload result message does not expose
raw or normalized condition-number values, so the report validates the behavior
through accepted identification and numerical agreement rather than printing
condition-number diagnostics directly.

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
- Condition-number gating is evaluated on a column-normalized least-squares
  matrix so heavy payloads are not rejected only because torque and bias
  columns have different physical scales.
- Force and torque residuals are checked during `/compute_payload` before a
  payload result is reported as successful.
- Tare-subtracted payload mass must be at least `0.05 kg` by default.
- `residual_error` reports the largest profile residual used for the final
  payload result.
