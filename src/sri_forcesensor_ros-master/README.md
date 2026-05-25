# SRI Force Sensor ROS Driver

This package uses TCP communication for an SRI force/torque sensor configured
to directly output decoupled Engineering Output. It does not use the old ATI
or raw ADC data path.

Expected sensor format:

```text
(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)
```

The node refuses to publish wrench data if `AT+SGDM=?` does not report the
expected `A01..A06` Engineering Output order.

## Usage

Configure the PC network to the same subnet as the sensor, power the sensor,
then run:

```
catkin_make
source devel/setup.bash
roslaunch sriforcesensor sri_ft_sensor.launch sensor_ip:=192.168.0.108 publish_rate:=200
```

The underlying executable is still available:

```bash
rosrun sriforcesensor forcesensor 192.168.0.108 200
```

The IP can be changed by RS485/USB from the SRI Windows tool. The publish rate
should match the force sensing rate.

## Topics

- `/sri_ft_sensor/raw_wrench`: raw Engineering Output converted to
  `geometry_msgs/WrenchStamped`.
- `/sri_ft_sensor/wrench`: ROS-side moving-average filtered wrench.
- `sri_force`: legacy/debug `geometry_msgs/Twist` topic.

Force units are N. Torque units are expected to be Nm. If the SRI sensor is
configured to output Nmm torque, launch with `torque_scale:=0.001`.

Use `sign_correction:="1 1 1 1 1 1"` to explicitly document channel signs.
Each entry must be `1` or `-1` and maps `A01..A06` to
`Fx,Fy,Fz,Tx,Ty,Tz`.

## Filter Parameters

The launch file exposes:

- `filter_enabled`, default `true`
- `filter_window_size`, default `10`
- `max_stddev_force`, default `2.0`
- `max_stddev_torque`, default `0.2`

The moving average is always a software data-chain element. The stddev limits
are used by the payload sampling stage as a stability gate; they do not by
themselves suppress `/sri_ft_sensor/wrench` messages.

## Important!

If the output is wrong, first check the terminal output from `AT+SGDM=?` and
verify channel order, Engineering Output mode, torque units, and signs with
single-axis calibration loads before running payload identification.

# Appendix
### Below is informaton from the original package. It obtain the force value by using raw ADC int output and the decompled matrix.
[https://bitbucket.org/hangnearlab/sri_forcesensor.git]

SRI Force sensor ros package
This repository contains the source code for the sri force sensor of ros package. This ROS package can be used to build communication between the sri force sensor and ROS. 

For citation please refer to:

Su, H., Yang, C., Ferrigno, G., & De Momi, E. (2019). Improved Human–Robot Collaborative Control of Redundant Robot for Teleoperated Minimally Invasive Surgery. IEEE Robotics and Automation Letters, 4(2), 1447-1453. (This paper is also indexed in IEEE International Conference on Robotics and Automation, 2019.)


Further questions please contact hang.su@polimi.it (www.nearlab.polimi.it) without hesitation. Thanks. 
