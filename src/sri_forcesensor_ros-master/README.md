# Update ROS Version for SRI Force Sensor
This package use TCP communication for SRI Force Sensor, which should be configurated to directly output decoupled force data, *without the need of calculating coupling matrixs (6x6 or 6x8) from raw data*.

**Instead of the original package below, this package directly obtains decoupled force data by setting the output format of SRI as Engineering Output, rather than ADC output. Details see `sriforcesensor.cpp` file.**

In test, we use the M4313XXX Force Sensor from `www.srisensor.com`, Ubuntu 20, ROS Noetic. Ubuntu 18 and ROS Melodic should also work.

## Usage
Configure the network to the same IP Adress mask, link the sensor and power it up:
```
catkin_make

rosrun sriforcesensor forcesensor 192.168.0.108 200
```
The IP can be changed by RS485 vis USB port in Windows. 
The Publish Rate should be congfigured the same as the force sensing rate.

## Important!
**If the output is wrong, please carefully check the Terminal Log Output and Guidance in `sriforcesensor.cpp` file. They will tell you how to configure the sensor correctly.**

# Appendix
### Below is informaton from the original package. It obtain the force value by using raw ADC int output and the decompled matrix.
[https://bitbucket.org/hangnearlab/sri_forcesensor.git]

SRI Force sensor ros package
This repository contains the source code for the sri force sensor of ros package. This ROS package can be used to build communication between the sri force sensor and ROS. 

For citation please refer to:

Su, H., Yang, C., Ferrigno, G., & De Momi, E. (2019). Improved Human–Robot Collaborative Control of Redundant Robot for Teleoperated Minimally Invasive Surgery. IEEE Robotics and Automation Letters, 4(2), 1447-1453. (This paper is also indexed in IEEE International Conference on Robotics and Automation, 2019.)


Further questions please contact hang.su@polimi.it (www.nearlab.polimi.it) without hesitation. Thanks. 

