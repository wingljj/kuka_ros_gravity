#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "load_calibration_node");
  ros::NodeHandle private_nh("~");

  ROS_ERROR_STREAM("The legacy tool_gravity demo is disabled for KR240 payload identification. "
                   "It previously executed MoveIt motions without the required manual confirmation "
                   "flow and used obsolete force scaling. Use the collect_load_profile action server "
                   "and RViz panel after generating kr240_moveit_config.");

  return 1;
}
