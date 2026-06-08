#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "load_calibration_node");
  ros::NodeHandle private_nh("~");

  ROS_ERROR_STREAM("旧的 tool_gravity 演示已为 KR240 负载辨识禁用。"
                   "它之前在没有所需手动确认流程的情况下执行 MoveIt 运动，"
                   "并使用过时的力缩放。请在生成 kr240_moveit_config 后使用 "
                   "collect_load_profile action 服务器和 RViz 面板。");

  return 1;
}
