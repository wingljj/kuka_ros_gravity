#include "ros/ros.h"
#include "netft_rdt_driver.h"
#include "geometry_msgs/WrenchStamped.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include <std_msgs/Bool.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <boost/program_options.hpp>
#include "lpfilter.h"

#include <dynamic_reconfigure/server.h>
#include <netft_utils/netftConfig.h>
#include "tf/transform_listener.h"

namespace po = boost::program_options;
using namespace std;

double tool_mass;
std::vector<double> tool_mass_center(3, 0.0);
std::vector<double> sensor_zero_point(6, 0.0);
std::vector<double> base_angle(2, 0.0);

void callback(netft_utils::netftConfig &config, uint32_t level) 
{  
	tool_mass = config.tool_mass;
    tool_mass_center[0] = config.tool_mass_center_x;
    tool_mass_center[1] = config.tool_mass_center_y;
    tool_mass_center[2] = config.tool_mass_center_z;
    sensor_zero_point[0] = config.sensor_zero_point_1;
    sensor_zero_point[1] = config.sensor_zero_point_2;
    sensor_zero_point[2] = config.sensor_zero_point_3;
    sensor_zero_point[3] = config.sensor_zero_point_4;
    sensor_zero_point[4] = config.sensor_zero_point_5;
    sensor_zero_point[5] = config.sensor_zero_point_6;
    base_angle[0] = config.base_angle_1;
    base_angle[1] = config.base_angle_2;
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "netft_modify_node");
  ros::NodeHandle nh;
//动态参数服务端
    dynamic_reconfigure::Server<netft_utils::netftConfig> server;    
	dynamic_reconfigure::Server<netft_utils::netftConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);    
	server.setCallback(f);  
    //姿态监听
    tf::TransformListener* listener = new tf::TransformListener(ros::Duration(300));

  float pub_rate_hz;
  string address;
  string frame_id;
  int32_t counts_per_force;
  int32_t counts_per_torque;

  //滤波器
  LPFilter* lp = new LPFilter(0.002,60,6);

  po::options_description desc("Options");
  desc.add_options()
    ("help", "display help")
    ("rate", po::value<float>(&pub_rate_hz)->default_value(500.0), "set publish rate (in hertz)")
    ("wrench", "publish older Wrench message type instead of WrenchStamped")
    ("address", po::value<string>(&address), "IP address of NetFT box")
    ("frame_id", po::value<string>(&frame_id)->default_value("base_link"), "frame_id for Wrench msgs")
    ("counts_per_force", po::value<int32_t>(&counts_per_force)->default_value(1000000), "Counts per force as listed by sensor webserver")
    ("counts_per_torque", po::value<int32_t>(&counts_per_torque)->default_value(1000000), "Counts per torque as listed by sensor webserver")
    ;
     
  po::positional_options_description p;
  p.add("address",  1);
  p.add("frame_id",  1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << endl;
    //usage(progname);
    exit(EXIT_SUCCESS);
  }      

  if (!vm.count("address"))
  {
    cout << desc << endl;
    cerr << "Please specify address of NetFT" << endl;
    exit(EXIT_FAILURE);
  }

  bool publish_wrench = false;
  if (vm.count("wrench"))
  {
    publish_wrench = true;
    ROS_WARN("Publishing NetFT data as geometry_msgs::Wrench is deprecated");
  }

  ros::Publisher ready_pub;
  std_msgs::Bool is_ready;
  ready_pub = nh.advertise<std_msgs::Bool>("netft_ready", 1);
  std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> netft;
  try
  {
    netft = std::shared_ptr<netft_rdt_driver::NetFTRDTDriver>(
      new netft_rdt_driver::NetFTRDTDriver(address, frame_id, counts_per_force, counts_per_torque));
    is_ready.data = true;
    ready_pub.publish(is_ready);
  }
  catch(std::runtime_error)
  {
    is_ready.data = false;
    ready_pub.publish(is_ready);
  }
  
  ros::Publisher pub;
  if (publish_wrench)
  {
    pub = nh.advertise<geometry_msgs::Wrench>("/my_cartesian_force_controller/ft_sensor_wrench", 100);
  }
  else 
  {
    pub = nh.advertise<geometry_msgs::WrenchStamped>("/my_cartesian_force_controller/ft_sensor_wrench", 100);
  }
  ros::Rate pub_rate(pub_rate_hz);
  geometry_msgs::WrenchStamped data;

  ros::Duration diag_pub_duration(1.0);
  ros::Publisher diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 2);
  diagnostic_msgs::DiagnosticArray diag_array;
  diag_array.status.reserve(1);
  diagnostic_updater::DiagnosticStatusWrapper diag_status;
  ros::Time last_diag_pub_time(ros::Time::now());

  while (ros::ok())
  {
    if (netft->waitForNewData())
    {
    tf::StampedTransform tempTransform;
    try
    {
        listener->waitForTransform("base_link", "tool0", ros::Time(0), ros::Duration(1.0));
        listener->lookupTransform("base_link", "tool0", ros::Time(0), tempTransform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    tf::Matrix3x3 ft2world = tf::Matrix3x3(tempTransform.getRotation());
    

      netft->getData(data);
      data.wrench.force.x *= 1000.0;
      data.wrench.force.y *= 1000.0;
      data.wrench.force.z *= 1000.0;
      data.wrench.torque.x *= 250.0;
      data.wrench.torque.y *= 250.0;
      data.wrench.torque.z *= 250.0;
      //添加滤波器
      // 提取数据到输入向量
      std::vector<double> input = {
        data.wrench.force.x,
        data.wrench.force.y,
        data.wrench.force.z,
        data.wrench.torque.x,
        data.wrench.torque.y,
        data.wrench.torque.z
      };

      std::vector<double> output(6, 0.0);

      // 使用LPFilter进行滤波
      if (lp->update(input, output))
      {
        // 将滤波后的数据赋值回data
        data.wrench.force.x = output[0];
        data.wrench.force.y = output[1];
        data.wrench.force.z = output[2];
        data.wrench.torque.x = output[3];
        data.wrench.torque.y = output[4];
        data.wrench.torque.z = output[5];
      }
      else
      {
        ROS_ERROR_STREAM("Failed to update filter.");
      }
        //重力补偿
        tf::Vector3 force_vector(output[0], output[1], output[2]);
        tf::Vector3 torque_vector(output[3], output[4], output[5]);
        tf::Vector3 G_force = ft2world.transpose() * force_vector;
        tf::Vector3 M_torque(
            G_force[2] * tool_mass_center[1] - G_force[1] * tool_mass_center[2],
            G_force[0] * tool_mass_center[2] - G_force[2] * tool_mass_center[0],
            G_force[1] * tool_mass_center[0] - G_force[0] * tool_mass_center[1]);
        data.wrench.force.x = data.wrench.force.x - sensor_zero_point[0] - G_force[0];
        data.wrench.force.y = data.wrench.force.y - sensor_zero_point[1] - G_force[1];
        data.wrench.force.z = data.wrench.force.z - sensor_zero_point[2] - G_force[2];
        data.wrench.torque.x = data.wrench.torque.x - sensor_zero_point[3] - M_torque[0];
        data.wrench.torque.y = data.wrench.torque.y - sensor_zero_point[4] - M_torque[1];
        data.wrench.torque.z = data.wrench.torque.z - sensor_zero_point[5] - M_torque[2];
        
      if (publish_wrench) 
      {
        //geometry_msgs::Wrench(data.wrench);
        pub.publish(data.wrench);
      }
      else 
      {
        pub.publish(data);
      }
    }
    
    ros::Time current_time(ros::Time::now());
    if ( (current_time - last_diag_pub_time) > diag_pub_duration )
    {
      diag_array.status.clear();
      netft->diagnostics(diag_status);
      diag_array.status.push_back(diag_status);
      diag_array.header.stamp = ros::Time::now();
      diag_pub.publish(diag_array);
      ready_pub.publish(is_ready);
      last_diag_pub_time = current_time;
    }
    
    ros::spinOnce();
    pub_rate.sleep();
  }
  
  return 0;
}
