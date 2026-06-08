#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/WrenchStamped.h"
#include <signal.h>
#include "TCPClient.h"
#include "sriforcesensor/SetFilterConfig.h"
#include "sriforcesensor/wrench_filter.h"
#include <Eigen/Dense>
#include <cmath>
#include <exception>
#include <array>
#include <string>
using namespace Eigen;
using namespace std;
TCPClient tcp;
#define M812X_CHN_NUMBER  6
MatrixXd m_dResultChValue=MatrixXd::Zero(1,M812X_CHN_NUMBER); //engineering output of each channel

void sig_exit(int s)
{
  tcp.exit();
  exit(0);
}

bool ConfigSystem(void)
{
  // Make sure that the parameter below is "(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)\r\n", 
  // where a 'E' is included, not 'C'.

  tcp.Send("AT+SGDM=?\r\n");
  string rec = tcp.read();
  ROS_INFO("##############");
  ROS_INFO("重要！请确保下方 [rec_SGDM] 响应为: '(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)'");
  ROS_INFO("1. SGDM 应在 WMA 之前包含 'E'（工程输出），而非 'C'（ADC输出）。");
  ROS_INFO("2. WMA 表示滤波参数，1 表示原始最新数据（无滤波）。");
  if( rec != "" ) {
    ROS_WARN("[rec_SGDM] 响应: %s", rec.c_str());
  }
  else {
    std::cout << "服务器无响应：立即退出！"  << endl;
    return false;
  }
  if (rec.find("(A01,A02,A03,A04,A05,A06);E;") == std::string::npos)
  {
    ROS_ERROR("SRI 传感器输出格式不符合预期的 '(A01,A02,A03,A04,A05,A06);E;'。拒绝发布力矩数据。");
    ROS_ERROR("请在运行负载辨识前将传感器配置为 '(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)'。");
    return false;
  }
  // ### If you want to solve the abot problem, uncomment the following lines to set once ### 
  // std::cout << "Set Sensor channels ..." << endl;
  // tcp.Send("AT+SGDM=(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)\r\n");
  // std::cout << tcp.read();

  tcp.Send("AT+SMPF=?\r\n");
  string rec_hz = tcp.read();
  if( rec_hz != "" )
  {
    ROS_INFO("##############");
    ROS_INFO("请确保发布频率与力采样频率设置相同。");
    ROS_WARN("力传感频率设置为: %s", rec_hz.c_str());
  }
  else {
    std::cout << "服务器无响应：立即退出！" << endl;
    return false;
  }
  // ### If you want to solve the abot problem, uncomment the following lines to set once ### 
  // std::cout << "Set Sensor Sensing Rate ..." << endl;
  // tcp.Send("AT+SMPF=500\r\n");
  // std::cout << tcp.read();

  return true;
}

bool handleSetFilterConfig(sriforcesensor::SetFilterConfig::Request& request,
                           sriforcesensor::SetFilterConfig::Response& response,
                           ros::NodeHandle& private_nh,
                           sriforcesensor::MovingAverageWrenchFilter& filter,
                           bool& filter_enabled,
                           double& max_stddev_force,
                           double& max_stddev_torque)
{
  response.success = false;
  if (!request.apply)
  {
    response.message = "SetFilterConfig 被拒绝：apply 标志为 false";
    return true;
  }

  try
  {
    const sriforcesensor::WrenchFilterConfig config =
        sriforcesensor::validateWrenchFilterConfig(request.enabled,
                                                   request.window_size,
                                                   request.max_stddev_force,
                                                   request.max_stddev_torque);
    filter_enabled = config.enabled;
    filter.setWindowSize(config.window_size);
    filter.reset();
    max_stddev_force = config.max_stddev_force;
    max_stddev_torque = config.max_stddev_torque;

    private_nh.setParam("filter/enabled", filter_enabled);
    private_nh.setParam("filter/window_size", static_cast<int>(config.window_size));
    private_nh.setParam("filter/max_stddev_force", max_stddev_force);
    private_nh.setParam("filter/max_stddev_torque", max_stddev_torque);

    std::ostringstream stream;
    stream << "已更新 SRI ROS 滤波: enabled=" << (filter_enabled ? "true" : "false")
           << " window=" << config.window_size
           << " max_stddev_force=" << max_stddev_force
           << " max_stddev_torque=" << max_stddev_torque;
    response.success = true;
    response.message = stream.str();
  }
  catch (const std::exception& ex)
  {
    response.message = ex.what();
  }
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sri_forcesensor");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  if (argc != 3) {
      std::cerr << "Usage: rosrun rosnode <IP address>  <Publish Rate>" << std::endl;
      std::cerr << "默认 IP: 192.168.0.108 (PC 网络可设置为 192.168.0.2)" << std::endl;
      std::cerr << "默认发布频率可为 200" << std::endl;
      return 1;
  }

  std::string ipAddress = argv[1];
  float l_rate = std::stof(argv[2]);
  std::cout << "IP Address: " << ipAddress << std::endl << "Publish rate:" << l_rate << std::endl;

  signal(SIGINT, sig_exit);
  if(tcp.setup(ipAddress, 4008)==true)
  {
    ROS_INFO("力传感器已连接！\n ");
  }else{
    ROS_ERROR("力传感器连接失败！");
    return -1;
  }
  // initialize the setting of the force sensor
  if (ConfigSystem() == false) return -1;

  // get real time force sensor data
  ROS_INFO("开始发布，频率: %f Hz", l_rate);
  tcp.Send("AT+GSD\r\n");

  geometry_msgs::Twist Forcevalue;
  geometry_msgs::WrenchStamped raw_wrench;
  geometry_msgs::WrenchStamped filtered_wrench;

  std::string frame_id;
  std::string raw_wrench_topic;
  std::string filtered_wrench_topic;
  bool filter_enabled;
  int filter_window_size;
  double max_stddev_force;
  double max_stddev_torque;
  double torque_scale;
  std::string sign_correction_string;
  private_nh.param<std::string>("frame_id", frame_id, "sri_ft_sensor");
  private_nh.param<std::string>("raw_wrench_topic", raw_wrench_topic, "/sri_ft_sensor/raw_wrench");
  private_nh.param<std::string>("filtered_wrench_topic", filtered_wrench_topic, "/sri_ft_sensor/wrench");
  private_nh.param<bool>("filter/enabled", filter_enabled, true);
  private_nh.param<int>("filter/window_size", filter_window_size, 10);
  private_nh.param<double>("filter/max_stddev_force", max_stddev_force, 2.0);
  private_nh.param<double>("filter/max_stddev_torque", max_stddev_torque, 0.2);
  private_nh.param<double>("torque_scale", torque_scale, 1.0);
  private_nh.param<std::string>("sign_correction", sign_correction_string, "1 1 1 1 1 1");
  sriforcesensor::WrenchFilterConfig filter_config;
  try
  {
    filter_config = sriforcesensor::validateWrenchFilterConfig(filter_enabled,
                                                               filter_window_size,
                                                               max_stddev_force,
                                                               max_stddev_torque);
  }
  catch (const std::exception& ex)
  {
    ROS_WARN("无效的 ROS 侧滤波配置 (%s)。回退到 enabled=true window=10 force_stddev=2.0 torque_stddev=0.2。",
             ex.what());
    filter_config = sriforcesensor::validateWrenchFilterConfig(true, 10, 2.0, 0.2);
  }

  filter_enabled = filter_config.enabled;
  filter_window_size = static_cast<int>(filter_config.window_size);
  max_stddev_force = filter_config.max_stddev_force;
  max_stddev_torque = filter_config.max_stddev_torque;

  sriforcesensor::MovingAverageWrenchFilter filter(filter_config.window_size);
  std::array<double, 6> sign_correction{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  {
    std::istringstream stream(sign_correction_string);
    for (std::size_t i = 0; i < sign_correction.size(); ++i)
    {
      if (!(stream >> sign_correction[i]) || (sign_correction[i] != 1.0 && sign_correction[i] != -1.0))
      {
        ROS_ERROR("无效的 sign_correction 参数。请使用六个值，每个为 1 或 -1。");
        return -1;
      }
    }
    double trailing = 0.0;
    if (stream >> trailing)
    {
      ROS_ERROR("无效的 sign_correction 参数。请使用恰好六个值，每个为 1 或 -1。");
      return -1;
    }
  }
  if (torque_scale <= 0.0)
  {
    ROS_ERROR("无效的 torque_scale 参数。Nm 使用 1.0，Nmm 工程输出使用 0.001。");
    return -1;
  }

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("sri_force", 1000);
  ros::Publisher raw_wrench_pub = n.advertise<geometry_msgs::WrenchStamped>(raw_wrench_topic, 1000);
  ros::Publisher filtered_wrench_pub = n.advertise<geometry_msgs::WrenchStamped>(filtered_wrench_topic, 1000);
  ros::ServiceServer filter_config_service = n.advertiseService<sriforcesensor::SetFilterConfig::Request,
                                                                sriforcesensor::SetFilterConfig::Response>(
      "sri_ft_sensor/set_filter_config",
      boost::bind(handleSetFilterConfig, _1, _2, boost::ref(private_nh), boost::ref(filter),
                  boost::ref(filter_enabled), boost::ref(max_stddev_force), boost::ref(max_stddev_torque)));

  ROS_INFO("发布 SRI 原始力矩到 %s，滤波力矩到 %s，坐标系 %s",
           raw_wrench_topic.c_str(), filtered_wrench_topic.c_str(), frame_id.c_str());
  ROS_INFO("ROS 侧滤波: enabled=%s window=%d max_stddev_force=%f max_stddev_torque=%f",
           filter_enabled ? "true" : "false", filter_window_size, max_stddev_force, max_stddev_torque);
  ROS_INFO("SRI 通道映射 A01..A06 -> Fx,Fy,Fz,Tx,Ty,Tz; torque_scale=%f sign_correction=[%f %f %f %f %f %f]",
           torque_scale,
           sign_correction[0], sign_correction[1], sign_correction[2],
           sign_correction[3], sign_correction[4], sign_correction[5]);

  ros::Rate loop_rate(l_rate);
  while (ros::ok())
  {
    // geometry_msgs::Twist
    if (!tcp.readrecieveBuffer_IEEEfloat32(m_dResultChValue))
    {
      ROS_WARN_THROTTLE(1.0, "跳过 SRI 发布：未收到完整有效的工程帧。");
      ros::spinOnce();
      loop_rate.sleep();
      continue;
    }

    // NaN/Inf guard: discard frames with non-finite sensor readings
    {
      bool non_finite = false;
      for (int i = 0; i < M812X_CHN_NUMBER; ++i)
      {
        if (!std::isfinite(m_dResultChValue(0, i)))
        {
          non_finite = true;
          break;
        }
      }
      if (non_finite)
      {
        ROS_WARN_THROTTLE(1.0, "跳过 SRI 发布：传感器帧包含 NaN 或 Inf。");
        ros::spinOnce();
        loop_rate.sleep();
        continue;
      }
    }

    for (int i = 0; i < M812X_CHN_NUMBER; ++i)
    {
      m_dResultChValue(0, i) *= sign_correction[static_cast<std::size_t>(i)];
    }
    m_dResultChValue(0, 3) *= torque_scale;
    m_dResultChValue(0, 4) *= torque_scale;
    m_dResultChValue(0, 5) *= torque_scale;

    Forcevalue.linear.x=m_dResultChValue(0,0);
    Forcevalue.linear.y=m_dResultChValue(0,1);
    Forcevalue.linear.z=m_dResultChValue(0,2);
    Forcevalue.angular.x=m_dResultChValue(0,3);
    Forcevalue.angular.y=m_dResultChValue(0,4);
    Forcevalue.angular.z=m_dResultChValue(0,5);

    raw_wrench.header.stamp = ros::Time::now();
    raw_wrench.header.frame_id = frame_id;
    raw_wrench.wrench.force.x = m_dResultChValue(0,0);
    raw_wrench.wrench.force.y = m_dResultChValue(0,1);
    raw_wrench.wrench.force.z = m_dResultChValue(0,2);
    raw_wrench.wrench.torque.x = m_dResultChValue(0,3);
    raw_wrench.wrench.torque.y = m_dResultChValue(0,4);
    raw_wrench.wrench.torque.z = m_dResultChValue(0,5);

    sriforcesensor::WrenchSample sample{{
      raw_wrench.wrench.force.x,
      raw_wrench.wrench.force.y,
      raw_wrench.wrench.force.z,
      raw_wrench.wrench.torque.x,
      raw_wrench.wrench.torque.y,
      raw_wrench.wrench.torque.z
    }};
    const sriforcesensor::WrenchSample filtered_sample = filter_enabled ? filter.update(sample) : sample;

    filtered_wrench = raw_wrench;
    filtered_wrench.wrench.force.x = filtered_sample.values[0];
    filtered_wrench.wrench.force.y = filtered_sample.values[1];
    filtered_wrench.wrench.force.z = filtered_sample.values[2];
    filtered_wrench.wrench.torque.x = filtered_sample.values[3];
    filtered_wrench.wrench.torque.y = filtered_sample.values[4];
    filtered_wrench.wrench.torque.z = filtered_sample.values[5];

    chatter_pub.publish(Forcevalue);
    raw_wrench_pub.publish(raw_wrench);
    filtered_wrench_pub.publish(filtered_wrench);
    // std::cout << m_dResultChValue << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
