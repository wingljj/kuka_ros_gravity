/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** 
 * Simple stand-alone ROS node that takes data from NetFT sensor and
 * Publishes it ROS topic
 */

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


namespace po = boost::program_options;
using namespace std;


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "netft_node");
  ros::NodeHandle nh;

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
