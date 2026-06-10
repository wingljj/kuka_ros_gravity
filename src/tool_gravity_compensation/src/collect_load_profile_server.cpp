#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tool_gravity_compensation/CollectLoadProfileAction.h>
#include <tool_gravity_compensation/ComputePayload.h>
#include <tool_gravity_compensation/SetSamplingConfig.h>
#include <tool_gravity_compensation/StepControl.h>
#include <tool_gravity_compensation/payload_identifier.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <exception>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace tool_gravity_compensation
{
namespace
{
std::array<double, 3> wrenchForce(const geometry_msgs::WrenchStamped& msg)
{
  return {{msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z}};
}

std::array<double, 3> wrenchTorque(const geometry_msgs::WrenchStamped& msg)
{
  return {{msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z}};
}

double maxStddev(const std::deque<geometry_msgs::WrenchStamped>& samples, bool force)
{
  std::array<double, 3> mean{{0.0, 0.0, 0.0}};
  for (const geometry_msgs::WrenchStamped& sample : samples)
  {
    const std::array<double, 3> values = force ? wrenchForce(sample) : wrenchTorque(sample);
    for (std::size_t i = 0; i < values.size(); ++i)
    {
      mean[i] += values[i];
    }
  }
  for (double& value : mean)
  {
    value /= static_cast<double>(samples.size());
  }

  std::array<double, 3> variance{{0.0, 0.0, 0.0}};
  for (const geometry_msgs::WrenchStamped& sample : samples)
  {
    const std::array<double, 3> values = force ? wrenchForce(sample) : wrenchTorque(sample);
    for (std::size_t i = 0; i < values.size(); ++i)
    {
      const double diff = values[i] - mean[i];
      variance[i] += diff * diff;
    }
  }

  double result = 0.0;
  for (double value : variance)
  {
    result = std::max(result, std::sqrt(value / static_cast<double>(samples.size())));
  }
  return result;
}

bool validProfileType(uint8_t profile_type)
{
  return profile_type == StepControl::Request::TOOL_ONLY ||
         profile_type == StepControl::Request::TOOL_PLUS_PAYLOAD;
}

constexpr int kDefaultFilterWindowSize = 10;
constexpr int kDefaultMinStableSamples = 20;
constexpr double kDefaultMaxStddevForce = 2.0;
constexpr double kDefaultMaxStddevTorque = 0.2;
constexpr double kDefaultSampleTimeout = 1.0;
constexpr int kMaxFilterWindowSize = 1000;
constexpr int kMaxMinStableSamples = 1000;
constexpr double kMaxStddevForce = 1000.0;
constexpr double kMaxStddevTorque = 1000.0;
constexpr double kMaxSampleTimeout = 60.0;

bool finiteInRange(double value, double min_value, double max_value)
{
  return std::isfinite(value) && value >= min_value && value <= max_value;
}

}  // namespace

class CollectLoadProfileServer
{
public:
  explicit CollectLoadProfileServer(ros::NodeHandle& nh)
    : nh_(nh)
    , private_nh_("~")
    , server_(nh_, "collect_load_profile",
              boost::bind(&CollectLoadProfileServer::execute, this, _1), false)
  {
    private_nh_.param<std::string>("wrench_topic", wrench_topic_, "/sri_ft_sensor/wrench");
    private_nh_.param<std::string>("sensor_frame", sensor_frame_, "sri_ft_sensor");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param<std::string>("move_group", move_group_, "manipulator");
    private_nh_.param<int>("sampling/filter_window_size", filter_window_size_, kDefaultFilterWindowSize);
    private_nh_.param<double>("sampling/max_stddev_force", max_stddev_force_, kDefaultMaxStddevForce);
    private_nh_.param<double>("sampling/max_stddev_torque", max_stddev_torque_, kDefaultMaxStddevTorque);
    private_nh_.param<int>("sampling/min_stable_samples", min_stable_samples_, kDefaultMinStableSamples);
    private_nh_.param<double>("sampling/sample_timeout", sample_timeout_s_, kDefaultSampleTimeout);
    sanitizeStartupSamplingConfig();
    window_capacity_ = std::max(filter_window_size_, min_stable_samples_);

    wrench_sub_ = nh_.subscribe(wrench_topic_, 100, &CollectLoadProfileServer::handleWrench, this);
    step_service_ = nh_.advertiseService("step_control", &CollectLoadProfileServer::handleStepControl, this);
    compute_service_ = nh_.advertiseService("compute_payload", &CollectLoadProfileServer::handleComputePayload, this);
    sampling_config_service_ = nh_.advertiseService("set_sampling_config",
                                                    &CollectLoadProfileServer::handleSetSamplingConfig, this);
    server_.start();

    ROS_WARN_STREAM("collect_load_profile 处于手动确认模式。"
                    "在启用低速 MoveIt 执行工作流之前，execute_motion 请求将被拒绝。");
    ROS_INFO_STREAM("已配置 wrench_topic=" << wrench_topic_
                    << " sensor_frame=" << sensor_frame_
                    << " base_frame=" << base_frame_
                    << " move_group=" << move_group_
                    << " window=" << window_capacity_
                    << " min_stable_samples=" << min_stable_samples_
                    << " max_stddev_force=" << max_stddev_force_
                    << " max_stddev_torque=" << max_stddev_torque_);
  }

private:
  typedef actionlib::SimpleActionServer<CollectLoadProfileAction> Server;

  void sanitizeStartupSamplingConfig()
  {
    if (filter_window_size_ < 1 || filter_window_size_ > kMaxFilterWindowSize)
    {
      ROS_WARN_STREAM("sampling/filter_window_size 超出安全范围，使用默认值 "
                      << kDefaultFilterWindowSize << "，原始值=" << filter_window_size_);
      filter_window_size_ = kDefaultFilterWindowSize;
    }
    if (min_stable_samples_ < 1 || min_stable_samples_ > kMaxMinStableSamples)
    {
      ROS_WARN_STREAM("sampling/min_stable_samples 超出安全范围，使用默认值 "
                      << kDefaultMinStableSamples << "，原始值=" << min_stable_samples_);
      min_stable_samples_ = kDefaultMinStableSamples;
    }
    if (!finiteInRange(max_stddev_force_, std::numeric_limits<double>::min(), kMaxStddevForce))
    {
      ROS_WARN_STREAM("sampling/max_stddev_force 非法，使用默认值 "
                      << kDefaultMaxStddevForce << "，原始值=" << max_stddev_force_);
      max_stddev_force_ = kDefaultMaxStddevForce;
    }
    if (!finiteInRange(max_stddev_torque_, std::numeric_limits<double>::min(), kMaxStddevTorque))
    {
      ROS_WARN_STREAM("sampling/max_stddev_torque 非法，使用默认值 "
                      << kDefaultMaxStddevTorque << "，原始值=" << max_stddev_torque_);
      max_stddev_torque_ = kDefaultMaxStddevTorque;
    }
    if (!finiteInRange(sample_timeout_s_, std::numeric_limits<double>::min(), kMaxSampleTimeout))
    {
      ROS_WARN_STREAM("sampling/sample_timeout 非法，使用默认值 "
                      << kDefaultSampleTimeout << "，原始值=" << sample_timeout_s_);
      sample_timeout_s_ = kDefaultSampleTimeout;
    }
  }

  bool validateSamplingConfig(const SetSamplingConfig::Request& request, std::string& error) const
  {
    if (request.filter_window_size < 1 || request.min_stable_samples < 1)
    {
      error = "filter_window_size 和 min_stable_samples 必须为正数";
      return false;
    }
    if (request.filter_window_size > kMaxFilterWindowSize)
    {
      std::ostringstream stream;
      stream << "filter_window_size 超过上限 " << kMaxFilterWindowSize;
      error = stream.str();
      return false;
    }
    if (request.min_stable_samples > kMaxMinStableSamples)
    {
      std::ostringstream stream;
      stream << "min_stable_samples 超过上限 " << kMaxMinStableSamples;
      error = stream.str();
      return false;
    }
    if (!finiteInRange(request.max_stddev_force, std::numeric_limits<double>::min(), kMaxStddevForce))
    {
      std::ostringstream stream;
      stream << "max_stddev_force 必须为有限正数且不超过 " << kMaxStddevForce;
      error = stream.str();
      return false;
    }
    if (!finiteInRange(request.max_stddev_torque, std::numeric_limits<double>::min(), kMaxStddevTorque))
    {
      std::ostringstream stream;
      stream << "max_stddev_torque 必须为有限正数且不超过 " << kMaxStddevTorque;
      error = stream.str();
      return false;
    }
    if (!finiteInRange(request.sample_timeout, std::numeric_limits<double>::min(), kMaxSampleTimeout))
    {
      std::ostringstream stream;
      stream << "sample_timeout 必须为有限正数且不超过 " << kMaxSampleTimeout << " 秒";
      error = stream.str();
      return false;
    }
    return true;
  }

  uint32_t samplesForProfile(uint8_t profile_type) const
  {
    if (profile_type == StepControl::Request::TOOL_ONLY)
    {
      return static_cast<uint32_t>(tool_observations_.size());
    }
    if (profile_type == StepControl::Request::TOOL_PLUS_PAYLOAD)
    {
      return static_cast<uint32_t>(total_observations_.size());
    }
    return 0;
  }

  void fillSampleCounts(uint8_t profile_type, StepControl::Response& response) const
  {
    response.samples_collected = samplesForProfile(profile_type);
    response.tool_samples_collected = static_cast<uint32_t>(tool_observations_.size());
    response.total_samples_collected = static_cast<uint32_t>(total_observations_.size());
  }

  void execute(const CollectLoadProfileGoalConstPtr& goal)
  {
    CollectLoadProfileResult result;
    result.result.header.stamp = ros::Time::now();
    result.result.header.frame_id = sensor_frame_;
    if (goal->execute_motion)
    {
      result.success = false;
      result.result.success = false;
      result.message = "自动 MoveIt 执行已禁用；请手动确认后使用 StepControl";
      result.result.message = result.message;
      server_.setAborted(result, result.message);
      return;
    }
    if (!goal->manual_confirmed)
    {
      result.success = false;
      result.result.success = false;
      result.message = "CollectLoadProfile 被拒绝：缺少手动确认";
      result.result.message = result.message;
      server_.setAborted(result, result.message);
      return;
    }

    const uint32_t requested_samples = goal->requested_samples == 0 ? 1 : goal->requested_samples;
    for (uint32_t index = 0; index < requested_samples; ++index)
    {
      if (server_.isPreemptRequested() || !ros::ok())
      {
        result.success = false;
        result.result.success = false;
        result.message = "CollectLoadProfile 在记录所有请求样本前被取消";
        result.result.message = result.message;
        server_.setPreempted(result, result.message);
        return;
      }

      StepControl::Request request;
      StepControl::Response response;
      request.profile_type = goal->profile_type;
      request.execute_motion = false;
      request.sample_wrench = true;
      request.manual_confirmed = goal->manual_confirmed;
      request.step_index = index;
      if (index < goal->pose_names.size())
      {
        request.target_pose_name = goal->pose_names[index];
      }
      if (!handleStepControl(request, response))
      {
        result.success = false;
        result.result.success = false;
        result.message = "StepControl 失败";
        result.result.message = result.message;
        server_.setAborted(result, result.message);
        return;
      }

      CollectLoadProfileFeedback feedback;
      feedback.current_step = index + 1;
      feedback.samples_collected = response.samples_collected;
      feedback.state = response.message;
      server_.publishFeedback(feedback);

      if (!response.accepted)
      {
        result.success = false;
        result.result.success = false;
        result.message = response.message;
        result.result.message = response.message;
        server_.setAborted(result, result.message);
        return;
      }
    }

    result.success = true;
    result.result.success = true;
    result.message = "已记录请求的手动力矩样本";
    result.result.message = result.message;
    server_.setSucceeded(result, result.message);
  }

  void handleWrench(const geometry_msgs::WrenchStampedConstPtr& msg)
  {
    // NaN/Inf guard: discard wrench messages with non-finite values
    if (!std::isfinite(msg->wrench.force.x) || !std::isfinite(msg->wrench.force.y) ||
        !std::isfinite(msg->wrench.force.z) || !std::isfinite(msg->wrench.torque.x) ||
        !std::isfinite(msg->wrench.torque.y) || !std::isfinite(msg->wrench.torque.z))
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "丢弃含 NaN/Inf 值的力矩消息");
      return;
    }
    wrench_window_.push_back(*msg);
    while (wrench_window_.size() > static_cast<std::size_t>(window_capacity_))
    {
      wrench_window_.pop_front();
    }
  }

  bool handleStepControl(StepControl::Request& request, StepControl::Response& response)
  {
    response.accepted = false;
    response.motion_done = false;
    response.sample_recorded = false;
    fillSampleCounts(request.profile_type, response);

    if (request.execute_motion)
    {
      response.message = "StepControl 被拒绝：自动运动执行已禁用";
      return true;
    }
    if (!request.sample_wrench)
    {
      response.message = "StepControl 被拒绝：sample_wrench 必须为 true";
      return true;
    }
    if (!validProfileType(request.profile_type))
    {
      response.message = "StepControl 被拒绝：无效的 profile_type";
      return true;
    }
    if (!request.manual_confirmed)
    {
      response.message = "StepControl 被拒绝：缺少手动确认";
      return true;
    }

    WrenchObservation observation;
    std::string error;
    if (!buildStableObservation(observation, error))
    {
      response.message = error;
      return true;
    }

    if (request.profile_type == StepControl::Request::TOOL_ONLY)
    {
      tool_observations_.push_back(observation);
    }
    else
    {
      total_observations_.push_back(observation);
    }

    response.accepted = true;
    response.sample_recorded = true;
    fillSampleCounts(request.profile_type, response);
    std::ostringstream stream;
    stream << "已记录稳定的力矩样本: TOOL_ONLY="
           << response.tool_samples_collected
           << " TOOL_PLUS_PAYLOAD=" << response.total_samples_collected;
    response.message = stream.str();
    return true;
  }

  bool handleComputePayload(ComputePayload::Request& request, ComputePayload::Response& response)
  {
    response.success = false;
    response.result.header.stamp = ros::Time::now();
    response.result.header.frame_id = sensor_frame_;
    response.result.success = false;

    if (!request.compute)
    {
      response.result.message = "ComputePayload 被拒绝：compute 标志为 false";
      response.message = response.result.message;
      return true;
    }
    if (tool_observations_.size() < request.min_samples || total_observations_.size() < request.min_samples)
    {
      std::ostringstream stream;
      stream << "需要至少 " << request.min_samples
             << " 个 TOOL_ONLY 和 TOOL_PLUS_PAYLOAD 样本；当前有 "
             << tool_observations_.size() << " 和 " << total_observations_.size();
      response.result.message = stream.str();
      response.message = response.result.message;
      return true;
    }

    try
    {
      const LoadInertialProfile tool_profile = estimateLoadProfileFromWrenches(tool_observations_);
      const LoadInertialProfile total_profile = estimateLoadProfileFromWrenches(total_observations_);
      const PayloadMassProperties payload = computePayloadFromProfiles(tool_profile, total_profile);

      response.success = true;
      response.result.success = true;
      response.result.mass_kg = payload.mass_kg;
      response.result.weight_n = payload.weight_n;
      response.result.com_sensor.x = payload.com_sensor_m[0];
      response.result.com_sensor.y = payload.com_sensor_m[1];
      response.result.com_sensor.z = payload.com_sensor_m[2];
      response.result.tool_mass = tool_profile.mass_kg;
      response.result.center_of_mass = response.result.com_sensor;
      response.result.residual_error = std::max(tool_profile.residual_error, total_profile.residual_error);
      response.result.message = "负载质量属性已在 sri_ft_sensor 坐标系中计算";
      response.message = response.result.message;
    }
    catch (const std::exception& ex)
    {
      response.result.message = ex.what();
      response.message = response.result.message;
    }
    return true;
  }

  bool handleSetSamplingConfig(SetSamplingConfig::Request& request, SetSamplingConfig::Response& response)
  {
    response.success = false;
    if (!request.apply)
    {
      response.message = "SetSamplingConfig 被拒绝：apply 标志为 false";
      return true;
    }
    std::string error;
    if (!validateSamplingConfig(request, error))
    {
      response.message = error;
      return true;
    }

    filter_window_size_ = request.filter_window_size;
    min_stable_samples_ = request.min_stable_samples;
    max_stddev_force_ = request.max_stddev_force;
    max_stddev_torque_ = request.max_stddev_torque;
    sample_timeout_s_ = request.sample_timeout;
    window_capacity_ = std::max(filter_window_size_, min_stable_samples_);
    while (wrench_window_.size() > static_cast<std::size_t>(window_capacity_))
    {
      wrench_window_.pop_front();
    }
    if (request.clear_profiles)
    {
      tool_observations_.clear();
      total_observations_.clear();
    }

    private_nh_.setParam("sampling/filter_window_size", filter_window_size_);
    private_nh_.setParam("sampling/min_stable_samples", min_stable_samples_);
    private_nh_.setParam("sampling/max_stddev_force", max_stddev_force_);
    private_nh_.setParam("sampling/max_stddev_torque", max_stddev_torque_);
    private_nh_.setParam("sampling/sample_timeout", sample_timeout_s_);

    std::ostringstream stream;
    stream << "已更新采样配置: window=" << window_capacity_
           << " min_stable_samples=" << min_stable_samples_
           << " max_stddev_force=" << max_stddev_force_
           << " max_stddev_torque=" << max_stddev_torque_
           << " sample_timeout=" << sample_timeout_s_;
    if (request.clear_profiles)
    {
      stream << " 并清除了已采集的配置";
    }
    response.success = true;
    response.message = stream.str();
    return true;
  }

  bool buildStableObservation(WrenchObservation& observation, std::string& error)
  {
    if (wrench_window_.size() < static_cast<std::size_t>(min_stable_samples_))
    {
      error = "StepControl 被拒绝：稳定窗口中力矩样本不足";
      return false;
    }
    if ((ros::Time::now() - wrench_window_.back().header.stamp).toSec() > sample_timeout_s_)
    {
      error = "StepControl 被拒绝：最新力矩样本已过期";
      return false;
    }

    const double force_stddev = maxStddev(wrench_window_, true);
    const double torque_stddev = maxStddev(wrench_window_, false);
    if (force_stddev > max_stddev_force_ || torque_stddev > max_stddev_torque_)
    {
      std::ostringstream stream;
      stream << "StepControl 被拒绝：力矩窗口不稳定 force_stddev="
             << force_stddev << " torque_stddev=" << torque_stddev;
      error = stream.str();
      return false;
    }

    geometry_msgs::WrenchStamped mean;
    for (const geometry_msgs::WrenchStamped& sample : wrench_window_)
    {
      mean.wrench.force.x += sample.wrench.force.x;
      mean.wrench.force.y += sample.wrench.force.y;
      mean.wrench.force.z += sample.wrench.force.z;
      mean.wrench.torque.x += sample.wrench.torque.x;
      mean.wrench.torque.y += sample.wrench.torque.y;
      mean.wrench.torque.z += sample.wrench.torque.z;
    }
    const double inv_count = 1.0 / static_cast<double>(wrench_window_.size());
    mean.wrench.force.x *= inv_count;
    mean.wrench.force.y *= inv_count;
    mean.wrench.force.z *= inv_count;
    mean.wrench.torque.x *= inv_count;
    mean.wrench.torque.y *= inv_count;
    mean.wrench.torque.z *= inv_count;

    tf::StampedTransform transform;
    try
    {
      tf_listener_.lookupTransform(base_frame_, sensor_frame_, ros::Time(0), transform);
    }
    catch (const tf::TransformException& ex)
    {
      error = std::string("StepControl 被拒绝：缺少 TF ") + base_frame_ + " -> " + sensor_frame_ + ": " + ex.what();
      return false;
    }

    const tf::Matrix3x3 basis = transform.getBasis();
    observation.force = wrenchForce(mean);
    observation.torque = wrenchTorque(mean);
    observation.base_R_sensor = {{
      basis[0][0], basis[0][1], basis[0][2],
      basis[1][0], basis[1][1], basis[1][2],
      basis[2][0], basis[2][1], basis[2][2]
    }};
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  Server server_;
  ros::Subscriber wrench_sub_;
  ros::ServiceServer step_service_;
  ros::ServiceServer compute_service_;
  ros::ServiceServer sampling_config_service_;
  tf::TransformListener tf_listener_;
  std::deque<geometry_msgs::WrenchStamped> wrench_window_;
  std::vector<WrenchObservation> tool_observations_;
  std::vector<WrenchObservation> total_observations_;
  std::string wrench_topic_;
  std::string sensor_frame_;
  std::string base_frame_;
  std::string move_group_;
  int filter_window_size_{10};
  int window_capacity_{20};
  double max_stddev_force_{2.0};
  double max_stddev_torque_{0.2};
  int min_stable_samples_{20};
  double sample_timeout_s_{1.0};
};
}  // namespace tool_gravity_compensation

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collect_load_profile_server");
  ros::NodeHandle nh;
  tool_gravity_compensation::CollectLoadProfileServer server(nh);
  ros::spin();
  return 0;
}
