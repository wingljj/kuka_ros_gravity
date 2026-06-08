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
    private_nh_.param<int>("sampling/filter_window_size", filter_window_size_, 10);
    private_nh_.param<double>("sampling/max_stddev_force", max_stddev_force_, 2.0);
    private_nh_.param<double>("sampling/max_stddev_torque", max_stddev_torque_, 0.2);
    private_nh_.param<int>("sampling/min_stable_samples", min_stable_samples_, 20);
    private_nh_.param<double>("sampling/sample_timeout", sample_timeout_s_, 1.0);
    filter_window_size_ = std::max(1, filter_window_size_);
    min_stable_samples_ = std::max(1, min_stable_samples_);
    window_capacity_ = std::max(filter_window_size_, min_stable_samples_);

    wrench_sub_ = nh_.subscribe(wrench_topic_, 100, &CollectLoadProfileServer::handleWrench, this);
    step_service_ = nh_.advertiseService("step_control", &CollectLoadProfileServer::handleStepControl, this);
    compute_service_ = nh_.advertiseService("compute_payload", &CollectLoadProfileServer::handleComputePayload, this);
    sampling_config_service_ = nh_.advertiseService("set_sampling_config",
                                                    &CollectLoadProfileServer::handleSetSamplingConfig, this);
    server_.start();

    ROS_WARN_STREAM("collect_load_profile is in manual-confirmation mode. "
                    "execute_motion requests are refused until the low-speed MoveIt execution workflow is enabled.");
    ROS_INFO_STREAM("Configured wrench_topic=" << wrench_topic_
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

  void execute(const CollectLoadProfileGoalConstPtr& goal)
  {
    CollectLoadProfileResult result;
    result.result.header.stamp = ros::Time::now();
    result.result.header.frame_id = sensor_frame_;
    if (goal->execute_motion)
    {
      result.success = false;
      result.result.success = false;
      result.message = "Automatic MoveIt execution is disabled; use StepControl after manual confirmation";
      result.result.message = result.message;
      server_.setAborted(result, result.message);
      return;
    }
    if (!goal->manual_confirmed)
    {
      result.success = false;
      result.result.success = false;
      result.message = "CollectLoadProfile refused: missing manual confirmation";
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
        result.message = "CollectLoadProfile canceled before all requested samples were recorded";
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
        result.message = "StepControl failed";
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
    result.message = "Requested manual wrench samples recorded";
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
      ROS_WARN_STREAM_THROTTLE(1.0, "Discarding wrench message with NaN/Inf values");
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
    response.samples_collected = samples_collected_;

    if (request.execute_motion)
    {
      response.message = "StepControl refused: automatic motion execution is disabled";
      return true;
    }
    if (!request.sample_wrench)
    {
      response.message = "StepControl refused: sample_wrench must be true";
      return true;
    }
    if (!validProfileType(request.profile_type))
    {
      response.message = "StepControl refused: invalid profile_type";
      return true;
    }
    if (!request.manual_confirmed)
    {
      response.message = "StepControl refused: missing manual confirmation";
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

    ++samples_collected_;
    response.accepted = true;
    response.sample_recorded = true;
    response.samples_collected = samples_collected_;
    response.message = "Stable wrench sample recorded";
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
      response.result.message = "ComputePayload refused: compute flag is false";
      response.message = response.result.message;
      return true;
    }
    if (tool_observations_.size() < request.min_samples || total_observations_.size() < request.min_samples)
    {
      std::ostringstream stream;
      stream << "Need at least " << request.min_samples
             << " samples for both TOOL_ONLY and TOOL_PLUS_PAYLOAD; have "
             << tool_observations_.size() << " and " << total_observations_.size();
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
      response.result.message = "Payload mass properties computed in sri_ft_sensor frame";
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
      response.message = "SetSamplingConfig refused: apply flag is false";
      return true;
    }
    if (request.filter_window_size < 1 || request.min_stable_samples < 1)
    {
      response.message = "filter_window_size and min_stable_samples must be positive";
      return true;
    }
    if (request.max_stddev_force <= 0.0 || request.max_stddev_torque <= 0.0 || request.sample_timeout <= 0.0)
    {
      response.message = "stability thresholds and sample_timeout must be positive";
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
      samples_collected_ = 0;
    }

    private_nh_.setParam("sampling/filter_window_size", filter_window_size_);
    private_nh_.setParam("sampling/min_stable_samples", min_stable_samples_);
    private_nh_.setParam("sampling/max_stddev_force", max_stddev_force_);
    private_nh_.setParam("sampling/max_stddev_torque", max_stddev_torque_);
    private_nh_.setParam("sampling/sample_timeout", sample_timeout_s_);

    std::ostringstream stream;
    stream << "Updated sampling config: window=" << window_capacity_
           << " min_stable_samples=" << min_stable_samples_
           << " max_stddev_force=" << max_stddev_force_
           << " max_stddev_torque=" << max_stddev_torque_
           << " sample_timeout=" << sample_timeout_s_;
    if (request.clear_profiles)
    {
      stream << " and cleared collected profiles";
    }
    response.success = true;
    response.message = stream.str();
    return true;
  }

  bool buildStableObservation(WrenchObservation& observation, std::string& error)
  {
    if (wrench_window_.size() < static_cast<std::size_t>(min_stable_samples_))
    {
      error = "StepControl refused: not enough wrench samples in the stability window";
      return false;
    }
    if ((ros::Time::now() - wrench_window_.back().header.stamp).toSec() > sample_timeout_s_)
    {
      error = "StepControl refused: latest wrench sample is stale";
      return false;
    }

    const double force_stddev = maxStddev(wrench_window_, true);
    const double torque_stddev = maxStddev(wrench_window_, false);
    if (force_stddev > max_stddev_force_ || torque_stddev > max_stddev_torque_)
    {
      std::ostringstream stream;
      stream << "StepControl refused: unstable wrench window force_stddev="
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
      error = std::string("StepControl refused: missing TF ") + base_frame_ + " -> " + sensor_frame_ + ": " + ex.what();
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
  uint32_t samples_collected_{0};
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
