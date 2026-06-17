#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace
{
struct ControlParams
{
  double kp_velocity = 1.0;
  double max_applied_vel = 1.0;
  double stop_speed_threshold = 0.05;
  bool enable_live_reload = true;
};

std::string readStringParam(const ros::NodeHandle& nh, const std::string& name, const std::string& fallback)
{
  std::string value;
  if (nh.getParam(name, value))
  {
    return value;
  }
  return fallback;
}

int readIntParam(const ros::NodeHandle& nh, const std::string& name, int fallback)
{
  int value = fallback;
  nh.param(name, value, fallback);
  return value;
}

double readDoubleParam(const ros::NodeHandle& nh, const std::string& name, double fallback)
{
  double value = fallback;
  nh.param(name, value, fallback);
  return value;
}

bool readBoolParam(const ros::NodeHandle& nh, const std::string& name, bool fallback)
{
  bool value = fallback;
  nh.param(name, value, fallback);
  return value;
}

std::string joinSunrayTopic(const std::string& agent_key, const std::string& suffix)
{
  return "/" + agent_key + "/sunray/" + suffix;
}

double clamp(double value, double limit)
{
  const double positive_limit = std::max(0.0, limit);
  return std::max(-positive_limit, std::min(positive_limit, value));
}

double norm3(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}

ControlParams loadControlParams(const ros::NodeHandle& pnh, const ControlParams& fallback)
{
  ControlParams params;
  params.kp_velocity = readDoubleParam(pnh, "module_c/kp_velocity", fallback.kp_velocity);
  params.max_applied_vel = readDoubleParam(pnh, "module_c/max_applied_vel", fallback.max_applied_vel);
  params.stop_speed_threshold =
      readDoubleParam(pnh, "module_c/stop_speed_threshold", fallback.stop_speed_threshold);
  params.enable_live_reload =
      readBoolParam(pnh, "module_c/enable_live_reload", fallback.enable_live_reload);
  return params;
}

std::string describeControlParams(const ControlParams& params)
{
  std::ostringstream ss;
  ss << "kp_velocity=" << params.kp_velocity << ", max_applied_vel=" << params.max_applied_vel
     << ", stop_speed_threshold=" << params.stop_speed_threshold
     << ", enable_live_reload=" << (params.enable_live_reload ? "true" : "false");
  return ss.str();
}

class ControlDemo
{
public:
  ControlDemo(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
  {
    const std::string agent_name = readStringParam(pnh_, "agent_name", "uav");
    const int agent_id = readIntParam(pnh_, "agent_id", 1);
    agent_key_ = agent_name + std::to_string(agent_id);
    const std::string shared_ns = "/" + agent_key_ + "/sunray";

    const std::string odom_topic_suffix =
        readStringParam(nh_, shared_ns + "/topics/localization_odom", "localization/local_odom");
    const std::string cmd_topic_suffix =
        readStringParam(nh_, shared_ns + "/topics/planning_cmd", "planning/mock_velocity_cmd");
    const std::string state_topic_suffix =
        readStringParam(nh_, shared_ns + "/topics/control_state", "control/mock_state");

    odom_topic_ = joinSunrayTopic(agent_key_, odom_topic_suffix);
    cmd_topic_ = joinSunrayTopic(agent_key_, cmd_topic_suffix);
    state_topic_ = joinSunrayTopic(agent_key_, state_topic_suffix);

    control_rate_hz_ = readDoubleParam(pnh_, "module_c/control_rate_hz", 20.0);
    refresh_rate_hz_ = readDoubleParam(pnh_, "module_c/refresh_rate_hz", 1.0);
    params_ = loadControlParams(pnh_, params_);

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &ControlDemo::odomCallback, this);
    cmd_sub_ = nh_.subscribe(cmd_topic_, 10, &ControlDemo::cmdCallback, this);
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 10);

    ROS_INFO_STREAM("[module_c_control] agent_key=" << agent_key_);
    ROS_INFO_STREAM("[module_c_control] subscribe odom: " << odom_topic_);
    ROS_INFO_STREAM("[module_c_control] subscribe cmd: " << cmd_topic_);
    ROS_INFO_STREAM("[module_c_control] publish state: " << state_topic_);
    ROS_INFO_STREAM("[module_c_control] initial params: " << describeControlParams(params_));
  }

  void spin()
  {
    ros::Rate rate(std::max(1.0, control_rate_hz_));
    ros::Time last_refresh = ros::Time::now();

    while (ros::ok())
    {
      const ros::Time now = ros::Time::now();
      if (params_.enable_live_reload &&
          (now - last_refresh).toSec() >= (1.0 / std::max(0.1, refresh_rate_hz_)))
      {
        refreshParams();
        last_refresh = now;
      }

      publishState(now);

      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  void cmdCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    latest_cmd_ = *msg;
    has_cmd_ = true;
  }

  void refreshParams()
  {
    const ControlParams old_params = params_;
    params_ = loadControlParams(pnh_, params_);
    if (describeControlParams(old_params) != describeControlParams(params_))
    {
      ROS_WARN_STREAM("[module_c_control] live params updated: " << describeControlParams(params_));
    }
  }

  void publishState(const ros::Time& stamp)
  {
    std_msgs::String state;
    std::ostringstream ss;

    if (!has_odom_ || !has_cmd_)
    {
      ss << "agent=" << agent_key_ << ", status=WAITING_INPUT"
         << ", has_odom=" << (has_odom_ ? "true" : "false")
         << ", has_planning_cmd=" << (has_cmd_ ? "true" : "false");
      state.data = ss.str();
      state_pub_.publish(state);
      return;
    }

    const double applied_vx =
        clamp(params_.kp_velocity * latest_cmd_.twist.linear.x, params_.max_applied_vel);
    const double applied_vy =
        clamp(params_.kp_velocity * latest_cmd_.twist.linear.y, params_.max_applied_vel);
    const double applied_vz =
        clamp(params_.kp_velocity * latest_cmd_.twist.linear.z, params_.max_applied_vel);
    const double applied_speed = norm3(applied_vx, applied_vy, applied_vz);
    const std::string status =
        applied_speed < params_.stop_speed_threshold ? "HOLD_OR_ARRIVED" : "TRACKING";

    ss << "stamp=" << stamp.toSec() << ", agent=" << agent_key_ << ", status=" << status
       << ", odom_xyz=(" << latest_odom_.pose.pose.position.x << ", "
       << latest_odom_.pose.pose.position.y << ", " << latest_odom_.pose.pose.position.z << ")"
       << ", planning_cmd=(" << latest_cmd_.twist.linear.x << ", " << latest_cmd_.twist.linear.y
       << ", " << latest_cmd_.twist.linear.z << ")"
       << ", applied_vel=(" << applied_vx << ", " << applied_vy << ", " << applied_vz << ")"
       << ", params={" << describeControlParams(params_) << "}";

    state.data = ss.str();
    state_pub_.publish(state);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cmd_sub_;
  ros::Publisher state_pub_;

  std::string agent_key_;
  std::string odom_topic_;
  std::string cmd_topic_;
  std::string state_topic_;
  double control_rate_hz_ = 20.0;
  double refresh_rate_hz_ = 1.0;

  ControlParams params_;
  nav_msgs::Odometry latest_odom_;
  geometry_msgs::TwistStamped latest_cmd_;
  bool has_odom_ = false;
  bool has_cmd_ = false;
};
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "module_c_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ControlDemo demo(nh, pnh);
  demo.spin();

  return 0;
}
