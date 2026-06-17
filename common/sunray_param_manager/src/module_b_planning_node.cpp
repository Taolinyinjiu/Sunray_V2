#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace
{
struct PlannerParams
{
  double target_x = 3.0;
  double target_y = 0.0;
  double target_z = 1.0;
  double kp_position = 0.8;
  double max_cmd_vel = 1.2;
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

std::string joinAgentFrame(const std::string& agent_key, const std::string& suffix)
{
  return agent_key + "/" + suffix;
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

PlannerParams loadPlannerParams(const ros::NodeHandle& pnh, const PlannerParams& fallback)
{
  PlannerParams params;
  params.target_x = readDoubleParam(pnh, "module_b/target_x", fallback.target_x);
  params.target_y = readDoubleParam(pnh, "module_b/target_y", fallback.target_y);
  params.target_z = readDoubleParam(pnh, "module_b/target_z", fallback.target_z);
  params.kp_position = readDoubleParam(pnh, "module_b/kp_position", fallback.kp_position);
  params.max_cmd_vel = readDoubleParam(pnh, "module_b/max_cmd_vel", fallback.max_cmd_vel);
  params.enable_live_reload =
      readBoolParam(pnh, "module_b/enable_live_reload", fallback.enable_live_reload);
  return params;
}

std::string describePlannerParams(const PlannerParams& params)
{
  std::ostringstream ss;
  ss << "target=(" << params.target_x << ", " << params.target_y << ", " << params.target_z
     << "), kp_position=" << params.kp_position << ", max_cmd_vel=" << params.max_cmd_vel
     << ", enable_live_reload=" << (params.enable_live_reload ? "true" : "false");
  return ss.str();
}

class PlanningDemo
{
public:
  PlanningDemo(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
  {
    const std::string agent_name = readStringParam(pnh_, "agent_name", "uav");
    const int agent_id = readIntParam(pnh_, "agent_id", 1);
    agent_key_ = agent_name + std::to_string(agent_id);
    const std::string shared_ns = "/" + agent_key_ + "/sunray";

    const std::string local_suffix =
        readStringParam(nh_, shared_ns + "/frames/local_suffix", "sunray_local");
    const std::string odom_topic_suffix =
        readStringParam(nh_, shared_ns + "/topics/localization_odom", "localization/local_odom");
    const std::string cmd_topic_suffix =
        readStringParam(nh_, shared_ns + "/topics/planning_cmd", "planning/mock_velocity_cmd");

    local_frame_ = joinAgentFrame(agent_key_, local_suffix);
    odom_topic_ = joinSunrayTopic(agent_key_, odom_topic_suffix);
    cmd_topic_ = joinSunrayTopic(agent_key_, cmd_topic_suffix);

    publish_rate_hz_ = readDoubleParam(pnh_, "module_b/publish_rate_hz", 10.0);
    refresh_rate_hz_ = readDoubleParam(pnh_, "module_b/refresh_rate_hz", 1.0);
    params_ = loadPlannerParams(pnh_, params_);

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &PlanningDemo::odomCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(cmd_topic_, 10);

    ROS_INFO_STREAM("[module_b_planning] agent_key=" << agent_key_);
    ROS_INFO_STREAM("[module_b_planning] subscribe odom: " << odom_topic_);
    ROS_INFO_STREAM("[module_b_planning] publish cmd: " << cmd_topic_);
    ROS_INFO_STREAM("[module_b_planning] frame: " << local_frame_);
    ROS_INFO_STREAM("[module_b_planning] initial params: " << describePlannerParams(params_));
  }

  void spin()
  {
    ros::Rate rate(std::max(1.0, publish_rate_hz_));
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

      publishCommand(now);

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

  void refreshParams()
  {
    const PlannerParams old_params = params_;
    params_ = loadPlannerParams(pnh_, params_);
    if (describePlannerParams(old_params) != describePlannerParams(params_))
    {
      ROS_WARN_STREAM("[module_b_planning] live params updated: " << describePlannerParams(params_));
    }
  }

  void publishCommand(const ros::Time& stamp)
  {
    if (!has_odom_)
    {
      ROS_WARN_THROTTLE(2.0, "[module_b_planning] waiting for odom: %s", odom_topic_.c_str());
      return;
    }

    const double ex = params_.target_x - latest_odom_.pose.pose.position.x;
    const double ey = params_.target_y - latest_odom_.pose.pose.position.y;
    const double ez = params_.target_z - latest_odom_.pose.pose.position.z;

    geometry_msgs::TwistStamped cmd;
    cmd.header.stamp = stamp;
    cmd.header.frame_id = local_frame_;
    cmd.twist.linear.x = clamp(params_.kp_position * ex, params_.max_cmd_vel);
    cmd.twist.linear.y = clamp(params_.kp_position * ey, params_.max_cmd_vel);
    cmd.twist.linear.z = clamp(params_.kp_position * ez, params_.max_cmd_vel);

    cmd_pub_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_pub_;

  std::string agent_key_;
  std::string local_frame_;
  std::string odom_topic_;
  std::string cmd_topic_;
  double publish_rate_hz_ = 10.0;
  double refresh_rate_hz_ = 1.0;

  PlannerParams params_;
  nav_msgs::Odometry latest_odom_;
  bool has_odom_ = false;
};
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "module_b_planning");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PlanningDemo demo(nh, pnh);
  demo.spin();

  return 0;
}
