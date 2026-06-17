#include <cmath>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace
{
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

std::string joinAgentFrame(const std::string& agent_key, const std::string& suffix)
{
  return agent_key + "/" + suffix;
}

std::string joinSunrayTopic(const std::string& agent_key, const std::string& suffix)
{
  return "/" + agent_key + "/sunray/" + suffix;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "module_a_localization");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  const std::string agent_name = readStringParam(pnh, "agent_name", "uav");
  const int agent_id = readIntParam(pnh, "agent_id", 1);
  const std::string agent_key = agent_name + std::to_string(agent_id);
  const std::string shared_ns = "/" + agent_key + "/sunray";

  const std::string local_suffix =
      readStringParam(nh, shared_ns + "/frames/local_suffix", "sunray_local");
  const std::string base_suffix =
      readStringParam(nh, shared_ns + "/frames/base_link_suffix", "base_link");
  const std::string odom_topic_suffix =
      readStringParam(nh, shared_ns + "/topics/localization_odom", "localization/local_odom");

  const std::string local_frame = joinAgentFrame(agent_key, local_suffix);
  const std::string base_frame = joinAgentFrame(agent_key, base_suffix);
  const std::string odom_topic = joinSunrayTopic(agent_key, odom_topic_suffix);

  const double publish_rate_hz = readDoubleParam(pnh, "module_a/publish_rate_hz", 20.0);
  const double start_x = readDoubleParam(pnh, "module_a/start_x", 0.0);
  const double start_y = readDoubleParam(pnh, "module_a/start_y", 0.0);
  const double height = readDoubleParam(pnh, "module_a/height", 1.0);
  const double simulated_vx = readDoubleParam(pnh, "module_a/simulated_vx", 0.05);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);

  ROS_INFO_STREAM("[module_a_localization] agent_key=" << agent_key);
  ROS_INFO_STREAM("[module_a_localization] publish odom: " << odom_topic);
  ROS_INFO_STREAM("[module_a_localization] frame: " << local_frame << " -> " << base_frame);
  ROS_INFO_STREAM("[module_a_localization] module_a params are read once at startup. "
                  << "Changing them with rosparam set requires restarting this node.");

  ros::Rate rate(std::max(1.0, publish_rate_hz));
  const ros::Time start_time = ros::Time::now();

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
    const double elapsed = (now - start_time).toSec();

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = local_frame;
    odom.child_frame_id = base_frame;
    odom.pose.pose.position.x = start_x + simulated_vx * elapsed;
    odom.pose.pose.position.y = start_y;
    odom.pose.pose.position.z = height;
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = simulated_vx;

    odom_pub.publish(odom);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
