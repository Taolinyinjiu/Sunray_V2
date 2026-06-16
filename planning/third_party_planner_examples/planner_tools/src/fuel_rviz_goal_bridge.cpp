#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

ros::Publisher g_waypoint_pub;

std::string g_rviz_topic = "/move_base_simple/goal";
std::string g_output_topic = "/waypoint_generator/waypoints";
std::string g_odom_topic = "/odom_world";
bool g_use_odom_height = true;
double g_fixed_height = 1.0;

nav_msgs::Odometry g_latest_odom;
bool g_has_odom = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  if (!msg) {
    return;
  }

  g_latest_odom = *msg;
  g_has_odom = true;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (!msg) {
    return;
  }

  nav_msgs::Path path_msg;
  path_msg.header = msg->header;
  if (path_msg.header.stamp.isZero()) {
    path_msg.header.stamp = ros::Time::now();
  }

  geometry_msgs::PoseStamped waypoint = *msg;
  waypoint.header = path_msg.header;

  if (g_use_odom_height) {
    if (!g_has_odom) {
      ROS_WARN_THROTTLE(
          1.0,
          "[planner_tools] fuel_rviz_goal_bridge is waiting for odom on %s before forwarding RViz goal",
          g_odom_topic.c_str());
      return;
    }
    waypoint.pose.position.z = g_latest_odom.pose.pose.position.z;
  } else {
    waypoint.pose.position.z = g_fixed_height;
  }

  path_msg.poses.push_back(waypoint);
  g_waypoint_pub.publish(path_msg);

  ROS_INFO(
      "[planner_tools] fuel_rviz_goal_bridge forwarded RViz goal topic=%s -> %s pos=(%.3f, %.3f, %.3f)",
      g_rviz_topic.c_str(),
      g_output_topic.c_str(),
      waypoint.pose.position.x,
      waypoint.pose.position.y,
      waypoint.pose.position.z);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fuel_rviz_goal_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param("rviz_topic", g_rviz_topic, g_rviz_topic);
  private_nh.param("output_topic", g_output_topic, g_output_topic);
  private_nh.param("odom_topic", g_odom_topic, g_odom_topic);
  private_nh.param("use_odom_height", g_use_odom_height, g_use_odom_height);
  private_nh.param("fixed_height", g_fixed_height, g_fixed_height);

  g_waypoint_pub = nh.advertise<nav_msgs::Path>(g_output_topic, 10);
  ros::Subscriber goal_sub = nh.subscribe(g_rviz_topic, 10, goalCallback);
  ros::Subscriber odom_sub;

  if (g_use_odom_height) {
    odom_sub = nh.subscribe(g_odom_topic, 10, odomCallback);
  }

  ROS_INFO(
      "[planner_tools] fuel_rviz_goal_bridge started rviz_topic=%s output_topic=%s odom_topic=%s use_odom_height=%s fixed_height=%.3f",
      g_rviz_topic.c_str(),
      g_output_topic.c_str(),
      g_odom_topic.c_str(),
      g_use_odom_height ? "true" : "false",
      g_fixed_height);

  ros::spin();
  return 0;
}
