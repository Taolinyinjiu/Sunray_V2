#ifndef UGV_CONTROL_CONFIG_H
#define UGV_CONTROL_CONFIG_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <string>

namespace sunray_ugv_control {

struct UGVControlConfig {
  std::string agent_name;
  int agent_id;
  int drive_type;
  double wait_velcmd_time;
  double point_pos_tolerance;
  double point_yaw_tolerance;
  bool enable_geo_fence_protection;
  Eigen::Vector3d fence_min;
  Eigen::Vector3d fence_max;

  static UGVControlConfig loadFromRos(ros::NodeHandle& nh);
  std::string driveTypeName() const;
};

struct UGVControllerParams {
  double kp_linear;
  double kp_angular;
  double max_linear_vel;
  double max_angular_vel;

  static UGVControllerParams loadFromRos(ros::NodeHandle& nh);
};

inline UGVControlConfig UGVControlConfig::loadFromRos(ros::NodeHandle& nh) {
  UGVControlConfig config;

  nh.param<std::string>("agent_name", config.agent_name, "ugv");
  nh.param<int>("agent_id", config.agent_id, 1);
  if (config.agent_id < 1) {
    ROS_WARN("[CONFIG] invalid agent_id=%d, fallback to 1.", config.agent_id);
    config.agent_id = 1;
  }

  nh.param<int>("drive_type", config.drive_type, 2);
  nh.param<double>("wait_velcmd_time", config.wait_velcmd_time, 5.0);
  nh.param<double>("point_pos_tolerance", config.point_pos_tolerance, 0.05);
  nh.param<double>("point_yaw_tolerance", config.point_yaw_tolerance, 0.10);
  nh.param<bool>("enable_geo_fence_protection", config.enable_geo_fence_protection, false);

  nh.param<double>("fence_min_x", config.fence_min.x(), -10.0);
  nh.param<double>("fence_min_y", config.fence_min.y(), -10.0);
  nh.param<double>("fence_min_z", config.fence_min.z(), -1.0);
  nh.param<double>("fence_max_x", config.fence_max.x(), 10.0);
  nh.param<double>("fence_max_y", config.fence_max.y(), 10.0);
  nh.param<double>("fence_max_z", config.fence_max.z(), 1.0);

  return config;
}

inline std::string UGVControlConfig::driveTypeName() const {
  if (drive_type == 1) {
    return "mecanum";
  }
  if (drive_type == 2) {
    return "differential";
  }
  return "unknown";
}

inline UGVControllerParams UGVControllerParams::loadFromRos(ros::NodeHandle& nh) {
  UGVControllerParams params;
  nh.param<double>("kp_linear", params.kp_linear, 0.5);
  nh.param<double>("kp_angular", params.kp_angular, 1.0);
  nh.param<double>("max_linear_vel", params.max_linear_vel, 1.0);
  nh.param<double>("max_angular_vel", params.max_angular_vel, 1.0);
  return params;
}

}  // namespace sunray_ugv_control

#endif  // UGV_CONTROL_CONFIG_H
