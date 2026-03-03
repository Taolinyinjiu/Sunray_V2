#include "px4_bridge/px4_data_reader.h"

void PX4_DataReader::state_callback(const mavros_msgs::State::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(system_state_mutex_);
  system_state_cache_.connected = msg->connected;
  system_state_cache_.armed = msg->armed;
  system_state_cache_.rc_input = msg->manual_input;
  system_state_cache_.flight_mode = flight_mode_from_string(msg->mode);
}

void PX4_DataReader::extended_state_callback(
    const mavros_msgs::ExtendedState::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(system_state_mutex_);
  system_state_cache_.landed_state =
      static_cast<px4_data_types::LandedState>(msg->landed_state);
}

void PX4_DataReader::system_status_callback(
    const mavros_msgs::SysStatus::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(system_state_mutex_);
  system_state_cache_.system_load = msg->load;
  system_state_cache_.voltage = msg->voltage_battery / 1000.0f;
  system_state_cache_.current = msg->current_battery / 100.0f;
  system_state_cache_.percent = msg->battery_remaining;
}

void PX4_DataReader::ekf2_status_callback(
    const mavros_msgs::EstimatorStatus::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(ekf2_state_mutex_);
  ekf2_state_cache_.state_codes = 0u;
  ekf2_state_cache_.allow_stabilize = msg->attitude_status_flag;
  ekf2_state_cache_.allow_altitude = msg->attitude_status_flag &&
                                      msg->velocity_vert_status_flag &&
                                      msg->pos_vert_abs_status_flag;
  ekf2_state_cache_.allow_position = msg->attitude_status_flag &&
                                      msg->velocity_horiz_status_flag &&
                                      msg->pos_horiz_rel_status_flag;

  if (msg->accel_error_status_flag) {
    ekf2_state_cache_.allow_altitude = false;
    ekf2_state_cache_.allow_position = false;
  }
}

void PX4_DataReader::optical_flow_callback(
    const mavros_msgs::OpticalFlowRad::ConstPtr &msg) {
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(opflow_mutex_);
  latest_opflow_cache_.timestamp = msg->header.stamp.toSec();
  latest_opflow_cache_.quality = msg->quality;
  latest_opflow_cache_.integration_time_us = msg->integration_time_us;
  latest_opflow_cache_.integrated_x = msg->integrated_x;
  latest_opflow_cache_.integrated_y = msg->integrated_y;
  latest_opflow_cache_.integrated_xgyro = msg->integrated_xgyro;
  latest_opflow_cache_.integrated_ygyro = msg->integrated_ygyro;
  latest_opflow_cache_.integrated_zgyro = msg->integrated_zgyro;
  latest_opflow_cache_.time_delta_distance_us = msg->time_delta_distance_us;
  latest_opflow_cache_.distance = msg->distance;
}

void PX4_DataReader::local_odometry_callback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(local_pose_mutex_);
  local_odometry_cache_.timestamp = msg->header.stamp.toSec();
  local_odometry_cache_.position.x() = msg->pose.pose.position.x;
  local_odometry_cache_.position.y() = msg->pose.pose.position.y;
  local_odometry_cache_.position.z() = msg->pose.pose.position.z;
  local_odometry_cache_.orientation.w() = msg->pose.pose.orientation.w;
  local_odometry_cache_.orientation.x() = msg->pose.pose.orientation.x;
  local_odometry_cache_.orientation.y() = msg->pose.pose.orientation.y;
  local_odometry_cache_.orientation.z() = msg->pose.pose.orientation.z;
  local_odometry_cache_.linear.x() = msg->twist.twist.linear.x;
  local_odometry_cache_.linear.y() = msg->twist.twist.linear.y;
  local_odometry_cache_.linear.z() = msg->twist.twist.linear.z;
  local_odometry_cache_.angular.x() = msg->twist.twist.angular.x;
  local_odometry_cache_.angular.y() = msg->twist.twist.angular.y;
  local_odometry_cache_.angular.z() = msg->twist.twist.angular.z;

  local_pose_cache_.position = local_odometry_cache_.position;
  local_pose_cache_.orientation = local_odometry_cache_.orientation;
}

void PX4_DataReader::local_velocity_callback(
    const geometry_msgs::TwistStamped::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(local_velocity_mutex_);
  local_velocity_cache_.linear.x() = msg->twist.linear.x;
  local_velocity_cache_.linear.y() = msg->twist.linear.y;
  local_velocity_cache_.linear.z() = msg->twist.linear.z;
  local_velocity_cache_.angular.x() = msg->twist.angular.x;
  local_velocity_cache_.angular.y() = msg->twist.angular.y;
  local_velocity_cache_.angular.z() = msg->twist.angular.z;
}

void PX4_DataReader::body_attitude_callback(
    const sensor_msgs::Imu::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(body_pose_mutex_);
  body_pose_cache_.position.x() = -1.0;
  body_pose_cache_.position.y() = -1.0;
  body_pose_cache_.position.z() = -1.0;
  body_pose_cache_.orientation.w() = msg->orientation.w;
  body_pose_cache_.orientation.x() = msg->orientation.x;
  body_pose_cache_.orientation.y() = msg->orientation.y;
  body_pose_cache_.orientation.z() = msg->orientation.z;
}

void PX4_DataReader::body_velocity_callback(
    const geometry_msgs::TwistStamped::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(body_velocity_mutex_);
  body_velocity_cache_.linear.x() = msg->twist.linear.x;
  body_velocity_cache_.linear.y() = msg->twist.linear.y;
  body_velocity_cache_.linear.z() = msg->twist.linear.z;
  body_velocity_cache_.angular.x() = msg->twist.angular.x;
  body_velocity_cache_.angular.y() = msg->twist.angular.y;
  body_velocity_cache_.angular.z() = msg->twist.angular.z;
}
