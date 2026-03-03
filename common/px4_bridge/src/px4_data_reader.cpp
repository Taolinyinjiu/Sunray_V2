#include "px4_bridge/px4_data_reader.h"

#include "ros/ros.h"

#include <cstdint>
#include <stdexcept>
#include <vector>

void ReaderOptions::enable_all() {
  read_system_state = true;
  read_ekf2_state = true;
  read_flow_state = true;
  read_local_pose = true;
  read_local_velocity = true;
  read_body_pose = true;
  read_body_velocity = true;
}

void ReaderOptions::disable_all() {
  read_system_state = false;
  read_ekf2_state = false;
  read_flow_state = false;
  read_local_pose = false;
  read_local_velocity = false;
  read_body_pose = false;
  read_body_velocity = false;
}

void PX4_DataReader::reset_state_defaults() {
  uav_id_ = 0;
  uav_namespace_.clear();

  system_state_cache_ = {};
  system_state_cache_.uav_id = 0;
  system_state_cache_.uav_name = "null";
  system_state_cache_.connected = false;
  system_state_cache_.armed = false;
  system_state_cache_.rc_input = false;
  system_state_cache_.system_load = 0;
  system_state_cache_.voltage = 0.0f;
  system_state_cache_.current = 0.0f;
  system_state_cache_.percent = 0.0f;
  system_state_cache_.flight_mode = px4_data_types::FlightMode::kUndefined;
  system_state_cache_.landed_state = px4_data_types::LandedState::kUndefined;

  ekf2_state_cache_ = {};
  ekf2_state_cache_.state_codes = 0;
  ekf2_state_cache_.allow_stabilize = false;
  ekf2_state_cache_.allow_altitude = false;
  ekf2_state_cache_.allow_position = false;

  latest_opflow_cache_ = {};
  latest_opflow_cache_.timestamp = 0.0;
  latest_opflow_cache_.quality = 0;
  latest_opflow_cache_.integration_time_us = 0;
  latest_opflow_cache_.integrated_x = 0.0f;
  latest_opflow_cache_.integrated_y = 0.0f;
  latest_opflow_cache_.integrated_xgyro = 0.0f;
  latest_opflow_cache_.integrated_ygyro = 0.0f;
  latest_opflow_cache_.integrated_zgyro = 0.0f;
  latest_opflow_cache_.time_delta_distance_us = 0;
  latest_opflow_cache_.distance = 0.0f;

  local_pose_cache_.position.setZero();
  local_pose_cache_.orientation = Eigen::Quaterniond::Identity();

  local_velocity_cache_.linear.setZero();
  local_velocity_cache_.angular.setZero();

  local_odometry_cache_.timestamp = 0.0;
  local_odometry_cache_.position.setZero();
  local_odometry_cache_.orientation = Eigen::Quaterniond::Identity();
  local_odometry_cache_.linear.setZero();
  local_odometry_cache_.angular.setZero();

  body_pose_cache_.position.setZero();
  body_pose_cache_.orientation = Eigen::Quaterniond::Identity();

  body_velocity_cache_.linear.setZero();
  body_velocity_cache_.angular.setZero();

  ekf2_hgt_ref_cache_ = {};
  ekf2_ev_ctrl_cache_ = {};
  ekf2_ev_delay_cache_ = {};
  ekf2_mag_type_cache_ = {};
  ekf2_mag_check_cache_ = {};
  ekf2_gps_ctrl_cache_ = {};
  ekf2_gps_check_cache_ = {};
  ekf2_gps_delay_cache_ = {};
  px4_rate_pid_cache_ = {};
  px4_velocity_pid_cache_ = {};
  px4_position_p_cache_ = {};
}

void PX4_DataReader::init_service_clients() {
  param_get_client_ = nh_.serviceClient<mavros_msgs::ParamGet>(
      uav_namespace_ + "/mavros/param/get");
}

bool PX4_DataReader::fetch_param_int(const std::string &param_name,
                                    int &out_value) {
  if (!param_get_client_) {
    ROS_WARN("param_get service client is not initialized.");
    return false;
  }

  if (!param_get_client_.exists() &&
      !param_get_client_.waitForExistence(ros::Duration(1.0))) {
    ROS_WARN("param_get service is unavailable for %s", param_name.c_str());
    return false;
  }

  mavros_msgs::ParamGet srv;
  srv.request.param_id = param_name;
  if (!param_get_client_.call(srv)) {
    ROS_WARN("Failed to call param get service for %s", param_name.c_str());
    return false;
  }

  if (!srv.response.success) {
    ROS_WARN("PX4 param %s not found or rejected by FCU", param_name.c_str());
    return false;
  }

  out_value = static_cast<int>(srv.response.value.integer);
  return true;
}

bool PX4_DataReader::fetch_param_float(const std::string &param_name,
                                      float &out_value) {
  if (!param_get_client_) {
    ROS_WARN("param_get service client is not initialized.");
    return false;
  }

  if (!param_get_client_.exists() &&
      !param_get_client_.waitForExistence(ros::Duration(1.0))) {
    ROS_WARN("param_get service is unavailable for %s", param_name.c_str());
    return false;
  }

  mavros_msgs::ParamGet srv;
  srv.request.param_id = param_name;
  if (!param_get_client_.call(srv)) {
    ROS_WARN("Failed to call param get service for %s", param_name.c_str());
    return false;
  }

  if (!srv.response.success) {
    ROS_WARN("PX4 param %s not found or rejected by FCU", param_name.c_str());
    return false;
  }

  out_value = static_cast<float>(srv.response.value.real);
  return true;
}

void PX4_DataReader::init_subscribers(ros::NodeHandle &nh,
                                     ReaderOptions options) {
  std::vector<SubscribeEntry> entries = {
      {&ReaderOptions::read_system_state, &PX4_DataReader::state_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::State>(
             uav_namespace_ + "/mavros/state", 10,
             &PX4_DataReader::state_callback, this);
       }},
      {&ReaderOptions::read_system_state, &PX4_DataReader::exstate_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::ExtendedState>(
             uav_namespace_ + "/mavros/extended_state", 10,
             &PX4_DataReader::extended_state_callback, this);
       }},
      {&ReaderOptions::read_system_state, &PX4_DataReader::sys_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::SysStatus>(
             uav_namespace_ + "/mavros/sys_status", 10,
             &PX4_DataReader::system_status_callback, this);
       }},
      {&ReaderOptions::read_ekf2_state, &PX4_DataReader::ekf2status_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::EstimatorStatus>(
             uav_namespace_ + "/mavros/estimator_status", 10,
             &PX4_DataReader::ekf2_status_callback, this);
       }},
      {&ReaderOptions::read_flow_state, &PX4_DataReader::opflow_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::OpticalFlowRad>(
             uav_namespace_ + "/mavros/px4flow/optical_flow_rad", 10,
             &PX4_DataReader::optical_flow_callback, this);
       }},
      {&ReaderOptions::read_local_pose, &PX4_DataReader::local_odom_sub_,
       [&]() {
         return nh.subscribe<nav_msgs::Odometry>(
             uav_namespace_ + "/mavros/local_position/odom", 10,
             &PX4_DataReader::local_odometry_callback, this);
       }},
      {&ReaderOptions::read_local_velocity, &PX4_DataReader::local_vel_sub_,
       [&]() {
         return nh.subscribe<geometry_msgs::TwistStamped>(
             uav_namespace_ + "/mavros/local_position/velocity_local", 10,
             &PX4_DataReader::local_velocity_callback, this);
       }},
      {&ReaderOptions::read_body_pose, &PX4_DataReader::body_att_sub_,
       [&]() {
         return nh.subscribe<sensor_msgs::Imu>(
             uav_namespace_ + "/mavros/imu/data", 10,
             &PX4_DataReader::body_attitude_callback, this);
       }},
      {&ReaderOptions::read_body_velocity, &PX4_DataReader::body_vel_sub_,
       [&]() {
         return nh.subscribe<geometry_msgs::TwistStamped>(
             uav_namespace_ + "/mavros/local_position/velocity_body", 10,
             &PX4_DataReader::body_velocity_callback, this);
       }},
  };

  for (const auto &entry : entries) {
    if (options.*(entry.flag)) {
      this->*(entry.handle) = entry.make();
    }
  }
}

PX4_DataReader::PX4_DataReader(ros::NodeHandle &nh) {
  reset_state_defaults();
  nh_ = nh;

  try {
    if (!nh.getParam("uav_id", uav_id_)) {
      throw std::runtime_error(
          "无法读取参数 'uav_id'。请检查 YAML 配置文件是否正确加载到 Parameter Server。");
    }
    if (!nh.getParam("uav_name", uav_namespace_)) {
      throw std::runtime_error(
          "无法读取参数 'uav_name'。请检查 YAML 配置文件是否正确加载到 Parameter Server。");
    }
  } catch (const std::exception &e) {
    ROS_FATAL("PX4_DataReader初始化失败: %s", e.what());
    throw;
  }

  uav_namespace_ = "/" + uav_namespace_ + std::to_string(uav_id_);

  ReaderOptions options;
  options.enable_all();
  init_service_clients();
  init_subscribers(nh, options);
}

PX4_DataReader::PX4_DataReader(ros::NodeHandle &nh, ReaderOptions options) {
  reset_state_defaults();
  nh_ = nh;

  try {
    if (!nh.getParam("uav_id", uav_id_)) {
      throw std::runtime_error(
          "无法读取参数 'uav_id'。请检查 YAML 配置文件是否正确加载到 Parameter Server。");
    }
    if (!nh.getParam("uav_name", uav_namespace_)) {
      throw std::runtime_error(
          "无法读取参数 'uav_name'。请检查 YAML 配置文件是否正确加载到 Parameter Server。");
    }
  } catch (const std::exception &e) {
    ROS_FATAL("PX4_DataReader初始化失败: %s", e.what());
    throw;
  }

  uav_namespace_ = "/" + uav_namespace_ + std::to_string(uav_id_);

  init_service_clients();
  init_subscribers(nh, options);
}

PX4_DataReader::~PX4_DataReader() {}

px4_data_types::FlightMode
PX4_DataReader::flight_mode_from_string(const std::string &mode) {
  if (mode == "OFFBOARD")
    return px4_data_types::FlightMode::kOffboard;
  if (mode == "POSCTL")
    return px4_data_types::FlightMode::kPosctl;
  if (mode == "ALTCTL")
    return px4_data_types::FlightMode::kAltctl;
  if (mode == "AUTO.TAKEOFF")
    return px4_data_types::FlightMode::kAutoTakeoff;
  if (mode == "AUTO.LAND")
    return px4_data_types::FlightMode::kAutoLand;
  if (mode == "AUTO.RTL")
    return px4_data_types::FlightMode::kAutoRtl;
  if (mode == "AUTO.MISSION")
    return px4_data_types::FlightMode::kAutoMission;
  if (mode == "AUTO.LOITER")
    return px4_data_types::FlightMode::kAutoLoiter;
  if (mode == "MANUAL")
    return px4_data_types::FlightMode::kManual;
  if (mode == "STABILIZED")
    return px4_data_types::FlightMode::kStabilized;
  if (mode == "ACRO")
    return px4_data_types::FlightMode::kAcro;
  return px4_data_types::FlightMode::kUndefined;
}

px4_data_types::SystemState PX4_DataReader::get_system_state(void) {
  std::lock_guard<std::mutex> lock(system_state_mutex_);
  px4_data_types::SystemState state = system_state_cache_;
  state.uav_id = static_cast<uint8_t>(uav_id_);
  return state;
}

px4_data_types::Ekf2State PX4_DataReader::get_ekf2_state(void) {
  std::lock_guard<std::mutex> lock(ekf2_state_mutex_);
  return ekf2_state_cache_;
}

px4_data_types::OpticalFlow PX4_DataReader::get_opflow_state(void) {
  std::lock_guard<std::mutex> lock(opflow_mutex_);
  return latest_opflow_cache_;
}

px4_data_types::Velocity PX4_DataReader::get_local_velocity(void) {
  std::lock_guard<std::mutex> lock(local_velocity_mutex_);
  return local_velocity_cache_;
}

px4_data_types::Pose PX4_DataReader::get_local_pose(void) {
  std::lock_guard<std::mutex> lock(local_pose_mutex_);
  return local_pose_cache_;
}

px4_data_types::Pose PX4_DataReader::get_body_pose(void) {
  std::lock_guard<std::mutex> lock(body_pose_mutex_);
  return body_pose_cache_;
}

px4_data_types::Velocity PX4_DataReader::get_body_velocity(void) {
  std::lock_guard<std::mutex> lock(body_velocity_mutex_);
  return body_velocity_cache_;
}

px4_param_types::EKF2_HGT_REF PX4_DataReader::fetch_ekf2_hgt_ref(void) {
  px4_param_types::EKF2_HGT_REF out = ekf2_hgt_ref_cache_;
  int raw = 0;
  if (fetch_param_int("EKF2_HGT_REF", raw)) {
    out.setRaw(raw);
  }
  ekf2_hgt_ref_cache_ = out;
  return out;
}

px4_param_types::EKF2_EV_CTRL PX4_DataReader::fetch_ekf2_ev_ctrl(void) {
  px4_param_types::EKF2_EV_CTRL out = ekf2_ev_ctrl_cache_;
  int raw = 0;
  if (fetch_param_int("EKF2_EV_CTRL", raw)) {
    out.mask = static_cast<uint32_t>(raw);
  }
  ekf2_ev_ctrl_cache_ = out;
  return out;
}

px4_param_types::EKF2_EV_DELAY PX4_DataReader::fetch_ekf2_ev_delay(void) {
  px4_param_types::EKF2_EV_DELAY out = ekf2_ev_delay_cache_;
  float raw = 0.0f;
  if (fetch_param_float("EKF2_EV_DELAY", raw)) {
    out.set_ms(raw);
  }
  ekf2_ev_delay_cache_ = out;
  return out;
}

px4_param_types::EKF2_MAG_TYPE PX4_DataReader::fetch_ekf2_mag_type(void) {
  px4_param_types::EKF2_MAG_TYPE out = ekf2_mag_type_cache_;
  int raw = 0;
  if (fetch_param_int("EKF2_MAG_TYPE", raw)) {
    out.setRaw(raw);
  }
  ekf2_mag_type_cache_ = out;
  return out;
}

px4_param_types::EKF2_MAG_CHECK PX4_DataReader::fetch_ekf2_mag_check(void) {
  px4_param_types::EKF2_MAG_CHECK out = ekf2_mag_check_cache_;
  int raw = 0;
  if (fetch_param_int("EKF2_MAG_CHECK", raw)) {
    out.mask = static_cast<uint32_t>(raw);
  }
  ekf2_mag_check_cache_ = out;
  return out;
}

px4_param_types::EKF2_GPS_CTRL PX4_DataReader::fetch_ekf2_gps_ctrl(void) {
  px4_param_types::EKF2_GPS_CTRL out = ekf2_gps_ctrl_cache_;
  int raw = 0;
  if (fetch_param_int("EKF2_GPS_CTRL", raw)) {
    out.mask = static_cast<uint32_t>(raw);
  }
  ekf2_gps_ctrl_cache_ = out;
  return out;
}

px4_param_types::EKF2_GPS_CHECK PX4_DataReader::fetch_ekf2_gps_check(void) {
  px4_param_types::EKF2_GPS_CHECK out = ekf2_gps_check_cache_;
  int raw = 0;
  if (fetch_param_int("EKF2_GPS_CHECK", raw)) {
    out.mask = static_cast<uint32_t>(raw);
  }
  ekf2_gps_check_cache_ = out;
  return out;
}

px4_param_types::EKF2_GPS_DELAY PX4_DataReader::fetch_ekf2_gps_delay(void) {
  px4_param_types::EKF2_GPS_DELAY out = ekf2_gps_delay_cache_;
  float raw = 0.0f;
  if (fetch_param_float("EKF2_GPS_DELAY", raw)) {
    out.set_ms(raw);
  }
  ekf2_gps_delay_cache_ = out;
  return out;
}

px4_param_types::PX4_RATE_PID PX4_DataReader::fetch_px4_rate_pid(void) {
  px4_param_types::PX4_RATE_PID out = px4_rate_pid_cache_;

  fetch_param_float("MC_ROLLRATE_P", out.roll.p.value);
  fetch_param_float("MC_ROLLRATE_I", out.roll.i.value);
  fetch_param_float("MC_ROLLRATE_D", out.roll.d.value);

  fetch_param_float("MC_PITCHRATE_P", out.pitch.p.value);
  fetch_param_float("MC_PITCHRATE_I", out.pitch.i.value);
  fetch_param_float("MC_PITCHRATE_D", out.pitch.d.value);

  fetch_param_float("MC_YAWRATE_P", out.yaw.p.value);
  fetch_param_float("MC_YAWRATE_I", out.yaw.i.value);
  fetch_param_float("MC_YAWRATE_D", out.yaw.d.value);

  px4_rate_pid_cache_ = out;
  return out;
}

px4_param_types::PX4_VELOCITY_PID PX4_DataReader::fetch_px4_velocity_pid(void) {
  px4_param_types::PX4_VELOCITY_PID out = px4_velocity_pid_cache_;

  fetch_param_float("MPC_XY_VEL_P_ACC", out.xy.p_acc.value);
  fetch_param_float("MPC_XY_VEL_I_ACC", out.xy.i_acc.value);
  fetch_param_float("MPC_XY_VEL_D_ACC", out.xy.d_acc.value);

  fetch_param_float("MPC_Z_VEL_P_ACC", out.z.p_acc.value);
  fetch_param_float("MPC_Z_VEL_I_ACC", out.z.i_acc.value);
  fetch_param_float("MPC_Z_VEL_D_ACC", out.z.d_acc.value);

  px4_velocity_pid_cache_ = out;
  return out;
}

px4_param_types::PX4_POSITION_P PX4_DataReader::fetch_px4_position_p(void) {
  px4_param_types::PX4_POSITION_P out = px4_position_p_cache_;

  fetch_param_float("MPC_XY_P", out.xy_p.value);
  fetch_param_float("MPC_Z_P", out.z_p.value);

  px4_position_p_cache_ = out;
  return out;
}
