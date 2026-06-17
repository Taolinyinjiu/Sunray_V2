#include "fake_mavros_bridge.h"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <cctype>

namespace {
constexpr uint8_t kMavResultAccepted = 0;
constexpr int64_t kEkf2EvCtrlVisionPosYaw = (1 << 0) | (1 << 1) | (1 << 3);
constexpr int64_t kEkf2HgtRefVision = 3;
constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiTitle = "\033[1;36m";
constexpr const char* kAnsiGood = "\033[1;32m";
constexpr const char* kAnsiWarn = "\033[1;33m";
}

FakeMavrosBridge::FakeMavrosBridge(ros::NodeHandle& nh, const std::string& uav_name)
    : nh_(nh), uav_name_(uav_name) {
    nh_.param("fake_mavros_bridge/mavros_publish_rate", mavros_publish_rate_hz_, 50.0);
    mavros_publish_rate_hz_ = std::max(1.0, mavros_publish_rate_hz_);
    nh_.param("fake_mavros_bridge/on_ground_height_threshold_m",
              on_ground_height_threshold_m_,
              0.05);
    nh_.param("fake_mavros_bridge/on_ground_velocity_threshold_mps",
              on_ground_velocity_threshold_mps_,
              0.10);
    nh_.param("fake_mavros_bridge/battery_voltage_v", battery_voltage_v_, 15.2f);
    nh_.param("fake_mavros_bridge/battery_current_a", battery_current_a_, 0.0f);
    nh_.param("fake_mavros_bridge/battery_remaining", battery_remaining_, 0.9f);
    int system_load_raw_param = static_cast<int>(system_load_raw_);
    nh_.param("fake_mavros_bridge/system_load_raw", system_load_raw_param, 150);
    system_load_raw_ = static_cast<uint16_t>(system_load_raw_param);
    nh_.param("fake_mavros_bridge/start_connected", connected_, true);
    nh_.param("fake_mavros_bridge/start_armed", armed_, true);
    nh_.param("fake_mavros_bridge/start_manual_input", manual_input_, true);
    nh_.param("fake_mavros_bridge/start_mode", current_mode_, std::string("OFFBOARD"));

    params_["EKF2_EV_CTRL"] = makeIntegerParamValue(kEkf2EvCtrlVisionPosYaw);
    params_["EKF2_HGT_REF"] = makeIntegerParamValue(kEkf2HgtRefVision);
    params_["EKF2_EV_DELAY"] = makeRealParamValue(0.0);

    odom_sub_ = nh_.subscribe(uav_name_ + "/sunray_mavros_sim/odom",
                              20,
                              &FakeMavrosBridge::odomCallback,
                              this);
    imu_sub_ = nh_.subscribe(uav_name_ + "/sunray_mavros_sim/imu",
                             20,
                             &FakeMavrosBridge::imuCallback,
                             this);
    local_setpoint_sub_ = nh_.subscribe(uav_name_ + "/mavros/setpoint_raw/local",
                                        20,
                                        &FakeMavrosBridge::localSetpointCallback,
                                        this);
    attitude_setpoint_sub_ = nh_.subscribe(uav_name_ + "/mavros/setpoint_raw/attitude",
                                           20,
                                           &FakeMavrosBridge::attitudeSetpointCallback,
                                           this);

    state_pub_ = nh_.advertise<mavros_msgs::State>(uav_name_ + "/mavros/state", 10);
    extended_state_pub_ =
        nh_.advertise<mavros_msgs::ExtendedState>(uav_name_ + "/mavros/extended_state", 10);
    sys_status_pub_ = nh_.advertise<mavros_msgs::SysStatus>(uav_name_ + "/mavros/sys_status", 10);
    estimator_status_pub_ =
        nh_.advertise<mavros_msgs::EstimatorStatus>(uav_name_ + "/mavros/estimator_status", 10);
    local_odom_pub_ =
        nh_.advertise<nav_msgs::Odometry>(uav_name_ + "/mavros/local_position/odom", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(uav_name_ + "/mavros/imu/data", 10);
    target_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        uav_name_ + "/mavros/setpoint_raw/target_local", 10);
    target_attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
        uav_name_ + "/mavros/setpoint_raw/target_attitude", 10);

    set_mode_srv_ = nh_.advertiseService(
        uav_name_ + "/mavros/set_mode", &FakeMavrosBridge::setModeService, this);
    arming_srv_ = nh_.advertiseService(
        uav_name_ + "/mavros/cmd/arming", &FakeMavrosBridge::armingService, this);
    command_long_srv_ = nh_.advertiseService(
        uav_name_ + "/mavros/cmd/command", &FakeMavrosBridge::commandLongService, this);
    param_get_srv_ = nh_.advertiseService(
        uav_name_ + "/mavros/param/get", &FakeMavrosBridge::paramGetService, this);
    param_set_srv_ = nh_.advertiseService(
        uav_name_ + "/mavros/param/set", &FakeMavrosBridge::paramSetService, this);

    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / mavros_publish_rate_hz_),
                                     &FakeMavrosBridge::publishTimerCallback,
                                     this);

    ROS_INFO("[fake_mavros_bridge] started for %s mode=%s armed=%s connected=%s",
             uav_name_.c_str(),
             current_mode_.c_str(),
             armed_ ? "true" : "false",
             connected_ ? "true" : "false");
}

void FakeMavrosBridge::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_odom_ = *msg;
    has_odom_ = true;

    local_odom_pub_.publish(latest_odom_);
}

void FakeMavrosBridge::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    latest_imu_ = *msg;
    has_imu_ = true;

    imu_pub_.publish(latest_imu_);
}

void FakeMavrosBridge::localSetpointCallback(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    latest_local_target_ = *msg;
    latest_local_target_.header.stamp = ros::Time::now();
    last_local_target_time_ = latest_local_target_.header.stamp;
    has_local_target_ = true;
    target_local_pub_.publish(latest_local_target_);
}

void FakeMavrosBridge::attitudeSetpointCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    latest_attitude_target_ = *msg;
    latest_attitude_target_.header.stamp = ros::Time::now();
    last_attitude_target_time_ = latest_attitude_target_.header.stamp;
    has_attitude_target_ = true;
    target_attitude_pub_.publish(latest_attitude_target_);
}

void FakeMavrosBridge::publishTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();

    mavros_msgs::State state_msg;
    state_msg.header.stamp = now;
    state_msg.connected = connected_;
    state_msg.armed = armed_;
    state_msg.guided = true;
    state_msg.manual_input = manual_input_;
    state_msg.mode = current_mode_;
    state_msg.system_status = system_status_;
    state_pub_.publish(state_msg);

    mavros_msgs::ExtendedState extended_state_msg;
    extended_state_msg.header.stamp = now;
    extended_state_msg.vtol_state = mavros_msgs::ExtendedState::VTOL_STATE_UNDEFINED;
    extended_state_msg.landed_state = estimateLandedState();
    extended_state_pub_.publish(extended_state_msg);

    mavros_msgs::SysStatus sys_status_msg;
    sys_status_msg.header.stamp = now;
    sys_status_msg.voltage_battery = static_cast<uint16_t>(battery_voltage_v_ * 1000.0f);
    sys_status_msg.current_battery = static_cast<int16_t>(battery_current_a_ * 100.0f);
    sys_status_msg.battery_remaining = static_cast<int8_t>(battery_remaining_ * 100.0f);
    sys_status_msg.load = system_load_raw_;
    sys_status_pub_.publish(sys_status_msg);

    mavros_msgs::EstimatorStatus estimator_msg;
    estimator_msg.header.stamp = now;
    estimator_msg.attitude_status_flag = true;
    estimator_msg.velocity_horiz_status_flag = true;
    estimator_msg.velocity_vert_status_flag = true;
    estimator_msg.pos_horiz_rel_status_flag = true;
    estimator_msg.pos_horiz_abs_status_flag = true;
    estimator_msg.pos_vert_abs_status_flag = true;
    estimator_msg.pos_vert_agl_status_flag = true;
    estimator_msg.pred_pos_horiz_abs_status_flag = true;
    estimator_msg.pred_pos_horiz_rel_status_flag = true;
    estimator_msg.gps_glitch_status_flag = false;
    estimator_msg.accel_error_status_flag = false;
    estimator_status_pub_.publish(estimator_msg);

    if (has_imu_) {
        latest_imu_.header.stamp = now;
        imu_pub_.publish(latest_imu_);
    }

    if (has_local_target_) {
        latest_local_target_.header.stamp = now;
        target_local_pub_.publish(latest_local_target_);
    }
    if (!has_attitude_target_) {
        latest_attitude_target_.header.stamp = now;
        latest_attitude_target_.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        latest_attitude_target_.body_rate.x = 0.0;
        latest_attitude_target_.body_rate.y = 0.0;
        latest_attitude_target_.body_rate.z = 0.0;
        latest_attitude_target_.thrust = 0.0;
        has_attitude_target_ = true;
    }
    if (has_attitude_target_) {
        latest_attitude_target_.header.stamp = now;
        target_attitude_pub_.publish(latest_attitude_target_);
    }
}

void FakeMavrosBridge::printStatus() const {
    std::cout << buildStatusPanel() << std::endl;
}

bool FakeMavrosBridge::setModeService(mavros_msgs::SetMode::Request& req,
                                      mavros_msgs::SetMode::Response& res) {
    if (!req.custom_mode.empty()) {
        const bool numeric_mode = std::all_of(req.custom_mode.begin(),
                                              req.custom_mode.end(),
                                              [](unsigned char ch) { return std::isdigit(ch); });
        if (numeric_mode) {
            current_mode_ = "CMODE(" + req.custom_mode + ")";
        } else {
            current_mode_ = req.custom_mode;
        }
    }
    connected_ = true;
    res.mode_sent = true;
    ROS_INFO("[fake_mavros_bridge] set_mode request=%s -> state=%s",
             req.custom_mode.c_str(),
             current_mode_.c_str());
    return true;
}

bool FakeMavrosBridge::armingService(mavros_msgs::CommandBool::Request& req,
                                     mavros_msgs::CommandBool::Response& res) {
    armed_ = req.value;
    connected_ = true;
    res.success = true;
    res.result = kMavResultAccepted;
    ROS_INFO("[fake_mavros_bridge] arming -> %s", armed_ ? "true" : "false");
    return true;
}

bool FakeMavrosBridge::commandLongService(mavros_msgs::CommandLong::Request& req,
                                          mavros_msgs::CommandLong::Response& res) {
    if (req.command == 400 && req.param1 == 0.0) {
        armed_ = false;
    }
    res.success = true;
    res.result = kMavResultAccepted;
    ROS_WARN("[fake_mavros_bridge] command_long accepted: command=%u", req.command);
    return true;
}

bool FakeMavrosBridge::paramGetService(mavros_msgs::ParamGet::Request& req,
                                       mavros_msgs::ParamGet::Response& res) {
    const auto it = params_.find(req.param_id);
    if (it == params_.end()) {
        res.success = false;
        res.value = makeIntegerParamValue(0);
        ROS_WARN("[fake_mavros_bridge] param_get miss: %s", req.param_id.c_str());
        return true;
    }

    res.success = true;
    res.value = it->second;
    return true;
}

bool FakeMavrosBridge::paramSetService(mavros_msgs::ParamSet::Request& req,
                                       mavros_msgs::ParamSet::Response& res) {
    params_[req.param_id] = req.value;
    res.success = true;
    res.value = req.value;
    ROS_INFO("[fake_mavros_bridge] param_set %s int=%ld real=%.3f",
             req.param_id.c_str(),
             static_cast<long>(req.value.integer),
             req.value.real);
    return true;
}

uint8_t FakeMavrosBridge::estimateLandedState() const {
    if (!has_odom_) {
        return mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED;
    }
    const double z = latest_odom_.pose.pose.position.z;
    const double vz = latest_odom_.twist.twist.linear.z;
    if (!armed_ && z <= std::max(on_ground_height_threshold_m_, 0.10)) {
        return mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    }
    if (z <= on_ground_height_threshold_m_ &&
        std::fabs(vz) <= on_ground_velocity_threshold_mps_) {
        return mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    }
    return mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
}

mavros_msgs::ParamValue FakeMavrosBridge::makeIntegerParamValue(int64_t value) {
    mavros_msgs::ParamValue param;
    param.integer = value;
    param.real = static_cast<double>(value);
    return param;
}

mavros_msgs::ParamValue FakeMavrosBridge::makeRealParamValue(double value) {
    mavros_msgs::ParamValue param;
    param.integer = static_cast<int64_t>(value);
    param.real = value;
    return param;
}

std::string FakeMavrosBridge::buildStatusPanel() const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    const auto ev_it = params_.find("EKF2_EV_CTRL");
    const auto hgt_it = params_.find("EKF2_HGT_REF");

    ss << kAnsiTitle << "=================== fake_mavros_bridge_node [" << uav_name_
       << "] ===================" << kAnsiReset << "\n";

    ss << kAnsiGood << " 基本状态 " << kAnsiReset
       << "connected = " << (connected_ ? kAnsiGood : kAnsiWarn) << (connected_ ? "true" : "false") << kAnsiReset
       << "  armed = " << (armed_ ? "true" : "false")
       << "  mode = " << current_mode_
       << "  发布频率 = " << mavros_publish_rate_hz_ << " Hz"
       << "\n"
       << "          odom = " << (has_odom_ ? kAnsiGood : kAnsiWarn) << (has_odom_ ? "正常" : "等待") << kAnsiReset
       << "  imu = " << (has_imu_ ? kAnsiGood : kAnsiWarn) << (has_imu_ ? "正常" : "等待") << kAnsiReset
       << "  local_sp = " << (has_local_target_ ? kAnsiGood : kAnsiWarn) << (has_local_target_ ? "正常" : "等待") << kAnsiReset
       << "  attitude_sp = " << (has_attitude_target_ ? kAnsiGood : kAnsiWarn) << (has_attitude_target_ ? "正常" : "等待") << kAnsiReset
       << "\n";

    ss << kAnsiGood << " 订阅话题 " << kAnsiReset
       << "里程计 -> " << uav_name_ << "/sunray_mavros_sim/odom"
       << "\n"
       << "          IMU -> " << uav_name_ << "/sunray_mavros_sim/imu"
       << "\n"
       << "          位置指令 -> " << uav_name_ << "/mavros/setpoint_raw/local"
       << "\n"
       << "          姿态指令 -> " << uav_name_ << "/mavros/setpoint_raw/attitude"
       << "\n";

    ss << kAnsiGood << " 发布话题 " << kAnsiReset
       << "状态 -> " << uav_name_ << "/mavros/state"
       << "\n"
       << "          扩展状态 -> " << uav_name_ << "/mavros/extended_state"
       << "  |  系统状态 -> " << uav_name_ << "/mavros/sys_status"
       << "\n"
       << "          估计器状态 -> " << uav_name_ << "/mavros/estimator_status"
       << "\n";
    ss << "          里程计 -> " << uav_name_ << "/mavros/local_position/odom"
       << "  |  IMU -> " << uav_name_ << "/mavros/imu/data"
       << "\n"
       << "          位置目标 -> " << uav_name_ << "/mavros/setpoint_raw/target_local"
       << "  |  姿态目标 -> " << uav_name_ << "/mavros/setpoint_raw/target_attitude"
       << "\n";
    ss << "          EKF参数: EKF2_EV_CTRL = " << (ev_it != params_.end() ? ev_it->second.integer : 0)
       << "  EKF2_HGT_REF = " << (hgt_it != params_.end() ? hgt_it->second.integer : 0);

    return ss.str();
}
