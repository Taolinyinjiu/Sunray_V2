#include "quadrotor_simulator.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace sunray_sim
{
namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiTitle = "\033[1;33m";
constexpr const char* kAnsiGood = "\033[1;32m";
constexpr const char* kAnsiWarn = "\033[1;33m";

Eigen::Matrix3d rotationBodyToWorld(const Eigen::Vector4d& quat)
{
    return sunray_imu_sim::rotationBodyToWorld(
        Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)));
}
}  // namespace

QuadrotorSimulator::QuadrotorSimulator(ros::NodeHandle& nh,
                                       const std::string& agent_name,
                                       const int agent_id,
                                       const Eigen::Vector3d& init_pos,
                                       const double init_yaw)
    : nh_(nh), agent_prefix_("/" + agent_name + std::to_string(std::max(agent_id, 1)))
{
    agent_frame_prefix_ = agent_name + std::to_string(std::max(agent_id, 1));
    input_.motor_rpm_des = Eigen::Vector4d::Zero();

    nh_.param<std::string>("global_frame_id", global_frame_id_, global_frame_id_);
    base_frame_id_ = agent_frame_prefix_ + "/base_link";
    nh_.param<std::string>("base_frame_id", base_frame_id_, base_frame_id_);
    if (base_frame_id_.empty())
    {
        base_frame_id_ = agent_frame_prefix_ + "/base_link";
    }
    nh_.param<double>("dynamics_update_rate", dynamics_update_rate_, 500.0);
    dynamics_update_rate_ = std::max(1.0, dynamics_update_rate_);
    dt_ = 1.0 / dynamics_update_rate_;
    nh_.param<double>("cmd_timeout", cmd_timeout_, 0.1);

    nh_.param<double>("dynamics/mass", dynamic_params_.mass, 1.5);
    nh_.param<double>("dynamics/gravity", dynamic_params_.gravity, 9.81);
    nh_.param<double>("dynamics/arm_length", dynamic_params_.arm_length, 0.25);
    double ixx = 0.0211;
    double iyy = 0.0219;
    double izz = 0.0366;
    nh_.param<double>("dynamics/Ixx", ixx, ixx);
    nh_.param<double>("dynamics/Iyy", iyy, iyy);
    nh_.param<double>("dynamics/Izz", izz, izz);
    dynamic_params_.I << ixx, 0.0, 0.0,
        0.0, iyy, 0.0,
        0.0, 0.0, izz;

    MotorParams motor_params;
    nh_.param<double>("motor/k_F", motor_params.k_F, 1.5e-5);
    nh_.param<double>("motor/k_T", motor_params.k_T, 2.5e-7);
    nh_.param<double>("motor/tau_up", motor_params.tau_up, 0.012);
    nh_.param<double>("motor/tau_down", motor_params.tau_down, 0.025);
    nh_.param<double>("motor/rpm_min", motor_params.rpm_min, 0.0);
    nh_.param<double>("motor/rpm_max", motor_params.rpm_max, 1200.0);

    quadrotor_.reset(new QuadrotorDynamics(dynamic_params_, motor_params));

    Eigen::Vector4d init_q(std::cos(0.5 * init_yaw), 0.0, 0.0, std::sin(0.5 * init_yaw));
    quadrotor_->reset(init_pos, init_q);
    previous_state_ = quadrotor_->getState();

    const sunray_imu_sim::ImuRosDefaults imu_defaults;
    const sunray_imu_sim::ImuModelConfig imu_config =
        sunray_imu_sim::loadImuModelConfig(nh_, dynamic_params_.gravity, imu_defaults);
    imu_model_.configure(imu_config);

    motor_rpm_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>(
        agent_prefix_ + "/sunray_sim/cmd_RPM", 100, &QuadrotorSimulator::motorRpmCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(agent_prefix_ + "/sunray_sim/odom", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(agent_prefix_ + "/sunray_sim/imu", 10);
    navsat_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(agent_prefix_ + "/sunray_sim/navsat", 10);

    update_timer_ = nh_.createTimer(ros::Duration(dt_),
                                    &QuadrotorSimulator::updateTimerCallback,
                                    this);

    ROS_INFO("[sunray_sim] dynamics started for %s at %.1f Hz, IMU %s",
             agent_prefix_.c_str(),
             dynamics_update_rate_,
             imu_model_.summary().c_str());
}

void QuadrotorSimulator::motorRpmCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 4)
    {
        ROS_WARN("[sunray_sim] cmd_RPM needs 4 values, got %zu", msg->data.size());
        return;
    }

    input_.motor_rpm_des << msg->data[0], msg->data[1], msg->data[2], msg->data[3];
    has_motor_rpm_cmd_ = true;
    last_motor_rpm_msg_time_ = ros::Time::now();
}

void QuadrotorSimulator::updateTimerCallback(const ros::TimerEvent&)
{
    const ros::Time now = ros::Time::now();
    const bool cmd_alive = has_motor_rpm_cmd_ &&
                           (cmd_timeout_ <= 0.0 ||
                            (now - last_motor_rpm_msg_time_).toSec() <= cmd_timeout_);
    if (cmd_alive)
    {
        quadrotor_->update(input_, dt_);
        sim_time_ += dt_;
    }
    else
    {
        quadrotor_->pause();
        input_.motor_rpm_des = Eigen::Vector4d::Zero();
    }

    const DroneState& state = quadrotor_->getState();
    publishOdometry(state);
    publishImu(state, now);
    publishNavSatFix(state, now);

    previous_state_ = state;
    has_previous_state_ = true;
}

Eigen::Vector3d QuadrotorSimulator::computeBodySpecificForce(const DroneState& state, const double dt) const
{
    if (!has_previous_state_ || dt <= 0.0)
    {
        return Eigen::Vector3d(0.0, 0.0, dynamic_params_.gravity);
    }
    const Eigen::Vector3d world_acc = (state.vel - previous_state_.vel) / dt;
    const Eigen::Vector3d gravity_world(0.0, 0.0, -dynamic_params_.gravity);
    const Eigen::Matrix3d body_to_world = rotationBodyToWorld(state.quat);
    return body_to_world.transpose() * (world_acc - gravity_world);
}

Eigen::Vector3d QuadrotorSimulator::computeBodyAngularAcceleration(const DroneState& state, const double dt) const
{
    if (!has_previous_state_ || dt <= 0.0)
    {
        return Eigen::Vector3d::Zero();
    }
    return (state.ang_vel - previous_state_.ang_vel) / dt;
}

void QuadrotorSimulator::publishOdometry(const DroneState& state)
{
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = global_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    odom_msg.pose.pose.position.x = state.pos(0);
    odom_msg.pose.pose.position.y = state.pos(1);
    odom_msg.pose.pose.position.z = state.pos(2);
    odom_msg.pose.pose.orientation.w = state.quat(0);
    odom_msg.pose.pose.orientation.x = state.quat(1);
    odom_msg.pose.pose.orientation.y = state.quat(2);
    odom_msg.pose.pose.orientation.z = state.quat(3);
    odom_msg.twist.twist.linear.x = state.vel(0);
    odom_msg.twist.twist.linear.y = state.vel(1);
    odom_msg.twist.twist.linear.z = state.vel(2);
    odom_msg.twist.twist.angular.x = state.ang_vel(0);
    odom_msg.twist.twist.angular.y = state.ang_vel(1);
    odom_msg.twist.twist.angular.z = state.ang_vel(2);
    odom_pub_.publish(odom_msg);
}

void QuadrotorSimulator::publishImu(const DroneState& state, const ros::Time& stamp)
{
    sunray_imu_sim::ImuTruth truth;
    truth.stamp = stamp;
    truth.frame_id = base_frame_id_;
    truth.sim_time = sim_time_;
    truth.dt = dt_;
    truth.orientation = Eigen::Quaterniond(state.quat(0), state.quat(1), state.quat(2), state.quat(3));
    truth.angular_velocity = state.ang_vel;
    truth.body_specific_force = computeBodySpecificForce(state, dt_);
    truth.body_angular_acceleration = computeBodyAngularAcceleration(state, dt_);
    imu_pub_.publish(imu_model_.generateMessage(truth));
}

void QuadrotorSimulator::publishNavSatFix(const DroneState& state, const ros::Time& stamp)
{
    sensor_msgs::NavSatFix navsat_msg;
    navsat_msg.header.stamp = stamp;
    navsat_msg.header.frame_id = global_frame_id_;
    const double ref_lat = 39.9042;
    const double ref_lon = 116.4074;
    const double ref_alt = 50.0;
    const double cos_lat = std::cos(ref_lat * kDegToRad);
    navsat_msg.latitude = ref_lat + state.pos(1) / 111000.0;
    navsat_msg.longitude = ref_lon + state.pos(0) / (111000.0 * cos_lat);
    navsat_msg.altitude = ref_alt + state.pos(2);
    navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    navsat_msg.position_covariance[0] = 1.0;
    navsat_msg.position_covariance[4] = 1.0;
    navsat_msg.position_covariance[8] = 2.0;
    navsat_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    navsat_pub_.publish(navsat_msg);
}

void QuadrotorSimulator::printStatus() const
{
    const DroneState& state = quadrotor_->getState();
    const bool cmd_alive = has_motor_rpm_cmd_ &&
                           (cmd_timeout_ <= 0.0 ||
                            (ros::Time::now() - last_motor_rpm_msg_time_).toSec() <= cmd_timeout_);
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    ss << kAnsiTitle << "=================== quadrotor_simulator [" << agent_prefix_
       << "] ===================" << kAnsiReset << "\n";

    ss << kAnsiGood << " 基本状态 " << kAnsiReset
       << "电机指令 = " << (cmd_alive ? kAnsiGood : kAnsiWarn) << (cmd_alive ? "正常" : "等待") << kAnsiReset
       << "  全局frame = " << global_frame_id_
       << "  机体frame = " << base_frame_id_
       << "  动力学频率 = " << dynamics_update_rate_ << " Hz"
       << "  dt = " << dt_ << " s"
       << "  cmd超时 = " << cmd_timeout_ << " s"
       << "\n";

    ss << kAnsiGood << " 订阅话题 " << kAnsiReset
       << "电机转速 -> " << agent_prefix_ << "/sunray_sim/cmd_RPM"
       << "\n";

    ss << kAnsiGood << " 发布话题 " << kAnsiReset
       << "里程计 -> " << agent_prefix_ << "/sunray_sim/odom"
       << "\n"
       << "          IMU -> " << agent_prefix_ << "/sunray_sim/imu"
       << "\n"
       << "          GNSS -> " << agent_prefix_ << "/sunray_sim/navsat"
       << "\n";

    ss << kAnsiGood << " 当前状态 " << kAnsiReset
       << "pos = (" << state.pos.x() << ", " << state.pos.y() << ", " << state.pos.z() << ") m"
       << "  vel = (" << state.vel.x() << ", " << state.vel.y() << ", " << state.vel.z() << ") m/s"
       << "\n"
       << "          euler = (roll " << state.euler_angles.x() * kRadToDeg
       << ", pitch " << state.euler_angles.y() * kRadToDeg
       << ", yaw " << state.euler_angles.z() * kRadToDeg << ") deg"
       << "\n"
       << "          rpm = (" << state.motor_rpm(0) << ", " << state.motor_rpm(1)
       << ", " << state.motor_rpm(2) << ", " << state.motor_rpm(3) << ")"
       << "  sim_time = " << sim_time_ << " s"
       << "\n";

    ss << kAnsiGood << " 动力学参数 " << kAnsiReset
       << "mass = " << dynamic_params_.mass << " kg"
       << "  gravity = " << dynamic_params_.gravity << " m/s^2"
       << "  arm_length = " << dynamic_params_.arm_length << " m";

    std::cout << ss.str() << std::endl;
}
}  // namespace sunray_sim
