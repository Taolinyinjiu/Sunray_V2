#include "ugv_simulator.h"

#include <boost/array.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace sunray_mavros_sim
{
namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiTitle = "\033[1;36m";
constexpr const char* kAnsiLabel = "\033[1;32m";
constexpr const char* kAnsiWarn = "\033[1;33m";
constexpr const char* kAnsiGood = "\033[1;32m";

void fillDiagonalCovariance(boost::array<double, 36>& covariance,
                            const double xx,
                            const double yy,
                            const double zz,
                            const double rr,
                            const double pp,
                            const double yyaw)
{
    covariance.assign(0.0);
    covariance[0] = xx;
    covariance[7] = yy;
    covariance[14] = zz;
    covariance[21] = rr;
    covariance[28] = pp;
    covariance[35] = yyaw;
}
}  // namespace

UgvSimulator::UgvSimulator(ros::NodeHandle& nh,
                           const std::string& agent_name,
                           const int agent_id,
                           const Eigen::Vector3d& init_pos,
                           const double init_yaw)
    : nh_(nh),
      agent_prefix_("/" + agent_name + std::to_string(std::max(agent_id, 1))),
      agent_frame_prefix_(agent_name + std::to_string(std::max(agent_id, 1))),
      dynamic_params_(loadDynamicParams()),
      dynamics_(dynamic_params_)
{
    nh_.param<std::string>("global_frame_id", global_frame_id_, global_frame_id_);
    nh_.param<double>("ugv/publish_rate", publish_rate_, publish_rate_);
    publish_rate_ = std::max(1.0, publish_rate_);

    base_frame_id_ = agent_frame_prefix_ + "/base_link";
    imu_frame_id_ = agent_frame_prefix_ + "/imu_link";
    nh_.param<std::string>("ugv/base_frame_id", base_frame_id_, base_frame_id_);
    nh_.param<std::string>("ugv/imu_frame_id", imu_frame_id_, imu_frame_id_);

    cmd_topic_ = agent_prefix_ + "/sunray/ugv_control/cmd_vel";
    odom_topic_ = agent_prefix_ + "/sunray_mavros_sim/odom";
    imu_topic_ = agent_prefix_ + "/sunray_mavros_sim/imu";

    dynamics_.reset(init_pos.x(), init_pos.y(), init_yaw);
    imu_model_.configure(loadImuConfig());

    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>(cmd_topic_, 20, &UgvSimulator::cmdCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 20);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 20);

    last_update_time_ = ros::Time::now();
    update_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                    &UgvSimulator::timerCallback,
                                    this);

    ROS_INFO("[ugv_simulator] started for %s init_pos=(%.2f, %.2f) init_yaw=%.2f rad",
             agent_prefix_.c_str(),
             init_pos.x(),
             init_pos.y(),
             init_yaw);
}

UgvDynamicParams UgvSimulator::loadDynamicParams() const
{
    UgvDynamicParams params;

    std::string drive_type = "differential";
    nh_.param<std::string>("ugv/drive_type", drive_type, drive_type);
    std::transform(drive_type.begin(), drive_type.end(), drive_type.begin(), ::tolower);
    params.drive_type =
        (drive_type == "mecanum") ? UgvDynamicParams::MECANUM : UgvDynamicParams::DIFFERENTIAL;

    nh_.param<double>("ugv/max_linear_x", params.max_linear_x, params.max_linear_x);
    nh_.param<double>("ugv/max_linear_y", params.max_linear_y, params.max_linear_y);
    nh_.param<double>("ugv/max_angular_z", params.max_angular_z, params.max_angular_z);
    nh_.param<double>("ugv/linear_acc_limit", params.linear_acc_limit, params.linear_acc_limit);
    nh_.param<double>("ugv/angular_acc_limit", params.angular_acc_limit, params.angular_acc_limit);
    nh_.param<double>("ugv/cmd_timeout", params.cmd_timeout, params.cmd_timeout);
    nh_.param<double>("ugv/odom_covariance_xy", params.odom_covariance_xy, params.odom_covariance_xy);
    nh_.param<double>("ugv/odom_covariance_yaw", params.odom_covariance_yaw, params.odom_covariance_yaw);
    return params;
}

sunray_imu_sim::ImuModelConfig UgvSimulator::loadImuConfig() const
{
    sunray_imu_sim::ImuRosDefaults defaults;
    defaults.preset = "adis16470";
    defaults.orientation_covariance = Eigen::Vector3d(1.0e6, 1.0e6, dynamic_params_.odom_covariance_yaw);
    defaults.angular_velocity_covariance = Eigen::Vector3d::Constant(1.0e-3);
    defaults.linear_acceleration_covariance = Eigen::Vector3d::Constant(2.0e-2);
    return sunray_imu_sim::loadImuModelConfig(nh_, 9.81, defaults);
}

void UgvSimulator::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    last_cmd_ = *msg;
    last_cmd_time_ = ros::Time::now();
    has_received_cmd_ = true;
}

void UgvSimulator::timerCallback(const ros::TimerEvent& event)
{
    const ros::Time now = event.current_real;
    double dt = (now - last_update_time_).toSec();
    if (dt <= 0.0)
    {
        dt = 1.0 / publish_rate_;
    }
    last_update_time_ = now;

    geometry_msgs::Twist cmd_to_apply;
    if (!last_cmd_time_.isZero() &&
        (dynamic_params_.cmd_timeout <= 0.0 ||
         (now - last_cmd_time_).toSec() <= dynamic_params_.cmd_timeout))
    {
        cmd_to_apply = last_cmd_;
    }

    dynamics_.update(cmd_to_apply, dt);
    sim_time_ += dt;
    publishOdometry(now);
    publishImu(now, dt);
}

geometry_msgs::Quaternion UgvSimulator::yawToQuaternion(const double yaw)
{
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
}

void UgvSimulator::publishOdometry(const ros::Time& stamp)
{
    const UgvState& state = dynamics_.state();

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = global_frame_id_;
    odom.child_frame_id = base_frame_id_;

    odom.pose.pose.position.x = state.position.x();
    odom.pose.pose.position.y = state.position.y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = yawToQuaternion(state.yaw);

    odom.twist.twist.linear.x = state.velocity_body.x();
    odom.twist.twist.linear.y = state.velocity_body.y();
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = state.yaw_rate;

    fillDiagonalCovariance(odom.pose.covariance,
                           dynamic_params_.odom_covariance_xy,
                           dynamic_params_.odom_covariance_xy,
                           1.0e6,
                           1.0e6,
                           1.0e6,
                           dynamic_params_.odom_covariance_yaw);
    fillDiagonalCovariance(odom.twist.covariance,
                           dynamic_params_.odom_covariance_xy,
                           dynamic_params_.odom_covariance_xy,
                           1.0e6,
                           1.0e6,
                           1.0e6,
                           dynamic_params_.odom_covariance_yaw);

    odom_pub_.publish(odom);
}

void UgvSimulator::publishImu(const ros::Time& stamp, const double dt)
{
    const UgvState& state = dynamics_.state();
    sunray_imu_sim::ImuTruth truth;
    truth.stamp = stamp;
    truth.frame_id = imu_frame_id_;
    truth.sim_time = sim_time_;
    truth.dt = dt;
    truth.orientation = Eigen::Quaterniond(std::cos(0.5 * state.yaw), 0.0, 0.0, std::sin(0.5 * state.yaw));
    truth.angular_velocity = Eigen::Vector3d(0.0, 0.0, state.yaw_rate);
    truth.body_specific_force = Eigen::Vector3d(state.acceleration_body.x(), state.acceleration_body.y(), 0.0);
    truth.body_angular_acceleration = Eigen::Vector3d(0.0, 0.0, state.yaw_acc);

    imu_pub_.publish(imu_model_.generateMessage(truth));
}

void UgvSimulator::printStatus() const
{
    const UgvState& state = dynamics_.state();
    const bool cmd_alive = has_received_cmd_ &&
                           !last_cmd_time_.isZero() &&
                           (dynamic_params_.cmd_timeout <= 0.0 ||
                            (ros::Time::now() - last_cmd_time_).toSec() <= dynamic_params_.cmd_timeout);
    const bool is_mecanum = dynamic_params_.drive_type == UgvDynamicParams::MECANUM;
    const char* type_color = is_mecanum ? kAnsiGood : kAnsiWarn;
    const char* type_name = is_mecanum ? "麦克纳姆轮" : "差速轮";
    const char* cmd_color = cmd_alive ? kAnsiGood : kAnsiWarn;
    const char* cmd_name = cmd_alive ? "正常" : "超时";

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << kAnsiTitle << "=================== ugv_simulator [" << agent_prefix_
       << "] ===================" << kAnsiReset << "\n";
    ss << kAnsiLabel << " 基本状态 " << kAnsiReset
       << "模型 = " << type_color << type_name << kAnsiReset
       << "  指令状态 = " << cmd_color << cmd_name << kAnsiReset
       << "  更新频率 = " << publish_rate_ << " Hz"
       << "  frame = " << global_frame_id_ << "\n";
    ss << kAnsiLabel << " 当前状态 " << kAnsiReset
       << "x = " << std::setw(6) << state.position.x() << " m"
       << "  y = " << std::setw(6) << state.position.y() << " m"
       << "  yaw = " << std::setw(6) << state.yaw * kRadToDeg << " deg"
       << "  vx = " << std::setw(6) << state.velocity_body.x() << " m/s"
       << "  vy = " << std::setw(6) << state.velocity_body.y() << " m/s"
       << "  wz = " << std::setw(6) << state.yaw_rate * kRadToDeg << " deg/s\n";
    ss << kAnsiLabel << " 话题输入 " << kAnsiReset
       << "cmd_vel -> " << cmd_topic_ << "\n";
    ss << kAnsiLabel << " 话题输出 " << kAnsiReset
       << "odom -> " << odom_topic_
       << "  imu -> " << imu_topic_;

    std::cout << ss.str() << std::endl;
}
}  // namespace sunray_mavros_sim
