#include "px4_control_sim.h"
#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
#include <sstream>

namespace {
constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiTitle = "\033[1;35m";
constexpr const char* kAnsiLabel = "\033[1;37m";
constexpr const char* kAnsiGood = "\033[1;32m";
constexpr const char* kAnsiWarn = "\033[1;33m";
constexpr const char* kAnsiMuted = "\033[0;90m";

double wrapAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double clampUnit(double value)
{
    if (value > 1.0) return 1.0;
    if (value < -1.0) return -1.0;
    return value;
}

double clampValue(double value, double min_value, double max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

double radToDeg(double rad)
{
    return rad * 180.0 / M_PI;
}

const char* stateText(bool ok)
{
    return ok ? "正常" : "异常";
}

std::string joinNames(const std::vector<std::string>& names)
{
    if (names.empty())
    {
        return "无启用输入字段";
    }

    std::ostringstream ss;
    for (size_t i = 0; i < names.size(); ++i)
    {
        if (i > 0)
        {
            ss << ", ";
        }
        ss << names[i];
    }
    return ss.str();
}
}

namespace sunray_sim
{

Px4ControlSim::Px4ControlSim(ros::NodeHandle& nh, const std::string& uav_name)
    : nh_(nh), uav_name_(uav_name), dt_(0.005)
{
    double px4_update_rate = 200.0;
    nh_.param("px4_update_rate", px4_update_rate, 200.0);
    px4_update_rate = std::max(1.0, px4_update_rate);
    dt_ = 1.0 / px4_update_rate;

    nh_.param("dynamics/mass", mass_, 1.5);
    nh_.param("dynamics/gravity", gravity_, 9.81);
    nh_.param("dynamics/arm_length", motor_arm_length_, 0.25);
    // 这些参数需要和 uav_simulator 动力学模型保持一致，否则同一条 MAVROS 指令
    // 在“控制转换”和“动力学仿真”两边会被解释成不同推力。
    nh_.param("motor/k_F", motor_k_f_, 1.5e-5);
    nh_.param("motor/k_T", motor_k_t_, 2.5e-7);
    nh_.param("motor/rpm_max", motor_max_rpm_, 1200.0);
    nh_.param("limits/max_vel_xy", max_vel_xy_, 0.8);
    nh_.param("limits/max_vel_z", max_vel_z_, 0.6);
    nh_.param("limits/max_acc_xy", max_acc_xy_, 1.2);
    nh_.param("limits/max_acc_z", max_acc_z_, 1.0);
    nh_.param("limits/max_tilt_deg", max_tilt_rad_, 15.0);
    nh_.param("setpoint_timeout", setpoint_timeout_, 0.5);
    setpoint_timeout_ = std::max(0.0, setpoint_timeout_);
    nh_.param("position_control/vel_ff_xy_gain_when_pos_active", vel_ff_xy_gain_when_pos_active_, 1.0);
    nh_.param("velocity_control/acc_xy_lpf_tau", acc_xy_lpf_tau_, 0.0);
    nh_.param("velocity_control/acc_ff_xy_gain", acc_ff_xy_gain_, 1.0);
    nh_.param("limits/max_roll_pitch_rate", max_roll_pitch_rate_, 0.5);
    nh_.param("limits/max_yaw_rate", max_yaw_rate_, 0.6);
    max_tilt_rad_ = max_tilt_rad_ * M_PI / 180.0;

    // 初始化PID参数
    initPIDParams();
    
    // 订阅mavros话题
    attitude_target_sub_ = nh_.subscribe(uav_name_ + "/mavros/setpoint_raw/attitude", 10, &Px4ControlSim::attitudeTargetCallback, this);
    position_target_sub_ = nh_.subscribe(uav_name_ + "/mavros/setpoint_raw/local", 10, &Px4ControlSim::positionTargetCallback, this);
    odom_sub_ = nh_.subscribe(uav_name_ + "/sunray_sim/odom", 10, &Px4ControlSim::odomCallback, this);
    mavros_state_sub_ = nh_.subscribe(uav_name_ + "/mavros/state", 10, &Px4ControlSim::mavrosStateCallback, this);
    
    // 发布电机RPM
    motor_rpm_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(uav_name_ + "/sunray_sim/cmd_RPM", 10);
    update_timer_ = nh_.createTimer(ros::Duration(dt_),
                                    &Px4ControlSim::updateTimerCallback,
                                    this);
    
    // 初始化状态
    current_state_.pos.setZero();
    current_state_.vel.setZero();
    current_state_.euler_angles.setZero();
    current_state_.ang_vel.setZero();
    
    // 初始化目标值（外部输入）
    px4_setpoint_.pos_xy.setZero();
    px4_setpoint_.pos_z = 0.0;
    px4_setpoint_.vel_xy.setZero();
    px4_setpoint_.vel_z = 0.0;
    px4_setpoint_.acc_xy.setZero();
    px4_setpoint_.acc_z = 0.0;
    px4_setpoint_.att_roll_pitch.setZero();
    px4_setpoint_.att_yaw = 0.0;
    px4_setpoint_.bodyrate_roll_pitch.setZero();
    px4_setpoint_.bodyrate_yaw_rate = 0.0;
    px4_setpoint_.collective_thrust = 0.0;
    
    // 初始化控制器期望值（内部计算）
    desired_state_.pos_xy.setZero();
    desired_state_.pos_z = 0.0;
    desired_state_.vel_xy.setZero();
    desired_state_.vel_z = 0.0;
    desired_state_.acc.setZero();
    desired_state_.att_roll_pitch.setZero();
    desired_state_.att_yaw = 0.0;
    desired_state_.bodyrate_roll_pitch.setZero();
    desired_state_.bodyrate_yaw_rate = 0.0;
    desired_state_.thrust.setZero();
    desired_state_.torque.setZero();
    desired_state_.motor_thrust.setZero();
    desired_state_.motor_rpm.setZero();
    
    // 初始化控制模式
    control_mode_ = BODYRATE_CONTROL;
    has_setpoint_ = false;
    use_pos_xy_ = false;
    use_vel_xy_ = false;
    use_acc_xy_ = false;
    use_pos_z_ = false;
    use_vel_z_ = false;
    use_acc_z_ = false;
    use_yaw_ = false;
    use_yaw_rate_ = false;
    
    ROS_INFO("Px4ControlSim initialized for %s at %.1f Hz", uav_name_.c_str(), px4_update_rate);
    ROS_INFO("Px4ControlSim params: mass=%.3f gravity=%.3f k_F=%.6e arm=%.3f rpm_max=%.1f",
             mass_,
             gravity_,
             motor_k_f_,
             motor_arm_length_,
             motor_max_rpm_);
}

void Px4ControlSim::updateTimerCallback(const ros::TimerEvent&)
{
    update();
}

void Px4ControlSim::initPIDParams()
{
    nh_.param("position_control/kp_x", pos_pid_kp_(0), 1.0);
    nh_.param("position_control/kp_y", pos_pid_kp_(1), 1.0);
    nh_.param("position_control/kp_z", pos_pid_kp_(2), 1.5);

    nh_.param("velocity_control/kp_x", vel_pid_kp_(0), 0.8);
    nh_.param("velocity_control/kp_y", vel_pid_kp_(1), 0.8);
    nh_.param("velocity_control/kp_z", vel_pid_kp_(2), 1.0);
    nh_.param("velocity_control/ki_x", vel_pid_ki_(0), 0.0);
    nh_.param("velocity_control/ki_y", vel_pid_ki_(1), 0.0);
    nh_.param("velocity_control/ki_z", vel_pid_ki_(2), 0.0);
    nh_.param("velocity_control/integral_limit", vel_integral_limit_, 1.0);
    vel_integral_limit_ = std::max(0.0, vel_integral_limit_);

    nh_.param("attitude_control/kp_roll", att_pid_kp_(0), 5.0);
    nh_.param("attitude_control/kp_pitch", att_pid_kp_(1), 5.0);
    nh_.param("attitude_control/kp_yaw", att_pid_kp_(2), 3.0);

    nh_.param("bodyrate_control/kp_roll", bodyrate_pid_kp_(0), 2.0);
    nh_.param("bodyrate_control/kp_pitch", bodyrate_pid_kp_(1), 2.0);
    nh_.param("bodyrate_control/kp_yaw", bodyrate_pid_kp_(2), 1.5);
    nh_.param("bodyrate_control/ki_roll", bodyrate_pid_ki_(0), 0.0);
    nh_.param("bodyrate_control/ki_pitch", bodyrate_pid_ki_(1), 0.0);
    nh_.param("bodyrate_control/ki_yaw", bodyrate_pid_ki_(2), 0.0);
    nh_.param("bodyrate_control/integral_limit", bodyrate_integral_limit_, 1.0);
    bodyrate_integral_limit_ = std::max(0.0, bodyrate_integral_limit_);

    vel_pid_integral_.setZero();
    filtered_acc_xy_.setZero();
    bodyrate_pid_integral_.setZero();
}

bool Px4ControlSim::isSupportedPositionTarget(const mavros_msgs::PositionTarget& msg,
                                                std::string& reason) const
{
    const uint16_t mask = msg.type_mask;
    const uint16_t known_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                mavros_msgs::PositionTarget::IGNORE_YAW |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    if ((mask & ~known_mask) != 0)
    {
        reason = "type_mask 含有未知位";
        return false;
    }
    if (msg.coordinate_frame != mavros_msgs::PositionTarget::FRAME_LOCAL_NED)
    {
        reason = "当前只支持 FRAME_LOCAL_NED，BODY/OFFSET 坐标系尚未实现转换";
        return false;
    }
    if ((mask & mavros_msgs::PositionTarget::FORCE) != 0)
    {
        reason = "不支持 FORCE 模式，acceleration_or_force 只按加速度前馈解释";
        return false;
    }

    const bool ignore_px = mask & mavros_msgs::PositionTarget::IGNORE_PX;
    const bool ignore_py = mask & mavros_msgs::PositionTarget::IGNORE_PY;
    const bool ignore_pz = mask & mavros_msgs::PositionTarget::IGNORE_PZ;
    const bool ignore_vx = mask & mavros_msgs::PositionTarget::IGNORE_VX;
    const bool ignore_vy = mask & mavros_msgs::PositionTarget::IGNORE_VY;
    const bool ignore_vz = mask & mavros_msgs::PositionTarget::IGNORE_VZ;
    const bool ignore_ax = mask & mavros_msgs::PositionTarget::IGNORE_AFX;
    const bool ignore_ay = mask & mavros_msgs::PositionTarget::IGNORE_AFY;
    const bool ignore_az = mask & mavros_msgs::PositionTarget::IGNORE_AFZ;
    const bool ignore_yaw = mask & mavros_msgs::PositionTarget::IGNORE_YAW;
    const bool ignore_yaw_rate = mask & mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    if (ignore_px != ignore_py)
    {
        reason = "不支持只给 x 或只给 y 的位置输入，pos_xy 必须成对出现";
        return false;
    }
    if (ignore_vx != ignore_vy)
    {
        reason = "不支持只给 vx 或只给 vy 的速度输入，vel_xy 必须成对出现";
        return false;
    }
    if (ignore_ax != ignore_ay)
    {
        reason = "不支持只给 ax 或只给 ay 的加速度输入，acc_xy 必须成对出现";
        return false;
    }

    const bool use_pos_xy = !ignore_px && !ignore_py;
    const bool use_pos_z = !ignore_pz;
    const bool use_vel_xy = !ignore_vx && !ignore_vy;
    const bool use_vel_z = !ignore_vz;
    const bool use_acc_xy = !ignore_ax && !ignore_ay;
    const bool use_acc_z = !ignore_az;
    const bool use_yaw = !ignore_yaw;
    const bool use_yaw_rate = !ignore_yaw_rate;

    if (!use_pos_xy && !use_pos_z &&
        !use_vel_xy && !use_vel_z &&
        !use_acc_xy && !use_acc_z &&
        !use_yaw && !use_yaw_rate)
    {
        reason = "所有 position/velocity/acceleration/yaw/yaw_rate 字段都被忽略";
        return false;
    }

    if (use_pos_xy && (!std::isfinite(msg.position.x) || !std::isfinite(msg.position.y)))
    {
        reason = "pos_xy 包含非有限数值";
        return false;
    }
    if (use_pos_z && !std::isfinite(msg.position.z))
    {
        reason = "pos_z 包含非有限数值";
        return false;
    }
    if (use_vel_xy && (!std::isfinite(msg.velocity.x) || !std::isfinite(msg.velocity.y)))
    {
        reason = "vel_xy 包含非有限数值";
        return false;
    }
    if (use_vel_z && !std::isfinite(msg.velocity.z))
    {
        reason = "vel_z 包含非有限数值";
        return false;
    }
    if (use_acc_xy && (!std::isfinite(msg.acceleration_or_force.x) ||
                       !std::isfinite(msg.acceleration_or_force.y)))
    {
        reason = "acc_xy 包含非有限数值";
        return false;
    }
    if (use_acc_z && !std::isfinite(msg.acceleration_or_force.z))
    {
        reason = "acc_z 包含非有限数值";
        return false;
    }
    if (use_yaw && !std::isfinite(msg.yaw))
    {
        reason = "yaw 包含非有限数值";
        return false;
    }
    if (use_yaw_rate && !std::isfinite(msg.yaw_rate))
    {
        reason = "yaw_rate 包含非有限数值";
        return false;
    }

    return true;
}

bool Px4ControlSim::isSupportedAttitudeTarget(const mavros_msgs::AttitudeTarget& msg,
                                                std::string& reason) const
{
    const uint8_t mask = msg.type_mask;
    const uint8_t known_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                               mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                               mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE |
                               mavros_msgs::AttitudeTarget::IGNORE_THRUST |
                               mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    if ((mask & ~known_mask) != 0)
    {
        reason = "type_mask 含有未知位";
        return false;
    }
    if ((mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) != 0)
    {
        reason = "不支持 IGNORE_THRUST，姿态/角速度控制必须带 thrust";
        return false;
    }
    if (!std::isfinite(msg.thrust))
    {
        reason = "thrust 包含非有限数值";
        return false;
    }

    const bool ignore_attitude = mask & mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    const bool ignore_roll_rate = mask & mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE;
    const bool ignore_pitch_rate = mask & mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
    const bool ignore_yaw_rate = mask & mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    const bool bodyrate_thrust =
        ignore_attitude && !ignore_roll_rate && !ignore_pitch_rate && !ignore_yaw_rate;
    const bool attitude_thrust =
        !ignore_attitude && ignore_roll_rate && ignore_pitch_rate && ignore_yaw_rate;
    const bool roll_pitch_yawrate_thrust =
        !ignore_attitude && ignore_roll_rate && ignore_pitch_rate && !ignore_yaw_rate;

    if (!bodyrate_thrust && !attitude_thrust && !roll_pitch_yawrate_thrust)
    {
        reason = "仅支持 body_rate+thrust、roll_pitch_yaw+thrust、roll_pitch_yawrate+thrust";
        return false;
    }

    if (bodyrate_thrust &&
        (!std::isfinite(msg.body_rate.x) ||
         !std::isfinite(msg.body_rate.y) ||
         !std::isfinite(msg.body_rate.z)))
    {
        reason = "body_rate 包含非有限数值";
        return false;
    }
    if (roll_pitch_yawrate_thrust && !std::isfinite(msg.body_rate.z))
    {
        reason = "yaw_rate 包含非有限数值";
        return false;
    }
    if (!ignore_attitude)
    {
        const double q_norm =
            std::sqrt(msg.orientation.w * msg.orientation.w +
                      msg.orientation.x * msg.orientation.x +
                      msg.orientation.y * msg.orientation.y +
                      msg.orientation.z * msg.orientation.z);
        if (!std::isfinite(q_norm) || q_norm < 1.0e-6)
        {
            reason = "orientation 四元数无效";
            return false;
        }
    }

    return true;
}

void Px4ControlSim::warnUnsupportedPositionTargetOnce(const mavros_msgs::PositionTarget& msg,
                                                        const std::string& reason)
{
    const uint32_t key = (static_cast<uint32_t>(msg.coordinate_frame) << 16) |
                         static_cast<uint32_t>(msg.type_mask);
    last_rejected_setpoint_reason_ = "PositionTarget frame=" +
                                     std::to_string(static_cast<int>(msg.coordinate_frame)) +
                                     " mask=" + std::to_string(msg.type_mask) +
                                     "，原因：" + reason;
    if (warned_position_target_keys_.insert(key).second)
    {
        ROS_WARN("[px4_control_sim] unsupported PositionTarget frame=%u type_mask=%u: %s",
                 msg.coordinate_frame,
                 msg.type_mask,
                 reason.c_str());
    }
}

void Px4ControlSim::warnUnsupportedAttitudeTargetOnce(const mavros_msgs::AttitudeTarget& msg,
                                                        const std::string& reason)
{
    last_rejected_setpoint_reason_ = "AttitudeTarget mask=" +
                                     std::to_string(static_cast<int>(msg.type_mask)) +
                                     "，原因：" + reason;
    if (warned_attitude_target_masks_.insert(msg.type_mask).second)
    {
        ROS_WARN("[px4_control_sim] unsupported AttitudeTarget type_mask=%u: %s",
                 msg.type_mask,
                 reason.c_str());
    }
}

void Px4ControlSim::markSupportedSetpoint()
{
    has_setpoint_ = true;
    setpoint_timed_out_ = false;
    last_supported_setpoint_time_ = ros::Time::now();
}

void Px4ControlSim::clearControllerOutput()
{
    desired_state_.thrust.setZero();
    desired_state_.torque.setZero();
    desired_state_.motor_thrust.setZero();
    desired_state_.motor_rpm.setZero();
    vel_pid_integral_.setZero();
    bodyrate_pid_integral_.setZero();
    filtered_acc_xy_.setZero();
    filtered_acc_xy_initialized_ = false;
}

void Px4ControlSim::attitudeTargetCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    mavros_msgs::AttitudeTarget att_target = *msg;
    std::string unsupported_reason;
    if (!isSupportedAttitudeTarget(att_target, unsupported_reason))
    {
        warnUnsupportedAttitudeTargetOnce(att_target, unsupported_reason);
        return;
    }

    const uint8_t mask = att_target.type_mask;
    const bool ignore_attitude = mask & mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    const bool ignore_roll_rate = mask & mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE;
    const bool ignore_pitch_rate = mask & mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
    const bool ignore_yaw_rate = mask & mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    
    // AttitudeTarget 在当前实现里只支持 3 类：
    // 1. body_rate + thrust
    // 2. roll_pitch_yaw + thrust
    // 3. roll_pitch_yawrate + thrust
    //
    // 这里不会去支持更自由的 attitude/body_rate 混合叠加语义，
    // 避免和 PositionTarget 一样再引入一套新的解释歧义。

    // 1) 忽略姿态，只给 body_rate + thrust
    if (ignore_attitude && !ignore_roll_rate && !ignore_pitch_rate && !ignore_yaw_rate)
    {
        px4_setpoint_.bodyrate_roll_pitch << att_target.body_rate.x, att_target.body_rate.y;
        px4_setpoint_.bodyrate_yaw_rate = att_target.body_rate.z;
        px4_setpoint_.collective_thrust = att_target.thrust;
        use_yaw_ = false;
        use_yaw_rate_ = false;
        control_mode_ = BODYRATE_CONTROL;
        markSupportedSetpoint();
    }
    // 2) 四元数提供 roll/pitch/yaw，忽略 body_rate
    else if (!ignore_attitude && ignore_roll_rate && ignore_pitch_rate && ignore_yaw_rate)
    {
        double qw = att_target.orientation.w;
        double qx = att_target.orientation.x;
        double qy = att_target.orientation.y;
        double qz = att_target.orientation.z;
        
        double roll = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
        double pitch = asin(clampUnit(2.0 * (qw * qy - qz * qx)));
        double yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        
        px4_setpoint_.att_roll_pitch << roll, pitch;
        px4_setpoint_.att_yaw = yaw;
        px4_setpoint_.collective_thrust = att_target.thrust;
        use_yaw_ = true;
        use_yaw_rate_ = false;
        control_mode_ = ATTITUDE_CONTROL;
        markSupportedSetpoint();
    }
    // 3) 四元数只提供 roll/pitch，yaw 轴由 body_rate.z 直接控制
    else if (!ignore_attitude && ignore_roll_rate && ignore_pitch_rate && !ignore_yaw_rate)
    {
        double qw = att_target.orientation.w;
        double qx = att_target.orientation.x;
        double qy = att_target.orientation.y;
        double qz = att_target.orientation.z;

        double roll = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
        double pitch = asin(clampUnit(2.0 * (qw * qy - qz * qx)));

        px4_setpoint_.att_roll_pitch << roll, pitch;
        px4_setpoint_.bodyrate_yaw_rate = att_target.body_rate.z;
        px4_setpoint_.collective_thrust = att_target.thrust;
        use_yaw_ = false;
        use_yaw_rate_ = true;
        control_mode_ = ATTITUDE_CONTROL;
        markSupportedSetpoint();
    }
}

void Px4ControlSim::positionTargetCallback(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    mavros_msgs::PositionTarget pos_target = *msg;
    std::string unsupported_reason;
    if (!isSupportedPositionTarget(pos_target, unsupported_reason))
    {
        warnUnsupportedPositionTargetOnce(pos_target, unsupported_reason);
        return;
    }

    const uint16_t mask = pos_target.type_mask;

    // PositionTarget 在这里按“字段叠加”解释，而不是“模式互斥”解释：
    // 1. pos_* 提供位置反馈项，进入 positionControl()
    // 2. vel_* 提供速度输入：
    //    - 只有 vel_* 时，它是直接速度目标
    //    - pos_* 和 vel_* 同时存在时，它是速度前馈项
    // 3. acc_* 提供加速度输入：
    //    - 只有 acc_* 时，它是直接加速度目标
    //    - vel_* 和/或 pos_* 同时存在时，它是加速度前馈项
    // 4. yaw 提供偏航角反馈目标
    // 5. yaw_rate 提供偏航角速度输入：
    //    - 只有 yaw_rate 时，它是直接角速度目标
    //    - yaw 和 yaw_rate 同时存在时，它是角速度前馈项
    //
    // 因此像下面这些复合模式都应当能被统一支持：
    // xyz_pos / xyz_pos_yaw / xyz_pos_yawrate
    // xyz_vel / xyz_vel_yaw / xyz_vel_yawrate
    // xy_vel_z_pos / xy_vel_z_pos_yaw / xy_vel_z_pos_yawrate
    // xy_pos_z_vel_yaw / xy_pos_z_vel_yawrate
    // xyz_pos_vel / xyz_pos_vel_yaw / xyz_pos_vel_yawrate
    // xyz_acc / xyz_acc_yaw / xyz_acc_yawrate
    // pos_vel_acc_yaw / pos_vel_acc_yawrate

    // PositionTarget 的 ignore 位是按单轴定义的，这里显式拆开，
    // 避免把 XY/Z 或 position/velocity 的位混淆。
    const bool ignore_px = mask & mavros_msgs::PositionTarget::IGNORE_PX;
    const bool ignore_py = mask & mavros_msgs::PositionTarget::IGNORE_PY;
    const bool ignore_pz = mask & mavros_msgs::PositionTarget::IGNORE_PZ;
    const bool ignore_vx = mask & mavros_msgs::PositionTarget::IGNORE_VX;
    const bool ignore_vy = mask & mavros_msgs::PositionTarget::IGNORE_VY;
    const bool ignore_vz = mask & mavros_msgs::PositionTarget::IGNORE_VZ;
    const bool ignore_ax = mask & mavros_msgs::PositionTarget::IGNORE_AFX;
    const bool ignore_ay = mask & mavros_msgs::PositionTarget::IGNORE_AFY;
    const bool ignore_az = mask & mavros_msgs::PositionTarget::IGNORE_AFZ;
    const bool ignore_yaw = mask & mavros_msgs::PositionTarget::IGNORE_YAW;
    const bool ignore_yaw_rate = mask & mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    use_pos_xy_ = !ignore_px && !ignore_py;
    use_vel_xy_ = !ignore_vx && !ignore_vy;
    use_acc_xy_ = !ignore_ax && !ignore_ay;
    use_pos_z_ = !ignore_pz;
    use_vel_z_ = !ignore_vz;
    use_acc_z_ = !ignore_az;
    use_yaw_ = !ignore_yaw;
    use_yaw_rate_ = !ignore_yaw_rate;

    if (use_pos_xy_) {
        px4_setpoint_.pos_xy << pos_target.position.x, pos_target.position.y;
    }
    if (use_pos_z_) {
        px4_setpoint_.pos_z = pos_target.position.z;
    }

    if (use_vel_xy_) {
        px4_setpoint_.vel_xy << pos_target.velocity.x, pos_target.velocity.y;
    }
    if (use_vel_z_) {
        px4_setpoint_.vel_z = pos_target.velocity.z;
    }

    if (use_acc_xy_) {
        px4_setpoint_.acc_xy << pos_target.acceleration_or_force.x, pos_target.acceleration_or_force.y;
    }
    if (use_acc_z_) {
        px4_setpoint_.acc_z = pos_target.acceleration_or_force.z;
    }

    if (use_yaw_) {
        px4_setpoint_.att_yaw = pos_target.yaw;
    }

    if (use_yaw_rate_) {
        px4_setpoint_.bodyrate_yaw_rate = pos_target.yaw_rate;
    }

    control_mode_ = (use_pos_xy_ || use_pos_z_) ? POSITION_CONTROL : VELOCITY_CONTROL;
    markSupportedSetpoint();
}

void Px4ControlSim::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 更新当前状态
    current_state_.pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    current_state_.vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    current_state_.ang_vel << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
    
    // 从四元数转换为欧拉角
    double qw = msg->pose.pose.orientation.w;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    
    current_state_.euler_angles(0) = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    current_state_.euler_angles(1) = asin(clampUnit(2.0 * (qw * qy - qz * qx)));
    current_state_.euler_angles(2) = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    has_odom_ = true;
}

void Px4ControlSim::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    mavros_armed_ = msg->armed;
    mavros_mode_ = msg->mode;
    has_mavros_state_ = true;

    if (!mavros_armed_)
    {
        has_setpoint_ = false;
        setpoint_timed_out_ = false;
        px4_setpoint_.collective_thrust = 0.0;
        px4_setpoint_.bodyrate_roll_pitch.setZero();
        px4_setpoint_.bodyrate_yaw_rate = 0.0;
        px4_setpoint_.att_roll_pitch.setZero();
        px4_setpoint_.att_yaw = 0.0;
        clearControllerOutput();
        publishZeroMotorRPM();
    }
}

void Px4ControlSim::update()
{
    if (has_mavros_state_ && !mavros_armed_)
    {
        publishZeroMotorRPM();
        return;
    }

    if (has_setpoint_ && setpoint_timeout_ > 0.0 &&
        !last_supported_setpoint_time_.isZero() &&
        (ros::Time::now() - last_supported_setpoint_time_).toSec() > setpoint_timeout_)
    {
        has_setpoint_ = false;
        setpoint_timed_out_ = true;
        clearControllerOutput();
        publishZeroMotorRPM();
        return;
    }

    if (!has_setpoint_)
    {
        // 没收到任何 setpoint 前不输出 RPM，避免节点刚启动就往动力学发零油门。
        return;
    }

    switch (control_mode_)
    {
    case POSITION_CONTROL:
    case VELOCITY_CONTROL:
        positionControl();
        velocityControl();
        accelerationToAttitude();
        // 根据以上计算 得到了 desired_state_.att_roll_pitch, desired_state_.att_yaw
        attitudeControl();
        bodyrateControl();
        break;
        
    case ATTITUDE_CONTROL:
        // 直接外部赋值 desired_state_.att_roll_pitch, desired_state_.att_yaw
        desired_state_.att_roll_pitch(0) = px4_setpoint_.att_roll_pitch(0);
        desired_state_.att_roll_pitch(1) = px4_setpoint_.att_roll_pitch(1);
        desired_state_.att_yaw = px4_setpoint_.att_yaw;
        attitudeControl();
        bodyrateControl();
        desired_state_.thrust = computeRealThrust(px4_setpoint_.collective_thrust);
        break;
        
    case BODYRATE_CONTROL:
        // 直接外部赋值 desired_state_.bodyrate_roll_pitch, desired_state_.bodyrate_yaw_rate
        desired_state_.bodyrate_roll_pitch = px4_setpoint_.bodyrate_roll_pitch;
        desired_state_.bodyrate_yaw_rate = px4_setpoint_.bodyrate_yaw_rate;
        bodyrateControl();
        desired_state_.thrust = computeRealThrust(px4_setpoint_.collective_thrust);
        break;
        
    default:
        desired_state_.acc.setZero();
        desired_state_.att_roll_pitch.setZero();
        desired_state_.att_yaw = 0.0;
        desired_state_.bodyrate_roll_pitch.setZero();
        desired_state_.bodyrate_yaw_rate = 0.0;
        desired_state_.thrust.setZero();
        desired_state_.torque.setZero();
        break;
    }
    
    // 步骤1：计算电机推力
    desired_state_.motor_thrust = computeMotorThrust(desired_state_.thrust(2), desired_state_.torque);
    
    // 步骤2：计算电机RPM
    desired_state_.motor_rpm = computeMotorRPMFromThrust(desired_state_.motor_thrust);
    
    // 步骤3：发布电机RPM
    publishMotorRPM(desired_state_.motor_rpm);
}

void Px4ControlSim::positionControl()
{
    desired_state_.vel_xy.setZero();
    desired_state_.vel_z = 0.0;

    if (use_pos_xy_) {
        Eigen::Vector2d pos_error_xy = px4_setpoint_.pos_xy - current_state_.pos.head<2>();
        desired_state_.vel_xy += pos_pid_kp_.head<2>().cwiseProduct(pos_error_xy);
    }
    if (use_vel_xy_) {
        const double vel_ff_xy_gain = use_pos_xy_ ? vel_ff_xy_gain_when_pos_active_ : 1.0;
        desired_state_.vel_xy += vel_ff_xy_gain * px4_setpoint_.vel_xy;
    }
    const double vel_xy_norm = desired_state_.vel_xy.norm();
    if (vel_xy_norm > max_vel_xy_ && vel_xy_norm > 1e-6) {
        desired_state_.vel_xy *= max_vel_xy_ / vel_xy_norm;
    }

    if (use_pos_z_) {
        double pos_error_z = px4_setpoint_.pos_z - current_state_.pos(2);
        desired_state_.vel_z += pos_pid_kp_(2) * pos_error_z;
    }
    if (use_vel_z_) {
        desired_state_.vel_z += px4_setpoint_.vel_z;
    }
    desired_state_.vel_z = clampValue(desired_state_.vel_z, -max_vel_z_, max_vel_z_);
}

void Px4ControlSim::velocityControl()
{
    Eigen::Vector3d desired_vel;
    desired_vel << desired_state_.vel_xy(0), desired_state_.vel_xy(1), desired_state_.vel_z;
    
    Eigen::Vector3d vel_error = desired_vel - current_state_.vel;// 计算积分项
    vel_pid_integral_ += vel_error * dt_;
    vel_pid_integral_ = vel_pid_integral_.cwiseMax(-vel_integral_limit_).cwiseMin(vel_integral_limit_);
    
    desired_state_.acc = vel_pid_kp_.cwiseProduct(vel_error) +
                         vel_pid_ki_.cwiseProduct(vel_pid_integral_);
    if (use_acc_xy_) {
        desired_state_.acc.head<2>() += acc_ff_xy_gain_ * px4_setpoint_.acc_xy;
    }
    if (use_acc_z_) {
        desired_state_.acc(2) += px4_setpoint_.acc_z;
    }
    const double acc_xy_norm = desired_state_.acc.head<2>().norm();
    if (acc_xy_norm > max_acc_xy_ && acc_xy_norm > 1e-6) {
        desired_state_.acc.head<2>() *= max_acc_xy_ / acc_xy_norm;
    }
    if (acc_xy_lpf_tau_ > 1e-6) {
        const double alpha = clampValue(dt_ / (acc_xy_lpf_tau_ + dt_), 0.0, 1.0);
        if (!filtered_acc_xy_initialized_) {
            filtered_acc_xy_ = desired_state_.acc.head<2>();
            filtered_acc_xy_initialized_ = true;
        } else {
            filtered_acc_xy_ += alpha * (desired_state_.acc.head<2>() - filtered_acc_xy_);
        }
        desired_state_.acc.head<2>() = filtered_acc_xy_;
    } else {
        filtered_acc_xy_initialized_ = false;
    }
    desired_state_.acc(2) = clampValue(desired_state_.acc(2), -max_acc_z_, max_acc_z_);
}

void Px4ControlSim::accelerationToAttitude()
{
    // 这里采用简化做法：
    // 先根据期望加速度求总推力方向，再把它近似映射成 roll/pitch。
    // 目的不是复刻 PX4 全部控制细节，而是把 MAVROS 指令转换成当前仿真可接受的中间量。
    const double commanded_acc_z = desired_state_.acc(2) + gravity_;
    double thrust_magnitude = sqrt(desired_state_.acc(0) * desired_state_.acc(0) + 
                                  desired_state_.acc(1) * desired_state_.acc(1) + 
                                  commanded_acc_z * commanded_acc_z) * mass_;
    
    if (thrust_magnitude > 0.0)
    {
        const double thrust_acc_magnitude = thrust_magnitude / mass_;
        const double yaw = use_yaw_ ? px4_setpoint_.att_yaw : current_state_.euler_angles(2);
        // World-frame lateral acceleration must be expressed in the body-aligned
        // horizontal plane of the desired yaw before solving roll/pitch.
        const double acc_forward = std::cos(yaw) * desired_state_.acc(0) + std::sin(yaw) * desired_state_.acc(1);
        const double acc_left = -std::sin(yaw) * desired_state_.acc(0) + std::cos(yaw) * desired_state_.acc(1);
        // For the body/world convention used by quadrotor_dynamics:
        // small pitch > 0 drives +X acceleration, and small roll < 0 drives +Y acceleration.
        desired_state_.att_roll_pitch(0) = -asin(clampUnit(acc_left / thrust_acc_magnitude));
        desired_state_.att_roll_pitch(1) = asin(clampUnit(acc_forward / thrust_acc_magnitude));
        desired_state_.att_roll_pitch(0) = clampValue(desired_state_.att_roll_pitch(0), -max_tilt_rad_, max_tilt_rad_);
        desired_state_.att_roll_pitch(1) = clampValue(desired_state_.att_roll_pitch(1), -max_tilt_rad_, max_tilt_rad_);
        desired_state_.att_yaw = yaw;
        desired_state_.thrust << 0.0, 0.0, thrust_magnitude;
    } else {
        ROS_WARN("thrust_magnitude wrong, set to 0.0");
        desired_state_.att_roll_pitch.setZero();
        desired_state_.att_yaw = 0.0;
        desired_state_.thrust.setZero();
    }
}

void Px4ControlSim::attitudeControl()
{
    Eigen::Vector3d desired_att;
    desired_att << desired_state_.att_roll_pitch(0), desired_state_.att_roll_pitch(1), desired_state_.att_yaw;
    
    Eigen::Vector3d att_error = desired_att - current_state_.euler_angles;
    // yaw 误差必须绕回 [-pi, pi]，否则跨越 -pi/pi 边界时会出现一个很大的假误差。
    att_error(2) = wrapAngle(att_error(2));

    desired_state_.bodyrate_roll_pitch(0) =
        clampValue(att_pid_kp_(0) * att_error(0), -max_roll_pitch_rate_, max_roll_pitch_rate_);
    desired_state_.bodyrate_roll_pitch(1) =
        clampValue(att_pid_kp_(1) * att_error(1), -max_roll_pitch_rate_, max_roll_pitch_rate_);
    
    double desired_yaw_rate = 0.0;
    if (use_yaw_) {
        desired_yaw_rate += att_pid_kp_(2) * att_error(2);
    }
    if (use_yaw_rate_) {
        desired_yaw_rate += px4_setpoint_.bodyrate_yaw_rate;
    }
    desired_state_.bodyrate_yaw_rate = clampValue(desired_yaw_rate, -max_yaw_rate_, max_yaw_rate_);
}

void Px4ControlSim::bodyrateControl()
{
    Eigen::Vector3d desired_bodyrate(desired_state_.bodyrate_roll_pitch(0), 
                                     desired_state_.bodyrate_roll_pitch(1), 
                                     desired_state_.bodyrate_yaw_rate);
    
    Eigen::Vector3d bodyrate_error = desired_bodyrate - current_state_.ang_vel;// 计算积分项
    bodyrate_pid_integral_ += bodyrate_error * dt_;
    bodyrate_pid_integral_ =
        bodyrate_pid_integral_.cwiseMax(-bodyrate_integral_limit_).cwiseMin(bodyrate_integral_limit_);
    
    desired_state_.torque = bodyrate_pid_kp_.cwiseProduct(bodyrate_error) + 
                           bodyrate_pid_ki_.cwiseProduct(bodyrate_pid_integral_);
}

Eigen::Vector3d Px4ControlSim::computeRealThrust(double collective_thrust)
{
    /**
     * 计算公式：
     * real_thrust = collective_thrust * 4 * max_rpm^2 * k_F
     * 
     * 其中：
     * - collective_thrust：归一化推力（0-1）
     * - 4：四旋翼电机数量
     * - max_rpm：最大电机转速（1200 rpm）
     * - k_F：推力系数（1.5e-5 N/(rpm^2)）
     * 
     * 当前实现采用“线性总推力百分比”约定：
     * 1. collective_thrust = 1 时，对应四个电机共同输出最大总推力
     * 2. collective_thrust = 0 时，对应总推力为 0
     * 3. 这是仿真接口假设，不代表 PX4 内部所有场景都严格按这个语义解释
     */
    
    double real_thrust = collective_thrust * 4.0 * motor_max_rpm_ * motor_max_rpm_ * motor_k_f_;
    
    // 构建推力向量（仅Z轴有推力）
    Eigen::Vector3d thrust_vector;
    thrust_vector << 0.0, 0.0, real_thrust;
    
    return thrust_vector;
}

Eigen::Vector4d Px4ControlSim::computeMotorThrust(double thrust, const Eigen::Vector3d& torque)
{
    // 这里默认混控矩阵与当前动力学模型采用同一套电机编号和符号定义。
    // 如果后续调整动力学中的电机顺序，这里必须同步修改。
    const double arm_projection = motor_arm_length_ / std::sqrt(2.0);
    const double yaw_force_gain = motor_k_t_ / motor_k_f_;
    const double max_motor_thrust = motor_k_f_ * motor_max_rpm_ * motor_max_rpm_;
    const double desired_total_thrust = clampValue(thrust, 0.0, 4.0 * max_motor_thrust);

    // 计算四个电机的期望推力
    Eigen::Vector4d motor_thrust;
    motor_thrust(0) = thrust / 4.0
                    - torque(0) / (4.0 * arm_projection)
                    - torque(1) / (4.0 * arm_projection)
                    - torque(2) / (4.0 * yaw_force_gain);
    motor_thrust(1) = thrust / 4.0
                    + torque(0) / (4.0 * arm_projection)
                    + torque(1) / (4.0 * arm_projection)
                    - torque(2) / (4.0 * yaw_force_gain);
    motor_thrust(2) = thrust / 4.0
                    + torque(0) / (4.0 * arm_projection)
                    - torque(1) / (4.0 * arm_projection)
                    + torque(2) / (4.0 * yaw_force_gain);
    motor_thrust(3) = thrust / 4.0
                    - torque(0) / (4.0 * arm_projection)
                    + torque(1) / (4.0 * arm_projection)
                    + torque(2) / (4.0 * yaw_force_gain);
    
    // 电机推力必须落在动力学可实现的范围内。
    motor_thrust = motor_thrust.cwiseMax(0.0).cwiseMin(max_motor_thrust);

    // 若因为负推力裁剪导致总推力被“凭空抬高”，按比例缩回期望总推力。
    // 否则在大姿态/偏航力矩下会出现“控制器要求下降，但动力学仍持续上冲”的假象。
    const double allocated_total_thrust = motor_thrust.sum();
    if (allocated_total_thrust > desired_total_thrust + 1e-6 && allocated_total_thrust > 1e-6)
    {
        motor_thrust *= desired_total_thrust / allocated_total_thrust;
    }
    
    return motor_thrust;
}

Eigen::Vector4d Px4ControlSim::computeMotorRPMFromThrust(const Eigen::Vector4d& motor_thrust)
{
    // 推力模型采用 T = k_F * rpm^2，因此反算 RPM 时需要开平方。
    Eigen::Vector4d motor_rpm;
    for (int i = 0; i < 4; i++)
    {
        motor_rpm(i) = sqrt(motor_thrust(i) / motor_k_f_);
    }
    
    // 限制电机RPM在合理范围内
    motor_rpm = motor_rpm.cwiseMax(0.0).cwiseMin(motor_max_rpm_);
    
    return motor_rpm;
}



void Px4ControlSim::publishMotorRPM(const Eigen::Vector4d& motor_rpm)
{
    // 发布电机RPM
    std_msgs::Float32MultiArray rpm_msg;
    rpm_msg.data.resize(4);
    for (int i = 0; i < 4; i++)
    {
        rpm_msg.data[i] = motor_rpm(i);
    }
    motor_rpm_pub_.publish(rpm_msg);
}

void Px4ControlSim::publishZeroMotorRPM()
{
    publishMotorRPM(Eigen::Vector4d::Zero());
}

void Px4ControlSim::printStatus() const
{
    std::cout << buildStatusPanel() << std::endl;
}

std::string Px4ControlSim::buildStatusPanel() const
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    const char* setpoint_color = has_setpoint_ ? kAnsiGood : kAnsiWarn;
    const std::string setpoint_status = has_setpoint_ ? "正常" : (setpoint_timed_out_ ? "超时" : "等待");
    const char* odom_color = has_odom_ ? kAnsiGood : kAnsiWarn;

    ss << kAnsiTitle << "=================== px4_control_sim_node [" << uav_name_
       << "] ===================" << kAnsiReset << "\n";

    ss << kAnsiGood << " 基本状态 " << kAnsiReset
       << "状态话题（订阅） -> " << uav_name_ << "/mavros/state"
       << "\n"
       << "          Name = " << uav_name_
       << "  控制周期 = " << dt_ << " s"
       << "  MAVROS状态 = " << (has_mavros_state_ ? kAnsiGood : kAnsiWarn) << stateText(has_mavros_state_) << kAnsiReset
       << "  解锁 = " << (mavros_armed_ ? "是" : "否")
       << "  PX4模式 = " << mavros_mode_
       << "\n";

    ss << kAnsiGood << " 本机状态 " << kAnsiReset
       << "里程计话题（订阅） -> " << uav_name_ << "/sunray_sim/odom"
       << "  ODOM状态 = " << odom_color << stateText(has_odom_) << kAnsiReset
       << "\n";

    switch (control_mode_)
    {
    case POSITION_CONTROL:
    case VELOCITY_CONTROL:
    {
        std::vector<std::string> enabled_inputs;
        if (use_pos_xy_) enabled_inputs.emplace_back("use_pos_xy");
        if (use_pos_z_) enabled_inputs.emplace_back("use_pos_z");
        if (use_vel_xy_) enabled_inputs.emplace_back("use_vel_xy");
        if (use_vel_z_) enabled_inputs.emplace_back("use_vel_z");
        if (use_acc_xy_) enabled_inputs.emplace_back("use_acc_xy");
        if (use_acc_z_) enabled_inputs.emplace_back("use_acc_z");
        if (use_yaw_) enabled_inputs.emplace_back("use_yaw");
        if (use_yaw_rate_) enabled_inputs.emplace_back("use_yaw_rate");

        ss << kAnsiGood << " 控制输入 " << kAnsiReset
           << "位置指令话题（订阅） -> " << uav_name_ << "/mavros/setpoint_raw/local"
           << "\n"
           << "          指令状态 = " << setpoint_color << setpoint_status << kAnsiReset
           << "  当前模式 = " << controlModeName() << " (" << joinNames(enabled_inputs) << ")"
           << "\n";
        if (use_pos_xy_)
        {
            ss << "           x = " << px4_setpoint_.pos_xy.x() << " m"
               << "  y = " << px4_setpoint_.pos_xy.y() << " m";
        }
        if (use_pos_z_)
        {
            ss << "  z = " << px4_setpoint_.pos_z << " m";
        }
        if (use_vel_xy_)
        {
            ss << "  vx = " << px4_setpoint_.vel_xy.x() << " m/s"
               << "  vy = " << px4_setpoint_.vel_xy.y() << " m/s";
        }
        if (use_vel_z_)
        {
            ss << "  vz = " << px4_setpoint_.vel_z << " m/s";
        }
        if (use_acc_xy_)
        {
            ss << "  ax = " << px4_setpoint_.acc_xy.x() << " m/s^2"
               << "  ay = " << px4_setpoint_.acc_xy.y() << " m/s^2";
        }
        if (use_acc_z_)
        {
            ss << "  az = " << px4_setpoint_.acc_z << " m/s^2";
        }
        if (use_yaw_)
        {
            ss << "  yaw = " << radToDeg(px4_setpoint_.att_yaw) << " deg";
        }
        if (use_yaw_rate_)
        {
            ss << "  yaw_rate = " << radToDeg(px4_setpoint_.bodyrate_yaw_rate) << " deg/s";
        }
        if (enabled_inputs.empty())
        {
            ss << "           无有效目标值";
        }
        ss << "\n";
        break;
    }
    case ATTITUDE_CONTROL:
    {
        std::vector<std::string> enabled_inputs{"use_attitude"};
        if (use_yaw_) enabled_inputs.emplace_back("use_yaw");
        if (use_yaw_rate_) enabled_inputs.emplace_back("use_yaw_rate");
        enabled_inputs.emplace_back("thrust");

        ss << kAnsiGood << " 控制输入 " << kAnsiReset
           << "姿态指令话题（订阅） -> " << uav_name_ << "/mavros/setpoint_raw/attitude"
           << "\n"
           << "          指令状态 = " << setpoint_color << setpoint_status << kAnsiReset
           << "  当前模式 = " << controlModeName() << " (" << joinNames(enabled_inputs) << ")"
           << "\n"
           << "           roll = " << radToDeg(px4_setpoint_.att_roll_pitch.x()) << " deg"
           << "  pitch = " << radToDeg(px4_setpoint_.att_roll_pitch.y()) << " deg";
        if (use_yaw_)
        {
            ss << "  yaw = " << radToDeg(px4_setpoint_.att_yaw) << " deg";
        }
        if (use_yaw_rate_)
        {
            ss << "  yaw_rate = " << radToDeg(px4_setpoint_.bodyrate_yaw_rate) << " deg/s";
        }
        ss << "  thrust = " << px4_setpoint_.collective_thrust * 100.0 << " %\n";
        break;
    }
    case BODYRATE_CONTROL:
        ss << kAnsiGood << " 控制输入 " << kAnsiReset
           << "姿态指令话题（订阅） -> " << uav_name_ << "/mavros/setpoint_raw/attitude"
           << "\n"
           << "          指令状态 = " << setpoint_color << setpoint_status << kAnsiReset
           << "  当前模式 = " << controlModeName() << " (use_bodyrate, thrust)"
           << "\n"
           << "           wx = " << radToDeg(px4_setpoint_.bodyrate_roll_pitch.x()) << " deg/s"
           << "  wy = " << radToDeg(px4_setpoint_.bodyrate_roll_pitch.y()) << " deg/s"
           << "  wz = " << radToDeg(px4_setpoint_.bodyrate_yaw_rate) << " deg/s"
           << "  thrust = " << px4_setpoint_.collective_thrust * 100.0 << " %"
           << "\n";
        break;
    default:
        ss << kAnsiGood << " 控制输入 " << kAnsiReset
           << "无有效输入话题\n"
           << "          指令状态 = " << setpoint_color << setpoint_status << kAnsiReset
           << "  当前模式 = " << controlModeName()
           << "\n";
        break;
    }

    if (!last_rejected_setpoint_reason_.empty())
    {
        ss << kAnsiWarn << " 最近拒绝 " << kAnsiReset
           << last_rejected_setpoint_reason_
           << "\n";
    }

    ss << kAnsiGood << " 控制输出 " << kAnsiReset
       << "电机话题（发布） -> " << uav_name_ << "/sunray_sim/cmd_RPM"
       << "\n"
       << "           期望姿态: roll = " << radToDeg(desired_state_.att_roll_pitch.x()) << " deg"
       << "  pitch = " << radToDeg(desired_state_.att_roll_pitch.y()) << " deg"
       << "  yaw = " << radToDeg(desired_state_.att_yaw) << " deg"
       << "\n";
    ss << "           总推力 = " << desired_state_.thrust.z() << " N"
       << "  电机输出 = (" << desired_state_.motor_rpm(0) << ", "
       << desired_state_.motor_rpm(1) << ", "
       << desired_state_.motor_rpm(2) << ", "
       << desired_state_.motor_rpm(3) << ") rpm";

    return ss.str();
}

const char* Px4ControlSim::controlModeName() const
{
    switch (control_mode_) {
    case POSITION_CONTROL:
        return "位置控制";
    case VELOCITY_CONTROL:
        return "速度控制";
    case ATTITUDE_CONTROL:
        return "姿态控制";
    case BODYRATE_CONTROL:
        return "角速度控制";
    default:
        return "UNKNOWN";
    }
}
}  // namespace sunray_sim
