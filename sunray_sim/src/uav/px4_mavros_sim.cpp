#include "px4_mavros_sim.h"

#include <std_msgs/Float32MultiArray.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace {
constexpr uint8_t kMavResultAccepted = 0;
constexpr int64_t kEkf2EvCtrlVisionPosYaw = (1 << 0) | (1 << 1) | (1 << 3);
constexpr int64_t kEkf2HgtRefVision = 3;
constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiTitle = "\033[1;35m";
constexpr const char* kAnsiGood = "\033[1;32m";
constexpr const char* kAnsiWarn = "\033[1;33m";

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

const char* readyText(bool ok)
{
    return ok ? "正常" : "等待";
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

Px4MavrosSim::Px4MavrosSim(ros::NodeHandle& nh, const std::string& uav_name)
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
    nh_.param("px4_mavros/mavros_publish_rate", mavros_publish_rate_hz_, 50.0);
    mavros_publish_rate_hz_ = std::max(1.0, mavros_publish_rate_hz_);
    nh_.param("px4_mavros/on_ground_height_threshold_m",
              on_ground_height_threshold_m_,
              0.05);
    nh_.param("px4_mavros/on_ground_velocity_threshold_mps",
              on_ground_velocity_threshold_mps_,
              0.10);
    nh_.param("px4_mavros/battery_voltage_v", battery_voltage_v_, 15.2f);
    nh_.param("px4_mavros/battery_current_a", battery_current_a_, 0.0f);
    nh_.param("px4_mavros/battery_remaining", battery_remaining_, 0.9f);
    int system_load_raw_param = static_cast<int>(system_load_raw_);
    nh_.param("px4_mavros/system_load_raw", system_load_raw_param, 150);
    system_load_raw_ = static_cast<uint16_t>(system_load_raw_param);
    nh_.param("px4_mavros/start_connected", connected_, true);
    nh_.param("px4_mavros/start_armed", armed_, true);
    nh_.param("px4_mavros/start_manual_input", manual_input_, true);
    nh_.param("px4_mavros/start_mode", current_mode_, std::string("OFFBOARD"));

    params_["EKF2_EV_CTRL"] = makeIntegerParamValue(kEkf2EvCtrlVisionPosYaw);
    params_["EKF2_HGT_REF"] = makeIntegerParamValue(kEkf2HgtRefVision);
    params_["EKF2_EV_DELAY"] = makeRealParamValue(0.0);

    // 初始化PID参数
    initPIDParams();
    
    attitude_target_sub_ = nh_.subscribe(
        uav_name_ + "/mavros/setpoint_raw/attitude", 10, &Px4MavrosSim::attitudeTargetCallback, this);
    position_target_sub_ = nh_.subscribe(
        uav_name_ + "/mavros/setpoint_raw/local", 10, &Px4MavrosSim::positionTargetCallback, this);
    odom_sub_ = nh_.subscribe(uav_name_ + "/sunray_sim/odom", 20, &Px4MavrosSim::odomCallback, this);
    imu_sub_ = nh_.subscribe(uav_name_ + "/sunray_sim/imu", 20, &Px4MavrosSim::imuCallback, this);
    navsat_sub_ = nh_.subscribe(uav_name_ + "/sunray_sim/navsat", 20, &Px4MavrosSim::navsatCallback, this);
    
    motor_rpm_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(uav_name_ + "/sunray_sim/cmd_RPM", 10);
    state_pub_ = nh_.advertise<mavros_msgs::State>(uav_name_ + "/mavros/state", 10);
    extended_state_pub_ =
        nh_.advertise<mavros_msgs::ExtendedState>(uav_name_ + "/mavros/extended_state", 10);
    sys_status_pub_ = nh_.advertise<mavros_msgs::SysStatus>(uav_name_ + "/mavros/sys_status", 10);
    estimator_status_pub_ =
        nh_.advertise<mavros_msgs::EstimatorStatus>(uav_name_ + "/mavros/estimator_status", 10);
    local_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(uav_name_ + "/mavros/local_position/odom", 10);
    local_pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(uav_name_ + "/mavros/local_position/pose", 10);
    local_velocity_pub_ =
        nh_.advertise<geometry_msgs::TwistStamped>(uav_name_ + "/mavros/local_position/velocity_local", 10);
    global_position_pub_ =
        nh_.advertise<sensor_msgs::NavSatFix>(uav_name_ + "/mavros/global_position/global", 10);
    gps_raw_pub_ =
        nh_.advertise<mavros_msgs::GPSRAW>(uav_name_ + "/mavros/gpsstatus/gps1/raw", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(uav_name_ + "/mavros/imu/data", 10);
    target_local_pub_ =
        nh_.advertise<mavros_msgs::PositionTarget>(uav_name_ + "/mavros/setpoint_raw/target_local", 10);
    target_attitude_pub_ =
        nh_.advertise<mavros_msgs::AttitudeTarget>(uav_name_ + "/mavros/setpoint_raw/target_attitude", 10);

    set_mode_srv_ =
        nh_.advertiseService(uav_name_ + "/mavros/set_mode", &Px4MavrosSim::setModeService, this);
    arming_srv_ =
        nh_.advertiseService(uav_name_ + "/mavros/cmd/arming", &Px4MavrosSim::armingService, this);
    command_long_srv_ =
        nh_.advertiseService(uav_name_ + "/mavros/cmd/command", &Px4MavrosSim::commandLongService, this);
    param_get_srv_ =
        nh_.advertiseService(uav_name_ + "/mavros/param/get", &Px4MavrosSim::paramGetService, this);
    param_set_srv_ =
        nh_.advertiseService(uav_name_ + "/mavros/param/set", &Px4MavrosSim::paramSetService, this);

    update_timer_ = nh_.createTimer(ros::Duration(dt_),
                                    &Px4MavrosSim::updateTimerCallback,
                                    this);
    mavros_publish_timer_ = nh_.createTimer(ros::Duration(1.0 / mavros_publish_rate_hz_),
                                            &Px4MavrosSim::mavrosPublishTimerCallback,
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
    
    ROS_INFO("[px4_mavros_sim] started for %s control=%.1f Hz mavros=%.1f Hz mode=%s armed=%s",
             uav_name_.c_str(),
             px4_update_rate,
             mavros_publish_rate_hz_,
             current_mode_.c_str(),
             armed_ ? "true" : "false");
    ROS_INFO("[px4_mavros_sim] params: mass=%.3f gravity=%.3f k_F=%.6e arm=%.3f rpm_max=%.1f",
             mass_,
             gravity_,
             motor_k_f_,
             motor_arm_length_,
             motor_max_rpm_);
}

void Px4MavrosSim::updateTimerCallback(const ros::TimerEvent&)
{
    update();
}

void Px4MavrosSim::initPIDParams()
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

bool Px4MavrosSim::isSupportedPositionTarget(const mavros_msgs::PositionTarget& msg,
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

bool Px4MavrosSim::isSupportedAttitudeTarget(const mavros_msgs::AttitudeTarget& msg,
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

void Px4MavrosSim::warnUnsupportedPositionTargetOnce(const mavros_msgs::PositionTarget& msg,
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
        ROS_WARN("[Px4MavrosSim] unsupported PositionTarget frame=%u type_mask=%u: %s",
                 msg.coordinate_frame,
                 msg.type_mask,
                 reason.c_str());
    }
}

void Px4MavrosSim::warnUnsupportedAttitudeTargetOnce(const mavros_msgs::AttitudeTarget& msg,
                                                        const std::string& reason)
{
    last_rejected_setpoint_reason_ = "AttitudeTarget mask=" +
                                     std::to_string(static_cast<int>(msg.type_mask)) +
                                     "，原因：" + reason;
    if (warned_attitude_target_masks_.insert(msg.type_mask).second)
    {
        ROS_WARN("[Px4MavrosSim] unsupported AttitudeTarget type_mask=%u: %s",
                 msg.type_mask,
                 reason.c_str());
    }
}

void Px4MavrosSim::markSupportedSetpoint()
{
    has_setpoint_ = true;
    setpoint_timed_out_ = false;
    last_supported_setpoint_time_ = ros::Time::now();
}

void Px4MavrosSim::clearControllerOutput()
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

void Px4MavrosSim::attitudeTargetCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    mavros_msgs::AttitudeTarget att_target = *msg;
    latest_attitude_target_ = att_target;
    latest_attitude_target_.header.stamp = ros::Time::now();
    has_attitude_target_ = true;
    target_attitude_pub_.publish(latest_attitude_target_);

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

void Px4MavrosSim::positionTargetCallback(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    mavros_msgs::PositionTarget pos_target = *msg;
    latest_local_target_ = pos_target;
    latest_local_target_.header.stamp = ros::Time::now();
    has_local_target_ = true;
    target_local_pub_.publish(latest_local_target_);

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

void Px4MavrosSim::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    latest_odom_ = *msg;
    has_odom_ = true;

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
    publishLocalPositionOutputs(ros::Time::now());
}

void Px4MavrosSim::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    latest_imu_ = *msg;
    has_imu_ = true;
    imu_pub_.publish(latest_imu_);
}

void Px4MavrosSim::navsatCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    latest_navsat_ = *msg;
    has_navsat_ = true;
    publishGlobalPositionOutputs(ros::Time::now());
}

void Px4MavrosSim::update()
{
    if (!armed_)
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

void Px4MavrosSim::mavrosPublishTimerCallback(const ros::TimerEvent&)
{
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
    estimator_status_pub_.publish(estimator_msg);

    if (has_odom_)
    {
        publishLocalPositionOutputs(now);
    }
    if (has_navsat_)
    {
        publishGlobalPositionOutputs(now);
    }
    if (has_imu_)
    {
        latest_imu_.header.stamp = now;
        imu_pub_.publish(latest_imu_);
    }
    if (has_local_target_)
    {
        latest_local_target_.header.stamp = now;
        target_local_pub_.publish(latest_local_target_);
    }
    if (has_attitude_target_)
    {
        latest_attitude_target_.header.stamp = now;
        target_attitude_pub_.publish(latest_attitude_target_);
    }
}

bool Px4MavrosSim::setModeService(mavros_msgs::SetMode::Request& req,
                                  mavros_msgs::SetMode::Response& res)
{
    if (!req.custom_mode.empty())
    {
        const bool numeric_mode = std::all_of(req.custom_mode.begin(),
                                              req.custom_mode.end(),
                                              [](unsigned char ch) { return std::isdigit(ch); });
        current_mode_ = numeric_mode ? ("CMODE(" + req.custom_mode + ")") : req.custom_mode;
    }
    connected_ = true;
    res.mode_sent = true;
    ROS_INFO("[px4_mavros_sim] set_mode request=%s -> mode=%s",
             req.custom_mode.c_str(),
             current_mode_.c_str());
    return true;
}

bool Px4MavrosSim::armingService(mavros_msgs::CommandBool::Request& req,
                                 mavros_msgs::CommandBool::Response& res)
{
    armed_ = req.value;
    connected_ = true;
    res.success = true;
    res.result = kMavResultAccepted;
    if (!armed_)
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
    ROS_INFO("[px4_mavros_sim] arming -> %s", armed_ ? "true" : "false");
    return true;
}

bool Px4MavrosSim::commandLongService(mavros_msgs::CommandLong::Request& req,
                                      mavros_msgs::CommandLong::Response& res)
{
    if (req.command == 400 && req.param1 == 0.0)
    {
        armed_ = false;
        has_setpoint_ = false;
        clearControllerOutput();
        publishZeroMotorRPM();
    }
    res.success = true;
    res.result = kMavResultAccepted;
    ROS_WARN("[px4_mavros_sim] command_long accepted: command=%u", req.command);
    return true;
}

bool Px4MavrosSim::paramGetService(mavros_msgs::ParamGet::Request& req,
                                   mavros_msgs::ParamGet::Response& res)
{
    const auto it = params_.find(req.param_id);
    if (it == params_.end())
    {
        res.success = false;
        res.value = makeIntegerParamValue(0);
        ROS_WARN("[px4_mavros_sim] param_get miss: %s", req.param_id.c_str());
        return true;
    }
    res.success = true;
    res.value = it->second;
    return true;
}

bool Px4MavrosSim::paramSetService(mavros_msgs::ParamSet::Request& req,
                                   mavros_msgs::ParamSet::Response& res)
{
    params_[req.param_id] = req.value;
    res.success = true;
    res.value = req.value;
    return true;
}

void Px4MavrosSim::positionControl()
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

void Px4MavrosSim::velocityControl()
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

void Px4MavrosSim::accelerationToAttitude()
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

void Px4MavrosSim::attitudeControl()
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

void Px4MavrosSim::bodyrateControl()
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

Eigen::Vector3d Px4MavrosSim::computeRealThrust(double collective_thrust)
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

Eigen::Vector4d Px4MavrosSim::computeMotorThrust(double thrust, const Eigen::Vector3d& torque)
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

Eigen::Vector4d Px4MavrosSim::computeMotorRPMFromThrust(const Eigen::Vector4d& motor_thrust)
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



void Px4MavrosSim::publishMotorRPM(const Eigen::Vector4d& motor_rpm)
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

void Px4MavrosSim::publishZeroMotorRPM()
{
    publishMotorRPM(Eigen::Vector4d::Zero());
}

void Px4MavrosSim::publishLocalPositionOutputs(const ros::Time& stamp)
{
    nav_msgs::Odometry odom_msg = latest_odom_;
    odom_msg.header.stamp = stamp;
    local_odom_pub_.publish(odom_msg);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = odom_msg.header;
    pose_msg.pose = odom_msg.pose.pose;
    local_pose_pub_.publish(pose_msg);

    geometry_msgs::TwistStamped velocity_msg;
    velocity_msg.header = odom_msg.header;
    velocity_msg.twist = odom_msg.twist.twist;
    local_velocity_pub_.publish(velocity_msg);
}

void Px4MavrosSim::publishGlobalPositionOutputs(const ros::Time& stamp)
{
    sensor_msgs::NavSatFix global_msg = latest_navsat_;
    global_msg.header.stamp = stamp;
    global_position_pub_.publish(global_msg);

    mavros_msgs::GPSRAW gps_msg;
    gps_msg.header = global_msg.header;
    gps_msg.fix_type = mavros_msgs::GPSRAW::GPS_FIX_TYPE_3D_FIX;
    gps_msg.lat = static_cast<int32_t>(std::llround(global_msg.latitude * 1.0e7));
    gps_msg.lon = static_cast<int32_t>(std::llround(global_msg.longitude * 1.0e7));
    gps_msg.alt = static_cast<int32_t>(std::llround(global_msg.altitude * 1000.0));
    gps_msg.eph = 100;
    gps_msg.epv = 150;
    gps_msg.vel = 0;
    if (has_odom_)
    {
        const double vx = latest_odom_.twist.twist.linear.x;
        const double vy = latest_odom_.twist.twist.linear.y;
        const double vz = latest_odom_.twist.twist.linear.z;
        gps_msg.vel = static_cast<uint16_t>(std::llround(std::hypot(std::hypot(vx, vy), vz) * 100.0));
    }
    gps_msg.satellites_visible = 12;
    gps_msg.alt_ellipsoid = gps_msg.alt;
    gps_msg.h_acc = 1000;
    gps_msg.v_acc = 1500;
    gps_msg.vel_acc = 100;
    gps_raw_pub_.publish(gps_msg);
}

uint8_t Px4MavrosSim::estimateLandedState() const
{
    if (!has_odom_)
    {
        return mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED;
    }
    const double z = latest_odom_.pose.pose.position.z;
    const double vz = latest_odom_.twist.twist.linear.z;
    if (!armed_ && z <= std::max(on_ground_height_threshold_m_, 0.10))
    {
        return mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    }
    if (z <= on_ground_height_threshold_m_ &&
        std::fabs(vz) <= on_ground_velocity_threshold_mps_)
    {
        return mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    }
    return mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
}

mavros_msgs::ParamValue Px4MavrosSim::makeIntegerParamValue(const int64_t value)
{
    mavros_msgs::ParamValue param;
    param.integer = value;
    param.real = static_cast<double>(value);
    return param;
}

mavros_msgs::ParamValue Px4MavrosSim::makeRealParamValue(const double value)
{
    mavros_msgs::ParamValue param;
    param.integer = static_cast<int64_t>(value);
    param.real = value;
    return param;
}

void Px4MavrosSim::printStatus() const
{
    std::cout << buildStatusPanel() << std::endl;
}

std::string Px4MavrosSim::buildStatusPanel() const
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    const char* setpoint_color = has_setpoint_ ? kAnsiGood : kAnsiWarn;
    const std::string setpoint_status = has_setpoint_ ? "正常" : (setpoint_timed_out_ ? "超时" : "等待");
    const char* odom_color = has_odom_ ? kAnsiGood : kAnsiWarn;

    ss << kAnsiTitle << "=================== px4_mavros_sim [" << uav_name_
       << "] ===================" << kAnsiReset << "\n";

    ss << kAnsiGood << " 基本状态 " << kAnsiReset
       << "connected = " << (connected_ ? kAnsiGood : kAnsiWarn) << (connected_ ? "true" : "false") << kAnsiReset
       << "  armed = " << (armed_ ? "true" : "false")
       << "  mode = " << current_mode_
       << "  控制周期 = " << dt_ << " s"
       << "  MAVROS发布 = " << mavros_publish_rate_hz_ << " Hz"
       << "\n";

    ss << kAnsiGood << " 本机状态 " << kAnsiReset
       << "odom -> " << uav_name_ << "/sunray_sim/odom"
       << "  " << odom_color << readyText(has_odom_) << kAnsiReset
       << "  imu = " << (has_imu_ ? kAnsiGood : kAnsiWarn) << readyText(has_imu_) << kAnsiReset
       << "  navsat = " << (has_navsat_ ? kAnsiGood : kAnsiWarn) << readyText(has_navsat_) << kAnsiReset
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
       << desired_state_.motor_rpm(3) << ") rpm"
       << "\n";

    ss << kAnsiGood << " MAVROS输出 " << kAnsiReset
       << "state, extended_state, sys_status, estimator_status"
       << "\n"
       << "           local_position/{odom,pose,velocity_local}, global_position/global, gpsstatus/gps1/raw, imu/data";

    return ss.str();
}

const char* Px4MavrosSim::controlModeName() const
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
