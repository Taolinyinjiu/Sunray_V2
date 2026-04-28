#include "controller/px4_origin_controller.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "control_data_types/mavros_helper_data_types.hpp"
#include "eigen_helper.hpp"
#include "string_uav_namespace_utils.hpp"
#include "utils/orientation_utils.hpp"
#include "utils/uav_param_utils.hpp"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>  // 引入Yaml-cpp库，用于读取yaml文件
#include <sunray_msgs/UAVControllerState.h>
#include <px4_param_manager/px4_param_types.h>
#include <px4_param_manager/px4_param_decode.h>

namespace {

controller_data_types::TargetTrajectoryPoint_t
desired_state_from_local_setpoint(const control_common::Mavros_SetpointLocal& setpoint,
                                  const control_common::UAVStateEstimate& current_odom) {
    controller_data_types::TargetTrajectoryPoint_t desired;
    desired.position = current_odom.position;
    desired.velocity = Eigen::Vector3d::Zero();
    desired.acceleration = Eigen::Vector3d::Zero();
    desired.jerk = Eigen::Vector3d::Zero();

    if ((setpoint.mask & control_common::Mavros_SetpointLocal::Mask::IgnorePx) == 0) {
        desired.position.x() = setpoint.position.x();
    }
    if ((setpoint.mask & control_common::Mavros_SetpointLocal::Mask::IgnorePy) == 0) {
        desired.position.y() = setpoint.position.y();
    }
    if ((setpoint.mask & control_common::Mavros_SetpointLocal::Mask::IgnorePz) == 0) {
        desired.position.z() = setpoint.position.z();
    }

    if ((setpoint.mask & control_common::Mavros_SetpointLocal::Mask::IgnoreVx) == 0) {
        desired.velocity.x() = setpoint.velocity.x();
    }
    if ((setpoint.mask & control_common::Mavros_SetpointLocal::Mask::IgnoreVy) == 0) {
        desired.velocity.y() = setpoint.velocity.y();
    }
    if ((setpoint.mask & control_common::Mavros_SetpointLocal::Mask::IgnoreVz) == 0) {
        desired.velocity.z() = setpoint.velocity.z();
    }

    if ((setpoint.mask & control_common::Mavros_SetpointLocal::Mask::IgnoreAfx) == 0) {
        desired.acceleration.x() = setpoint.accel_or_force.x();
    }
    if ((setpoint.mask & control_common::Mavros_SetpointLocal::Mask::IgnoreAfy) == 0) {
        desired.acceleration.y() = setpoint.accel_or_force.y();
    }
    if ((setpoint.mask & control_common::Mavros_SetpointLocal::Mask::IgnoreAfz) == 0) {
        desired.acceleration.z() = setpoint.accel_or_force.z();
    }

    desired.yaw = ((setpoint.mask & control_common::Mavros_SetpointLocal::Mask::IgnoreYaw) == 0)
                      ? setpoint.yaw
                      : eigen_helper::get_yaw_from_orientation(current_odom.orientation);
    desired.yaw_rate =
        ((setpoint.mask & control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate) == 0)
            ? setpoint.yaw_rate
            : 0.0;
    return desired;
}

Eigen::Vector3d quaternion_error_to_vector(const Eigen::Quaterniond& current,
                                           const Eigen::Quaterniond& reference) {
    const Eigen::Quaterniond q_err = current.inverse() * reference;
    return std::copysign(1.0, q_err.w()) * Eigen::Vector3d(q_err.x(), q_err.y(), q_err.z());
}

uint8_t controller_reference_frame(const control_common::Mavros_SetpointLocal& setpoint) {
    switch (setpoint.frame) {
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Ned:
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Offset_Ned:
        return sunray_msgs::UAVControllerState::FRAME_BODY;
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned:
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Offset_Ned:
    default:
        return sunray_msgs::UAVControllerState::FRAME_LOCAL;
    }
}

}  // namespace

// 构造函数，读取参数
PX4_OriginController::PX4_OriginController(ros::NodeHandle& nh)
    : nh_(nh), mavros_helper_(nh_), mavros_param_(nh_) {
    // 读取节点名
    std::string node_name = ros::this_node::getName();
    // 构造私有节点句柄，用于读取节点私有参数,controller主要读取yaml路径
    ros::NodeHandle private_nh_("~");
    if (private_nh_.getParam("config_yamlfile_path", config_yamlfile_path_)) {
        if (config_yamlfile_path_.empty()) {  // 路径为空，抛出异常
            throw std::runtime_error("yaml_path connot be empty");
        }
    } else {  // 读取失败，抛出异常
        throw std::runtime_error("missing param" + node_name + "/config_yamlfile_path");
    }
    // 优先读取节点私有参数中的 uav_name/uav_id，单机场景再回退到全局参数
    uav_ns_ = sunray_control::load_uav_namespace_or_throw(nh_);
}

bool PX4_OriginController::init() {
    // 加载参数并校验参数
    load_and_validate_config_or_throw();
    // 配置读取完毕，初始化mavros_helper
    MavrosHelper_ConfigList config_list(true);
    if (!mavros_helper_.init(config_list)) {
        throw std::runtime_error("mavros_helper init failed");
    }
    if (config_param_.fuse_odom_type != 0) {  // 融合外部里程计到px4，设置对应参数,注册定时器
        // 检查外部里程计融合相关参数
        ensure_fusion_param_ready_or_throw();  // 该函数检查失败会抛出异常
        mavros_helper_.set_vision_fuse_type(config_param_.fuse_odom_type);
        // 创建定时器
        pub_vision_pose_timer_ =
            nh_.createTimer(ros::Duration(1.0 / config_param_.fuse_odom_frequency),
                            &PX4_OriginController::pub_vision_fuse_timer_cb,
                            this);
    }
    // 初始化话题发布者
    controller_state_pub_ =
        nh_.advertise<sunray_msgs::UAVControllerState>(uav_ns_ + "/sunray/controller_state", 10);
    // 初始化发布px4_state数据的定时器
    pub_px4_state_timer = nh_.createTimer(ros::Duration(1.0 / pub_px4_state_freq_),
                                          &PX4_OriginController::pub_px4_state_timer_cb,
                                          this);
    return true;
}

// is_ready的返回结果作为Sunray_FSM : OFF -> INIT 的判断状态
// 我们需要讨论，什么是控制器所关心的？
// 1. 里程计是否超时
// 2. 各项连接是否正确
// 3. 飞控正常工作
bool PX4_OriginController::is_ready() {
    control_common::Mavros_State mavros_state = mavros_helper_.get_state();
    if (mavros_state.connected != true) {
        // px4未正常连接，判断为超时
        return false;
    }
    // 判断mavros_helper提取的各项数据，是否满足要求
    bool mavros_ready = mavros_helper_.is_ready();
    if (!mavros_ready) {
        return false;
    }

    controller_ready_.store(true, std::memory_order_relaxed);
    return true;
}

void PX4_OriginController::set_current_odom(const control_common::UAVStateEstimate& odom) {
    uav_odometry_ = odom;
    const bool has_valid_timestamp = !odom.timestamp.isZero();
    const bool is_new_odom = has_valid_timestamp && odom.timestamp != last_odom_timestamp_;
    has_uav_odometry_.store(has_valid_timestamp, std::memory_order_relaxed);
    if (is_new_odom) {
        last_odom_timestamp_ = odom.timestamp;
        can_fuse_.store(true, std::memory_order_relaxed);
    }
}

void PX4_OriginController::cache_local_setpoint(
    const control_common::Mavros_SetpointLocal& setpoint) {
    last_setpoint_ = setpoint;
    last_setpoint_.timestamp = ros::Time::now();
    last_setpoint_.valid = true;
    desired_state_ = desired_state_from_local_setpoint(setpoint, uav_odometry_);
}

// -------------运动相关接口------------

// 任务流程描述为 takoff ： [mode]position -> [mode]offboard setpoint 流持续发送
//              land  :  [mode]offboard -> [mode]position setpoint 流停止发送
// 因此我们提供一个set_postion_mode函数用于设置为position模式
void PX4_OriginController::set_position_mode() {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    control_common::Mavros_State state = mavros_helper_.get_state();
    if (state.flight_mode != control_common::FlightMode::Posctl) {
        mavros_helper_.set_px4_mode(control_common::FlightMode::Posctl);
    }
}

void PX4_OriginController::reset_point_motion_context() {
    move_point_curve_.clear();
    point_arrival_state_ = arrival_helper::State{};
    point_complete_.store(false, std::memory_order_relaxed);
    point_target_initialized_ = false;
    body_point_target_initialized_ = false;
    last_point_ = controller_data_types::TargetPoint_t{};
    last_point_body_ = controller_data_types::TargetBodyPoint_t{};
}

double PX4_OriginController::update_limited_yaw_target(double target_yaw, const ros::Time& now) {
    return reference_limit_helper::update_slewed_yaw_target(
        yaw_reference_state_, target_yaw, uav_odometry_.get_yaw(), max_yaw_rate_rad_s_, now);
}

void PX4_OriginController::warn_if_trajectory_exceeds_limits(
    const controller_data_types::TargetTrajectoryPoint_t& trajpoint) const {
    if (!reference_limit_helper::trajectory_reference_exceeds_limits(
            trajpoint.velocity, trajpoint.yaw_rate, max_velocity_, max_yaw_rate_rad_s_)) {
        return;
    }

    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[PX4_OriginController][" << uav_ns_
                                   << "] trajectory reference exceeds velocity_param limits: vel="
                                   << trajpoint.velocity.transpose() << " max="
                                   << max_velocity_.transpose() << " yaw_rate="
                                   << trajpoint.yaw_rate << " max_yaw_rate="
                                   << max_yaw_rate_rad_s_);
}

bool PX4_OriginController::move_point_impl(controller_data_types::TargetPoint_t point,
                                           bool preserve_body_point_context) {
    constexpr double kNewTargetPosEps = 1e-3;
    if (!preserve_body_point_context) {
        body_point_target_initialized_ = false;
    }

    const bool is_new_target = !point_target_initialized_ ||
                               (point.position - last_point_.position).norm() > kNewTargetPosEps;
    if (is_new_target) {
        point_arrival_state_ = arrival_helper::State{};
        point_complete_.store(false, std::memory_order_relaxed);
        last_point_ = point;
        point_target_initialized_ = true;
        const Eigen::Vector3d start_position =
            last_setpoint_.valid ? last_setpoint_.position : uav_odometry_.position;
        const Eigen::Vector3d start_velocity =
            last_setpoint_.valid ? last_setpoint_.velocity : uav_odometry_.velocity;
        move_point_curve_.clear();
        move_point_curve_.set_start_trajpoint(start_position, start_velocity);
        move_point_curve_.set_end_trajpoint(point.position, Eigen::Vector3d::Zero());
        move_point_curve_.set_curve_maxvel(reference_limit_helper::compute_point_curve_maxvel(
            start_position, point.position, max_velocity_));
    }

    control_common::Mavros_SetpointLocal send_setpoint;
    send_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    send_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreVx |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreVy |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreVz |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreAfz |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
    const curve::QuinticCurveState curve_result = move_point_curve_.get_result();
    // Keep point mode close to historical "position hold" semantics:
    // smooth only the position reference, and avoid quintic velocity/acceleration
    // feedforward that can make move_point much more aggressive. Mask them out as
    // well so PX4 consumes this as a position+yaw reference instead of a mixed setpoint.
    send_setpoint.position = curve_result.valid ? curve_result.position : point.position;
    send_setpoint.velocity = Eigen::Vector3d::Zero();
    send_setpoint.accel_or_force = Eigen::Vector3d::Zero();
    send_setpoint.yaw = update_limited_yaw_target(point.yaw, ros::Time::now());
    send_setpoint.yaw_rate = 0.0;
    mavros_helper_.pub_local_setpoint(send_setpoint);
    cache_local_setpoint(send_setpoint);

    if (point_complete_.load(std::memory_order_relaxed)) {
        return true;
    }

    const ros::Time now = ros::Time::now();
    const double pos_err = (uav_odometry_.position - last_point_.position).norm();
    const double vel_err = uav_odometry_.velocity.norm();
    if (!arrival_helper::update_and_check(
            point_arrival_state_, arrival_judge_config_, pos_err, vel_err, now)) {
        point_complete_.store(false, std::memory_order_relaxed);
        return false;
    }

    point_complete_.store(true, std::memory_order_relaxed);
    hover_point_ = last_point_.position;
    hover_yaw_ = last_point_.yaw;
    return true;
}

bool PX4_OriginController::takeoff(double relative_takeoff_height, double max_takeoff_velocity) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    // 如何设计呢？起始这里的问题是，我们如何触发？
    // 实现思路为这样，我们不断的触发这个函数直到达到预设的起飞高度，也就是这样
    // ------sunray_fsm--------
    // while(controller_update_freq){
    //   switch(control_cmd){
    //       controller.takeoff();//状态机根据控制器的频率持续调用
    //   }
    // }
    // 首先如果控制器状态未就绪，拒绝起飞，返回false
    // 本函数在确定到达起飞位置后，返回true，其他情况均返回false，控制器依据返回true切换为hover状态
    if (land_complete_.load(std::memory_order_relaxed)) {
        land_complete_.store(false, std::memory_order_relaxed);
    }
    if (!controller_ready_.load(std::memory_order_relaxed)) {
        return false;
    }
    if (takeoff_complete_.load(std::memory_order_relaxed)) {
        control_common::Mavros_SetpointLocal current_setpoint;
        current_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
        current_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreVx |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreVy |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreVz |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreAfz |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;

        control_common::Mavros_Pose current_odom = mavros_helper_.get_local_pose();
        current_setpoint.position = current_odom.position;
        current_setpoint.yaw = mavros_helper_.get_yaw_rad();
        mavros_helper_.pub_local_setpoint(current_setpoint);
        cache_local_setpoint(current_setpoint);
        return true;
    }
    ros::Time now = ros::Time::now();
    // 通过五次项曲线类，确定是否为第一次进入takeoff函数
    if (!quint_curve_.is_ready()) {
        takeoff_arrival_state_ = arrival_helper::State{};
        // 首次进入五次项曲线，注入参数
        // 起点参数为当前里程计值，速度为0
        quint_curve_.set_start_trajpoint(uav_odometry_.position, Eigen::Vector3d::Zero());
        // 终点参数为当前里程计+z轴相对期望高度，速度为0
        quint_curve_.set_end_trajpoint(uav_odometry_.position +
                                           Eigen::Vector3d(0, 0, relative_takeoff_height),
                                       Eigen::Vector3d::Zero());
        // 根据起飞过程最大速度反推时间
        quint_curve_.set_curve_maxvel(max_takeoff_velocity);
        // quint_curve并不显式的设置开始运动的时间，我们以第一次调用get_result的时刻为开始运动的时间
        // 我们需要考虑无人机是否需要一点时间来进行加速，也就是说从电机桨叶不转动到转动的过程，是先不计算数据的
        // 顺便记录一下初始时刻的yaw角和地面高度
        takeoff_yaw_ = mavros_helper_.get_yaw_rad();
        takeoff_ground_height = uav_odometry_.position.z();
    }
    control_common::Mavros_State px4_state = mavros_helper_.get_state();

    if (px4_state.flight_mode != control_common::FlightMode::Offboard) {
        takeoff_arrival_state_ = arrival_helper::State{};
        last_arm_time_ = ros::Time(0);
        // 默认的模式应该是position模式
        // 首先我们需要切换为offboard模式，切换模式需要发送至少2Hz的控制指令，因此我们先设置控制指令为零速度指令
        control_common::Mavros_SetpointLocal setpoint_cmd;
        setpoint_cmd.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
        setpoint_cmd.velocity = Eigen::Vector3d::Zero();
        setpoint_cmd.yaw = takeoff_yaw_;
        mavros_helper_.pub_local_setpoint(setpoint_cmd);
        cache_local_setpoint(setpoint_cmd);

        if (start_checkout_offboard_time_ == ros::Time(0)) {
            start_checkout_offboard_time_ = now;
            last_checkout_offboard_time_ = ros::Time(0);
        }
        if (last_checkout_offboard_time_ == ros::Time(0) ||
            (now - last_checkout_offboard_time_).toSec() >= 0.3) {
            mavros_helper_.set_px4_mode(control_common::FlightMode::Offboard);
            last_checkout_offboard_time_ = now;
        }
        if ((now - start_checkout_offboard_time_).toSec() > 3) {
            // TODO: 如何处理超过3s也无法切换为offboard的情况
        }
    }

    if (px4_state.flight_mode == control_common::FlightMode::Offboard) {
        // 清除切换offboard的上下文
        start_checkout_offboard_time_ = ros::Time(0);
        last_checkout_offboard_time_ = ros::Time(0);
        // 如果飞控没有解锁，则尝试解锁
        if (px4_state.armed == false) {
            takeoff_arrival_state_ = arrival_helper::State{};
            last_arm_time_ = ros::Time(0);
            mavros_helper_.set_arm(true);
        } else {
            // 注意到起飞阶段由于无人机在地面，与在空中的动力学分析不一致，起飞阶段setpoint具有严重的滞后
            // 这里尝试一种起飞方式，先通过速度控制，实现离地，然后切换为轨迹控制
            if (last_arm_time_ == ros::Time(0)) {
                control_common::Mavros_SetpointLocal setpoint_cmd;
                setpoint_cmd.mask = control_common::Mavros_SetpointLocal::Mask::IgnorePz |
                                    control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                                    control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                                    control_common::Mavros_SetpointLocal::Mask::IgnoreAfz;
                Eigen::Vector3d start_pos = quint_curve_.get_start_position();
                setpoint_cmd.position = start_pos;
                setpoint_cmd.velocity = Eigen::Vector3d(0, 0, 0.3);
                setpoint_cmd.yaw = takeoff_yaw_;
                mavros_helper_.pub_local_setpoint(setpoint_cmd);
                cache_local_setpoint(setpoint_cmd);

                if (uav_odometry_.position.z() - takeoff_ground_height > 0.05) {
                    last_arm_time_ = now;
                    Eigen::Vector3d renew_pos = start_pos;
                    renew_pos.z() = uav_odometry_.position.z();
                    Eigen::Vector3d renew_vel = uav_odometry_.velocity;
                    quint_curve_.set_start_trajpoint(renew_pos,
                                                     Eigen::Vector3d(0, 0, renew_vel.z()));
                } else {
                    return false;
                }
            }

            curve::QuinticCurveState curve_result = quint_curve_.get_result();
            control_common::Mavros_SetpointLocal setpoint_cmd;
            setpoint_cmd.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
            setpoint_cmd.position = curve_result.position;
            setpoint_cmd.velocity = curve_result.velocity;
            setpoint_cmd.accel_or_force = curve_result.acceleration;
            setpoint_cmd.yaw = takeoff_yaw_;
            mavros_helper_.pub_local_setpoint(setpoint_cmd);
            cache_local_setpoint(setpoint_cmd);

            const double pos_err = (uav_odometry_.position - quint_curve_.get_end_position()).norm();
            const double vel_err = uav_odometry_.velocity.norm();
            if (!arrival_helper::update_and_check(
                    takeoff_arrival_state_, takeoff_arrival_config_, pos_err, vel_err, now)) {
                return false;
            }

            takeoff_complete_.store(true, std::memory_order_relaxed);
            hover_point_ = quint_curve_.get_end_position();
            hover_yaw_ = takeoff_yaw_;
            start_checkout_offboard_time_ = ros::Time(0);
            last_checkout_offboard_time_ = ros::Time(0);
            last_arm_time_ = ros::Time(0);
            takeoff_arrival_state_ = arrival_helper::State{};
            quint_curve_.clear();
            return true;
        }
    }
    return false;
}


bool PX4_OriginController::is_takeoff_complete() {
    return takeoff_complete_.load(std::memory_order_relaxed);
}

bool PX4_OriginController::is_point_complete() {
    return point_complete_.load(std::memory_order_relaxed);
}

#ifdef Raptor_Test
bool PX4_OriginController::land(bool land_type, double max_land_velocity) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    if (land_type == 1) {
        // 切换为px4的auto land模式
        mavros_helper_.set_px4_mode(control_common::FlightMode::AutoLand);
        return mavros_helper_.get_state().landed_state == control_common::LandedState::OnGround;
    }
    const ros::Time now = ros::Time::now();
    if (land_complete_.load(std::memory_order_relaxed)) {
        return true;
    }
    if (start_land_time_ == ros::Time(0)) {
        start_land_time_ = now;
        land_touchground_time_ = ros::Time(0);
        land_near_ground_ = false;
        land_yaw_ = mavros_helper_.get_yaw_rad();
        hover_point_ = last_setpoint_.valid ? last_setpoint_.position : uav_odometry_.position;
    }
    const double descend_rate = std::max(0.05, std::abs(max_land_velocity));
    const double elapsed = (now - start_land_time_).toSec();
    control_common::Mavros_SetpointLocal setpoint_cmd;
    setpoint_cmd.position = hover_point_;
    setpoint_cmd.position.z() =
        std::max(hover_point_.z() - descend_rate * elapsed, takeoff_ground_height - 0.1);
    setpoint_cmd.yaw = land_yaw_;
    mavros_helper_.pub_local_setpoint(setpoint_cmd);
    cache_local_setpoint(setpoint_cmd);

    const bool near_ground = uav_odometry_.position.z() <= takeoff_ground_height + 0.08;
    const bool velocity_low = uav_odometry_.velocity.norm() < 0.15;
    const bool landed =
        mavros_helper_.get_state().landed_state == control_common::LandedState::OnGround;
    land_near_ground_ = near_ground;
    if ((near_ground && velocity_low) || landed) {
        if (land_touchground_time_ == ros::Time(0)) {
            land_touchground_time_ = now;
        }
        if ((now - land_touchground_time_).toSec() > 1.0) {
            mavros_helper_.set_arm(false);
            if (!mavros_helper_.get_state().armed) {
                land_complete_.store(true, std::memory_order_relaxed);
                takeoff_complete_.store(false, std::memory_order_relaxed);
                start_land_time_ = ros::Time(0);
                land_near_ground_ = false;
                land_touchground_time_ = ros::Time(0);
                return true;
            }
        }
    } else {
        land_touchground_time_ = ros::Time(0);
    }
    return false;
}

#endif

// 仅测试
#ifndef Raptor_Test
bool PX4_OriginController::land(bool land_type, double max_land_velocity) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    if (land_type == 1) {
        // 切换为px4的auto land模式
        mavros_helper_.set_px4_mode(control_common::FlightMode::AutoLand);
        bool land_state =
            mavros_helper_.get_state().landed_state == control_common::LandedState::OnGround;
        return land_state;
    }
    ros::Time now = ros::Time::now();
    if (land_complete_.load(std::memory_order_relaxed)) {
        return true;
    }
    // 第一次进入land函数，先初始化用到的变量
    if (start_land_time_ == ros::Time(0)) {
        // 清除掉五次项曲线的参数，然后重新填入
        quint_curve_.clear();

        // 使用当前setpoint的输出position部分为轨迹的起点，保证setpoint的连续性
        control_common::Mavros_SetpointLocal last_setpoint;
        last_setpoint = mavros_helper_.get_target_local();
        Eigen::Vector3d last_setpoint_position = last_setpoint.position;
        quint_curve_.set_start_trajpoint(last_setpoint_position, Eigen::Vector3d::Zero());
        // 使用当前位置的xy和地面高度+2作为轨迹的终点,速度使用0.1
        Eigen::Vector3d land_position = Eigen::Vector3d(
            uav_odometry_.position.x(), uav_odometry_.position.y(), takeoff_ground_height + 0.2);
        Eigen::Vector3d land_vel = Eigen::Vector3d(0, 0, -0.2);
        // 设置轨迹的终点参数
        quint_curve_.set_end_trajpoint(land_position, land_vel);
        // 使用最大降落速度求解时间
        quint_curve_.set_curve_maxvel(max_land_velocity);
        // 记录当前的yaw角
        land_yaw_ = mavros_helper_.get_yaw_rad();
        // 更新时间戳
        start_land_time_ = now;
    }
    if (land_near_ground_ == false) {
        // 得到五次项曲线输出
        curve::QuinticCurveState curve_result;
        curve_result = quint_curve_.get_result();
        // 使用五次项输出填充setpoint_cmd
        control_common::Mavros_SetpointLocal setpoint_cmd;
// setpoint_cmd.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
#ifndef Raptor_Test
        setpoint_cmd.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
#endif
        setpoint_cmd.position = curve_result.position;
#ifndef Raptor_Test
        setpoint_cmd.velocity = curve_result.velocity;
        setpoint_cmd.accel_or_force = curve_result.acceleration;
#endif
        // yaw角设置为降落触发时的yaw角
        setpoint_cmd.yaw = land_yaw_;
        // 发布setpotin_cmd
        mavros_helper_.pub_local_setpoint(setpoint_cmd);
        cache_local_setpoint(setpoint_cmd);
    }
    // 检查轨迹是否执行到目标点，也就是近地段
    bool near_ground = (uav_odometry_.position.z() - quint_curve_.get_end_position().z() < 0.05);
    bool velocity_low = (uav_odometry_.velocity.norm() < 0.15);
    control_common::LandedState px4_landed = mavros_helper_.get_state().landed_state;
    if (near_ground && velocity_low) {
        land_near_ground_ = true;
    }
    // 轨迹结束后，设置为锁xy然后持续下降
    if (land_near_ground_ == true) {
        control_common::Mavros_SetpointLocal setpoint_cmd;
// setpoint_cmd.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
#ifndef Raptor_Test
        setpoint_cmd.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate |
                            control_common::Mavros_SetpointLocal::Mask::IgnorePz;
#endif
        curve::QuinticCurveState curve_result;
        curve_result = quint_curve_.get_result();
        setpoint_cmd.position = curve_result.position;
        setpoint_cmd.velocity = curve_result.velocity;

        setpoint_cmd.yaw = land_yaw_;
        // 持续检查是否接触地面
        velocity_low = std::abs(uav_odometry_.velocity.z()) < 0.1;
        if (velocity_low == true) {
            setpoint_cmd.velocity.z() -= 0.2;
        }
        bool px4_land = px4_landed == control_common::LandedState::OnGround;
        if (velocity_low && px4_land) {  // 考虑传感器漂移，这里用||逻辑
            if (land_touchground_time_ == ros::Time(0)) {
                land_touchground_time_ = now;
            }
            if ((now - land_touchground_time_).toSec() > 1.0) {
                mavros_helper_.set_arm(false);  // 上锁
                if (mavros_helper_.get_state().armed == false) {
                    // 上锁成功，清理上下文
                    land_complete_.store(true, std::memory_order_relaxed);
                    takeoff_complete_.store(false, std::memory_order_relaxed);
                    quint_curve_.clear();
                    start_land_time_ = ros::Time(0);
                    land_near_ground_ = false;
                    land_touchground_time_ = ros::Time(0);
                }
                return land_complete_.load(std::memory_order_relaxed);
            }
        } else {
            land_touchground_time_ = ros::Time(0);
        }
        mavros_helper_.pub_local_setpoint(setpoint_cmd);
        cache_local_setpoint(setpoint_cmd);
    }
    return false;
}

#endif

bool PX4_OriginController::is_land_complete() {
    return land_complete_.load(std::memory_order_relaxed);
}

bool PX4_OriginController::set_hover_point(control_common::UAVStateEstimate current_odom) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    hover_point_ = current_odom.position;
    hover_yaw_ = current_odom.get_yaw();
    return true;
}

bool PX4_OriginController::hover() {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    // hover 阶段，我们认为使用上一次的setpoint中的position值用于悬停
    control_common::Mavros_SetpointLocal hover_setpoint;
    hover_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    // hover_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreVx |
    //                       control_common::Mavros_SetpointLocal::Mask::IgnoreVy |
    //                       control_common::Mavros_SetpointLocal::Mask::IgnoreVz |
    //                       control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
    //                       control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
    //                       control_common::Mavros_SetpointLocal::Mask::IgnoreAfz |
    //                       control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
    hover_setpoint.position = hover_point_;
    hover_setpoint.yaw = hover_yaw_;
    mavros_helper_.pub_local_setpoint(hover_setpoint);
    cache_local_setpoint(hover_setpoint);
    // 是否应当分析误差来确定返回值为true or false呢
    return true;
}

bool PX4_OriginController::emergency_kill() {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    return mavros_helper_.emergency_kill();
}

bool PX4_OriginController::move_point(controller_data_types::TargetPoint_t point) {
    return move_point_impl(point, false);
}

bool PX4_OriginController::move_velocity(controller_data_types::TargetVelocity_t velocity) {
    reset_point_motion_context();
    // 请注意，velocity是一个比较危险的接口，我们会默认返回true
    velocity.velocity =
        reference_limit_helper::clamp_velocity_per_axis(velocity.velocity, max_velocity_);
    control_common::Mavros_SetpointLocal velocity_setpoint;
    velocity_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
#ifndef Raptor_Test
    velocity_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnorePx |
                             control_common::Mavros_SetpointLocal::Mask::IgnorePy |
                             control_common::Mavros_SetpointLocal::Mask::IgnorePz |
                             control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                             control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                             control_common::Mavros_SetpointLocal::Mask::IgnoreAfz;
#endif

    velocity_setpoint.velocity = velocity.velocity;
    if (std::abs(velocity.yaw_rate) > 1e-6) {
        yaw_reference_state_.reset();
        velocity_setpoint.yaw = uav_odometry_.get_yaw();
        velocity_setpoint.yaw_rate =
            reference_limit_helper::clamp_yaw_rate(velocity.yaw_rate, max_yaw_rate_rad_s_);
    } else {
        const ros::Time now = velocity.stamp.isZero() ? ros::Time::now() : velocity.stamp;
        velocity_setpoint.yaw = update_limited_yaw_target(velocity.yaw, now);
        velocity_setpoint.yaw_rate = 0.0;
    }
    mavros_helper_.pub_local_setpoint(velocity_setpoint);
    cache_local_setpoint(velocity_setpoint);
    return true;
}

bool PX4_OriginController::move_trajectory(
    controller_data_types::TargetTrajectoryPoint_t trajpoint) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    warn_if_trajectory_exceeds_limits(trajpoint);

    control_common::Mavros_SetpointLocal trajpoint_setpoint;
    trajpoint_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    trajpoint_setpoint.position = trajpoint.position;
    trajpoint_setpoint.velocity = trajpoint.velocity;
    trajpoint_setpoint.accel_or_force = trajpoint.acceleration;
    trajpoint_setpoint.yaw = trajpoint.yaw;
    trajpoint_setpoint.yaw_rate = trajpoint.yaw_rate;
    mavros_helper_.pub_local_setpoint(trajpoint_setpoint);
    cache_local_setpoint(trajpoint_setpoint);
    desired_state_ = trajpoint;
    // 我们如何设置轨迹的反馈呢？这里的问题是，拿到了轨迹的反馈，有什么用呢？
    // 轨迹是一个高频变化的值，因此拿到轨迹的反馈，其实没有什么用，so
    // 返回的是true表示接受到了o.o
    return true;
}
// BUG:
// body系的函数存在一点小问题,主要表现在与move_point使用方式上不一致，body系在持续调用的时候，如果point不变，认为是原先不变的一个目标点
bool PX4_OriginController::move_point_body(controller_data_types::TargetBodyPoint_t point) {
    // 新期望点判断常量
    constexpr double kPosEps = 1e-3;
    constexpr double kYawEps = 1e-3;
    // 是否为新目标点状态
    bool is_new_body_target = !body_point_target_initialized_;
    controller_data_types::TargetPoint_t world_point = last_point_;
    // 计算差值
    const double dxy = (point.position_xy - last_point_body_.position_xy).norm();
    if (dxy > kPosEps || std::abs(point.fixed_height - last_point_body_.fixed_height) > kPosEps)
        is_new_body_target = true;

    if (!is_new_body_target && std::abs(point.yaw - last_point_body_.yaw) > kYawEps) {
        is_new_body_target = true;
    }

    if (is_new_body_target) {
        // 得到当前的yaw角
        const double yaw = mavros_helper_.get_yaw_rad();
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        // 做body系到local系的转换
        const Eigen::Vector2d p_b = point.position_xy;
        Eigen::Vector2d delta_w;
        delta_w.x() = c * p_b.x() - s * p_b.y();
        delta_w.y() = s * p_b.x() + c * p_b.y();
        // 目标点 = 当前点 + 增量点
        Eigen::Vector3d des_position = Eigen::Vector3d::Zero();
        des_position.x() = uav_odometry_.position.x() + delta_w.x();
        des_position.y() = uav_odometry_.position.y() + delta_w.y();
        des_position.z() = point.fixed_height;
        world_point.position = des_position;

        // yaw语义：若给了yaw，按“相对机体yaw增量”处理；否则保持当前朝向
        world_point.yaw = yaw + point.yaw;
        last_point_body_ = point;
        body_point_target_initialized_ = true;
    }
    // 复用已有move_point逻辑（到点判定/缓存/停留判定）
    return move_point_impl(world_point, true);
}
bool PX4_OriginController::move_velocity_body(
    controller_data_types::TargetBodyVelocity_t velocity) {
    reset_point_motion_context();

    // 读取当前的yaw角
    const double yaw = mavros_helper_.get_yaw_rad();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    // 转换输入的body velocity -> world velocity
    Eigen::Vector2d v_w_xy;
    v_w_xy.x() = c * velocity.velocity_xy.x() - s * velocity.velocity_xy.y();
    v_w_xy.y() = s * velocity.velocity_xy.x() + c * velocity.velocity_xy.y();

    control_common::Mavros_SetpointLocal velocity_setpoint;
    velocity_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    velocity_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnorePx |
                             control_common::Mavros_SetpointLocal::Mask::IgnorePy |
                             control_common::Mavros_SetpointLocal::Mask::IgnoreVz |
                             control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                             control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                             control_common::Mavros_SetpointLocal::Mask::IgnoreAfz;
    velocity_setpoint.position.z() = velocity.fixed_height;
    const Eigen::Vector3d limited_velocity = reference_limit_helper::clamp_velocity_per_axis(
        Eigen::Vector3d(v_w_xy.x(), v_w_xy.y(), 0.0), max_velocity_);
    velocity_setpoint.velocity.x() = limited_velocity.x();
    velocity_setpoint.velocity.y() = limited_velocity.y();
    velocity_setpoint.velocity.z() = 0.0;
    if (std::abs(velocity.yaw_rate) > 1e-6) {
        yaw_reference_state_.reset();
        velocity_setpoint.yaw = yaw;
        velocity_setpoint.yaw_rate =
            reference_limit_helper::clamp_yaw_rate(velocity.yaw_rate, max_yaw_rate_rad_s_);
    } else {
        velocity_setpoint.yaw = update_limited_yaw_target(yaw + velocity.yaw, ros::Time::now());
        velocity_setpoint.yaw_rate = 0.0;
    }
    mavros_helper_.pub_local_setpoint(velocity_setpoint);
    cache_local_setpoint(velocity_setpoint);
    return true;
}
// WGS84不知道怎么测试，先放一边
bool PX4_OriginController::move_point_wgs84(geographic_msgs::GeoPoint point) {
    reset_point_motion_context();
    yaw_reference_state_.reset();
    return false;
}
// -------------起降状态查询接口------------

void PX4_OriginController::pub_controller_state() {
    update_log_snapshot();

    if (!has_uav_odometry_.load(std::memory_order_relaxed)) {
        return;
    }

    const control_common::Mavros_SetpointLocal px4_local_target = mavros_helper_.get_target_local();
    const control_common::Mavros_SetpointAttitude px4_attitude_target =
        mavros_helper_.get_target_attitude();
    const control_common::Mavros_SetpointLocal controller_local_output =
        last_setpoint_.valid ? last_setpoint_ : px4_local_target;

    sunray_msgs::UAVControllerState msg;
    msg.header.stamp =
        uav_odometry_.timestamp.isZero() ? ros::Time::now() : uav_odometry_.timestamp;
    msg.reference_frame = controller_reference_frame(controller_local_output);
    msg.controller_type = sunray_msgs::UAVControllerState::PX4_ORIGINAL_CONTROLLER;

    msg.desired_pos = eigen_helper::to_ros_point(desired_state_.position);
    msg.desired_vel = eigen_helper::to_ros_vector3(desired_state_.velocity);
    msg.desired_acc = eigen_helper::to_ros_vector3(desired_state_.acceleration);
    msg.desired_yaw = desired_state_.yaw;
    msg.desired_yawrate = desired_state_.yaw_rate;

    msg.current_pos = eigen_helper::to_ros_point(uav_odometry_.position);
    msg.current_vel = eigen_helper::to_ros_vector3(uav_odometry_.velocity);
    msg.current_attitude = eigen_helper::to_ros_quaternion(uav_odometry_.orientation);
    msg.current_bodyrate = eigen_helper::to_ros_vector3(uav_odometry_.bodyrate);
    msg.current_yaw = eigen_helper::get_yaw_from_orientation(uav_odometry_.orientation);

    msg.pos_error = eigen_helper::to_ros_vector3(desired_state_.position - uav_odometry_.position);
    msg.vel_error = eigen_helper::to_ros_vector3(desired_state_.velocity - uav_odometry_.velocity);
    msg.yaw_error = eigen_helper::wrap_angle(desired_state_.yaw - msg.current_yaw);
    if (px4_attitude_target.valid) {
        msg.attitude_error = eigen_helper::to_ros_vector3(
            quaternion_error_to_vector(uav_odometry_.orientation, px4_attitude_target.orientation));
    }

    msg.position_from_ctrl = eigen_helper::to_ros_vector3(controller_local_output.position);
    msg.velocity_from_ctrl = eigen_helper::to_ros_vector3(controller_local_output.velocity);
    msg.acceleration_from_ctrl =
        eigen_helper::to_ros_vector3(controller_local_output.accel_or_force);
    msg.yaw_from_ctrl = controller_local_output.yaw;
    msg.yawrate_from_ctrl = controller_local_output.yaw_rate;
    // PX4 原生控制器的姿态/角速度/推力输出由 PX4 内环生成，这里使用回读值对齐消息语义。
    msg.attitude_from_ctrl = eigen_helper::to_ros_quaternion(px4_attitude_target.orientation);
    msg.bodyrate_from_ctrl = eigen_helper::to_ros_vector3(px4_attitude_target.body_rate);
    msg.thrust_from_ctrl = px4_attitude_target.thrust;

    msg.position_from_px4 = eigen_helper::to_ros_vector3(px4_local_target.position);
    msg.velocity_from_px4 = eigen_helper::to_ros_vector3(px4_local_target.velocity);
    msg.acceleration_from_px4 = eigen_helper::to_ros_vector3(px4_local_target.accel_or_force);
    msg.yaw_from_px4 = px4_local_target.yaw;
    msg.yawrate_from_px4 = px4_local_target.yaw_rate;
    msg.attitude_from_px4 = eigen_helper::to_ros_quaternion(px4_attitude_target.orientation);
    msg.bodyrate_from_px4 = eigen_helper::to_ros_vector3(px4_attitude_target.body_rate);
    msg.thrust_from_px4 = px4_attitude_target.thrust;

    controller_state_pub_.publish(msg);
}
// clang-format off
void PX4_OriginController::load_and_validate_config_or_throw() {
    // 首先在构造函数中我们已经判断了config_yamlfile_path_非空,因此这里不再判断
    YAML::Node root;  // 构造一个YAML文件的根节点
    // 由于读取的过程可能引发异常，因此使用try语法
    try {
        root = YAML::LoadFile(config_yamlfile_path_); // 从指定的路径中读取yaml文件并解析为YAML::Node
    } catch (const YAML::Exception& e) {  // 如果解析的过程中发生错误，捕捉异常
        throw std::runtime_error("Failed to load yaml file '" + config_yamlfile_path_ + ":" + e.what());
    }
    // 顺利读取，取出字段basic_param的部分
    const YAML::Node basic_param = root["basic_param"];
    // 如果basic_param为空，或者不是键值对的形式，则抛出异常
    if (!basic_param || !basic_param.IsMap()) {
        throw std::runtime_error("the yaml file '" + config_yamlfile_path_ + "' is missing a valid basic_param map");
    }
    // 由于OriginalController需要的参数不多，我们直接拿字段读
    if (!basic_param["fuse_odom_type"]) {
        throw std::runtime_error("miss param 'fuse_odom_type'");
    } else {
        config_param_.fuse_odom_type = basic_param["fuse_odom_type"].as<int>();
    }
    if (!basic_param["fuse_odom_frequency"]) {
        throw std::runtime_error("miss param 'fuse_odom_frequency'");
    } else {
        config_param_.fuse_odom_frequency = basic_param["fuse_odom_frequency"].as<double>();
    }
    // 检查一下参数是否正常
    if(config_param_.fuse_odom_type != 0 && config_param_.fuse_odom_type != 1 && config_param_.fuse_odom_type != 2 ){
        throw std::runtime_error("param 'fuse_odom_type' value must be 0,1,2");
    }
    // 限制融合的频率
    config_param_.fuse_odom_frequency = std::max(10.0, config_param_.fuse_odom_frequency);
    config_param_.fuse_odom_frequency = std::min(200.0, config_param_.fuse_odom_frequency);

    const YAML::Node arrival_judge_param = root["arrival_judge_param"];
    if (!arrival_judge_param || !arrival_judge_param.IsMap()) {
        throw std::runtime_error("the yaml file '" + config_yamlfile_path_ +
                                 "' is missing a valid arrival_judge_param map");
    }
    if (!arrival_judge_param["judge_stabile_time_s"]) {
        throw std::runtime_error("miss param 'arrival_judge_param.judge_stabile_time_s'");
    }
    if (!arrival_judge_param["pos_stabile_err_m"]) {
        throw std::runtime_error("miss param 'arrival_judge_param.pos_stabile_err_m'");
    }
    if (!arrival_judge_param["vel_stabile_err_mps"]) {
        throw std::runtime_error("miss param 'arrival_judge_param.vel_stabile_err_mps'");
    }

    arrival_judge_config_.stable_time_s = arrival_judge_param["judge_stabile_time_s"].as<double>();
    arrival_judge_config_.pos_err_m = arrival_judge_param["pos_stabile_err_m"].as<double>();
    arrival_judge_config_.vel_err_mps = arrival_judge_param["vel_stabile_err_mps"].as<double>();

    takeoff_arrival_config_ = arrival_judge_config_;
    takeoff_arrival_config_.require_pos_ok_before_vel_only = true;

    if (arrival_judge_config_.stable_time_s <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.judge_stabile_time_s' must > 0");
    }
    if (arrival_judge_config_.pos_err_m <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.pos_stabile_err_m' must > 0");
    }
    if (arrival_judge_config_.vel_err_mps <= 0.0) {
        throw std::runtime_error("param 'arrival_judge_param.vel_stabile_err_mps' must > 0");
    }

    const YAML::Node velocity_param = root["velocity_param"];
    if (!velocity_param || !velocity_param.IsMap()) {
        throw std::runtime_error("the yaml file '" + config_yamlfile_path_ +
                                 "' is missing a valid velocity_param map");
    }
    const YAML::Node max_velocity = velocity_param["max_velocity"];
    if (!max_velocity || !max_velocity.IsMap() || !max_velocity["x_vel"] || !max_velocity["y_vel"] ||
        !max_velocity["z_vel"]) {
        throw std::runtime_error("miss param 'velocity_param.max_velocity.(x_vel|y_vel|z_vel)'");
    }
    max_velocity_.x() = max_velocity["x_vel"].as<double>();
    max_velocity_.y() = max_velocity["y_vel"].as<double>();
    max_velocity_.z() = max_velocity["z_vel"].as<double>();
    if (max_velocity_.x() <= 0.0 || max_velocity_.y() <= 0.0 || max_velocity_.z() <= 0.0) {
        throw std::runtime_error("param 'velocity_param.max_velocity.*' must > 0");
    }
    if (!velocity_param["yaw_rate"]) {
        throw std::runtime_error("miss param 'velocity_param.yaw_rate'");
    }
    max_yaw_rate_rad_s_ = deg2rad(velocity_param["yaw_rate"].as<double>());
    if (max_yaw_rate_rad_s_ <= 0.0) {
        throw std::runtime_error("param 'velocity_param.yaw_rate' must > 0");
    }
}
// clang-format on

void PX4_OriginController::ensure_fusion_param_ready_or_throw() {
    if (config_param_.fuse_odom_type == 0) {
        return;  // 不做视觉融合时无需检查
    }
    // 构造lambda表达式简化后续重读
    auto check_param = [this]() -> bool {
        px4_param_decode::EKF2_EV_CTRL ev_ctrl_read;
        px4_param_decode::EKF2_HGT_REF hgt_ref_read;
        mavros_param_.read_param(&ev_ctrl_read);
        mavros_param_.read_param(&hgt_ref_read);

        bool ev_ok = ev_ctrl_read.enable_horizontal_position() &&
                     ev_ctrl_read.enable_vertical_position() && ev_ctrl_read.enable_yaw();
        bool hgt_ok = hgt_ref_read.is_vision();
        return ev_ok && hgt_ok;
    };
    // 先读一次，如果满足直接结束
    if (check_param()) {
        return;
    }
    // 不满足，写入目标参数
    px4_param_types::EKF2_EV_CTRL ev_ctrl_write;
    ev_ctrl_write.enable_Horizontalposition();
    ev_ctrl_write.enable_Verticalposition();
    ev_ctrl_write.enable_Yaw();
    mavros_param_.set_param(ev_ctrl_write);

    px4_param_types::EKF2_HGT_REF hgt_ref_write;
    hgt_ref_write.enable_vision();
    mavros_param_.set_param(hgt_ref_write);

    // TODO: 这里需要重启EKF2

    // 重读确认
    const int max_retry = 5;
    const double retry_interval_sec = 0.2;
    for (int i = 0; i < max_retry; ++i) {
        ros::Duration(retry_interval_sec).sleep();
        if (check_param()) {
            return;
        }
    }

    // 修改失败，重试无效，抛出异常
    throw std::runtime_error("Failed to apply fusion params after retries: "
                             "require EKF2_EV_CTRL(hpos,vpos,yaw)=on and EKF2_HGT_REF=vision");
}

bool PX4_OriginController::check_px4_basic_state() {
    // px4的基础状态，指的是什么呢？
    // 1. 处于position模式(这点是我们需要确认的，默认是position模式，遥控器也是双回中)
    // 2. 处于未解锁模式
    // 3. 着地检测器处于地面
    const control_common::Mavros_State st = mavros_helper_.get_state();

    const bool mode_ok = (st.flight_mode == control_common::FlightMode::Posctl);
    const bool land_ok = (st.landed_state == control_common::LandedState::OnGround);
    const bool arm_ok = (st.armed == false);

    return mode_ok && land_ok && arm_ok;
}

bool PX4_OriginController::check_mavros_stream_ready() {
    return mavros_helper_.is_ready();
}

void PX4_OriginController::pub_px4_state_timer_cb(const ros::TimerEvent&) {
    mavros_helper_.pub_px4_state();
}

void PX4_OriginController::pub_vision_fuse_timer_cb(const ros::TimerEvent&) {
    if (!can_fuse_.exchange(false, std::memory_order_relaxed)) {
        return;
    }
    mavros_helper_.pub_vision_pose(uav_odometry_);
}
