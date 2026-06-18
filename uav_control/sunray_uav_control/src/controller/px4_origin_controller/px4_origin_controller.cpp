#include "controller/px4_origin_controller.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "control_data_types/mavros_helper_data_types.hpp"
#include "eigen_helper.hpp"
#include "utils/control_config_loader.hpp"
#include "utils/body_frame_reference_helper.hpp"
#include "utils/orientation_utils.hpp"
#include <ros/ros.h>
#include <algorithm>
#include <cmath>

namespace {

constexpr double kNewTargetYawEps = 1e-3;

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

mavros_msgs::PositionTarget to_position_target_msg(
    const control_common::Mavros_SetpointLocal& setpoint) {
    mavros_msgs::PositionTarget msg;
    msg.header.stamp = setpoint.timestamp.isZero() ? ros::Time::now() : setpoint.timestamp;

    switch (setpoint.frame) {
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned:
        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        break;
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Offset_Ned:
        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_OFFSET_NED;
        break;
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Ned:
        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        break;
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Offset_Ned:
        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;
        break;
    }

    msg.type_mask = setpoint.mask;
    msg.position.x = setpoint.position.x();
    msg.position.y = setpoint.position.y();
    msg.position.z = setpoint.position.z();
    msg.velocity.x = setpoint.velocity.x();
    msg.velocity.y = setpoint.velocity.y();
    msg.velocity.z = setpoint.velocity.z();
    msg.acceleration_or_force.x = setpoint.accel_or_force.x();
    msg.acceleration_or_force.y = setpoint.accel_or_force.y();
    msg.acceleration_or_force.z = setpoint.accel_or_force.z();
    msg.yaw = setpoint.yaw;
    msg.yaw_rate = setpoint.yaw_rate;
    return msg;
}

}  // namespace

// 构造函数，读取参数
PX4_OriginController::PX4_OriginController(ros::NodeHandle& nh) : nh_(nh) {
    (void)sunray_config::get_control_config_paths_or_throw();
}

bool PX4_OriginController::init() {
    // 加载参数并校验参数
    load_and_validate_config_or_throw();
    // 配置读取完毕，初始化mavros_helper
    mavros_helper_.init();
    if (fuse_odom_type_ != 0) {  // 融合外部里程计到px4，注册定时器
        mavros_helper_.set_vision_fuse_type(fuse_odom_type_);
        // 创建定时器
        pub_vision_pose_timer_ =
            nh_.createTimer(ros::Duration(1.0 / fuse_odom_frequency_),
                            &PX4_OriginController::pub_vision_fuse_timer_cb,
                            this);
    }
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
    has_uav_odometry_.store(has_valid_timestamp, std::memory_order_relaxed);
    if (has_valid_timestamp) {
        last_odom_timestamp_ = odom.timestamp;
    }
}

void PX4_OriginController::set_external_odom_for_fusion(
    const control_common::UAVStateEstimate& odom) {
    external_fusion_odometry_ = odom;
    const bool has_valid_timestamp = !odom.timestamp.isZero();
    const bool is_new_odom = has_valid_timestamp && odom.timestamp != last_fusion_odom_timestamp_;
    if (is_new_odom) {
        last_fusion_odom_timestamp_ = odom.timestamp;
        can_fuse_.store(true, std::memory_order_relaxed);
    }
}

void PX4_OriginController::cache_local_setpoint(
    const control_common::Mavros_SetpointLocal& setpoint) {
    std::lock_guard<std::mutex> lock(last_setpoint_mutex_);
    last_setpoint_ = setpoint;
    last_setpoint_.timestamp = ros::Time::now();
    last_setpoint_.valid = true;
    desired_state_ = desired_state_from_local_setpoint(setpoint, uav_odometry_);
}

bool PX4_OriginController::get_last_position_target(mavros_msgs::PositionTarget& msg) const {
    std::lock_guard<std::mutex> lock(last_setpoint_mutex_);
    if (!last_setpoint_.valid) {
        return false;
    }
    msg = to_position_target_msg(last_setpoint_);
    return true;
}

// -------------运动相关接口------------

// 任务流程描述为 takoff ： [mode]position -> [mode]offboard setpoint 流持续发送
//              land  :  [mode]offboard -> [mode]position setpoint 流停止发送
// 因此我们提供一个set_postion_mode函数用于设置为position模式
void PX4_OriginController::set_position_mode() {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    clear_cached_setpoint();
    control_common::Mavros_State state = mavros_helper_.get_state();
    if (state.flight_mode != control_common::FlightMode::Posctl) {
        mavros_helper_.set_px4_mode(control_common::FlightMode::Posctl);
    }
}

void PX4_OriginController::clear_motion_curve() {
    motion_curve_.clear();
    motion_curve_owner_ = MotionCurveOwner::None;
}

void PX4_OriginController::begin_motion_curve(MotionCurveOwner owner) {
    motion_curve_.clear();
    motion_curve_owner_ = owner;
}

void PX4_OriginController::reset_point_motion_context() {
    if (motion_curve_owner_ == MotionCurveOwner::MovePoint) {
        clear_motion_curve();
    }
    point_arrival_state_ = arrival_helper::State{};
    point_complete_.store(false, std::memory_order_relaxed);
    point_target_initialized_ = false;
    body_point_target_initialized_ = false;
    last_point_ = controller_data_types::TargetPoint_t{};
    last_point_body_ = controller_data_types::TargetBodyPoint_t{};
}

void PX4_OriginController::clear_cached_setpoint() {
    std::lock_guard<std::mutex> lock(last_setpoint_mutex_);
    last_setpoint_ = control_common::Mavros_SetpointLocal{};
    desired_state_ = has_uav_odometry_.load(std::memory_order_relaxed)
                         ? make_hold_desired_state(uav_odometry_)
                         : controller_data_types::TargetTrajectoryPoint_t{};
}

controller_data_types::TargetTrajectoryPoint_t PX4_OriginController::make_hold_desired_state(
    const control_common::UAVStateEstimate& odom) const {
    controller_data_types::TargetTrajectoryPoint_t desired_state;
    desired_state.position = odom.position;
    desired_state.velocity = Eigen::Vector3d::Zero();
    desired_state.acceleration = Eigen::Vector3d::Zero();
    desired_state.jerk = Eigen::Vector3d::Zero();
    desired_state.yaw = eigen_helper::get_yaw_from_orientation(odom.orientation);
    desired_state.yaw_rate = 0.0;
    return desired_state;
}

void PX4_OriginController::publish_hold_setpoint(const Eigen::Vector3d& position, double yaw) {
    control_common::Mavros_SetpointLocal hold_setpoint;
    hold_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    hold_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreVx |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreVy |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreVz |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreAfz |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
    hold_setpoint.position = position;
    hold_setpoint.velocity = Eigen::Vector3d::Zero();
    hold_setpoint.accel_or_force = Eigen::Vector3d::Zero();
    hold_setpoint.yaw = yaw;
    hold_setpoint.yaw_rate = 0.0;
    mavros_helper_.pub_local_setpoint(hold_setpoint);
    cache_local_setpoint(hold_setpoint);
}

void PX4_OriginController::reset_after_landing_success() {
    takeoff_complete_.store(false, std::memory_order_relaxed);
    point_complete_.store(false, std::memory_order_relaxed);
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    start_land_time_ = ros::Time(0);
    land_near_ground_ = false;
    land_touchground_time_ = ros::Time(0);
    start_checkout_offboard_time_ = ros::Time(0);
    last_checkout_offboard_time_ = ros::Time(0);
    last_arm_time_ = ros::Time(0);
    clear_cached_setpoint();
    if (has_uav_odometry_.load(std::memory_order_relaxed)) {
        hover_point_ = uav_odometry_.position;
        hover_yaw_ = uav_odometry_.get_yaw();
    } else {
        hover_point_ = Eigen::Vector3d::Zero();
        hover_yaw_ = 0.0;
    }
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
        "[PX4_OriginController] trajectory reference exceeds velocity_param limits: vel="
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

    const bool is_new_target =
        !point_target_initialized_ ||
        (point.position - last_point_.position).norm() > kNewTargetPosEps ||
        std::abs(normalize_angle_rad(point.yaw - last_point_.yaw)) > kNewTargetYawEps;
    const bool need_rebuild_curve =
        is_new_target || motion_curve_owner_ != MotionCurveOwner::MovePoint;
    if (need_rebuild_curve) {
        point_arrival_state_ = arrival_helper::State{};
        point_complete_.store(false, std::memory_order_relaxed);
        last_point_ = point;
        point_target_initialized_ = true;
        begin_motion_curve(MotionCurveOwner::MovePoint);
        motion_curve_.set_start_trajpoint(uav_odometry_.position, uav_odometry_.velocity);
        motion_curve_.set_end_trajpoint(point.position, Eigen::Vector3d::Zero());
        motion_curve_.set_curve_maxvel(reference_limit_helper::compute_point_curve_maxvel(
            uav_odometry_.position, point.position, max_velocity_));
    }

    const ros::Time now = ros::Time::now();
    control_common::Mavros_SetpointLocal send_setpoint;
    send_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    send_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreAfz |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
    const curve::QuinticCurveState curve_result = motion_curve_.get_result();
    const bool curve_still_running = curve_result.valid && !motion_curve_.is_finished();
    if (curve_still_running) {
        send_setpoint.position = curve_result.position;
        send_setpoint.velocity = curve_result.velocity;
    } else {
        send_setpoint.mask |= control_common::Mavros_SetpointLocal::Mask::IgnoreVx |
                              control_common::Mavros_SetpointLocal::Mask::IgnoreVy |
                              control_common::Mavros_SetpointLocal::Mask::IgnoreVz;
        send_setpoint.position = last_point_.position;
        send_setpoint.velocity = Eigen::Vector3d::Zero();
    }
    send_setpoint.accel_or_force = Eigen::Vector3d::Zero();
    send_setpoint.yaw = update_limited_yaw_target(point.yaw, now);
    send_setpoint.yaw_rate = 0.0;
    mavros_helper_.pub_local_setpoint(send_setpoint);
    cache_local_setpoint(send_setpoint);

    if (point_complete_.load(std::memory_order_relaxed)) {
        return true;
    }

    const double pos_err = (uav_odometry_.position - last_point_.position).norm();
    const double vel_err = uav_odometry_.velocity.norm();
    const double yaw_err =
        std::abs(normalize_angle_rad(last_point_.yaw - uav_odometry_.get_yaw()));
    const double yaw_rate_err = std::abs(uav_odometry_.bodyrate.z());
    const bool arrival_ok = arrival_helper::update_pose_and_check(point_arrival_state_,
                                                                  arrival_judge_config_,
                                                                  pos_err,
                                                                  vel_err,
                                                                  yaw_err,
                                                                  yaw_rate_err,
                                                                  now);
    if (!arrival_ok || curve_still_running) {
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
        return hover();
    }
    ros::Time now = ros::Time::now();
    if (motion_curve_owner_ != MotionCurveOwner::Takeoff) {
        takeoff_arrival_state_ = arrival_helper::State{};
        begin_motion_curve(MotionCurveOwner::Takeoff);
        motion_curve_.set_start_trajpoint(uav_odometry_.position, Eigen::Vector3d::Zero());
        motion_curve_.set_end_trajpoint(uav_odometry_.position +
                                           Eigen::Vector3d(0, 0, relative_takeoff_height),
                                       Eigen::Vector3d::Zero());
        motion_curve_.set_curve_maxvel(max_takeoff_velocity);
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
                Eigen::Vector3d start_pos = motion_curve_.get_start_position();
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
                    motion_curve_.set_start_trajpoint(renew_pos,
                                                      Eigen::Vector3d(0, 0, renew_vel.z()));
                } else {
                    return false;
                }
            }

            curve::QuinticCurveState curve_result = motion_curve_.get_result();
            control_common::Mavros_SetpointLocal setpoint_cmd;
            setpoint_cmd.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
            setpoint_cmd.position = curve_result.position;
            setpoint_cmd.velocity = curve_result.velocity;
            setpoint_cmd.accel_or_force = curve_result.acceleration;
            setpoint_cmd.yaw = takeoff_yaw_;
            mavros_helper_.pub_local_setpoint(setpoint_cmd);
            cache_local_setpoint(setpoint_cmd);

            const Eigen::Vector3d takeoff_goal = motion_curve_.get_end_position();
            // PX4 原生位置环在起飞末段可能保留较大的水平稳态误差。
            // TAKEOFF 只需要确认“达到目标高度并稳定下来”，否则会因为 xy 误差卡在 TAKEOFF。
            const double pos_err = std::abs(uav_odometry_.position.z() - takeoff_goal.z());
            const double vel_err = uav_odometry_.velocity.norm();
            if (!arrival_helper::update_and_check(
                    takeoff_arrival_state_, arrival_judge_config_, pos_err, vel_err, now)) {
                return false;
            }

            takeoff_complete_.store(true, std::memory_order_relaxed);
            hover_point_ = motion_curve_.get_end_position();
            hover_yaw_ = takeoff_yaw_;
            start_checkout_offboard_time_ = ros::Time(0);
            last_checkout_offboard_time_ = ros::Time(0);
            last_arm_time_ = ros::Time(0);
            takeoff_arrival_state_ = arrival_helper::State{};
            clear_motion_curve();
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
        clear_motion_curve();
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
                reset_after_landing_success();
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
        clear_motion_curve();
        // 切换为px4的auto land模式
        mavros_helper_.set_px4_mode(control_common::FlightMode::AutoLand);
        const bool land_state =
            mavros_helper_.get_state().landed_state == control_common::LandedState::OnGround;
        if (land_state) {
            reset_after_landing_success();
        }
        return land_state;
    }
    ros::Time now = ros::Time::now();
    if (land_complete_.load(std::memory_order_relaxed)) {
        return true;
    }
    if (motion_curve_owner_ != MotionCurveOwner::Land || start_land_time_ == ros::Time(0)) {
        begin_motion_curve(MotionCurveOwner::Land);
        land_touchground_time_ = ros::Time(0);
        land_near_ground_ = false;
        // 使用当前setpoint的输出position部分为轨迹的起点，保证setpoint的连续性
        control_common::Mavros_SetpointLocal last_setpoint;
        last_setpoint = mavros_helper_.get_target_local();
        Eigen::Vector3d last_setpoint_position = last_setpoint.position;
        motion_curve_.set_start_trajpoint(last_setpoint_position, Eigen::Vector3d::Zero());
        // 使用当前位置的xy和地面高度+2作为轨迹的终点,速度使用0.1
        Eigen::Vector3d land_position = Eigen::Vector3d(
            uav_odometry_.position.x(), uav_odometry_.position.y(), takeoff_ground_height + 0.2);
        Eigen::Vector3d land_vel = Eigen::Vector3d(0, 0, -0.2);
        // 设置轨迹的终点参数
        motion_curve_.set_end_trajpoint(land_position, land_vel);
        // 使用最大降落速度求解时间
        motion_curve_.set_curve_maxvel(max_land_velocity);
        // 记录当前的yaw角
        land_yaw_ = mavros_helper_.get_yaw_rad();
        // 更新时间戳
        start_land_time_ = now;
    }
    if (land_near_ground_ == false) {
        // 得到五次项曲线输出
        curve::QuinticCurveState curve_result;
        curve_result = motion_curve_.get_result();
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
    bool near_ground = (uav_odometry_.position.z() - motion_curve_.get_end_position().z() < 0.05);
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
        curve_result = motion_curve_.get_result();
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
                    clear_motion_curve();
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

uint8_t PX4_OriginController::current_px4_landed_state() const {
    return static_cast<uint8_t>(mavros_helper_.get_state().landed_state);
}

bool PX4_OriginController::set_hover_point(control_common::UAVStateEstimate current_odom) {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    hover_point_ = current_odom.position;
    hover_yaw_ = current_odom.get_yaw();
    publish_hold_setpoint(hover_point_, hover_yaw_);
    return true;
}

bool PX4_OriginController::set_hover_point_to_last_target() {
    if (!point_target_initialized_) {
        return false;
    }
    const controller_data_types::TargetPoint_t target_snapshot = last_point_;
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    hover_point_ = target_snapshot.position;
    hover_yaw_ = target_snapshot.yaw;
    publish_hold_setpoint(hover_point_, hover_yaw_);
    return true;
}

bool PX4_OriginController::hover() {
    publish_hold_setpoint(hover_point_, hover_yaw_);
    return true;
}

bool PX4_OriginController::emergency_kill() {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    return mavros_helper_.emergency_kill();
}

bool PX4_OriginController::move_point(controller_data_types::TargetPoint_t point) {
    return move_point_impl(point, false);
}

bool PX4_OriginController::move_velocity(controller_data_types::TargetVelocity_t velocity) {
    clear_motion_curve();
    reset_point_motion_context();
    // 请注意，velocity是一个比较危险的接口，我们会默认返回true
    const bool fixed_height_active = velocity.fixed_height > 0.0;
    velocity.velocity =
        reference_limit_helper::clamp_velocity_per_axis(velocity.velocity, max_velocity_);
    if (fixed_height_active) {
        velocity.velocity.z() = 0.0;
    }
    control_common::Mavros_SetpointLocal velocity_setpoint;
    velocity_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
#ifndef Raptor_Test
    if (fixed_height_active) {
        velocity_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnorePx |
                                 control_common::Mavros_SetpointLocal::Mask::IgnorePy |
                                 control_common::Mavros_SetpointLocal::Mask::IgnoreVz |
                                 control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                                 control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                                 control_common::Mavros_SetpointLocal::Mask::IgnoreAfz;
    } else {
        velocity_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnorePx |
                                 control_common::Mavros_SetpointLocal::Mask::IgnorePy |
                                 control_common::Mavros_SetpointLocal::Mask::IgnorePz |
                                 control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                                 control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                                 control_common::Mavros_SetpointLocal::Mask::IgnoreAfz;
    }
#endif

    velocity_setpoint.velocity = velocity.velocity;
    if (fixed_height_active) {
        velocity_setpoint.position.z() = velocity.fixed_height;
    }
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
    clear_motion_curve();
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
        world_point = body_frame_reference_helper::to_world_point(uav_odometry_, point);
        last_point_body_ = point;
        body_point_target_initialized_ = true;
    }
    // 复用已有move_point逻辑（到点判定/缓存/停留判定）
    return move_point_impl(world_point, true);
}
bool PX4_OriginController::move_velocity_body(
    controller_data_types::TargetBodyVelocity_t velocity) {
    clear_motion_curve();
    reset_point_motion_context();
    const controller_data_types::TargetVelocity_t world_velocity =
        body_frame_reference_helper::to_world_velocity(uav_odometry_, velocity);

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
        world_velocity.velocity, max_velocity_);
    velocity_setpoint.velocity.x() = limited_velocity.x();
    velocity_setpoint.velocity.y() = limited_velocity.y();
    velocity_setpoint.velocity.z() = limited_velocity.z();
    if (std::abs(velocity.yaw_rate) > 1e-6) {
        yaw_reference_state_.reset();
        velocity_setpoint.yaw = mavros_helper_.get_yaw_rad();
        velocity_setpoint.yaw_rate =
            reference_limit_helper::clamp_yaw_rate(world_velocity.yaw_rate, max_yaw_rate_rad_s_);
    } else {
        velocity_setpoint.yaw = update_limited_yaw_target(world_velocity.yaw, ros::Time::now());
        velocity_setpoint.yaw_rate = 0.0;
    }
    mavros_helper_.pub_local_setpoint(velocity_setpoint);
    cache_local_setpoint(velocity_setpoint);
    return true;
}
// WGS84不知道怎么测试，先放一边
bool PX4_OriginController::move_point_wgs84(geographic_msgs::GeoPoint point) {
    clear_motion_curve();
    reset_point_motion_context();
    yaw_reference_state_.reset();
    return false;
}
// -------------起降状态查询接口------------

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
    mavros_helper_.pub_vision_pose(external_fusion_odometry_);
}
