/**
 * @file sunray_fsm_control.cpp
 * @brief
 * sunray_fsm_control部分描述了Sunray项目中，状态机是如何与控制器进行交互的，或者说在不同的阶段，我们是如何调用不同的函数的
 * @version 0.1
 * @date 2026-04-20
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "statemachine/sunray_fsm.hpp"
#include "utils/orientation_utils.hpp"
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

// -------------------------控制指令执行函数------------------
// 首先我们是这样来设计这个函数的，我们为这个函数新建一个线程，以200Hz的频率or100Hz的频率来运行，这个频率取决于config文件中的controller_update_frequency参数决定
// 然后这个函数是一个void类型，因为线程单独运行并不需要返回值
void Sunray_FSM::update_controller_output() {
    // 我们需要根据状态机自身的状态，来决定如何调用控制器的api函数，但是这里实际上并不涉及到对状态机状态切换的请求，
    // 比如当前为INIT状态，控制命令的回调函数接受到了takeoff的控制命令，那么回调函数会触发一次request，然后外部某个函数检查，更新状态
    // 这个函数，只负责当前状态下的函数调用流程
    // 根据线程安全的设计理念，由于fsm_state与uav_control_cmd这两个变量，在主线程中被写入，在这里被读取，因此需要使用线程锁来避免读取到未定义的值
    sunray_fsm::SunrayState fsm_state_snapshot;
    control_common::UavControlCmd uav_control_cmd;
    control_common::UAVStateEstimate uav_odom;
    bool has_valid_odom = false;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        fsm_state_snapshot = fsm_state_;
    }
    {
        std::lock_guard<std::mutex> lk2(cmd_mutex_);
        uav_control_cmd = last_control_cmd_;
    }
    {
        std::lock_guard<std::mutex> lk2(odom_mutex_);
        has_valid_odom = has_valid_odometry_;
        uav_odom = last_odometry_;
    }
    if (fsm_state_snapshot != sunray_fsm::SunrayState::RETURN) {
        return_height_initialized_ = false;
    }
    // 根据当前状态机的状态，决定如何调用控制器的函数
    switch (fsm_state_snapshot) {
    // OFF状态:
    // sunray_fsm启动后的第一个状态，当控制器就绪后(is_ready()函数为true)切换为INIT状态，表示无人机可以正常进行任务
    case sunray_fsm::SunrayState::OFF: {
        break;
    }
    // INIT状态：init状态可以认为是每次任务开始前的第一个状态
    // 当接受到takeoff命令，我们从 init-> takeoff -> hover
    // 当接受到land命令，我们从 hover/move -> land -> init
    // 因此，从一次完整的飞行任务来看，我们认为，init状态应该保持为position模式，当任务开始，我们输出setpoint流并切换为offboard模式
    case sunray_fsm::SunrayState::INIT: {
        sunray_controller_->set_position_mode();
        break;
    }
    // 起飞阶段
    case sunray_fsm::SunrayState::TAKEOFF: {
        // 实际上takeoff函数是一个bool类型的函数
        sunray_controller_->takeoff(active_takeoff_relative_height_.load(),
                                    active_takeoff_max_velocity_.load());
        break;
    }
    // 悬停阶段
    case sunray_fsm::SunrayState::HOVER: {
        sunray_controller_->hover();
        break;
    }
    // 降落阶段
    case sunray_fsm::SunrayState::LAND: {
        const double land_velocity = effective_land_max_velocity(uav_control_cmd);
        active_land_max_velocity_.store(land_velocity);
        sunray_controller_->land(fsm_config_.takeoff_land_param.land_type,
                                 land_velocity);
        break;
    }
    // 返航阶段
    // TODO: 根据参数决定是否切换Land
    case sunray_fsm::SunrayState::RETURN: {
        if (!has_valid_odom) {
            break;
        }
        controller_data_types::TargetPoint_t home_target;
        home_target.position = home_point_;
        // RETURN 会高频重复进入，因此返航高度只在首次进入时锁定一次。
        // 如果每个周期都重写目标 z，返航路径会随当前高度漂移，无法形成稳定的 home 目标点。
        if (!return_height_initialized_) {
            home_point_.z() = uav_odom.position.z();
            return_height_initialized_ = true;
        }
        home_target.position.z() = home_point_.z();
        home_target.yaw = last_set_yaw_;
        sunray_controller_->move_point(home_target);
        // 如果到达了返航点,则根据参数配置,决定是降落还是切换为hover
        if (sunray_controller_->is_point_complete()) {
            if (fsm_config_.takeoff_land_param.return_with_land == true) {
                enqueue_fsm_event(sunray_fsm::SunrayEvent::LAND_REQUEST);
            } else {
                enqueue_fsm_event(sunray_fsm::SunrayEvent::RETURN_COMPLETED);
            }
        }
        break;
    }
    // 移动阶段
    case sunray_fsm::SunrayState::MOVE: {
        switch (uav_control_cmd.control_cmd) {
        case control_common::UavControlCmd::ControlCmd::MOVE_POINT: {
            controller_data_types::TargetPoint_t point;
            point.position = uav_control_cmd.position;

            if (uav_control_cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW) {
                point.yaw = uav_control_cmd.yaw;
            } else {
                point.yaw = last_set_yaw_;
            }

            sunray_controller_->move_point(point);
            break;
        }
        case control_common::UavControlCmd::ControlCmd::MOVE_POINT_BODY: {
            controller_data_types::TargetBodyPoint_t point;
            // 根据消息定义,我们认为xy坐标给的是body系,而z轴高度给的是惯性系
            point.position_xy = uav_control_cmd.body_position_xy;
            point.fixed_height = uav_control_cmd.fixed_height;
            // 同样存在对yaw角的判断,yaw角也是惯性系
            if (uav_control_cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW) {
                point.yaw = uav_control_cmd.yaw;
            } else {
                point.yaw = last_set_yaw_;
            }
            sunray_controller_->move_point_body(point);
            break;
        }
        case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY: {
            controller_data_types::TargetVelocity_t velocity;
            velocity.stamp = uav_control_cmd.timestamp;
            velocity.velocity = uav_control_cmd.velocity;
            velocity.fixed_height = uav_control_cmd.fixed_height;
            if (uav_control_cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW) {
                velocity.yaw = uav_control_cmd.yaw;
            } else if (uav_control_cmd.yaw_mode ==
                       control_common::UavControlCmd::YawMode::SET_YAWRATE) {
                velocity.yaw_rate = uav_control_cmd.yaw_rate;
            } else {
                velocity.yaw = last_set_yaw_;
            }
            sunray_controller_->move_velocity(velocity);
            break;
        }
        case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY_BODY: {
            controller_data_types::TargetBodyVelocity_t velocity;
            velocity.stamp = uav_control_cmd.timestamp;
            velocity.velocity_xy = uav_control_cmd.body_velocity_xy;
            velocity.fixed_height = uav_control_cmd.fixed_height;
            // yaw角相关为惯性系
            if (uav_control_cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW) {
                velocity.yaw = uav_control_cmd.yaw;
            } else if (uav_control_cmd.yaw_mode ==
                       control_common::UavControlCmd::YawMode::SET_YAWRATE) {
                velocity.yaw_rate = uav_control_cmd.yaw_rate;
            } else {
                velocity.yaw = last_set_yaw_;
            }
            sunray_controller_->move_velocity_body(velocity);
            break;
        }
        case control_common::UavControlCmd::ControlCmd::MOVE_TRAJECTORY: {
            controller_data_types::TargetTrajectoryPoint_t traj_point;
            traj_point.position = uav_control_cmd.position;
            traj_point.velocity = uav_control_cmd.velocity;
            traj_point.acceleration = uav_control_cmd.acceleration;
            traj_point.jerk = uav_control_cmd.jerk;
            if (uav_control_cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW) {
                traj_point.yaw = uav_control_cmd.yaw;
            } else if (uav_control_cmd.yaw_mode ==
                       control_common::UavControlCmd::YawMode::SET_YAWRATE) {
                traj_point.yaw_rate = uav_control_cmd.yaw_rate;
            } else {
                traj_point.yaw = last_set_yaw_;
            }
            sunray_controller_->move_trajectory(traj_point);
            break;
        }
        case control_common::UavControlCmd::ControlCmd::MOVE_POINT_WGS84: {
            break;
        }
            //
        }
        break;
    }
    // 紧急锁桨阶段
    case sunray_fsm::SunrayState::EMERGENCY_KILL: {
        sunray_controller_->emergency_kill();
        break;
    }
    }
}

// 控制器循环更新
void Sunray_FSM::controller_update_loop() {
    // 首先得到更新的频率
    const double hz = fsm_config_.basic_param.controller_update_frequency;
    ros::Rate rate(hz);
    ros::Time last_pushed_odom_stamp(0);
    // 循环
    while (ros::ok() && !stop_controller_thread_.load(std::memory_order_relaxed)) {
        control_common::UAVStateEstimate odom_snapshot;
        control_common::UAVStateEstimate raw_odom_snapshot;
        const bool has_valid_odom = get_latest_valid_odometry(odom_snapshot);
        if (has_valid_odom && odom_snapshot.timestamp != last_pushed_odom_stamp) {
            {
                std::lock_guard<std::mutex> lk(odom_mutex_);
                raw_odom_snapshot = last_raw_odometry_;
            }
            sunray_controller_->set_current_odom(odom_snapshot);
            sunray_controller_->set_external_odom_for_fusion(raw_odom_snapshot);
            last_pushed_odom_stamp = odom_snapshot.timestamp;
        }
        update_controller_output();
        rate.sleep();
    }
}

// -----------------------状态机辅助函数-----------------

// 更新home点,用于 INIT -> Takeoff
bool Sunray_FSM::update_home_point() {
    // 打一份里程计快照?
    control_common::UAVStateEstimate odom_snapshot;
    if (!get_latest_valid_odometry(odom_snapshot)) {
        return false;
    }
    // 更新home点
    home_point_ = odom_snapshot.position;
    control_common::UavControlCmd cmd_snapshot;
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        cmd_snapshot = last_control_cmd_;
    }
    double takeoff_height = fsm_config_.takeoff_land_param.takeoff_relative_height;
    double takeoff_velocity = fsm_config_.takeoff_land_param.takeoff_max_velocity;
    // update_home_point 在 TAKEOFF_REQUEST 转移 action 中调用，此时 guard 已完成校验；
    // 这里若仍解析失败，则回退到 config 当前值，避免 action 失败打断合法起飞。
    (void)resolve_takeoff_command_params(cmd_snapshot, &takeoff_height, &takeoff_velocity, nullptr);
    active_takeoff_relative_height_.store(takeoff_height);
    active_takeoff_max_velocity_.store(takeoff_velocity);
    home_point_.z() = takeoff_height;
    return true;
}

bool Sunray_FSM::resolve_takeoff_command_params(const control_common::UavControlCmd& cmd,
                                                double* relative_takeoff_height,
                                                double* max_takeoff_velocity,
                                                std::string* reject_reason) const {
    auto set_reject_reason = [reject_reason](const std::string& reason) {
        if (reject_reason != nullptr) {
            *reject_reason = reason;
        }
    };
    if (relative_takeoff_height == nullptr || max_takeoff_velocity == nullptr) {
        set_reject_reason("internal error: null takeoff parameter output");
        return false;
    }

    double resolved_height = fsm_config_.takeoff_land_param.takeoff_relative_height;
    double resolved_velocity = fsm_config_.takeoff_land_param.takeoff_max_velocity;
    const bool is_takeoff_cmd =
        cmd.control_cmd == control_common::UavControlCmd::ControlCmd::TAKEOFF;

    if (is_takeoff_cmd && std::isfinite(cmd.takeoff_relative_height) &&
        cmd.takeoff_relative_height > 0.0) {
        resolved_height = cmd.takeoff_relative_height;
    }
    if (!std::isfinite(resolved_height) || resolved_height <= 0.0) {
        set_reject_reason("takeoff_relative_height must be finite and > 0");
        return false;
    }

    const double max_takeoff_height =
        std::max(0.0, fsm_config_.local_fence_param.z_max - fsm_config_.local_fence_param.z_min);
    if (max_takeoff_height <= 0.0) {
        set_reject_reason("invalid local_fence z range, cannot derive takeoff height clamp");
        return false;
    }
    resolved_height = std::clamp(resolved_height, 0.0, max_takeoff_height);
    if (resolved_height <= 0.0) {
        set_reject_reason("takeoff_relative_height clamp result must be > 0");
        return false;
    }

    if (cmd.control_cmd == control_common::UavControlCmd::ControlCmd::TAKEOFF &&
        std::isfinite(cmd.takeoff_max_velocity) &&
        cmd.takeoff_max_velocity > 0.0) {
        resolved_velocity = cmd.takeoff_max_velocity;
    }
    if (!std::isfinite(resolved_velocity) || resolved_velocity <= 0.0) {
        set_reject_reason("takeoff_max_velocity must be finite and > 0");
        return false;
    }

    const double max_vertical_velocity = fsm_config_.velocity_param.max_velocity.z();
    if (!std::isfinite(max_vertical_velocity) || max_vertical_velocity <= 0.0) {
        set_reject_reason("velocity_param.max_velocity.z must be finite and > 0");
        return false;
    }
    resolved_velocity = std::clamp(resolved_velocity, 0.0, max_vertical_velocity);
    if (resolved_velocity <= 0.0) {
        set_reject_reason("takeoff_max_velocity clamp result must be > 0");
        return false;
    }

    *relative_takeoff_height = resolved_height;
    *max_takeoff_velocity = resolved_velocity;
    return true;
}

double Sunray_FSM::effective_land_max_velocity(
    const control_common::UavControlCmd& cmd) const {
    if (cmd.control_cmd == control_common::UavControlCmd::ControlCmd::LAND &&
        std::isfinite(cmd.land_max_velocity) &&
        cmd.land_max_velocity > 0.0) {
        return cmd.land_max_velocity;
    }
    return fsm_config_.takeoff_land_param.land_max_velocity;
}

// 设置Hover点，用于 Move -> Hover
bool Sunray_FSM::set_hover_point_from_latest_odom() {
    control_common::UAVStateEstimate odom_snapshot;
    if (!get_latest_valid_odometry(odom_snapshot)) {
        return false;
    }
    return sunray_controller_->set_hover_point(odom_snapshot);
}

// POINT_COMPLETED 转移专用：将悬停点锁定到最近一次 move_point 目标，
// 避免到达判断时刻的位置误差被作为悬停点。控制器没有可用目标时回退到里程计。
bool Sunray_FSM::set_hover_point_to_target_or_odom() {
    if (sunray_controller_->set_hover_point_to_last_target()) {
        return true;
    }
    return set_hover_point_from_latest_odom();
}
