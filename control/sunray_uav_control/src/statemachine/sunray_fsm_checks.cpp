#include "statemachine/sunray_fsm.hpp"
#include <ros/ros.h>

namespace {

bool finite_vec3(const Eigen::Vector3d& value) {
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

}  // namespace


// 检查controller是否就绪
void Sunray_FSM::check_controller_ready() {
    // 仅在OFF状态下才判断
    if (fsm_state_ != sunray_fsm::SunrayState::OFF) {
        return;
    }

    bool odom_ready = false;
    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        odom_ready = has_valid_odometry_ &&
                     (ros::Time::now() - last_odometry_.timestamp).toSec() <=
                         fsm_config_.msg_timeout_param.local_odometry;
    }

    controller_ready_ = odom_ready && sunray_controller_->is_ready();
    if (controller_ready_) {
        enqueue_fsm_event(sunray_fsm::SunrayEvent::CONTROLLER_READY);
    }
}

// 检查是否允许起飞
bool Sunray_FSM::check_allow_takeoff() {
    // protect_param 已迁移给 system_check，当前 FSM 不再承担起飞授权策略。
    // 在 system_check 接管前，这里仅保持最小行为：控制器就绪后允许 TAKEOFF 转移。
    allow_takeoff_ = true;
    return allow_takeoff_;
}

// 检查是否存在消息超时的情况
void Sunray_FSM::check_rosmsg_timeout() {
    // 根据输入检查。我们一共配置了四个回调函数，分别是 外部里程计回调函数,
    // 外部里程计状态回调函数，无人机控制指令回调函数，系统状态检查回调函数
    // 由于外部里程计状态目前没有使用(原先设计的使用外部里程计状态配置EKF2参数等部分，我们认为需要从fsm中拆出去）
    // 状态检查部分由于没有实现完也没有使用，所以这里只需要配置外部里程计回调函数和无人机控制指令回调函数
    ros::Time now = ros::Time::now();
    // 打份快照
    control_common::UAVStateEstimate odom_snapshot;
    control_common::UavControlCmd last_cmd_snapshot;
    {
        std::lock_guard<std::mutex> lk1(odom_mutex_);
        odom_snapshot = last_odometry_;
    }
    {
        std::lock_guard<std::mutex> lk2(cmd_mutex_);
        last_cmd_snapshot = last_control_cmd_;
    }

    if (odom_snapshot.timestamp != ros::Time(0) &&
        (now - odom_snapshot.timestamp).toSec() > fsm_config_.msg_timeout_param.local_odometry) {
        // enqueue_fsm_event(sunray_fsm::SunrayEvent::KILL_REQUEST);
        // 里程计超时，切KILL？其实并不保险我个人认为
    }
    if (last_cmd_snapshot.timestamp != ros::Time(0) &&
        (now - last_cmd_snapshot.timestamp).toSec() > 0.2) {
        control_msg_lost_ = true;
    } else {
        control_msg_lost_ = false;
    }
}
void Sunray_FSM::check_move_completed() {
    // 我们在这个函数中对运动状态进行判断，主要分为两类
    // 1. takeoff() land() 对起飞/降落结果的判断
    // 2. is_point_complete() 对move point是否完成的判断
    // 3. 控制指令流是否停止

    // 首先我们对起飞/降落是否完成进行判断，前提是处于对应的模式
    sunray_fsm::SunrayState current_state;  // 先打一份快照
    {
        std::lock_guard<std::mutex> lk1(state_mutex_);
        current_state = fsm_state_;
    }
    if (current_state == sunray_fsm::SunrayState::TAKEOFF) {
        bool takeoff_state = sunray_controller_->is_takeoff_complete();
        if (takeoff_state) {
            enqueue_fsm_event(sunray_fsm::SunrayEvent::TAKEOFF_COMPLETED);
        }
    } else if (current_state == sunray_fsm::SunrayState::LAND) {
        bool land_state = sunray_controller_->is_land_complete();
        if (land_state) {
            enqueue_fsm_event(sunray_fsm::SunrayEvent::LAND_COMPLETED);
        }
    }
    // 对运动是否完成的判断，存在前提条件： control_cmd没有保持10Hz的发布频率
    if (!control_msg_lost_) {
        return;
    }
    // 如果当前不在move状态，则退出
    if (current_state != sunray_fsm::SunrayState::MOVE) {
        return;
    }
    // 给control_cmd打一份快照
    control_common::UavControlCmd current_cmd;
    {
        std::lock_guard<std::mutex> lk1(cmd_mutex_);
        current_cmd = last_control_cmd_;
    }
    // 其中，只有move_point / move_point_body/ move_point_wgs84
    // 需要判断是否到达位置，剩余的直接切换为hover
    if (current_cmd.control_cmd == control_common::UavControlCmd::ControlCmd::MOVE_POINT ||
        current_cmd.control_cmd == control_common::UavControlCmd::ControlCmd::MOVE_POINT_BODY ||
        current_cmd.control_cmd == control_common::UavControlCmd::ControlCmd::MOVE_POINT_WGS84) {
        bool move_state = sunray_controller_->is_point_complete();
        if (move_state) {
            enqueue_fsm_event(sunray_fsm::SunrayEvent::POINT_COMPLETED);
        }
    } else {
        enqueue_fsm_event(sunray_fsm::SunrayEvent::HOVER_REQUEST);
    }
}

bool Sunray_FSM::validate_odometry_sample(const control_common::UAVStateEstimate& odom,
                                          std::string* invalid_reason) const {
    auto set_invalid_reason = [invalid_reason](const char* reason) {
        if (invalid_reason != nullptr) {
            *invalid_reason = reason;
        }
    };

    if (odom.timestamp.isZero()) {
        set_invalid_reason("timestamp is zero");
        return false;
    }
    if (!finite_vec3(odom.position)) {
        set_invalid_reason("position is not finite");
        return false;
    }
    if (!finite_vec3(odom.velocity)) {
        set_invalid_reason("velocity is not finite");
        return false;
    }
    if (!finite_vec3(odom.bodyrate)) {
        set_invalid_reason("bodyrate is not finite");
        return false;
    }

    if (!std::isfinite(odom.orientation.w()) || !std::isfinite(odom.orientation.x()) ||
        !std::isfinite(odom.orientation.y()) || !std::isfinite(odom.orientation.z())) {
        set_invalid_reason("orientation is not finite");
        return false;
    }

    const double quaternion_norm =
        std::sqrt(odom.orientation.w() * odom.orientation.w() +
                  odom.orientation.x() * odom.orientation.x() +
                  odom.orientation.y() * odom.orientation.y() +
                  odom.orientation.z() * odom.orientation.z());
    if (quaternion_norm < 1e-6) {
        set_invalid_reason("orientation norm is degenerate");
        return false;
    }
    if (std::fabs(quaternion_norm - 1.0) > 0.2) {
        set_invalid_reason("orientation norm deviates from unit quaternion");
        return false;
    }
    return true;
}

bool Sunray_FSM::get_latest_valid_odometry(control_common::UAVStateEstimate& odom) const {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    if (!has_valid_odometry_) {
        return false;
    }
    odom = last_odometry_;
    return true;
}
