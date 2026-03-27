#include "controller/base_controller.hpp"
#include "sunray_msgs/Px4State.h"

namespace uav_control {
void BaseController::set_current_odom(const control_common::UAVStateEstimate& odom) {
    uav_odometry_ = control_common::UAVStateEstimate(odom);
}

void BaseController::set_fuse_frequency(double fuse_odom_frequency) {
    fuse_odom_frequency_ = fuse_odom_frequency;
}

std::string flightmode_to_string(control_common::FlightMode flight) {
    switch (flight) {
    case control_common::FlightMode::Undefined:
        return "UNDEFINED";
    case control_common::FlightMode::Manual:
        return "MANUAL";
    case control_common::FlightMode::Acro:
        return "ACRO";
    case control_common::FlightMode::Altctl:
        return "ALTCTL";
    case control_common::FlightMode::Posctl:
        return "POSCTL";
    case control_common::FlightMode::Offboard:
        return "OFFBOARD";
    case control_common::FlightMode::Stabilized:
        return "STABILIZED";
    case control_common::FlightMode::Rattitude:
        return "RATTITUDE";
    case control_common::FlightMode::AutoMission:
        return "AUTO.MISSION";
    case control_common::FlightMode::AutoLoiter:
        return "AUTO.LOITER";
    case control_common::FlightMode::AutoRtl:
        return "AUTO.RTL";
    case control_common::FlightMode::AutoLand:
        return "AUTO.LAND";
    case control_common::FlightMode::AutoRtgs:
        return "AUTO.RTGS";
    case control_common::FlightMode::AutoReady:
        return "AUTO.READY";
    case control_common::FlightMode::AutoTakeoff:
        return "AUTO.TAKEOFF";
    default:
        return "UNKNOWN";
    }
}

std::string landed_to_string(control_common::LandedState land_state) {
    switch (land_state) {
    case control_common::LandedState::Undefined:
        return "UNDEFINED";
    case control_common::LandedState::OnGround:
        return "ON_GROUND";
    case control_common::LandedState::InAir:
        return "IN_AIR";
    case control_common::LandedState::Takeoff:
        return "TAKEOFF";
    case control_common::LandedState::Landing:
        return "LANDING";
    }
    return "UNKNOWN";
}

void BaseController::publish_timer_cb(const ros::TimerEvent&) {
    // 发布数据相关定时器,首先发布Px4State
    sunray_msgs::Px4State px4_state_msg;
    // 填充时间戳
    px4_state_msg.header.stamp = ros::Time::now();
    // 获取mavros_helper的数据
    control_common::Mavros_State mavros_state;
    mavros_state = mavros_helper_.get_state();
    // 这部分从mavros/state获取数据
    px4_state_msg.connected = mavros_state.connected;
    px4_state_msg.rc_available = mavros_state.rc_input;
    px4_state_msg.armed = mavros_state.armed;
    px4_state_msg.flight_mode = static_cast<uint8_t>(mavros_state.flight_mode);
    px4_state_msg.flight_mode_name = flightmode_to_string(mavros_state.flight_mode);
    px4_state_msg.system_status = mavros_state.system_status;
    // 这部分从mavros/extended_state获取数据
    px4_state_msg.landed_state = static_cast<uint8_t>(mavros_state.landed_state);
    px4_state_msg.landed_state_name = landed_to_string(mavros_state.landed_state);
    // 这部分从mavros/sys_status获取数据
    px4_state_msg.battery_voltage_v = mavros_state.voltage;
    px4_state_msg.battery_current_a = mavros_state.current;
    px4_state_msg.battery_percentage = mavros_state.percent;
    px4_state_msg.fcu_load = mavros_state.system_load;
    // 填充vision_pose相关
    if (fuse_odom_frequency_ != 0) {
        px4_state_msg.enable_vision_fuse = true;
    } else {
        px4_state_msg.enable_vision_fuse = false;
    }
    px4_state_msg.vision_pose = vision_pose_;
}

}  // namespace uav_control
