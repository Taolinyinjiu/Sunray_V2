// 设计意图，为Mavros_Helper类提供数据类型
#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <ros/time.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SysStatus.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/Imu.h>

namespace control_common {

// clang-format off

enum class FlightMode : uint8_t {
    Undefined =0,
    Manual,
    Acro,
    Altctl,
    Posctl,
    Offboard,
    Stabilized,
    Rattitude,
    AutoMission,
    AutoLoiter,
    AutoRtl,
    AutoLand,
    AutoRtgs,
    AutoReady,
    AutoTakeoff,
    Raptor
};

enum class LandedState : uint8_t {
    Undefined = 0,
    OnGround,
    InAir,
    Takeoff,
    Landing
};

std::string flightmode_to_string(control_common::FlightMode flight);
std::string landed_to_string(control_common::LandedState land_state);

enum class VisionFuseType : uint8_t{
    Undefined = 0,
    Vision_pose,
    Odometry
};

// clang-format on

struct Mavros_State {
    bool connected = false;
    bool armed = false;
    bool rc_input = false;
    uint8_t system_status = 0;
    float system_load = 0;
    float voltage = 0.0f;
    float current = 0.0f;
    float percent = 0.0f;
    FlightMode flight_mode = FlightMode::Undefined;
    LandedState landed_state = LandedState::Undefined;
    ros::Time timestamp = ros::Time(0);
    bool valid = false;

    void update_from(const mavros_msgs::State& msg, const std::string& raptor_cmode = "");
    void update_from(const mavros_msgs::ExtendedState& msg);
    void update_from(const mavros_msgs::SysStatus& msg);
};

struct Mavros_Estimator {
    bool attitude_valid = false;
    bool local_hroiz_valid = false;
    bool local_vertical_valid = false;
    bool global_hroiz_valid = false;
    bool global_vertical_valid = false;
    bool gps_error = false;
    bool acc_error = false;
    ros::Time timestamp = ros::Time(0);
    bool valid = false;

    Mavros_Estimator() = default;
    Mavros_Estimator(const mavros_msgs::EstimatorStatus& msg);
};

struct Mavros_Pose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct Mavros_Velocity {
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
};

struct Mavros_GPS {
    uint8_t satellites = 0;
    int8_t gps_status = 0;   // fix_type as signed (兼容旧接口)
    uint8_t gps_service = 0; // fix_type as unsigned (兼容 Px4State msg)
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double latitude_raw = 0.0;
    double longitude_raw = 0.0;
    double altitude_amsl = 0.0;

    Mavros_GPS() = default;
    Mavros_GPS(const mavros_msgs::GPSRAW& msg);
};

struct Mavros_IMU {
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelection = Eigen::Vector3d::Zero();
    ros::Time stamp = ros::Time(0);

    Mavros_IMU() = default;
    Mavros_IMU(const sensor_msgs::Imu& msg);
};

struct Mavros_RC {
    std::vector<uint16_t> channels;
    uint8_t rssi = 0;
    ros::Time timestamp = ros::Time(0);
    bool valid = false;

    Mavros_RC() = default;
    Mavros_RC(const mavros_msgs::RCIn& msg);
};

struct Mavros_SetpointLocal {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum class Mavros_LocalFrame : uint8_t {
        Local_Ned = 1,
        Local_Offset_Ned = 7,
        Body_Ned = 8,
        Body_Offset_Ned = 9
    };

    enum Mask : uint16_t {
        IgnorePx = 1u,
        IgnorePy = 2u,
        IgnorePz = 4u,
        IgnoreVx = 8u,
        IgnoreVy = 16u,
        IgnoreVz = 32u,
        IgnoreAfx = 64u,
        IgnoreAfy = 128u,
        IgnoreAfz = 256u,
        ForceSetpoint = 512u,
        IgnoreYaw = 1024u,
        IgnoreYawRate = 2048u
    };

    Mavros_LocalFrame frame = Mavros_LocalFrame::Local_Ned;
    uint16_t mask = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_or_force = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double yaw_rate = 0.0;
    ros::Time timestamp = ros::Time(0);
    bool valid = false;

    Mavros_SetpointLocal() = default;
    Mavros_SetpointLocal(const mavros_msgs::PositionTarget& msg);
};

struct Mavros_SetpointAttitude {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum Mask : uint8_t {
        IgnoreRollRate = 1u,
        IgnorePitchRate = 2u,
        IgnoreYawRate = 4u,
        IgnoreThrust = 64u,
        IgnoreAttitude = 128u
    };

    uint8_t mask = 0;
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
    double thrust = 0.0;
    ros::Time timestamp = ros::Time(0);
    bool valid = false;

    Mavros_SetpointAttitude() = default;
    Mavros_SetpointAttitude(const mavros_msgs::AttitudeTarget& msg);
};

// ======================== inline 实现 ========================

inline void Mavros_State::update_from(const mavros_msgs::State& msg, const std::string& raptor_cmode) {
    timestamp = msg.header.stamp;
    connected = msg.connected;
    armed = msg.armed;
    rc_input = msg.manual_input;
    system_status = msg.system_status;

    if (msg.mode == mavros_msgs::State::MODE_PX4_MANUAL) {
        flight_mode = FlightMode::Manual;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_ACRO) {
        flight_mode = FlightMode::Acro;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_ALTITUDE) {
        flight_mode = FlightMode::Altctl;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_POSITION) {
        flight_mode = FlightMode::Posctl;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_OFFBOARD) {
        flight_mode = FlightMode::Offboard;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_STABILIZED) {
        flight_mode = FlightMode::Stabilized;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_RATTITUDE) {
        flight_mode = FlightMode::Rattitude;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_MISSION) {
        flight_mode = FlightMode::AutoMission;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_LOITER) {
        flight_mode = FlightMode::AutoLoiter;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_RTL) {
        flight_mode = FlightMode::AutoRtl;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_LAND) {
        flight_mode = FlightMode::AutoLand;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_RTGS) {
        flight_mode = FlightMode::AutoRtgs;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_READY) {
        flight_mode = FlightMode::AutoReady;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_TAKEOFF) {
        flight_mode = FlightMode::AutoTakeoff;
    } else if (!raptor_cmode.empty() && msg.mode == raptor_cmode) {
        flight_mode = FlightMode::Raptor;
    } else {
        flight_mode = FlightMode::Undefined;
    }
}

inline void Mavros_State::update_from(const mavros_msgs::ExtendedState& msg) {
    timestamp = msg.header.stamp;
    switch (msg.landed_state) {
    case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND:
        landed_state = LandedState::OnGround;
        break;
    case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR:
        landed_state = LandedState::InAir;
        break;
    case mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF:
        landed_state = LandedState::Takeoff;
        break;
    case mavros_msgs::ExtendedState::LANDED_STATE_LANDING:
        landed_state = LandedState::Landing;
        break;
    default:
        landed_state = LandedState::Undefined;
        break;
    }
}

inline void Mavros_State::update_from(const mavros_msgs::SysStatus& msg) {
    timestamp = msg.header.stamp;
    system_load = static_cast<float>(msg.load) / 1000.0f;
    if (msg.voltage_battery != UINT16_MAX) {
        voltage = static_cast<float>(msg.voltage_battery) / 1000.0f;
    }
    if (msg.current_battery != -1) {
        current = static_cast<float>(msg.current_battery) / 1000.0f;
    }
    if (msg.battery_remaining != -1) {
        percent = static_cast<float>(msg.battery_remaining) / 100.0f;
    }
}

inline Mavros_Estimator::Mavros_Estimator(const mavros_msgs::EstimatorStatus& msg) {
    timestamp = msg.header.stamp;
    attitude_valid = msg.attitude_status_flag;
    local_hroiz_valid = msg.velocity_horiz_status_flag && msg.pos_horiz_rel_status_flag;
    local_vertical_valid = msg.velocity_vert_status_flag &&
                           msg.pos_vert_abs_status_flag &&
                           msg.pos_vert_agl_status_flag;
    global_hroiz_valid = msg.pos_horiz_abs_status_flag && msg.pred_pos_horiz_abs_status_flag;
    global_vertical_valid = msg.pos_vert_abs_status_flag;
    gps_error = msg.gps_glitch_status_flag;
    acc_error = msg.accel_error_status_flag;
    valid = true;
}

inline Mavros_GPS::Mavros_GPS(const mavros_msgs::GPSRAW& msg) {
    satellites = msg.satellites_visible;
    gps_status = static_cast<int8_t>(msg.fix_type);
    gps_service = msg.fix_type;
    latitude = static_cast<double>(msg.lat) * 1e-7;
    longitude = static_cast<double>(msg.lon) * 1e-7;
    altitude = static_cast<double>(msg.alt) * 1e-3;
    latitude_raw = static_cast<double>(msg.lat);
    longitude_raw = static_cast<double>(msg.lon);
    altitude_amsl = static_cast<double>(msg.alt) * 1e-3;
}

inline Mavros_IMU::Mavros_IMU(const sensor_msgs::Imu& msg) {
    stamp = msg.header.stamp;
    orientation.w() = msg.orientation.w;
    orientation.x() = msg.orientation.x;
    orientation.y() = msg.orientation.y;
    orientation.z() = msg.orientation.z;
    accelection.x() = msg.linear_acceleration.x;
    accelection.y() = msg.linear_acceleration.y;
    accelection.z() = msg.linear_acceleration.z;
    bodyrate.x() = msg.angular_velocity.x;
    bodyrate.y() = msg.angular_velocity.y;
    bodyrate.z() = msg.angular_velocity.z;
}

inline Mavros_RC::Mavros_RC(const mavros_msgs::RCIn& msg) {
    timestamp = msg.header.stamp;
    channels = msg.channels;
    rssi = msg.rssi;
    valid = true;
}

inline Mavros_SetpointLocal::Mavros_SetpointLocal(const mavros_msgs::PositionTarget& msg) {
    timestamp = msg.header.stamp;
    valid = true;
    switch (msg.coordinate_frame) {
    case mavros_msgs::PositionTarget::FRAME_LOCAL_NED:
        frame = Mavros_LocalFrame::Local_Ned;
        break;
    case mavros_msgs::PositionTarget::FRAME_LOCAL_OFFSET_NED:
        frame = Mavros_LocalFrame::Local_Offset_Ned;
        break;
    case mavros_msgs::PositionTarget::FRAME_BODY_NED:
        frame = Mavros_LocalFrame::Body_Ned;
        break;
    case mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED:
        frame = Mavros_LocalFrame::Body_Offset_Ned;
        break;
    default:
        break;
    }
    mask = msg.type_mask;
    position.x() = msg.position.x;
    position.y() = msg.position.y;
    position.z() = msg.position.z;
    velocity.x() = msg.velocity.x;
    velocity.y() = msg.velocity.y;
    velocity.z() = msg.velocity.z;
    accel_or_force.x() = msg.acceleration_or_force.x;
    accel_or_force.y() = msg.acceleration_or_force.y;
    accel_or_force.z() = msg.acceleration_or_force.z;
    yaw = msg.yaw;
    yaw_rate = msg.yaw_rate;
}

inline Mavros_SetpointAttitude::Mavros_SetpointAttitude(const mavros_msgs::AttitudeTarget& msg) {
    timestamp = msg.header.stamp;
    valid = true;
    mask = msg.type_mask;
    orientation.x() = msg.orientation.x;
    orientation.y() = msg.orientation.y;
    orientation.z() = msg.orientation.z;
    orientation.w() = msg.orientation.w;
    body_rate.x() = msg.body_rate.x;
    body_rate.y() = msg.body_rate.y;
    body_rate.z() = msg.body_rate.z;
    thrust = msg.thrust;
}

};  // namespace control_common
