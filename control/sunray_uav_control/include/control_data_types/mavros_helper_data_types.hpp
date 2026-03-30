// 设计意图，为Mavros_Helper类提供数据类型
#pragma once

#include <cstdint>
#include <Eigen/Dense>
#include <ros/time.h>

namespace control_common {

// clang-format off
// 由于clang-format 破坏了下面两个枚举排版上的感觉，所以这里禁用格式化，提高代码可读性

// FlightMode与Mavros消息定义相对应，主要是强类型枚举飞控当前的状态
// 这里不使用常量开头的k，因为我们希望这里与mavros形成对应关系
enum class FlightMode : uint8_t {
    Undefined =0,  // 这里存在一个未定义作为初始值，并不在mavros的消息定义里面，但是我们为了在没有连接到mavros时也能成功初始化且不带来语义上的疑惑，所以引入了Undefined
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
    AutoTakeoff
};

// LandedState 与mavros消息定义对应，用于表示px4的着地检测器状态
enum class LandedState : uint8_t {
    Undefined = 0,
    OnGround,
    InAir,
    Takeoff,
    Landing
};

enum class VisionFuseType : uint8_t{
    Undefined = 0,
    Vision_pose,
    Odometry
};

// clang-format on

struct Mavros_State {  // Mavros_State表示这个类型的数据是从mavros中与state相关的话题中获取的
    bool connected = false;  // connected表示px4是否响应了mavros的心跳包，表示px4是否连接
    bool armed = false;         // 表示px4是否解锁
    bool rc_input = false;      // 表示是否有rc遥控器的输入
    uint8_t system_status = 0;  // 表示系统状态?似乎这里和GPS也有一些关系
    float system_load = 0;      // 当前飞控的cpu负载
    float voltage = 0.0f;       // 当前电池电压
    float current = 0.0f;       // 当前电池电流
    float percent = 0.0f;       // 当前电池百分比
    FlightMode flight_mode = FlightMode::Undefined;     // 当前飞控模式
    LandedState landed_state = LandedState::Undefined;  // 当前飞控的着地检测器状态
    ros::Time timestamp = ros::Time(0);
    bool valid = false;
};

struct Mavros_Estimator {  // Mavros_Estimator表示这个类型是从mavros的estimator话题中获取的
    bool attitude_valid = false;         // 表示当前无人机姿态估计有效
    bool local_hroiz_valid = false;      // 表示当前无人机水平位置估计有效
    bool local_vertical_valid = false;   // 表示当前无人机高度位置估计有效
    bool global_hroiz_valid = false;     // 表示当前无人机全球经纬位置估计有效
    bool global_vertical_valid = false;  // 表示当前无人机的全球高度位置估计有效
    bool gps_error = false;              // 表示当前gps出现错误
    bool acc_error = false;              // 表示当前加速度计出现错误
    ros::Time timestamp = ros::Time(0);
    bool valid = false;
};

struct Mavros_Pose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

struct Mavros_Velocity {
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
};

struct Mavros_GPS {
    uint8_t satellites;
    int8_t gps_status;
    uint8_t gps_service;
    double latitude;
    double longitude;
    double altitude;
    double latitude_raw;
    double longitude_raw;
    double altitude_amsl;
};

// 姿态数据使用Eigen::Vector3d返回欧拉角，使用Eigen::Quaternion返回四元数,设计两个函数，使用后缀区分

struct Mavros_SetpointLocal {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum class Mavros_LocalFrame : uint8_t {
        Local_Ned = 1,
        Local_Offset_Ned = 7,
        Body_Ned = 8,
        Body_Offset_Ned = 9
    };
    // 与 MAVROS PositionTarget::type_mask 对齐。
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

    // setpoint 中位置/速度/加速度字段的解释坐标系。
    Mavros_LocalFrame frame = Mavros_LocalFrame::Local_Ned;
    // 指定哪些字段参与控制，哪些字段被忽略。
    uint16_t mask = 0;

    // 目标位置。
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    // 目标速度。
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    // 目标加速度，或在 `kForceSetpoint` 置位时表示目标力。
    Eigen::Vector3d accel_or_force = Eigen::Vector3d::Zero();

    // 目标偏航角。
    double yaw = 0.0;
    // 目标偏航角速度。
    double yaw_rate = 0.0;
    ros::Time timestamp = ros::Time(0);
    bool valid = false;
};

struct Mavros_SetpointAttitude {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 与 MAVROS AttitudeTarget::type_mask 对齐。
    enum Mask : uint8_t {
        IgnoreRollRate = 1u,
        IgnorePitchRate = 2u,
        IgnoreYawRate = 4u,
        IgnoreThrust = 64u,
        IgnoreAttitude = 128u
    };

    // 指定姿态、角速度、推力中哪些字段参与控制。
    uint8_t mask = 0;

    // 目标姿态。
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    // 机体系目标角速度。
    Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
    // 归一化总推力。
    double thrust = 0.0;
    ros::Time timestamp = ros::Time(0);
    bool valid = false;
};

};  // namespace control_common
