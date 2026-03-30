/**
 * 构造本文件的目的在于对自定义话题uav_control_cmd.msg进行解析，通过构造一个结构体来填充数据
 */
#pragma once

#include <cstdint>
#include <Eigen/Dense>
#include <geographic_msgs/GeoPoint.h>
namespace control_common {
struct UavControlCmd {
    enum class CmdSource : uint8_t {
        SUNRAY_STATION = 0,  // Sunray地面站控制
        RC_CONTROLLER = 1,   // RC遥控器控制
        TERMINAL = 2,        // 终端控制
        CONTROL_CMD = 3,     // 控制命令(所有从代码中实现的控制)
        UNDEFINE = 4
    };
    enum class ControlCmd : uint8_t {
        TAKEOFF = 0,             // [特殊]起飞指令
        LAND = 1,                // [特殊]降落指令
        RETURN = 2,              // [特殊]返航指令
        KILL = 3,                // [特殊]锁桨指令
        HOVER = 4,               // [特殊]悬停指令：在当前位置悬停
        MOVE_POINT = 5,          // 移动指令：移动到某一点
        MOVE_VELOCITY = 6,       // 速度指令：控制无人机的速度
        MOVE_TRAJECTORY = 7,     // 轨迹指令：跟踪轨迹
        MOVE_POINT_BODY = 8,     // 移动指令：移动到机体系下的某一点
        MOVE_VELOCITY_BODY = 9,  // 速度指令：控制无人机在机体系下的速度
        MOVE_POINT_WGS84 = 10,   // 移动指令：移动到经纬高系下的某一点
        UNDEFINE = 11
    };
    enum class YawMode : uint8_t {
        SET_YAW = 0,      // 设置yaw角
        KEEP_YAW = 1,     // 保持当前yaw角不变
        SET_YAWRATE = 2,  // 设置yaw角的角速度
        UNDEFINE = 3
    };

    CmdSource cmd_source{CmdSource::UNDEFINE};     // 控制指令来源
    ControlCmd control_cmd{ControlCmd::UNDEFINE};  // 控制指令类型
    // ---------------根据ControlCmd决定msg中的数据如何填充---------------------
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
    Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d attitude{Eigen::Vector3d::Zero()};
    // ------------这个特殊一点，经纬高---------------
    Eigen::Vector3d wgs84_position{Eigen::Vector3d::Zero()};
    // ------------- yaw角控制模式---------------
    YawMode yaw_mode{YawMode::UNDEFINE};
    double yaw{0.0};
    double yaw_rate{0.0};
    // ---------------是否固定为当前高度----------------
    bool fixed_height{false};
};

};  // namespace control_common
