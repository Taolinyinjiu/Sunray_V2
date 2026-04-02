/**
 * 构造本文件的目的在于对自定义话题uav_control_cmd.msg进行解析，通过构造一个结构体来填充数据
 */
#pragma once

#include <cstdint>
#include <Eigen/Dense>
#include <geographic_msgs/GeoPoint.h>
#include <sunray_msgs/UAVControlCMD.h>
namespace control_common {
struct UavControlCmd {
    enum class CmdSource : uint8_t {
        UNDEFINE = 0,    // 未定义的来源
        SUNRAY_STATION,  // Sunray地面站控制
        RC_CONTROLLER,   // RC遥控器控制
        TERMINAL,        // 终端控制
        CONTROL_CMD      // 控制命令(所有从代码中实现的控制)
    };
    enum class ControlCmd : uint8_t {
        UNDEFINE = 0,        //	未知指令
        TAKEOFF,             // [特殊]起飞指令
        LAND,                // [特殊]降落指令
        RETURN,              // [特殊]返航指令
        KILL,                // [特殊]锁桨指令
        HOVER,               // [特殊]悬停指令：在当前位置悬停
        MOVE_POINT,          // 移动指令：移动到某一点
        MOVE_VELOCITY,       // 速度指令：控制无人机的速度
        MOVE_TRAJECTORY,     // 轨迹指令：跟踪轨迹
        MOVE_POINT_BODY,     // 移动指令：移动到机体系下的某一点
        MOVE_VELOCITY_BODY,  // 速度指令：控制无人机在机体系下的速度
        MOVE_POINT_WGS84     // 移动指令：移动到经纬高系下的某一点

    };
    enum class YawMode : uint8_t {
        KEEP_YAW = 0,     // 保持当前yaw角不变
        SET_YAW = 1,      // 设置yaw角
        SET_YAWRATE = 2,  // 设置yaw角的角速度
    };

    CmdSource cmd_source{CmdSource::UNDEFINE};     // 控制指令来源
    ControlCmd control_cmd{ControlCmd::UNDEFINE};  // 控制指令类型
    // ---------------根据ControlCmd决定msg中的数据如何填充---------------------
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
    // ------------这个特殊一点，经纬高---------------
    // wgs84_position = [latitude_deg, longitude_deg, altitude_m]
    geographic_msgs::GeoPoint wgs84_position;
		// ------------- yaw角控制模式---------------
    YawMode yaw_mode{YawMode::KEEP_YAW};
    double yaw{0.0};
    double yaw_rate{0.0};
    // ---------------是否固定当前高度----------------
    bool fixed_height{false};
    // 设计一个辅助的构造函数，用来快速的提取控制话题中的数据
    UavControlCmd() = default;
    UavControlCmd(const sunray_msgs::UAVControlCMD& msg);
};


// ---------------一个简单的构造函数，用于简化提取ros msg的数据--------------
inline UavControlCmd::UavControlCmd(const sunray_msgs::UAVControlCMD& msg) {
    // 由于我们msg定义和结构体定义一一对应，实际上这里可以使用隐式类型转换，但是为了代码的可读性，我么使用显式的switch‘
    // 首先枚举命令来源
    switch (msg.cmd_source) {
    case sunray_msgs::UAVControlCMD::UNDEFINE:
        cmd_source = CmdSource::UNDEFINE;
        break;
    case sunray_msgs::UAVControlCMD::SUNRAY_STATION:
        cmd_source = CmdSource::SUNRAY_STATION;
        break;
    case sunray_msgs::UAVControlCMD::RC_CONTROLLER:
        cmd_source = CmdSource::RC_CONTROLLER;
        break;
    case sunray_msgs::UAVControlCMD::TERMINAL:
        cmd_source = CmdSource::TERMINAL;
        break;
    case sunray_msgs::UAVControlCMD::CONTROL_CMD:
        cmd_source = CmdSource::CONTROL_CMD;
        break;
    default:
        cmd_source = CmdSource::UNDEFINE;
				// 实际上，这里需要添加打印到日志，然后打印当前的msg.cmd_source
        break;
    }
    // 然后枚举控制命令
    switch (msg.control_cmd) {
    case sunray_msgs::UAVControlCMD::UNDEFINE:
        control_cmd = ControlCmd::UNDEFINE;
        break;
    case sunray_msgs::UAVControlCMD::TAKEOFF:
        control_cmd = ControlCmd::TAKEOFF;
        break;
    case sunray_msgs::UAVControlCMD::LAND:
        control_cmd = ControlCmd::LAND;
        break;
    case sunray_msgs::UAVControlCMD::RETURN:
        control_cmd = ControlCmd::RETURN;
        break;
    case sunray_msgs::UAVControlCMD::KILL:
        control_cmd = ControlCmd::KILL;
        break;
    case sunray_msgs::UAVControlCMD::HOVER:
        control_cmd = ControlCmd::HOVER;
        break;
    case sunray_msgs::UAVControlCMD::MOVE_POINT:
        control_cmd = ControlCmd::MOVE_POINT;
        break;
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY:
        control_cmd = ControlCmd::MOVE_VELOCITY;
        break;
    case sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY:
        control_cmd = ControlCmd::MOVE_TRAJECTORY;
        break;
    case sunray_msgs::UAVControlCMD::MOVE_POINT_BODY:
        control_cmd = ControlCmd::MOVE_POINT_BODY;
        break;
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY:
        control_cmd = ControlCmd::MOVE_VELOCITY_BODY;
        break;
    case sunray_msgs::UAVControlCMD::MOVE_POINT_WGS84:
        control_cmd = ControlCmd::MOVE_POINT_WGS84;
        break;
    default:
        control_cmd = ControlCmd::UNDEFINE;
				// 实际上，这里需要添加打印到日志，然后打印当前的msg.control_cmd
        break;
    }
    // 枚举yaw角控制模式
    switch (msg.yaw_mode) {
    case sunray_msgs::UAVControlCMD::KEEP_YAW:
        yaw_mode = YawMode::KEEP_YAW;
        break;
    case sunray_msgs::UAVControlCMD::SET_YAW:
        yaw_mode = YawMode::SET_YAW;
        break;
    case sunray_msgs::UAVControlCMD::SET_YAWRATE:
        yaw_mode = YawMode::SET_YAWRATE;
        break;
    default:
        yaw_mode = YawMode::KEEP_YAW;
				// 实际上，这里需要添加打印到日志，然后打印当前的msg.yaw_mode
        break;
    }
		// 提取位置数据
		position.x() = msg.desired_pos.x;
		position.y() = msg.desired_pos.y;
		position.z() = msg.desired_pos.z;
		// 提取速度数据
		velocity.x() = msg.desired_vel.x;
		velocity.y() = msg.desired_vel.y;
		velocity.z() = msg.desired_vel.z;
		// 提取加速度数据
		acceleration.x() = msg.desired_acc.x;
		acceleration.y() = msg.desired_acc.y;
		acceleration.z() = msg.desired_acc.z;
		// 提取加加速度数据
		jerk.x() = msg.desired_jerk.x;
		jerk.y() = msg.desired_jerk.y;
		jerk.z() = msg.desired_jerk.z;
		// 提取yaw角数据
		yaw = msg.desired_yaw;
		yaw_rate = msg.desired_yaw_rate;
		// 提取 wgs84 格式数据
		wgs84_position.altitude = msg.wgs84_pos.altitude;
		wgs84_position.latitude = msg.wgs84_pos.latitude;
		wgs84_position.longitude = msg.wgs84_pos.longitude;
		// 提取 固定当前高度参数(需要思考这个参数有什么用)
		fixed_height = msg.fixed_height;
};

};  // namespace control_common
