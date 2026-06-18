/**
 * 构造本文件的目的在于对自定义话题uav_control_cmd.msg进行解析，通过构造一个结构体来填充数据
 */
#pragma once

#include <cstdint>
#include <Eigen/Dense>
#include <geographic_msgs/GeoPoint.h>
#include <string>
#include <sunray_msgs/UAVControlCMD.h>
namespace control_common {
struct UavControlCmd {
    // ----------- 命令来源枚举 -------------
    enum class CmdSource : uint8_t {
        UNDEFINE = 0,         // 未定义的来源
        SUNRAY_STATION = 1,   // Sunray地面站控制
        RC_CONTROLLER = 2,    // RC遥控器控制
        TERMINAL = 3,         // 终端控制
        SWARM_CONTROL = 4,    // 来自集群模块的控制命令
        PLANNING = 5,         // 来自规划模块的控制指令
        EXAMPLE_DEMO = 6      // 来自示例模块的控制指令
    };
    // ------------- 控制命令枚举 ------------
    enum class ControlCmd : uint8_t {
        UNDEFINE = 0,            //	未知指令
        TAKEOFF = 1,             // [特殊]起飞指令
        LAND = 2,                // [特殊]降落指令
        RETURN = 3,              // [特殊]返航指令
        KILL = 4,                // [特殊]锁桨指令
        HOVER = 5,               // [特殊]悬停指令：在当前位置悬停
        MOVE_POINT = 6,          // 移动指令：移动到某一点
        MOVE_VELOCITY = 7,       // 速度指令：控制无人机的速度
        MOVE_TRAJECTORY = 8,     // 轨迹指令：跟踪轨迹
        MOVE_POINT_BODY = 9,     // 移动指令：移动到机体系下的某一点
        MOVE_VELOCITY_BODY = 10, // 速度指令：控制无人机在机体系下的速度
        MOVE_POINT_WGS84 = 11    // 移动指令：移动到经纬高系下的某一点
    };
    // ------------偏航角控制模式枚举---------
    enum class YawMode : uint8_t {
        KEEP_YAW = 0,     // 保持当前yaw角不变
        SET_YAW = 1,      // 设置惯性系绝对yaw角
        SET_YAWRATE = 2,  // 设置yaw角速度
    };
    // ------时间戳-----
    ros::Time timestamp{ros::Time(0)};
    // 控制指令
    CmdSource cmd_source{CmdSource::UNDEFINE};     // 控制指令来源
    ControlCmd control_cmd{ControlCmd::UNDEFINE};  // 控制指令类型
    // ---------------根据ControlCmd决定msg中的数据如何填充---------------------
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
    Eigen::Vector2d body_position_xy{Eigen::Vector2d::Zero()};
    Eigen::Vector2d body_velocity_xy{Eigen::Vector2d::Zero()};
    // ------------这个特殊一点，经纬高---------------
    // wgs84_position = [latitude_deg, longitude_deg, altitude_m]
    geographic_msgs::GeoPoint wgs84_position;
    // ------------- yaw角控制模式---------------
    YawMode yaw_mode{YawMode::KEEP_YAW};
    double yaw{0.0};
    double yaw_rate{0.0};
    // body系控制时，z轴使用世界系固定高度语义
    // velocity控制时，可选使用fixed_height让z轴高度保持不变
    double fixed_height{0.0};
    // TAKEOFF控制时可选覆盖yaml默认起飞参数；<=0表示使用yaml默认值
    double takeoff_relative_height{0.0};
    double takeoff_max_velocity{0.0};
    // LAND控制时可选覆盖yaml默认降落速度；<=0表示使用yaml默认值
    double land_max_velocity{0.0};
    uint64_t yunlink_session_id{0};
    uint64_t yunlink_message_id{0};
    uint64_t yunlink_correlation_id{0};
    // 设计一个辅助的构造函数，用来快速的提取控制话题中的数据
    UavControlCmd() = default;
    UavControlCmd(const sunray_msgs::UAVControlCMD& msg);
};

// ---------------一个简单的构造函数，用于简化提取ros msg的数据--------------
inline UavControlCmd::UavControlCmd(const sunray_msgs::UAVControlCMD& msg) {
    timestamp = msg.header.stamp;
    // 内部枚举值与 UAVControlCMD.msg 保持一一对应，直接做静态类型转换即可。
    cmd_source = static_cast<CmdSource>(msg.cmd_source);
    control_cmd = static_cast<ControlCmd>(msg.control_cmd);
    yaw_mode = static_cast<YawMode>(msg.yaw_mode);
    // 默认提取惯性系数据
    // 位置数据
    position.x() = msg.desired_pos.x;
    position.y() = msg.desired_pos.y;
    position.z() = msg.desired_pos.z;
    // 速度数据
    velocity.x() = msg.desired_vel.x;
    velocity.y() = msg.desired_vel.y;
    velocity.z() = msg.desired_vel.z;
    // 加速度数据 
    acceleration.x() = msg.desired_acc.x;
    acceleration.y() = msg.desired_acc.y;
    acceleration.z() = msg.desired_acc.z;
    // 加加速度数据
    jerk.x() = msg.desired_jerk.x;
    jerk.y() = msg.desired_jerk.y;
    jerk.z() = msg.desired_jerk.z;

    // body系控制单独携带 body-xy + fixed_height，需要覆盖默认解析结果；
    // yaw / yaw_rate 仍按惯性系语义提取
    if (control_cmd == ControlCmd::MOVE_POINT_BODY) {
        body_position_xy.x() = msg.desired_body_xy_pos.x;
        body_position_xy.y() = msg.desired_body_xy_pos.y;
    } else if (control_cmd == ControlCmd::MOVE_VELOCITY_BODY) {
        body_velocity_xy.x() = msg.desired_body_xy_vel.x;
        body_velocity_xy.y() = msg.desired_body_xy_vel.y;
    }
    // 提取yaw角数据
    yaw = msg.desired_yaw;
    yaw_rate = msg.desired_yaw_rate;
    // 提取 wgs84 格式数据
    wgs84_position.altitude = msg.desired_wgs84_pos.altitude;
    wgs84_position.latitude = msg.desired_wgs84_pos.latitude;
    wgs84_position.longitude = msg.desired_wgs84_pos.longitude;
    fixed_height = msg.fixed_height;
    takeoff_relative_height = msg.takeoff_relative_height;
    takeoff_max_velocity = msg.takeoff_max_velocity;
    land_max_velocity = msg.land_max_velocity;
    yunlink_session_id = msg.yunlink_session_id;
    yunlink_message_id = msg.yunlink_message_id;
    yunlink_correlation_id = msg.yunlink_correlation_id;
};

};  // namespace control_common
