/*
本程序功能：
    1、定义 ControlCommandMapper 类，负责将 ORCA 避障输出映射为底层飞控指令
    2、根据 ORCA 状态（RUN/ARRIVED/STOP）自动选择速度控制或位置控制模式
    3、提供 publishHover/publishTakeoff/publishLand/publishPosTarget 等直接控制接口
    4、publishFromOrca 内含纵向 P 控制器，根据目标高度与当前高度差生成 vz
    5、发布 sunray_msgs::UAVControlCMD 到 /{agent_name}{id}/sunray/uav_control_cmd
*/
#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sunray_msgs/OrcaCmd.h>
#include <sunray_msgs/UAVControlCMD.h>

namespace agent_swarm
{

// 控制指令映射器
class ControlCommandMapper
{
  public:
    // 初始化发布器
    void init(ros::NodeHandle &nh, const std::string &agent_name, int agent_id, int agent_type);
    // 发布悬停/停止
    void publishHover();
    // 发布起飞指令（仅 UAV 有效），altitude 为目标起飞高度(m)
    void publishTakeoff(double altitude);
    // 发布降落指令（仅 UAV 有效）
    void publishLand();
    // 发布位置控制指令（优先用于集群移动）
    void publishPosTarget(const geometry_msgs::Pose &target_pose);
    // 从 ORCA 指令生成控制指令
    void publishFromOrca(const sunray_msgs::OrcaCmd &cmd, double current_z);

  private:
    sunray_msgs::UAVControlCMD makeBaseCmd() const;

    int agent_type_{0};
    ros::Publisher uav_pub_{};
};

} // namespace agent_swarm
