// 中文说明：ORCA 客户端接口，订阅 /orca_cmd
#pragma once

#include <ros/ros.h>
#include <sunray_msgs/OrcaCmd.h>

namespace agent_swarm
{

// ORCA 客户端
class OrcaClient
{
  public:
    // 初始化订阅
    void init(ros::NodeHandle &nh, const std::string &agent_name, int agent_id);
    // 判断数据是否超时
    bool isFresh(double timeout_sec) const;
    // 获取最新 ORCA 指令
    bool getLastCmd(sunray_msgs::OrcaCmd &out) const;

  private:
    // 回调函数
    void cmdCallback(const sunray_msgs::OrcaCmd::ConstPtr &msg);

    sunray_msgs::OrcaCmd last_cmd_{};
    ros::Time last_stamp_{};
    bool has_cmd_{false};
    ros::Subscriber sub_{};
};

} // namespace agent_swarm
