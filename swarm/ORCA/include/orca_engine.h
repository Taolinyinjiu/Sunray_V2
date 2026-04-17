// 中文说明：ORCA 引擎接口，RVO 避障计算
#pragma once

#include "RVO.h"
#include "obstacle_builder.h"
#include <map>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sunray_msgs/OrcaCmd.h>
#include <sunray_msgs/OrcaSetup.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

namespace orca_swarm
{

// ORCA 参数配置
struct OrcaParams
{
    float neighbor_dist{1.5f};
    float time_horizon{2.0f};
    float time_horizon_obst{2.0f};
    float radius{0.3f};
    float max_speed{0.5f};
    float time_step{0.1f};
};

// ORCA 引擎
class OrcaEngine
{
  public:
    OrcaEngine();
    ~OrcaEngine();

    // 初始化引擎
    void init(int agent_id, int agent_num, const OrcaParams &params, const GeoFence &fence);
    // 更新智能体状态
    void updateAgentState(int idx, const nav_msgs::Odometry &odom);
    // 处理设置指令
    void handleSetup(int idx, const sunray_msgs::OrcaSetup &msg);
    // 运行一步计算
    bool step(sunray_msgs::OrcaCmd &out);

    // 标志位与围栏可视化
    bool startFlag() const
    {
        return start_flag_;
    }
    bool goalReachedPrinted() const
    {
        return goal_reached_printed_;
    }
    const visualization_msgs::MarkerArray &fenceMarkers() const
    {
        return obstacle_builder_.markers();
    }

  private:
    void setupAgents();
    void setupObstacles();
    bool reachedGoal(int i) const;

    int agent_id_{1};
    int agent_num_{1};
    uint8_t orca_state_{sunray_msgs::OrcaCmd::INIT};
    bool start_flag_{false};
    bool goal_reached_printed_{false};
    bool arrived_goal_{false};

    float goal_pos_[3]{0.0f, 0.0f, 0.0f};
    float goal_yaw_{0.0f};

    OrcaParams params_{};
    GeoFence fence_{};
    ObstacleBuilder obstacle_builder_{};

    std::map<int, nav_msgs::Odometry> agent_state_{};
    std::vector<bool> odom_valid_{};

    RVO::RVOSimulator *sim_{nullptr};
};

} // namespace orca_swarm
