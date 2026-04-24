/*
本程序功能：
    1、定义 OrcaParams、GeoFence 关联接口和 OrcaEngine 引擎封装
    2、基于 RVO::RVOSimulator 管理多智能体状态、目标点、围栏障碍与避障参数
    3、对外提供状态更新、指令处理和单步 ORCA 计算接口，输出统一的 OrcaOutput 结果
*/
#pragma once

#include "RVO.h"
#include "obstacle_builder.h"
#include "orca_types.h"
#include <map>
#include <vector>

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
    void updateAgentState(int idx, const AgentState &state);
    // 处理设置指令
    void handleSetup(int idx, const OrcaSetupCmd &msg);
    // 运行一步计算
    bool step(double current_time, OrcaOutput &out);

  private:
    void setupAgents();
    void setupObstacles();
    bool reachedGoal(int i) const;

    int agent_id_{1};
    int agent_num_{1};
    uint8_t orca_state_{OrcaOutput::INIT};
    bool start_flag_{false};
    bool arrived_goal_{false};

    float goal_pos_[3]{0.0f, 0.0f, 0.0f};
    float goal_yaw_{0.0f};

    OrcaParams params_{};
    GeoFence fence_{};
    ObstacleBuilder obstacle_builder_{};

    std::map<int, AgentState> agent_state_{};
    std::vector<bool> odom_valid_{};

    RVO::RVOSimulator *sim_{nullptr};
};

} // namespace orca_swarm
