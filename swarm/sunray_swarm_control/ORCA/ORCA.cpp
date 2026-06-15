/*
本文件功能：
    1、实现纯 C++ 的 ORCA 封装类，对原生 RVO2 提供稳定的数值化 API
    2、负责维护 agent 状态与目标点，并在每步计算前同步到原生 RVO2
    3、输出每个 agent 的期望速度与 ORCA 修正后速度，供上层控制逻辑使用
 */
#include "ORCA.h"

#include "RVO.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace orca_swarm
{

namespace
{

// 避免在目标点极近时做归一化，防止数值抖动。
constexpr double kMinDistance = 1e-6;
constexpr double kDeadlockTimeThreshold = 1.0;
constexpr double kDeadlockPrefSpeedRatio = 0.2;
constexpr double kDeadlockCmdSpeedRatio = 0.08;
constexpr double kDeterministicLateralBiasRatio = 0.6;
constexpr float kSafetyRadiusPadding = 0.03F;
constexpr double kArrivalSlowDownDistance = 0.30;
constexpr double kArrivalMinSpeedRatio = 0.12;
constexpr double kPi = 3.14159265358979323846;

} // namespace

ORCA::ORCA() = default;

ORCA::~ORCA() = default;

void ORCA::init(const int agent_num,
                const float neighbor_dist,
                const float time_horizon,
                const float radius,
                const float max_speed,
                const float time_step,
                const int max_neighbors)
{
    agent_num_ = std::max(0, agent_num);
    max_neighbors_ = (max_neighbors > 0) ? max_neighbors : std::max(1, agent_num_);
    neighbor_dist_ = neighbor_dist;
    time_horizon_ = time_horizon;
    radius_ = radius;
    max_speed_ = max_speed;
    time_step_ = time_step;
    agents_.assign(agent_num_, AgentData{});

    sim_.reset(new RVO::RVOSimulator());
    sim_->setTimeStep(time_step_);

    // 这里调用的是原生 RVO2 的默认 agent 参数设置接口。
    // 上层 formation 会按 radius 生成几何安全间距；ORCA 内部额外加 3cm 裕量，
    // 用于抵消离散积分、浮点误差和目标点边界贴近时造成的安全距离轻微跌破。
    // obstacle 相关功能目前不开放，因此将 obstacle 时间域与普通时间域保持一致。
    const float solver_radius = radius_ + kSafetyRadiusPadding;
    sim_->setAgentDefaults(neighbor_dist_,
                           static_cast<size_t>(max_neighbors_),
                           time_horizon_,
                           time_horizon_,
                           solver_radius,
                           max_speed_);

    for (int i = 0; i < agent_num_; ++i)
    {
        sim_->addAgent(RVO::Vector2(0.0F, 0.0F));
    }
}

bool ORCA::setAgentState(const int idx, const double x, const double y, const double vx, const double vy)
{
    if (!is_valid_index(idx))
    {
        return false;
    }

    AgentData &agent = agents_[idx];
    agent.x = x;
    agent.y = y;
    agent.vx = vx;
    agent.vy = vy;
    return true;
}

bool ORCA::setAgentGoal(const int idx, const double x, const double y)
{
    if (!is_valid_index(idx))
    {
        return false;
    }

    AgentData &agent = agents_[idx];
    agent.goal_x = x;
    agent.goal_y = y;
    agent.goal_enabled = true;
    return true;
}

bool ORCA::clearAgentGoal(const int idx)
{
    if (!is_valid_index(idx))
    {
        return false;
    }

    AgentData &agent = agents_[idx];
    agent.goal_x = agent.x;
    agent.goal_y = agent.y;
    agent.goal_enabled = false;
    return true;
}

bool ORCA::addCircleObstacle(const double x, const double y, const double radius, const int vertex_num)
{
    if (!sim_ || radius <= 0.0 || vertex_num < 8)
    {
        return false;
    }

    std::vector<RVO::Vector2> vertices;
    vertices.reserve(static_cast<size_t>(vertex_num));

    for (int i = 0; i < vertex_num; ++i)
    {
        const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(vertex_num);
        vertices.emplace_back(static_cast<float>(x + radius * std::cos(angle)),
                              static_cast<float>(y + radius * std::sin(angle)));
    }

    if (sim_->addObstacle(vertices) == RVO::RVO_ERROR)
    {
        return false;
    }

    // 原生 RVO2 要求 obstacle 添加完成后显式构建障碍物 KD-tree。
    sim_->processObstacles();
    return true;
}

bool ORCA::computePreferredVelocity(const int idx, double &pref_vx, double &pref_vy) const
{
    if (!is_valid_index(idx))
    {
        return false;
    }

    const AgentData &agent = agents_[idx];
    if (!agent.goal_enabled)
    {
        pref_vx = 0.0;
        pref_vy = 0.0;
        return true;
    }

    const double dx = agent.goal_x - agent.x;
    const double dy = agent.goal_y - agent.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= kMinDistance)
    {
        pref_vx = 0.0;
        pref_vy = 0.0;
        return true;
    }

    // 当距离目标点较近时降低期望速度，但保留一个小的最低推进速度。
    // 纯 proportional 速度在目标点附近容易被 ORCA 约束压到极低，导致卡在到达阈值外侧。
    double desired_speed = static_cast<double>(max_speed_);
    if (distance < kArrivalSlowDownDistance)
    {
        const double min_arrival_speed = kArrivalMinSpeedRatio * static_cast<double>(max_speed_);
        desired_speed = std::max(min_arrival_speed,
                                 static_cast<double>(max_speed_) * distance / kArrivalSlowDownDistance);
    }
    desired_speed = std::min(desired_speed, static_cast<double>(max_speed_));
    const double goal_dir_x = dx / distance;
    const double goal_dir_y = dy / distance;
    double desired_dir_x = goal_dir_x;
    double desired_dir_y = goal_dir_y;
    bool has_congestion_conflict = false;

    for (int other_idx = 0; other_idx < agent_num_; ++other_idx)
    {
        if (other_idx == idx)
        {
            continue;
        }

        const AgentData &other_agent = agents_[other_idx];
        if (!other_agent.goal_enabled)
        {
            continue;
        }

        const double rel_x = other_agent.x - agent.x;
        const double rel_y = other_agent.y - agent.y;
        const double rel_distance = std::sqrt(rel_x * rel_x + rel_y * rel_y);
        if (rel_distance > static_cast<double>(neighbor_dist_) || rel_distance <= kMinDistance)
        {
            continue;
        }

        const double other_goal_dx = other_agent.goal_x - other_agent.x;
        const double other_goal_dy = other_agent.goal_y - other_agent.y;
        const double other_goal_distance = std::sqrt(other_goal_dx * other_goal_dx + other_goal_dy * other_goal_dy);
        if (other_goal_distance <= kMinDistance)
        {
            continue;
        }

        const double other_goal_dir_x = other_goal_dx / other_goal_distance;
        const double other_goal_dir_y = other_goal_dy / other_goal_distance;
        const double direction_dot = goal_dir_x * other_goal_dir_x + goal_dir_y * other_goal_dir_y;
        if (direction_dot < 0.98)
        {
            has_congestion_conflict = true;
            break;
        }
    }

    // 对称互换场景中，所有 agent 可能被 ORCA 推到“刚好不碰但谁也过不去”的死区。
    // 当邻居已经进入 ORCA 搜索半径且目标方向明显不一致，或某个 agent 已经低速僵持时，
    // 给目标方向叠加一个固定手性的侧向分量，用于提前打破完全对称的互换路径。
    // 该偏置使用固定手性，不使用随机数，因此同一场景可重复复现。
    if (has_congestion_conflict || agent.deadlock_time >= kDeadlockTimeThreshold)
    {
        const double lateral_dir_x = -goal_dir_y;
        const double lateral_dir_y = goal_dir_x;
        desired_dir_x = goal_dir_x + kDeterministicLateralBiasRatio * lateral_dir_x;
        desired_dir_y = goal_dir_y + kDeterministicLateralBiasRatio * lateral_dir_y;

        const double desired_dir_norm = std::sqrt(desired_dir_x * desired_dir_x + desired_dir_y * desired_dir_y);
        if (desired_dir_norm > kMinDistance)
        {
            desired_dir_x /= desired_dir_norm;
            desired_dir_y /= desired_dir_norm;
        }
        else
        {
            desired_dir_x = goal_dir_x;
            desired_dir_y = goal_dir_y;
        }
    }

    pref_vx = desired_dir_x * desired_speed;
    pref_vy = desired_dir_y * desired_speed;
    return true;
}

bool ORCA::GetOrcaVelCmd(const int idx, double &cmd_vx, double &cmd_vy)
{
    if (agent_num_ <= 0 || !sim_ || !is_valid_index(idx))
    {
        return false;
    }

    for (int i = 0; i < agent_num_; ++i)
    {
        AgentData &agent = agents_[i];

        // 先把外部世界中的状态同步到底层 ORCA 求解器。
        sim_->setAgentPosition(static_cast<size_t>(i),
                               RVO::Vector2(static_cast<float>(agent.x), static_cast<float>(agent.y)));
        sim_->setAgentVelocity(static_cast<size_t>(i),
                               RVO::Vector2(static_cast<float>(agent.vx), static_cast<float>(agent.vy)));

        double pref_vx = 0.0;
        double pref_vy = 0.0;
        computePreferredVelocity(i, pref_vx, pref_vy);

        // ORCA 并不直接接收“目标点”，而是接收“期望速度”作为避碰前输入。
        sim_->setAgentPrefVelocity(static_cast<size_t>(i),
                                   RVO::Vector2(static_cast<float>(pref_vx), static_cast<float>(pref_vy)));

        agent.pref_vx = pref_vx;
        agent.pref_vy = pref_vy;
    }

    // 执行ORCA算法
    sim_->doStep();

    for (int i = 0; i < agent_num_; ++i)
    {
        AgentData &agent = agents_[i];
        const RVO::Vector2 &cmd_velocity = sim_->getAgentVelocity(static_cast<size_t>(i));
        agent.cmd_vx = cmd_velocity.x();
        agent.cmd_vy = cmd_velocity.y();

        const double pref_speed = std::sqrt(agent.pref_vx * agent.pref_vx + agent.pref_vy * agent.pref_vy);
        const double cmd_speed = std::sqrt(agent.cmd_vx * agent.cmd_vx + agent.cmd_vy * agent.cmd_vy);
        if (agent.goal_enabled && pref_speed > kDeadlockPrefSpeedRatio * static_cast<double>(max_speed_) &&
            cmd_speed < kDeadlockCmdSpeedRatio * static_cast<double>(max_speed_))
        {
            agent.deadlock_time += static_cast<double>(time_step_);
        }
        else
        {
            agent.deadlock_time = 0.0;
        }
    }

    const AgentData &agent = agents_[idx];
    cmd_vx = agent.cmd_vx;
    cmd_vy = agent.cmd_vy;
    return true;
}

bool ORCA::is_valid_index(const int idx) const
{
    return idx >= 0 && idx < agent_num_;
}

} // namespace orca_swarm
