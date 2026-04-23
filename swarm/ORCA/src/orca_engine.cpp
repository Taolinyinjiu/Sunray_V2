/*
本程序功能：
    1、实现 OrcaEngine 的初始化流程，创建 RVO 仿真器并配置 Agent、围栏和参数
    2、接收外部里程计状态与 GOAL/RUN/STOP 指令，同步到内部 ORCA 仿真环境
    3、执行单步 ORCA 计算，输出当前状态、目标点和速度控制结果，并处理到达判定
*/
#include "orca_engine.h"

namespace orca_swarm
{

OrcaEngine::OrcaEngine() = default;

OrcaEngine::~OrcaEngine()
{
    delete sim_;
    sim_ = nullptr;
}

void OrcaEngine::init(int agent_id, int agent_num, const OrcaParams &params, const GeoFence &fence)
{
    delete sim_;
    sim_ = nullptr;

    agent_id_ = agent_id;
    agent_num_ = agent_num;
    params_ = params;
    fence_ = fence;

    sim_ = new RVO::RVOSimulator();
    odom_valid_.assign(agent_num_, false);

    for (int i = 0; i < agent_num_; ++i)
    {
        agent_state_[i] = AgentState{};
    }

    setupAgents();
    setupObstacles();
}

void OrcaEngine::setupAgents()
{
    sim_->setAgentDefaults(params_.neighbor_dist, agent_num_, params_.time_horizon, params_.time_horizon_obst,
                           params_.radius, params_.max_speed);
    sim_->setTimeStep(params_.time_step);

    for (int i = 0; i < agent_num_; ++i)
    {
        const auto &state = agent_state_[i];
        RVO::Vector2 pos(state.x, state.y);
        sim_->addAgent(pos);
    }
}

void OrcaEngine::setupObstacles()
{
    obstacle_builder_.init(fence_);
    obstacle_builder_.apply(sim_);
}

void OrcaEngine::updateAgentState(int idx, const AgentState &state)
{
    agent_state_[idx] = state;
}

void OrcaEngine::handleSetup(int idx, const OrcaSetupCmd &msg)
{
    if (msg.cmd == OrcaSetupCmd::GOAL || msg.cmd == OrcaSetupCmd::GOAL_RUN)
    {
        sim_->setAgentGoal(idx, RVO::Vector2(msg.desired_pos[0], msg.desired_pos[1]));
        if (agent_id_ - 1 == idx)
        {
            goal_pos_[0] = msg.desired_pos[0];
            goal_pos_[1] = msg.desired_pos[1];
            goal_pos_[2] = msg.desired_pos[2];
            goal_yaw_ = msg.desired_yaw;
            start_flag_ = true;

            // 防抖：如果当前位置已在新目标的到达半径内，保持 ARRIVED，不重置
            // 否则正常切 RUN 去追新目标
            if (arrived_goal_ && reachedGoal(idx))
            {
                // 已到位且新目标在到达范围内，不重置
            }
            else
            {
                arrived_goal_ = false;
                orca_state_ = (msg.cmd == OrcaSetupCmd::GOAL_RUN) ? OrcaOutput::RUN : OrcaOutput::INIT;
            }
        }
    }
    else if (msg.cmd == OrcaSetupCmd::STOP)
    {
        orca_state_ = OrcaOutput::STOP;
        start_flag_ = false;
    }
    else if (msg.cmd == OrcaSetupCmd::RUN)
    {
        orca_state_ = OrcaOutput::RUN;
        start_flag_ = true;
    }
}

bool OrcaEngine::reachedGoal(int i) const
{
    RVO::Vector2 rvo_goal = sim_->getAgentGoal(i);
    float dx = static_cast<float>(agent_state_.at(i).x) - rvo_goal.x();
    float dy = static_cast<float>(agent_state_.at(i).y) - rvo_goal.y();
    float dist2 = dx * dx + dy * dy;
    return dist2 < 0.15f * 0.15f;
}

bool OrcaEngine::step(double current_time, OrcaOutput &out)
{
    out = OrcaOutput{};
    out.timestamp = current_time;
    out.state = orca_state_;
    out.goal_pos[0] = goal_pos_[0];
    out.goal_pos[1] = goal_pos_[1];
    out.goal_pos[2] = goal_pos_[2];
    out.goal_yaw = goal_yaw_;
    out.linear[2] = 0.0f;
    out.angular[0] = 0.0f;
    out.angular[1] = 0.0f;

    bool agent_state_ready = true;
    for (int i = 0; i < agent_num_; ++i)
    {
        const auto &state = agent_state_[i];
        odom_valid_[i] = (state.timestamp > 0.0) && ((current_time - state.timestamp) <= 1.0);
        if (!odom_valid_[i])
        {
            agent_state_ready = false;
        }
    }

    int idx = agent_id_ - 1;
    if (!start_flag_ || orca_state_ == OrcaOutput::INIT || !agent_state_ready)
    {
        orca_state_ = OrcaOutput::INIT;
        out.state = orca_state_;
        out.linear[0] = 0.0f;
        out.linear[1] = 0.0f;
        out.angular[2] = 0.0f;
        return false;
    }

    if (!arrived_goal_)
    {
        arrived_goal_ = reachedGoal(idx);
    }

    for (int i = 0; i < agent_num_; ++i)
    {
        if (odom_valid_[i])
        {
            RVO::Vector2 pos(agent_state_[i].x, agent_state_[i].y);
            RVO::Vector2 vel(agent_state_[i].vx, agent_state_[i].vy);
            sim_->setAgentPosition(i, pos);
            sim_->setAgentVelocity(i, vel);
        }
        else
        {
            // 过期 agent 移到远处，避免幽灵位置影响避障决策
            sim_->setAgentPosition(i, RVO::Vector2(1e4f, 1e4f));
            sim_->setAgentVelocity(i, RVO::Vector2(0.0f, 0.0f));
        }
    }

    // 始终运行 ORCA 计算，确保障碍物/围栏避障在任何状态下都生效
    sim_->computeVel();

    if (orca_state_ == OrcaOutput::STOP)
    {
        out.linear[0] = 0.0f;
        out.linear[1] = 0.0f;
        out.angular[2] = 0.0f;
        return true;
    }

    if (arrived_goal_ || orca_state_ == OrcaOutput::ARRIVED)
    {
        orca_state_ = OrcaOutput::ARRIVED;
        out.state = orca_state_;
        // ARRIVED 时仍输出 ORCA 避障速度（正常到位时接近 0，靠近围栏时产生排斥）
        RVO::Vector2 vel = sim_->getAgentVelCMD(idx);
        out.linear[0] = vel.x();
        out.linear[1] = vel.y();
        out.angular[2] = 0.0f;
        return true;
    }

    // 正常运行输出速度
    RVO::Vector2 vel = sim_->getAgentVelCMD(idx);
    orca_state_ = OrcaOutput::RUN;
    out.state = orca_state_;
    out.linear[0] = vel.x();
    out.linear[1] = vel.y();
    out.angular[2] = 0.0f;
    return false;
}

} // namespace orca_swarm
