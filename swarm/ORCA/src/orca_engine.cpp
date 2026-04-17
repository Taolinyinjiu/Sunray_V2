// 中文说明：ORCA 引擎实现，计算速度与状态
#include "orca_engine.h"
#include <tf/transform_datatypes.h>

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
    agent_id_ = agent_id;
    agent_num_ = agent_num;
    params_ = params;
    fence_ = fence;

    sim_ = new RVO::RVOSimulator();
    odom_valid_.assign(agent_num_, false);

    for (int i = 0; i < agent_num_; ++i)
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time(0);
        agent_state_[i] = odom;
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
        const auto &odom = agent_state_[i];
        RVO::Vector2 pos(odom.pose.pose.position.x, odom.pose.pose.position.y);
        sim_->addAgent(pos);
    }
}

void OrcaEngine::setupObstacles()
{
    obstacle_builder_.init(fence_);
    obstacle_builder_.apply(sim_);
}

void OrcaEngine::updateAgentState(int idx, const nav_msgs::Odometry &odom)
{
    agent_state_[idx] = odom;
}

void OrcaEngine::handleSetup(int idx, const sunray_msgs::OrcaSetup &msg)
{
    if (msg.cmd == sunray_msgs::OrcaSetup::GOAL || msg.cmd == sunray_msgs::OrcaSetup::GOAL_RUN)
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
                goal_reached_printed_ = false;
                orca_state_ =
                    (msg.cmd == sunray_msgs::OrcaSetup::GOAL_RUN) ? sunray_msgs::OrcaCmd::RUN : sunray_msgs::OrcaCmd::INIT;
            }
        }
    }
    else if (msg.cmd == sunray_msgs::OrcaSetup::STOP)
    {
        orca_state_ = sunray_msgs::OrcaCmd::STOP;
        start_flag_ = false;
    }
    else if (msg.cmd == sunray_msgs::OrcaSetup::RUN)
    {
        orca_state_ = sunray_msgs::OrcaCmd::RUN;
        start_flag_ = true;
    }
}

bool OrcaEngine::reachedGoal(int i) const
{
    RVO::Vector2 rvo_goal = sim_->getAgentGoal(i);
    float dx = agent_state_.at(i).pose.pose.position.x - rvo_goal.x();
    float dy = agent_state_.at(i).pose.pose.position.y - rvo_goal.y();
    float dist2 = dx * dx + dy * dy;
    return dist2 < 0.15f * 0.15f;
}

bool OrcaEngine::step(sunray_msgs::OrcaCmd &out)
{
    out.header.stamp = ros::Time::now();
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
        odom_valid_[i] = (ros::Time::now() - agent_state_[i].header.stamp).toSec() <= 1.0;
        if (!odom_valid_[i])
        {
            agent_state_ready = false;
        }
    }

    int idx = agent_id_ - 1;
    if (!start_flag_ || orca_state_ == sunray_msgs::OrcaCmd::INIT || !agent_state_ready)
    {
        orca_state_ = sunray_msgs::OrcaCmd::INIT;
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
            RVO::Vector2 pos(agent_state_[i].pose.pose.position.x, agent_state_[i].pose.pose.position.y);
            RVO::Vector2 vel(agent_state_[i].twist.twist.linear.x, agent_state_[i].twist.twist.linear.y);
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

    if (orca_state_ == sunray_msgs::OrcaCmd::STOP)
    {
        out.linear[0] = 0.0f;
        out.linear[1] = 0.0f;
        out.angular[2] = 0.0f;
        return true;
    }

    if (arrived_goal_ || orca_state_ == sunray_msgs::OrcaCmd::ARRIVED)
    {
        if (!goal_reached_printed_ && arrived_goal_)
        {
            goal_reached_printed_ = true;
        }
        orca_state_ = sunray_msgs::OrcaCmd::ARRIVED;
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
    orca_state_ = sunray_msgs::OrcaCmd::RUN;
    out.state = orca_state_;
    out.linear[0] = vel.x();
    out.linear[1] = vel.y();
    out.angular[2] = 0.0f;
    return false;
}

} // namespace orca_swarm
