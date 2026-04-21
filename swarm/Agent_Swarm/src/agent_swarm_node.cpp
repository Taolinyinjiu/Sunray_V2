// 中文说明：AgentSwarm 节点实现，驱动编队计算与控制输出
#include "agent_swarm_node.h"
#include <algorithm>
#include <boost/bind.hpp>
#include <cmath>
#include <tf/transform_datatypes.h>

namespace agent_swarm
{
namespace
{

template <typename T>
T clampValue(T value, T min_value, T max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

const char *toStateString(SwarmState state)
{
    switch (state)
    {
    case SwarmState::INIT:
        return "INIT";
    case SwarmState::TAKEOFF:
        return "TAKEOFF";
    case SwarmState::LAND:
        return "LAND";
    case SwarmState::HOVER:
        return "HOVER";
    case SwarmState::FORMATION:
        return "FORMATION";
    case SwarmState::ORCA_RETURN_HOME:
        return "ORCA_RETURN_HOME";
    default:
        return "UNKNOWN";
    }
}

bool hasExplicitLeaderGoal(const geometry_msgs::Point &pt, float yaw)
{
    return std::fabs(pt.x) > 1e-6 || std::fabs(pt.y) > 1e-6 || std::fabs(pt.z) > 1e-6 || std::fabs(yaw) > 1e-6f;
}

geometry_msgs::Pose makeLeaderPose(const geometry_msgs::Point &pt, float yaw)
{
    geometry_msgs::Pose pose;
    pose.position = pt;
    pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return pose;
}

std::string formationNameFromEnum(uint8_t formation)
{
    switch (formation)
    {
    case 1:
        return "ring";
    case 2:
        return "line";
    case 3:
        return "column";
    case 4:
        return "v_shape";
    case 5:
        return "wedge";
    case 6:
        return "custom";
    default:
        return "";
    }
}

orca_swarm::AgentState toOrcaAgentState(const nav_msgs::Odometry &odom)
{
    orca_swarm::AgentState state;
    state.x = odom.pose.pose.position.x;
    state.y = odom.pose.pose.position.y;
    state.vx = odom.twist.twist.linear.x;
    state.vy = odom.twist.twist.linear.y;
    state.timestamp = odom.header.stamp.toSec();
    return state;
}

orca_swarm::OrcaSetupCmd toOrcaSetupCmd(const sunray_msgs::OrcaSetup &msg)
{
    orca_swarm::OrcaSetupCmd cmd;
    cmd.cmd = msg.cmd;
    cmd.desired_pos[0] = msg.desired_pos[0];
    cmd.desired_pos[1] = msg.desired_pos[1];
    cmd.desired_pos[2] = msg.desired_pos[2];
    cmd.desired_yaw = msg.desired_yaw;
    return cmd;
}

sunray_msgs::OrcaCmd toRosOrcaCmd(const orca_swarm::OrcaOutput &out, const ros::Time &stamp)
{
    sunray_msgs::OrcaCmd cmd;
    cmd.header.stamp = stamp;
    cmd.state = out.state;
    cmd.goal_pos[0] = out.goal_pos[0];
    cmd.goal_pos[1] = out.goal_pos[1];
    cmd.goal_pos[2] = out.goal_pos[2];
    cmd.goal_yaw = out.goal_yaw;
    cmd.linear[0] = out.linear[0];
    cmd.linear[1] = out.linear[1];
    cmd.linear[2] = out.linear[2];
    cmd.angular[0] = out.angular[0];
    cmd.angular[1] = out.angular[1];
    cmd.angular[2] = out.angular[2];
    return cmd;
}

} // namespace

AgentSwarmNode::AgentSwarmNode(ros::NodeHandle &nh) : nh_(nh)
{
    nh_.param<int>("agent_id", agent_id_, 1);
    nh_.param<int>("agent_num", agent_num_, 1);
    nh_.param<int>("agent_type", agent_type_, 0);
    nh_.param<int>("leader_id", leader_id_, 1);
    nh_.param<std::string>("agent_name", agent_name_, std::string("uav"));
    nh_.param<std::string>("formation_policy", policy_name_, std::string("ring"));
    nh_.param<double>("spacing", spacing_, 1.0);
    nh_.param<bool>("use_fixed_altitude", use_fixed_altitude_, true);
    nh_.param<double>("fixed_altitude", fixed_altitude_, 1.0);
    nh_.param<double>("spacing_min", spacing_min_, 0.3);
    nh_.param<double>("spacing_max", spacing_max_, 5.0);
    nh_.param<double>("spacing_scale_up", spacing_scale_up_, 1.2);
    nh_.param<double>("spacing_scale_down", spacing_scale_down_, 0.8);
    nh_.param<double>("leader_timeout", leader_timeout_, 1.0);
    nh_.param<double>("orca_timeout", orca_timeout_, 1.0);
    nh_.param<double>("goal_z_tolerance", goal_z_tolerance_, 0.2);
    nh_.param<bool>("leader_publish_goal", leader_publish_goal_, true);

    orca_swarm::OrcaParams orca_params;
    double neighbor_dist = static_cast<double>(orca_params.neighbor_dist);
    double time_horizon = static_cast<double>(orca_params.time_horizon);
    double time_horizon_obst = static_cast<double>(orca_params.time_horizon_obst);
    double radius = static_cast<double>(orca_params.radius);
    double max_speed = static_cast<double>(orca_params.max_speed);
    double time_step = static_cast<double>(orca_params.time_step);
    nh_.param<double>("orca_params/neighborDist", neighbor_dist, neighbor_dist);
    nh_.param<double>("orca_params/timeHorizon", time_horizon, time_horizon);
    nh_.param<double>("orca_params/timeHorizonObst", time_horizon_obst, time_horizon_obst);
    nh_.param<double>("orca_params/radius", radius, radius);
    nh_.param<double>("orca_params/maxSpeed", max_speed, max_speed);
    nh_.param<double>("orca_params/time_step", time_step, time_step);
    orca_params.neighbor_dist = static_cast<float>(neighbor_dist);
    orca_params.time_horizon = static_cast<float>(time_horizon);
    orca_params.time_horizon_obst = static_cast<float>(time_horizon_obst);
    orca_params.radius = static_cast<float>(radius);
    orca_params.max_speed = static_cast<float>(max_speed);
    orca_params.time_step = static_cast<float>(time_step);

    orca_swarm::GeoFence fence;
    double geo_min_x = static_cast<double>(fence.min_x);
    double geo_max_x = static_cast<double>(fence.max_x);
    double geo_min_y = static_cast<double>(fence.min_y);
    double geo_max_y = static_cast<double>(fence.max_y);
    nh_.param<double>("geo_fence/min_x", geo_min_x, geo_min_x);
    nh_.param<double>("geo_fence/max_x", geo_max_x, geo_max_x);
    nh_.param<double>("geo_fence/min_y", geo_min_y, geo_min_y);
    nh_.param<double>("geo_fence/max_y", geo_max_y, geo_max_y);
    fence.min_x = static_cast<float>(geo_min_x);
    fence.max_x = static_cast<float>(geo_max_x);
    fence.min_y = static_cast<float>(geo_min_y);
    fence.max_y = static_cast<float>(geo_max_y);

    custom_policy_ = std::make_shared<CustomPolicy>();
    policy_ = (policy_name_ == "custom") ? std::static_pointer_cast<FormationPolicy>(custom_policy_)
                                          : policy_factory_.create(policy_name_);

    leader_tracker_.init(nh_, leader_id_, agent_name_);
    state_cache_.init(nh_, agent_num_, agent_type_, agent_name_, leader_id_);
    goal_dispatcher_.init(nh_, agent_name_, agent_id_);
    orca_engine_.init(agent_id_, agent_num_, orca_params, fence);
    control_mapper_.init(nh_, agent_name_, agent_id_, agent_type_);

    for (int i = 0; i < agent_num_; ++i)
    {
        const int id = i + 1;
        const std::string topic = "/" + agent_name_ + std::to_string(id) + "/orca/setup";
        orca_setup_subs_[id] = nh_.subscribe<sunray_msgs::OrcaSetup>(
            topic, 10, boost::bind(&AgentSwarmNode::orcaSetupCb, this, _1, i));
    }

    const std::string self_prefix = "/" + agent_name_ + std::to_string(agent_id_);
    self_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(self_prefix + "/sunray/localization/local_odom", 10,
                                                       &AgentSwarmNode::selfOdomCb, this);
    if (agent_type_ == 0)
    {
        self_fsm_state_sub_ = nh_.subscribe<sunray_msgs::UAVControlFSMState>(
            self_prefix + "/sunray/fsm/state", 10, &AgentSwarmNode::selfFsmStateCb, this);
    }

    uav_swarm_cmd_sub_ = nh_.subscribe<sunray_msgs::UAVSwarmCMD>("/sunray/swarm/uav_swarm_cmd", 10,
                                                                 &AgentSwarmNode::onUavSwarmCmd, this);
    ugv_swarm_cmd_sub_ = nh_.subscribe<sunray_msgs::UGVSwarmCMD>("/sunray/swarm/ugv_swarm_cmd", 10,
                                                                 &AgentSwarmNode::onUgvSwarmCmd, this);
    formation_offsets_sub_ = nh_.subscribe<sunray_msgs::FormationOffsets>("/sunray/formation_offsets", 10,
                                                                          &AgentSwarmNode::formationOffsetsCb, this);
    leader_goal_sub_ =
        nh_.subscribe<geometry_msgs::PoseStamped>("/sunray/leader_goal", 10, &AgentSwarmNode::leaderGoalCb, this);

    double goal_rate = 20.0;
    double control_rate = 20.0;
    double state_pub_rate = 20.0;
    nh_.param<double>("goal_rate", goal_rate, 20.0);
    nh_.param<double>("control_rate", control_rate, 20.0);
    nh_.param<double>("state_pub_rate", state_pub_rate, 20.0);

    goal_timer_ = nh_.createTimer(ros::Duration(1.0 / goal_rate), &AgentSwarmNode::goalTimerCb, this);
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate), &AgentSwarmNode::controlTimerCb, this);
    state_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / state_pub_rate), &AgentSwarmNode::statePublishTimerCb, this);
}

bool AgentSwarmNode::cacheHomeFromCurrentPose()
{
    if (self_odom_ready_)
    {
        home_pose_ = self_odom_.pose.pose;
        home_set_ = true;
        return true;
    }

    const auto &all_states = state_cache_.states();
    auto it = all_states.find(agent_id_);
    if (it != all_states.end())
    {
        home_pose_ = it->second.pose.pose;
        home_set_ = true;
        return true;
    }

    if (last_goal_valid_)
    {
        home_pose_ = last_goal_;
        home_set_ = true;
        return true;
    }

    return false;
}

void AgentSwarmNode::onUavSwarmCmd(const sunray_msgs::UAVSwarmCMD::ConstPtr &msg)
{
    if (agent_type_ != 0)
    {
        return;
    }

    const SwarmState requested = state_machine_.requestedState();
    switch (msg->swarm_cmd)
    {
    case sunray_msgs::UAVSwarmCMD::TAKEOFF:
        if (requested != SwarmState::INIT)
        {
            return;
        }
        formation_change_active_ = false;
        leader_goal_active_ = false;
        return_home_active_ = false;
        cacheHomeFromCurrentPose();
        state_machine_.setRequestedState(SwarmState::TAKEOFF);
        return;

    case sunray_msgs::UAVSwarmCMD::LAND:
        if (requested == SwarmState::INIT || requested == SwarmState::TAKEOFF)
        {
            return;
        }
        formation_change_active_ = false;
        return_home_active_ = false;
        state_machine_.setRequestedState(SwarmState::LAND);
        return;

    case sunray_msgs::UAVSwarmCMD::HOVER:
        if (requested == SwarmState::INIT || requested == SwarmState::TAKEOFF || requested == SwarmState::LAND)
        {
            return;
        }
        formation_change_active_ = false;
        leader_goal_active_ = false;
        return_home_active_ = false;
        state_machine_.setRequestedState(SwarmState::HOVER);
        return;

    case sunray_msgs::UAVSwarmCMD::SWARM_RETURN:
        if (!home_set_ && !cacheHomeFromCurrentPose())
        {
            ROS_WARN("SWARM_RETURN ignored: home pose unavailable.");
            state_machine_.setRequestedState(SwarmState::HOVER);
            return;
        }
        formation_change_active_ = false;
        return_home_active_ = true;
        last_goal_valid_ = false;
        state_machine_.setRequestedState(SwarmState::ORCA_RETURN_HOME);
        return;

    case sunray_msgs::UAVSwarmCMD::SWARM_FORMATION:
        if (requested == SwarmState::INIT || requested == SwarmState::TAKEOFF || requested == SwarmState::LAND ||
            requested == SwarmState::ORCA_RETURN_HOME)
        {
            return;
        }
        formation_change_active_ = true;
        last_goal_valid_ = false;
        if (msg->formation_param > 0.0f)
        {
            spacing_ = clampValue<double>(msg->formation_param, spacing_min_, spacing_max_);
        }

        if (msg->formation == sunray_msgs::UAVSwarmCMD::EXPAND)
        {
            spacing_ = std::min(spacing_ * spacing_scale_up_, spacing_max_);
        }
        else if (msg->formation == sunray_msgs::UAVSwarmCMD::CONTRACT)
        {
            spacing_ = std::max(spacing_ * spacing_scale_down_, spacing_min_);
        }
        else
        {
            const std::string next_policy = formationNameFromEnum(msg->formation);
            if (!next_policy.empty())
            {
                policy_name_ = next_policy;
                policy_ = (policy_name_ == "custom") ? std::static_pointer_cast<FormationPolicy>(custom_policy_)
                                                     : policy_factory_.create(policy_name_);
            }
        }

        if (hasExplicitLeaderGoal(msg->leader_pos, msg->leader_yaw))
        {
            leader_goal_ = makeLeaderPose(msg->leader_pos, msg->leader_yaw);
            leader_goal_active_ = true;
            if (use_fixed_altitude_)
            {
                fixed_altitude_ = leader_goal_.position.z;
            }
        }
        else
        {
            leader_goal_active_ = false;
        }

        state_machine_.setRequestedState(SwarmState::FORMATION);
        return;

    default:
        return;
    }
}

void AgentSwarmNode::onUgvSwarmCmd(const sunray_msgs::UGVSwarmCMD::ConstPtr &msg)
{
    if (agent_type_ != 1)
    {
        return;
    }

    const SwarmState requested = state_machine_.requestedState();
    switch (msg->swarm_cmd)
    {
    case sunray_msgs::UGVSwarmCMD::HOLD:
        if (requested != SwarmState::INIT)
        {
            formation_change_active_ = false;
            leader_goal_active_ = false;
            return_home_active_ = false;
            state_machine_.setRequestedState(SwarmState::HOVER);
        }
        return;

    case sunray_msgs::UGVSwarmCMD::RETURN:
        if (!home_set_ && !cacheHomeFromCurrentPose())
        {
            ROS_WARN("UGV RETURN ignored: home pose unavailable.");
            return;
        }
        formation_change_active_ = false;
        return_home_active_ = true;
        last_goal_valid_ = false;
        state_machine_.setRequestedState(SwarmState::ORCA_RETURN_HOME);
        return;

    case sunray_msgs::UGVSwarmCMD::MOVE:
        leader_goal_ = makeLeaderPose(msg->desired_pos, msg->desired_yaw);
        leader_goal_active_ = true;
        last_goal_valid_ = false;
        state_machine_.setRequestedState(SwarmState::FORMATION);
        return;

    case sunray_msgs::UGVSwarmCMD::SWARM_FORMATION:
        if (requested == SwarmState::INIT || requested == SwarmState::ORCA_RETURN_HOME)
        {
            return;
        }
        formation_change_active_ = true;
        last_goal_valid_ = false;
        if (msg->formation_param > 0.0f)
        {
            spacing_ = clampValue<double>(msg->formation_param, spacing_min_, spacing_max_);
        }
        if (msg->formation == sunray_msgs::UGVSwarmCMD::EXPAND)
        {
            spacing_ = std::min(spacing_ * spacing_scale_up_, spacing_max_);
        }
        else if (msg->formation == sunray_msgs::UGVSwarmCMD::CONTRACT)
        {
            spacing_ = std::max(spacing_ * spacing_scale_down_, spacing_min_);
        }
        else
        {
            const std::string next_policy = formationNameFromEnum(msg->formation);
            if (!next_policy.empty())
            {
                policy_name_ = next_policy;
                policy_ = (policy_name_ == "custom") ? std::static_pointer_cast<FormationPolicy>(custom_policy_)
                                                     : policy_factory_.create(policy_name_);
            }
        }
        if (hasExplicitLeaderGoal(msg->leader_pos, msg->leader_yaw))
        {
            leader_goal_ = makeLeaderPose(msg->leader_pos, msg->leader_yaw);
            leader_goal_active_ = true;
        }
        else
        {
            leader_goal_active_ = false;
        }
        state_machine_.setRequestedState(SwarmState::FORMATION);
        return;

    default:
        return;
    }
}

void AgentSwarmNode::goalTimerCb(const ros::TimerEvent &)
{
    const bool leader_ok = leader_tracker_.isFresh(leader_timeout_);
    const bool orca_ok = localOrcaFresh();
    const SwarmState state = state_machine_.effectiveState(leader_ok, orca_ok);
    if (state != SwarmState::FORMATION && state != SwarmState::ORCA_RETURN_HOME)
    {
        return;
    }

    if (state == SwarmState::ORCA_RETURN_HOME)
    {
        if (!home_set_)
        {
            return;
        }
        geometry_msgs::Pose target = home_pose_;
        if (use_fixed_altitude_ && agent_type_ == 0)
        {
            target.position.z = fixed_altitude_;
        }
        last_goal_ = target;
        last_goal_valid_ = true;
        dispatchOrcaGoal(target, true);
        return;
    }

    geometry_msgs::Pose leader_pose;
    if (!leader_tracker_.getLeaderPose(leader_pose))
    {
        return;
    }

    geometry_msgs::Pose ref_pose = leader_goal_active_ ? leader_goal_ : leader_pose;
    FormationContext ctx;
    ctx.agent_id = agent_id_;
    ctx.agent_num = agent_num_;
    ctx.leader_id = leader_id_;
    ctx.spacing = spacing_;

    geometry_msgs::Pose target;
    if (policy_ && policy_->computeTarget(ref_pose, ctx, target))
    {
        if (use_fixed_altitude_ && agent_type_ == 0)
        {
            target.position.z = fixed_altitude_;
        }
        last_goal_ = target;
        last_goal_valid_ = true;
        dispatchOrcaGoal(target, true);
        return;
    }

    if (!leader_publish_goal_)
    {
        return;
    }

    const int leader_index = (leader_id_ > 100) ? (leader_id_ - 100) : leader_id_;
    if (ctx.agent_id != leader_index)
    {
        return;
    }

    geometry_msgs::Pose passthrough_pose = leader_goal_active_ ? leader_goal_ : leader_pose;
    if (use_fixed_altitude_ && agent_type_ == 0)
    {
        passthrough_pose.position.z = fixed_altitude_;
    }
    last_goal_ = passthrough_pose;
    last_goal_valid_ = true;
    dispatchOrcaGoal(passthrough_pose, true);
}

void AgentSwarmNode::controlTimerCb(const ros::TimerEvent &)
{
    const bool leader_ok = leader_tracker_.isFresh(leader_timeout_);
    const bool orca_ok = localOrcaFresh();
    const SwarmState state = state_machine_.effectiveState(leader_ok, orca_ok);
    if (!last_effective_state_valid_ || state != last_effective_state_)
    {
        ROS_INFO("Swarm state -> %s (leader_ok=%d orca_ok=%d)", toStateString(state), leader_ok ? 1 : 0,
                 orca_ok ? 1 : 0);
        last_effective_state_ = state;
        last_effective_state_valid_ = true;
    }

    if (state == SwarmState::INIT)
    {
        return;
    }
    if (state == SwarmState::HOVER)
    {
        control_mapper_.publishHover();
        return;
    }
    if (state == SwarmState::TAKEOFF)
    {
        control_mapper_.publishTakeoff(fixed_altitude_);
        if (self_fsm_state_ready_ &&
            self_fsm_state_.sunray_fsm_state == sunray_msgs::UAVControlFSMState::FSM_HOVER)
        {
            state_machine_.setRequestedState(SwarmState::HOVER);
            control_mapper_.publishHover();
        }
        return;
    }
    if (state == SwarmState::LAND)
    {
        control_mapper_.publishLand();
        if (self_fsm_state_ready_ &&
            self_fsm_state_.sunray_fsm_state == sunray_msgs::UAVControlFSMState::FSM_INIT)
        {
            state_machine_.setRequestedState(SwarmState::INIT);
        }
        return;
    }

    if (!has_orca_cmd_)
    {
        control_mapper_.publishHover();
        return;
    }
    const sunray_msgs::OrcaCmd &orca_cmd = last_orca_cmd_;

    if (orca_cmd.state == sunray_msgs::OrcaCmd::ARRIVED &&
        (formation_change_active_ || leader_goal_active_ || return_home_active_) && last_goal_valid_)
    {
        const double dx = orca_cmd.goal_pos[0] - last_goal_.position.x;
        const double dy = orca_cmd.goal_pos[1] - last_goal_.position.y;
        const double dz = orca_cmd.goal_pos[2] - last_goal_.position.z;
        const double dist2 = dx * dx + dy * dy + dz * dz;
        bool z_reached = true;
        if (agent_type_ == 0 && self_odom_ready_)
        {
            z_reached = std::fabs(self_odom_.pose.pose.position.z - last_goal_.position.z) <= goal_z_tolerance_;
        }
        if (dist2 < 0.04 && z_reached)
        {
            if (return_home_active_)
            {
                formation_change_active_ = false;
                leader_goal_active_ = false;
            }
            else
            {
                formation_change_active_ = false;
                leader_goal_active_ = false;
                return_home_active_ = false;
                state_machine_.setRequestedState(SwarmState::HOVER);
                control_mapper_.publishHover();
                return;
            }
        }
    }

    const double current_z = self_odom_ready_ ? self_odom_.pose.pose.position.z : last_goal_.position.z;
    control_mapper_.publishFromOrca(orca_cmd, current_z);
}

void AgentSwarmNode::statePublishTimerCb(const ros::TimerEvent &)
{
    updateLocalOrca();
}

void AgentSwarmNode::selfOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    self_odom_ = *msg;
    self_odom_ready_ = true;
}

void AgentSwarmNode::selfFsmStateCb(const sunray_msgs::UAVControlFSMState::ConstPtr &msg)
{
    self_fsm_state_ = *msg;
    self_fsm_state_ready_ = true;
}

void AgentSwarmNode::orcaSetupCb(const sunray_msgs::OrcaSetup::ConstPtr &msg, int idx)
{
    if (idx == agent_id_ - 1)
    {
        return;
    }
    orca_engine_.handleSetup(idx, toOrcaSetupCmd(*msg));
}

void AgentSwarmNode::leaderGoalCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    leader_goal_ = msg->pose;
    leader_goal_active_ = true;
    last_goal_valid_ = false;
    if (use_fixed_altitude_ && agent_type_ == 0)
    {
        fixed_altitude_ = leader_goal_.position.z;
    }
    state_machine_.setRequestedState(SwarmState::FORMATION);
}

void AgentSwarmNode::formationOffsetsCb(const sunray_msgs::FormationOffsets::ConstPtr &msg)
{
    if (!custom_policy_)
    {
        return;
    }
    std::vector<Offset2D> offsets;
    offsets.reserve(msg->offsets.size());
    for (const auto &pt : msg->offsets)
    {
        offsets.push_back({pt.x, pt.y});
    }
    custom_policy_->setOffsets(offsets);
}

void AgentSwarmNode::dispatchOrcaGoal(const geometry_msgs::Pose &target_pose, bool run_mode)
{
    sunray_msgs::OrcaSetup msg;
    msg.header.stamp = ros::Time::now();
    msg.cmd = run_mode ? sunray_msgs::OrcaSetup::GOAL_RUN : sunray_msgs::OrcaSetup::GOAL;
    msg.desired_pos[0] = target_pose.position.x;
    msg.desired_pos[1] = target_pose.position.y;
    msg.desired_pos[2] = target_pose.position.z;
    msg.desired_yaw = tf::getYaw(target_pose.orientation);

    orca_engine_.handleSetup(agent_id_ - 1, toOrcaSetupCmd(msg));
    goal_dispatcher_.publishGoal(target_pose, run_mode);
}

void AgentSwarmNode::updateLocalOrca()
{
    const auto &all_states = state_cache_.states();
    for (const auto &kv : all_states)
    {
        const int idx = kv.first - 1;
        if (idx < 0 || idx >= agent_num_)
        {
            continue;
        }
        orca_engine_.updateAgentState(idx, toOrcaAgentState(kv.second));
    }

    const ros::Time now = ros::Time::now();
    orca_swarm::OrcaOutput out;
    orca_engine_.step(now.toSec(), out);
    last_orca_cmd_ = toRosOrcaCmd(out, now);
    last_orca_cmd_stamp_ = now;
    has_orca_cmd_ = true;
}

bool AgentSwarmNode::localOrcaFresh() const
{
    if (!has_orca_cmd_)
    {
        return false;
    }
    return (ros::Time::now() - last_orca_cmd_stamp_).toSec() <= orca_timeout_;
}

} // namespace agent_swarm

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent_swarm_node");
    ros::NodeHandle nh("~");

    agent_swarm::AgentSwarmNode node(nh);
    ros::spin();
    return 0;
}
