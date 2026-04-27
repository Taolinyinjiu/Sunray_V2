/*
本程序功能：
    1、实现 UGV 集群控制节点 UGVSwarmNode，统一管理参数、订阅器、发布器与定时器
    2、接收集群指令、Leader 目标、自身里程计和自定义阵型偏移，生成当前编队目标点
    3、联动 FormationStateMachine、FormationPolicy、OrcaEngine 完成编队跟随与避障控制
    4、将 ORCA 输出转换为地面平台控制目标，支持保持、编队、返航等模式，并提供 main 入口
*/
#include <algorithm>
#include <boost/bind.hpp>
#include <cmath>
#include <map>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_datatypes.h>

#include <sunray_msgs/OrcaCmd.h>
#include <sunray_msgs/OrcaSetup.h>
#include <sunray_msgs/UGVSwarmCMD.h>
#include <sunray_msgs/UGVSwarmState.h>

#include "agent_state_cache.h"
#include "control_command_mapper.h"
#include "formation_policies.h"
#include "formation_policy_factory.h"
#include "formation_state_machine.h"
#include "goal_dispatcher.h"
#include "orca_engine.h"

namespace swarm_control
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

uint8_t toSwarmStateMsg(SwarmState state)
{
    return static_cast<uint8_t>(state);
}

bool hasExplicitLeaderGoal(const geometry_msgs::Point &pt, float yaw)
{
    return std::fabs(pt.x) > 1e-6 || std::fabs(pt.y) > 1e-6 || std::fabs(pt.z) > 1e-6 || std::fabs(yaw) > 1e-6f;
}

geometry_msgs::Pose makePose(const geometry_msgs::Point &pt, float yaw)
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

orca_swarm::OrcaSetupCmd toOrcaSetupCMD(const sunray_msgs::OrcaSetup &msg)
{
    orca_swarm::OrcaSetupCmd cmd;
    cmd.cmd = msg.cmd;
    cmd.desired_pos[0] = msg.desired_pos[0];
    cmd.desired_pos[1] = msg.desired_pos[1];
    cmd.desired_pos[2] = msg.desired_pos[2];
    cmd.desired_yaw = msg.desired_yaw;
    return cmd;
}

sunray_msgs::OrcaCmd toRosOrcaCMD(const orca_swarm::OrcaOutput &out, const ros::Time &stamp)
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

class UGVSwarmNode
{
  public:
    explicit UGVSwarmNode(ros::NodeHandle &nh) : nh_(nh)
    {
        nh_.param<int>("agent_id", agent_id_, 1);
        nh_.param<int>("agent_num", agent_num_, 1);
        nh_.param<std::string>("agent_name", agent_name_, std::string("ugv"));
        nh_.param<std::string>("formation_policy", policy_name_, std::string("keep_formation"));
        nh_.param<double>("spacing", spacing_, 1.0);
        nh_.param<double>("spacing_min", spacing_min_, 0.3);
        nh_.param<double>("spacing_max", spacing_max_, 5.0);
        nh_.param<double>("spacing_scale_up", spacing_scale_up_, 1.2);
        nh_.param<double>("spacing_scale_down", spacing_scale_down_, 0.8);
        nh_.param<double>("orca_timeout", orca_timeout_, 1.0);
        nh_.param<double>("virtual_leader_x", virtual_leader_x_, 0.0);
        nh_.param<double>("virtual_leader_y", virtual_leader_y_, 0.0);
        nh_.param<double>("virtual_leader_z", virtual_leader_z_, 0.0);
        nh_.param<double>("virtual_leader_yaw", virtual_leader_yaw_, 0.0);

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

        state_cache_.init(nh_, agent_num_, 1, agent_name_);
        goal_dispatcher_.init(nh_, agent_name_, agent_id_);
        orca_engine_.init(agent_id_, agent_num_, orca_params, fence);
        control_mapper_.init(nh_, agent_name_, agent_id_, 1);
        initVirtualLeaderGoal();

        for (int i = 0; i < agent_num_; ++i)
        {
            const int id = i + 1;
            const std::string topic = "/" + agent_name_ + std::to_string(id) + "/orca/setup";
            orca_setup_subs_[id] = nh_.subscribe<sunray_msgs::OrcaSetup>(
                topic, 10, boost::bind(&UGVSwarmNode::orcaSetupCb, this, _1, i));
        }

        const std::string self_prefix = "/" + agent_name_ + std::to_string(agent_id_);
        self_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(self_prefix + "/sunray/localization/local_odom", 10,
                                                           &UGVSwarmNode::selfOdomCb, this);
        ugv_swarm_cmd_sub_ = nh_.subscribe<sunray_msgs::UGVSwarmCMD>("/sunray/swarm/ugv_swarm_cmd", 10,
                                                                     &UGVSwarmNode::onUGVSwarmCMD, this);
        ugv_swarm_state_pub_ = nh_.advertise<sunray_msgs::UGVSwarmState>("/sunray/swarm/ugv_swarm_state", 10, true);
        double goal_rate = 20.0;
        double control_rate = 20.0;
        double state_pub_rate = 20.0;
        nh_.param<double>("goal_rate", goal_rate, 20.0);
        nh_.param<double>("control_rate", control_rate, 20.0);
        nh_.param<double>("state_pub_rate", state_pub_rate, 20.0);

        goal_timer_ = nh_.createTimer(ros::Duration(1.0 / goal_rate), &UGVSwarmNode::goalTimerCb, this);
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate), &UGVSwarmNode::controlTimerCb, this);
        state_pub_timer_ =
            nh_.createTimer(ros::Duration(1.0 / state_pub_rate), &UGVSwarmNode::statePublishTimerCb, this);
    }

  private:
    void onUGVSwarmCMD(const sunray_msgs::UGVSwarmCMD::ConstPtr &msg)
    {
        const SwarmState requested = state_machine_.requestedState();
        switch (msg->swarm_cmd)
        {
        case sunray_msgs::UGVSwarmCMD::SWARM_HOLD:
            if (requested == SwarmState::INIT)
            {
                return;
            }
            clearCommandFlags();
            cacheHoldFromCurrentPose();
            state_machine_.setRequestedState(SwarmState::HOVER);
            acceptSwarmCommand(*msg);
            return;

        case sunray_msgs::UGVSwarmCMD::SWARM_RETURN:
            if (!home_set_ && !cacheHomeFromCurrentPose())
            {
                ROS_WARN("UGV RETURN ignored: home pose unavailable.");
                return;
            }
            clearCommandFlags();
            return_home_active_ = true;
            last_goal_valid_ = false;
            state_machine_.setRequestedState(SwarmState::ORCA_RETURN_HOME);
            acceptSwarmCommand(*msg);
            return;

        case sunray_msgs::UGVSwarmCMD::SWARM_FORMATION:
            if (requested == SwarmState::INIT || requested == SwarmState::ORCA_RETURN_HOME)
            {
                return;
            }
            if (!applyFormationCommand(msg->formation, msg->formation_param, msg->leader_pos, msg->leader_yaw,
                                       msg->custom_offsets))
            {
                return;
            }
            state_machine_.setRequestedState(SwarmState::FORMATION);
            acceptSwarmCommand(*msg);
            return;

        default:
            return;
        }
    }

    void goalTimerCb(const ros::TimerEvent &)
    {
        const bool leader_ok = true;
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
            last_goal_ = home_pose_;
            last_goal_valid_ = true;
            dispatchOrcaGoal(home_pose_, true);
            return;
        }

        geometry_msgs::Pose ref_pose = leader_goal_;

        FormationContext ctx;
        ctx.agent_id = agent_id_;
        ctx.agent_num = agent_num_;
        ctx.spacing = spacing_;

        geometry_msgs::Pose target;
        if (policy_ && policy_->computeTarget(ref_pose, ctx, target))
        {
            last_goal_ = target;
            last_goal_valid_ = true;
            dispatchOrcaGoal(target, true);
            return;
        }

        return;
    }

    void controlTimerCb(const ros::TimerEvent &)
    {
        const bool leader_ok = true;
        const bool orca_ok = localOrcaFresh();
        const SwarmState state = state_machine_.effectiveState(leader_ok, orca_ok);
        if (!last_effective_state_valid_ || state != last_effective_state_)
        {
            ROS_INFO("Swarm state -> %s (leader_ok=%d orca_ok=%d)", toStateString(state), leader_ok ? 1 : 0,
                     orca_ok ? 1 : 0);
            last_effective_state_ = state;
            last_effective_state_valid_ = true;
            publishSwarmState(state);
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

        if (!has_orca_cmd_)
        {
            return;
        }

        if (isActiveGoalReached(last_orca_cmd_))
        {
            setHoldPose(last_goal_);
            clearCommandFlags();
            state_machine_.setRequestedState(SwarmState::HOVER);
            control_mapper_.publishHover();
            publishSwarmState(SwarmState::HOVER);
            return;
        }

        control_mapper_.publishFromOrca(last_orca_cmd_, 0.0);
    }

    void statePublishTimerCb(const ros::TimerEvent &)
    {
        updateLocalOrca();
    }

    void acceptSwarmCommand(const sunray_msgs::UGVSwarmCMD &msg)
    {
        current_swarm_cmd_ = msg;
        has_current_swarm_cmd_ = true;
        publishSwarmState(state_machine_.requestedState());
    }

    void publishSwarmState(SwarmState state)
    {
        sunray_msgs::UGVSwarmState msg;
        msg.header.stamp = ros::Time::now();
        if (has_current_swarm_cmd_)
        {
            msg.current_cmd = current_swarm_cmd_;
        }
        msg.state = toSwarmStateMsg(state);
        msg.swarm_num = static_cast<uint32_t>(agent_num_);
        ugv_swarm_state_pub_.publish(msg);
    }

    void selfOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
    {
        self_odom_ = *msg;
        self_odom_ready_ = true;
    }

    void orcaSetupCb(const sunray_msgs::OrcaSetup::ConstPtr &msg, int idx)
    {
        if (idx == agent_id_ - 1)
        {
            return;
        }
        orca_engine_.handleSetup(idx, toOrcaSetupCMD(*msg));
    }

    void initVirtualLeaderGoal()
    {
        geometry_msgs::Point pt;
        pt.x = virtual_leader_x_;
        pt.y = virtual_leader_y_;
        pt.z = virtual_leader_z_;
        leader_goal_ = makePose(pt, static_cast<float>(virtual_leader_yaw_));
        leader_goal_active_ = true;
    }

    void applyCustomOffsets(const std::vector<geometry_msgs::Point> &custom_offsets)
    {
        if (!custom_policy_)
        {
            return;
        }
        std::vector<Offset2D> offsets;
        offsets.reserve(custom_offsets.size());
        for (const auto &pt : custom_offsets)
        {
            offsets.push_back({pt.x, pt.y});
        }
        custom_policy_->setOffsets(offsets);
    }

    bool captureCurrentFormationAsCustom(float reference_yaw)
    {
        if (!custom_policy_)
        {
            return false;
        }

        auto all_states = state_cache_.states();
        if (self_odom_ready_)
        {
            all_states[agent_id_] = self_odom_;
        }
        if (static_cast<int>(all_states.size()) < agent_num_)
        {
            ROS_WARN("UGV KEEP_FORMATION: current formation capture skipped, only %zu/%d odometry states available.",
                     all_states.size(), agent_num_);
            return false;
        }

        double center_x = 0.0;
        double center_y = 0.0;
        for (int id = 1; id <= agent_num_; ++id)
        {
            auto it = all_states.find(id);
            if (it == all_states.end())
            {
                ROS_WARN("UGV KEEP_FORMATION: current formation capture skipped, missing odometry for %s%d.",
                         agent_name_.c_str(), id);
                return false;
            }
            center_x += it->second.pose.pose.position.x;
            center_y += it->second.pose.pose.position.y;
        }
        center_x /= static_cast<double>(agent_num_);
        center_y /= static_cast<double>(agent_num_);

        const double c = std::cos(static_cast<double>(reference_yaw));
        const double s = std::sin(static_cast<double>(reference_yaw));
        const double spacing = std::max(spacing_, 1e-6);
        std::vector<Offset2D> offsets;
        offsets.reserve(agent_num_);
        for (int id = 1; id <= agent_num_; ++id)
        {
            const auto &pose = all_states[id].pose.pose;
            const double dx = pose.position.x - center_x;
            const double dy = pose.position.y - center_y;
            offsets.push_back({(c * dx + s * dy) / spacing, (-s * dx + c * dy) / spacing});
        }

        custom_policy_->setOffsets(offsets);
        policy_name_ = "custom";
        policy_ = custom_policy_;
        ROS_INFO("UGV KEEP_FORMATION: captured current physical formation as custom offsets.");
        return true;
    }

    bool cacheHomeFromCurrentPose()
    {
        if (self_odom_ready_)
        {
            home_pose_ = self_odom_.pose.pose;
            home_set_ = true;
            return true;
        }

        const auto all_states = state_cache_.states();
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

    bool cacheHoldFromCurrentPose()
    {
        if (self_odom_ready_)
        {
            setHoldPose(self_odom_.pose.pose);
            return true;
        }
        if (last_goal_valid_)
        {
            setHoldPose(last_goal_);
            return true;
        }
        if (home_set_)
        {
            setHoldPose(home_pose_);
            return true;
        }
        return false;
    }

    void setHoldPose(const geometry_msgs::Pose &pose)
    {
        hold_pose_ = pose;
        hold_pose_valid_ = true;
    }

    void clearCommandFlags()
    {
        formation_change_active_ = false;
        return_home_active_ = false;
    }

    bool applyFormationCommand(uint8_t formation, float formation_param, const geometry_msgs::Point &leader_pos,
                               float leader_yaw, const std::vector<geometry_msgs::Point> &custom_offsets)
    {
        formation_change_active_ = true;
        return_home_active_ = false;
        last_goal_valid_ = false;
        const bool explicit_leader_goal = hasExplicitLeaderGoal(leader_pos, leader_yaw);

        if (formation_param > 0.0f)
        {
            spacing_ = clampValue<double>(formation_param, spacing_min_, spacing_max_);
        }

        if (formation == sunray_msgs::UGVSwarmCMD::KEEP_FORMATION && explicit_leader_goal)
        {
            if (!captureCurrentFormationAsCustom(leader_yaw))
            {
                formation_change_active_ = false;
                ROS_WARN("UGV KEEP_FORMATION command ignored: unable to capture current physical formation.");
                return false;
            }
        }
        else if (formation == sunray_msgs::UGVSwarmCMD::EXPAND)
        {
            spacing_ = std::min(spacing_ * spacing_scale_up_, spacing_max_);
        }
        else if (formation == sunray_msgs::UGVSwarmCMD::CONTRACT)
        {
            spacing_ = std::max(spacing_ * spacing_scale_down_, spacing_min_);
        }
        else
        {
            if (formation == sunray_msgs::UGVSwarmCMD::CUSTOM)
            {
                applyCustomOffsets(custom_offsets);
            }
            const std::string next_policy = formationNameFromEnum(formation);
            if (!next_policy.empty())
            {
                policy_name_ = next_policy;
                policy_ = (policy_name_ == "custom") ? std::static_pointer_cast<FormationPolicy>(custom_policy_)
                                                     : policy_factory_.create(policy_name_);
            }
        }

        if (explicit_leader_goal)
        {
            leader_goal_ = makePose(leader_pos, leader_yaw);
            leader_goal_active_ = true;
        }
        else
        {
            leader_goal_active_ = true;
        }
        return true;
    }

    bool isActiveGoalReached(const sunray_msgs::OrcaCmd &orca_cmd) const
    {
        if (orca_cmd.state != sunray_msgs::OrcaCmd::ARRIVED)
        {
            return false;
        }
        if (!(formation_change_active_ || leader_goal_active_ || return_home_active_))
        {
            return false;
        }
        if (!last_goal_valid_)
        {
            return false;
        }

        const double dx = orca_cmd.goal_pos[0] - last_goal_.position.x;
        const double dy = orca_cmd.goal_pos[1] - last_goal_.position.y;
        const double dz = orca_cmd.goal_pos[2] - last_goal_.position.z;
        return (dx * dx + dy * dy + dz * dz) < 0.04;
    }

    void dispatchOrcaGoal(const geometry_msgs::Pose &target_pose, bool run_mode)
    {
        sunray_msgs::OrcaSetup msg;
        msg.header.stamp = ros::Time::now();
        msg.cmd = run_mode ? sunray_msgs::OrcaSetup::GOAL_RUN : sunray_msgs::OrcaSetup::GOAL;
        msg.desired_pos[0] = target_pose.position.x;
        msg.desired_pos[1] = target_pose.position.y;
        msg.desired_pos[2] = target_pose.position.z;
        msg.desired_yaw = tf::getYaw(target_pose.orientation);

        orca_engine_.handleSetup(agent_id_ - 1, toOrcaSetupCMD(msg));
        goal_dispatcher_.publishGoal(target_pose, run_mode);
    }

    void updateLocalOrca()
    {
        const auto all_states = state_cache_.states();
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
        last_orca_cmd_ = toRosOrcaCMD(out, now);
        last_orca_cmd_stamp_ = now;
        has_orca_cmd_ = true;
    }

    bool localOrcaFresh() const
    {
        if (!has_orca_cmd_)
        {
            return false;
        }
        return (ros::Time::now() - last_orca_cmd_stamp_).toSec() <= orca_timeout_;
    }

    ros::NodeHandle nh_;

    int agent_id_{1};
    int agent_num_{1};
    std::string agent_name_{"ugv"};
    std::string policy_name_{"keep_formation"};
    double spacing_{1.0};
    double spacing_min_{0.3};
    double spacing_max_{5.0};
    double spacing_scale_up_{1.2};
    double spacing_scale_down_{0.8};
    double orca_timeout_{1.0};
    double virtual_leader_x_{0.0};
    double virtual_leader_y_{0.0};
    double virtual_leader_z_{0.0};
    double virtual_leader_yaw_{0.0};

    FormationStateMachine state_machine_{};
    FormationPolicyFactory policy_factory_{};
    std::shared_ptr<FormationPolicy> policy_{};
    std::shared_ptr<CustomPolicy> custom_policy_{};
    AgentStateCache state_cache_{};
    GoalDispatcher goal_dispatcher_{};
    orca_swarm::OrcaEngine orca_engine_{};
    ControlCommandMapper control_mapper_{};

    ros::Subscriber ugv_swarm_cmd_sub_{};
    ros::Publisher ugv_swarm_state_pub_{};
    ros::Subscriber self_odom_sub_{};
    std::map<int, ros::Subscriber> orca_setup_subs_{};
    ros::Timer goal_timer_{};
    ros::Timer control_timer_{};
    ros::Timer state_pub_timer_{};

    nav_msgs::Odometry self_odom_{};
    bool self_odom_ready_{false};
    bool formation_change_active_{false};
    bool leader_goal_active_{false};
    bool return_home_active_{false};
    bool home_set_{false};
    bool hold_pose_valid_{false};
    geometry_msgs::Pose home_pose_{};
    geometry_msgs::Pose hold_pose_{};
    geometry_msgs::Pose leader_goal_{};
    geometry_msgs::Pose last_goal_{};
    bool last_goal_valid_{false};
    sunray_msgs::OrcaCmd last_orca_cmd_{};
    sunray_msgs::UGVSwarmCMD current_swarm_cmd_{};
    ros::Time last_orca_cmd_stamp_{};
    bool has_orca_cmd_{false};
    bool has_current_swarm_cmd_{false};
    SwarmState last_effective_state_{SwarmState::INIT};
    bool last_effective_state_valid_{false};
};

} // namespace swarm_control

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_swarm_node");
    ros::NodeHandle nh("~");

    swarm_control::UGVSwarmNode node(nh);
    ros::spin();
    return 0;
}
