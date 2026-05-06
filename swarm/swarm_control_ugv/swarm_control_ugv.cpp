/*
本文件功能：
    1、实现 Swarm_Control_UGV 的集群控制状态机
    2、实现 UGVSwarmCMD 接收、formation 目标点计算、ORCA 速度解算和 UGVControlCMD 输出
    3、定时发布 UGVSwarmState 供外部监控/地面站查看当前集群控制状态
*/
#include "swarm_control_ugv.h"
#include "sunray_log.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace swarm_control
{

Swarm_Control_UGV::Swarm_Control_UGV(ros::NodeHandle &nh)
    : nh_(nh)
{
    // 1. 读取基础参数。
    nh_.param("agent_id", params_.agent_id, 1);
    nh_.param("swarm_num", params_.swarm_num, 1);
    nh_.param("agent_name", params_.agent_name, std::string("ugv"));
    nh_.param("control_loop_hz", params_.control_loop_hz, 50.0 /*Hz*/);
    nh_.param("peer_odom_timeout", params_.peer_odom_timeout, 0.1 /*秒*/);
    nh_.param("dynamic_prepare_wait_time", params_.dynamic_prepare_wait_time, 2.0 /*秒*/);
    params_.dynamic_prepare_wait_time = std::max(0.0, params_.dynamic_prepare_wait_time);

    // 2. 读取到达阈值参数。
    nh_.param("goal_xy_tolerance", params_.goal_xy_tolerance, 0.1 /*米*/);
    nh_.param("goal_yaw_tolerance", params_.goal_yaw_tolerance, 0.1 /*rad*/);

    // 3. 读取场地限制参数，用于 formation 判断目标点合法性。
    nh_.param("field/x_min", params_.field_x_min, -50.0 /*米*/);
    nh_.param("field/x_max", params_.field_x_max, 50.0 /*米*/);
    nh_.param("field/y_min", params_.field_y_min, -50.0 /*米*/);
    nh_.param("field/y_max", params_.field_y_max, 50.0 /*米*/);
    nh_.param("field/z_min", params_.field_z_min, -1.0 /*米*/);
    nh_.param("field/z_max", params_.field_z_max, 1.0 /*米*/);

    // 4. 读取 ORCA 参数。
    nh_.param("orca/neighbor_dist", params_.orca_neighbor_dist, 2.0 /*米*/);
    nh_.param("orca/time_horizon", params_.orca_time_horizon, 2.0 /*秒*/);
    nh_.param("orca/radius", params_.orca_radius, 0.3 /*米*/);
    nh_.param("orca/max_speed", params_.orca_max_speed, 0.8 /*米/秒*/);
    nh_.param("orca/max_neighbors", params_.orca_max_neighbors, -1);

    // 5. 读取圆形静态障碍物参数，ORCA 用于避障，formation 用于目标点合法性检查。
    nh_.param("static_obstacles/enabled", params_.static_obstacles_enabled, false);
    if (params_.static_obstacles_enabled)
    {
        nh_.getParam("static_obstacles/x", params_.static_obstacle_x);
        nh_.getParam("static_obstacles/y", params_.static_obstacle_y);
        nh_.getParam("static_obstacles/radius", params_.static_obstacle_radius);

        if (params_.static_obstacle_x.size() != params_.static_obstacle_y.size() ||
            params_.static_obstacle_x.size() != params_.static_obstacle_radius.size())
        {
            SUNRAY_WARN("static_obstacles param size mismatch: x={} y={} radius={}",
                        params_.static_obstacle_x.size(),
                        params_.static_obstacle_y.size(),
                        params_.static_obstacle_radius.size());
            params_.static_obstacle_x.clear();
            params_.static_obstacle_y.clear();
            params_.static_obstacle_radius.clear();
        }
    }

    odom_caches_.assign(static_cast<size_t>(params_.swarm_num + 1), OdomCache{});

    formation_.init(params_.swarm_num, params_.orca_radius, params_.orca_max_speed, params_.field_x_min,
                    params_.field_x_max, params_.field_y_min, params_.field_y_max, params_.field_z_min,
                    params_.field_z_max);
    orca_.init(params_.swarm_num, static_cast<float>(params_.orca_neighbor_dist),
               static_cast<float>(params_.orca_time_horizon), static_cast<float>(params_.orca_radius),
               static_cast<float>(params_.orca_max_speed), static_cast<float>(1.0 / params_.control_loop_hz),
               params_.orca_max_neighbors);

    for (size_t i = 0; i < params_.static_obstacle_x.size(); ++i)
    {
        if (params_.static_obstacle_radius[i] <= 0.0)
        {
            SUNRAY_WARN("ignore invalid static_obstacle_{} radius={:.2f}", i + 1, params_.static_obstacle_radius[i]);
            continue;
        }

        const bool formation_ok = formation_.addCircleObstacle(params_.static_obstacle_x[i],
                                                               params_.static_obstacle_y[i],
                                                               params_.static_obstacle_radius[i]);
        const bool orca_ok = orca_.addCircleObstacle(params_.static_obstacle_x[i],
                                                     params_.static_obstacle_y[i],
                                                     params_.static_obstacle_radius[i]);
        if (!formation_ok || !orca_ok)
        {
            SUNRAY_WARN("add static obstacle failed: x={:.2f} y={:.2f} radius={:.2f}",
                        params_.static_obstacle_x[i],
                        params_.static_obstacle_y[i],
                        params_.static_obstacle_radius[i]);
        }
    }

    const std::string self_ns = "/" + params_.agent_name + std::to_string(params_.agent_id);
    const std::string local_odom_topic = self_ns + "/sunray/localization/local_odom";
    const std::string ugv_fsm_state_topic = self_ns + "/sunray/ugv_control/ugv_control_fsm_state";
    const std::string control_cmd_topic = self_ns + "/sunray/ugv_control/control_cmd";
    const std::string swarm_cmd_topic = "/sunray/swarm/ugv_swarm_cmd";
    const std::string swarm_state_topic = "/sunray/swarm/ugv_swarm_state";

    local_odom_sub_ = nh_.subscribe(local_odom_topic, 20, &Swarm_Control_UGV::localOdomCallback, this);
    ugv_fsm_state_sub_ = nh_.subscribe(ugv_fsm_state_topic, 20, &Swarm_Control_UGV::ugvFsmStateCallback, this);

    peer_odom_subs_.clear();
    peer_odom_subs_.reserve(std::max(0, params_.swarm_num - 1));
    for (int agent_id = 1; agent_id <= params_.swarm_num; ++agent_id)
    {
        if (agent_id == params_.agent_id)
        {
            continue;
        }

        const std::string peer_topic =
            "/" + params_.agent_name + std::to_string(agent_id) + "/sunray/localization/local_odom";
        peer_odom_subs_.push_back(
            nh_.subscribe<nav_msgs::Odometry>(peer_topic, 20,
                                              [this, agent_id](const nav_msgs::Odometry::ConstPtr &msg) {
                                                  peerOdomCallback(msg, agent_id);
                                              }));
    }

    swarm_cmd_sub_ =
        nh_.subscribe<sunray_msgs::UGVSwarmCMD>(swarm_cmd_topic, 10, &Swarm_Control_UGV::swarmCmdCallback, this);
    control_cmd_pub_ = nh_.advertise<sunray_msgs::UGVControlCMD>(control_cmd_topic, 10);
    swarm_state_pub_ = nh_.advertise<sunray_msgs::UGVSwarmState>(swarm_state_topic, 10);

    swarm_control_fsm_timer_ =
        nh_.createTimer(ros::Duration(1.0 / params_.control_loop_hz), &Swarm_Control_UGV::swarm_control_main_loop, this);
    swarm_state_pub_timer_ = nh_.createTimer(ros::Duration(0.1), &Swarm_Control_UGV::swarmStatePubTimerCallback, this);

    SUNRAY_INFO("========== swarm_control_ugv init ==========");
    SUNRAY_INFO("agent_id={}", params_.agent_id);
    SUNRAY_INFO("swarm_num={}", params_.swarm_num);
    SUNRAY_INFO("local_odom_topic={}", local_odom_topic);
    SUNRAY_INFO("ugv_fsm_state_topic={}", ugv_fsm_state_topic);
    SUNRAY_INFO("swarm_cmd_topic={}", swarm_cmd_topic);
    SUNRAY_INFO("swarm_state_topic={}", swarm_state_topic);
    SUNRAY_INFO("control_cmd_topic={}", control_cmd_topic);
    SUNRAY_INFO("control_loop_hz={:.1f}", params_.control_loop_hz);
    SUNRAY_INFO("peer_odom_timeout={:.3f}", params_.peer_odom_timeout);
    SUNRAY_INFO("dynamic_prepare_wait_time={:.3f}", params_.dynamic_prepare_wait_time);
    SUNRAY_INFO("orca_radius={:.2f}", params_.orca_radius);
    SUNRAY_INFO("orca_max_speed={:.2f}", params_.orca_max_speed);
    SUNRAY_INFO("static_obstacles_num={}", params_.static_obstacle_x.size());
    SUNRAY_INFO("============================================");

    ugv_swarm_state_.fsm_state = sunray_msgs::UGVSwarmState::INIT;
    ugv_swarm_state_.agent_id = static_cast<uint8_t>(params_.agent_id);
    ugv_swarm_state_.swarm_num = static_cast<uint32_t>(params_.swarm_num);
}

void Swarm_Control_UGV::swarm_control_main_loop(const ros::TimerEvent &)
{
    switch (ugv_swarm_state_.fsm_state)
    {
    case sunray_msgs::UGVSwarmState::INIT:
        break;

    case sunray_msgs::UGVSwarmState::ARRIVED:
        publishHoldCommand();
        break;

    case sunray_msgs::UGVSwarmState::RETURN_HOME:
        if (hasReachedGoal())
        {
            switchToArrived(HoldPointSource::GOAL_POINT);
            break;
        }
        goal_point_ = return_point_;
        updateOrcaCommand();
        break;

    case sunray_msgs::UGVSwarmState::SWARM_STATIC_FORMATION:
        if (hasReachedGoal())
        {
            switchToArrived(HoldPointSource::GOAL_POINT);
            break;
        }

        goal_point_.valid = getFormationGoalForAgent(ugv_swarm_cmd_.formation_cmd, params_.agent_id, 0.0, goal_point_);
        if (goal_point_.valid)
        {
            updateOrcaCommand();
        }
        else
        {
            switchToArrived(HoldPointSource::CURRENT_ODOM);
        }
        break;

    case sunray_msgs::UGVSwarmState::SWARM_DYNAMIC_FORMATION_PREPARE:
    {
        const ros::Time now = ros::Time::now();

        if (!getFormationGoalForAgent(ugv_swarm_cmd_.formation_cmd, params_.agent_id, 0.0, goal_point_))
        {
            switchToArrived(HoldPointSource::CURRENT_ODOM);
            break;
        }

        updateOrcaCommand();

        if (areAllAgentsAtDynamicInitialGoal())
        {
            if (dynamic_prepare_ready_time_.isZero())
            {
                dynamic_prepare_ready_time_ = now;
            }

            if ((now - dynamic_prepare_ready_time_).toSec() >= params_.dynamic_prepare_wait_time)
            {
                dynamic_prepare_ready_time_ = ros::Time(0.0);
                dynamic_formation_start_time_ = now;
                ugv_swarm_state_.fsm_state = sunray_msgs::UGVSwarmState::SWARM_DYNAMIC_FORMATION;
            }
        }
        else
        {
            dynamic_prepare_ready_time_ = ros::Time(0.0);
        }
        break;
    }

    case sunray_msgs::UGVSwarmState::SWARM_DYNAMIC_FORMATION:
    {
        const double elapsed_time = std::max(0.0, (ros::Time::now() - dynamic_formation_start_time_).toSec());
        if (elapsed_time >= static_cast<double>(ugv_swarm_cmd_.formation_cmd.dynamic_time))
        {
            switchToArrived(HoldPointSource::CURRENT_ODOM);
            break;
        }

        goal_point_.valid =
            getFormationGoalForAgent(ugv_swarm_cmd_.formation_cmd, params_.agent_id, elapsed_time, goal_point_);
        if (goal_point_.valid)
        {
            updateOrcaCommand();
        }
        else
        {
            switchToArrived(HoldPointSource::CURRENT_ODOM);
        }
        break;
    }

    default:
        ugv_swarm_state_.fsm_state = sunray_msgs::UGVSwarmState::INIT;
        break;
    }
}

void Swarm_Control_UGV::localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!isValidAgentId(params_.agent_id))
    {
        return;
    }

    OdomCache &cache = odom_caches_[static_cast<size_t>(params_.agent_id)];
    cache.odom = *msg;
    cache.receive_time = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    cache.received = true;

    if (!return_point_.valid)
    {
        fillGoalPointFromSelfOdom(return_point_);
    }
}

void Swarm_Control_UGV::ugvFsmStateCallback(const sunray_msgs::UGVControlFSMState::ConstPtr &msg)
{
    ugv_control_fsm_state_ = *msg;
    has_ugv_fsm_state_ = true;
}

void Swarm_Control_UGV::peerOdomCallback(const nav_msgs::Odometry::ConstPtr &msg, const int agent_id)
{
    if (!isValidAgentId(agent_id))
    {
        return;
    }

    OdomCache &cache = odom_caches_[static_cast<size_t>(agent_id)];
    cache.odom = *msg;
    cache.receive_time = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    cache.received = true;
}

void Swarm_Control_UGV::swarmCmdCallback(const sunray_msgs::UGVSwarmCMD::ConstPtr &msg)
{
    if (!isReadyForSwarmCmd(*msg))
    {
        return;
    }

    if (msg->agent_id != 99U && msg->agent_id != static_cast<uint8_t>(params_.agent_id))
    {
        return;
    }

    sunray_msgs::UGVSwarmCMD received_cmd = *msg;
    if (received_cmd.formation_cmd.header.stamp.isZero())
    {
        received_cmd.formation_cmd.header.stamp = received_cmd.header.stamp;
    }

    switch (received_cmd.swarm_cmd)
    {
    case sunray_msgs::UGVSwarmCMD::SWARM_HOLD:
        switchToArrived(HoldPointSource::CURRENT_ODOM);
        publishHoldCommand();
        break;

    case sunray_msgs::UGVSwarmCMD::SWARM_RETURN:
        if (!return_point_.valid)
        {
            return;
        }
        goal_point_ = return_point_;
        ugv_swarm_state_.fsm_state = sunray_msgs::UGVSwarmState::RETURN_HOME;
        break;

    case sunray_msgs::UGVSwarmCMD::SWARM_FORMATION:
    {
        if (received_cmd.formation_cmd.formation_type == sunray_msgs::Formation::STATIC_KEEP_FORMATION)
        {
            std::vector<double> current_pos_x(static_cast<size_t>(params_.swarm_num), 0.0);
            std::vector<double> current_pos_y(static_cast<size_t>(params_.swarm_num), 0.0);
            std::vector<double> current_pos_z(static_cast<size_t>(params_.swarm_num), 0.0);
            std::vector<double> current_yaw(static_cast<size_t>(params_.swarm_num), 0.0);

            for (int agent_id = 1; agent_id <= params_.swarm_num; ++agent_id)
            {
                const nav_msgs::Odometry &odom = odom_caches_[static_cast<size_t>(agent_id)].odom;
                const size_t idx = static_cast<size_t>(agent_id - 1);
                current_pos_x[idx] = odom.pose.pose.position.x;
                current_pos_y[idx] = odom.pose.pose.position.y;
                current_pos_z[idx] = odom.pose.pose.position.z;
                current_yaw[idx] = getYawFromOdom(odom);
            }

            if (!formation_.CaptureKeepFormation(current_pos_x.data(), current_pos_y.data(), current_pos_z.data(),
                                                 current_yaw.data(), params_.swarm_num))
            {
                ROS_WARN_THROTTLE(2.0, "capture STATIC_KEEP_FORMATION snapshot failed.");
                return;
            }
        }

        GoalPoint candidate_goal;
        candidate_goal.valid = getFormationGoalForAgent(received_cmd.formation_cmd, params_.agent_id, 0.0, candidate_goal);
        if (!candidate_goal.valid)
        {
            ROS_WARN_THROTTLE(2.0, "formation cmd rejected: can not compute valid goal point.");
            return;
        }

        goal_point_ = candidate_goal;
        if (isDynamicFormationType(received_cmd.formation_cmd.formation_type))
        {
            dynamic_prepare_ready_time_ = ros::Time(0.0);
            dynamic_formation_start_time_ = ros::Time(0.0);
            ugv_swarm_state_.fsm_state = sunray_msgs::UGVSwarmState::SWARM_DYNAMIC_FORMATION_PREPARE;
        }
        else
        {
            ugv_swarm_state_.fsm_state = sunray_msgs::UGVSwarmState::SWARM_STATIC_FORMATION;
        }
        break;
    }

    default:
        ROS_WARN_THROTTLE(2.0, "ignore ugv swarm cmd: unsupported swarm_cmd=%d",
                          static_cast<int>(received_cmd.swarm_cmd));
        return;
    }

    ugv_swarm_cmd_ = received_cmd;
}

void Swarm_Control_UGV::swarmStatePubTimerCallback(const ros::TimerEvent &)
{
    const ros::Time now = ros::Time::now();

    ugv_swarm_state_.header.stamp = now;
    ugv_swarm_state_.agent_id = static_cast<uint8_t>(params_.agent_id);
    ugv_swarm_state_.swarm_num = static_cast<uint32_t>(params_.swarm_num);
    ugv_swarm_state_.swarm_cmd = ugv_swarm_cmd_;

    ugv_swarm_state_.self_odom_ready = hasLocalOdom();
    ugv_swarm_state_.self_odom = nav_msgs::Odometry{};
    if (ugv_swarm_state_.self_odom_ready)
    {
        ugv_swarm_state_.self_odom = odom_caches_[static_cast<size_t>(params_.agent_id)].odom;
    }

    ugv_swarm_state_.ready_peer_num = 0;
    ugv_swarm_state_.peers_odom_ready = true;
    for (int agent_id = 1; agent_id <= params_.swarm_num; ++agent_id)
    {
        if (agent_id == params_.agent_id)
        {
            continue;
        }

        const OdomCache &cache = odom_caches_[static_cast<size_t>(agent_id)];
        const bool peer_ready = cache.received && ((now - cache.receive_time).toSec() <= params_.peer_odom_timeout);
        if (peer_ready)
        {
            ++ugv_swarm_state_.ready_peer_num;
        }
        else
        {
            ugv_swarm_state_.peers_odom_ready = false;
        }
    }

    ugv_swarm_state_.target_valid = false;
    ugv_swarm_state_.target_pos = geometry_msgs::Point{};
    ugv_swarm_state_.target_yaw = 0.0f;

    const GoalPoint *target_point = nullptr;
    switch (ugv_swarm_state_.fsm_state)
    {
    case sunray_msgs::UGVSwarmState::RETURN_HOME:
    case sunray_msgs::UGVSwarmState::SWARM_STATIC_FORMATION:
    case sunray_msgs::UGVSwarmState::SWARM_DYNAMIC_FORMATION_PREPARE:
    case sunray_msgs::UGVSwarmState::SWARM_DYNAMIC_FORMATION:
        target_point = goal_point_.valid ? &goal_point_ : nullptr;
        break;

    case sunray_msgs::UGVSwarmState::ARRIVED:
        target_point = hold_point_.valid ? &hold_point_ : nullptr;
        break;

    default:
        break;
    }

    if (target_point != nullptr)
    {
        ugv_swarm_state_.target_valid = true;
        ugv_swarm_state_.target_pos.x = target_point->x;
        ugv_swarm_state_.target_pos.y = target_point->y;
        ugv_swarm_state_.target_pos.z = target_point->z;
        ugv_swarm_state_.target_yaw = static_cast<float>(target_point->yaw);
    }

    ugv_swarm_state_.ugv_cmd = ugv_control_cmd_;
    swarm_state_pub_.publish(ugv_swarm_state_);
}

bool Swarm_Control_UGV::isValidAgentId(const int agent_id) const
{
    return agent_id >= 1 && agent_id <= params_.swarm_num;
}

bool Swarm_Control_UGV::isReadyForSwarmCmd(const sunray_msgs::UGVSwarmCMD &cmd) const
{
    if (!hasLocalOdom())
    {
        ROS_WARN_THROTTLE(2.0, "ignore ugv swarm cmd: local odom is unavailable.");
        return false;
    }

    if (!has_ugv_fsm_state_)
    {
        ROS_WARN_THROTTLE(2.0, "ignore ugv swarm cmd: ugv fsm state is unavailable.");
        return false;
    }

    const bool need_peer_odom = (cmd.swarm_cmd == sunray_msgs::UGVSwarmCMD::SWARM_FORMATION ||
                                 cmd.swarm_cmd == sunray_msgs::UGVSwarmCMD::SWARM_RETURN);
    if (!need_peer_odom)
    {
        return true;
    }

    const ros::Time now = ros::Time::now();
    for (int agent_id = 1; agent_id <= params_.swarm_num; ++agent_id)
    {
        if (agent_id == params_.agent_id)
        {
            continue;
        }

        const OdomCache &cache = odom_caches_[static_cast<size_t>(agent_id)];
        if (!cache.received)
        {
            ROS_WARN_THROTTLE(2.0, "ignore ugv swarm cmd: peer odom is unavailable, missing_agent_id=%d", agent_id);
            return false;
        }

        const double odom_age = (now - cache.receive_time).toSec();
        if (odom_age > params_.peer_odom_timeout)
        {
            ROS_WARN_THROTTLE(2.0,
                              "ignore ugv swarm cmd: peer odom timed out, timeout_agent_id=%d odom_age=%.3fs "
                              "timeout_threshold=%.3fs",
                              agent_id,
                              odom_age,
                              params_.peer_odom_timeout);
            return false;
        }
    }

    return true;
}

bool Swarm_Control_UGV::hasLocalOdom() const
{
    return isValidAgentId(params_.agent_id) && odom_caches_[static_cast<size_t>(params_.agent_id)].received;
}

double Swarm_Control_UGV::getYawFromOdom(const nav_msgs::Odometry &odom)
{
    const geometry_msgs::Quaternion &q = odom.pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double Swarm_Control_UGV::normalizeYaw(const double yaw)
{
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

bool Swarm_Control_UGV::isDynamicFormationType(const uint8_t formation_type)
{
    return formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_RING ||
           formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON ||
           formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE;
}

bool Swarm_Control_UGV::hasReachedGoal() const
{
    return hasAgentReachedGoal(params_.agent_id, goal_point_);
}

bool Swarm_Control_UGV::getFormationGoalForAgent(const sunray_msgs::Formation &formation_cmd,
                                                 const int agent_id,
                                                 const double formation_time,
                                                 GoalPoint &goal_point)
{
    goal_point = GoalPoint{};
    goal_point.valid = formation_.GetFormationGoal(formation_cmd,
                                                   agent_id,
                                                   formation_time,
                                                   goal_point.x,
                                                   goal_point.y,
                                                   goal_point.z,
                                                   goal_point.yaw);
    return goal_point.valid;
}

bool Swarm_Control_UGV::hasAgentReachedGoal(const int agent_id, const GoalPoint &goal_point) const
{
    if (!goal_point.valid || !isValidAgentId(agent_id) || !odom_caches_[static_cast<size_t>(agent_id)].received)
    {
        return false;
    }

    const nav_msgs::Odometry &odom = odom_caches_[static_cast<size_t>(agent_id)].odom;
    const double dx = goal_point.x - odom.pose.pose.position.x;
    const double dy = goal_point.y - odom.pose.pose.position.y;
    const double yaw_error = normalizeYaw(goal_point.yaw - getYawFromOdom(odom));

    return std::hypot(dx, dy) <= params_.goal_xy_tolerance && std::abs(yaw_error) <= params_.goal_yaw_tolerance;
}

bool Swarm_Control_UGV::isAgentOdomReady(const int agent_id, const ros::Time &now) const
{
    if (!isValidAgentId(agent_id))
    {
        return false;
    }

    const OdomCache &cache = odom_caches_[static_cast<size_t>(agent_id)];
    return cache.received && ((now - cache.receive_time).toSec() <= params_.peer_odom_timeout);
}

bool Swarm_Control_UGV::areAllAgentsAtDynamicInitialGoal()
{
    const ros::Time now = ros::Time::now();
    for (int agent_id = 1; agent_id <= params_.swarm_num; ++agent_id)
    {
        if (!isAgentOdomReady(agent_id, now))
        {
            return false;
        }

        GoalPoint initial_goal;
        if (!getFormationGoalForAgent(ugv_swarm_cmd_.formation_cmd, agent_id, 0.0, initial_goal) ||
            !hasAgentReachedGoal(agent_id, initial_goal))
        {
            return false;
        }
    }

    return true;
}

void Swarm_Control_UGV::updateOrcaCommand()
{
    if (!goal_point_.valid)
    {
        return;
    }

    for (int agent_id = 1; agent_id <= params_.swarm_num; ++agent_id)
    {
        const size_t idx = static_cast<size_t>(agent_id);
        const int orca_idx = agent_id - 1;
        const nav_msgs::Odometry &odom = odom_caches_[idx].odom;
        orca_.setAgentState(orca_idx, odom.pose.pose.position.x, odom.pose.pose.position.y,
                            odom.twist.twist.linear.x, odom.twist.twist.linear.y);
    }

    orca_.setAgentGoal(params_.agent_id - 1, goal_point_.x, goal_point_.y);

    double vx = 0.0;
    double vy = 0.0;
    if (!orca_.GetOrcaVelCmd(params_.agent_id - 1, vx, vy))
    {
        return;
    }

    ugv_control_cmd_ = sunray_msgs::UGVControlCMD{};
    ugv_control_cmd_.header.stamp = ros::Time::now();
    ugv_control_cmd_.cmd_source = sunray_msgs::UGVControlCMD::SWARM_CONTROL;
    ugv_control_cmd_.control_cmd = sunray_msgs::UGVControlCMD::MOVE_VELOCITY;
    ugv_control_cmd_.desired_vel.x = vx;
    ugv_control_cmd_.desired_vel.y = vy;
    ugv_control_cmd_.desired_yaw = goal_point_.yaw;
    control_cmd_pub_.publish(ugv_control_cmd_);
}

void Swarm_Control_UGV::fillGoalPointFromSelfOdom(GoalPoint &goal_point) const
{
    const nav_msgs::Odometry &self_odom = odom_caches_[static_cast<size_t>(params_.agent_id)].odom;
    goal_point.x = self_odom.pose.pose.position.x;
    goal_point.y = self_odom.pose.pose.position.y;
    goal_point.z = self_odom.pose.pose.position.z;
    goal_point.yaw = getYawFromOdom(self_odom);
    goal_point.valid = true;
}

void Swarm_Control_UGV::switchToArrived(const HoldPointSource hold_point_source)
{
    dynamic_prepare_ready_time_ = ros::Time(0.0);

    if (hold_point_source == HoldPointSource::GOAL_POINT && goal_point_.valid)
    {
        hold_point_ = goal_point_;
    }
    else
    {
        fillGoalPointFromSelfOdom(hold_point_);
    }

    ugv_swarm_state_.fsm_state = sunray_msgs::UGVSwarmState::ARRIVED;
}

void Swarm_Control_UGV::publishHoldCommand()
{
    ugv_control_cmd_ = sunray_msgs::UGVControlCMD{};
    ugv_control_cmd_.header.stamp = ros::Time::now();
    ugv_control_cmd_.cmd_source = sunray_msgs::UGVControlCMD::SWARM_CONTROL;
    ugv_control_cmd_.control_cmd = sunray_msgs::UGVControlCMD::HOLD;
    control_cmd_pub_.publish(ugv_control_cmd_);
}

} // namespace swarm_control
