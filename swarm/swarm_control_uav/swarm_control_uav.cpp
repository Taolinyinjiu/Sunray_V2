/*
本文件功能：
    1、实现 Swarm_Control_UAV 的集群控制基础状态机
    2、实现集群指令接收、formation 目标点计算、ORCA 速度解算和控制指令输出
    3、定时发布 UAVSwarmState 供 UI 侧查看当前集群控制状态
*/
#include "swarm_control_uav.h"
#include "sunray_log.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace swarm_control
{

Swarm_Control_UAV::Swarm_Control_UAV(ros::NodeHandle &nh)
    : nh_(nh)
{
    // 1. 读取节点参数，确定本机编号、集群规模、控制频率、阵型参数和 ORCA 参数。
    // 基本参数
    nh_.param("uav_id", params_.uav_id, 1);
    nh_.param("swarm_num", params_.swarm_num, 1);
    nh_.param("uav_name", params_.uav_name, std::string("uav"));
    nh_.param("control_loop_hz", params_.control_loop_hz, 50.0 /*Hz*/);
    nh_.param("peer_odom_timeout", params_.peer_odom_timeout, 0.1 /*秒*/);
    // 目标点判断参数
    nh_.param("goal_xy_tolerance", params_.goal_xy_tolerance, 0.1 /*米*/);
    nh_.param("goal_z_tolerance", params_.goal_z_tolerance, 0.1 /*米*/);
    nh_.param("goal_yaw_tolerance", params_.goal_yaw_tolerance, 0.1 /*rad*/);
    nh_.param("goal_z_kp", params_.goal_z_kp, 1.0);
    nh_.param("goal_z_vel_limit", params_.goal_z_vel_limit, 0.8 /*米/秒*/);
    // 场地限制参数- 主要用于判定阵型目标点是否落在场地内部，否则无法生成阵型目标点
    nh_.param("field/x_min", params_.field_x_min, -50.0 /*米*/);
    nh_.param("field/x_max", params_.field_x_max, 50.0 /*米*/);
    nh_.param("field/y_min", params_.field_y_min, -50.0 /*米*/);
    nh_.param("field/y_max", params_.field_y_max, 50.0 /*米*/);
    nh_.param("field/z_min", params_.field_z_min, 0.0 /*米*/);
    nh_.param("field/z_max", params_.field_z_max, 3.0 /*米*/);
    // ORCA算法参数
    nh_.param("orca/neighbor_dist", params_.orca_neighbor_dist, 2.0 /*米*/);
    nh_.param("orca/time_horizon", params_.orca_time_horizon, 2.0 /*秒*/);
    nh_.param("orca/radius", params_.orca_radius, 0.2 /*米*/);
    nh_.param("orca/max_speed", params_.orca_max_speed, 1.0 /*米/秒*/);
    nh_.param("orca/max_neighbors", params_.orca_max_neighbors, -1);
    // 圆形静态障碍物参数，ORCA 用于避障，formation 用于目标点合法性检查。
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

    // 2. 为本机与其他成员创建里程计缓存，0 号位保留不用，索引与 agent_id 对齐。
    odom_caches_.assign(static_cast<size_t>(params_.swarm_num + 1), OdomCache{});

    // 3. 初始化纯算法模块：
    //    formation 负责根据虚拟领机和阵型参数生成目标点；
    //    ORCA 负责在编队运动时解算本机期望平面速度。
    formation_.init(params_.swarm_num, params_.orca_radius, params_.orca_max_speed, params_.field_x_min,
                    params_.field_x_max, params_.field_y_min, params_.field_y_max, params_.field_z_min,
                    params_.field_z_max);
    orca_.init(params_.swarm_num, static_cast<float>(params_.orca_neighbor_dist),
               static_cast<float>(params_.orca_time_horizon), static_cast<float>(params_.orca_radius),
               static_cast<float>(params_.orca_max_speed), static_cast<float>(1.0 / params_.control_loop_hz),
               params_.orca_max_neighbors);

    // 3.1 加载圆形静态障碍物，后续可由参数文件替换为地图/感知模块输入。
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

    // 4. 创建 ROS 通信接口。
    const std::string self_ns = "/" + params_.uav_name + std::to_string(params_.uav_id);
    const std::string local_odom_topic = self_ns + "/sunray/localization/local_odom";
    const std::string uav_fsm_state_topic = self_ns + "/sunray/fsm/state";
    const std::string control_cmd_topic = self_ns + "/sunray/uav_control_cmd";
    const std::string swarm_cmd_topic = "/sunray/swarm/uav_swarm_cmd";
    const std::string swarm_state_topic = "/sunray/swarm/uav_swarm_state";

    // 【订阅】无人机自身odom信息，localization_fusion_node -> 本节点
    local_odom_sub_ = nh_.subscribe(local_odom_topic, 20, &Swarm_Control_UAV::localOdomCallback, this);
    // 【订阅】无人机控制状态信息，uav_control_node -> 本节点
    uav_fsm_state_sub_ = nh_.subscribe(uav_fsm_state_topic, 20, &Swarm_Control_UAV::uavFsmStateCallback, this);

    // 【订阅】邻居无人机控制的odom信息，其他无人机localization_fusion_node -> 通信模块 -> 本节点
    peer_odom_subs_.clear();
    peer_odom_subs_.reserve(std::max(0, params_.swarm_num - 1));
    for (int agent_id = 1; agent_id <= params_.swarm_num; ++agent_id)
    {
        if (agent_id == params_.uav_id)
        {
            continue;
        }

        const std::string peer_topic =
            "/" + params_.uav_name + std::to_string(agent_id) + "/sunray/localization/local_odom";
        peer_odom_subs_.push_back(
            nh_.subscribe<nav_msgs::Odometry>(peer_topic, 20,
                                              [this, agent_id](const nav_msgs::Odometry::ConstPtr &msg) {
                                                  peerOdomCallback(msg, agent_id);
                                              }));
    }
    // 【订阅】无人机集群控制指令，外部模块（如地面站、Terminal控制节点） -> 本节点
    swarm_cmd_sub_ = nh_.subscribe<sunray_msgs::UAVSwarmCMD>(swarm_cmd_topic, 10, &Swarm_Control_UAV::swarmCmdCallback, this);
    // 【发布】无人机控制指令，本节点 -> uav_control_node
    control_cmd_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>(control_cmd_topic, 10);
    // 【发布】无人机集群控制状态，本节点 -> 其他节点（如地面站等）
    swarm_state_pub_ = nh_.advertise<sunray_msgs::UAVSwarmState>(swarm_state_topic, 10);

    // 【定时器】集群控制主循环，根据集群控制指令生成无人机控制指令并发布
    swarm_control_fsm_timer_ =
        nh_.createTimer(ros::Duration(1.0 / params_.control_loop_hz), &Swarm_Control_UAV::swarm_control_main_loop, this);
    // 【定时器】定时发布无人机集群控制状态
    swarm_state_pub_timer_ = nh_.createTimer(ros::Duration(0.1), &Swarm_Control_UAV::swarmStatePubTimerCallback, this);

    // 5. 打印初始化结果，便于确认参数加载、话题绑定和阵型/ORCA 基础配置是否正确。
    SUNRAY_INFO("========== swarm_control_uav init ==========");
    SUNRAY_INFO("uav_id={}", params_.uav_id);
    SUNRAY_INFO("swarm_num={}", params_.swarm_num);
    SUNRAY_INFO("local_odom_topic={}", local_odom_topic);
    SUNRAY_INFO("uav_fsm_state_topic={}", uav_fsm_state_topic);
    SUNRAY_INFO("swarm_cmd_topic={}", swarm_cmd_topic);
    SUNRAY_INFO("swarm_state_topic={}", swarm_state_topic);
    SUNRAY_INFO("control_cmd_topic={}", control_cmd_topic);
    SUNRAY_INFO("peer_odom_timeout={:.3f}", params_.peer_odom_timeout);
    SUNRAY_INFO("control_loop_hz={:.1f}", params_.control_loop_hz);
    SUNRAY_INFO("static_obstacles_enabled={}", params_.static_obstacles_enabled);
    SUNRAY_INFO("static_obstacles_num={}", params_.static_obstacle_x.size());
    for (size_t i = 0; i < params_.static_obstacle_x.size(); ++i)
    {
        SUNRAY_INFO("static_obstacle_{}=(x={:.2f}, y={:.2f}, r={:.2f})",
                    i + 1,
                    params_.static_obstacle_x[i],
                    params_.static_obstacle_y[i],
                    params_.static_obstacle_radius[i]);
    }
    SUNRAY_INFO("============================================");

    // 无人机集群状态初始化
    uav_swarm_state_.fsm_state = sunray_msgs::UAVSwarmState::INIT;
    uav_swarm_state_.agent_id = static_cast<uint8_t>(params_.uav_id);
    uav_swarm_state_.swarm_num = static_cast<uint32_t>(params_.swarm_num);
}

void Swarm_Control_UAV::swarm_control_main_loop(const ros::TimerEvent &)
{
    switch (uav_swarm_state_.fsm_state)
    {
    case sunray_msgs::UAVSwarmState::INIT:
        // do nothing
        break;

    case sunray_msgs::UAVSwarmState::TAKEOFF:
        // 无人机控制器已切换至FSM_HOVER，证明起飞成功，则将本程序状态机切换为ARRIVED
        if (uav_control_fsm_state_.sunray_fsm_state == sunray_msgs::UAVControlFSMState::FSM_HOVER)
        {
            // 记录当前位置为返航点
            fillGoalPointFromSelfOdom(return_point_);
            switchToArrived(HoverPointSource::CURRENT_ODOM);
        }
        break;

    case sunray_msgs::UAVSwarmState::LAND:
        // 无人机控制器已切换至FSM_INIT，证明降落成功，则将本程序状态机切换为INIT
        if (uav_control_fsm_state_.sunray_fsm_state == sunray_msgs::UAVControlFSMState::FSM_INIT)
        {
            uav_swarm_state_.fsm_state = sunray_msgs::UAVSwarmState::INIT;
        }
        break;

    case sunray_msgs::UAVSwarmState::RETURN_HOME:
        // 如果本机已经抵达目标点，则切换至ARRIVED悬停
        if (hasReachedGoal())
        {
            switchToArrived(HoverPointSource::GOAL_POINT);
            break;
        }
        // 将返航点设置为目标点
        goal_point_ = return_point_;
        // 调用ORCA算法，并发布无人机控制指令话题
        updateOrcaCommand();
        break;

    case sunray_msgs::UAVSwarmState::ARRIVED:
        uav_control_cmd_ = sunray_msgs::UAVControlCMD{};
        uav_control_cmd_.header.stamp = ros::Time::now();
        uav_control_cmd_.cmd_source = sunray_msgs::UAVControlCMD::SWARM_CONTROL;
        uav_control_cmd_.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
        uav_control_cmd_.control_cmd = sunray_msgs::UAVControlCMD::MOVE_POINT;
        uav_control_cmd_.desired_pos.x = hover_point_.x;
        uav_control_cmd_.desired_pos.y = hover_point_.y;
        uav_control_cmd_.desired_pos.z = hover_point_.z;
        uav_control_cmd_.desired_yaw = static_cast<float>(hover_point_.yaw);
        control_cmd_pub_.publish(uav_control_cmd_);
        break;

    case sunray_msgs::UAVSwarmState::SWARM_STATIC_FORMATION:
        // 如果本机已经抵达目标点，则切换至ARRIVED悬停
        if (hasReachedGoal())
        {
            switchToArrived(HoverPointSource::GOAL_POINT);
            break;
        }

        // 静态阵型：目标点固定或最终收敛到固定点，因此保留“到达后切换 ARRIVED”的逻辑。
        goal_point_.valid = formation_.GetFormationGoal(uav_swarm_cmd_.formation_cmd, params_.uav_id, 0.0,
                                                        goal_point_.x, goal_point_.y, goal_point_.z, goal_point_.yaw);

        if (goal_point_.valid)
        {
            updateOrcaCommand();
        }
        else
        {
            switchToArrived(HoverPointSource::CURRENT_ODOM);
        }
        break;

    case sunray_msgs::UAVSwarmState::SWARM_DYNAMIC_FORMATION:
    {
        const double elapsed_time = std::max(0.0, (ros::Time::now() - dynamic_formation_start_time_).toSec());
        // 超时则切换为悬停
        if (elapsed_time >= static_cast<double>(uav_swarm_cmd_.formation_cmd.dynamic_time))
        {
            switchToArrived(HoverPointSource::CURRENT_ODOM);
            break;
        }

        goal_point_.valid = formation_.GetFormationGoal(uav_swarm_cmd_.formation_cmd, params_.uav_id, elapsed_time,
                                                        goal_point_.x, goal_point_.y, goal_point_.z, goal_point_.yaw);
        if (goal_point_.valid)
        {
            updateOrcaCommand();
        }
        else
        {
            switchToArrived(HoverPointSource::CURRENT_ODOM);
        }
        break;
    }
    }
}

void Swarm_Control_UAV::localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    OdomCache &cache = odom_caches_[static_cast<size_t>(params_.uav_id)];
    cache.odom = *msg;
    cache.receive_time = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    cache.received = true;
}

void Swarm_Control_UAV::uavFsmStateCallback(const sunray_msgs::UAVControlFSMState::ConstPtr &msg)
{
    uav_control_fsm_state_ = *msg;
    has_uav_fsm_state_ = true;
}

void Swarm_Control_UAV::peerOdomCallback(const nav_msgs::Odometry::ConstPtr &msg, const int agent_id)
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

void Swarm_Control_UAV::swarmCmdCallback(const sunray_msgs::UAVSwarmCMD::ConstPtr &msg)
{
    // 接收集群控制指令前，必须要确保已经收到自身和邻居odom信息才可以
    if (!isReadyForSwarmCmd(*msg))
    {
        return;
    }

    // 判断ID是否符合
    if (msg->agent_id != 99U && msg->agent_id != static_cast<uint8_t>(params_.uav_id))
    {
        return;
    }

    sunray_msgs::UAVSwarmCMD received_cmd = *msg;
    if (received_cmd.formation_cmd.header.stamp.isZero())
    {
        received_cmd.formation_cmd.header.stamp = received_cmd.header.stamp;
    }

    switch (received_cmd.swarm_cmd)
    {
    case sunray_msgs::UAVSwarmCMD::SWARM_TAKEOFF:
        // 需要同时满足两个条件：当前状态为INIT，无人机控制状态机为INIT，才可以触发起飞指令
        if (uav_swarm_state_.fsm_state != sunray_msgs::UAVSwarmState::INIT ||
            uav_control_fsm_state_.sunray_fsm_state != sunray_msgs::UAVControlFSMState::FSM_INIT)
        {
            return;
        }
        // 状态切换为TAKEOFF
        uav_swarm_state_.fsm_state = sunray_msgs::UAVSwarmState::TAKEOFF;
        // 发布起飞的控制指令
        uav_control_cmd_ = sunray_msgs::UAVControlCMD{};
        uav_control_cmd_.header.stamp = ros::Time::now();
        uav_control_cmd_.cmd_source = sunray_msgs::UAVControlCMD::SWARM_CONTROL;
        uav_control_cmd_.control_cmd = sunray_msgs::UAVControlCMD::TAKEOFF;
        control_cmd_pub_.publish(uav_control_cmd_);
        break;

    case sunray_msgs::UAVSwarmCMD::SWARM_LAND:
        // swarm_control_fsm_state_为INIT和TAKEOFF时，不接收LAND指令
        if (uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::INIT ||
            uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::TAKEOFF)
        {
            return;
        }
        // 状态切换为LAND
        uav_swarm_state_.fsm_state = sunray_msgs::UAVSwarmState::LAND;
        // 发布降落的控制指令
        uav_control_cmd_ = sunray_msgs::UAVControlCMD{};
        uav_control_cmd_.header.stamp = ros::Time::now();
        uav_control_cmd_.cmd_source = sunray_msgs::UAVControlCMD::SWARM_CONTROL;
        uav_control_cmd_.control_cmd = sunray_msgs::UAVControlCMD::LAND;
        control_cmd_pub_.publish(uav_control_cmd_);
        break;

    case sunray_msgs::UAVSwarmCMD::SWARM_HOVER:
        if (uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::INIT ||
            uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::TAKEOFF ||
            uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::LAND)
        {
            return;
        }
        switchToArrived(HoverPointSource::CURRENT_ODOM);
        break;

    case sunray_msgs::UAVSwarmCMD::SWARM_RETURN:
        if (uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::INIT ||
            uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::TAKEOFF ||
            uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::LAND || !return_point_.valid)
        {
            return;
        }
        // 状态切换为RETURN_HOME
        uav_swarm_state_.fsm_state = sunray_msgs::UAVSwarmState::RETURN_HOME;
        break;

    case sunray_msgs::UAVSwarmCMD::SWARM_FORMATION:
    {
        if (uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::INIT ||
            uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::TAKEOFF ||
            uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::LAND ||
            uav_swarm_state_.fsm_state == sunray_msgs::UAVSwarmState::RETURN_HOME)
        {
            return;
        }

        // STATIC_KEEP_FORMATION : 在命令到来瞬间抓拍当前全集群相对位姿。
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
        candidate_goal.valid = formation_.GetFormationGoal(received_cmd.formation_cmd, params_.uav_id, 0.0, candidate_goal.x,
                                        candidate_goal.y, candidate_goal.z, candidate_goal.yaw);
        if (!candidate_goal.valid)
        {
            ROS_WARN_THROTTLE(2.0, "formation cmd rejected: can not compute valid goal point.");
            return;
        }

        if (isDynamicFormationType(received_cmd.formation_cmd.formation_type))
        {
            // 切换到SWARM_DYNAMIC_FORMATION，并记录动态阵型开始时间
            dynamic_formation_start_time_ = ros::Time::now();
            uav_swarm_state_.fsm_state = sunray_msgs::UAVSwarmState::SWARM_DYNAMIC_FORMATION;
        }
        else
        {
            // 切换到SWARM_STATIC_FORMATION
            uav_swarm_state_.fsm_state = sunray_msgs::UAVSwarmState::SWARM_STATIC_FORMATION;
        }
        break;
    }

    default:
        ROS_WARN_THROTTLE(2.0, "ignore swarm cmd: unsupported swarm_cmd=%d", static_cast<int>(received_cmd.swarm_cmd));
        return;
    }

    uav_swarm_cmd_ = received_cmd;
}

void Swarm_Control_UAV::swarmStatePubTimerCallback(const ros::TimerEvent &)
{
    const ros::Time now = ros::Time::now();

    uav_swarm_state_.header.stamp = now;
    uav_swarm_state_.agent_id = static_cast<uint8_t>(params_.uav_id);
    uav_swarm_state_.swarm_num = static_cast<uint32_t>(params_.swarm_num);
    uav_swarm_state_.swarm_cmd = uav_swarm_cmd_;

    uav_swarm_state_.self_odom_ready = hasLocalOdom();
    uav_swarm_state_.self_odom = nav_msgs::Odometry{};
    if (uav_swarm_state_.self_odom_ready)
    {
        uav_swarm_state_.self_odom = odom_caches_[static_cast<size_t>(params_.uav_id)].odom;
    }

    uav_swarm_state_.ready_peer_num = 0;
    uav_swarm_state_.peers_odom_ready = true;

    for (int agent_id = 1; agent_id <= params_.swarm_num; ++agent_id)
    {
        if (agent_id == params_.uav_id)
        {
            continue;
        }

        const OdomCache &cache = odom_caches_[static_cast<size_t>(agent_id)];
        const bool peer_ready = cache.received && ((now - cache.receive_time).toSec() <= params_.peer_odom_timeout);
        if (peer_ready)
        {
            ++uav_swarm_state_.ready_peer_num;
        }
        else
        {
            uav_swarm_state_.peers_odom_ready = false;
        }
    }

    uav_swarm_state_.target_valid = false;
    uav_swarm_state_.target_pos = geometry_msgs::Point{};
    uav_swarm_state_.target_yaw = 0.0f;

    const GoalPoint *target_point = nullptr;
    switch (uav_swarm_state_.fsm_state)
    {
    case sunray_msgs::UAVSwarmState::RETURN_HOME:
    case sunray_msgs::UAVSwarmState::SWARM_STATIC_FORMATION:
    case sunray_msgs::UAVSwarmState::SWARM_DYNAMIC_FORMATION:
        target_point = goal_point_.valid ? &goal_point_ : nullptr;
        break;

    case sunray_msgs::UAVSwarmState::ARRIVED:
        target_point = hover_point_.valid ? &hover_point_ : nullptr;
        break;

    default:
        break;
    }

    if (target_point != nullptr)
    {
        uav_swarm_state_.target_valid = true;
        uav_swarm_state_.target_pos.x = target_point->x;
        uav_swarm_state_.target_pos.y = target_point->y;
        uav_swarm_state_.target_pos.z = target_point->z;
        uav_swarm_state_.target_yaw = static_cast<float>(target_point->yaw);
    }

    uav_swarm_state_.uav_cmd = uav_control_cmd_;
    swarm_state_pub_.publish(uav_swarm_state_);
}

bool Swarm_Control_UAV::isValidAgentId(const int agent_id) const
{
    return agent_id >= 1 && agent_id <= params_.swarm_num;
}

bool Swarm_Control_UAV::isReadyForSwarmCmd(const sunray_msgs::UAVSwarmCMD &cmd) const
{
    if (!hasLocalOdom())
    {
        ROS_WARN_THROTTLE(2.0, "ignore swarm cmd: local odom is unavailable.");
        return false;
    }

    if (!has_uav_fsm_state_)
    {
        ROS_WARN_THROTTLE(2.0, "ignore swarm cmd: uav fsm state is unavailable.");
        return false;
    }

    const bool need_peer_odom = (cmd.swarm_cmd == sunray_msgs::UAVSwarmCMD::SWARM_FORMATION ||
                                 cmd.swarm_cmd == sunray_msgs::UAVSwarmCMD::SWARM_RETURN);
    if (!need_peer_odom)
    {
        return true;
    }

    const ros::Time now = ros::Time::now();
    for (int agent_id = 1; agent_id <= params_.swarm_num; ++agent_id)
    {
        if (agent_id == params_.uav_id)
        {
            continue;
        }

        const OdomCache &cache = odom_caches_[static_cast<size_t>(agent_id)];
        if (!cache.received)
        {
            ROS_WARN_THROTTLE(2.0, "ignore swarm cmd: peer odom is unavailable, missing_agent_id=%d", agent_id);
            return false;
        }

        const double odom_age = (now - cache.receive_time).toSec();
        if (odom_age > params_.peer_odom_timeout)
        {
            ROS_WARN_THROTTLE(2.0,
                              "ignore swarm cmd: peer odom timed out, timeout_agent_id=%d odom_age=%.3fs "
                              "timeout_threshold=%.3fs",
                              agent_id,
                              odom_age,
                              params_.peer_odom_timeout);
            return false;
        }
    }

    return true;
}

bool Swarm_Control_UAV::hasLocalOdom() const
{
    return isValidAgentId(params_.uav_id) && odom_caches_[static_cast<size_t>(params_.uav_id)].received;
}

double Swarm_Control_UAV::getYawFromOdom(const nav_msgs::Odometry &odom)
{
    const geometry_msgs::Quaternion &q = odom.pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double Swarm_Control_UAV::normalizeYaw(const double yaw)
{
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

double Swarm_Control_UAV::clamp(const double value, const double min_value, const double max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

bool Swarm_Control_UAV::isDynamicFormationType(const uint8_t formation_type)
{
    return formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_RING ||
           formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON ||
           formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE;
}

bool Swarm_Control_UAV::hasReachedGoal() const
{
    if (!goal_point_.valid)
    {
        return false;
    }

    const nav_msgs::Odometry &self_odom = odom_caches_[static_cast<size_t>(params_.uav_id)].odom;
    const double dx = goal_point_.x - self_odom.pose.pose.position.x;
    const double dy = goal_point_.y - self_odom.pose.pose.position.y;
    const double dz = goal_point_.z - self_odom.pose.pose.position.z;
    const double yaw_error = normalizeYaw(goal_point_.yaw - getYawFromOdom(self_odom));

    const double xy_error = std::sqrt(dx * dx + dy * dy);
    return xy_error <= params_.goal_xy_tolerance && std::abs(dz) <= params_.goal_z_tolerance &&
           std::abs(yaw_error) <= params_.goal_yaw_tolerance;
}

void Swarm_Control_UAV::updateOrcaCommand()
{
    // 1.ORCA算法更新当前状态 - 所有智能体的位置和速度
    for (int agent_id = 1; agent_id <= params_.swarm_num; ++agent_id)
    {
        const size_t idx = static_cast<size_t>(agent_id);
        const int orca_idx = agent_id - 1;

        // 本机与其他智能体都统一从 odom 缓存中读取状态。
        const nav_msgs::Odometry &odom = odom_caches_[idx].odom;
        orca_.setAgentState(orca_idx, odom.pose.pose.position.x, odom.pose.pose.position.y,
                            odom.twist.twist.linear.x, odom.twist.twist.linear.y);
    }

    // 2.ORCA算法更新当前状态 - 本机的目标点
    orca_.setAgentGoal(params_.uav_id - 1, goal_point_.x, goal_point_.y);

    // 3.从ORCA算法中获取速度
    double vx = 0.0;
    double vy = 0.0;
    if (!orca_.GetOrcaVelCmd(params_.uav_id - 1, vx, vy))
    {
        return;
    }

    // 4.发布无人机控制指令，目前z轴采用P控制计算速度
    const nav_msgs::Odometry &self_odom = odom_caches_[static_cast<size_t>(params_.uav_id)].odom;
    const double z_error = goal_point_.z - self_odom.pose.pose.position.z;
    const double vz = clamp(z_error * params_.goal_z_kp, -params_.goal_z_vel_limit, params_.goal_z_vel_limit);

    uav_control_cmd_ = sunray_msgs::UAVControlCMD{};
    uav_control_cmd_.header.stamp = ros::Time::now();
    uav_control_cmd_.cmd_source = sunray_msgs::UAVControlCMD::SWARM_CONTROL;
    uav_control_cmd_.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    uav_control_cmd_.control_cmd = sunray_msgs::UAVControlCMD::MOVE_VELOCITY;
    uav_control_cmd_.desired_vel.x = vx;
    uav_control_cmd_.desired_vel.y = vy;
    uav_control_cmd_.desired_vel.z = vz;
    uav_control_cmd_.desired_yaw = static_cast<float>(goal_point_.yaw);
    control_cmd_pub_.publish(uav_control_cmd_);
}

void Swarm_Control_UAV::fillGoalPointFromSelfOdom(GoalPoint &goal_point) const
{
    const nav_msgs::Odometry &self_odom = odom_caches_[static_cast<size_t>(params_.uav_id)].odom;
    goal_point.x = self_odom.pose.pose.position.x;
    goal_point.y = self_odom.pose.pose.position.y;
    goal_point.z = self_odom.pose.pose.position.z;
    goal_point.yaw = getYawFromOdom(self_odom);
    goal_point.valid = true;
}

void Swarm_Control_UAV::switchToArrived(const HoverPointSource hover_point_source)
{
    if (hover_point_source == HoverPointSource::GOAL_POINT && goal_point_.valid)
    {
        hover_point_ = goal_point_;
    }
    else
    {
        fillGoalPointFromSelfOdom(hover_point_);
    }

    uav_swarm_state_.fsm_state = sunray_msgs::UAVSwarmState::ARRIVED;
}

} // namespace swarm_control
