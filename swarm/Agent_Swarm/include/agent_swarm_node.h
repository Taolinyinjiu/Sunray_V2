// 中文说明：AgentSwarm 节点入口与调度，负责订阅/发布、定时器与状态机驱动
#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <sunray_msgs/FormationOffsets.h>
#include <sunray_msgs/OrcaSetup.h>
#include <sunray_msgs/UAVControlFSMState.h>
#include <sunray_msgs/UAVSwarmCMD.h>
#include <sunray_msgs/UGVSwarmCMD.h>

#include "agent_state_cache.h"
#include "control_command_mapper.h"
#include "formation_policies.h"
#include "formation_policy_factory.h"
#include "formation_state_machine.h"
#include "goal_dispatcher.h"
#include "leader_tracker.h"
#include "orca_engine.h"

namespace agent_swarm
{

// AgentSwarm 入口节点
class AgentSwarmNode
{
  public:
    explicit AgentSwarmNode(ros::NodeHandle &nh);

  private:
    void onUavSwarmCmd(const sunray_msgs::UAVSwarmCMD::ConstPtr &msg);
    void onUgvSwarmCmd(const sunray_msgs::UGVSwarmCMD::ConstPtr &msg);
    // 目标点定时生成
    void goalTimerCb(const ros::TimerEvent &event);
    // 控制定时发布
    void controlTimerCb(const ros::TimerEvent &event);
    // 状态定时同步到本地 ORCA
    void statePublishTimerCb(const ros::TimerEvent &event);
    void selfOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
    void selfFsmStateCb(const sunray_msgs::UAVControlFSMState::ConstPtr &msg);
    void orcaSetupCb(const sunray_msgs::OrcaSetup::ConstPtr &msg, int idx);
    // Leader 目标点回调
    void leaderGoalCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    // 自定义阵型偏移量回调
    void formationOffsetsCb(const sunray_msgs::FormationOffsets::ConstPtr &msg);
    bool cacheHomeFromCurrentPose();
    void dispatchOrcaGoal(const geometry_msgs::Pose &target_pose, bool run_mode);
    void updateLocalOrca();
    bool localOrcaFresh() const;

    ros::NodeHandle nh_;

    int agent_id_{1};
    int agent_num_{1};
    int agent_type_{0};
    int leader_id_{1};
    std::string agent_name_{"uav"};
    std::string policy_name_{"ring"};
    double spacing_{1.0};
    bool use_fixed_altitude_{true};
    double fixed_altitude_{1.0};
    double spacing_min_{0.3};
    double spacing_max_{5.0};
    double spacing_scale_up_{1.2};
    double spacing_scale_down_{0.8};
    double leader_timeout_{1.0};
    double orca_timeout_{1.0};
    double goal_z_tolerance_{0.2};
    bool leader_publish_goal_{true};

    FormationStateMachine state_machine_{};
    LeaderTracker leader_tracker_{};
    FormationPolicyFactory policy_factory_{};
    std::shared_ptr<FormationPolicy> policy_{};
    std::shared_ptr<CustomPolicy> custom_policy_{};
    AgentStateCache state_cache_{};
    GoalDispatcher goal_dispatcher_{};
    orca_swarm::OrcaEngine orca_engine_{};
    ControlCommandMapper control_mapper_{};

    ros::Subscriber uav_swarm_cmd_sub_{};
    ros::Subscriber ugv_swarm_cmd_sub_{};
    ros::Subscriber formation_offsets_sub_{};
    ros::Subscriber self_odom_sub_{};
    ros::Subscriber self_fsm_state_sub_{};
    ros::Subscriber leader_goal_sub_{};
    std::map<int, ros::Subscriber> orca_setup_subs_{};
    ros::Timer goal_timer_{};
    ros::Timer control_timer_{};
    ros::Timer state_pub_timer_{};

    nav_msgs::Odometry self_odom_{};
    bool self_odom_ready_{false};
    sunray_msgs::UAVControlFSMState self_fsm_state_{};
    bool self_fsm_state_ready_{false};
    bool formation_change_active_{false};
    bool leader_goal_active_{false};
    bool return_home_active_{false};
    bool home_set_{false};
    geometry_msgs::Pose home_pose_{};
    geometry_msgs::Pose leader_goal_{};
    geometry_msgs::Pose last_goal_{};
    bool last_goal_valid_{false};
    sunray_msgs::OrcaCmd last_orca_cmd_{};
    ros::Time last_orca_cmd_stamp_{};
    bool has_orca_cmd_{false};
    SwarmState last_effective_state_{SwarmState::INIT};
    bool last_effective_state_valid_{false};
};

} // namespace agent_swarm
