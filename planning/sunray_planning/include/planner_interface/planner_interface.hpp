#pragma once

#include <ros/node_handle.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVPlanningState.h>

#include "../planner_datatypes.hpp"

class PlannerInterface {
  public:
    virtual ~PlannerInterface() = default;

    virtual void init(ros::NodeHandle& nh, const PlannerRuntimeConfig& config);
    virtual bool send_goal(const PlanningTarget& target) = 0;

    bool is_ready() const;
    bool fetch_latest_control_cmd(sunray_msgs::UAVControlCMD& cmd, const ros::Time& now) const;
    PlannerSnapshot get_state() const;
    const PlannerRuntimeConfig& get_config() const;

  protected:
    virtual void bind_topics(ros::NodeHandle& nh) = 0;

    void set_latest_control_cmd(const sunray_msgs::UAVControlCMD& cmd);
    void set_planner_state(PlannerExecState planner_state, const ros::Time& stamp);
    void mark_goal_sent(const ros::Time& stamp);
    void unified_state_callback(const sunray_msgs::UAVPlanningState::ConstPtr& msg);

    PlannerRuntimeConfig config_;
    PlannerSnapshot snapshot_;
    sunray_msgs::UAVControlCMD latest_cmd_;
    ros::Subscriber unified_state_sub_;
};
