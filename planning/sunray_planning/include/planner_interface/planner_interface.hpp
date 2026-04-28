#pragma once

#include <ros/node_handle.h>
#include <sunray_msgs/UAVPlanningState.h>

#include "../planner_datatypes.hpp"
#include "planner_position_cmd.hpp"

class PlannerInterface {
  public:
    virtual ~PlannerInterface() = default;

    virtual void init(ros::NodeHandle& nh, const std::string& uav_ns);
    virtual bool send_goal(const PlanningTarget& target) = 0;
    virtual PlannerType planner_type() const = 0;

    bool is_ready() const;
    bool fetch_latest_position_cmd(PlannerPositionCommand& cmd, const ros::Time& now) const;
    PlannerSnapshot get_state(const ros::Time& now = ros::Time()) const;
    const std::string& goal_topic() const;
    const std::string& goal_frame_id() const;
    const std::string& position_cmd_topic() const;
    const std::string& planner_state_topic() const;

  protected:
    virtual void bind_topics(ros::NodeHandle& nh) = 0;
    virtual std::string default_goal_topic(const std::string& uav_ns) const = 0;
    virtual std::string default_goal_frame_id() const;
    virtual std::string default_position_cmd_topic(const std::string& uav_ns) const = 0;
    virtual std::string default_planner_state_topic(const std::string& uav_ns) const;
    virtual double default_cmd_timeout_sec() const;
    virtual double default_state_timeout_sec() const;

    void set_latest_position_cmd(const PlannerPositionCommand& cmd);
    void set_planner_state(PlannerExecState planner_state, const ros::Time& stamp);
    void mark_goal_sent(const ros::Time& stamp);
    void unified_state_callback(const sunray_msgs::UAVPlanningState::ConstPtr& msg);

    std::string uav_ns_;
    std::string goal_topic_;
    std::string goal_frame_id_;
    std::string position_cmd_topic_;
    std::string planner_state_topic_;
    double cmd_timeout_sec_{0.3};
    double state_timeout_sec_{1.0};
    PlannerSnapshot snapshot_;
    PlannerPositionCommand latest_position_cmd_;
    ros::Subscriber unified_state_sub_;
};
