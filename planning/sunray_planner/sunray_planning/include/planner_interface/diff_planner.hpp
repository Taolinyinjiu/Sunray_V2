#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sunray_planner_msgs/DiffPositionCommand.h>

#include "planner_interface.hpp"

class DiffPlanner : public PlannerInterface {
  public:
    void init(ros::NodeHandle& private_nh) override;
    bool send_goal(const PlanningTarget& target) override;
    bool get_planner_positioncmd(PlannerPositionCommand& cmd) override;
    bool is_ready() const override;
    PlannerSnapshot get_planner_state() const override;

  private:
    void position_cmd_callback(const sunray_planner_msgs::DiffPositionCommand::ConstPtr& msg);

    ros::Publisher goal_pub_;
    ros::Subscriber position_cmd_sub_;

    std::string uav_ns_;
    double cmd_timeout_sec_{0.3};
    bool ready_{false};
    PlannerSnapshot snapshot_;
    PlannerPositionCommand latest_cmd_;
};
