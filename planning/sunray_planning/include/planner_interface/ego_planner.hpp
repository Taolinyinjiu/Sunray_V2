#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <ego_planner_msgs/PositionCommand.h>

#include "planner_interface.hpp"

class EgoPlanner : public PlannerInterface {
  public:
    bool send_goal(const PlanningTarget& target) override;

  private:
    void bind_topics(ros::NodeHandle& nh) override;
    void position_cmd_callback(const ego_planner_msgs::PositionCommand::ConstPtr& msg);

    ros::Publisher goal_pub_;
    ros::Subscriber position_cmd_sub_;
};
