#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <sunray_planner_msgs/EgoPositionCommand.h>

#include "planner_interface.hpp"

class EgoPlanner : public PlannerInterface {
  public:
    bool send_goal(const PlanningTarget& target) override;

  private:
    void bind_topics(ros::NodeHandle& nh) override;
    void position_cmd_callback(const sunray_planner_msgs::EgoPositionCommand::ConstPtr& msg);

    ros::Publisher goal_pub_;
    ros::Subscriber position_cmd_sub_;
};
