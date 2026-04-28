#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <sunray_planner_msgs/DiffPositionCommand.h>

#include "planner_interface.hpp"

class DiffPlanner : public PlannerInterface {
  public:
    bool send_goal(const PlanningTarget& target) override;
    PlannerType planner_type() const override;

  private:
    void bind_topics(ros::NodeHandle& nh) override;
    std::string default_goal_topic(const std::string& uav_ns) const override;
    std::string default_goal_frame_id() const override;
    std::string default_position_cmd_topic(const std::string& uav_ns) const override;
    void position_cmd_callback(const sunray_planner_msgs::DiffPositionCommand::ConstPtr& msg);

    ros::Publisher goal_pub_;
    ros::Subscriber position_cmd_sub_;
};
