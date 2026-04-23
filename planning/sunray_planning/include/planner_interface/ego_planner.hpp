#pragma once

#include "planner_interface.hpp"
#include <string>

class EgoPlanner : public Planner_Interface {
  public:
    void set_nodehandle(ros::NodeHandle& nh) override;
    bool load_param();
    void init() override;
    bool is_ready() const override;
    bool send_goal(const PlanningTarget& target) override;
    PlannerPositionCommand get_position_cmd() override;
    PlannerSnapshot get_snapshot() const override;

  private:
    static std::string load_uav_namespace_or_throw(ros::NodeHandle& nh);
    static std::string load_config_from_yaml(const std::string& yaml_path);

    ros::NodeHandle* nh_{nullptr};
    std::string config_yamlfile_path_;
    std::string uav_ns_;
    std::string position_cmd_topic_;
    PlanningTarget target_{};
    PlannerPositionCommand position_cmd_{};
    PlannerSnapshot snapshot_{};
};
