#pragma once

#include <memory>

#include <ros/ros.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlFSMState.h>
#include <sunray_msgs/UAVPlanningCMD.h>
#include <sunray_msgs/UAVPlanningState.h>

#include "planner_datatypes.hpp"
#include "planner_interface/planner_interface.hpp"

class PlanningFSM {
  public:
    PlanningFSM(ros::NodeHandle& nh);
    ~PlanningFSM() = default;

    void init();
    void process();
    void printf_terminal();

  private:
    void load_param();
    void init_logger();
    bool is_ready();
    void pub_planning_state();
    void pub_control_cmd();
    void planning_cmd_callback(const sunray_msgs::UAVPlanningCMD::ConstPtr& msg);
    void control_fsm_state_callback(const sunray_msgs::UAVControlFSMState::ConstPtr& msg);
    void process_timer_callback(const ros::TimerEvent& event);
    void planning_state_timer_callback(const ros::TimerEvent& event);
    std::string make_log_file_path() const;
    bool has_fresh_control_fsm_state(const ros::Time& now) const;
    bool control_fsm_allows_planning_goal(const ros::Time& now, std::string& reason) const;
    PlanningFsmState effective_fsm_state(const ros::Time& now) const;
    sunray_msgs::UAVControlCMD build_trajectory_control_cmd(const PlannerPositionCommand& planner_cmd,
                                                            uint8_t cmd_source) const;

    sunray_msgs::UAVControlCMD build_special_control_cmd(uint8_t control_cmd,
                                                         const PlannerSnapshot& snapshot,
                                                         uint8_t cmd_source =
                                                             sunray_msgs::UAVControlCMD::CONTROL_CMD) const;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::unique_ptr<PlannerInterface> planner_;

    ros::Subscriber planning_cmd_sub_;
    ros::Subscriber control_fsm_state_sub_;
    ros::Publisher control_pub_;
    ros::Publisher planning_state_pub_;
    ros::Timer process_timer_;
    ros::Timer planning_state_timer_;

    std::string selected_planner_type_;
    std::string uav_ns_;
    std::string planning_cmd_sub_topic_;
    std::string control_pub_topic_;
    std::string planning_state_pub_topic_;
    std::string control_fsm_state_sub_topic_;

    double process_rate_hz_{50.0};
    double state_pub_rate_hz_{10.0};
    double control_fsm_state_timeout_sec_{1.0};
    bool auto_hover_on_timeout_{true};
    bool follow_control_fsm_{true};
    bool log_save_{false};

    PlanningFsmState fsm_state_{PlanningFsmState::INIT};
    bool task_active_{false};
    bool task_arrived_{false};
    bool hover_hold_{false};
    bool control_fsm_trajectory_ack_{false};
    uint8_t passthrough_control_cmd_{sunray_msgs::UAVControlCMD::UNDEFINE};
    uint32_t task_id_{0};
    bool has_control_fsm_state_{false};

    bool has_last_planning_cmd_{false};
    sunray_msgs::UAVPlanningCMD last_planning_cmd_;
    sunray_msgs::UAVControlFSMState last_control_fsm_state_;
    PlanningTarget active_target_;

    ros::Time last_terminal_log_stamp_;
    ros::Time last_control_fsm_state_stamp_;
    ros::Time last_goal_accept_stamp_;
    std::string log_file_path_;
};
