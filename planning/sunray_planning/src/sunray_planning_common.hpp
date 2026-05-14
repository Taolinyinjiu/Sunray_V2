#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlState.h>
#include <sunray_msgs/UAVPlanningCMD.h>
#include <sunray_msgs/UAVPlanningState.h>

namespace sunray_planning {

constexpr uint8_t PLANNING_FRAME_UNDEFINE = 0;
constexpr uint8_t PLANNING_FRAME_SUNRAY_LOCAL = 1;
constexpr uint8_t PLANNING_FRAME_SUNRAY_GLOBAL = 2;

inline std::string planning_frame_to_string(const uint8_t planning_frame) {
    switch (planning_frame) {
    case PLANNING_FRAME_SUNRAY_LOCAL:
        return "SUNRAY_LOCAL";
    case PLANNING_FRAME_SUNRAY_GLOBAL:
        return "SUNRAY_GLOBAL";
    default:
        return "UNDEFINE";
    }
}

inline std::string goal_type_to_string(const std::size_t waypoint_count) {
    switch (waypoint_count) {
    case 0:
        return "UNDEFINE";
    case 1:
        return "GOAL_SINGLE";
    default:
        return "GOAL_MULTI";
    }
}

inline std::string cmd_source_to_string(const std::string& cmd_source) {
    return cmd_source.empty() ? "UNDEFINE" : cmd_source;
}

inline std::string planning_cmd_to_string(const uint8_t plan_cmd) {
    switch (plan_cmd) {
    case sunray_msgs::UAVPlanningCMD::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVPlanningCMD::LAND:
        return "LAND";
    case sunray_msgs::UAVPlanningCMD::HOVER:
        return "HOVER";
    case sunray_msgs::UAVPlanningCMD::PLAN_RETURN:
        return "PLAN_RETURN";
    case sunray_msgs::UAVPlanningCMD::PLAN_LOCAL_GOAL:
        return "PLAN_LOCAL_GOAL";
    case sunray_msgs::UAVPlanningCMD::PLAN_GLOBAL_GOAL:
        return "PLAN_GLOBAL_GOAL";
    default:
        return "UNDEFINE";
    }
}

inline std::string control_fsm_state_to_string(const uint8_t control_fsm_state) {
    switch (control_fsm_state) {
    case sunray_msgs::UAVControlState::OFF:
        return "OFF";
    case sunray_msgs::UAVControlState::INIT:
        return "INIT";
    case sunray_msgs::UAVControlState::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVControlState::HOVER:
        return "HOVER";
    case sunray_msgs::UAVControlState::RETURN:
        return "RETURN";
    case sunray_msgs::UAVControlState::LAND:
        return "LAND";
    case sunray_msgs::UAVControlState::MOVE:
        return "MOVE";
    case sunray_msgs::UAVControlState::EMERGENCY_KILL:
        return "EMERGENCY_KILL";
    default:
        return "UNDEFINE";
    }
}

inline bool is_planning_goal_cmd(const uint8_t plan_cmd) {
    return plan_cmd == sunray_msgs::UAVPlanningCMD::PLAN_LOCAL_GOAL ||
           plan_cmd == sunray_msgs::UAVPlanningCMD::PLAN_GLOBAL_GOAL;
}

inline uint8_t planning_cmd_to_frame(const uint8_t plan_cmd) {
    switch (plan_cmd) {
    case sunray_msgs::UAVPlanningCMD::PLAN_LOCAL_GOAL:
        return PLANNING_FRAME_SUNRAY_LOCAL;
    case sunray_msgs::UAVPlanningCMD::PLAN_GLOBAL_GOAL:
        return PLANNING_FRAME_SUNRAY_GLOBAL;
    default:
        return PLANNING_FRAME_UNDEFINE;
    }
}

inline uint32_t clamp_waypoint_index(const sunray_msgs::UAVPlanningCMD& planning_cmd,
                                     const uint32_t waypoint_index) {
    if (planning_cmd.waypoints.empty()) {
        return 0;
    }
    return std::min<uint32_t>(waypoint_index,
                              static_cast<uint32_t>(planning_cmd.waypoints.size() - 1));
}

inline bool has_planning_context(const bool has_last_planning_cmd,
                                 const sunray_msgs::UAVPlanningCMD& planning_cmd,
                                 const bool task_active,
                                 const bool task_arrived,
                                 const bool hover_hold) {
    return has_last_planning_cmd && is_planning_goal_cmd(planning_cmd.plan_cmd) &&
           (task_active || task_arrived || hover_hold);
}

}  // namespace sunray_planning
