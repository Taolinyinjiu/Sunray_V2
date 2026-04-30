#pragma once

#include <algorithm>
#include <cstdint>
#include <string>

#include <sunray_msgs/UAVControlFSMState.h>
#include <sunray_msgs/UAVPlanningCMD.h>
#include <sunray_msgs/UAVPlanningState.h>

namespace sunray_planning {

inline std::string planning_frame_to_string(const uint8_t planning_frame) {
    switch (planning_frame) {
    case sunray_msgs::UAVPlanningState::SUNRAY_LOCAL:
        return "SUNRAY_LOCAL";
    case sunray_msgs::UAVPlanningState::SUNRAY_GLOBAL:
        return "SUNRAY_GLOBAL";
    default:
        return "UNDEFINE";
    }
}

inline std::string goal_type_to_string(const uint8_t goal_type) {
    switch (goal_type) {
    case sunray_msgs::UAVPlanningState::GOAL_SINGLE:
        return "GOAL_SINGLE";
    case sunray_msgs::UAVPlanningState::GOAL_MULTI:
        return "GOAL_MULTI";
    default:
        return "UNDEFINE";
    }
}

inline std::string cmd_source_to_string(const uint8_t cmd_source) {
    switch (cmd_source) {
    case sunray_msgs::UAVPlanningState::SUNRAY_STATION:
        return "SUNRAY_STATION";
    case sunray_msgs::UAVPlanningState::RC_CONTROLLER:
        return "RC_CONTROLLER";
    case sunray_msgs::UAVPlanningState::TERMINAL:
        return "TERMINAL";
    case sunray_msgs::UAVPlanningState::CONTROL_CMD:
        return "CONTROL_CMD";
    default:
        return "UNDEFINE";
    }
}

inline std::string planning_control_cmd_to_string(const uint8_t control_cmd) {
    switch (control_cmd) {
    case sunray_msgs::UAVPlanningCMD::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVPlanningCMD::LAND:
        return "LAND";
    case sunray_msgs::UAVPlanningCMD::RETURN:
        return "RETURN";
    case sunray_msgs::UAVPlanningCMD::KILL:
        return "KILL";
    case sunray_msgs::UAVPlanningCMD::HOVER:
        return "HOVER";
    case sunray_msgs::UAVPlanningCMD::PLANNING_LOCAL:
        return "PLANNING_LOCAL";
    case sunray_msgs::UAVPlanningCMD::PLANNING_GLOBAL:
        return "PLANNING_GLOBAL";
    case sunray_msgs::UAVPlanningCMD::UNDEFINE:
    default:
        return "UNDEFINE";
    }
}

inline std::string control_fsm_state_to_string(const uint8_t control_fsm_state) {
    switch (control_fsm_state) {
    case sunray_msgs::UAVControlFSMState::FSM_OFF:
        return "OFF";
    case sunray_msgs::UAVControlFSMState::FSM_INIT:
        return "INIT";
    case sunray_msgs::UAVControlFSMState::FSM_TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVControlFSMState::FSM_HOVER:
        return "HOVER";
    case sunray_msgs::UAVControlFSMState::FSM_RETURN:
        return "RETURN";
    case sunray_msgs::UAVControlFSMState::FSM_LAND:
        return "LAND";
    case sunray_msgs::UAVControlFSMState::FSM_MOVE:
        return "MOVE";
    case sunray_msgs::UAVControlFSMState::EMERGENCY_KILL:
        return "EMERGENCY_KILL";
    default:
        return "UNDEFINE";
    }
}

inline bool is_planning_goal_cmd(const uint8_t control_cmd) {
    return control_cmd == sunray_msgs::UAVPlanningCMD::PLANNING_LOCAL ||
           control_cmd == sunray_msgs::UAVPlanningCMD::PLANNING_GLOBAL;
}

inline uint8_t planning_cmd_to_frame(const uint8_t control_cmd) {
    switch (control_cmd) {
    case sunray_msgs::UAVPlanningCMD::PLANNING_LOCAL:
        return sunray_msgs::UAVPlanningState::SUNRAY_LOCAL;
    case sunray_msgs::UAVPlanningCMD::PLANNING_GLOBAL:
        return sunray_msgs::UAVPlanningState::SUNRAY_GLOBAL;
    default:
        return sunray_msgs::UAVPlanningState::UNDEFINE;
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
    return has_last_planning_cmd && is_planning_goal_cmd(planning_cmd.control_cmd) &&
           (task_active || task_arrived || hover_hold);
}

}  // namespace sunray_planning