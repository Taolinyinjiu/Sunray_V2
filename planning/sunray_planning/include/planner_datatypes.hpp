#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "../src/sunray_planning_common.hpp"

enum class PlannerType : uint8_t {
    UNDEFINE = 0,
    EGO = 1,
    DIFF = 2,
    FUEL = 3,
    SUPER = 4
};

enum class PlannerExecState : uint8_t {
    UNDEFINE = 0,
    INIT = 1,
    WAIT_TARGET = 2,
    GENERATE = 3,
    REPLAN = 4,
    EXEC = 5,
    PAUSE = 6,
    SUCCESS = 7,
    FAIL = 8,
    EMERGENCY_STOP = 9
};

enum class PlanningFsmState : uint8_t {
    INIT = 0,
    READY = 1,
    PLANNING = 2,
    ARRIVED = 3,
    HOVER = 4,
    LAND = 5,
    TAKEOFF = 6,
    RETURN = 7,
    MOVE = 8,
    EMERGENCY_KILL = 9
};

struct PlannerSnapshot {
    bool ready{false};
    bool goal_active{false};
    bool has_valid_output{false};
    PlannerType planner_type{PlannerType::UNDEFINE};
    PlannerExecState planner_state{PlannerExecState::UNDEFINE};
    std::string planner_state_string{"UNDEFINE"};
    uint32_t current_waypoint_index{0};
    ros::Time last_goal_stamp;
    ros::Time last_output_stamp;
    ros::Time last_state_stamp;
};

struct PlanningTarget {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    uint8_t planning_frame{sunray_planning::PLANNING_FRAME_SUNRAY_LOCAL};
    std::string cmd_source{"UNDEFINE"};
    uint32_t waypoint_count{1};
    uint32_t waypoint_index{0};
};

inline std::string normalize_planner_type(const std::string& planner_type) {
    std::string normalized = planner_type;
    std::transform(normalized.begin(),
                   normalized.end(),
                   normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return normalized;
}

inline PlannerType planner_type_from_string(const std::string& planner_type) {
    const std::string normalized = normalize_planner_type(planner_type);
    if (normalized == "ego") {
        return PlannerType::EGO;
    }
    if (normalized == "diff") {
        return PlannerType::DIFF;
    }
    if (normalized == "fuel") {
        return PlannerType::FUEL;
    }
    if (normalized == "super") {
        return PlannerType::SUPER;
    }
    return PlannerType::UNDEFINE;
}

inline std::string planner_type_to_string(const PlannerType planner_type) {
    switch (planner_type) {
    case PlannerType::EGO:
        return "EGO";
    case PlannerType::DIFF:
        return "DIFF";
    case PlannerType::FUEL:
        return "FUEL";
    case PlannerType::SUPER:
        return "SUPER";
    case PlannerType::UNDEFINE:
    default:
        return "UNDEFINE";
    }
}

inline std::string planner_exec_state_to_string(const PlannerExecState planner_state) {
    switch (planner_state) {
    case PlannerExecState::INIT:
        return "INIT";
    case PlannerExecState::WAIT_TARGET:
        return "WAIT_TARGET";
    case PlannerExecState::GENERATE:
        return "GENERATE";
    case PlannerExecState::REPLAN:
        return "REPLAN";
    case PlannerExecState::EXEC:
        return "EXEC";
    case PlannerExecState::PAUSE:
        return "PAUSE";
    case PlannerExecState::SUCCESS:
        return "SUCCESS";
    case PlannerExecState::FAIL:
        return "FAIL";
    case PlannerExecState::EMERGENCY_STOP:
        return "EMERGENCY_STOP";
    case PlannerExecState::UNDEFINE:
    default:
        return "UNDEFINE";
    }
}

inline std::string planning_fsm_state_to_string(const PlanningFsmState fsm_state) {
    switch (fsm_state) {
    case PlanningFsmState::INIT:
        return "INIT";
    case PlanningFsmState::READY:
        return "READY";
    case PlanningFsmState::PLANNING:
        return "PLANNING";
    case PlanningFsmState::ARRIVED:
        return "ARRIVED";
    case PlanningFsmState::HOVER:
        return "HOVER";
    case PlanningFsmState::LAND:
        return "LAND";
    case PlanningFsmState::TAKEOFF:
        return "TAKEOFF";
    case PlanningFsmState::RETURN:
        return "RETURN";
    case PlanningFsmState::MOVE:
        return "MOVE";
    case PlanningFsmState::EMERGENCY_KILL:
        return "EMERGENCY_KILL";
    default:
        return "UNDEFINE";
    }
}
