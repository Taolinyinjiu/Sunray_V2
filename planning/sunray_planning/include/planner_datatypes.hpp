#pragma once

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

// planner的具体执行状态
enum class PlannerExecState : uint8_t {
    UNDEFINE = 0,       // 未定义状态
    INIT = 1,           // 初始化状态
    WAIT_TARGET = 2,    // 初始化完成，等待目标点
    GENERATE = 3,       // 轨迹生成
    REPLAN = 4,         // 轨迹重规划
    EXEC = 5,           // 执行轨迹
    PAUSE = 6,          // 暂停
    SUCCESS = 7,        // 成功 对应无人机到达目标点
    FAIL = 8,           // 失败，对应无人机无法到达目标点，比如规划失败，无可行路径
    EMERGENCY_STOP = 9  // 紧急停止
};

struct PlannerSnapshot {
    bool ready{false};
    bool goal_active{false};
    bool has_valid_output{false};
    PlannerExecState planner_state{PlannerExecState::UNDEFINE};
    std::string planner_state_string;
    ros::Time last_goal_stamp;
    ros::Time last_output_stamp;
};

struct PlanningTarget {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double yaw{0.0};
};