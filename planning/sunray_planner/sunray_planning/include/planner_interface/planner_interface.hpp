#pragma once

#include <ros/node_handle.h>

#include "../planner_datatypes.hpp"
#include "../planner_position_cmd.hpp"

class PlannerInterface {
  public:
    virtual ~PlannerInterface() = default;

    // 初始化：基于 agent_key 绑定 planner 相关话题
    virtual void init(ros::NodeHandle& private_nh) = 0;

    // 向 planner 发送目标点
    virtual bool send_goal(const PlanningTarget& target) = 0;

    // 获取 planner 最新输出的 position_cmd；无有效输出时返回 false
    virtual bool get_planner_positioncmd(PlannerPositionCommand& cmd) = 0;

    // planner 是否已就绪
    virtual bool is_ready() const = 0;

    // 获取 planner 当前状态快照
    virtual PlannerSnapshot get_planner_state() const = 0;
};
