// 我希望这是一个基类，状态机持有，子类为不同planner的适配项
// 根据launch参数，决定实例化对应子类
// 首先这个接口描述的是planner的状态，简单地来说，这个接口实例化的对象，持有对planner发布话题的订阅者，然后总结他们的话题数据，然后转给fsm状态机

#pragma once

#include "../planner_datatypes.hpp"
#include "../planner_position_cmd.hpp"
#include <ros/time.h>
#include <ros/node_handle.h>

class Planner_Interface {
  public:
    // 析构函数
    virtual ~Planner_Interface() = default;
    // 加载ros句柄
    virtual void set_nodehandle(ros::NodeHandle& nh) { (void)nh; }
    // 初始化订阅者，发布者
    virtual void init() {}
    // 用于判断planner是否准备就绪
    virtual bool is_ready() const = 0;
    // 发送规划目标点
    virtual bool send_goal(const PlanningTarget& target) = 0;
    // 在每个适配器中进行操作，对于那些轨迹求解比较麻烦的，就不内置轨迹求解器
    virtual PlannerPositionCommand get_position_cmd() = 0;
    // 得到轨迹状态快照，用于发布state与log
    virtual PlannerSnapshot get_snapshot() const = 0;
};
