/**
 * @file sunray_fsm.hpp
 * @brief 基本结构设计为，外部使用200Hz的ROS异步Spin 内部使用20Hz的Timer检查状态
 * @see
 *
 */

#pragma once

#include <ros/node_handle.h>
#include "sunray_fsm_param.hpp"
#include "sunray_state_types.hpp"

// Sunray_FSM指的是Sunray项目框架中的无人机控制状态机，然而由于我们希望Sunray项目的框架能够实现的，兼容或者说具有强的扩展性，
// 在实现的过程中，我们希望这个Sunray_FSM能够进行一定的抽象，因此SUnray_FSM与mavros进行解耦
// 也就说是Sunray_FSM只涉及到状态机的实现，不涉及到具体的无人机控制逻辑
// 为了达到这一点，我们得到这样的一些思路
// 1. 设计一个类用于检查无人机的状态，这样做的好处是将FSM与Mavros解耦
// 2. 设计一个类用于检查无人机的参数，这样做的好处是将FSM与Mavros解耦
// 设计一个类用于实现无人机的控制，这样做的好处是将FSM与Mavros解耦
// 这三点，全部交给controller来实现

class Sunray_FSM {
  public:
    explicit Sunray_FSM(ros::NodeHandle& nh);
    ~Sunray_FSM() = default;
    bool init();  // 包括配置读取，ros订阅+发布，初始化定时器
    void update();  // 状态机更新，将数据丢进控制器，或者根据当前state与controlcmd决定输出
    void show_logs();                // 在终端中打印各种log?0.0
    double get_update_freq() const;  // 得到控制器更新的频率

  private:
    // init() = load_param() + set_subscriber() + set_publisher()
    bool load_param();                                 // 读取参数
    void init_subscriber();                            // 初始化订阅者
    void init_publisher();                             // 初始化发布者
    bool handle_event(sunray_fsm::SunrayEvent event);  // 状态切换句柄，根据事件决定如何切换
    sunray_fsm::sunray_fsm_config_t fsm_config;  // 状态机参数结构体
};
