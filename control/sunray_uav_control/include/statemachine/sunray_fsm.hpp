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
#include "sunray_msgs/UAVControlCMD.h"
#include "sunray_msgs/OdomStatus.h"
#include <nav_msgs/Odometry.h>
#include "controller/controller_interface.hpp"
#include "control_data_types/uav_control_cmd_types.hpp"
// Sunray_FSM指的是Sunray项目框架中的无人机控制状态机，然而由于我们希望Sunray项目的框架能够实现的，兼容或者说具有强的扩展性，
// 在实现的过程中，我们希望这个Sunray_FSM能够进行一定的抽象，因此SUnray_FSM与mavros进行解耦
// 也就说是Sunray_FSM只涉及到状态机的实现，不涉及到具体的无人机控制逻辑
// 为了达到这一点，我们得到这样的一些思路
// 1. 设计一个类(mavros_helper)用于检查无人机的状态，这样做的好处是将FSM与Mavros解耦
// 2. 设计一个类(mavros_param)用于检查无人机的参数，这样做的好处是将FSM与Mavros解耦
// 设计一个类用于实现无人机的控制，这样做的好处是将FSM与Mavros解耦
// 这三点，全部交给controller来实现
// 整体状态流程
//
//
//  OFF
//   |
//   |      ->   检查状态  -(状态检查不通过) -> 返回(后续继续检查)
//   |              |
//   |              V
//(切换状态) <-  状态检查通过
//   |
//   V
//  INIT

class Sunray_FSM {
  public:
    explicit Sunray_FSM(ros::NodeHandle& nh);  // 构造Sunray_FSM状态机，需要传递ros句柄
    ~Sunray_FSM() = default;  // TODO: 析构函数，这里需要处理控制器线程

    void init();  // 包括配置读取，ros订阅+发布，初始化定时器
    void update();  // 状态机更新，将数据丢进控制器，或者根据当前state与controlcmd决定输出
    void show_logs();  // 在终端中打印各种log? 0.0

  private:
    // ------------------------- 表驱动FSM类型 -------------------------
    // using 类型别名，用于简化代码
    using GuardFn = std::function<bool()>;  // 守卫条件，用于判断本次状态转移是否允许发生
    using ActionFn = std::function<bool()>;  // 执行状态，在发生状态转移的时候执行的动作
    // 状态转移表结构体
    struct Transition {
        sunray_fsm::SunrayState from;   // 状态机当前状态
        sunray_fsm::SunrayEvent event;  // 发生的事件(control_cmd)
        sunray_fsm::SunrayState to;     // 状态机要转移到的状态
        GuardFn guard;                  // 判断是否允许转移
        ActionFn action;                // 转移成功后执行的命令
    };

    // ------------------------- FSM核心 -------------------------
    void init_transition_table();                             // 初始化状态转移表
    const std::vector<Transition>& get_transition_table();    // 获取状态转移表
    bool handle_event(sunray_fsm::SunrayEvent event);         // 状态转移句柄
    bool handle_global_event(sunray_fsm::SunrayEvent event);  // KILL等全局高优先级事件

    // ------------------------- 函数 -------------------------
    // init() = load_param() + set_subscriber() + set_publisher() + register_controller
    void load_param();           // 读取参数
    void init_subscriber();      // 初始化订阅者
    void init_publisher();       // 初始化发布者
    void register_controller();  // 注册控制器

    // 状态检查部分
    bool check_allow_takeoff();  // 检查是否允许起飞
    bool check_allow_move();     // 检查是否允许运动

    // --------------------------话题回调函数------------------------
    // 为了保持话题的高频回调，基本上回调函数都只负责将收到的消息转换为结构体变量缓存，不做其他处理
    void local_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);  // local系里程计回调函数
    void localization_state_callback(const sunray_msgs::OdomStatus& msg);  // 用于查看里程计状态
    void uav_control_cmd_callback(const sunray_msgs::UAVControlCMD& msg);  // 订阅控制指令话题
    void system_check_callback();  // 系统状态检查话题

    // --------------------------定时器回调函数-------------------------
    void
    pub_sunray_fsm_state();  // 发布状态机的相关信息，自定义话题类型，频率由config.yaml文件固定，从fsm_config_读取
    void pub_controller_state();  // 发布控制器的相关信息
    // -------------------------控制指令执行函数------------------
    bool update_controller_output(control_common::UavControlCmd control_cmd);
    // ------------------特殊指令，单次触发--------------
    // 这个由于还没想好，要不要把起飞降落的参数设置为可以通过话题的方式来修改，先设置为使用yaml文件的吧
    bool takeoff();
    bool land();
    bool hover();        // hover也没有参数，因为控制器的hover就没有参数
    bool return_home();  // return = move_point_local() + land()
    bool emergency_kill();
    // ------------------运动指令，持续触发--------------
    bool move_point_local(controller_data_types::TargetPoint_t point);
    bool move_velocity_local(controller_data_types::TargetVelocity_t velocity);
    bool move_velocity_body(controller_data_types::TargetVelocity_t velocity);
    bool move_trajectory(controller_data_types::TargetTrajectoryPoint_t trajpoint);
    // 发布到body系的pointpoint模式需要单独讨论
    bool move_point_body(controller_data_types::TargetPoint_t point);
    // 这个数据类型还没想好，先放一边吧
    bool move_point_wgs84();

    // ------------------------- 变量 ---------------------------
    // ROS句柄
    ros::NodeHandle nh_;
    // ROS订阅者
    ros::Subscriber local_odom_sub_;
    ros::Subscriber localization_state_sub_;
    ros::Subscriber uav_control_cmd_sub_;
    ros::Subscriber system_check_sub_;
    // ROS话题发布者
    ros::Publisher sunray_fsm_state_pub_;
    // ROS定时器
    ros::Timer sunray_fsm_state_timer_;
    // 控制器指针
    std::shared_ptr<Controller_Interface> sunray_controller_;  // 全局唯一的控制器实例
    // 里程计缓存
    control_common::UAVStateEstimate last_odometry_;
    // 最新的控制指令
    control_common::UavControlCmd last_control_cmd_;
    // 缓存相关状态
    bool allow_takeoff_{false};  // 允许起飞
    bool allow_move_{false};     // 允许移动
    bool is_flip_{false};        // 当前是否出现姿态大幅偏转
    bool is_fence_{false};       // 当前是否超过地理围栏/电子围栏的限制
    sunray_fsm::SunrayState fsm_state_{sunray_fsm::SunrayState::OFF};  // 当前状态
    sunray_fsm::sunray_fsm_config_t fsm_config_;                       // 状态机参数结构体
};
