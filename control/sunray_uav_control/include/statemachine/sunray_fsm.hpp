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
#include <queue>
#include <thread>
#include <atomic>
#include <mutex>
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
    ~Sunray_FSM();  // TODO: 析构函数，这里需要处理控制器线程

    void init();                    // 包括配置读取，ros订阅+发布，初始化定时器
    double get_update_frequency();  // 获取更新频率
    void process();                 // 状态机低频更新，处理事件队列
    void show_logs();               // 在终端中打印各种log? 0.0

  private:
    // ------------------------- 表驱动FSM类型 -------------------------

    // ------------------------- FSM核心 -------------------------
    void init_transition_table();  // 初始化状态转移表
    const std::vector<sunray_fsm::Transition>&
    get_transition_table();  // 获取状态转移表，请注意，每一条状态转移的规则，对应一个结构体，所有结构体加起来，对应一个容器，因此使用vecotr作为返回类型
    bool handle_event(sunray_fsm::SunrayEvent event);  // 状态转移句柄
    bool handle_global_event(
        sunray_fsm::SunrayEvent
            event);  // KILL等全局高优先级事件(实际上目前只有kill一个事件在这里被处理)

    // ------------------------- 函数 -------------------------
    // init() = load_param() + set_subscriber() + set_publisher() + register_controller
    void load_param();           // 读取参数
    void init_subscriber();      // 初始化订阅者
    void init_publisher();       // 初始化发布者
    void register_controller();  // 注册控制器

    // 状态检查部分
    void check_controller_ready();  // 检查controller是否就绪，就绪则Sunray_FSM State: OFF -> INIT
    bool check_allow_takeoff();   // 检查是否允许起飞
    void check_rosmsg_timeout();  // 检查是否存在消息超时
    // bool check_allow_move();     // 检查是否允许运动,这个需要再细想一下

    // --------------------------话题回调函数------------------------
    // 为了保持话题的高频回调，基本上回调函数都只负责将收到的消息转换为结构体变量缓存，不做其他处理
    void local_odom_callback(const nav_msgs::Odometry& msg);  // local系里程计回调函数
    void localization_state_callback(const sunray_msgs::OdomStatus& msg);  // 用于查看里程计状态
    void uav_control_cmd_callback(const sunray_msgs::UAVControlCMD& msg);  // 订阅控制指令话题
    void system_check_callback();  // 系统状态检查话题

    // --------------------------定时器回调函数-------------------------
    void
    pub_sunray_fsm_state();  // 发布状态机的相关信息，自定义话题类型，频率由config.yaml文件固定，从fsm_config_读取
    void pub_controller_state();  // 发布控制器的相关信息
    // -------------------------控制指令执行函数------------------
    void controller_update_loop();  // 控制线程主循环
    void update_controller_output();
    // ------------------特殊指令，单次触发--------------
    // 这个由于还没想好，要不要把起飞降落的参数设置为可以通过话题的方式来修改，先设置为使用yaml文件的吧
    bool takeoff();
    bool land();
    bool hover();        // hover也没有参数，因为控制器的hover就没有参数
    bool return_home();  // return = move_point_local() + land()
    bool emergency_kill();
    // ------------------运动指令，持续触发--------------
    bool move_point(control_common::UavControlCmd cmd);
    bool move_velocity(control_common::UavControlCmd cmd);
    bool move_trajectory(control_common::UavControlCmd cmd);
    bool move_point_wgs84(control_common::UavControlCmd cmd);

    // 更新home点
    bool update_home_point();
    void check_move_completed();
    // -----------------------状态机事件入口函数---------------------
    // 向状态机事件队列中填入数据
    void enqueue_fsm_event(sunray_fsm::SunrayEvent input_event);
    // 处理状态机事件队列中的事件
    void process_fsm_event_queue();
    // ------------------------- 变量 ---------------------------
    // 无人机相关信息
    std::string uav_ns_;
    // ROS句柄
    ros::NodeHandle nh_;
    // ROS订阅者
    ros::Subscriber local_odom_sub_;
    ros::Subscriber localization_state_sub_;
    ros::Subscriber uav_control_cmd_sub_;
    ros::Subscriber system_check_sub_;
    // ROS话题发布者
    ros::Publisher sunray_fsm_state_pub_;

    // 控制器指针
    std::shared_ptr<Controller_Interface> sunray_controller_;  // 全局唯一的控制器实例
    // 里程计缓存
    control_common::UAVStateEstimate last_odometry_;
    // 最新的控制指令
    control_common::UavControlCmd last_control_cmd_;
    // 缓存相关状态
    bool allow_takeoff_{false};  // 允许起飞
    bool is_flip_{false};        // 当前是否出现姿态大幅偏转
    bool is_fence_{false};       // 当前是否超过地理围栏/电子围栏的限制

    Eigen::Vector3d home_point_{Eigen::Vector3d::Zero()};

    // 用于判断状态or切换状态的变量
    bool controller_ready_{false};
    bool control_msg_lost_{false};
    // 控制器循环的线程相关
    std::thread controller_update_thread_;
    std::atomic<bool> stop_controller_thread_{false};
    // 由于使用线程，因此给使用到的变量加一下锁
    mutable std::mutex state_mutex_;  // fsm当前状态
    mutable std::mutex odom_mutex_;
    mutable std::mutex cmd_mutex_;
    mutable std::mutex event_mutex_;  // fsm当前事件队列

    sunray_fsm::SunrayState fsm_state_{sunray_fsm::SunrayState::OFF};  // 当前状态
    std::queue<sunray_fsm::SunrayEvent> fsm_event_queue_;              // 事件队列
    sunray_fsm::sunray_fsm_config_t fsm_config_;                       // 状态机参数结构体
    std::vector<sunray_fsm::Transition> sunray_state_transmit_table_;  // 成员变量
    bool rc_connected{false};  // 遥控器连接状态，这里是从sunray_rc_joy_node节点传递
};
