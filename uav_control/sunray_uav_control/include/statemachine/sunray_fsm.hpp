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
#include <nav_msgs/Odometry.h>
#include "controller/controller_interface.hpp"
#include "control_data_types/uav_control_cmd_types.hpp"
#include "odom_filter/odom_kalman_filter.hpp"
#include <deque>
#include <queue>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>

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

  private:
    // ------------------------- 表驱动FSM类型 -------------------------

    struct QueuedFsmEvent_t {
        sunray_fsm::SunrayEvent event{sunray_fsm::SunrayEvent::CONTROLLER_READY};
        bool has_control_cmd{false};
        control_common::UavControlCmd control_cmd;
    };

    // ------------------------- FSM核心 -------------------------
    // 初始化状态转移表
    void init_transition_table();  
    // 获取状态转移表，表中的每条规则描述一个 current_state + event 到 target_state 的转移条件和转移动作。
    const std::vector<sunray_fsm::Transition>& get_transition_table();
    // 处理一个已入队的FSM事件,参数为命令快照
    bool handle_event(const QueuedFsmEvent_t& queued_event);  
    // 处理KILL等全局高优先级事件,参数为命令快照
    bool handle_global_event(const QueuedFsmEvent_t& queued_event);  

    // ------------------------- 函数 -------------------------
    // 初始化部分
    // init() = load_param() + init_subscriber() + init_publisher() + init_timer() + init_controller
    void load_param();           // 读取参数
    void init_subscriber();      // 初始化订阅者
    void init_publisher();       // 初始化发布者
    void init_timer();           // 初始化定时器
    void init_controller();      // 初始化无人机控制器

    // 状态检查部分
    // 1. 检查controller是否就绪，就绪则Sunray_FSM State: OFF -> INIT
    void check_controller_ready();  
    // 2. 检查是否允许起飞：check{起飞高度与速度，起飞权限，RC连接...}
    bool check_allow_takeoff(double relative_takeoff_height, double max_takeoff_velocity);
    // 3. 检查是否存在消息超时
    void check_rosmsg_timeout();  
    
    // 里程计滤波处理部分 TODO:使用小状态卡尓曼滤波器估计滤波，或者a-b滤波器
    bool validate_odometry_sample(const control_common::UAVStateEstimate& odom,
                                  std::string* invalid_reason = nullptr) const;
    bool has_fresh_valid_odometry() const;
    bool get_latest_valid_odometry(control_common::UAVStateEstimate& odom) const;

    // 从最新的里程计数据设置悬停点
    bool set_hover_point_from_latest_odom();
    
    // POINT_COMPLETED 后专用：优先把悬停点设为最近一次 move_point 目标，
    // 控制器没有可用目标时回退到 set_hover_point_from_latest_odom()。
    bool set_hover_point_to_target_or_odom();
    
    // 从起飞命令求解参数？
    bool resolve_takeoff_command_params(const control_common::UavControlCmd& cmd,
                                        double* relative_takeoff_height,
                                        double* max_takeoff_velocity,
                                        std::string* reject_reason = nullptr) const;
    double effective_land_max_velocity(const control_common::UavControlCmd& cmd) const;

    // --------------------------话题回调函数------------------------
    // 为了保持话题的高频回调，基本上回调函数都只负责将收到的消息转换为结构体变量缓存，不做其他处理
    void local_odom_callback(const nav_msgs::Odometry& msg);  // local系里程计回调函数
    void uav_control_cmd_callback(const sunray_msgs::UAVControlCMD& msg);  // 订阅控制指令话题

    // --------------------------定时器回调函数-------------------------
    void
    pub_sunray_fsm_state();  // 发布状态机的相关信息，自定义话题类型，频率由config.yaml文件固定，从fsm_config_读取
    // -------------------------控制指令执行函数------------------
    void controller_update_loop();  // 控制线程主循环
    void update_controller_output();

    // 更新home点
    bool update_home_point();
    void check_move_completed();
    
    // -----------------------状态机事件入口函数---------------------
    // 内部状态事件：由控制器状态、完成条件、超时检查等内部流程触发，不携带新的控制命令。
    void enqueue_fsm_event(sunray_fsm::SunrayEvent input_event);
    // 外部命令事件：由UAVControlCMD触发，携带触发本次转移的命令快照，供guard/action/状态提交使用。
    void enqueue_fsm_event(sunray_fsm::SunrayEvent input_event,
                           const control_common::UavControlCmd& control_cmd);
    // 处理状态机事件队列中的事件
    void process_fsm_event_queue();
    // ------------------------- 变量 ---------------------------
    // 无人机相关信息
    std::string uav_ns_;
    std::string agent_name_{"uav"};
    int agent_id_{1};
    // ROS句柄
    ros::NodeHandle nh_;
    // ROS订阅者
    ros::Subscriber local_odom_sub_;
    ros::Subscriber uav_control_cmd_sub_;
    // ROS话题发布者
    ros::Publisher sunray_fsm_state_pub_;
    ros::Publisher sunray_odom_debug_pub_;
    // 控制器指针
    std::shared_ptr<Controller_Interface> sunray_controller_;  // 全局唯一的控制器实例
    // 里程计缓存
    control_common::OdomKalmanFilter odom_kf_;
    control_common::UAVStateEstimate last_raw_odometry_;
    // 控制闭环使用的里程计。启用 odom_filter 后该字段为 filtered odom。
    control_common::UAVStateEstimate last_odometry_;
    nav_msgs::Odometry last_self_odom_msg_;
    ros::Time last_raw_odom_receive_time_{ros::Time(0)};
    ros::Time last_valid_odom_receive_time_{ros::Time(0)};
    double odom_frequency_hz_{0.0};
    bool odom_meets_rate_target_{false};
    bool has_self_odom_msg_{false};
    bool has_valid_odometry_{false};
    // 最新的控制指令
    control_common::UavControlCmd last_control_cmd_;
    control_common::UavControlCmd transition_control_cmd_;
    bool has_transition_control_cmd_{false};
    std::atomic<double> active_takeoff_relative_height_{0.0};
    std::atomic<double> active_takeoff_max_velocity_{0.0};
    std::atomic<double> active_land_max_velocity_{0.0};
    // 注意到当设置为Keep_YAW模式时,如果持续从里程计数据中提取yaw角,会导致yaw角缓慢漂移,因此我们直接缓存yaw角控制命令
    double last_set_yaw_{0.0};  // 默认起飞时刻yaw为0.0,只有当Set_YAW后才会改变

    // 缓存相关状态
    bool allow_takeoff_{false};  // 允许起飞
    bool is_flip_{false};        // 当前是否出现姿态大幅偏转
    bool is_fence_{false};       // 当前是否超过地理围栏/电子围栏的限制

    // 用于return返航阶段的返航点，在起飞阶段自动刷新
    Eigen::Vector3d home_point_{Eigen::Vector3d::Zero()};
    bool return_height_initialized_{false};

    // controller_ready_ = true 则状态机允许由OFF -> INIT
    bool controller_ready_{false}; 
    // control_msg_lost_ = true 并且缓存的控制指令为velocity / trajectory 则状态机切换为 hover    
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
    std::queue<QueuedFsmEvent_t> fsm_event_queue_;                         // 事件队列
    sunray_fsm::sunray_fsm_config_t fsm_config_;                       // 状态机参数结构体
    std::vector<sunray_fsm::Transition> sunray_state_transmit_table_;  // 成员变量
    bool rc_connected{false};  // 遥控器连接状态，这里是从sunray_rc_joy_node节点传递？
    std::deque<double> odom_rate_samples_s_;
};
