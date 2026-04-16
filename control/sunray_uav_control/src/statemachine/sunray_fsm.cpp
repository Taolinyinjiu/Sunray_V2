#include "statemachine/sunray_fsm.hpp"
#include "statemachine/sunray_fsm_loadparam.hpp"
#include "string_uav_namespace_utils.hpp"
#include <stdexcept>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>  // 引入Yaml-cpp库，用于读取yaml文件
#include <sunray_msgs/UAVControlFSMState.h>
#include "controller/px4_origin_controller.hpp"
#include "controller/geometric_controller.hpp"
#include <sunray_msgs/OdomStatus.h>
#include "utils/orientation_utils.hpp"
// Sunray_FSM更新路径
//
// 低频 ros node : Surnay_FSM->process()
//     - 状态切换
//     - 数据发布
// 高频 Sunray_FSM Therad controller_update_loop()
//     - 设置里程计
//     - 控制器输出

// 构造函数
Sunray_FSM::Sunray_FSM(ros::NodeHandle& nh) {
    nh_ = nh;
};
// 析构函数
Sunray_FSM::~Sunray_FSM() {
    stop_controller_thread_.store(true, std::memory_order_relaxed);
    if (controller_update_thread_.joinable()) {
        controller_update_thread_.join();
    }
}
void Sunray_FSM::init() {
    // 请注意init中所有函数均为无返回类型,初始化主要对静态参数进行检查，检查不通过，抛出具体异常
    load_param();
    init_publisher();
    init_subscriber();
    register_controller();

    // 初始化状态转移表
    init_transition_table();
    // 控制器更新线程
    stop_controller_thread_.store(false, std::memory_order_relaxed);
    controller_update_thread_ = std::thread(&Sunray_FSM::controller_update_loop, this);
}

// 获取状态机的更新频率
double Sunray_FSM::get_update_frequency() {
    return fsm_config_.basic_param.supervisor_update_frequency;
}

// process向外部提供，使用方式定义为
// ----ros node----
// Sunray_FSM sunray_fsm(nh); // 实例化类并传入全局句柄
// sunray_fsm.init(); // 初始化类
// double update_hz = sunray_fsm.get_update_freqency();  // 获得状态机的更新频率
// ros::Rate rate(update_hz); // 设置节点更新频率
// while(ros::ok()){
//    sunray_fsm.process();
// }

void Sunray_FSM::process() {
    // 检查是否有消息超时
    check_rosmsg_timeout();
    // 检查运动是否完成
    check_move_completed();
    // 检查控制器状态 OFF -> INIT
    check_controller_ready();
    // 更新是否允许起飞 FLAG: INIT -> TAKEOFF
    check_allow_takeoff();
    // 处理状态转移事件
    process_fsm_event_queue();
    // 发布状态
    pub_sunray_fsm_state();
}

// OFF -> INIT -> TAKEOFF -> HOVER -> MOVE
// |                 |         |        |
// LAND  <- - -  - - - - - - - | - - -  |
//                             |        |
// RETUAN <- - - - - - - - - - - - - -  |
//
// LAND / RETURN　-> INIT
// note：这里并没有处理emergency_kill相关状态，因为我们在设计的时候认为emergency_kill是一个比较危险的状态，主要表现在空中停机等一系列措施，
// 当需要进入emergency状态时，我们需要思考的是如何减少无人机的损伤，以及如何避免对周围的伤害，不重启/检查无人机就立即复飞是一个不太可能的情况
void Sunray_FSM::init_transition_table() {
    // 首先，检查表是否为空，如果非空则返回，避免重复初始化
    if (!sunray_state_transmit_table_.empty())
        return;
    // always是一个lambda表达式，用于表示，这一项不需要进行检查or执行
    auto always = [] { return true; };

    // struct Transition {
    //     sunray_fsm::SunrayState current_state;   // 状态机当前状态
    //     sunray_fsm::SunrayEvent event;           // 发生的事件(control_cmd)
    //     sunray_fsm::SunrayState transmit_state;  // 状态机要转移到的状态
    //     std::function<bool()> guard;             // 判断是否允许转移
    //     std::function<bool()> action;            // 转移成功后执行的命令
    // };

    // OFF -> INIT,要求控制器准备就绪
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::OFF,
                                            sunray_fsm::SunrayEvent::CONTROLLER_READY,
                                            sunray_fsm::SunrayState::INIT,
                                            [this] { return controller_ready_; },
                                            always});

    // INIT -> TAKEOFF，要求takeoff标识符置位true
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::INIT,
                                            sunray_fsm::SunrayEvent::TAKEOFF_REQUEST,
                                            sunray_fsm::SunrayState::TAKEOFF,
                                            [this] { return allow_takeoff_; },
                                            [this] { return update_home_point(); }});

    // TAKEOFF -> HOVER
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::TAKEOFF,
                                            sunray_fsm::SunrayEvent::TAKEOFF_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            always});

    // HOVER -> MOVE (POINT)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::POINT_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // HOVER -> MOVE (VELOCITY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::VELOCITY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // HOVER -> MOVE (TRAJECTORY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::TRAJECTORY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // HOVER -> MOVE (POINT_WGS84)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::POINT_WGS84_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // MOVE -> MOVE (POINT)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // MOVE -> MOVE (VELOCITY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::VELOCITY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // MOVE -> MOVE (TRAJECTORY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::TRAJECTORY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});

    // MOVE -> MOVE (POINT_WGS84)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_WGS84_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            always,
                                            always});
    // MOVE -> HOVER (HOVER_REQUEST)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::HOVER_REQUEST,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return sunray_controller_->set_hover_point(last_odometry_); }});
    // MOVE -> HOVER (POINT_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return sunray_controller_->set_hover_point(last_odometry_); }});

    // MOVE -> HOVER (VELOCITY_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::VELOCITY_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return sunray_controller_->set_hover_point(last_odometry_); }});

    // MOVE -> HOVER (TRAJECTORY_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::TRAJECTORY_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return sunray_controller_->set_hover_point(last_odometry_); }});

    // MOVE -> HOVER (POINT_WGS84_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_WGS84_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            [this] { return sunray_controller_->set_hover_point(last_odometry_); }});
    // HOVER -> RETURN
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::RETURN_REQUEST,
                                            sunray_fsm::SunrayState::RETURN,
                                            always,
                                            always});

    // MOVE -> RETURN
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::RETURN_REQUEST,
                                            sunray_fsm::SunrayState::RETURN,
                                            always,
                                            always});
    // HOVER-> LAND
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::LAND_REQUEST,
                                            sunray_fsm::SunrayState::LAND,
                                            always,
                                            always});
    // MOVE -> LAND
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::LAND_REQUEST,
                                            sunray_fsm::SunrayState::LAND,
                                            always,
                                            always});
    // RETURN -> LAND
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::RETURN,
                                            sunray_fsm::SunrayEvent::LAND_REQUEST,
                                            sunray_fsm::SunrayState::LAND,
                                            always,
                                            always});
    // RETURN -> LAND
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::RETURN,
                                            sunray_fsm::SunrayEvent::RETURN_COMPLETED,
                                            sunray_fsm::SunrayState::LAND,
                                            always,
                                            always});
    // LAND -> READY
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::LAND,
                                            sunray_fsm::SunrayEvent::LAND_COMPLETED,
                                            sunray_fsm::SunrayState::INIT,
                                            always,
                                            always});
}

const std::vector<sunray_fsm::Transition>& Sunray_FSM::get_transition_table() {
    // 如果状态转移表没有初始化(表容器为空)，则进行一次初始化
    if (sunray_state_transmit_table_.empty()) {
        init_transition_table();
    }
    // 返回表容器，类成员sunray_state_transmit_table_
    return sunray_state_transmit_table_;
}
// 处理全局高优先级状态
bool Sunray_FSM::handle_global_event(sunray_fsm::SunrayEvent event) {
    switch (event) {
    // 全局最高优先级：紧急锁桨
    // 任意状态收到该事件，都需要进入EMERGENCY_KILL
    case sunray_fsm::SunrayEvent::KILL_REQUEST: {
        // 切换状态为
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            fsm_state_ = sunray_fsm::SunrayState::EMERGENCY_KILL;
        }
        // 执行对应函数
        emergency_kill();
        return true;
    }
    default:
        // 非全局事件，交给普通状态转移表处理
        return false;
    }
}
// 处理正常情况下的状态转移
bool Sunray_FSM::handle_event(sunray_fsm::SunrayEvent event) {
    // 首先丢进全局高优先级状态检查
    if (handle_global_event(event)) {
        return true;
    }
    // handle_event() -> false 说明不是kill状态，按照正常流程往下走
    // 打个快照
    sunray_fsm::SunrayState current_state;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        current_state = fsm_state_;
    }
    // 获取缓存的状态转移表
    const auto state_table = get_transition_table();
    // 使用迭代器 查找匹配字段
    for (const auto& t : state_table) {
        // 如果当前迭代器找到的状态转移规则不是针对当前状态的，则跳过
        if (t.current_state != current_state) {
            continue;
        }
        // 如果当前迭代器找到的状态转移规则不是针对当前事件的，跳过
        if (t.event != event) {
            continue;
        }
        // 那么运行到这里，就应当得到了针对当前状态和事件的转移规则
        // 这里分为两个方面，一个是判断是否允许转移，一个是转移后执行的函数
        // 首先判断是否允许转移
        // (t.guard ? t.guard() : true) -> 如果t.guard存在，则返回t.guard()的结果
        //                                 如果t.guard不存在，则直接返回true
        const bool allow_checkout_state = (t.guard ? t.guard() : true);
        if (!allow_checkout_state) {
            return false;
        }
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            fsm_state_ = t.transmit_state;
        }
        // (t.action ? t.action() : true) -> 如果t.action存在，则执行t.action()
        //                                   如果t.action不存在，则直接返回true
        const bool need_action = (t.action ? t.action() : true);
        // 但其是我们不关心action的结果，因为写到这里的时候，我意识到handle_event其实是用作controlcmd的回调和运动判断完成的回调，而不是一个高频更新的函数
        //  因此这里的action并不能够与无人机的运动相关联
        //  但是我认为保留这一项可能有一些好处，比如action可以作为日志，存储每一次状态切换的细节
        return true;
    }
    return false;
}
// 从yaml文件中加载参数
void Sunray_FSM::load_param() {
    // 1. 读取uav_name 与 uav_id,拼接uav_ns并标准化
    std::string uav_name;
    int uav_id;
    if (nh_.getParam("/uav_name", uav_name)) {
        if (uav_name.empty()) {  // 如果为空，抛出异常
            throw std::runtime_error("uav_name connot be empty");
        }
    } else {  // 如果读取失败，抛出异常
        throw std::runtime_error("missing param /uav_name");
    }
    if (nh_.getParam("/uav_id", uav_id)) {
        if (uav_id <= 0)
            throw std::runtime_error("/uav_id cannot <= 0");
    } else {
        throw std::runtime_error("missing param /uav_id");
    }
    // 拼接uav_name+uav_id
    std::string uav_ns = uav_name + std::to_string(uav_id);
    // 标准化
    uav_ns_ = sunray_common::normalize_uav_ns(uav_ns);
    // 2. 读取config.yaml路径
    // 读取节点名
    std::string node_name = ros::this_node::getName();
    // 构造私有节点句柄，用于读取节点私有参数,这里主要是config.yaml路径
    ros::NodeHandle private_nh_("~");
    std::string config_yamlfile_path_;
    if (private_nh_.getParam("config_yamlfile_path", config_yamlfile_path_)) {
        if (config_yamlfile_path_.empty()) {  // 路径为空，抛出异常
            throw std::runtime_error("yaml_path connot be empty");
        }
    } else {  // 读取失败，抛出异常
        throw std::runtime_error("missing param" + node_name + "/config_yamlfile_path");
    }
    // 3. 依据yaml_path,构造yaml节点读取文件，根据字段填充结构体
    YAML::Node root;  // 构造一个YAML文件的根节点
    // 由于读取的过程可能引发异常，因此使用try语法
    try {
        root =
            YAML::LoadFile(config_yamlfile_path_);  // 从指定的路径中读取yaml文件并解析为YAML::Node
    } catch (const YAML::Exception& e) {  // 如果解析的过程中发生错误，捕捉异常
        throw std::runtime_error("Failed to load yaml file '" + config_yamlfile_path_ + ":" +
                                 e.what());
    }
    // 顺利读取，取出各个字段对应的部分
    // 由于这里会写的很长，所以将他们分为几个不同的函数用来填充结构体
    ROS_INFO("Loading sunray_control_config.yaml from %s start", config_yamlfile_path_.c_str());
    ROS_INFO("basic");
    loadBasicParam(root["basic_param"], fsm_config_.basic_param);
    ROS_INFO("protect");
    loadProtectParam(root["protect_param"], fsm_config_.protect_param);
    ROS_INFO("msg_timeout");
    loadMsgTimeoutParam(root["msg_timeout_param"], fsm_config_.msg_timeout_param);
    ROS_INFO("takeoff_land");
    loadTakeoffLandParam(root["takeoff_land_param"], fsm_config_.takeoff_land_param);
    ROS_INFO("arrival_judge");
    loadArrivalJudgeParam(root["arrival_judge_param"], fsm_config_.arrival_judge_param);
    ROS_INFO("local_fence");
    loadLocalFenceParam(root["local_fence_param"], fsm_config_.local_fence_param);
    ROS_INFO("velocity");
    loadVelocityParam(root["velocity_param"], fsm_config_.velocity_param);
    ROS_INFO("Loading sunray_control_config.yaml from %s end", config_yamlfile_path_.c_str());

    fsm_config_.basic_param.odom_topic_name =
        sunray_common::replace_uav_ns(fsm_config_.basic_param.odom_topic_name, uav_ns_);
}
// 初始化订阅者
void Sunray_FSM::init_subscriber() {
    // 订阅外部里程计
    local_odom_sub_ = nh_.subscribe(
        fsm_config_.basic_param.odom_topic_name, 10, &Sunray_FSM::local_odom_callback, this);
    // 订阅外部定位源状态
    localization_state_sub_ = nh_.subscribe(uav_ns_ + "/sunray/localization/odom_status",
                                            10,
                                            &Sunray_FSM::localization_state_callback,
                                            this);
    // 订阅无人机控制指令
    uav_control_cmd_sub_ = nh_.subscribe(
        uav_ns_ + "/sunray/uav_control_cmd", 10, &Sunray_FSM::uav_control_cmd_callback, this);
    // 订阅状态检查指令
    // 由于系统状态检查没有自定义话题类型，这里先注释掉
    // system_check_sub_ = nh_.subscribe(
    //     uav_ns_ + "/sunray/uav_system_check", 10, &Sunray_FSM::system_check_callback, this);
}
// 初始化发布者
void Sunray_FSM::init_publisher() {
    // 似乎只有一个状态机状态发布者
    sunray_fsm_state_pub_ =
        nh_.advertise<sunray_msgs::UAVControlFSMState>(uav_ns_ + "/sunray/fsm/state", 10);
    sunray_odom_debug_pub_ =
        nh_.advertise<nav_msgs::Odometry>(uav_ns_ + "/surnay/debug/odometrry", 10);
}
// 为状态机注册控制器
void Sunray_FSM::register_controller() {
    // 根据参数结构体中的controller_type选择要注册的控制器
    // 根据调用的顺序来看，对controller_type的检查已经在load_param()函数中被进行了，因此这里我们不进行检查，直接使用就好
    // 根据sunray_control_config.yaml中的定义，控制器的类型分别为
    // 0: px4_origin_controller px4原生控制器
    // 1: sunray_attitude_controller 预留
    // 2: geometric_controller Sunray 几何控制器
    switch (fsm_config_.basic_param.controller_types) {
    case 0: {
        sunray_controller_ = std::make_shared<PX4_OriginController>(nh_);
        // 在这里做控制器的初始化,可以不用在外面判断控制器的类型再写报错异常信息
        if (sunray_controller_->init() == false) {
            // 如果控制器初始化失败，抛出异常
            throw std::runtime_error("{Px4 Original Controller initialization failed");
        }
        break;
    }
    case 1: {
        sunray_controller_ = std::make_shared<Geometric_Controller>(nh_);
        // 在这里做控制器的初始化,可以不用在外面判断控制器的类型再写报错异常信息
        if (sunray_controller_->init() == false) {
            // 如果控制器初始化失败，抛出异常
            throw std::runtime_error("{Geometric_Controller initialization failed");
        }
        break;
    }
    case 2: {
        // TODO:将2修改为RL_RAPTOR控制器
        sunray_controller_ = std::make_shared<Geometric_Controller>(nh_);
        // 在这里做控制器的初始化,可以不用在外面判断控制器的类型再写报错异常信息
        if (sunray_controller_->init() == false) {
            // 如果控制器初始化失败，抛出异常
            throw std::runtime_error("{Geometric_Controller initialization failed");
        }
        break;
    }
        // 这里没有default分支，因为load_param()界定了fsm_config_.basic_param.controller_types只会有0，1，2三个值，不可能有其他值被传入
    }
}

// 检查controller是否就绪
void Sunray_FSM::check_controller_ready() {
    // 打一份快照
    sunray_fsm::SunrayState current_state;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        current_state = fsm_state_;
    }
    // 仅在OFF状态下才判断
    if (current_state == sunray_fsm::SunrayState::OFF) {
        controller_ready_ = sunray_controller_->is_ready();
        if (controller_ready_) {
            ROS_INFO("controller ready");
            enqueue_fsm_event(sunray_fsm::SunrayEvent::CONTROLLER_READY);
        }
    }
}
// 检查是否允许起飞
bool Sunray_FSM::check_allow_takeoff() {
    // 这里的思路是这样的，首先函数的结果作为类变量allow_takeoff_的值缓存，所有需要判断能否起飞的程序，都是直接判断变量allow_takeoff_是否为true来进行起飞
    // check_allow_takeoff()实际上是一个被定时器调用，综合分析各个消息，来决定是否起飞的函数，可以认为是一个信息处理中枢
    // 1. 首先是配置文件中 takeoff_with_code
    // 是否为true，如果该值为false，则要求rc节点存在并且持续发送连接状态
    // 2. 外部system_check模块是否正常工作，如果正常工作则其数据需要被参考，比如是否允许起飞之类

    // 构造临时变量
    bool temp_allow_state;

    // 首先读取参数takeoff_with_code
    if (fsm_config_.protect_param.takeoff_with_code == true) {
        temp_allow_state = true;
    } else {
        temp_allow_state = false;
    }
    // 这里处理遥控器传递的信息
    if (rc_connected == true) {  // 首先遥控器要连接
        // 这里判断需要再设计一下
    }
    // 将临时变量更新到类变量
    allow_takeoff_ = temp_allow_state;
    return allow_takeoff_;
}
// 检查是否存在消息超时的情况
void Sunray_FSM::check_rosmsg_timeout() {
    // 根据输入检查。我们一共配置了四个回调函数，分别是 外部里程计回调函数,
    // 外部里程计状态回调函数，无人机控制指令回调函数，系统状态检查回调函数
    // 由于外部里程计状态目前没有使用(原先设计的使用外部里程计状态配置EKF2参数等部分，我们认为需要从fsm中拆出去）
    // 状态检查部分由于没有实现完也没有使用，所以这里只需要配置外部里程计回调函数和无人机控制指令回调函数
    ros::Time now = ros::Time::now();
    // 打份快照
    control_common::UAVStateEstimate odom_snapshot;
    control_common::UavControlCmd last_cmd_snapshot;
    {
        std::lock_guard<std::mutex> lk1(odom_mutex_);
        odom_snapshot = last_odometry_;
    }
    {
        std::lock_guard<std::mutex> lk2(cmd_mutex_);
        last_cmd_snapshot = last_control_cmd_;
    }

    if (odom_snapshot.timestamp != ros::Time(0) &&
        (now - odom_snapshot.timestamp).toSec() > fsm_config_.msg_timeout_param.local_odometry) {
        // enqueue_fsm_event(sunray_fsm::SunrayEvent::KILL_REQUEST);
        // 里程计超时，切KILL？其实并不保险我个人认为
    }
    if (last_cmd_snapshot.timestamp != ros::Time(0) &&
        (now - last_cmd_snapshot.timestamp).toSec() > 0.2) {
        control_msg_lost_ = true;
    } else {
        control_msg_lost_ = false;
    }
}
void Sunray_FSM::check_move_completed() {
    // 我们在这个函数中对运动状态进行判断，主要分为两类
    // 1. takeoff() land() 对起飞/降落结果的判断
    // 2. is_point_complete() 对move point是否完成的判断

    // 首先我们对起飞/降落是否完成进行判断，前提是处于对应的模式
    sunray_fsm::SunrayState current_state;  // 先打一份快照
    {
        std::lock_guard<std::mutex> lk1(state_mutex_);
        current_state = fsm_state_;
    }
    if (current_state == sunray_fsm::SunrayState::TAKEOFF) {
        bool takeoff_state = sunray_controller_->is_takeoff_complete();
        if (takeoff_state) {
            enqueue_fsm_event(sunray_fsm::SunrayEvent::TAKEOFF_COMPLETED);
        }
    } else if (current_state == sunray_fsm::SunrayState::LAND) {
        bool land_state = sunray_controller_->is_land_complete();
        if (land_state) {
            enqueue_fsm_event(sunray_fsm::SunrayEvent::LAND_COMPLETED);
        }
    }
    // 对运动是否完成的判断，存在前提条件： control_cmd没有保持10Hz的发布频率
    if (!control_msg_lost_) {
        return;
    }
    // 如果当前不在move状态，则退出
    if (current_state != sunray_fsm::SunrayState::MOVE) {
        return;
    }
    // 给control_cmd打一份快照
    control_common::UavControlCmd current_cmd;
    {
        std::lock_guard<std::mutex> lk1(cmd_mutex_);
        current_cmd = last_control_cmd_;
    }
    // 其中，只有move_point / move_point_body/ move_point_wgs84
    // 需要判断是否到达位置，剩余的直接切换为hover
    if (current_cmd.control_cmd == control_common::UavControlCmd::ControlCmd::MOVE_POINT ||
        current_cmd.control_cmd == control_common::UavControlCmd::ControlCmd::MOVE_POINT_BODY ||
        current_cmd.control_cmd == control_common::UavControlCmd::ControlCmd::MOVE_POINT_WGS84) {
        bool move_state = sunray_controller_->is_point_complete();
        if (move_state) {
            enqueue_fsm_event(sunray_fsm::SunrayEvent::POINT_COMPLETED);
        }
    } else {
        enqueue_fsm_event(sunray_fsm::SunrayEvent::HOVER_REQUEST);
    }
}

// --------------------话题回调函数----------------------

// 外部里程计回调函数
void Sunray_FSM::local_odom_callback(const nav_msgs::Odometry& msg) {
    control_common::UAVStateEstimate temp(msg);
    std::lock_guard<std::mutex> lk(odom_mutex_);
    last_odometry_ = temp;
}
// 外部里程计状态回调函数
void Sunray_FSM::localization_state_callback(const sunray_msgs::OdomStatus& msg) {
    // TODO
}

// 无人机控制指令回调函数
void Sunray_FSM::uav_control_cmd_callback(const sunray_msgs::UAVControlCMD& msg) {
    // 构造临时变量
    control_common::UavControlCmd temp_cmd(msg);
    // 更新缓存
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        last_control_cmd_ = temp_cmd;
    }
    // 将control_cmd中的指令，转换为sunray_fsm的事件请求
    switch (temp_cmd.control_cmd) {
    case control_common::UavControlCmd::ControlCmd::TAKEOFF:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::TAKEOFF_REQUEST);
        break;
    case control_common::UavControlCmd::ControlCmd::LAND:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::LAND_REQUEST);
        break;
    case control_common::UavControlCmd::ControlCmd::RETURN:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::RETURN_REQUEST);
        break;
    case control_common::UavControlCmd::ControlCmd::KILL:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::KILL_REQUEST);
        break;
    case control_common::UavControlCmd::ControlCmd::HOVER:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::HOVER_REQUEST);
        break;
    // local系和body系都是同一个point_request和point_completed
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT:
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_BODY:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::POINT_REQUEST);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY:
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY_BODY:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::VELOCITY_REQUEST);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_TRAJECTORY:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::TRAJECTORY_REQUEST);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_WGS84:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::POINT_WGS84_REQUEST);
        break;
    case control_common::UavControlCmd::ControlCmd::UNDEFINE:
    default:
        // 未定义指令不入队，避免污染事件流
        break;
    }
}
// 系统检查回调函数
void Sunray_FSM::system_check_callback() {
    // TODO
}

// ----------------定时器回调函数----------------
// 发布Sunray_FSM的相关信息
void Sunray_FSM::pub_sunray_fsm_state() {
    // >  这里我们删除了px4state消息类型，因为我们想要让状态机与px4相关的，进行一个解耦
    // 打一份快照
    sunray_fsm::SunrayState fsm_state_snapshot;
    control_common::UavControlCmd last_cmd_snapshot;
    {
        std::lock_guard<std::mutex> lk1(state_mutex_);
        fsm_state_snapshot = fsm_state_;
    }
    {
        std::lock_guard<std::mutex> lk2(cmd_mutex_);
        last_cmd_snapshot = last_control_cmd_;
    }
    // 构建要发布的消息
    sunray_msgs::UAVControlFSMState fsm_state_msg;
    // 首先填充话题头
    fsm_state_msg.header.stamp = ros::Time::now();
    // 当前起飞参数
    fsm_state_msg.takeoff_relative_height = fsm_config_.takeoff_land_param.takeoff_relative_height;
    fsm_state_msg.takeoff_max_velocity = fsm_config_.takeoff_land_param.takeoff_max_velocity;
    // 当前降落参数
    fsm_state_msg.land_type = fsm_config_.takeoff_land_param.land_type;
    fsm_state_msg.land_max_velocity = fsm_config_.takeoff_land_param.land_max_velocity;
    // 当前返航的目标点
    fsm_state_msg.home_point.x = home_point_.x();
    fsm_state_msg.home_point.y = home_point_.y();
    fsm_state_msg.home_point.z = home_point_.z();
    // 当前的控制指令(由于两者的数据类型不同，一个是uint8另一个是强类型枚举，但是由于值都是一一对应的，因此这里用类型转换)
    fsm_state_msg.control_cmd = static_cast<uint8_t>(last_cmd_snapshot.control_cmd);
    // 状态机的状态同样使用静态类型转换
    fsm_state_msg.sunray_fsm_state = static_cast<uint8_t>(fsm_state_snapshot);
    // 发布消息
    sunray_fsm_state_pub_.publish(fsm_state_msg);
}
// 发布Sunray_Controller的相关信息
void Sunray_FSM::pub_controller_state() {
    sunray_controller_->pub_controller_state();
}
// -------------------------控制指令执行函数------------------
bool Sunray_FSM::takeoff() {
    return sunray_controller_->takeoff(fsm_config_.takeoff_land_param.takeoff_relative_height,
                                       fsm_config_.takeoff_land_param.takeoff_max_velocity);
}
bool Sunray_FSM::land() {
    return sunray_controller_->land(fsm_config_.takeoff_land_param.land_type,
                                    fsm_config_.takeoff_land_param.land_max_velocity);
}
bool Sunray_FSM::hover() {
    return sunray_controller_->hover();
}
bool Sunray_FSM::return_home() {
    controller_data_types::TargetPoint_t home_target;
    control_common::UAVStateEstimate odom_snapshot;
    home_target.position = home_point_;
    // 从里程计提取yaw角
    {
        std::lock_guard<std::mutex> lk2(odom_mutex_);
        odom_snapshot = last_odometry_;
    }
    home_target.yaw = quaternion_to_yaw_rad(odom_snapshot.orientation);
    // 持续发送返航点（不是一次性）
    if (!sunray_controller_->move_point(home_target)) {
        return false;
    }
    // 到达判据交给控制器
    return sunray_controller_->is_point_complete();
}
bool Sunray_FSM::emergency_kill() {
    return sunray_controller_->emergency_kill();
}
bool Sunray_FSM::move_point(control_common::UavControlCmd cmd) {
    control_common::UAVStateEstimate odom_snapshot;
    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        odom_snapshot = last_odometry_;
    }
    const double current_yaw = quaternion_to_yaw_rad(odom_snapshot.orientation);

    // 根据local系还是body系决定调用的函数
    if (cmd.control_cmd == control_common::UavControlCmd::ControlCmd::MOVE_POINT) {
        controller_data_types::TargetPoint_t point;
        point.position = cmd.position;
        point.yaw = (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW)
                        ? cmd.yaw
                        : current_yaw;
        return sunray_controller_->move_point(point);
    } else {
        controller_data_types::TargetBodyPoint_t point;
        point.position_xy = cmd.body_position_xy;
        point.fixed_height = cmd.fixed_height;
        if (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW) {
            point.yaw = cmd.yaw - current_yaw;
        } else {
            point.yaw = 0.0;
        }
        return sunray_controller_->move_point_body(point);
    }
}
bool Sunray_FSM::move_velocity(control_common::UavControlCmd cmd) {
    control_common::UAVStateEstimate odom_snapshot;
    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        odom_snapshot = last_odometry_;
    }
    const double current_yaw = quaternion_to_yaw_rad(odom_snapshot.orientation);

    // 根据local系和body系决定调用的函数
    if (cmd.control_cmd == control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY) {
        controller_data_types::TargetVelocity_t velocity;
        velocity.stamp = cmd.timestamp;
        velocity.velocity = cmd.velocity;
        velocity.yaw = (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW)
                           ? cmd.yaw
                           : current_yaw;
        velocity.yaw_rate = (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAWRATE)
                                ? cmd.yaw_rate
                                : 0.0;
        return sunray_controller_->move_velocity(velocity);
    } else {
        controller_data_types::TargetBodyVelocity_t velocity;
        velocity.stamp = cmd.timestamp;
        velocity.velocity_xy = cmd.body_velocity_xy;
        velocity.fixed_height = cmd.fixed_height;
        if (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW) {
            velocity.yaw = cmd.yaw - current_yaw;
        } else {
            velocity.yaw = 0.0;
        }
        velocity.yaw_rate = (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAWRATE)
                                ? cmd.yaw_rate
                                : 0.0;
        return sunray_controller_->move_velocity_body(velocity);
    }
}
bool Sunray_FSM::move_trajectory(control_common::UavControlCmd cmd) {
    control_common::UAVStateEstimate odom_snapshot;
    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        odom_snapshot = last_odometry_;
    }
    const double current_yaw = quaternion_to_yaw_rad(odom_snapshot.orientation);

    // 提取轨迹类型
    controller_data_types::TargetTrajectoryPoint_t traj_point;
    traj_point.position = cmd.position;
    traj_point.velocity = cmd.velocity;
    traj_point.acceleration = cmd.acceleration;
    traj_point.jerk = cmd.jerk;
    traj_point.yaw = (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW)
                         ? cmd.yaw
                         : current_yaw;
    traj_point.yaw_rate = (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAWRATE)
                              ? cmd.yaw_rate
                              : 0.0;
    return sunray_controller_->move_trajectory(traj_point);
}
bool Sunray_FSM::move_point_wgs84(control_common::UavControlCmd cmd) {
    // 提取wgs84坐标系下的目标点
    geographic_msgs::GeoPoint geo_point;
    geo_point.altitude = cmd.wgs84_position.altitude;
    geo_point.latitude = cmd.wgs84_position.latitude;
    geo_point.longitude = cmd.wgs84_position.longitude;
    return sunray_controller_->move_point_wgs84(geo_point);
}
// 更新home点
bool Sunray_FSM::update_home_point() {
    // 打一份里程计快照?
    control_common::UAVStateEstimate odom_snapshot;
    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        odom_snapshot = last_odometry_;
    }
    // 更新home点
    home_point_ = odom_snapshot.position;
    home_point_.z() = fsm_config_.takeoff_land_param.takeoff_relative_height;
    return true;
}

// 首先我们是这样来设计这个函数的，我们为这个函数新建一个线程，以200Hz的频率or100Hz的频率来运行，这个频率取决于config文件中的controller_update_frequency参数决定
// 然后这个函数是一个void类型，因为线程单独运行并不需要返回值
void Sunray_FSM::update_controller_output() {
    // 然后我们需要根据状态机自身的状态，来决定如何调用控制器的api函数，但是这里实际上并不涉及到对状态机状态切换的请求，
    // 比如当前为INIT状态，控制命令的回调函数接受到了takeoff的控制命令，那么回调函数会触发一次request，然后外部某个函数检查，更新状态
    // 这个函数，只负责当前状态下的函数调用流程
    sunray_fsm::SunrayState fsm_state_snapshot;
    control_common::UavControlCmd last_cmd_snapshot;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        fsm_state_snapshot = fsm_state_;
    }
    {
        std::lock_guard<std::mutex> lk2(cmd_mutex_);
        last_cmd_snapshot = last_control_cmd_;
    }
    switch (fsm_state_snapshot) {
    case sunray_fsm::SunrayState::OFF: {
        // OFF状态下我们什么都不做
        break;
    }
    // INIT模式更新为：切换为offboard模式，监听rc_controller的输入，持续发布setpoint流
    case sunray_fsm::SunrayState::INIT: {
        sunray_controller_->on_ground_keep_setpoint();
        break;
    }
    case sunray_fsm::SunrayState::TAKEOFF: {
        // 起飞状态下，我们疯狂调用takeoff函数
        takeoff();
        break;
    }
    case sunray_fsm::SunrayState::HOVER: {
        hover();
        break;
    }
    case sunray_fsm::SunrayState::RETURN: {
        // 值得注意的是，控制器并没有提供return状态的函数，因为控制器自身并不存储历史状态
        // 所以我们要先使用move_point回到home点，然后进入land状态
        // 所以我们需要先持续的调用move_point而不是一次，因为一次的调用，会在到达return点的时候切换为hover状态
        // 那么基本的流程就是，接受到return指令，持续调用move_point到return点，然后获取控制器到达情况，并切换为land
        // 持续飞向 home 点，到达后请求降落
        if (return_home()) {
            enqueue_fsm_event(sunray_fsm::SunrayEvent::LAND_REQUEST);
        }
        break;
    }
    case sunray_fsm::SunrayState::LAND: {
        // 降落状态下，我们疯狂调用takeoff函数
        land();
        break;
    }
    case sunray_fsm::SunrayState::MOVE: {
        // move阶段这里存在的问题是，有多种的move方式，因此我们需要根据last_control_cmd中的指令决定调用的函数
        if (last_cmd_snapshot.control_cmd ==
                control_common::UavControlCmd::ControlCmd::MOVE_POINT ||
            last_cmd_snapshot.control_cmd ==
                control_common::UavControlCmd::ControlCmd::MOVE_POINT_BODY) {
            move_point(last_cmd_snapshot);
        } else if (last_cmd_snapshot.control_cmd ==
                       control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY ||
                   last_cmd_snapshot.control_cmd ==
                       control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY_BODY) {
            move_velocity(last_cmd_snapshot);
        } else if (last_cmd_snapshot.control_cmd ==
                   control_common::UavControlCmd::ControlCmd::MOVE_TRAJECTORY) {
            move_trajectory(last_cmd_snapshot);
        } else if (last_cmd_snapshot.control_cmd ==
                   control_common::UavControlCmd::ControlCmd::MOVE_POINT_WGS84) {
            move_point_wgs84(last_cmd_snapshot);
        }
        break;
    }
    case sunray_fsm::SunrayState::EMERGENCY_KILL: {
        if (emergency_kill()) {
            enqueue_fsm_event(sunray_fsm::SunrayEvent::KILL_COMPLETED);
        }
        break;
    }
    }
}

// 向状态事件队列中传入事件
// note:如果传入的事件与队列中最新的事件相同，则不会传入
void Sunray_FSM::enqueue_fsm_event(sunray_fsm::SunrayEvent input_event) {
    std::lock_guard<std::mutex> lk(event_mutex_);
    if (!fsm_event_queue_.empty() && fsm_event_queue_.back() == input_event) {
        // 如果事件队列非空，并且传入的与队列中最新的是一致的，那么就return
        return;
    }
    fsm_event_queue_.push(input_event);
}
// 处理状态机事件中的事件
void Sunray_FSM::process_fsm_event_queue() {
    sunray_fsm::SunrayEvent event;
    {
        std::lock_guard<std::mutex> lk(event_mutex_);
        if (fsm_event_queue_.empty()) {
            return;
        }
        event = fsm_event_queue_.front();
        fsm_event_queue_.pop();
    }
    handle_event(event);
}
// 控制器循环更新
void Sunray_FSM::controller_update_loop() {
    // 首先得到更新的频率
    const double hz = fsm_config_.basic_param.controller_update_frequency;
    ros::Rate rate(hz);
    // 循环
    while (ros::ok() && !stop_controller_thread_.load(std::memory_order_relaxed)) {
        control_common::UAVStateEstimate odom_snapshot;
        {
            std::lock_guard<std::mutex> lk(odom_mutex_);
            odom_snapshot = last_odometry_;
        }
        sunray_controller_->set_current_odom(odom_snapshot);
        update_controller_output();
        pub_controller_state();  // 在这里才输出controller的状态，是因为我们使用的是基于反馈的控制器，如果输入量没有变化的话，没有什么意义
        rate.sleep();
    }
}
