#include "statemachine/sunray_fsm.hpp"
#include "statemachine/sunray_fsm_loadparam.hpp"
#include "string_uav_namespace_utils.hpp"
#include <stdexcept>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>  // 引入Yaml-cpp库，用于读取yaml文件
#include <sunray_msgs/UAVControlFSMState.h>
// 构造函数
Sunray_FSM::Sunray_FSM(ros::NodeHandle& nh) {
    nh_ = nh;
};

void Sunray_FSM::init() {
    // 请注意init中所有函数均为无返回类型,初始化主要对静态参数进行检查，检查不通过，抛出具体异常
    load_param();
    init_publisher();
    init_subscriber();
    register_controller();

    // 初始化状态转移表
    init_transition_table();
}

// OFF -> INIT -> TAKEOFF -> HOVER -> MOVE
//                   |         |        |
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

    // OFF -> INIT
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::OFF,
                                            sunray_fsm::SunrayEvent::CONTROLLER_READY,
                                            sunray_fsm::SunrayState::INIT,
                                            [this] { return controller_ready_; },
                                            always});

    // INIT -> TAKEOFF
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::INIT,
                                            sunray_fsm::SunrayEvent::TAKEOFF_REQUEST,
                                            sunray_fsm::SunrayState::TAKEOFF,
                                            [this] { return allow_takeoff_; },
                                            always});

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
                                            [this] { return allow_move_; },
                                            always});

    // HOVER -> MOVE (VELOCITY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::VELOCITY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            [this] { return allow_move_; },
                                            always});

    // HOVER -> MOVE (TRAJECTORY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::TRAJECTORY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            [this] { return allow_move_; },
                                            always});

    // HOVER -> MOVE (POINT_WGS84)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::POINT_WGS84_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            [this] { return allow_move_; },
                                            always});

    // MOVE -> MOVE (POINT)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            [this] { return allow_move_; },
                                            always});

    // MOVE -> MOVE (VELOCITY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::VELOCITY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            [this] { return allow_move_; },
                                            always});

    // MOVE -> MOVE (TRAJECTORY)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::TRAJECTORY_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            [this] { return allow_move_; },
                                            always});

    // MOVE -> MOVE (POINT_WGS84)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_WGS84_REQUEST,
                                            sunray_fsm::SunrayState::MOVE,
                                            [this] { return allow_move_; },
                                            always});
    // MOVE -> HOVER (POINT_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            always});

    // MOVE -> HOVER (VELOCITY_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::VELOCITY_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            always});

    // MOVE -> HOVER (TRAJECTORY_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::TRAJECTORY_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            always});

    // MOVE -> HOVER (POINT_WGS84_COMPLETED)
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::POINT_WGS84_COMPLETED,
                                            sunray_fsm::SunrayState::HOVER,
                                            always,
                                            always});
    // HOVER -> RETURN
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::HOVER,
                                            sunray_fsm::SunrayEvent::RETURN_REQUEST,
                                            sunray_fsm::SunrayState::RETURN,
                                            [this] { return allow_move_; },
                                            always});

    // MOVE -> RETURN
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::MOVE,
                                            sunray_fsm::SunrayEvent::RETURN_REQUEST,
                                            sunray_fsm::SunrayState::RETURN,
                                            [this] { return allow_move_; },
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
    // LAND -> INIT
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::LAND,
                                            sunray_fsm::SunrayEvent::LAND_COMPLETED,
                                            sunray_fsm::SunrayState::INIT,
                                            always,
                                            always});
    // RETURN -> INIT
    sunray_state_transmit_table_.push_back({sunray_fsm::SunrayState::RETURN,
                                            sunray_fsm::SunrayEvent::RETURN_COMPLETED,
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
        fsm_state_ = sunray_fsm::SunrayState::EMERGENCY_KILL;
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
    if (handle_event(event)) {
        return true;
    }
    // handle_event() -> false 说明不是kill状态，按照正常流程往下走
    bool return_state;
    // 获取缓存的状态转移表
    const auto state_table = get_transition_table();
    // 使用迭代器 查找匹配字段
    for (const auto& t : state_table) {
        // 如果当前迭代器找到的状态转移规则不是针对当前状态的，则跳过
        if (t.current_state != fsm_state_) {
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
        if (allow_checkout_state) {
            fsm_state_ = t.transmit_state;
        }
        // (t.action ? t.action() : true) -> 如果t.action存在，则执行t.action()
        //                                   如果t.action不存在，则直接返回true
        const bool need_action = (t.action ? t.action() : true);
        // 但其是我们不关心action的结果，因为写到这里的时候，我意识到handle_event其实是用作controlcmd的回调和运动判断完成的回调，而不是一个高频更新的函数
        //  因此这里的action并不能够与无人机的运动相关联
        //  但是我认为保留这一项可能有一些好处，比如action可以作为日志，存储每一次状态切换的细节
        return_state = allow_checkout_state;
    }
    return return_state;
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
    loadBasicParam(root["basic_param"], fsm_config_.basic_param);
    loadProtectParam(root["protect_param"], fsm_config_.protect_param);
    loadMsgTimeoutParam(root["msg_timeout_param"], fsm_config_.msg_timeout_param);
    loadTakeoffLandParam(root["takeoff_land_param"], fsm_config_.takeoff_land_param);
    loadMissionErrorParam(root["mission_error_param"], fsm_config_.mission_error_param);
    loadLocalFenceParam(root["local_fence_param"], fsm_config_.local_fence_param);
    loadVelocityParam(root["velocity_param"], fsm_config_.velocity_param);
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
}
