#include "statemachine/sunray_fsm.hpp"
#include "statemachine/sunray_fsm_loadparam.hpp"
#include "string_uav_namespace_utils.hpp"
#include "utils/uav_param_utils.hpp"
#include <cmath>
#include <stdexcept>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>  // 引入Yaml-cpp库，用于读取yaml文件
#include <sunray_msgs/UAVControlFSMState.h>
#include "controller/px4_origin_controller.hpp"
#include "controller/geometric_controller.hpp"
#include "controller/raptor_controller.hpp"
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

namespace {

double round_to_3_decimals(double value) {
    return std::round(value * 1000.0) / 1000.0;
}

}  // namespace

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
    // 打印一次性静态信息
    show_static_info();
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

// 从yaml文件中加载参数
void Sunray_FSM::load_param() {
    // 1. 优先读取节点私有参数中的 uav_name/uav_id，单机场景再回退到全局参数
    uav_ns_ = sunray_control::load_uav_namespace_or_throw(nh_);
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
    // 标准化里程计话题
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
    // 状态机当前状态发布者
    sunray_fsm_state_pub_ =
        nh_.advertise<sunray_msgs::UAVControlFSMState>(uav_ns_ + "/sunray/fsm/state", 10);
    sunray_odom_debug_pub_ =
        nh_.advertise<nav_msgs::Odometry>(uav_ns_ + "/surnay/debug/odometrry", 10);
}

// 为状态机注册控制器
void Sunray_FSM::register_controller() {
    switch (fsm_config_.basic_param.controller_types) {
    case 0: {
        sunray_controller_ = std::make_shared<PX4_OriginController>(nh_);
        if (sunray_controller_->init() == false) {
            // 如果控制器初始化失败，抛出异常
            throw std::runtime_error("{Px4 Original Controller initialization failed");
        }
        break;
    }
    case 1: {
        sunray_controller_ = std::make_shared<Geometric_Controller>(nh_);
        if (sunray_controller_->init() == false) {
            throw std::runtime_error("{Geometric_Controller initialization failed");
        }
        break;
    }
    case 2: {
        sunray_controller_ = std::make_shared<Raptor_Controller>(nh_);
        if (sunray_controller_->init() == false) {
            throw std::runtime_error("{Raptor_Controller initialization failed");
        }
        break;
    }
        // 这里没有default分支，因为load_param()界定了fsm_config_.basic_param.controller_types只会有0，1，2三个值，不可能有其他值被传入
    }
}




// --------------------话题回调函数----------------------

// 外部里程计回调函数
void Sunray_FSM::local_odom_callback(const nav_msgs::Odometry& msg) {
    const control_common::UAVStateEstimate odom_sample(msg);
    const ros::Time receive_time = ros::Time::now();

    std::string invalid_reason;
    double odom_frequency_hz = 0.0;
    bool odom_below_target_rate = false;
    bool accept_sample = false;

    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        last_raw_odometry_ = odom_sample;

        if (!last_raw_odom_receive_time_.isZero()) {
            const double dt = (receive_time - last_raw_odom_receive_time_).toSec();
            if (dt > 1e-6) {
                odom_frequency_hz_ = 1.0 / dt;
            }
        }
        last_raw_odom_receive_time_ = receive_time;
        odom_meets_rate_target_ = odom_frequency_hz_ >= 100.0;
        odom_frequency_hz = odom_frequency_hz_;
        odom_below_target_rate = (odom_frequency_hz > 0.0) && !odom_meets_rate_target_;

        accept_sample = validate_odometry_sample(odom_sample, &invalid_reason);
        if (accept_sample) {
            last_odometry_ = odom_sample;
            last_valid_odom_receive_time_ = receive_time;
            has_valid_odometry_ = true;
        }
    }

    (void)invalid_reason;
    (void)odom_below_target_rate;
    (void)odom_frequency_hz;
}

// 外部里程计状态回调函数
void Sunray_FSM::localization_state_callback(const sunray_msgs::OdomStatus& msg) {
    // TODO
}

// 无人机控制指令回调函数
void Sunray_FSM::uav_control_cmd_callback(const sunray_msgs::UAVControlCMD& msg) {
    // 构造临时变量
    control_common::UavControlCmd temp_cmd(msg);
    if (fsm_config_.basic_param.controller_types == 2 &&
        (temp_cmd.control_cmd == control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY ||
         temp_cmd.control_cmd == control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY_BODY)) {
        ROS_WARN_THROTTLE(1.0,
                          "[Sunray_FSM][%s] Raptor controller does not support velocity commands, ignore request",
                          uav_ns_.c_str());
        return;
    }
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
    // 当前返航的目标点。这里对外发布时收敛到小数点后三位，避免出现接近0的浮点噪声。
    fsm_state_msg.home_point.x = round_to_3_decimals(home_point_.x());
    fsm_state_msg.home_point.y = round_to_3_decimals(home_point_.y());
    fsm_state_msg.home_point.z = round_to_3_decimals(home_point_.z());
    // 当前的控制指令(由于两者的数据类型不同，一个是uint8另一个是强类型枚举，但是由于值都是一一对应的，因此这里用类型转换)
    fsm_state_msg.control_cmd = static_cast<uint8_t>(last_cmd_snapshot.control_cmd);
    // 状态机的状态同样使用静态类型转换
    fsm_state_msg.sunray_fsm_state = static_cast<uint8_t>(fsm_state_snapshot);
    // 发布消息
    sunray_fsm_state_pub_.publish(fsm_state_msg);
}
