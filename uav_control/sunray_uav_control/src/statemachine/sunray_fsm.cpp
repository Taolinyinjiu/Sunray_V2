#include "statemachine/sunray_fsm.hpp"
#include "statemachine/sunray_fsm_loadparam.hpp"
#include "agent_key_helper.hpp"
#include "utils/control_config_loader.hpp"
#include <cmath>
#include <ros/ros.h>
#include <stdexcept>
#include <yaml-cpp/yaml.h>  // 引入Yaml-cpp库，用于读取yaml文件
#include <sunray_msgs/UAVControlState.h>
#include "controller/px4_origin_controller.hpp"
#include "controller/geometric_controller.hpp"
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <algorithm>
#include <cctype>

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

geometry_msgs::Vector3 toVector3Msg(const Eigen::Vector3d& value) {
    geometry_msgs::Vector3 msg;
    msg.x = value.x();
    msg.y = value.y();
    msg.z = value.z();
    return msg;
}

bool is_streaming_move_cmd(control_common::UavControlCmd::ControlCmd cmd) {
    return cmd == control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY ||
           cmd == control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY_BODY ||
           cmd == control_common::UavControlCmd::ControlCmd::MOVE_TRAJECTORY;
}

bool command_requires_fresh_odometry(control_common::UavControlCmd::ControlCmd cmd) {
    switch (cmd) {
    case control_common::UavControlCmd::ControlCmd::TAKEOFF:
    case control_common::UavControlCmd::ControlCmd::LAND:
    case control_common::UavControlCmd::ControlCmd::RETURN:
    case control_common::UavControlCmd::ControlCmd::HOVER:
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT:
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY:
    case control_common::UavControlCmd::ControlCmd::MOVE_TRAJECTORY:
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_BODY:
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY_BODY:
        return true;
    case control_common::UavControlCmd::ControlCmd::KILL:
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_WGS84:
    case control_common::UavControlCmd::ControlCmd::UNDEFINE:
    default:
        return false;
    }
}

void split_agent_key(const std::string& agent_key, std::string& agent_name, int& agent_id) {
    std::string key = agent_key;
    if (!key.empty() && key.front() == '/') {
        key.erase(key.begin());
    }

    std::size_t digit_pos = key.size();
    while (digit_pos > 0 && std::isdigit(static_cast<unsigned char>(key[digit_pos - 1]))) {
        --digit_pos;
    }

    if (digit_pos == key.size()) {
        agent_name = key;
        agent_id = 0;
        return;
    }

    agent_name = key.substr(0, digit_pos);
    try {
        agent_id = std::stoi(key.substr(digit_pos));
    } catch (const std::exception&) {
        agent_id = 0;
    }
}

void update_input_rate(double& averaged_rate_hz,
                       std::deque<double>& sample_times_s,
                       const double sample_time_s,
                       const std::size_t window_size = 20) {
    if (!std::isfinite(sample_time_s)) {
        return;
    }

    if (!sample_times_s.empty() && sample_time_s <= sample_times_s.back()) {
        if (std::abs(sample_time_s - sample_times_s.back()) < 1e-6) {
            return;
        }
        sample_times_s.clear();
    }

    sample_times_s.push_back(sample_time_s);
    while (sample_times_s.size() > window_size) {
        sample_times_s.pop_front();
    }

    if (sample_times_s.size() < 2) {
        return;
    }

    const double duration_s = sample_times_s.back() - sample_times_s.front();
    if (duration_s > 1e-4) {
        averaged_rate_hz = static_cast<double>(sample_times_s.size() - 1) / duration_s;
    }
}

}  // namespace

// 构造函数
Sunray_FSM::Sunray_FSM(ros::NodeHandle& nh) {
    nh_ = nh;
    ros::NodeHandle private_nh("~");
    bool use_private_agent_key = false;
    private_nh.param("use_private_agent_key", use_private_agent_key, false);
    uav_ns_ = use_private_agent_key
                  ? sunray_common::get_agent_key_from_private()
                  : sunray_common::get_agent_key_from_global();
    split_agent_key(uav_ns_, agent_name_, agent_id_);
}
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
    init_controller();

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
    // 处理状态转移事件
    process_fsm_event_queue();
    // 发布状态
    pub_sunray_fsm_state();
}

// 从yaml文件中加载参数
void Sunray_FSM::load_param() {
    // 读取 base 配置，并可选叠加 airframe 配置。airframe 中相同参数优先级更高。
    YAML::Node root = sunray_config::load_control_config_or_throw("Sunray_FSM");
    // 顺利读取，取出各个字段对应的部分
    // 由于这里会写的很长，所以将他们分为几个不同的函数用来填充结构体
    loadBasicParam(root["basic_param"], fsm_config_.basic_param);
    loadMsgTimeoutParam(root["msg_timeout_param"], fsm_config_.msg_timeout_param);
    loadTakeoffLandParam(root["takeoff_land_param"], fsm_config_.takeoff_land_param);
    active_takeoff_relative_height_.store(fsm_config_.takeoff_land_param.takeoff_relative_height);
    active_takeoff_max_velocity_.store(fsm_config_.takeoff_land_param.takeoff_max_velocity);
    active_land_max_velocity_.store(fsm_config_.takeoff_land_param.land_max_velocity);
    loadLocalFenceParam(root["local_fence_param"], fsm_config_.local_fence_param);
    loadVelocityParam(root["velocity_param"], fsm_config_.velocity_param);
    loadOdomFilterParam(root["odom_filter_param"], fsm_config_.odom_filter_param);
    odom_kf_.init(fsm_config_.odom_filter_param);
    // 标准化里程计话题
    fsm_config_.basic_param.odom_topic_name =
        sunray_common::replace_agent_key(fsm_config_.basic_param.odom_topic_name, uav_ns_);
}
// 初始化订阅者
void Sunray_FSM::init_subscriber() {
    // 订阅外部里程计
    local_odom_sub_ = nh_.subscribe(
        fsm_config_.basic_param.odom_topic_name, 10, &Sunray_FSM::local_odom_callback, this);
    // 订阅无人机控制指令
    uav_control_cmd_sub_ = nh_.subscribe(
        uav_ns_ + "/sunray/uav_control/control_cmd", 10, &Sunray_FSM::uav_control_cmd_callback, this);
}

// 初始化发布者
void Sunray_FSM::init_publisher() {
    // 状态机当前状态发布者
    sunray_fsm_state_pub_ =
        nh_.advertise<sunray_msgs::UAVControlState>(uav_ns_ + "/sunray/uav_control/control_state", 10);
    sunray_odom_debug_pub_ =
        nh_.advertise<nav_msgs::Odometry>(uav_ns_ + "/surnay/debug/odometrry", 10);
}

// 为状态机注册控制器
void Sunray_FSM::init_controller() {
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
        // 这里没有default分支，因为 load_param() 已界定 controller_types 只会有 0 或 1。
    }
}

// --------------------话题回调函数----------------------

// 外部里程计回调函数
void Sunray_FSM::local_odom_callback(const nav_msgs::Odometry& msg) {
    const ros::Time receive_time = ros::Time::now();
    control_common::UAVStateEstimate odom_sample(msg);
    // 控制输入滤波使用接收时刻计算 dt，避免仿真器或上游节点发布的虚拟时间戳扰动滤波器。
    odom_sample.timestamp = receive_time;

    std::string invalid_reason;
    double odom_frequency_hz = 0.0;
    bool odom_below_target_rate = false;
    bool accept_sample = false;

    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        last_raw_odometry_ = odom_sample;
        last_self_odom_msg_ = msg;
        has_self_odom_msg_ = true;
        update_input_rate(odom_frequency_hz_, odom_rate_samples_s_, receive_time.toSec());
        last_raw_odom_receive_time_ = receive_time;
        odom_meets_rate_target_ = odom_frequency_hz_ >= 100.0;
        odom_frequency_hz = odom_frequency_hz_;
        odom_below_target_rate = (odom_frequency_hz > 0.0) && !odom_meets_rate_target_;

        accept_sample = validate_odometry_sample(odom_sample, &invalid_reason);
        if (accept_sample) {
            last_odometry_ = odom_kf_.update(odom_sample);
            last_valid_odom_receive_time_ = receive_time;
            has_valid_odometry_ = true;
        } else {
            has_valid_odometry_ = false;
            odom_kf_.reset();
        }
    }

    if (!accept_sample && !invalid_reason.empty()) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[Sunray_FSM] odometry rejected: " << invalid_reason);
    }
    (void)odom_below_target_rate;
    (void)odom_frequency_hz;
}

// 无人机控制指令回调函数
void Sunray_FSM::uav_control_cmd_callback(const sunray_msgs::UAVControlCMD& msg) {
    // 构造临时变量
    control_common::UavControlCmd temp_cmd(msg);
    // 强制使用收到指令的时刻作为时间戳，避免上游忘记填充 header.stamp 时
    // 让命令时间戳永远停留在 ros::Time(0)，进而导致
    // check_rosmsg_timeout 无法判定指令流断开，move 状态卡死无法切回 hover。
    temp_cmd.timestamp = ros::Time::now();
    sunray_fsm::SunrayState current_state;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        current_state = fsm_state_;
    }
    const bool can_refresh_streaming_move_cmd =
        current_state == sunray_fsm::SunrayState::MOVE && is_streaming_move_cmd(temp_cmd.control_cmd);
    if (command_requires_fresh_odometry(temp_cmd.control_cmd) && !has_fresh_valid_odometry()) {
        ROS_WARN_STREAM("[Sunray_FSM] command rejected: odometry is not fresh or valid, cmd="
                        << static_cast<int>(temp_cmd.control_cmd));
        return;
    }
    if (can_refresh_streaming_move_cmd) {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        last_control_cmd_ = temp_cmd;
        return;
    }
    if (temp_cmd.control_cmd == control_common::UavControlCmd::ControlCmd::TAKEOFF) {
        double takeoff_height = fsm_config_.takeoff_land_param.takeoff_relative_height;
        double takeoff_velocity = fsm_config_.takeoff_land_param.takeoff_max_velocity;
        std::string reject_reason;
        if (!resolve_takeoff_command_params(
                temp_cmd, &takeoff_height, &takeoff_velocity, &reject_reason)) {
            ROS_WARN_STREAM("[Sunray_FSM] takeoff command rejected: "
                            << (reject_reason.empty() ? "invalid takeoff parameters"
                                                      : reject_reason));
            return;
        }
    }
    // 将control_cmd中的指令，转换为sunray_fsm的事件请求
    switch (temp_cmd.control_cmd) {
    case control_common::UavControlCmd::ControlCmd::TAKEOFF:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::TAKEOFF_REQUEST, temp_cmd);
        break;
    case control_common::UavControlCmd::ControlCmd::LAND:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::LAND_REQUEST, temp_cmd);
        break;
    case control_common::UavControlCmd::ControlCmd::RETURN:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::RETURN_REQUEST, temp_cmd);
        break;
    case control_common::UavControlCmd::ControlCmd::KILL:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::KILL_REQUEST, temp_cmd);
        break;
    case control_common::UavControlCmd::ControlCmd::HOVER:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::HOVER_REQUEST, temp_cmd);
        break;
    // local系和body系都是同一个point_request和point_completed
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT:
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_BODY:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::POINT_REQUEST, temp_cmd);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY:
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY_BODY:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::VELOCITY_REQUEST, temp_cmd);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_TRAJECTORY:
        enqueue_fsm_event(sunray_fsm::SunrayEvent::TRAJECTORY_REQUEST, temp_cmd);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_WGS84:
        ROS_WARN_STREAM("[Sunray_FSM] MOVE_POINT_WGS84 command rejected: reserved capability is not supported by runtime control path");
        break;
    case control_common::UavControlCmd::ControlCmd::UNDEFINE:
    default:
        // 未定义指令不入队，避免污染事件流
        ROS_WARN_STREAM("[Sunray_FSM] unsupported or undefined command: "
                        << static_cast<int>(temp_cmd.control_cmd));
        break;
    }
}

// ----------------定时器回调函数----------------
// 发布Sunray_FSM的相关信息
void Sunray_FSM::pub_sunray_fsm_state() {
    // >  这里我们删除了px4state消息类型，因为我们想要让状态机与px4相关的，进行一个解耦
    // 打一份快照
    sunray_fsm::SunrayState fsm_state_snapshot;
    control_common::UavControlCmd last_cmd_snapshot;
    nav_msgs::Odometry self_odom_snapshot;
    bool has_valid_odometry_snapshot = false;
    bool has_self_odom_msg_snapshot = false;
    ros::Time last_valid_odom_receive_time_snapshot{0.0};
    {
        std::scoped_lock lock(state_mutex_, cmd_mutex_);
        fsm_state_snapshot = fsm_state_;
        last_cmd_snapshot = last_control_cmd_;
    }
    const ros::Time publish_time = ros::Time::now();
    bool odometry_usable_snapshot = false;
    {
        std::lock_guard<std::mutex> lk3(odom_mutex_);
        self_odom_snapshot = last_self_odom_msg_;
        has_valid_odometry_snapshot = has_valid_odometry_;
        has_self_odom_msg_snapshot = has_self_odom_msg_;
        last_valid_odom_receive_time_snapshot = last_valid_odom_receive_time_;
        odometry_usable_snapshot = has_valid_odometry_snapshot &&
                                   !last_valid_odom_receive_time_snapshot.isZero() &&
                                   (publish_time - last_valid_odom_receive_time_snapshot).toSec() <=
                                       fsm_config_.msg_timeout_param.local_odometry;
    }
    // 构建要发布的消息
    sunray_msgs::UAVControlState fsm_state_msg;
    // 首先填充话题头
    fsm_state_msg.header.stamp = publish_time;
    fsm_state_msg.agent_name = agent_name_;
    fsm_state_msg.agent_id = static_cast<uint8_t>(std::clamp(agent_id_, 0, 255));
    fsm_state_msg.controller_types = fsm_config_.basic_param.controller_types;
    // 当前起飞参数
    fsm_state_msg.takeoff_relative_height = active_takeoff_relative_height_.load();
    fsm_state_msg.takeoff_max_velocity = active_takeoff_max_velocity_.load();
    // 当前降落参数
    fsm_state_msg.land_type = fsm_config_.takeoff_land_param.land_type;
    fsm_state_msg.land_max_velocity = active_land_max_velocity_.load();
    // 当前返航的目标点。这里对外发布时收敛到小数点后三位，避免出现接近0的浮点噪声。
    fsm_state_msg.home_point.x = round_to_3_decimals(home_point_.x());
    fsm_state_msg.home_point.y = round_to_3_decimals(home_point_.y());
    fsm_state_msg.home_point.z = round_to_3_decimals(home_point_.z());
    // 当前的控制指令
    fsm_state_msg.last_cmd.header.stamp = last_cmd_snapshot.timestamp;
    fsm_state_msg.last_cmd.cmd_source = static_cast<uint8_t>(last_cmd_snapshot.cmd_source);
    fsm_state_msg.last_cmd.control_cmd = static_cast<uint8_t>(last_cmd_snapshot.control_cmd);
    fsm_state_msg.last_cmd.desired_pos = toVector3Msg(last_cmd_snapshot.position);
    fsm_state_msg.last_cmd.desired_vel = toVector3Msg(last_cmd_snapshot.velocity);
    fsm_state_msg.last_cmd.desired_acc = toVector3Msg(last_cmd_snapshot.acceleration);
    fsm_state_msg.last_cmd.desired_jerk = toVector3Msg(last_cmd_snapshot.jerk);
    fsm_state_msg.last_cmd.desired_body_xy_pos.x = last_cmd_snapshot.body_position_xy.x();
    fsm_state_msg.last_cmd.desired_body_xy_pos.y = last_cmd_snapshot.body_position_xy.y();
    fsm_state_msg.last_cmd.desired_body_xy_vel.x = last_cmd_snapshot.body_velocity_xy.x();
    fsm_state_msg.last_cmd.desired_body_xy_vel.y = last_cmd_snapshot.body_velocity_xy.y();
    fsm_state_msg.last_cmd.fixed_height = last_cmd_snapshot.fixed_height;
    fsm_state_msg.last_cmd.takeoff_relative_height = last_cmd_snapshot.takeoff_relative_height;
    fsm_state_msg.last_cmd.takeoff_max_velocity = last_cmd_snapshot.takeoff_max_velocity;
    fsm_state_msg.last_cmd.land_max_velocity = last_cmd_snapshot.land_max_velocity;
    fsm_state_msg.last_cmd.desired_wgs84_pos = last_cmd_snapshot.wgs84_position;
    fsm_state_msg.last_cmd.yaw_mode = static_cast<uint8_t>(last_cmd_snapshot.yaw_mode);
    fsm_state_msg.last_cmd.desired_yaw = last_cmd_snapshot.yaw;
    fsm_state_msg.last_cmd.desired_yaw_rate = last_cmd_snapshot.yaw_rate;
    fsm_state_msg.last_cmd.yunlink_session_id = last_cmd_snapshot.yunlink_session_id;
    fsm_state_msg.last_cmd.yunlink_message_id = last_cmd_snapshot.yunlink_message_id;
    fsm_state_msg.last_cmd.yunlink_correlation_id = last_cmd_snapshot.yunlink_correlation_id;
    // 状态机状态
    fsm_state_msg.control_state = static_cast<uint8_t>(fsm_state_snapshot);
    if (has_self_odom_msg_snapshot) {
        fsm_state_msg.self_odom = self_odom_snapshot;
    }
    mavros_msgs::PositionTarget position_target;
    mavros_msgs::AttitudeTarget attitude_target;
    if (odometry_usable_snapshot && sunray_controller_ &&
        sunray_controller_->get_last_position_target(position_target)) {
        fsm_state_msg.controller_output_type = sunray_msgs::UAVControlState::OUTPUT_POSITION_TARGET;
        fsm_state_msg.position_target = position_target;
    } else if (odometry_usable_snapshot && sunray_controller_ &&
               sunray_controller_->get_last_attitude_target(attitude_target)) {
        fsm_state_msg.controller_output_type = sunray_msgs::UAVControlState::OUTPUT_ATTITUDE_TARGET;
        fsm_state_msg.attitude_target = attitude_target;
    } else {
        fsm_state_msg.controller_output_type = sunray_msgs::UAVControlState::OUTPUT_NONE;
    }
    // 与已有消息定义保持兼容：仅暴露关键的里程计健康状态
    fsm_state_msg.odometry_lost = !odometry_usable_snapshot;
    fsm_state_msg.odometry_valid = odometry_usable_snapshot;
    // 发布消息
    sunray_fsm_state_pub_.publish(fsm_state_msg);
}
