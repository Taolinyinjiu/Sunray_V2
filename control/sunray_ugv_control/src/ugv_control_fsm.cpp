#include "sunray_ugv_control/ugv_control_fsm.h"

#include "sunray_ugv_control/ugv_param_utils.h"
#include <sunray_msgs/UGVControlFSMState.h>
#include <yaml-cpp/yaml.h>

namespace sunray_ugv_control {

namespace {

template <typename T>
T load_scalar(const YAML::Node& node, const char* key, const T& fallback) {
    return node && node[key] ? node[key].as<T>() : fallback;
}

PIDGains load_pid(const YAML::Node& node,
                  const char* kp_key,
                  const char* ki_key,
                  const char* kd_key) {
    PIDGains gains;
    gains.kp = load_scalar(node, kp_key, 0.0);
    gains.ki = load_scalar(node, ki_key, 0.0);
    gains.kd = load_scalar(node, kd_key, 0.0);
    return gains;
}

}  // namespace

UGVControlFSM::UGVControlFSM(ros::NodeHandle& nh) : nh_(nh) {}

void UGVControlFSM::init() {
    load_config();
    init_publishers();
    init_subscribers();

    // 控制器只负责底层跟踪，所有安全相关决策都留在 FSM 中统一处理。
    controller_ = std::make_shared<UGVController>(nh_,
                                                  config_.controller,
                                                  config_.basic.controller_state_topic_name);

    active_cmd_ = make_hold_command();
    control_timer_ =
        nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.basic.controller_update_frequency)),
                        &UGVControlFSM::control_timer_callback,
                        this);
}

double UGVControlFSM::get_update_frequency() const {
    return config_.basic.supervisor_update_frequency;
}

void UGVControlFSM::load_config() {
    ugv_ns_ = load_ugv_namespace_or_throw(nh_);

    ros::NodeHandle private_nh("~");
    std::string config_path;
    if (!private_nh.getParam("config_yamlfile_path", config_path) || config_path.empty()) {
        throw std::runtime_error("missing param ~config_yamlfile_path");
    }

    const YAML::Node config = YAML::LoadFile(config_path);

    // 先把话题中的命名空间占位符展开，后续节点逻辑只处理实际话题名。
    const YAML::Node basic = config["basic_param"];
    config_.basic.ugv_type = load_scalar(basic, "ugv_types", 0);
    config_.basic.controller_update_frequency =
        load_scalar(basic, "controller_update_frequency", 100.0);
    config_.basic.supervisor_update_frequency =
        load_scalar(basic, "supervisor_update_frequency", 20.0);
    config_.basic.controller_state_pub_frequency =
        load_scalar(basic, "controller_state_pub_frequency", 100.0);
    config_.basic.odom_topic_name =
        replace_ugv_ns(load_scalar<std::string>(basic,
                                                "odom_topic_name",
                                                "${ugv_ns}/sunray/localization/local_odom"),
                       ugv_ns_);
    config_.basic.odom_status_topic_name =
        replace_ugv_ns(load_scalar<std::string>(basic,
                                                "odom_status_topic_name",
                                                "${ugv_ns}/sunray/localization/odom_status"),
                       ugv_ns_);
    config_.basic.control_cmd_topic_name =
        replace_ugv_ns(load_scalar<std::string>(basic,
                                                "control_cmd_topic_name",
                                                "${ugv_ns}/sunray/ugv_control/control_cmd"),
                       ugv_ns_);
    config_.basic.cmd_vel_topic_name =
        replace_ugv_ns(load_scalar<std::string>(basic, "cmd_vel_topic_name", "${ugv_ns}/sunray/ugv_control/cmd_vel"),
                       ugv_ns_);
    config_.basic.fsm_state_topic_name =
        replace_ugv_ns(load_scalar<std::string>(basic,
                                                "fsm_state_topic_name",
                                                "${ugv_ns}/sunray/ugv_control/ugv_control_fsm_state"),
                       ugv_ns_);
    config_.basic.controller_state_topic_name =
        replace_ugv_ns(load_scalar<std::string>(basic,
                                                "controller_state_topic_name",
                                                "${ugv_ns}/sunray/ugv_control/ugv_controller_state"),
                       ugv_ns_);

    const YAML::Node timeout = config["timeout_param"];
    config_.timeout.wait_poscmd_time = load_scalar(timeout, "wait_poscmd_time", 2.0);
    config_.timeout.wait_velcmd_time = load_scalar(timeout, "wait_velcmd_time", 0.3);
    config_.timeout.odom_timeout = load_scalar(timeout, "odom_timeout", 0.5);

    const YAML::Node fence = config["geo_fence_param"];
    config_.fence.x_min = load_scalar(fence, "x_min", -10.0);
    config_.fence.x_max = load_scalar(fence, "x_max", 10.0);
    config_.fence.y_min = load_scalar(fence, "y_min", -10.0);
    config_.fence.y_max = load_scalar(fence, "y_max", 10.0);
    config_.fence.z_min = load_scalar(fence, "z_min", -1.0);
    config_.fence.z_max = load_scalar(fence, "z_max", 1.0);

    const YAML::Node home = config["home_param"];
    config_.home.use_current_pose_as_home = load_scalar(home, "use_current_pose_as_home", true);
    config_.home.home_point.x = load_scalar(home, "home_x", 0.0);
    config_.home.home_point.y = load_scalar(home, "home_y", 0.0);
    config_.home.home_point.z = load_scalar(home, "home_z", 0.0);
    config_.home.home_yaw = load_scalar(home, "home_yaw", 0.0);
    home_runtime_ = config_.home;
    home_initialized_ = !config_.home.use_current_pose_as_home;

    const YAML::Node controller = config["controller_param"];
    config_.controller.ugv_type = config_.basic.ugv_type;
    config_.controller.state_pub_frequency = config_.basic.controller_state_pub_frequency;
    config_.controller.point_x = load_pid(controller, "point_x_kp", "point_x_ki", "point_x_kd");
    config_.controller.point_y = load_pid(controller, "point_y_kp", "point_y_ki", "point_y_kd");
    config_.controller.point_yaw =
        load_pid(controller, "point_yaw_kp", "point_yaw_ki", "point_yaw_kd");
    config_.controller.vel_x = load_pid(controller, "vel_x_kp", "vel_x_ki", "vel_x_kd");
    config_.controller.vel_y = load_pid(controller, "vel_y_kp", "vel_y_ki", "vel_y_kd");
    config_.controller.vel_yaw =
        load_pid(controller, "vel_yaw_kp", "vel_yaw_ki", "vel_yaw_kd");
    config_.controller.lateral_to_yaw_gain = load_scalar(controller, "lateral_to_yaw_gain", 1.0);
    config_.controller.max_linear_x = load_scalar(controller, "max_linear_x", 1.0);
    config_.controller.max_linear_y = load_scalar(controller, "max_linear_y", 1.0);
    config_.controller.max_angular_z = load_scalar(controller, "max_angular_z", 1.0);
    config_.controller.goal_pos_tolerance = load_scalar(controller, "goal_pos_tolerance", 0.15);
    config_.controller.goal_yaw_tolerance = load_scalar(controller, "goal_yaw_tolerance", 0.20);
}

void UGVControlFSM::init_subscribers() {
    odom_sub_ = nh_.subscribe(config_.basic.odom_topic_name, 20, &UGVControlFSM::odom_callback, this);
    odom_status_sub_ =
        nh_.subscribe(config_.basic.odom_status_topic_name, 10, &UGVControlFSM::odom_status_callback, this);
    control_cmd_sub_ =
        nh_.subscribe(config_.basic.control_cmd_topic_name, 20, &UGVControlFSM::control_cmd_callback, this);
}

void UGVControlFSM::init_publishers() {
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(config_.basic.cmd_vel_topic_name, 10);
    fsm_state_pub_ =
        nh_.advertise<sunray_msgs::UGVControlFSMState>(config_.basic.fsm_state_topic_name, 10);
}

void UGVControlFSM::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_odom_ = *msg;
    last_odom_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    odom_received_ = true;
    inside_geo_fence_ = check_geo_fence(last_odom_.pose.pose.position.x,
                                        last_odom_.pose.pose.position.y,
                                        last_odom_.pose.pose.position.z,
                                        config_.fence);

    try_initialize_home_locked(ros::Time::now());
}

void UGVControlFSM::odom_status_callback(const sunray_msgs::OdomStatus::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_status_received_ = true;
    odom_status_valid_ = msg->has_odometry && !msg->odom_timeout;
}

void UGVControlFSM::control_cmd_callback(const sunray_msgs::UGVControlCMD::ConstPtr& msg) {
    sunray_msgs::UGVControlCMD cmd = *msg;
    if (cmd.header.stamp.isZero()) {
        cmd.header.stamp = ros::Time::now();
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_cmd_ = cmd;
        // 回调函数只负责缓存最新指令和目标状态。
        // 实际控制输出统一放在定时器线程里执行。
        switch (cmd.control_cmd) {
        case sunray_msgs::UGVControlCMD::HOLD:
            state_ = State::HOLD;
            break;
        case sunray_msgs::UGVControlCMD::RETURN:
            state_ = State::RETURN;
            break;
        case sunray_msgs::UGVControlCMD::MOVE_POINT:
        case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
        case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
        case sunray_msgs::UGVControlCMD::MOVE_WGS84:
            state_ = State::MOVE;
            break;
        default:
            state_ = State::HOLD;
            active_cmd_ = make_hold_command();
            break;
        }
    }

    if (cmd.control_cmd == sunray_msgs::UGVControlCMD::HOLD) {
        publish_zero_cmd();
    }
}

bool UGVControlFSM::is_motion_command(const sunray_msgs::UGVControlCMD& cmd) const {
    switch (cmd.control_cmd) {
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
        return true;
    default:
        return false;
    }
}

bool UGVControlFSM::is_command_fresh(const sunray_msgs::UGVControlCMD& cmd, const ros::Time& now) const {
    if (cmd.header.stamp.isZero()) {
        return false;
    }

    const double age = (now - cmd.header.stamp).toSec();
    // 单次点位任务持续执行到到点或安全回退；流式速度指令必须持续刷新。
    switch (cmd.control_cmd) {
    case sunray_msgs::UGVControlCMD::MOVE_POINT:
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
        return true;
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
        return age <= config_.timeout.wait_velcmd_time;
    default:
        return true;
    }
}

bool UGVControlFSM::odom_is_valid_locked(const ros::Time& now) const {
    if (!odom_received_) {
        return false;
    }

    const bool local_timeout = (now - last_odom_stamp_).toSec() > config_.timeout.odom_timeout;
    // 优先使用 localization_fusion 汇总后的状态，
    // 但仍然保留本地超时检测，避免外部状态失效时失去保护。
    if (odom_status_received_) {
        return odom_status_valid_ && !local_timeout;
    }
    return !local_timeout;
}

void UGVControlFSM::try_initialize_home_locked(const ros::Time& now) {
    if (home_initialized_ || !config_.home.use_current_pose_as_home ||
        !odom_status_received_ || !odom_is_valid_locked(now)) {
        return;
    }

    home_runtime_.home_point = last_odom_.pose.pose.position;
    home_runtime_.home_yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
    home_initialized_ = true;
}

void UGVControlFSM::enter_hold(const std::string& reason) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ != State::HOLD) {
            ROS_WARN_STREAM("switch ugv fsm to HOLD: " << reason);
        }
        state_ = State::HOLD;
        active_cmd_ = make_hold_command();
    }
    publish_zero_cmd();
}

void UGVControlFSM::publish_zero_cmd() {
    geometry_msgs::Twist zero_cmd;
    cmd_vel_pub_.publish(zero_cmd);
}

sunray_msgs::UGVControlCMD UGVControlFSM::make_hold_command() const {
    sunray_msgs::UGVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.control_cmd = sunray_msgs::UGVControlCMD::HOLD;
    return cmd;
}

sunray_msgs::UGVControlCMD UGVControlFSM::make_return_point_command() const {
    sunray_msgs::UGVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_POINT;
    cmd.desired_pos = home_runtime_.home_point;
    cmd.desired_yaw = home_runtime_.home_yaw;
    return cmd;
}

void UGVControlFSM::control_timer_callback(const ros::TimerEvent&) {
    UGVKinematicState current_state;
    State current_fsm_state = State::INIT;
    bool odom_valid = false;
    bool home_ready = false;
    sunray_msgs::UGVControlCMD active_cmd;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_fsm_state = state_;
        active_cmd = active_cmd_;
        odom_valid = odom_is_valid_locked(ros::Time::now());
        home_ready = home_initialized_;

        current_state.position = last_odom_.pose.pose.position;
        current_state.velocity = last_odom_.twist.twist.linear;
        current_state.yaw = quaternion_to_yaw(last_odom_.pose.pose.orientation);
    }

    // 先对共享状态做一次快照，再在不持有 FSM 锁的情况下完成控制计算。
    controller_->set_current_state(current_state);

    if (!odom_valid) {
        cmd_vel_pub_.publish(controller_->hold());
        return;
    }

    switch (current_fsm_state) {
    case State::INIT:
    case State::HOLD:
        cmd_vel_pub_.publish(controller_->hold());
        return;

    case State::RETURN: {
        if (!home_ready) {
            cmd_vel_pub_.publish(controller_->hold());
            return;
        }
        // RETURN 被实现为朝缓存 home 点执行的一次性点位跟踪任务。
        const sunray_msgs::UGVControlCMD return_cmd = make_return_point_command();
        const geometry_msgs::Twist cmd_vel = controller_->move_point(return_cmd);
        cmd_vel_pub_.publish(cmd_vel);
        if (controller_->reached_point(return_cmd)) {
            enter_hold("return target reached");
        }
        return;
    }

    case State::MOVE:
        break;
    }

    switch (active_cmd.control_cmd) {
    case sunray_msgs::UGVControlCMD::MOVE_POINT: {
        const geometry_msgs::Twist cmd_vel = controller_->move_point(active_cmd);
        cmd_vel_pub_.publish(cmd_vel);
        if (controller_->reached_point(active_cmd)) {
            enter_hold("move point target reached");
        }
        break;
    }
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY:
        cmd_vel_pub_.publish(controller_->move_velocity(active_cmd));
        break;
    case sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY:
        cmd_vel_pub_.publish(controller_->move_velocity_body(active_cmd));
        break;
    case sunray_msgs::UGVControlCMD::MOVE_WGS84:
        // 需求中要求保留该接口；当前阶段为了安全起见，不执行 WGS84 控制。
        cmd_vel_pub_.publish(controller_->hold());
        break;
    default:
        cmd_vel_pub_.publish(controller_->hold());
        break;
    }
}

void UGVControlFSM::process() {
    const ros::Time now = ros::Time::now();
    bool odom_valid = false;
    bool inside_geo_fence = true;
    State current_state = State::INIT;
    sunray_msgs::UGVControlCMD active_cmd;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_valid = odom_is_valid_locked(now);
        inside_geo_fence = inside_geo_fence_;
        current_state = state_;
        active_cmd = active_cmd_;
    }

    // 所有因安全原因触发的 HOLD 回退，都在这个低频监督循环里统一处理。
    if ((current_state == State::MOVE || current_state == State::RETURN) && !odom_valid) {
        enter_hold("odometry timeout");
    }

    if (!inside_geo_fence) {
        enter_hold("geo fence violated");
    }

    if (current_state == State::MOVE && is_motion_command(active_cmd) &&
        !is_command_fresh(active_cmd, now)) {
        enter_hold("control command timeout");
    }

    publish_fsm_state();
}

void UGVControlFSM::publish_fsm_state() {
    sunray_msgs::UGVControlFSMState msg;
    msg.header.stamp = ros::Time::now();
    msg.geo_fence_x_min = config_.fence.x_min;
    msg.geo_fence_x_max = config_.fence.x_max;
    msg.geo_fence_y_min = config_.fence.y_min;
    msg.geo_fence_y_max = config_.fence.y_max;
    msg.geo_fence_z_min = config_.fence.z_min;
    msg.geo_fence_z_max = config_.fence.z_max;

    sunray_msgs::UGVControlCMD active_cmd;
    State current_state = State::INIT;
    bool odom_valid = false;
    bool inside_geo_fence = true;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_cmd = active_cmd_;
        current_state = state_;
        odom_valid = odom_is_valid_locked(msg.header.stamp);
        inside_geo_fence = inside_geo_fence_;
        msg.home_point = home_runtime_.home_point;
    }

    // UI/调试话题由两部分组成：FSM 持有的安全状态 + 控制器持有的跟踪状态。
    const sunray_msgs::UGVControllerState controller_state = controller_->get_status_snapshot();
    msg.active_control_cmd = active_cmd;
    msg.fsm_state = static_cast<uint8_t>(current_state);
    msg.desired_pos = controller_state.desired_pos;
    msg.desired_vel = controller_state.desired_vel;
    msg.desired_yaw = controller_state.desired_yaw;
    msg.current_pos = controller_state.current_pos;
    msg.current_vel = controller_state.current_vel;
    msg.current_yaw = controller_state.current_yaw;
    msg.controller_cmd_vel = controller_state.cmd_vel;
    msg.odom_valid = odom_valid;
    msg.control_cmd_valid = !is_motion_command(active_cmd) || is_command_fresh(active_cmd, msg.header.stamp);
    msg.inside_geo_fence = inside_geo_fence;

    fsm_state_pub_.publish(msg);
}

double UGVControlFSM::quaternion_to_yaw(const geometry_msgs::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace sunray_ugv_control
