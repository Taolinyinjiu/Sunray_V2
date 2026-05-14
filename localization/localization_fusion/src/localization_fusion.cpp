#include "localization_fusion.hpp"

/*
最新执行模型（2026-05-14）：

1. localization_fusion 不再关心 world 系
2. TF 树固定为：
       sunray_global -> {agent}/sunray_local -> {agent}/base_link
3. odometry_topic 输入永远表示 local 主里程计输入
4. odometry_callback 中完成外参变换后立即发布 local_odom
5. relocalization_topic 输入是 base_link 在 sunray_global 下的位姿
6. relocalization_callback 只更新 T_global_local
7. health_timer_ 统一负责：
   - 计算并发布 global_odom
   - 发布 global_to_local_tf_
   - 发布 local_to_base_tf_
   - 检查超时
   - 发布 OdomState
*/

#include <cmath>
#include <deque>
#include <stdexcept>

#include <Eigen/Dense>
#include <sunray_msgs/OdomState.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "agent_key_helper.hpp"

namespace {

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
        const double raw_hz = static_cast<double>(sample_times_s.size() - 1) / duration_s;
        constexpr double alpha = 0.1;
        averaged_rate_hz = (averaged_rate_hz < 1e-6)
                               ? raw_hz
                               : alpha * raw_hz + (1.0 - alpha) * averaged_rate_hz;
    }
}

std::string strip_leading_slash(std::string value) {
    while (!value.empty() && value.front() == '/') {
        value.erase(value.begin());
    }
    return value;
}

bool is_unit_quaternion(const Eigen::Quaterniond& q) {
    return std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) &&
           std::isfinite(q.w()) &&
           std::abs(q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w() - 1.0) <
               1e-2;
}

geometry_msgs::Pose transform_to_pose_msg(const tf2::Transform& transform) {
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = transform.getOrigin().x();
    pose_msg.position.y = transform.getOrigin().y();
    pose_msg.position.z = transform.getOrigin().z();
    pose_msg.orientation = tf2::toMsg(transform.getRotation());
    return pose_msg;
}

geometry_msgs::TransformStamped make_identity_transform(const std::string& parent_frame,
                                                        const std::string& child_frame) {
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = ros::Time::now();
    transform_msg.header.frame_id = parent_frame;
    transform_msg.child_frame_id = child_frame;
    transform_msg.transform.translation.x = 0.0;
    transform_msg.transform.translation.y = 0.0;
    transform_msg.transform.translation.z = 0.0;
    transform_msg.transform.rotation.x = 0.0;
    transform_msg.transform.rotation.y = 0.0;
    transform_msg.transform.rotation.z = 0.0;
    transform_msg.transform.rotation.w = 1.0;
    return transform_msg;
}

}  // namespace

LocalizationFusion::LocalizationFusion(ros::NodeHandle& nh) {
    nh_ = nh;

    const std::string node_name = ros::this_node::getName();
    ros::NodeHandle private_nh("~");

    if (!private_nh.getParam("source_id", selected_source_id_)) {
        throw std::runtime_error("missing param " + node_name + "/source_id");
    }
    if (!private_nh.getParam("config_yamlfile_path", config_yamlfile_path_)) {
        throw std::runtime_error("missing param " + node_name + "/config_yamlfile_path");
    }
    if (!private_nh.getParam("health_rate_hz", health_rate_hz_)) {
        throw std::runtime_error("missing param " + node_name + "/health_rate_hz");
    }
    if (!private_nh.getParam("use_receive_time", use_receive_time_)) {
        throw std::runtime_error("missing param " + node_name + "/use_receive_time");
    }
    private_nh.param("tf_world_global", tf_world_global_, false);
    private_nh.param("tf_local_world", tf_local_world_, false);

    bool use_private_agent_key = false;
    private_nh.param("use_private_agent_key", use_private_agent_key, false);
    agent_key_ = use_private_agent_key ? sunray_common::get_agent_key_from_private()
                                       : sunray_common::get_agent_key_from_global();

    const std::string agent_frame_prefix = strip_leading_slash(agent_key_);
    private_nh.param("world_frame_id", world_frame_id_, std::string("world"));
    private_nh.param("global_frame_id", global_frame_id_, std::string("sunray_global"));
    private_nh.param("local_frame_id", local_frame_id_,
                     agent_frame_prefix + "/sunray_local");
    private_nh.param("base_frame_id", base_frame_id_, agent_frame_prefix + "/base_link");
}

bool LocalizationFusion::load_param() {
    selected_source_ = load_config_from_yaml(config_yamlfile_path_, selected_source_id_);
    if (selected_source_.source_id != -1) {
        has_selected_source_ = true;
        return true;
    }
    return false;
}

bool LocalizationFusion::Init() {
    if (!load_param()) {
        return false;
    }

    if (tf_world_global_ && tf_local_world_) {
        throw std::runtime_error(
            "tf_world_global and tf_local_world cannot both be true at the same time");
    }

    selected_source_.odometry_topic =
        sunray_common::replace_agent_key(selected_source_.odometry_topic, agent_key_);
    if (selected_source_.odometry_topic.empty()) {
        throw std::runtime_error("selected source config the odometry topic missing value");
    }

    selected_source_.relocalization_topic =
        sunray_common::replace_agent_key(selected_source_.relocalization_topic, agent_key_);
    has_relocalization_input_ = !selected_source_.relocalization_topic.empty();

    global_odometry_topic_ = sunray_common::replace_agent_key(global_odometry_topic_, agent_key_);
    local_odometry_topic_ = sunray_common::replace_agent_key(local_odometry_topic_, agent_key_);
    odom_state_topic_ = sunray_common::replace_agent_key(odom_state_topic_, agent_key_);
    if (global_odometry_topic_.empty() || local_odometry_topic_.empty() ||
        odom_state_topic_.empty()) {
        throw std::runtime_error("localization fusion config has empty topic");
    }

    odometry_sub_ = nh_.subscribe(
        selected_source_.odometry_topic, 50, &LocalizationFusion::odometry_callback, this);
    if (has_relocalization_input_) {
        relocalization_sub_ = nh_.subscribe(selected_source_.relocalization_topic,
                                            50,
                                            &LocalizationFusion::relocalization_callback,
                                            this);
    }

    local_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(local_odometry_topic_, 10);
    global_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(global_odometry_topic_, 10);
    odom_state_pub_ = nh_.advertise<sunray_msgs::OdomState>(odom_state_topic_, 10);

    health_rate_hz_ = std::max(1.0, health_rate_hz_);
    health_timer_ = nh_.createTimer(
        ros::Duration(1.0 / health_rate_hz_), &LocalizationFusion::healthtimer_callback, this);

    global_to_local_tf_.header.frame_id = global_frame_id_;
    global_to_local_tf_.child_frame_id = local_frame_id_;
    global_to_local_tf_.transform.translation.x = 0.0;
    global_to_local_tf_.transform.translation.y = 0.0;
    global_to_local_tf_.transform.translation.z = 0.0;
    global_to_local_tf_.transform.rotation.x = 0.0;
    global_to_local_tf_.transform.rotation.y = 0.0;
    global_to_local_tf_.transform.rotation.z = 0.0;
    global_to_local_tf_.transform.rotation.w = 1.0;
    global_local_tf_valid_ = false;
    odometry_valid_ = false;
    relocalization_valid_ = false;

    if (tf_world_global_) {
        static_tf_broadcaster_.sendTransform(
            make_identity_transform(world_frame_id_, global_frame_id_));
    }
    if (tf_local_world_) {
        static_tf_broadcaster_.sendTransform(
            make_identity_transform(local_frame_id_, world_frame_id_));
    }

    return true;
}

nav_msgs::Odometry
LocalizationFusion::transform_source_odom_to_local(const nav_msgs::Odometry& msg) const {
    nav_msgs::Odometry local_msg = msg;

    tf2::Transform T_source_pose;
    tf2::fromMsg(msg.pose.pose, T_source_pose);

    tf2::Transform T_source_base;
    const Eigen::Matrix4d& T = selected_source_.source_frame_to_base;
    tf2::Matrix3x3 rotation(
        T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
    T_source_base.setBasis(rotation);
    T_source_base.setOrigin(tf2::Vector3(T(0, 3), T(1, 3), T(2, 3)));

    const tf2::Transform T_local_base = T_source_pose * T_source_base;
    local_msg.pose.pose = transform_to_pose_msg(T_local_base);

    const tf2::Matrix3x3 R = T_source_base.getBasis();
    const tf2::Vector3 linear_src(msg.twist.twist.linear.x,
                                  msg.twist.twist.linear.y,
                                  msg.twist.twist.linear.z);
    const tf2::Vector3 angular_src(msg.twist.twist.angular.x,
                                   msg.twist.twist.angular.y,
                                   msg.twist.twist.angular.z);
    const tf2::Vector3 linear_dst = R * linear_src;
    const tf2::Vector3 angular_dst = R * angular_src;
    local_msg.twist.twist.linear.x = linear_dst.x();
    local_msg.twist.twist.linear.y = linear_dst.y();
    local_msg.twist.twist.linear.z = linear_dst.z();
    local_msg.twist.twist.angular.x = angular_dst.x();
    local_msg.twist.twist.angular.y = angular_dst.y();
    local_msg.twist.twist.angular.z = angular_dst.z();

    local_msg.header.frame_id = local_frame_id_;
    local_msg.child_frame_id = base_frame_id_;
    return local_msg;
}

bool LocalizationFusion::build_global_odom_from_local(const nav_msgs::Odometry& local_msg,
                                                      nav_msgs::Odometry& global_msg) const {
    tf2::Transform T_global_local;
    tf2::Transform T_local_base;
    tf2::fromMsg(global_to_local_tf_.transform, T_global_local);
    tf2::fromMsg(local_msg.pose.pose, T_local_base);

    const tf2::Transform T_global_base = T_global_local * T_local_base;
    global_msg = local_msg;
    global_msg.header.frame_id = global_frame_id_;
    global_msg.child_frame_id = base_frame_id_;
    global_msg.pose.pose = transform_to_pose_msg(T_global_base);

    const tf2::Quaternion q_global_local = T_global_local.getRotation();
    const tf2::Matrix3x3 R_global_local(q_global_local);
    const tf2::Vector3 linear_local(local_msg.twist.twist.linear.x,
                                    local_msg.twist.twist.linear.y,
                                    local_msg.twist.twist.linear.z);
    const tf2::Vector3 angular_local(local_msg.twist.twist.angular.x,
                                     local_msg.twist.twist.angular.y,
                                     local_msg.twist.twist.angular.z);
    const tf2::Vector3 linear_global = R_global_local * linear_local;
    const tf2::Vector3 angular_global = R_global_local * angular_local;
    global_msg.twist.twist.linear.x = linear_global.x();
    global_msg.twist.twist.linear.y = linear_global.y();
    global_msg.twist.twist.linear.z = linear_global.z();
    global_msg.twist.twist.angular.x = angular_global.x();
    global_msg.twist.twist.angular.y = angular_global.y();
    global_msg.twist.twist.angular.z = angular_global.z();
    return true;
}

void LocalizationFusion::update_local_to_base_tf_from_odom(const nav_msgs::Odometry& local_odom) {
    local_to_base_tf_.header.stamp = local_odom.header.stamp;
    local_to_base_tf_.header.frame_id = local_frame_id_;
    local_to_base_tf_.child_frame_id = base_frame_id_;
    local_to_base_tf_.transform.translation.x = local_odom.pose.pose.position.x;
    local_to_base_tf_.transform.translation.y = local_odom.pose.pose.position.y;
    local_to_base_tf_.transform.translation.z = local_odom.pose.pose.position.z;
    local_to_base_tf_.transform.rotation = local_odom.pose.pose.orientation;
}

void LocalizationFusion::odometry_callback(const nav_msgs::OdometryConstPtr& msg) {
    nav_msgs::Odometry local_msg = transform_source_odom_to_local(*msg);
    const ros::Time receive_time = ros::Time::now();
    if (use_receive_time_) {
        local_msg.header.stamp = receive_time;
    }

    local_odom_pub_.publish(local_msg);

    latest_local_odom_ = local_msg;
    last_published_local_odom_ = local_msg;
    odometry_received_stamp_ = local_msg.header.stamp;
    odometry_last_receive_time_ = receive_time;
    has_local_odom_ = true;
    odometry_valid_ = true;
    update_input_rate(odometry_update_hz, hz_stamps_, receive_time.toSec());
    update_local_to_base_tf_from_odom(local_msg);
}

void LocalizationFusion::relocalization_callback(const nav_msgs::OdometryConstPtr& msg) {
    nav_msgs::Odometry global_msg = *msg;
    const ros::Time receive_time = ros::Time::now();

    const Eigen::Vector3d position(global_msg.pose.pose.position.x,
                                   global_msg.pose.pose.position.y,
                                   global_msg.pose.pose.position.z);
    const Eigen::Quaterniond quat(global_msg.pose.pose.orientation.w,
                                  global_msg.pose.pose.orientation.x,
                                  global_msg.pose.pose.orientation.y,
                                  global_msg.pose.pose.orientation.z);

    const bool odometry_finite =
        std::isfinite(position.x()) && std::isfinite(position.y()) && std::isfinite(position.z());
    if (!odometry_finite || !is_unit_quaternion(quat)) {
        relocalization_valid_ = false;
        return;
    }

    relocalization_valid_ = true;
    latest_relocalization_odom_ = *msg;
    relocalization_received_stamp_ = msg->header.stamp;
    relocalization_last_receive_time_ = receive_time;
    has_relocalization_odom_ = true;

    if (!has_local_odom_) {
        return;
    }

    if (odometry_last_receive_time_.isZero() ||
        (receive_time - odometry_last_receive_time_).toSec() > selected_source_.timeout_s) {
        return;
    }

    tf2::Transform T_global_base;
    tf2::Transform T_local_base;
    tf2::fromMsg(global_msg.pose.pose, T_global_base);
    tf2::fromMsg(latest_local_odom_.pose.pose, T_local_base);

    const tf2::Transform T_global_local = T_global_base * T_local_base.inverse();
    global_to_local_tf_.header.stamp = global_msg.header.stamp;
    global_to_local_tf_.header.frame_id = global_frame_id_;
    global_to_local_tf_.child_frame_id = local_frame_id_;
    global_to_local_tf_.transform = tf2::toMsg(T_global_local);
    global_local_tf_valid_ = true;
}

void LocalizationFusion::healthtimer_callback(const ros::TimerEvent& e) {
    (void)e;
    const ros::Time now = ros::Time::now();

    if (has_local_odom_) {
        odometry_valid_ = !odometry_last_receive_time_.isZero() &&
                          (now - odometry_last_receive_time_).toSec() <=
                              selected_source_.timeout_s;
    } else {
        odometry_valid_ = false;
    }

    relocalization_valid_ = has_relocalization_input_ && relocalization_valid_ &&
                            has_relocalization_odom_;

    if (odometry_valid_) {
        nav_msgs::Odometry global_msg;
        if (build_global_odom_from_local(latest_local_odom_, global_msg)) {
            global_msg.header.stamp = latest_local_odom_.header.stamp;
            global_odom_pub_.publish(global_msg);
            last_published_global_odom_ = global_msg;
        }
    }

    if (odometry_valid_) {
        local_to_base_tf_.header.stamp = latest_local_odom_.header.stamp;
        tf_broadcaster_.sendTransform(local_to_base_tf_);
    }

    global_to_local_tf_.header.stamp = now;
    tf_broadcaster_.sendTransform(global_to_local_tf_);

    sunray_msgs::OdomState state_msgs;
    state_msgs.header.stamp = now;
    state_msgs.external_source = selected_source_id_;
    state_msgs.odometry_valid = odometry_valid_;
    state_msgs.relocalization_valid = relocalization_valid_;
    state_msgs.odometry_received_stamp = odometry_received_stamp_;
    state_msgs.relocalization_received_stamp = relocalization_received_stamp_;
    state_msgs.has_config_relocalization = has_relocalization_input_;
    state_msgs.odometry_update_hz = odometry_update_hz;
    state_msgs.global_frame_name = global_frame_id_;
    state_msgs.local_frame_name = local_frame_id_;
    state_msgs.base_frame_name = base_frame_id_;
    state_msgs.local_odom = last_published_local_odom_;
    state_msgs.global_odom = last_published_global_odom_;
    if (has_relocalization_odom_) {
        state_msgs.recive_relocalization_msg = latest_relocalization_odom_;
    }
    odom_state_pub_.publish(state_msgs);
}

void LocalizationFusion::Spin() {
    ros::spin();
}
