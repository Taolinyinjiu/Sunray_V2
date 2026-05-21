/**
 * @file localization_fusion.cpp
 * @brief 定位融合节点的核心实现。
 *
 * 本文件负责把不同外部定位源发布的里程计统一转换到 Sunray 约定的local/global 坐标体系中，
 * 并对外发布标准化的 local_odom、global_odom、TF 关系以及 OdomState 状态快照。
 *
 * 主要流程：
 * 1. 启动时读取定位源配置，订阅外部 odom 和可选的重定位话题。
 * 2. 收到外部 odom 后，根据配置中的外参转换为 sunray_local 下的机体位姿。
 * 3. 根据 global_to_local_tf 计算 global_odom，并维护 local_to_base_tf。
 * 4. 收到重定位消息时，更新 global_to_local_tf，实现 local 系到 global 系的对齐。
 * 5. 定时检查 odom 是否超时，并发布 TF 与 OdomState，方便监控工具和 RViz 使用。
 */

#include "localization_fusion.hpp"

LocalizationFusion::LocalizationFusion(ros::NodeHandle& nh) {
    nh_ = nh;

    node_name = ros::this_node::getName();
    ros::NodeHandle private_nh("~");

    if (!private_nh.getParam("source_id", selected_source_id_)) {
        throw std::runtime_error("missing param " + node_name + "/source_id");
    }
    if (!private_nh.getParam("config_yamlfile_path", config_yamlfile_path_)) {
        throw std::runtime_error("missing param " + node_name + "/config_yamlfile_path");
    }

    bool use_private_agent_key = false;
    private_nh.param("use_private_agent_key", use_private_agent_key, false);
    agent_key_ = use_private_agent_key ? sunray_common::get_agent_key_from_private()
                                       : sunray_common::get_agent_key_from_global();
}

bool LocalizationFusion::Init() {
    if (!load_param()) {
        return false;
    }

    selected_source_.odometry_topic = sunray_common::replace_agent_key(selected_source_.odometry_topic, agent_key_);
    selected_source_.relocalization_topic = sunray_common::replace_agent_key(selected_source_.relocalization_topic, agent_key_);

    global_odometry_topic_ = sunray_common::replace_agent_key(global_odometry_topic_, agent_key_);
    local_odometry_topic_ = sunray_common::replace_agent_key(local_odometry_topic_, agent_key_);
    odom_state_topic_ = sunray_common::replace_agent_key(odom_state_topic_, agent_key_);

    // 【订阅话题】 外部定位源的odom话题
    odometry_sub_ = nh_.subscribe(selected_source_.odometry_topic, 50, &LocalizationFusion::odometry_callback, this);
    // 【订阅话题】 外部定位源的重定位话题
    if (!selected_source_.relocalization_topic.empty()) {
        relocalization_sub_ = nh_.subscribe(selected_source_.relocalization_topic, 50, &LocalizationFusion::relocalization_callback, this);
    }

    // 【发布话题】 local_odom、global_odom、odom_state
    local_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(local_odometry_topic_, 10);
    global_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(global_odometry_topic_, 10);
    odom_state_pub_ = nh_.advertise<sunray_msgs::OdomState>(odom_state_topic_, 10);

    // 【定时器】定时发布odom_state，默认为10Hz
    health_timer_ = nh_.createTimer(ros::Duration(0.1), &LocalizationFusion::healthtimer_callback, this);

    // odom_state 赋初值
    odom_state.external_source = selected_source_id_;
    odom_state.subtopic_name_external_odom = selected_source_.odometry_topic;
    odom_state.subtopic_name_external_relocalization = selected_source_.relocalization_topic;
    odom_state.pubtopic_name_local_odom = local_odometry_topic_;
    odom_state.pubtopic_name_global_odom = global_odometry_topic_;
    odom_state.odometry_valid = false;
    odom_state.odometry_update_hz = 0.0;

    const std::string agent_frame_prefix = strip_leading_slash(agent_key_);
    odom_state.world_frame_name = std::string("world");
    odom_state.global_frame_name = agent_frame_prefix + "/sunray_global";
    odom_state.local_frame_name = agent_frame_prefix + "/sunray_local";
    odom_state.base_frame_name = agent_frame_prefix + "/base_link";
    odom_state.world_to_global_tf = make_identity_transform(odom_state.world_frame_name, odom_state.global_frame_name);
    odom_state.global_to_local_tf = make_identity_transform(odom_state.global_frame_name, odom_state.local_frame_name);
    odom_state.local_to_base_tf = make_identity_transform(odom_state.local_frame_name, odom_state.base_frame_name);

    // 发布world和global的静态变换（默认为零变换）
    static_tf_broadcaster_.sendTransform(odom_state.world_to_global_tf);

    return true;
}

// 重要回调：处理外部定位源的odom数据
void LocalizationFusion::odometry_callback(const nav_msgs::OdometryConstPtr& msg) 
{
    // 收到消息更新则认为odom有效
    odom_state.odometry_valid = true;
    // 使用当前时间作为local_odom的时间戳
    odom_state.local_odom.header.stamp = ros::Time::now();
    // 根据传感器到机体中心的外参矩阵计算local_odom
    odom_state.local_odom = transform_source_odom_to_local(*msg);
    // 发布local_odom
    local_odom_pub_.publish(odom_state.local_odom);
    // 保存odom收到的时间戳，用于后续有效性判断
    odometry_last_receive_time_ = odom_state.local_odom.header.stamp;
    
    // 计算更新频率
    update_input_rate(odom_state.odometry_update_hz, hz_stamps_, odom_state.local_odom.header.stamp.toSec());
    // 根据local_odom计算local_to_base_tf
    update_local_to_base_tf_from_odom(odom_state.local_odom);
    // 根据local_odom和global_to_local_tf计算global_odom并发布
    build_global_odom_from_local(odom_state.local_odom, odom_state.global_odom);
    // 发布global_odom
    global_odom_pub_.publish(odom_state.global_odom);
}

// 重要回调：处理外部定位源的重定位数据
void LocalizationFusion::relocalization_callback(const nav_msgs::OdometryConstPtr& msg) 
{
    // 如果没有外部定位源的odom数据则返回
    if (!odom_state.odometry_valid) {
        return;
    }

    nav_msgs::Odometry relocalization_msg = *msg;
    // 计算global和local之间的转换关系global_to_local_tf
    tf2::Transform T_global_base;
    tf2::Transform T_local_base;
    tf2::fromMsg(relocalization_msg.pose.pose, T_global_base);
    tf2::fromMsg(odom_state.local_odom.pose.pose, T_local_base);

    const tf2::Transform T_global_local = T_global_base * T_local_base.inverse();
    odom_state.global_to_local_tf.header.frame_id = odom_state.global_frame_name;
    odom_state.global_to_local_tf.child_frame_id = odom_state.local_frame_name;
    odom_state.global_to_local_tf.transform = tf2::toMsg(T_global_local);
}

// 重要定时器
void LocalizationFusion::healthtimer_callback(const ros::TimerEvent& e) 
{
    // 判断外部定位源的odom是否超时
    odom_state.odometry_valid = !odometry_last_receive_time_.isZero() && (ros::Time::now() - odometry_last_receive_time_).toSec() <= selected_source_.timeout_s;

    if (odom_state.odometry_valid) 
    {
        // 发布local_to_base_tf
        odom_state.local_to_base_tf.header.stamp = ros::Time::now();
        tf_broadcaster_.sendTransform(odom_state.local_to_base_tf);
        // 发布global_to_local_tf
        odom_state.global_to_local_tf.header.stamp = ros::Time::now();
        tf_broadcaster_.sendTransform(odom_state.global_to_local_tf);
    }

    odom_state.header.stamp = ros::Time::now();
    // 发布odom_state
    odom_state_pub_.publish(odom_state);
}

bool LocalizationFusion::load_param() {
    selected_source_ = load_config_from_yaml(config_yamlfile_path_, selected_source_id_);
    if (selected_source_.source_id != -1) {
        return true;
    }
    return false;
}

nav_msgs::Odometry LocalizationFusion::transform_source_odom_to_local(const nav_msgs::Odometry& msg) const {
    nav_msgs::Odometry local_msg = msg;

    tf2::Transform T_source_pose;
    tf2::fromMsg(msg.pose.pose, T_source_pose);

    tf2::Transform T_source_base;
    const Eigen::Matrix4d& T = selected_source_.source_frame_to_base;
    tf2::Matrix3x3 rotation(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
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

    local_msg.header.frame_id = odom_state.local_frame_name;
    local_msg.child_frame_id = odom_state.base_frame_name;
    return local_msg;
}

bool LocalizationFusion::build_global_odom_from_local(const nav_msgs::Odometry& local_msg, nav_msgs::Odometry& global_msg) const {
    tf2::Transform T_global_local;
    tf2::Transform T_local_base;
    tf2::fromMsg(odom_state.global_to_local_tf.transform, T_global_local);
    tf2::fromMsg(local_msg.pose.pose, T_local_base);

    const tf2::Transform T_global_base = T_global_local * T_local_base;
    global_msg = local_msg;
    global_msg.header.frame_id = odom_state.global_frame_name;
    global_msg.child_frame_id = odom_state.base_frame_name;
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
    odom_state.local_to_base_tf.header.frame_id = odom_state.local_frame_name;
    odom_state.local_to_base_tf.child_frame_id = odom_state.base_frame_name;
    odom_state.local_to_base_tf.transform.translation.x = local_odom.pose.pose.position.x;
    odom_state.local_to_base_tf.transform.translation.y = local_odom.pose.pose.position.y;
    odom_state.local_to_base_tf.transform.translation.z = local_odom.pose.pose.position.z;
    odom_state.local_to_base_tf.transform.rotation = local_odom.pose.pose.orientation;
}
