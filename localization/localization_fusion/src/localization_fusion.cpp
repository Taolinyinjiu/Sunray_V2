/**
 * @file localization_fusion.cpp
 * @author your name (you@domain.com)
 * @brief 本文件作为localization_fusion.hpp中函数声明的实现
 * @version 0.1
 * @date 2026-03-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "localization_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <sunray_msgs/OdomStatus.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "localization_fusion_utils.hpp"

namespace localization_fusion {
namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

void SetZeroVector3(geometry_msgs::Vector3 &vec) {
  vec.x = 0.0;
  vec.y = 0.0;
  vec.z = 0.0;
}

void SetIdentityQuaternion(geometry_msgs::Quaternion &quat) {
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = 0.0;
  quat.w = 1.0;
}

void SetIdentityTransform(geometry_msgs::Transform &transform) {
  SetZeroVector3(transform.translation);
  SetIdentityQuaternion(transform.rotation);
}

void FillStatusOdomFields(const nav_msgs::Odometry &odom,
                          geometry_msgs::Vector3 &position,
                          geometry_msgs::Vector3 &velocity,
                          geometry_msgs::Vector3 &attitude_rpy_deg,
                          geometry_msgs::Quaternion &attitude_quat) {
  position.x = odom.pose.pose.position.x;
  position.y = odom.pose.pose.position.y;
  position.z = odom.pose.pose.position.z;

  velocity.x = odom.twist.twist.linear.x;
  velocity.y = odom.twist.twist.linear.y;
  velocity.z = odom.twist.twist.linear.z;

  SetZeroVector3(attitude_rpy_deg);
  SetIdentityQuaternion(attitude_quat);

  const auto &q_msg = odom.pose.pose.orientation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  if (q.length2() <= 1e-12) {
    return;
  }

  q.normalize();
  attitude_quat.x = q.x();
  attitude_quat.y = q.y();
  attitude_quat.z = q.z();
  attitude_quat.w = q.w();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  attitude_rpy_deg.x = roll * kRadToDeg;
  attitude_rpy_deg.y = pitch * kRadToDeg;
  attitude_rpy_deg.z = yaw * kRadToDeg;
}

std::string StripLeadingSlash(const std::string &frame_id) {
  if (!frame_id.empty() && frame_id.front() == '/') {
    return frame_id.substr(1);
  }
  return frame_id;
}

tf2::Quaternion NormalizeQuaternion(const geometry_msgs::Quaternion &q_msg) {
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  if (!std::isfinite(q_msg.w) || !std::isfinite(q_msg.x) ||
      !std::isfinite(q_msg.y) || !std::isfinite(q_msg.z) ||
      q.length2() <= 1e-12) {
    return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
  }
  q.normalize();
  return q;
}

tf2::Transform TfFromPose(const geometry_msgs::Pose &pose) {
  const tf2::Quaternion q = NormalizeQuaternion(pose.orientation);
  const tf2::Vector3 t(pose.position.x, pose.position.y, pose.position.z);
  return tf2::Transform(q, t);
}

geometry_msgs::Pose PoseFromTf(const tf2::Transform &tf) {
  geometry_msgs::Pose pose;
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  tf2::Quaternion q = tf.getRotation();
  if (q.length2() <= 1e-12) {
    q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
  } else {
    q.normalize();
  }
  pose.orientation = tf2::toMsg(q);
  return pose;
}

geometry_msgs::Transform TransformFromPose(const geometry_msgs::Pose &pose) {
  geometry_msgs::Transform t;
  t.translation.x = pose.position.x;
  t.translation.y = pose.position.y;
  t.translation.z = pose.position.z;
  t.rotation = tf2::toMsg(NormalizeQuaternion(pose.orientation));
  return t;
}

} // namespace

LocalizationFusion::LocalizationFusion(ros::NodeHandle &nh,
                                       ros::NodeHandle &pnh)
    : nh_(nh), private_nh_(pnh) {}

bool LocalizationFusion::Init() {
  if (!LoadRuntimeParams()) { // 检查是否读取了运行时参数
    ROS_ERROR("[localization_fusion] Failed to load runtime params.");
    return false;
  }
  if (!LoadSourceConfigs()) { // 检查能否读取到localization_sources.yaml中所构建的参数
    ROS_ERROR("[localization_fusion] Failed to load source configs.");
    return false;
  }

  const SourceConfig *cfg =
      FindSelectedSource(); // 读取launch文件中指定的source_id对应的定位源配置数据
  if (cfg ==
      nullptr) { // 如果配置数据为空指针，说明source_id对应的定位源配置数据可能没有被配置或者写对位置
    ROS_ERROR("[localization_fusion] source_id=%d not found in sources_list.",
              selected_source_id_);
    return false;
  }
  // 检查配置文件与回环检测模式是否满足要求
  if (!ValidateMode(*cfg, selected_relocalization_mode_)) {
    ROS_ERROR("[localization_fusion] Invalid source/loop mode combination.");
    return false;
  }
  global_frame_id_ = StripLeadingSlash(global_frame_id_);
  local_frame_id_ = StripLeadingSlash(local_frame_id_);
  base_frame_id_ = StripLeadingSlash(base_frame_id_);
  ResetGlobalToLocalIdentity();
  // 初始化ros发布者，订阅者，定时器
  SetupPublishers();
  SetupSubscribers();
  SetupTimers();
  // 返回初始化成功
  return true;
}

// 在实例化节点中运行单spin环节
void LocalizationFusion::Spin() { ros::spin(); }

// 设置local系，global系，odom_state发布者
void LocalizationFusion::SetupPublishers() {
  const std::string local_odom_topic =
      uav_ns.empty() ? "/sunray/localization/local_odom"
                     : uav_ns + "/sunray/localization/local_odom";
  const std::string global_odom_topic =
      uav_ns.empty() ? "/sunray/localization/global_odom"
                     : uav_ns + "/sunray/localization/global_odom";
  const std::string localization_status =
      uav_ns.empty() ? "/sunray/localization/odom_status"
                     : uav_ns + "/sunray/localization/odom_status";

  local_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(local_odom_topic, 10);
  global_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(global_odom_topic, 10);
  odom_state_pub_ =
      nh_.advertise<sunray_msgs::OdomStatus>(localization_status, 10);
}

// 设置local系，global系，loop_check对应的订阅者
void LocalizationFusion::SetupSubscribers() {
  // 引用当前选择的订阅源的配置信息
  const SourceConfig *cfg = FindSelectedSource();
  // 保护，如果是空指针则结束函数,因为其他部分已经做了保护性编程
  if (cfg == nullptr) {
    return;
  }
  // 如果配置文件指定了local系话题，则进行订阅
  if (localization_fusion_types::HasLocalTopic(cfg->source_topics)) {
    local_sub_ = nh_.subscribe(cfg->source_topics.local_topic, 50,
                               &LocalizationFusion::LocalOdomCallback, this);
  }
  // 如果配置文件指定了global系话题，则进行订阅
  if (localization_fusion_types::HasGlobalTopic(cfg->source_topics)) {
    global_sub_ = nh_.subscribe(cfg->source_topics.global_topic, 50,
                                &LocalizationFusion::GlobalOdomCallback, this);
  }

  if (selected_relocalization_mode_ != RelocalizationMode::DISABLE &&
      localization_fusion_types::HasRelocalizationTopic(cfg->source_topics)) {
    relocalization_sub_ =
        nh_.subscribe(cfg->source_topics.relocalization_topic, 10,
                      &LocalizationFusion::RelocalizationCallback, this);
  }
}

// 设置健康状态检查定时器
void LocalizationFusion::SetupTimers() {
  // 设置默认的检查频率为10Hz
  double health_rate_hz = 10.0;
  // 读取私有参数中指定的检查频率
  private_nh_.param<double>("health_rate_hz", health_rate_hz, 10.0);
  // 保证至少1Hz的检查频率
  health_rate_hz = std::max(1.0, health_rate_hz);
  // 注册定时器
  health_timer_ = nh_.createTimer(ros::Duration(1.0 / health_rate_hz),
                                  &LocalizationFusion::OnHealthTimer, this);
}

// local系里程计回调函数
void LocalizationFusion::LocalOdomCallback(
    const nav_msgs::OdometryConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_local_meas_ = *msg;
  has_local_meas_ = true;
  last_local_rx_time_ = ros::Time::now();
  ProcessLocalOdom(*msg);
}

// global系回调函数
void LocalizationFusion::GlobalOdomCallback(
    const nav_msgs::OdometryConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_global_meas_ = *msg;
  has_global_meas_ = true;
  last_global_rx_time_ = ros::Time::now();
  ProcessGlobalOdom(*msg);
}

// 重定位触发回调函数
void LocalizationFusion::RelocalizationCallback(
    const nav_msgs::OdometryConstPtr &msg) {
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  last_relocalization_meas_ = *msg;
  has_relocalization_meas_ = true;
  last_relocalization_rx_time_ = ros::Time::now();

  global_to_local_tf_.header.frame_id = global_frame_id_;
  global_to_local_tf_.child_frame_id = local_frame_id_;
  global_to_local_tf_.transform = TransformFromPose(msg->pose.pose);
}

// 定时器回调函数
void LocalizationFusion::OnHealthTimer(const ros::TimerEvent & /*e*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  const SourceConfig *cfg = FindSelectedSource();
  const double timeout_s = (cfg != nullptr) ? cfg->timeout_s : 0.2;

  local_odom_valid_ =
      has_local_meas_ && !IsTimedOut(last_local_rx_time_, timeout_s);
  global_odom_valid_ =
      has_global_meas_ && !IsTimedOut(last_global_rx_time_, timeout_s);
  PublishOdomState();

  const ros::Time now = ros::Time::now();
  if (selected_relocalization_mode_ == RelocalizationMode::DISABLE) {
    ResetGlobalToLocalIdentity();
  }
  BroadcastGlobalToLocalTf(now);
}

// 处理local系回调函数
void LocalizationFusion::ProcessLocalOdom(const nav_msgs::Odometry &local_msg) {
  const SourceConfig *cfg = FindSelectedSource();
  if (cfg == nullptr) {
    return;
  }

  // local -> base_link
  nav_msgs::Odometry local_out = local_msg;
  local_out.header.frame_id = local_frame_id_;
  local_out.child_frame_id = base_frame_id_;
  if (local_out.header.stamp.isZero()) {
    local_out.header.stamp = ros::Time::now();
  }
  PublishLocalOdom(local_out);

  // global -> local
  tf2::Transform T_g_l;
  tf2::fromMsg(global_to_local_tf_.transform, T_g_l);

  if (cfg->capabilities.publish_mode ==
      localization_fusion_types::PublishMode::DUAL_DIRECT) {
    if (has_global_meas_) {
      nav_msgs::Odometry global_out = last_global_meas_;
      global_out.header.frame_id = global_frame_id_;
      global_out.child_frame_id = base_frame_id_;
      if (global_out.header.stamp.isZero()) {
        global_out.header.stamp = local_out.header.stamp;
      }
      PublishGlobalOdom(global_out);
    }
    return;
  }

  // SINGLE_COPY / INIT_TRANSFORM：global_odom 由 local_odom + (global->local)
  // 构造
  const tf2::Transform T_l_b = TfFromPose(local_out.pose.pose);
  const tf2::Transform T_g_b = T_g_l * T_l_b;

  nav_msgs::Odometry global_out = local_out;
  global_out.header.frame_id = global_frame_id_;
  global_out.pose.pose = PoseFromTf(T_g_b);
  PublishGlobalOdom(global_out);
}

// 处理global系回调函数
void LocalizationFusion::ProcessGlobalOdom(
    const nav_msgs::Odometry &global_msg) {
  const SourceConfig *cfg = FindSelectedSource();
  if (cfg == nullptr) {
    return;
  }

  nav_msgs::Odometry global_out = global_msg;
  global_out.header.frame_id = global_frame_id_;
  global_out.child_frame_id = base_frame_id_;
  if (global_out.header.stamp.isZero()) {
    global_out.header.stamp = ros::Time::now();
  }
  PublishGlobalOdom(global_out);

  // global -> local
  tf2::Transform T_g_l;
  tf2::fromMsg(global_to_local_tf_.transform, T_g_l);

  if (cfg->capabilities.publish_mode ==
      localization_fusion_types::PublishMode::DUAL_DIRECT) {
    if (has_local_meas_) {
      nav_msgs::Odometry local_out = last_local_meas_;
      local_out.header.frame_id = local_frame_id_;
      local_out.child_frame_id = base_frame_id_;
      if (local_out.header.stamp.isZero()) {
        local_out.header.stamp = global_out.header.stamp;
      }
      PublishLocalOdom(local_out);
    }
    return;
  }

  // SINGLE_COPY / INIT_TRANSFORM：local_odom 由 global_odom +
  // inv(global->local) 构造
  const tf2::Transform T_g_b = TfFromPose(global_out.pose.pose);
  const tf2::Transform T_l_b = T_g_l.inverse() * T_g_b;

  nav_msgs::Odometry local_out = global_out;
  local_out.header.frame_id = local_frame_id_;
  local_out.pose.pose = PoseFromTf(T_l_b);
  PublishLocalOdom(local_out);
}

// 将里程计数据发布到local系下
void LocalizationFusion::PublishLocalOdom(const nav_msgs::Odometry &msg) {
  nav_msgs::Odometry out = msg;
  out.header.frame_id = local_frame_id_;
  out.child_frame_id = base_frame_id_;
  if (out.header.stamp.isZero()) {
    out.header.stamp = ros::Time::now();
  }
  local_odom_pub_.publish(out);

  BroadcastGlobalToLocalTf(out.header.stamp);
  BroadcastLocalToBaseTf(out);
}
// 将里程计数据发布到global系下
void LocalizationFusion::PublishGlobalOdom(const nav_msgs::Odometry &msg) {
  nav_msgs::Odometry out = msg;
  out.header.frame_id = global_frame_id_;
  out.child_frame_id = base_frame_id_;
  if (out.header.stamp.isZero()) {
    out.header.stamp = ros::Time::now();
  }
  global_odom_pub_.publish(out);

  BroadcastGlobalToLocalTf(out.header.stamp);
}
// 发布当前里程计状态
void LocalizationFusion::PublishOdomState() {
  sunray_msgs::OdomStatus state_msg;
  state_msg.header.stamp = ros::Time::now();
  state_msg.header.frame_id.clear();

  state_msg.external_source = 0u;
  state_msg.provides_local = false;
  state_msg.provides_global = false;
  state_msg.provides_loopcheck = false;
  state_msg.odom_valid = false;
  state_msg.loopcheck_valid =
      (selected_relocalization_mode_ != RelocalizationMode::DISABLE) &&
      has_relocalization_meas_;
  state_msg.publish_mode =
      static_cast<uint8_t>(localization_fusion_types::PublishMode::SINGLE_COPY);

  state_msg.external_local_topic.clear();
  state_msg.external_global_topic.clear();
  state_msg.external_loopcheck_topic.clear();

  SetZeroVector3(state_msg.local_position);
  SetZeroVector3(state_msg.local_velocity);
  SetZeroVector3(state_msg.local_attitude_rpy);
  SetIdentityQuaternion(state_msg.local_attitude_quat);

  SetZeroVector3(state_msg.global_position);
  SetZeroVector3(state_msg.global_velocity);
  SetZeroVector3(state_msg.global_attitude_rpy);
  SetIdentityQuaternion(state_msg.global_attitude_quat);

  state_msg.loopcheck_tf_parent_frame.clear();
  state_msg.loopcheck_tf_child_frame.clear();
  SetIdentityTransform(state_msg.loopcheck_tf);

  const SourceConfig *cfg = FindSelectedSource();
  if (cfg != nullptr) {
    state_msg.external_source = static_cast<uint8_t>(cfg->source_id);
    state_msg.provides_local = cfg->capabilities.provides_local;
    state_msg.provides_global = cfg->capabilities.provides_global;
    state_msg.external_local_topic = cfg->source_topics.local_topic;
    state_msg.external_global_topic = cfg->source_topics.global_topic;
    state_msg.external_loopcheck_topic =
        cfg->source_topics.relocalization_topic;
    state_msg.publish_mode =
        static_cast<uint8_t>(cfg->capabilities.publish_mode);

    bool supports_loopcheck = false;
    for (const auto mode : cfg->capabilities.supported_relocalization) {
      if (mode != RelocalizationMode::DISABLE) {
        supports_loopcheck = true;
        break;
      }
    }
    state_msg.provides_loopcheck = supports_loopcheck;

    if (cfg->capabilities.provides_local && cfg->capabilities.provides_global) {
      state_msg.odom_valid = local_odom_valid_ && global_odom_valid_;
    } else if (cfg->capabilities.provides_local) {
      state_msg.odom_valid = local_odom_valid_;
    } else if (cfg->capabilities.provides_global) {
      state_msg.odom_valid = global_odom_valid_;
    } else {
      state_msg.odom_valid = false;
    }
  } else {
    state_msg.odom_valid = local_odom_valid_ || global_odom_valid_;
  }

  if (local_odom_valid_) {
    state_msg.header.frame_id = last_local_meas_.header.frame_id;
    FillStatusOdomFields(last_local_meas_, state_msg.local_position,
                         state_msg.local_velocity, state_msg.local_attitude_rpy,
                         state_msg.local_attitude_quat);
  }

  if (global_odom_valid_) {
    if (state_msg.header.frame_id.empty()) {
      state_msg.header.frame_id = last_global_meas_.header.frame_id;
    }
    FillStatusOdomFields(
        last_global_meas_, state_msg.global_position, state_msg.global_velocity,
        state_msg.global_attitude_rpy, state_msg.global_attitude_quat);
  }

  if (!global_to_local_tf_.header.frame_id.empty() ||
      !global_to_local_tf_.child_frame_id.empty()) {
    state_msg.loopcheck_tf_parent_frame = global_to_local_tf_.header.frame_id;
    state_msg.loopcheck_tf_child_frame = global_to_local_tf_.child_frame_id;
    state_msg.loopcheck_tf = global_to_local_tf_.transform;
  }

  odom_state_pub_.publish(state_msg);
}

void LocalizationFusion::ResetGlobalToLocalIdentity() {
  global_to_local_tf_.header.frame_id = global_frame_id_;
  global_to_local_tf_.child_frame_id = local_frame_id_;
  global_to_local_tf_.transform.translation.x = 0.0;
  global_to_local_tf_.transform.translation.y = 0.0;
  global_to_local_tf_.transform.translation.z = 0.0;
  global_to_local_tf_.transform.rotation.x = 0.0;
  global_to_local_tf_.transform.rotation.y = 0.0;
  global_to_local_tf_.transform.rotation.z = 0.0;
  global_to_local_tf_.transform.rotation.w = 1.0;
}

void LocalizationFusion::BroadcastGlobalToLocalTf(const ros::Time &stamp) {
  global_to_local_tf_.header.stamp = stamp;
  tf_broadcaster_.sendTransform(global_to_local_tf_);
}

void LocalizationFusion::BroadcastLocalToBaseTf(
    const nav_msgs::Odometry &local_odom) {
  local_to_base_tf_.header.stamp = local_odom.header.stamp.isZero()
                                       ? ros::Time::now()
                                       : local_odom.header.stamp;
  local_to_base_tf_.header.frame_id = local_frame_id_;
  local_to_base_tf_.child_frame_id = base_frame_id_;
  local_to_base_tf_.transform = TransformFromPose(local_odom.pose.pose);
  tf_broadcaster_.sendTransform(local_to_base_tf_);
}

// 检查当前传入的时间戳是否超时
bool LocalizationFusion::IsTimedOut(const ros::Time &last_stamp,
                                    double timeout_s) const {
  if (last_stamp.isZero()) {
    return true;
  }
  return (ros::Time::now() - last_stamp).toSec() > timeout_s;
}
// 根据传入的source_id输出对应的配置信息，如果没有则返回 nullptr
const LocalizationFusion::SourceConfig *
LocalizationFusion::FindSelectedSource() const {
  auto it = sources_.find(selected_source_id_);
  if (it == sources_.end()) {
    return nullptr;
  }
  return &(it->second);
}

} // namespace localization_fusion
