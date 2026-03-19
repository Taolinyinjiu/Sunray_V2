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
#include <std_msgs/String.h>

#include "localization_fusion_utils.hpp"

namespace localization_fusion {

LocalizationFusion::LocalizationFusion(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), private_nh_(pnh) {}

bool LocalizationFusion::Init() {
  if (!LoadRuntimeParams()) {	// 检查是否读取了运行时参数
    ROS_ERROR("[localization_fusion] Failed to load runtime params.");
    return false;
  }
  if (!LoadSourceConfigs()) {	// 检查能否读取到localization_sources.yaml中所构建的参数
    ROS_ERROR("[localization_fusion] Failed to load source configs.");
    return false;
  }

  const SourceConfig *cfg = FindSelectedSource();	// 读取launch文件中指定的source_id对应的定位源配置数据
  if (cfg == nullptr) {	// 如果配置数据为空指针，说明source_id对应的定位源配置数据可能没有被配置或者写对位置
    ROS_ERROR("[localization_fusion] source_id=%d not found in sources_list.",
              selected_source_id_);
    return false;
  }	
	// 检查配置文件与回环检测模式是否满足要求
  if (!ValidateMode(*cfg, selected_loop_mode_)) {
    ROS_ERROR("[localization_fusion] Invalid source/loop mode combination.");
    return false;
  }	
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
  local_odom_pub_ =
      nh_.advertise<nav_msgs::Odometry>("/sunray/localization/local_odom", 10);
  global_odom_pub_ =
      nh_.advertise<nav_msgs::Odometry>("/sunray/localization/global_odom", 10);
  odom_state_pub_ =
      nh_.advertise<std_msgs::String>("/sunray/localization/odom_state", 10);
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
void LocalizationFusion::LocalOdomCallback(const nav_msgs::OdometryConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_local_meas_ = *msg;
  has_local_meas_ = true;
  last_local_rx_time_ = ros::Time::now();
  ProcessLocalOdom(*msg);
}

// global系回调函数
void LocalizationFusion::GlobalOdomCallback(const nav_msgs::OdometryConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_global_meas_ = *msg;
  has_global_meas_ = true;
  last_global_rx_time_ = ros::Time::now();
  ProcessGlobalOdom(*msg);
}

// 定时器回调函数
void LocalizationFusion::OnHealthTimer(const ros::TimerEvent & /*e*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  const SourceConfig *cfg = FindSelectedSource();
  const double timeout_s = (cfg != nullptr) ? cfg->timeout_s : 0.2;

  local_odom_valid_ = has_local_meas_ && !IsTimedOut(last_local_rx_time_, timeout_s);
  global_odom_valid_ = has_global_meas_ && !IsTimedOut(last_global_rx_time_, timeout_s);
  PublishOdomState();
}

// 处理local系回调函数
void LocalizationFusion::ProcessLocalOdom(const nav_msgs::Odometry &local_msg) {
  PublishLocalOdom(local_msg);

  const SourceConfig *cfg = FindSelectedSource();
  if (cfg == nullptr) {
    return;
  }

  switch (cfg->capabilities.publish_mode) {
    case localization_fusion_types::PublishMode::SINGLE_COPY:
      PublishGlobalOdom(local_msg);
      break;
    case localization_fusion_types::PublishMode::DUAL_DIRECT:
      if (has_global_meas_) {
        PublishGlobalOdom(last_global_meas_);
      }
      break;
    case localization_fusion_types::PublishMode::INIT_TRANSFORM:
      TryUpdateMapToOdom();
      BroadcastMapToOdomTf();
      break;
    default:
      break;
  }
}

// 处理global系回调函数
void LocalizationFusion::ProcessGlobalOdom(const nav_msgs::Odometry &global_msg) {
  const SourceConfig *cfg = FindSelectedSource();
  if (cfg == nullptr) {
    return;
  }

  switch (cfg->capabilities.publish_mode) {
    case localization_fusion_types::PublishMode::SINGLE_COPY:
      PublishLocalOdom(global_msg);
      PublishGlobalOdom(global_msg);
      break;
    case localization_fusion_types::PublishMode::DUAL_DIRECT:
      PublishGlobalOdom(global_msg);
      if (has_local_meas_) {
        PublishLocalOdom(last_local_meas_);
      }
      break;
    case localization_fusion_types::PublishMode::INIT_TRANSFORM:
      PublishGlobalOdom(global_msg);
      break;
    default:
      break;
  }
}

// 将里程计数据发布到local系下
void LocalizationFusion::PublishLocalOdom(const nav_msgs::Odometry &msg) {
  local_odom_pub_.publish(msg);
}
// 将里程计数据发布到global系下
void LocalizationFusion::PublishGlobalOdom(const nav_msgs::Odometry &msg) {
  global_odom_pub_.publish(msg);
}
// 发布当前里程计状态
void LocalizationFusion::PublishOdomState() {
  std_msgs::String state_msg;
  state_msg.data =
      std::string("local_valid=") + (local_odom_valid_ ? "true" : "false") +
      ",global_valid=" + (global_odom_valid_ ? "true" : "false");
  odom_state_pub_.publish(state_msg);
}

// --------------回环检测相关----------------------
void LocalizationFusion::TryUpdateMapToOdom() {
  // 占位实现：后续根据 local/global 观测计算 map->odom 变换。
  map_to_odom_tf_.header.frame_id = "map";
  map_to_odom_tf_.child_frame_id = "odom";
}

void LocalizationFusion::BroadcastMapToOdomTf() {
  map_to_odom_tf_.header.stamp = ros::Time::now();
  tf_broadcaster_.sendTransform(map_to_odom_tf_);
}

// 检查当前传入的时间戳是否超时
bool LocalizationFusion::IsTimedOut(const ros::Time &last_stamp, double timeout_s) const {
  if (last_stamp.isZero()) {
    return true;
  }
  return (ros::Time::now() - last_stamp).toSec() > timeout_s;
}
// 根据传入的source_id输出对应的配置信息，如果没有则返回 nullptr
const LocalizationFusion::SourceConfig *LocalizationFusion::FindSelectedSource() const {
  auto it = sources_.find(selected_source_id_);
  if (it == sources_.end()) {
    return nullptr;
  }
  return &(it->second);
}

} // namespace localization_fusion
