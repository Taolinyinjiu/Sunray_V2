/**
 * @file localization_fusion.h
 * @author your name (you@domain.com)
 * @brief
 * 设计意图：本模块负责对各种定位算法进行选择和检查，输出正确的全局定位话题和局部定位话题，且保证对px4无人机，无人机车，机器狗，DJI无人机通用
 * @version 0.1
 * @date 2026-03-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <ros/node_handle.h>

#pragma once

#include <mutex>
#include <unordered_map>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include "localization_fusion_types.hpp"

namespace localization_fusion {

class LocalizationFusion final {
public:
  explicit LocalizationFusion(ros::NodeHandle &nh, ros::NodeHandle &pnh);

  // 启动入口：读配置、校验模式、创建 pub/sub/timer
  bool Init();

  // 主循环（单线程 ros::spin）
  void Spin();

  // 运行时热切换模式接口（当前需求关闭，仅保留占位说明，后续需要时再开启）
  // bool SetMode(int source_id, localization_fusion_types::LoopMode loop_mode);

private:
 	// 使用using 简化命名空间编码
  using LoopMode = localization_fusion_types::LoopMode;
  // using PublishMode = localization_fusion_types::PublishMode; // TODO: 需要按发布策略分流时再启用
  using SourceConfig = localization_fusion_types::SourceConfig;

  // ---- 初始化阶段 ----
  bool LoadRuntimeParams();	// 加载运行时参数，这里指的是launch文件指定的定位源，当前的loop模式，定位源消息超时时间等参数
  bool LoadSourceConfigs();	// 加载定位源配置，需要先使用LoadRuntimeParams()得到指定的定位源以及定位源的source_id，然后从config.yaml中读取
  bool ValidateMode(const SourceConfig &cfg, LoopMode loop_mode) const;	// 在启动前检查 当前选择的 source + loop 组合是否符合Sunray项目准则、可正常运行
  void SetupPublishers();	// 设置ros话题发布者
  void SetupSubscribers();	// 设置ros话题订阅者，订阅local_topic,global_topic,loop_topic
  void SetupTimers();	// 设置定时器，目前为周期性的检查local/global是否收到数据，是否超时，更新local_odom_valid/global_odom_valid以及发布odom_state

  // ---- 回调函数 ----
  void LocalOdomCallback(const nav_msgs::OdometryConstPtr &msg);	// local系话题回调函数
  void GlobalOdomCallback(const nav_msgs::OdometryConstPtr &msg);	// global系话题回调函数
  // void LoopOdomCallback(const nav_msgs::OdometryConstPtr &msg); // TODO: 开启回环输入时启用
  void OnHealthTimer(const ros::TimerEvent &e);	// 周期检查 local/global 是否收到新数据,判断是否超时,更新 local_odom_valid/global_odom_valid,发布 odom_state

  // ---- 核心逻辑 ----
  void ProcessLocalOdom(const nav_msgs::Odometry &local_msg);		//处理“收到 local 输入”后的主流程：按当前 publish_mode 决定如何生成并发布 local/global
  void ProcessGlobalOdom(const nav_msgs::Odometry &global_msg);		//处理“收到 global 输入”后的主流程：更新缓存、按模式补齐另一侧输出或触发变换更新
  void PublishLocalOdom(const nav_msgs::Odometry &msg);	// 只负责发布 local_odom（封装统一 frame、stamp、pub 调用）。
  void PublishGlobalOdom(const nav_msgs::Odometry &msg);	// 只负责发布 global_odom（同上，封装发布细节）。
  void PublishOdomState();	// 发布状态话题（如 local_odom_valid/global_odom_valid、当前 source/loop 等
  
	// ------回环相关-----
	void TryUpdateMapToOdom();	// 尝试根据当前 local/global 数据估计或更新 T_map_odom（数据不满足条件时不更新）。
  void BroadcastMapToOdomTf();	// 把当前 T_map_odom 通过 TF 广播出去（map -> odom）
	
	// -----辅助函数------
  bool IsTimedOut(const ros::Time &last_stamp, double timeout_s) const;	// 判断链路是否超时
  const SourceConfig *FindSelectedSource() const;	// 从sources_ 配置表里找到当前 selected_source_id_ 对应的源配置，并返回一个SourceConfig配置的指针

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;	// 私有句柄，用于处理launch文件中的参数

  ros::Subscriber local_sub_;		// local系话题 订阅者
  ros::Subscriber global_sub_;	// global系话题 订阅者
  // ros::Subscriber loop_sub_; // TODO: 开启回环输入时启用

  ros::Publisher local_odom_pub_;	// 局部定位话题 local_odom发布者
  ros::Publisher global_odom_pub_;	// 全局定位话题 global_odom 发布者
  ros::Publisher odom_state_pub_;		// 定位状态话题 odom_state发布者

  ros::Timer health_timer_;			// 健康检查定时器：周期检查 local/global 是否超时，并发布 odom_state
  tf2_ros::TransformBroadcaster tf_broadcaster_;		// TF 广播器：发布 map -> odom 坐标变换

  // 配置与模式
  std::unordered_map<int, SourceConfig> sources_;	// 使用哈希表，将localization_sources.yaml中所有可用的定位源配置，缓存成一个source_id->配置的快速查找表
  int selected_source_id_{-1};	// launch文件中所指定的定位源id
  LoopMode selected_loop_mode_{LoopMode::DISABLE};	// 回环检测的模式

  // 运行时状态
  mutable std::mutex mutex_;	

  nav_msgs::Odometry last_local_meas_;	// 最新的local系观测值
  nav_msgs::Odometry last_global_meas_;	// 最新的global系观测值
  bool has_local_meas_{false};	// local系回调函数是否接受到数据
  bool has_global_meas_{false};	// global系回调函数是否接受到数据

  ros::Time last_local_rx_time_;	// 最新的local系数据接收时间
  ros::Time last_global_rx_time_;	// 最新的global系数据接收时间
  bool local_odom_valid_{false};	// local系数据是否有效
  bool global_odom_valid_{false};	// global系数据是否有效

  // aruco/bootstrap 相关
  // bool bootstrap_initialized_{false}; // TODO: 开启 aruco_loop/坐标初始化逻辑时启用
  geometry_msgs::TransformStamped map_to_odom_tf_;	// 发布local系到global系的tf变换
};

} // namespace localization_fusion
