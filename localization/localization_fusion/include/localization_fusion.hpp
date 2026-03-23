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

#pragma once

#include <ros/node_handle.h>

#include <mutex>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include "localization_fusion_types.hpp"

// 由于对函数添加了注释，导致clangd格式化后冗长，因此禁止格式化
// clang-format off

class LocalizationFusion {
public:
  explicit LocalizationFusion(ros::NodeHandle &nh);
	~LocalizationFusion() = default;

  // 启动入口：读配置、校验模式、创建 pub/sub/timer
  bool Init();

  // 主循环（单线程 ros::spin）
  void Spin();

private:

  // ---- 初始化阶段 ----
	bool load_param(); // 加载参数，校验参数

	void set_publisher(); // 设置ros话题发布者,发布~/sunray/localization/global_odom local_odom odom_status TF
  void set_subscriber(); // 设置ros话题订阅者，订阅odometry_topic,relocalization_topic
  void set_timer(); // 设置定时器，目前为周期性的检查odometry话题回调是否收到数据，是否超时，更新odometry_valid以及发布odom_state

  // ---- 回调函数 ----
	void odometry_callback(const nav_msgs::OdometryConstPtr &msg);	// 里程计输入
  void relocalization_callback(const nav_msgs::OdometryConstPtr &msg); // 重定位输入：local 在 global 下的位姿（global->local）
  void healthtimer_callback(const ros::TimerEvent &e); // 周期检查 local/global 是否收到新数据,判断是否超时,更新 local_odom_valid/global_odom_valid,发布 odom_state

  // ---- 发布localization_fusion状态 ----
  void publish_odom_state(); // 发布 OdomStatus 状态话题

  // ------TF 广播相关-----
  void broadcast_global_to_local_tf(const ros::Time &stamp);	// 发布 sunray_global -> sunray_local
  void broadcast_local_to_base_tf(const nav_msgs::Odometry &local_odom);	// 发布 sunray_local -> base_link

  // -----辅助函数------
  bool is_timed_out(const ros::Time &last_stamp,
                  double timeout_s) const; // 判断链路是否超时



private:
  // ---------------- ROS 相关参数 ---------------- 
  ros::NodeHandle nh_;
	// ROS 话题订阅者
  ros::Subscriber odometry_sub_;       // 里程计话题 订阅者
  ros::Subscriber relocalization_sub_; // 重定位话题 订阅者 
	// ROS话题发布者
  ros::Publisher local_odom_pub_;  // 局部定位话题 local_odom发布者
  ros::Publisher global_odom_pub_; // 全局定位话题 global_odom 发布者
  ros::Publisher odom_state_pub_;  // 定位状态话题 odom_state发布者
	// ROS定时器
  ros::Timer health_timer_; // 健康检查定时器：周期检查 local/global
                            // 是否超时，并发布 odom_state
	// TF 广播器
  tf2_ros::TransformBroadcaster tf_broadcaster_; 

  // ---------------- Config 相关参数 ---------------- 
  std::string uav_ns_; // 从全局参数中读取 uav_id+ uav_name 组成uav_namespace
  std::string config_yamlfile_path_; // yaml 参数文件的路径
	int selected_source_id_{-1}; // launch文件中所指定的定位源编号
  SourceConfig selected_source_{}; 	// 结合launch文件中的source_id,从config文件中构建读取参数并缓存 
	bool has_selected_source_{false}; // 状态标识符，表示是否读取到了配置参数
	double health_rate_hz_{10};
	
	// 输出topic约定
	std::string global_odometry_topic_{"${uav_ns}/sunray/localization/global_odom"};
	std::string local_odometry_topic_{"${uav_ns}/sunray/localization/global_odom"};
	std::string odom_status_topic_{"${uav_ns}/sunray/localization/odom_status"};
	
  // 输出 frame 约定：sunray_global -> sunray_local -> base_link
  std::string global_frame_id_{"sunray_global"};
  std::string local_frame_id_{"sunray_local"};
  std::string base_frame_id_{"base_link"};

  // 线程锁
  mutable std::mutex mutex_;

  nav_msgs::Odometry last_odometry_data_;  // 最新的odometry数据
  nav_msgs::Odometry last_relocalization_data_; // 最新的重定位数据
	
	bool has_odometry_data_{false};          // odometry回调函数是否接受到数据
	bool has_relocalization_data_{false};		// relocalization回调函数是否接受到数据

  ros::Time last_odometry_rx_time_;  // 最新的odometry数据接收时间
  ros::Time last_relocalization_rx_time_; // 最新的relocalization数据接收时间

	bool odometry_data_valid_{false};  // local系数据是否有效
	bool relocalization_data_valid{false};	// relocalization数据是否有效

  geometry_msgs::TransformStamped global_to_local_tf_; // sunray_global -> sunray_local
  geometry_msgs::TransformStamped local_to_base_tf_; // sunray_local -> base_link
};

// clang-format on