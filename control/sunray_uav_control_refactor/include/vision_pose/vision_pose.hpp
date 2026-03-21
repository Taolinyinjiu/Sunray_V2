/**
 * @file vision_pose.hpp
 * @author your name (you@domain.com)
 * @brief vision_pose 节点：读取定位状态与里程计，按配置同步 PX4 融合参数并向 MAVROS 发布外部视觉位姿/里程计。
 * @version 0.1
 * @date 2026-03-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/OdomStatus.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

class VisionPose final {
public:
  // 构造函数：
  // 1) 读取运行时参数（话题名/发布频率/校验策略等）
  // 2) 读取 config（~sources_list）
  // 3) 订阅 localization 输出的 local_odom 与 OdomStatus
  // 4) 构造发布定时器，按 fuse_type 向 MAVROS 发布
  explicit VisionPose(ros::NodeHandle &nh);
  ~VisionPose() = default;

  // 初始化入口：读配置、创建 pub/sub/client/timer，并在启动前完成一次参数校验。
  bool Init();

  // 主循环（单线程 ros::spin）
  void Spin();

private:
  enum class ParamType { kInt = 0, kFloat = 1 };

  struct ParamSpec {
    std::string name{};
    ParamType type{ParamType::kInt};
    int int_value{0};
    double float_value{0.0};
  };

  struct SourceConfig {
    std::string source_name{};
    int source_id{-1};
    int fuse_type{0}; // 0: vision_pose, 1: odometry
    std::vector<ParamSpec> params{};
  };

  // ---- 初始化阶段 ----
  bool LoadRuntimeParams();
  bool LoadSourceConfigs();
  bool SetupMavrosClientsAndPublishers();
  void SetupSubscribers();
  void SetupPublishTimer();
  bool ApplySourceConfig(int source_id);

  // ---- 参数检查/设置 ----
  bool check_param(const SourceConfig &cfg);
  bool read_param_int(const std::string &name, int *out_value);
  bool read_param_float(const std::string &name, double *out_value);
  bool set_param_int(const std::string &name, int value);
  bool set_param_float(const std::string &name, double value);

  // ---- 同步循环 ----
  bool WaitForReady(const ros::Duration &timeout);
  void Disable_fuse();
  void PublishOnce(const ros::Time &now);

  // ---- 回调函数 ----
  void LocalOdomCallback(const nav_msgs::OdometryConstPtr &msg);
  void OdomStatusCallback(const sunray_msgs::OdomStatusConstPtr &msg);
  void OnPublishTimer(const ros::TimerEvent &e);

  const SourceConfig *FindSource(int source_id) const;

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber local_odom_sub_;
  ros::Subscriber odom_status_sub_;

  ros::Publisher vision_pose_pub_;
  ros::Publisher odom_pub_;
  ros::Timer publish_timer_;

  ros::ServiceClient param_get_client_;
  ros::ServiceClient param_set_client_;

  // 配置与运行时参数
  std::unordered_map<int, SourceConfig> sources_;
  int active_source_id_{-1};
  int active_fuse_type_{0};
  const SourceConfig *active_cfg_{nullptr};

  std::string local_odom_topic_{"/sunray/localization/local_odom"};
  std::string odom_status_topic_{"/sunray/localization/odom_status"};
  std::string mavros_ns_{"/mavros"};

  double publish_rate_hz_{30.0};
  double odom_timeout_s_{0.5};
  double init_wait_timeout_s_{5.0};
  bool auto_set_px4_params_{false};
  bool strict_param_check_{true};
  bool allow_source_switch_{true};

  // 运行时状态
  mutable std::mutex mutex_;
  bool fuse_enabled_{true};

  bool has_local_odom_{false};
  nav_msgs::Odometry latest_local_odom_{};
  ros::Time latest_local_odom_rx_time_{};

  bool has_odom_status_{false};
  sunray_msgs::OdomStatus latest_odom_status_{};
  ros::Time latest_odom_status_rx_time_{};
  int pending_source_id_{-1};
};
