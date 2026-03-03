/**
 * @file px4_data_reader.h
 * @brief PX4 数据读取器接口定义。
 *
 * 本文件定义 `PX4_DataReader` 对外接口，用于读取：
 * - 飞行状态与估计状态；
 * - 位姿、速度、光流等实时数据；
 * - PX4 参数服务中的关键参数。
 */

#pragma once

#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/EstimatorStatus.h"
#include "mavros_msgs/ExtendedState.h"
#include "mavros_msgs/ParamGet.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SysStatus.h"
#include "mavros_msgs/OpticalFlowRad.h"
#include "nav_msgs/Odometry.h"
#include "px4_bridge/px4_data_types.h"
#include "px4_bridge/px4_param_types.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Imu.h"

#include <functional>
#include <mutex>
#include <string>

/**
 * @brief ReaderOptions 订阅开关集合。
 *
 * 默认全部关闭，由用户按需开启。
 */
struct ReaderOptions {
  /// 是否订阅系统状态（连接、解锁、电池等）。
  bool read_system_state;
  /// 是否订阅 EKF2 状态。
  bool read_ekf2_state;
  /// 是否订阅光流数据。
  bool read_flow_state;
  /// 是否订阅 local 位姿。
  bool read_local_pose;
  /// 是否订阅 local 速度。
  bool read_local_velocity;
  /// 是否订阅 body 位姿。
  bool read_body_pose;
  /// 是否订阅 body 速度。
  bool read_body_velocity;

  /**
   * @brief 默认构造函数（全部关闭）。
   */
  ReaderOptions() { disable_all(); };

  /**
   * @brief 打开所有订阅开关。
   */
  void enable_all();

  /**
   * @brief 关闭所有订阅开关。
   */
  void disable_all();
};

/**
 * @brief PX4 数据读取器。
 *
 * 通过订阅 MAVROS 话题与调用参数服务提供统一读取接口。
 */
class PX4_DataReader {
public:
  /**
   * @brief 默认构造（启用全量订阅）。
   * @param nh ROS 节点句柄
   */
  PX4_DataReader(ros::NodeHandle &nh);

  /**
   * @brief 按开关选择性订阅构造。
   * @param nh ROS 节点句柄
   * @param options 订阅开关集合
   */
  PX4_DataReader(ros::NodeHandle &nh, ReaderOptions options);

  /**
   * @brief 根据开关初始化订阅者。
   * @param nh ROS 节点句柄
   * @param options 订阅开关集合
   */
  void init_subscribers(ros::NodeHandle &nh, ReaderOptions options);

  /**
   * @brief 析构函数。
   */
  ~PX4_DataReader();

  /**
   * @brief 获取系统状态缓存。
   * @return 当前系统状态
   */
  px4_data_types::SystemState get_system_state(void);

  /**
   * @brief 获取 EKF2 状态缓存。
   * @return 当前 EKF2 状态
   */
  px4_data_types::Ekf2State get_ekf2_state(void);

  /**
   * @brief 获取最近光流状态缓存。
   * @return 光流状态
   */
  px4_data_types::OpticalFlow get_opflow_state(void);

  /**
   * @brief 获取 local 位姿缓存。
   * @return local 位姿
   */
  px4_data_types::Pose get_local_pose(void);

  /**
   * @brief 获取 local 速度缓存。
   * @return local 速度
   */
  px4_data_types::Velocity get_local_velocity(void);

  /**
   * @brief 获取 body 位姿缓存。
   * @return body 位姿
   */
  px4_data_types::Pose get_body_pose(void);

  /**
   * @brief 获取 body 速度缓存。
   * @return body 速度
   */
  px4_data_types::Velocity get_body_velocity(void);

  /**
   * @brief 实时读取 EKF2_HGT_REF 参数。
   * @return EKF2_HGT_REF 参数结构
   */
  px4_param_types::EKF2_HGT_REF fetch_ekf2_hgt_ref(void);

  /**
   * @brief 实时读取 EKF2_EV_CTRL 参数。
   * @return EKF2_EV_CTRL 参数结构
   */
  px4_param_types::EKF2_EV_CTRL fetch_ekf2_ev_ctrl(void);

  /**
   * @brief 实时读取 EKF2_EV_DELAY 参数。
   * @return EKF2_EV_DELAY 参数结构
   */
  px4_param_types::EKF2_EV_DELAY fetch_ekf2_ev_delay(void);

  /**
   * @brief 实时读取 EKF2_MAG_TYPE 参数。
   * @return EKF2_MAG_TYPE 参数结构
   */
  px4_param_types::EKF2_MAG_TYPE fetch_ekf2_mag_type(void);

  /**
   * @brief 实时读取 EKF2_MAG_CHECK 参数。
   * @return EKF2_MAG_CHECK 参数结构
   */
  px4_param_types::EKF2_MAG_CHECK fetch_ekf2_mag_check(void);

  /**
   * @brief 实时读取 EKF2_GPS_CTRL 参数。
   * @return EKF2_GPS_CTRL 参数结构
   */
  px4_param_types::EKF2_GPS_CTRL fetch_ekf2_gps_ctrl(void);

  /**
   * @brief 实时读取 EKF2_GPS_CHECK 参数。
   * @return EKF2_GPS_CHECK 参数结构
   */
  px4_param_types::EKF2_GPS_CHECK fetch_ekf2_gps_check(void);

  /**
   * @brief 实时读取 EKF2_GPS_DELAY 参数。
   * @return EKF2_GPS_DELAY 参数结构
   */
  px4_param_types::EKF2_GPS_DELAY fetch_ekf2_gps_delay(void);

  /**
   * @brief 实时读取角速度环 PID 参数。
   * @return PX4_RATE_PID 参数结构
   */
  px4_param_types::PX4_RATE_PID fetch_px4_rate_pid(void);

  /**
   * @brief 实时读取速度环参数。
   * @return PX4_VELOCITY_PID 参数结构
   */
  px4_param_types::PX4_VELOCITY_PID fetch_px4_velocity_pid(void);

  /**
   * @brief 实时读取位置环参数。
   * @return PX4_POSITION_P 参数结构
   */
  px4_param_types::PX4_POSITION_P fetch_px4_position_p(void);

  /**
   * @brief 将 MAVROS 模式字符串转换为 FlightMode。
   * @param mode 模式字符串
   * @return FlightMode 枚举值
   */
  static px4_data_types::FlightMode flight_mode_from_string(const std::string &mode);

private:
  /**
   * @brief 订阅表项，用于按开关动态创建订阅者。
   */
  struct SubscribeEntry {
    bool ReaderOptions::* flag;
    ros::Subscriber PX4_DataReader::* handle;
    std::function<ros::Subscriber()> make;
  };

  /**
   * @brief 重置全部缓存为默认值。
   */
  void reset_state_defaults();

  /**
   * @brief 初始化参数服务客户端。
   */
  void init_service_clients();

  /**
   * @brief 读取整型参数。
   * @param param_name 参数名
   * @param out_value 输出值
   * @return true 成功，false 失败
   */
  bool fetch_param_int(const std::string &param_name, int &out_value);

  /**
   * @brief 读取浮点参数。
   * @param param_name 参数名
   * @param out_value 输出值
   * @return true 成功，false 失败
   */
  bool fetch_param_float(const std::string &param_name, float &out_value);

  /**
   * @brief 话题回调：基础飞行状态。
   * @param msg 状态消息
   */
  void state_callback(const mavros_msgs::State::ConstPtr &msg);

  /**
   * @brief 话题回调：扩展状态。
   * @param msg 扩展状态消息
   */
  void extended_state_callback(const mavros_msgs::ExtendedState::ConstPtr &msg);

  /**
   * @brief 话题回调：系统状态。
   * @param msg 系统状态消息
   */
  void system_status_callback(const mavros_msgs::SysStatus::ConstPtr &msg);

  /**
   * @brief 话题回调：EKF2 状态。
   * @param msg EKF2 状态消息
   */
  void ekf2_status_callback(const mavros_msgs::EstimatorStatus::ConstPtr &msg);

  /**
   * @brief 话题回调：光流。
   * @param msg 光流消息
   */
  void optical_flow_callback(const mavros_msgs::OpticalFlowRad::ConstPtr &msg);

  /**
   * @brief 话题回调：local 里程计。
   * @param msg 里程计消息
   */
  void local_odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);

  /**
   * @brief 话题回调：local 速度。
   * @param msg 速度消息
   */
  void local_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);

  /**
   * @brief 话题回调：body 姿态。
   * @param msg IMU 消息
   */
  void body_attitude_callback(const sensor_msgs::Imu::ConstPtr &msg);

  /**
   * @brief 话题回调：body 速度。
   * @param msg 速度消息
   */
  void body_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);

  /// ROS 节点句柄。
  ros::NodeHandle nh_;
  /// 参数读取服务客户端。
  ros::ServiceClient param_get_client_;

  /// 状态订阅者。
  ros::Subscriber state_sub_;
  /// 扩展状态订阅者。
  ros::Subscriber exstate_sub_;
  /// 系统状态订阅者。
  ros::Subscriber sys_sub_;
  /// EKF2 状态订阅者。
  ros::Subscriber ekf2status_sub_;
  /// 光流订阅者。
  ros::Subscriber opflow_sub_;
  /// local 里程计订阅者。
  ros::Subscriber local_odom_sub_;
  /// local 速度订阅者。
  ros::Subscriber local_vel_sub_;
  /// body 姿态订阅者。
  ros::Subscriber body_att_sub_;
  /// body 速度订阅者。
  ros::Subscriber body_vel_sub_;

  /// 系统状态缓存互斥锁。
  mutable std::mutex system_state_mutex_;
  /// EKF2 状态缓存互斥锁。
  mutable std::mutex ekf2_state_mutex_;
  /// 光流缓存互斥锁。
  mutable std::mutex opflow_mutex_;
  /// local 位姿缓存互斥锁。
  mutable std::mutex local_pose_mutex_;
  /// local 速度缓存互斥锁。
  mutable std::mutex local_velocity_mutex_;
  /// body 位姿缓存互斥锁。
  mutable std::mutex body_pose_mutex_;
  /// body 速度缓存互斥锁。
  mutable std::mutex body_velocity_mutex_;

  /// UAV ID。
  int uav_id_;
  /// UAV 命名空间（例如 `/uav1`）。
  std::string uav_namespace_;

  /// 系统状态缓存。
  px4_data_types::SystemState system_state_cache_;
  /// EKF2 状态缓存。
  px4_data_types::Ekf2State ekf2_state_cache_;
  /// 最近一帧光流缓存。
  px4_data_types::OpticalFlow latest_opflow_cache_;
  /// local 位姿缓存。
  px4_data_types::Pose local_pose_cache_;
  /// local 速度缓存。
  px4_data_types::Velocity local_velocity_cache_;
  /// local 里程计缓存。
  px4_data_types::Odometry local_odometry_cache_;
  /// body 位姿缓存。
  px4_data_types::Pose body_pose_cache_;
  /// body 速度缓存。
  px4_data_types::Velocity body_velocity_cache_;

  /// EKF2_HGT_REF 缓存。
  px4_param_types::EKF2_HGT_REF ekf2_hgt_ref_cache_;
  /// EKF2_EV_CTRL 缓存。
  px4_param_types::EKF2_EV_CTRL ekf2_ev_ctrl_cache_;
  /// EKF2_EV_DELAY 缓存。
  px4_param_types::EKF2_EV_DELAY ekf2_ev_delay_cache_;
  /// EKF2_MAG_TYPE 缓存。
  px4_param_types::EKF2_MAG_TYPE ekf2_mag_type_cache_;
  /// EKF2_MAG_CHECK 缓存。
  px4_param_types::EKF2_MAG_CHECK ekf2_mag_check_cache_;
  /// EKF2_GPS_CTRL 缓存。
  px4_param_types::EKF2_GPS_CTRL ekf2_gps_ctrl_cache_;
  /// EKF2_GPS_CHECK 缓存。
  px4_param_types::EKF2_GPS_CHECK ekf2_gps_check_cache_;
  /// EKF2_GPS_DELAY 缓存。
  px4_param_types::EKF2_GPS_DELAY ekf2_gps_delay_cache_;
  /// 角速度环参数缓存。
  px4_param_types::PX4_RATE_PID px4_rate_pid_cache_;
  /// 速度环参数缓存。
  px4_param_types::PX4_VELOCITY_PID px4_velocity_pid_cache_;
  /// 位置环参数缓存。
  px4_param_types::PX4_POSITION_P px4_position_p_cache_;
};
