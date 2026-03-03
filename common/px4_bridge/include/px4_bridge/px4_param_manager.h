/**
 * @file px4_param_manager.h
 * @brief PX4 参数管理器（含 EKF2 参数设置与模块重启）
 *
 * @details
 * 这个类是“参数写入 + 必要的模块重启”的一体化封装，核心目标是：
 * 1. 通过 MAVROS `/param/set` 统一写入 PX4 参数；
 * 2. 对“写入后需重启模块才生效”的参数（如 EKF2 一部分参数）做自动化处理；
 * 3. 把 NSH-MAVLink 的细节隐藏起来，降低上层调用复杂度。
 *
 * 架构上分成两条通道：
 * A) 参数通道（service）：
 *    - 使用 `/uavX/mavros/param/set` 写参数。
 * B) 模块控制通道（topic）：
 *    - 使用 `/uavX/mavlink/to` 与 `/uavX/mavlink/from` 发送/接收
 *      MAVLink SERIAL_CONTROL，以执行 NSH 命令（ekf2 stop/start/status）。
 *
 * 注意事项：
 * - 该类依赖参数 `uav_id` / `uav_name`，用于拼接命名空间。
 * - EKF2 参数接口写入后会调用 `boot_ekf2()` 以确保生效。
 * - `boot_ekf2()` 会等待 shell prompt 与 estimator 更新，属于阻塞调用。
 */

#pragma once

#include "mavros_msgs/EstimatorStatus.h"
#include "mavros_msgs/Mavlink.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"

#include <condition_variable>
#include <mutex>
#include <regex>
#include <string>

#include "px4_bridge/px4_param_types.h"

class PX4_ParamManager {
public:
  /**
   * @brief 构造函数：读取 UAV 标识并创建必要通信对象。
   *
   * 初始化完成后，类会持有：
   * - param_set_client_：参数设置 service client；
   * - mavlink_to_pub_：发 NSH 命令；
   * - mavlink_from_sub_：收 NSH 回显；
   * - estimator_sub_：监听 estimator_status 变化用于重启后健康确认。
   *
   * @param nh ROS NodeHandle（需要能访问参数服务器）
   */
  explicit PX4_ParamManager(ros::NodeHandle nh);

  ~PX4_ParamManager() = default;

  /**
   * @brief 设置 EKF2_HGT_REF 参数。
   * @param hgt_ref 目标高度参考源
   * @return true 设置成功，false 设置失败
   */
  bool set_ekf2_hgt_ref(const px4_param_types::EKF2_HGT_REF &hgt_ref);

  /**
   * @brief 设置 EKF2_EV_CTRL 参数。
   * @param ev_ctrl 外部视觉融合控制位掩码
   * @return true 设置成功，false 设置失败
   */
  bool set_ekf2_ev_ctrl(const px4_param_types::EKF2_EV_CTRL &ev_ctrl);

  /**
   * @brief 设置 EKF2_EV_DELAY 参数（毫秒）。
   * @param ev_delay 外部视觉延迟参数
   * @return true 设置成功，false 设置失败
   */
  bool set_ekf2_ev_delay(const px4_param_types::EKF2_EV_DELAY &ev_delay);

  /**
   * @brief 设置 EKF2_MAG_TYPE 参数。
   * @param mag_type 磁力计融合模式
   * @return true 设置成功，false 设置失败
   */
  bool set_ekf2_mag_type(const px4_param_types::EKF2_MAG_TYPE &mag_type);

  /**
   * @brief 设置 EKF2_MAG_CHECK 参数。
   * @param mag_check 磁力计检查项位掩码
   * @return true 设置成功，false 设置失败
   */
  bool set_ekf2_mag_check(const px4_param_types::EKF2_MAG_CHECK &mag_check);

  /**
   * @brief 设置 EKF2_GPS_CTRL 参数。
   * @param gps_ctrl GPS 融合控制位掩码
   * @return true 设置成功，false 设置失败
   */
  bool set_ekf2_gps_ctrl(const px4_param_types::EKF2_GPS_CTRL &gps_ctrl);

  /**
   * @brief 设置 EKF2_GPS_CHECK 参数。
   * @param gps_check GPS 质量检查项位掩码
   * @return true 设置成功，false 设置失败
   */
  bool set_ekf2_gps_check(const px4_param_types::EKF2_GPS_CHECK &gps_check);

  /**
   * @brief 设置 EKF2_GPS_DELAY 参数（毫秒）。
   * @param gps_delay GPS 融合延迟参数
   * @return true 设置成功，false 设置失败
   */
  bool set_ekf2_gps_delay(const px4_param_types::EKF2_GPS_DELAY &gps_delay);

  /**
   * @brief 批量设置 PX4 原生角速度环 PID 参数。
   * @param rate_pid 角速度环参数（ROLL/PITCH/YAW）
   * @return true 设置成功，false 设置失败
   */
  bool set_px4_rate_pid(const px4_param_types::PX4_RATE_PID &rate_pid);

  /**
   * @brief 批量设置 PX4 速度环参数。
   * @param velocity_pid 速度环参数（XY 合并，Z 独立）
   * @return true 设置成功，false 设置失败
   */
  bool set_px4_velocity_pid(
      const px4_param_types::PX4_VELOCITY_PID &velocity_pid);

  /**
   * @brief 批量设置 PX4 位置环参数。
   * @param position_p 位置环参数（XY 合并，Z 独立）
   * @return true 设置成功，false 设置失败
  */
  bool set_px4_position_p(const px4_param_types::PX4_POSITION_P &position_p);

  /**
   * @brief 重启 EKF2 并验证重启成功。
   * 请注意，当设置了与EKF2有关参数时，根据参数类型确定是否需要重启EKF2模块
   *
   * 验证链路：
   * - status 前检查（应在运行）
   * - stop 后检查（应停止）
   * - start 后检查（应恢复运行）
   * - estimator_status 更新检查（应有新数据）
   */
  bool boot_ekf2(void);

private:
  /**
   * @brief UAV 编号（来自参数服务器）。
   */
  int uav_id_{0};

  /**
   * @brief UAV 名称前缀（来自参数服务器）。
   */
  std::string uav_name_{"null"};

  /**
   * @brief MAVROS 命名空间（例如 `/uav1/mavros`）。
   */
  std::string mavros_ns_;

  /**
   * @brief 构造阶段初始化状态。
   */
  bool initialized_{false};

  /**
   * @brief 参数设置服务客户端（`/uavX/mavros/param/set`）。
   */
  ros::ServiceClient param_set_client_;

  /**
   * @brief MAVLink 下行发布器（`/uavX/mavlink/to`）。
   */
  ros::Publisher mavlink_to_pub_;

  /**
   * @brief MAVLink 上行订阅器（`/uavX/mavlink/from`）。
   */
  ros::Subscriber mavlink_from_sub_;

  /**
   * @brief EKF2 状态订阅器（`/uavX/mavros/estimator_status`）。
   */
  ros::Subscriber estimator_sub_;

  /**
   * @brief 回调线程与主流程同步互斥量。
   */
  mutable std::mutex mtx_;

  /**
   * @brief 回调线程与主流程同步条件变量。
   */
  std::condition_variable cv_;

  /**
   * @brief NSH 输出缓存。
   */
  std::string nsh_buf_;

  /**
   * @brief estimator_status 回调计数。
   */
  int estimator_updates_{0};

  /**
   * @brief ANSI 控制序列清洗正则。
   */
  std::regex ansi_re_;

  /**
   * @brief 非可见控制字符清洗正则。
   */
  std::regex ctrl_re_;

  /**
   * @brief 前置检查：对象是否初始化且服务可用。
   * @return true 可继续执行，false 应中止
   */
  bool ensure_client_ready(void);

  /**
   * @brief 写入整型参数（MAVROS ParamSet 封装）。
   * @param name 参数名
   * @param value 整型值
   * @return true 写入成功，false 写入失败
   */
  bool setParamInt(const std::string &name, int value);

  /**
   * @brief 写入浮点参数（MAVROS ParamSet 封装）。
   * @param name 参数名
   * @param value 浮点值
   * @return true 写入成功，false 写入失败
   */
  bool setParamFloat(const std::string &name, float value);

  /**
   * @brief MAVLink 接收回调，解析 NSH 输出文本。
   * @param ros_msg MAVLink ROS 消息
   */
  void rx_cb(const mavros_msgs::Mavlink::ConstPtr &ros_msg);

  /**
   * @brief 估计器状态回调，仅维护更新计数。
   * @param msg estimator_status 消息
   */
  void estimator_cb(const mavros_msgs::EstimatorStatus::ConstPtr &msg);

  /**
   * @brief 发送单条 NSH 命令。
   * @param cmd 命令文本
   */
  void send_nsh(const std::string &cmd);

  /**
   * @brief 发送 NSH 命令并等待 shell prompt。
   * @param cmd 命令文本
   * @param timeout_sec 超时秒数
   * @return 命令输出文本
   */
  std::string run_nsh_cmd(const std::string &cmd, double timeout_sec);

  /**
   * @brief 清洗文本中的控制符与 ANSI 序列。
   * @param text 原始文本
   * @return 清洗后的文本
   */
  std::string sanitize_text(const std::string &text) const;

  /**
   * @brief 获取 estimator_status 回调计数。
   * @return 当前计数
   */
  int estimator_count() const;

  /**
   * @brief 等待 estimator_status 计数增长。
   * @param prev_count 基准计数
   * @param timeout_sec 超时秒数
   * @return true 等待到更新，false 超时
   */
  bool wait_estimator_update(int prev_count, double timeout_sec);
};
