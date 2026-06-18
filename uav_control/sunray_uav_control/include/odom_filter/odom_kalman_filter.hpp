/**
 * @file odom_kalman_filter.hpp
 * @brief Sunray 控制输入侧的轻量里程计卡尔曼滤波器。
 *
 * 设计边界：
 * 1. 本类只处理 control_common::UAVStateEstimate，不直接依赖 ROS topic 或 FSM。
 * 2. 本类面向“本地控制器输入滤波”，不负责向 PX4 EKF 转发外部定位。
 * 3. 只对位置和速度使用 6 维线性 KF；姿态和角速度透传。
 */

#pragma once

#include "control_data_types/uav_state_estimate.hpp"
#include <Eigen/Dense>

namespace control_common {

// 里程计卡尔曼滤波器参数结构体。
// 只包含通用线性 KF 所需的过程噪声和测量噪声；参数合法性由配置加载层检查。
struct OdomKalmanFilterParam_t {
    // 连续白噪声加速度模型的过程噪声强度，单位近似 m^2/s^3。
    // 越大越跟随机动，越小越平滑但滞后更明显。
    // 经验范围：
    //   0.05-0.2  : 静稳平台或低噪声仿真，输出更平滑但响应更慢；
    //   0.3-1.0   : 常见 VIO / LiDAR SLAM 控制输入滤波；
    //   1.0-3.0   : 高机动或上游延迟较小时使用，噪声会更容易进入输出。
    double process_noise_acc{0.5};

    // 位置/速度测量标准差。滤波器内部会平方后放入测量噪声矩阵 R。
    double meas_noise_pos{0.05};
    double meas_noise_vel{0.10};
};

// 6 维 [position, velocity] 卡尔曼滤波器。
//
// 调用约定：
// - 上游负责数据质量控制，本滤波器默认输入位置和速度可信，只处理小噪声。
// - update() 内部由 raw_odom.timestamp 计算 dt。
// - 若上游消息时间戳不可信，调用方应把 raw_odom.timestamp 填成接收时刻。
// - 本类不加锁；跨线程访问时由外层 FSM 或调用者保护。
class OdomKalmanFilter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 构造函数
    OdomKalmanFilter() = default;

    // 载入参数并重置滤波器状态。
    void init(const OdomKalmanFilterParam_t& params);
    // 清空内部状态。下一次有效 update() 会按首帧重新初始化。
    void reset();
    // 当前滤波器是否已经用有效测量初始化。
    bool is_initialized() const;
    
    // 使用 raw_odom.timestamp 计算 dt，执行预测和固定 6 维位置+速度观测更新。
    // 返回值保留原始 timestamp/orientation/bodyrate，只替换滤波后的 position/velocity。
    UAVStateEstimate update(const UAVStateEstimate& raw_odom);
    
    // 返回内部 6 维状态 [px, py, pz, vx, vy, vz]^T。
    const Eigen::Matrix<double, 6, 1>& get_state() const;
    // 返回状态协方差 P。
    const Eigen::Matrix<double, 6, 6>& get_covariance() const;

  private:
    // 使用当前测量初始化。位置或速度无效时失败。
    bool initialize(const UAVStateEstimate& raw_odom);
    // 匀速模型预测。
    void predict(double dt);
    // 固定 6 维位置+速度观测更新。
    bool update_position_velocity(const Eigen::Vector3d& pos_meas,
                                  const Eigen::Vector3d& vel_meas);
    // NaN/Inf 保护与协方差对角线最小正值保护。
    void enforce_numeric_safety(const UAVStateEstimate& raw_odom);
    // 组装输出：滤波 position/velocity，orientation/bodyrate 透传。
    UAVStateEstimate compose_output(const UAVStateEstimate& raw_odom) const;

  private:
    OdomKalmanFilterParam_t params_{};

    // x_ = [px, py, pz, vx, vy, vz]^T。
    Eigen::Matrix<double, 6, 1> x_{Eigen::Matrix<double, 6, 1>::Zero()};
    // 状态协方差。
    Eigen::Matrix<double, 6, 6> P_{Eigen::Matrix<double, 6, 6>::Identity()};
    ros::Time last_timestamp_{ros::Time(0)};
    bool initialized_{false};
    bool has_last_timestamp_{false};
};

}  // namespace control_common
