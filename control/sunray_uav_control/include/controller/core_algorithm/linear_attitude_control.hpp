/**
 * 本文件作为se3控制器的核心数据模型部分，与ros无关，输出量为CBTR(归一化推力+Bodyrate)
 */
#pragma once

#include "control_data_types/controller_desired_types.hpp"  // 选取本文件中的trajpoint作为期望输入
#include "control_data_types/uav_state_estimate.hpp"  // 选取本文件中的uav_estimate作为里程计输入
#include "mavros_helper/mavros_helper.hpp"
#include <queue>

// 需要定义一些参数,这些参数在sunray_control_config.yaml中被定义，然后px4_se3_controller.cpp进行读取，再传递到这里
struct gain_t {
    double Kx;
    double Ky;
    double Kz;
};

struct Linear_AttitudeControl_Param_t {
    int output_type;
    double gravity;
    double accurate_thrust_model;
    double hover_percentage;
    gain_t pos_gain;
    gain_t vel_gain;
};

// 输出数据类型
struct Linear_AttitudeControl_Output_t {
    Eigen::Quaterniond orientation;  // 姿态
    Eigen::Vector3d bodyrates;       // 机体角速率
    double thrust;                   // 归一化推力
};

class Linear_AttitudeControl {
  public:
    Linear_AttitudeControl() = default;
    ~Linear_AttitudeControl() = default;
    // 在这里加载参数而不是在构造函数中加载，因为构造函数中加载在后续实现的时候会麻烦一点
    void load_param(Linear_AttitudeControl_Param_t& param);
    Linear_AttitudeControl_Output_t
    calculateControl(const controller_data_types::TargetTrajectoryPoint_t des_state,
                     const control_common::UAVStateEstimate& current_odom,
                     const control_common::Mavros_IMU& imu);
    // 推力模型估计
    bool estimateThrustModel(const Eigen::Vector3d& est_v);
    // 重置推力映射
    void resetThrustMapping(void);
    // 内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    Linear_AttitudeControl_Param_t param_;
    std::queue<std::pair<ros::Time, double>> timed_thrust_;
    static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;
    // 基于加速度的推力估计参数
    const double rho2_ = 0.998;  // 增大该数值，会降低动态性能
    double thr2acc_;             // thrust_to_accelection 推力到加速度的映射参数
    double P_;
    double computeDesiredCollectiveThrustSignal(
        const Eigen::Vector3d& des_acc);              // 基于加速度计算归一化推力
    double fromQuaternion2yaw(Eigen::Quaterniond q);  // 从四元数提取yaw角
};
