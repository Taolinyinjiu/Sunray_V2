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

struct ControlParam {
    Eigen::Vector3d Kp{Eigen::Vector3d(3.0, 3.0, 3.0)};
    Eigen::Vector3d Kv{Eigen::Vector3d(3.0, 3.0, 3.0)};
    Eigen::Vector3d Kvi{Eigen::Vector3d(0.3, 0.3, 0.3)};  // PID 增益
    double mass_kg = 1.0;                                 // 起飞全重
    double gravity_mps2{9.81};
    double position_integral_error_xy_m{0.2};
    double position_integral_error_z_m{0.2};
    double gravity = 9.81;                      // 重力加速度
    double hover_percent = 0.37;                // 悬停时的推力百分比 (0.0~1.0)
    double max_pos_error = 0.3;                 // 位置误差截断 (米)
    double max_vel_error = 2.0;                 // 速度误差截断 (米/秒)
    double tilt_angle_max_rad = 0.52;           // 最大倾角 (约30度)
    double min_command_thrust = 0.1;            // 最小输出推力
    double controller_hz = 100.0;               // 控制频率
    Eigen::Vector3d int_max = {0.1, 0.1, 0.1};  // 积分限幅
    Eigen::Vector3d int_e_v_{Eigen::Vector3d::Zero()};
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
    Linear_AttitudeControl_Output_t
    calculateControl(const controller_data_types::TargetTrajectoryPoint_t des_state,
                     const control_common::UAVStateEstimate& current_odom,
                     const control_common::Mavros_IMU& imu,
                     bool enable_integral);
    // 推力模型估计
    bool estimateThrustModel(const Eigen::Vector3d& est_v);
    // 重置推力映射
    void resetThrustMapping(void);
    // 内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    Linear_AttitudeControl_Param_t param_;
    ControlParam test_control_param_;
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
