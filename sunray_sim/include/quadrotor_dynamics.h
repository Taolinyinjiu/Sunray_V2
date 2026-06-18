#ifndef QUADROTOR_DYNAMICS_H
#define QUADROTOR_DYNAMICS_H

#include <Eigen/Dense>
#include "common_types.h"

namespace sunray_sim
{
class QuadrotorDynamics
{
public:
    // 构造函数 - 通过参数初始化
    QuadrotorDynamics(const DynamicParams& dyn_params, const MotorParams& motor_params);
    ~QuadrotorDynamics(){};

    // 无人机刚体动力学参数
    DynamicParams dynamic_params;   
    // 电机参数
    MotorParams motor_params;
    // 无人机状态
    DroneState drone_state;
    // 无人机输入
    DroneInput drone_input;

    double sim_time;      // 仿真时间，单位：s
    double dt;            // 时间步长，单位：s

    // 重置状态
    void reset(const Eigen::Vector3d& init_pos, const Eigen::Vector4d& init_quat);
    // 暂停动力学状态，用于无电机指令时冻结仿真。
    void pause();
    // 更新无人机状态
    void update(const DroneInput& input, double dt);

    // 获取当前状态
    const DroneState& getState() const { return drone_state; }
    const DynamicParams& getParams() const { return dynamic_params; }
    const MotorParams& getMotorParams() const { return motor_params; }

private:
    
    double motorRpmDynamics(double rpm_des, double rpm_act, double dt);
    void updateMotors(double dt);
    void computeMotorThrust();
    void computeTotalThrust();
    void computeTotalTorque();
    void updateAngularVelocity(double dt);
    void updateAttitude(double dt);
    void updateVelocity(double dt);
    void updatePosition(double dt);

    double clamp(double val, double min, double max);
    Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Vector4d& quat);
    Eigen::Vector3d quaternionToEuler(const Eigen::Vector4d& quat);
    Eigen::Vector4d eulerToQuaternion(const Eigen::Vector3d& euler);
    Eigen::Vector4d quaternionMultiplication(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);
    Eigen::Vector4d quaternionNormalize(const Eigen::Vector4d& quat);
};
}  // namespace sunray_sim

#endif
