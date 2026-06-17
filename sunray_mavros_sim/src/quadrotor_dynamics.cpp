#include "quadrotor_dynamics.h"
#include <cmath>

QuadrotorDynamics::QuadrotorDynamics(const DynamicParams& dyn_params, const MotorParams& motor_p)
    : dynamic_params(dyn_params), motor_params(motor_p), sim_time(0.0)
{
    // 计算惯性矩阵的逆
    dynamic_params.I_inv = dynamic_params.I.inverse();
    
    // 初始化状态
    drone_state.pos = Eigen::Vector3d::Zero();
    drone_state.quat = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    drone_state.vel = Eigen::Vector3d::Zero();
    drone_state.ang_vel = Eigen::Vector3d::Zero();
    drone_state.motor_omega = Eigen::Vector4d::Zero();
    drone_state.motor_thrust = Eigen::Vector4d::Zero();
    drone_state.total_thrust = 0.0;
    drone_state.total_torque = Eigen::Vector3d::Zero();
    drone_state.euler_angles = Eigen::Vector3d::Zero();
}

void QuadrotorDynamics::reset(const Eigen::Vector3d& init_pos, const Eigen::Vector4d& init_quat)
{
    drone_state.pos = init_pos;
    drone_state.quat = quaternionNormalize(init_quat);
    drone_state.vel = Eigen::Vector3d::Zero();
    drone_state.ang_vel = Eigen::Vector3d::Zero();
    drone_state.motor_omega = Eigen::Vector4d::Zero();
    drone_state.motor_thrust = Eigen::Vector4d::Zero();
    drone_state.total_thrust = 0.0;
    drone_state.total_torque = Eigen::Vector3d::Zero();
    drone_state.euler_angles = Eigen::Vector3d::Zero();
    sim_time = 0.0;
}

void QuadrotorDynamics::hold()
{
    drone_state.vel = Eigen::Vector3d::Zero();
    drone_state.ang_vel = Eigen::Vector3d::Zero();
    drone_state.motor_omega = Eigen::Vector4d::Zero();
    drone_state.motor_thrust = Eigen::Vector4d::Zero();
    drone_state.total_thrust = 0.0;
    drone_state.total_torque = Eigen::Vector3d::Zero();
}

void QuadrotorDynamics::update(const DroneInput& input, double dt)
{
    this->drone_input = input;
    this->dt = dt;

    // 计算电机转速(根据期望转速更新实际转速)
    updateMotors(dt);
    // 计算单个电机推力(根据电机转速更新推力)
    computeMotorThrust();
    // 计算总推力(根据电机推力更新总推力)
    computeTotalThrust();
    // 计算总扭矩（X型布局）(根据电机推力更新总扭矩)
    computeTotalTorque();
    // 更新角速度(根据总扭矩更新角速度)
    updateAngularVelocity(dt);
    // 更新姿态（四元数）(根据角速度更新姿态)
    updateAttitude(dt);
    // 更新速度(根据姿态更新速度)
    updateVelocity(dt);
    // 更新位置(根据速度更新位置)
    updatePosition(dt);
    
    sim_time += dt;
}

double QuadrotorDynamics::clamp(double val, double min, double max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * 电机动力学模型（一阶响应）
 * 数学公式：ω(t+dt) = ω(t) + (ω_des - ω(t)) / τ * dt
 * 其中：
 * - τ 为时间常数（上升或下降），单位s，反映电机响应速度，τ越小，响应越快
 * - ω_des 为期望转速，单位rpm，反映电机目标转速
 * - ω(t) 为当前转速，单位rpm，反映电机当前转速
 * - dt 为时间步长，单位s，反映模拟时间间隔
 */
double QuadrotorDynamics::motorDynamics(double omega_des, double omega_act, double dt)
{
    // 选择上升或下降时间常数，根据期望转速和当前转速的关系
    double tau = (omega_des > omega_act) ? motor_params.tau_up : motor_params.tau_down;
    // 计算下一个时间步的转速
    double omega_next = omega_act + (omega_des - omega_act) / tau * dt;
    // 对转速进行限制
    omega_next = clamp(omega_next, motor_params.omega_min, motor_params.omega_max);
    // 返回数值
    return omega_next;
}

void QuadrotorDynamics::updateMotors(double dt)
{
    for (int i = 0; i < 4; ++i)
    {
        drone_state.motor_omega(i) = motorDynamics(drone_input.motor_omega_des(i), drone_state.motor_omega(i), dt);
    }
}

/**
 * 计算每个电机的推力
 * 数学公式：F_i = k_F * ω_i²
 * 其中：
 * - k_F 为电机推力系数，单位：N/rpm²
 * - ω_i 为第i个电机的转速，单位：rpm
 * - F_i 为第i个电机的推力，单位：N
 */
void QuadrotorDynamics::computeMotorThrust()
{
    for (int i = 0; i < 4; ++i)
    {
        drone_state.motor_thrust(i) = motor_params.k_F * drone_state.motor_omega(i) * drone_state.motor_omega(i);
    }
}

void QuadrotorDynamics::computeTotalThrust()
{
    drone_state.total_thrust = drone_state.motor_thrust.sum();
}

/**
 * 计算无人机的总扭矩（X型布局）
 * 数学公式：
 * - 滚转扭矩 τ_x = L/√2 * (F2 + F3 - F1 - F4)
 * - 俯仰扭矩 τ_y = L/√2 * (F2 + F4 - F1 - F3)
 * - 偏航扭矩 τ_z = k_T * (ω3² + ω4² - ω1² - ω2²)
 * 其中：
 * - L 为无人机臂长，单位：m
 * - F1-F4 为四个电机的推力，单位：N
 * - ω1-ω4 为四个电机的转速，单位：rpm
 * - k_T 为电机扭矩系数，单位：Nm/rpm
 * 电机布局（从上往下看）：
 *         F3 (左前，逆时针)       F1 (右前，顺时针)
 *              \                     /
 *               \                   /
 *                \                 /
 *                 \               /
 *                  \             /
 *                   \           /
 *                    \         /
 *                     \       /
 *                      \     /
 *                       \   /
 *                        \ /
 *                         X
 *                        / \
 *                       /   \
 *                      /     \
 *                     /       \
 *                    /         \
 *                   /           \
 *                  /             \
 *                 /               \
 *        F2 (左后，顺时针)       F4 (右后，逆时针)
 * 
 * 坐标系（右手法则）：
 * - X轴：朝前
 * - Y轴：朝左
 * - Z轴：朝上
 * 
 * 扭矩方向说明：
 * - τ_x 正方向：绕X轴正方向（前Y轴反方向翻滚）
 * - τ_y 正方向：绕Y轴正方向（前倾）
 * - τ_z 正方向：绕Z轴正方向（逆时针偏航）
 */
void QuadrotorDynamics::computeTotalTorque()
{
    double L = dynamic_params.arm_length;
    double k_T = motor_params.k_T;
    
    double F1 = drone_state.motor_thrust(0);
    double F2 = drone_state.motor_thrust(1);
    double F3 = drone_state.motor_thrust(2);
    double F4 = drone_state.motor_thrust(3);
    
    double omega1 = drone_state.motor_omega(0);
    double omega2 = drone_state.motor_omega(1);
    double omega3 = drone_state.motor_omega(2);
    double omega4 = drone_state.motor_omega(3);
    
    double tau_x = L / sqrt(2.0) * (F2 + F3 - F1 - F4);
    double tau_y = L / sqrt(2.0) * (F2 + F4 - F1 - F3);
    double tau_z = k_T * (omega3*omega3 + omega4*omega4 - omega1*omega1 - omega2*omega2);
    
    drone_state.total_torque << tau_x, tau_y, tau_z;
}

/**
 * 更新无人机角速度
 * 数学公式：ω̇ = I⁻¹ (τ - ω × (I ω))
 * 其中：
 * - I 为惯性矩阵
 * - I⁻¹ 为惯性矩阵的逆
 * - τ 为总扭矩（机体坐标系）
 * - ω 为当前角速度（机体坐标系）
 * - ω × (I ω) 为哥氏力和离心力（机体坐标系）
 * - ω̇ 为角加速度（机体坐标系）
 * 
 * 说明：
 * 角速度和扭矩都在机体坐标系下计算，不需要坐标系转换
 * 惯性矩阵是在机体坐标系下定义的
 */
void QuadrotorDynamics::updateAngularVelocity(double dt)
{
    Eigen::Vector3d ang_acc = dynamic_params.I_inv * (drone_state.total_torque - drone_state.ang_vel.cross(dynamic_params.I * drone_state.ang_vel));
    
    drone_state.ang_vel += ang_acc * dt;
}

/**
 * 更新无人机姿态（四元数）
 * 数学公式：q̇ = 0.5 * q ⊗ ω
 * 其中：
 * - q 为当前四元数
 * - ω 为角速度向量（表示为四元数 [0, ω_x, ω_y, ω_z]）
 * - ⊗ 为四元数乘法
 * - q̇ 为四元数的导数
 * 四元数归一化：q = q / ||q||
 */
void QuadrotorDynamics::updateAttitude(double dt)
{
    Eigen::Vector4d omega_quat;
    omega_quat << 0.0, drone_state.ang_vel(0), drone_state.ang_vel(1), drone_state.ang_vel(2);
    
    Eigen::Vector4d q_dot = 0.5 * quaternionMultiplication(drone_state.quat, omega_quat);
    
    drone_state.quat += q_dot * dt;
    drone_state.quat = quaternionNormalize(drone_state.quat);
    
    drone_state.euler_angles = quaternionToEuler(drone_state.quat);
}

/**
 * 更新无人机速度
 * 数学公式：v(t+dt) = v(t) + a(t) * dt
 * 其中加速度 a(t) 计算如下：
 * a(t) = (R * F_thrust) / m + g
 * 各项含义：
 * - R 为旋转矩阵（从机体坐标系到世界坐标系）
 * - F_thrust 为机体系下的总推力（Z轴方向）
 * - m 为无人机质量
 * - g 为重力加速度（世界坐标系Z轴负方向）
 * 
 * 坐标系转换：
 * 1. 电机推力在机体坐标系下沿Z轴方向
 * 2. 通过旋转矩阵 R 将推力转换到世界坐标系
 * 3. 重力直接在世界坐标系下计算
 * 4. 所有力在世界坐标系下合成，计算加速度
 */
void QuadrotorDynamics::updateVelocity(double dt)
{
    Eigen::Matrix3d R = quaternionToRotationMatrix(drone_state.quat);
    Eigen::Vector3d thrust_body(0.0, 0.0, drone_state.total_thrust);
    Eigen::Vector3d thrust_world = R * thrust_body;
    
    Eigen::Vector3d gravity(0.0, 0.0, -dynamic_params.gravity);
    
    Eigen::Vector3d acc = (thrust_world / dynamic_params.mass) + gravity;
    
    drone_state.vel += acc * dt;
}

/**
 * 更新无人机位置
 * 数学公式：p(t+dt) = p(t) + v(t) * dt
 * 其中：
 * - p(t) 为当前位置
 * - v(t) 为当前速度
 * - dt 为时间步长
 */
void QuadrotorDynamics::updatePosition(double dt)
{
    drone_state.pos += drone_state.vel * dt;

    // Keep the vehicle on the ground plane until thrust is sufficient to lift it.
    if (drone_state.pos(2) < 0.0)
    {
        drone_state.pos(2) = 0.0;
        if (drone_state.vel(2) < 0.0)
        {
            drone_state.vel(2) = 0.0;
        }
    }
}

/**
 * 将四元数转换为旋转矩阵
 * 数学公式：
 * R = [
 *   1-2(q2²+q3²)   2(q1q2-q0q3)   2(q1q3+q0q2)
 *   2(q1q2+q0q3)   1-2(q1²+q3²)   2(q2q3-q0q1)
 *   2(q1q3-q0q2)   2(q2q3+q0q1)   1-2(q1²+q2²)
 * ]
 * 其中 q = [q0, q1, q2, q3] 为四元数
 */
Eigen::Matrix3d QuadrotorDynamics::quaternionToRotationMatrix(const Eigen::Vector4d& quat)
{
    double q0 = quat(0);
    double q1 = quat(1);
    double q2 = quat(2);
    double q3 = quat(3);
    
    Eigen::Matrix3d R;
    R << q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2),
         2*(q1*q2 + q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 - q0*q1),
         2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
    return R;
}

/**
 * 将四元数转换为欧拉角（roll, pitch, yaw）
 * 数学公式：
 * roll = atan2(2(q0q1 + q2q3), 1-2(q1²+q2²))
 * pitch = asin(2(q0q2 - q3q1))
 * yaw = atan2(2(q0q3 + q1q2), 1-2(q2²+q3²))
 * 其中 q = [q0, q1, q2, q3] 为四元数
 * 角度范围：
 * - roll: [-π, π]
 * - pitch: [-π/2, π/2]
 * - yaw: [-π, π]
 */
Eigen::Vector3d QuadrotorDynamics::quaternionToEuler(const Eigen::Vector4d& quat)
{
    double q0 = quat(0);
    double q1 = quat(1);
    double q2 = quat(2);
    double q3 = quat(3);
    
    double roll = atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2));
    double pitch = asin(2.0*(q0*q2 - q3*q1));
    double yaw = atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3));
    
    return Eigen::Vector3d(roll, pitch, yaw);
}

/**
 * 将欧拉角转换为四元数
 * 数学公式：
 * q0 = cos(roll/2)cos(pitch/2)cos(yaw/2) + sin(roll/2)sin(pitch/2)sin(yaw/2)
 * q1 = sin(roll/2)cos(pitch/2)cos(yaw/2) - cos(roll/2)sin(pitch/2)sin(yaw/2)
 * q2 = cos(roll/2)sin(pitch/2)cos(yaw/2) + sin(roll/2)cos(pitch/2)sin(yaw/2)
 * q3 = cos(roll/2)cos(pitch/2)sin(yaw/2) - sin(roll/2)sin(pitch/2)cos(yaw/2)
 * 其中 roll, pitch, yaw 为欧拉角
 */
Eigen::Vector4d QuadrotorDynamics::eulerToQuaternion(const Eigen::Vector3d& euler)
{
    double roll = euler(0);
    double pitch = euler(1);
    double yaw = euler(2);
    
    double cr = cos(roll/2.0);
    double sr = sin(roll/2.0);
    double cp = cos(pitch/2.0);
    double sp = sin(pitch/2.0);
    double cy = cos(yaw/2.0);
    double sy = sin(yaw/2.0);
    
    Eigen::Vector4d quat;
    quat(0) = cr*cp*cy + sr*sp*sy;
    quat(1) = sr*cp*cy - cr*sp*sy;
    quat(2) = cr*sp*cy + sr*cp*sy;
    quat(3) = cr*cp*sy - sr*sp*cy;
    
    return quat;
}

/**
 * 四元数乘法
 * 数学公式：
 * q1 ⊗ q2 = [
 *   q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
 *   q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
 *   q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
 *   q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
 * ]
 * 其中 q = [w, x, y, z]
 */
Eigen::Vector4d QuadrotorDynamics::quaternionMultiplication(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2)
{
    Eigen::Vector4d result;
    result(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3);
    result(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2);
    result(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1);
    result(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0);
    return result;
}

Eigen::Vector4d QuadrotorDynamics::quaternionNormalize(const Eigen::Vector4d& quat)
{
    double norm = quat.norm();
    if (norm < 1e-10)
    {
        return Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    }
    return quat / norm;
}
