#include <Eigen/Dense>

/**
 * 四旋翼无人机动力学模型
 * 
 * 坐标系定义（右手法则）：
 * - 世界坐标系（惯性坐标系）：
 *   - X轴：朝前
 *   - Y轴：朝左
 *   - Z轴：竖直朝上
 *   （右手拇指指向X轴，食指指向Y轴，中指指向Z轴）
 * 
 * - 机体坐标系：
 *   - X轴：无人机前端
 *   - Y轴：无人机左侧
 *   - Z轴：无人机顶部（与世界坐标系Z轴方向一致）
 *   （右手拇指指向X轴，食指指向Y轴，中指指向Z轴）
 * 
 * 姿态表示：
 * - 四元数：[w, x, y, z]，表示从世界坐标系到机体坐标系的旋转
 * - 欧拉角：[roll, pitch, yaw]，单位：弧度
 *   - roll：绕X轴旋转（横滚）
 *   - pitch：绕Y轴旋转（俯仰）
 *   - yaw：绕Z轴旋转（偏航）
 */

// 无人机状态结构体
struct DroneState
{
    Eigen::Vector4d motor_omega;     // 电机转速，单位：rpm
    Eigen::Vector4d motor_thrust;    // 电机推力，单位：N
    double total_thrust;             // 总推力，单位：N（机体坐标系Z轴方向）
    Eigen::Vector3d total_torque;    // 三轴总扭矩，单位：Nm（机体坐标系）
    Eigen::Vector3d ang_vel;         // 无人机角速度，单位：rad/s（机体坐标系）
    Eigen::Vector3d euler_angles;    // 欧拉角（roll, pitch, yaw），单位：rad（世界坐标系→机体坐标系）
    Eigen::Vector4d quat;            // 无人机姿态，单位：四元数（世界坐标系→机体坐标系）
    Eigen::Vector3d vel;             // 无人机速度，单位：m/s（世界坐标系）
    Eigen::Vector3d pos;             // 无人机位置，单位：m（世界坐标系）
};

// 无人机刚体动力学参数结构体
struct DynamicParams
{
    double mass;                    // 无人机质量，单位：kg
    double gravity;                 // 重力加速度，单位：m/s^2
    Eigen::Matrix3d I;             // 无人机惯性矩阵，单位：kg·m²
    Eigen::Matrix3d I_inv;         // 无人机惯性矩阵的逆，单位：kg⁻¹·m⁻²
    double arm_length;              // 无人机臂长（无人机中心到电机中心的距离），单位：m
};

// 电机参数结构体
struct MotorParams
{
    double k_F;                     // 电机推力系数，单位：N/rpm^2
    double k_T;                     // 电机扭矩系数，单位：Nm/rpm
    double tau_up;            // 电机上升时间常数，单位：s
    double tau_down;          // 电机下降时间常数，单位：s
    double omega_min;         // 电机最小转速，单位：rpm
    double omega_max;         // 电机最大转速，单位：rpm
};

// 无人机输入结构体
struct DroneInput
{
    Eigen::Vector4d motor_omega_des; // 电机期望转速，单位：rpm
};
