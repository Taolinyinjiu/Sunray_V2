#ifndef SUNRAY_SIM_PX4_CONTROL_H
#define SUNRAY_SIM_PX4_CONTROL_H

#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <cstdint>
#include <set>
#include <string>

namespace sunray_sim
{
class Px4ControlSim
{
public:// 构造函数
    Px4ControlSim(ros::NodeHandle& nh, const std::string& uav_name);
    void update();
    void printStatus() const;
    
private:
    /*
     * POSITION_CONTROL 支持的 PositionTarget 组合模式
     *
     * 说明：
     * 1. 这里的“支持”指的是控制器能按字段叠加语义正确解释 setpoint。
     * 2. 实现方式不是为每个 type_mask 单独写分支，而是统一拆成：
     *    - 位置反馈：pos_xy / pos_z
     *    - 速度输入：vel_xy / vel_z
     *    - 加速度输入：acc_xy / acc_z
     *    - 航向控制：yaw
     *    - 航向角速度输入：yaw_rate
     * 3. 其中“输入”是否表现为前馈，取决于是否同时存在上一级目标：
     *    - 只有 vel 时：vel 是直接速度目标，不叫前馈更准确
     *    - pos + vel 时：vel 作为位置环输出上的速度前馈
     *    - 只有 acc 时：acc 是直接加速度目标，不叫前馈更准确
     *    - vel + acc 或 pos + vel + acc 时：acc 作为速度环输出上的加速度前馈
     *    - 只有 yaw_rate 时：yaw_rate 是直接偏航角速度目标
     *    - yaw + yaw_rate 时：yaw_rate 作为偏航控制上的角速度前馈
     * 4. 因此只要组合能映射到这几类输入，就应当可用。
     *
     * 当前应支持的常用模式包括：
     * - xyz_pos
     * - xyz_pos_yaw
     * - xyz_pos_yawrate
     * - xyz_vel
     * - xyz_vel_yaw
     * - xyz_vel_yawrate
     * - xy_vel_z_pos
     * - xy_vel_z_pos_yaw
     * - xy_vel_z_pos_yawrate
     * - xy_pos_z_vel_yaw
     * - xy_pos_z_vel_yawrate
     * - xyz_pos_vel
     * - xyz_pos_vel_yaw
     * - xyz_pos_vel_yawrate
     * - xyz_acc
     * - xyz_acc_yaw
     * - xyz_acc_yawrate
     * - pos_vel_acc_yaw
     * - pos_vel_acc_yawrate
     *
     * 不属于正常控制模式的：
     * - RAPTOR：只是一个“忽略 acceleration”的掩码片段，不是完整模式
     * - NONE_TYPE：所有字段都忽略，没有控制意义
     */
    /*
     * ATTITUDE_CONTROL / BODYRATE_CONTROL 支持的 AttitudeTarget 模式
     *
     * 当前只支持 3 类：
     * 1. body_rate + thrust
     *    - type_mask: IGNORE_ATTITUDE
     *    - 语义：直接跟踪 body_rate.xyz，thrust 直接换算为总推力
     *
     * 2. roll_pitch_yaw + thrust
     *    - type_mask: IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE
     *    - 语义：四元数提供完整姿态目标，thrust 直接换算为总推力
     *
     * 3. roll_pitch_yawrate + thrust
     *    - type_mask: IGNORE_ROLL_RATE | IGNORE_PITCH_RATE
     *    - 语义：四元数只取 roll/pitch，yaw 轴由 body_rate.z 直接控制
     *
     * 其他 AttitudeTarget 组合当前都不定义为支持模式。
     */
    // 控制模式
    enum ControlMode {
        POSITION_CONTROL,
        VELOCITY_CONTROL,
        ATTITUDE_CONTROL,
        BODYRATE_CONTROL
    };
    
    ControlMode control_mode_;
    bool has_setpoint_;
    bool use_pos_xy_;
    bool use_vel_xy_;
    bool use_acc_xy_;
    bool use_pos_z_;
    bool use_vel_z_;
    bool use_acc_z_;
    bool use_yaw_;
    bool use_yaw_rate_;
    bool has_odom_{false};
    bool mavros_armed_{true};
    bool has_mavros_state_{false};
    bool setpoint_timed_out_{false};
    
    // ROS节点和话题
    ros::NodeHandle nh_;
    std::string uav_name_;
    
    // 订阅者
    ros::Subscriber attitude_target_sub_;
    ros::Subscriber position_target_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber mavros_state_sub_;
    
    // 发布者
    ros::Publisher motor_rpm_pub_;
    ros::Timer update_timer_;
    std::string mavros_mode_{"OFFBOARD"};
    ros::Time last_supported_setpoint_time_;
    std::string last_rejected_setpoint_reason_;
    std::set<uint32_t> warned_position_target_keys_;
    std::set<uint8_t> warned_attitude_target_masks_;
    
    // 当前状态（由里程计话题获取）
    struct State {
        Eigen::Vector3d pos;            // 位置 [x, y, z]，单位：m（世界坐标系）
        Eigen::Vector3d vel;            // 速度 [vx, vy, vz]，单位：m/s（世界坐标系）
        Eigen::Vector3d euler_angles;   // 欧拉角 [roll, pitch, yaw]，单位：rad（世界坐标系→机体坐标系）
        Eigen::Vector3d ang_vel;        // 角速度 [ωx, ωy, ωz]，单位：rad/s（机体坐标系）
    } current_state_;
    
    // 目标值（由外部输入赋值得到，来自MAVROS话题）
    struct PX4_Setpoint {
        Eigen::Vector2d pos_xy;             // 位置设定值 [x, y]，单位：m（世界坐标系）
        double pos_z;                       // 位置设定值 z，单位：m（世界坐标系）
        Eigen::Vector2d vel_xy;             // 速度设定值 [vx, vy]，单位：m/s（世界坐标系）
        double vel_z;                       // 速度设定值 vz，单位：m/s（世界坐标系）
        Eigen::Vector2d acc_xy;             // 加速度前馈 [ax, ay]，单位：m/s²（世界坐标系）
        double acc_z;                       // 加速度前馈 az，单位：m/s²（世界坐标系）
        Eigen::Vector2d att_roll_pitch;     // 姿态设定值 [roll, pitch]，单位：rad
        double att_yaw;                     // 偏航角设定值，单位：rad
        Eigen::Vector2d bodyrate_roll_pitch;// 角速度设定值 [ωx, ωy]，单位：rad/s（机体坐标系）
        double bodyrate_yaw_rate;           // 偏航角速度设定值，单位：rad/s（机体坐标系）
        double collective_thrust;           // 归一化推力，范围：[0, 1]，0表示0推力，1表示最大推力
    } px4_setpoint_;

    // 控制器期望值（由内部控制器计算得到）
    struct DesiredState {
        Eigen::Vector2d pos_xy;             // 期望位置 [x, y]，单位：m（世界坐标系）
        double pos_z;                       // 期望位置 z，单位：m（世界坐标系）
        Eigen::Vector2d vel_xy;             // 期望速度 [vx, vy]，单位：m/s（世界坐标系）
        double vel_z;                       // 期望速度 vz，单位：m/s（世界坐标系）
        Eigen::Vector3d acc;                // 期望加速度 [ax, ay, az]，单位：m/s²（世界坐标系）
        Eigen::Vector2d att_roll_pitch;     // 期望姿态 [roll, pitch]，单位：rad
        double att_yaw;                     // 期望偏航角，单位：rad
        Eigen::Vector2d bodyrate_roll_pitch;// 期望角速度 [ωx, ωy]，单位：rad/s（机体坐标系）
        double bodyrate_yaw_rate;           // 期望偏航角速度，单位：rad/s（机体坐标系）
        Eigen::Vector3d thrust;             // 期望推力 [0, 0, T]，单位：N（机体坐标系Z轴方向）
        Eigen::Vector3d torque;             // 期望力矩 [τx, τy, τz]，单位：N·m（机体坐标系）
        Eigen::Vector4d motor_thrust;       // 期望电机推力 [T1, T2, T3, T4]，单位：N
        Eigen::Vector4d motor_rpm;                // 期望电机转速 [rpm1, rpm2, rpm3, rpm4]，单位：rpm
    } desired_state_;

    // 控制参数
    double dt_;                     // 控制周期，单位：s
    double mass_;
    double gravity_;
    double motor_k_f_;
    double motor_max_rpm_;
    double motor_arm_length_;
    double motor_k_t_;
    double max_vel_xy_;
    double max_vel_z_;
    double max_acc_xy_;
    double max_acc_z_;
    double max_tilt_rad_;
    double setpoint_timeout_{0.5};
    double vel_ff_xy_gain_when_pos_active_;
    double acc_xy_lpf_tau_;
    double acc_ff_xy_gain_;
    double max_roll_pitch_rate_;
    double max_yaw_rate_;
    
    // PID参数
    Eigen::Vector3d pos_pid_kp_;
    
    Eigen::Vector3d vel_pid_kp_;
    Eigen::Vector3d vel_pid_ki_;
    Eigen::Vector3d vel_pid_integral_;
    double vel_integral_limit_;
    Eigen::Vector2d filtered_acc_xy_;
    bool filtered_acc_xy_initialized_{false};
    
    Eigen::Vector3d att_pid_kp_;
    
    Eigen::Vector3d bodyrate_pid_kp_;
    Eigen::Vector3d bodyrate_pid_ki_;
    Eigen::Vector3d bodyrate_pid_integral_;
    double bodyrate_integral_limit_;
    
    // 初始化PID参数
    void initPIDParams();
    
    // 回调函数
    void attitudeTargetCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
    void positionTargetCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    void updateTimerCallback(const ros::TimerEvent& event);
    bool isSupportedPositionTarget(const mavros_msgs::PositionTarget& msg, std::string& reason) const;
    bool isSupportedAttitudeTarget(const mavros_msgs::AttitudeTarget& msg, std::string& reason) const;
    void warnUnsupportedPositionTargetOnce(const mavros_msgs::PositionTarget& msg, const std::string& reason);
    void warnUnsupportedAttitudeTargetOnce(const mavros_msgs::AttitudeTarget& msg, const std::string& reason);
    void markSupportedSetpoint();
    void clearControllerOutput();
    std::string buildStatusPanel() const;
    const char* controlModeName() const;
    
    // 控制子函数
    /**
     * 位置控制函数
     * 功能：根据位置设定值计算期望速度
     * 输入：
     *   - 外部输入：px4_setpoint_.pos_xy、px4_setpoint_.pos_z（位置设定值）
     *   - 当前状态：current_state_.pos（当前位置）
     *   - 控制输入：位置反馈 + 速度前馈
     * 输出：
     *   - desired_state_.vel_xy、desired_state_.vel_z（期望速度）
     */
    void positionControl();
    
    /**
     * 速度控制函数
     * 功能：根据速度设定值计算期望加速度
     * 输入：
     *   - 期望速度：desired_state_.vel_xy、desired_state_.vel_z（来自位置控制和/或速度前馈）
     *   - 当前状态：current_state_.vel（当前速度）
     *   - PID参数：vel_pid_kp_、vel_pid_ki_
     *   - 加速度前馈：px4_setpoint_.acc_xy、px4_setpoint_.acc_z
     * 输出：
     *   - desired_state_.acc（期望加速度）
     */
    void velocityControl();
    
    /**
     * 加速度转换为姿态函数
     * 功能：将期望加速度转换为期望姿态角和推力
     * 输入：
     *   - desired_state_.acc（期望加速度）
     * 输出：
     *   - desired_state_.att_roll_pitch、desired_state_.att_yaw（期望姿态）
     *   - desired_state_.thrust（期望推力）
     */
    void accelerationToAttitude();
    
    /**
     * 姿态控制函数
     * 功能：根据姿态设定值计算期望角速度
     * 输入：
     *   - 期望姿态：desired_state_.att_roll_pitch、desired_state_.att_yaw（来自加速度转换）或 px4_setpoint_.att_roll_pitch、px4_setpoint_.att_yaw（直接姿态控制）
     *   - 当前状态：current_state_.euler_angles（当前姿态）
     *   - PID参数：att_pid_kp_
     * 输出：
     *   - desired_state_.bodyrate_roll_pitch、desired_state_.bodyrate_yaw_rate（期望角速度）
     */
    void attitudeControl();
    
    /**
     * 角速度控制函数
     * 功能：根据角速度设定值计算期望力矩
     * 输入：
     *   - 期望角速度：desired_state_.bodyrate_roll_pitch、desired_state_.bodyrate_yaw_rate（来自姿态控制）或 px4_setpoint_.bodyrate_roll_pitch、px4_setpoint_.bodyrate_yaw_rate（直接角速度控制）
     *   - 当前状态：current_state_.ang_vel（当前角速度）
     *   - PID参数：bodyrate_pid_kp_、bodyrate_pid_ki_
     * 输出：
     *   - desired_state_.torque（期望力矩）
     */
    void bodyrateControl();
    
    // 电机控制相关子函数
    /**
     * 计算真实推力函数
     * 功能：将归一化推力转换为真实推力
     * 输入：
     *   - collective_thrust：归一化推力（0-1）
     * 输出：
     *   - 返回：Eigen::Vector3d（真实推力向量，单位：N）
     */
    Eigen::Vector3d computeRealThrust(double collective_thrust);
    
    /**
     * 计算电机推力函数
     * 功能：根据期望推力和力矩计算四个电机的单独推力
     * 输入：
     *   - thrust：期望总推力（来自desired_state_.thrust(2)）
     *   - torque：期望力矩（来自desired_state_.torque）
     * 输出：
     *   - 返回：Eigen::Vector4d（四个电机的推力）
     *   - 存储到：desired_state_.motor_thrust
     */
    Eigen::Vector4d computeMotorThrust(double thrust, const Eigen::Vector3d& torque);
    
    /**
     * 计算电机转速函数
     * 功能：根据电机推力计算电机转速
     * 输入：
     *   - motor_thrust：四个电机的推力（来自desired_state_.motor_thrust）
     * 输出：
     *   - 返回：Eigen::Vector4d（四个电机的转速）
     *   - 存储到：desired_state_.motor_rpm
     */
    Eigen::Vector4d computeMotorRPMFromThrust(const Eigen::Vector4d& motor_thrust);
    
    /**
     * 发布电机转速函数
     * 功能：将电机转速发布到ROS话题
     * 输入：
     *   - motor_rpm：四个电机的转速（来自desired_state_.motor_rpm）
     * 输出：
     *   - 无（发布到ROS话题 uav_name_/sunray_sim/cmd_RPM）
     */
    void publishMotorRPM(const Eigen::Vector4d& motor_rpm);
    void publishZeroMotorRPM();
};
}  // namespace sunray_sim

#endif  // SUNRAY_SIM_PX4_CONTROL_H
