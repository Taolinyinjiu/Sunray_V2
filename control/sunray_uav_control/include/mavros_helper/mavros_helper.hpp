/**
 * @file mavros_helper.hpp
 * @brief 将mavros的接口进行抽象，向控制器提供快速的控制方式
 * @details
    1. 订阅绝大部分Mavros的状态话题（参考原来的Sunray项目框架）
    2. 发布大部分Mavros的控制话题/服务
    3. Vision_pose话题
    4. 发布PX4State话题，属于自定义消息sunray_msgs::PX4_State
    5. 修改构造函数为仅存储ros句柄，显示调用init()函数进行初始化
    6. TODO: [ ] 实现odometry/in接口的外部里程计融合
             [-] pub_vision_pose()函数需要将传递的外部里程计数据拷贝到external_odometry_data_
             [ ] 修改is_ready()的校验逻辑
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-30
 * @version 0.3
 *
 *
 */

#pragma once

#include "control_data_types/mavros_helper_data_types.hpp"
#include "control_data_types/uav_state_estimate.hpp"
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <shared_mutex>
#include <atomic>
#include <type_traits>

#include <mavros_msgs/CommandLong.h>
#include <nav_msgs/Odometry.h>

class MavrosHelper {
  public:
    MavrosHelper();
    ~MavrosHelper() = default;
    // init 函数，用于mavros构造完成后显式初始化
    // 职责: 1. 初始化订阅者
    //      2. 初始化发布者
    //      3. 检查订阅者与发布者是否成功初始化
    // 使用void作为返回类型，不满足工作条件的情况会抛出运行时异常，直接结束节点
    void init();
    // 检查mavros_helper是否稳定工作
    bool is_ready();
    control_common::Mavros_State get_state();
    control_common::Mavros_Estimator get_estimator_status();
    control_common::UAVStateEstimate get_odometry();
    control_common::Mavros_Pose get_local_pose();
    control_common::Mavros_Velocity get_local_velocity();
    control_common::Mavros_IMU get_imu_data();
    Eigen::Quaterniond get_attitude_quat();
    Eigen::Vector3d get_attitude_euler_rad();
    Eigen::Vector3d get_attitude_euler_deg();
    double get_yaw_rad();
    double get_yaw_deg();
    control_common::Mavros_SetpointLocal get_target_local();
    control_common::Mavros_SetpointAttitude get_target_attitude();
    /*----------------以下部分与uav_control强相关----------------- */
    void set_vision_fuse_type(int fuse_type);
    bool pub_vision_pose(const control_common::UAVStateEstimate& uav_state);
    bool set_px4_mode(control_common::FlightMode flight_mode);
    bool set_px4_external_mode(uint8_t external_mode_index);
    bool set_arm(bool arm_state);
    bool emergency_kill();
    bool reboot_px4();
    bool pub_local_setpoint(const control_common::Mavros_SetpointLocal& setpoint_local);
    bool pub_attitude_setpoint(const control_common::Mavros_SetpointAttitude& setpoint_attitude);
    bool pub_px4_state();
    /* ---------------设置与读取px4参数---------------------*/
    // 模板函数，需要头文件中进行定义而不是在cpp中
    template <typename T> bool set_param(const T& param) {
        using raw_type = typename T::raw_type;
        static_assert(std::is_same<decltype(param.encode()), raw_type>::value,
                      "encode() return type must match raw_type");

        return set_param_raw(T::param_name, param.encode());
    }
    template <typename T> bool read_param(T* param) {
        // 读保护,如果传入空指针返回false
        if (param == nullptr) {
            return false;
        }

        typename T::raw_type raw{};
        // 读取参数原始值
        if (!read_param_raw(T::param_name, &raw)) {
            return false;
        }
        // 解码
        param->decode(raw);
        return true;
    }
  private:
    ros::Subscriber state_sub_;
    ros::Subscriber extended_state_sub_;
    ros::Subscriber sys_sub_;
    ros::Subscriber estimator_sub_;
    ros::Subscriber local_odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber setpoint_local_sub_;
    ros::Subscriber setpoint_attitude_sub_;
    ros::Subscriber gps_raw_sub_;
    ros::Subscriber rc_in_sub_;

    ros::Publisher vision_pose_pub_;
    ros::Publisher vision_odometry_pub_;
    ros::Publisher setpoint_local_pub_;
    ros::Publisher setpoint_attitude_pub_;
    ros::Publisher px4_state_pub_;

    ros::ServiceClient px4_arm_client_;
    ros::ServiceClient px4_mode_client_;
    ros::ServiceClient px4_cmdlong_client_;  // 用于emergency kill 和 reboot

    // PX4参数读写
    ros::ServiceClient param_set_client_;
    ros::ServiceClient param_get_client_;

    ros::NodeHandle nh_;
    std::string uav_ns_;
    std::string raptor_cmode_;  // 预计算的 Raptor CMODE 字符串

    std::atomic<bool> mavros_ready{false};
    control_common::VisionFuseType fuse_vision_type_ = control_common::VisionFuseType::Undefined;
    control_common::Mavros_State mavros_state_data_;
    control_common::Mavros_Estimator mavros_estimator_data_;
    control_common::UAVStateEstimate mavros_odometry_data_;
    control_common::UAVStateEstimate external_odometry_data_;
    control_common::Mavros_Pose mavros_local_pose_data_;
    control_common::Mavros_Velocity mavros_local_vel_data_;
    Eigen::Quaterniond mavros_attitude_data_;
    control_common::Mavros_SetpointLocal mavros_setpoint_local_data_;
    control_common::Mavros_SetpointAttitude mavros_setpoint_attitude_data_;
    control_common::Mavros_GPS mavros_gps_;
    control_common::Mavros_IMU mavros_imu_data_;
    control_common::Mavros_RC mavros_rc_data_;

    bool param_initialized_ = false;

    mutable std::shared_mutex data_mutex_;

    void mavros_state_callback(const mavros_msgs::State& msg);
    void mavros_externdedstate_callback(const mavros_msgs::ExtendedState& msg);
    void mavros_sys_callback(const mavros_msgs::SysStatus& msg);
    void mavros_estimator_callback(const mavros_msgs::EstimatorStatus& msg);
    void mavros_localodom_callback(const nav_msgs::Odometry& msg);
    void mavros_imu_callback(const sensor_msgs::Imu& msg);
    void mavros_setpoint_local_callback(const mavros_msgs::PositionTarget& msg);
    void mavros_setpoint_attitude_callback(const mavros_msgs::AttitudeTarget& msg);
    void mavros_gps_raw_callback(const mavros_msgs::GPSRAW& msg);
    void mavros_rc_in_callback(const mavros_msgs::RCIn& msg);

    // 通过调用mavros提供的服务,向px4飞控写入参数
    bool set_param_raw(const char* name, int value);
    bool set_param_raw(const char* name, double value);
    // 通过调用mavros提供的服务,读取px4飞控指定的参数
    bool read_param_raw(const char* name, int* value);
    bool read_param_raw(const char* name, double* value);
};
