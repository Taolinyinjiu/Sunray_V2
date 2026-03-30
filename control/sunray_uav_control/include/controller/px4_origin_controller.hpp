/**
 * @file px4_origin_controller.hpp
 * @brief 基于px4原生控制环的控制器，从Controller_Interface派生
 * @details
    1. 引入Mavros_Helper类用于读取px4数据
    2. 引入Mavros_ParamManager类用于读取+修改 px4参数
    3. 向Sunray_FSM提供规范的函数实现
    4. 发布controller_state话题，属于自定义消息sunray_msgs::UAVControllerState
    5. 修改构造函数为仅存储ros句柄，显示调用init()函数进行初始化
    6. TODO: [ ] 实现public中需要的函数
             [-] 实现controller_state_pub_的初始化
             [-] 实现pub_timer_的初始化
             [ ] 实现px4_mode和px4_arm的同步(从mavros_helper中读取数据)
             [ ] desired_state_初始化
             [-] 引入Mavros_ParamManager类，修改EV_CTRL以及相关参数
             [ ] 修改Mavros_ParamManager类名为MavrosParam
             [ ] 引入px4_nsh_simulator 实现重启ekf2或者将ekf2重启放在Mavros_Param或者Helper？
             [ ] 考虑是否为不同的定位源引入不同的参数配置？比如EV_Delay之类？
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-30
 * @version 0.2
 *
 *
 */

#pragma once
#include "controller/controller_interface.hpp"
#include "mavros_helper/mavros_helper.hpp"
#include <px4_param_manager/px4_param_manager.h>
#include <ros/node_handle.h>
#include <string>

// 在这里构造参数结构体？对于原生控制器，要检查什么参数呢
struct Original_Param {
    // 1, vision_pose
    int fuse_odom_type{0};
    double fuse_odom_frequency{0.0};
    // 2. 暂时没想到
};

class PX4_OriginController : public Controller_Interface {
  public:
    PX4_OriginController(ros::NodeHandle& nh);
    ~PX4_OriginController() override = default;
    // ------------声明周期相关----------------
    bool init() override;  // init()读取config.yaml文件中的相关配置
    bool is_ready() override;  // is_ready()检查各项指标是否正常，比如里程计频率，融合参数等等
    // -------------状态注入---------------
    void set_current_odom(const control_common::UAVStateEstimate& odom) override;
    // -------------运动相关接口------------
    // 触发起飞，参数为起飞高度和最大起飞速度
    bool takeoff(double relative_takeoff_height, double max_takeoff_velocity) override;
    // 触发降落，参数为降落类型和最大降落速度
    bool land(bool land_type, double max_land_velocity) override;
    // 在当前点悬停(运动过程触发立即停止并进入悬停)
    bool hover() override;
    // 运动到某一点
    bool move_point(controller_data_types::TargetPoint_t point) override;
    // 以速度控制的方式运动
    bool move_velocity(controller_data_types::TargetVelocity_t velocity) override;
    // 控制无人机跟踪轨迹点
    bool move_trajectory(controller_data_types::TargetTrajectoryPoint_t trajpoint) override;
    // 运动到机体系的某一点
    bool move_point_body(controller_data_types::TargetPoint_t point) override;
    // 以机体系速度的方式运动
    bool move_velocity_body(controller_data_types::TargetVelocity_t velocity) override;
    // 移动到WGS84下的某一点
    bool move_point_wgs84(controller_data_types::TargetPoint_t point) override;
    // ---------------------起降状态查询接口-----------------------
    bool is_takeoff_complete() override;
    bool is_land_complete() override;
    // ----------------------控制器状态话题更新函数-----------------
    void pub_controller_state();  // 发布controller_state
  private:
    // ----------------------配置相关-----------------------
    std::string config_yamlfile_path_;
    std::string uav_ns_;
    Original_Param config_param_;
    // ----------------------检查参数-----------------------
    bool check_param();
    // ---------------------定时器回调函数---------------------
    void pub_px4_state_timer_cb(const ros::TimerEvent&);    // 发布PX4State话题
    void pub_vision_fuse_timer_cb(const ros::TimerEvent&);  // 发布vision_fuse
    // 将pub_controller_state() 移出定时器回调函数，其发布由状态机手动调用，以保持数据的新鲜度
    // -------------------ros句柄与mavros辅助类-----------------
    ros::NodeHandle nh_;
    MavrosHelper mavros_helper_;
    PX4_ParamManager mavros_param_;
    // --------------------px4状态-----------------------
    control_common::FlightMode px4_mode_ = control_common::FlightMode::Undefined;
    control_common::LandedState px4_land_ = control_common::LandedState::Undefined;
    bool px4_arm_ = false;
    double pub_px4_state_freq_ = 100.0;  // 考虑简单点，在这里硬编码px4state的发布频率
    bool param_state_ = false;
    // --------------------里程计状态---------------------
    control_common::UAVStateEstimate uav_odometry_;
    bool has_uav_odometry_{false};
    // --------------------期望状态--------------------
    controller_data_types::TargetTrajectoryPoint_t desired_state_;
    // ros话题发布者
    ros::Publisher controller_state_pub_;  // 发布controller状态话题数据
    // ros定时器
    ros::Timer pub_px4_state_timer;     // 用于发布话题数据的定时器
    ros::Timer pub_vision_pose_timer_;  // 用于发布vision_pose的定时器
};
