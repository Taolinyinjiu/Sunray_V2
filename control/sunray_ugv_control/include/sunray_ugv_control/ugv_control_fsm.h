#pragma once

#include "sunray_ugv_control/ugv_control_utils.h"
#include "sunray_ugv_control/ugv_controller.h"
#include <memory>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sunray_msgs/OdomStatus.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVControlFSMState.h>

namespace sunray_ugv_control {

// ROS 话题连接关系与监督循环频率配置。
struct BasicConfig {
    int ugv_type{sunray_msgs::UGVControllerState::MECANUM};
    double controller_update_frequency{100.0};
    double supervisor_update_frequency{20.0};
    double controller_state_pub_frequency{100.0};

    std::string odom_topic_name;
    std::string odom_status_topic_name;
    std::string control_cmd_topic_name;
    std::string cmd_vel_topic_name;
    std::string fsm_state_topic_name;
    std::string controller_state_topic_name;
};

// 触发安全回退到 HOLD 的各类超时阈值。
struct TimeoutConfig {
    double wait_poscmd_time{2.0};
    double wait_velcmd_time{0.3};
    double odom_timeout{0.5};
};

// RETURN 模式使用的返航点配置。
struct HomeConfig {
    bool use_current_pose_as_home{true};
    geometry_msgs::Point home_point;
    double home_yaw{0.0};
};

struct UGVFSMConfig {
    BasicConfig basic;
    TimeoutConfig timeout;
    GeoFence fence;
    HomeConfig home;
    UGVControllerConfig controller;
};

class UGVControlFSM {
  public:
    explicit UGVControlFSM(ros::NodeHandle& nh);

    void init();
    double get_update_frequency() const;
    void process();

  private:
    // UGV 状态机刻意保持精简：
    // INIT 用于上电初始化，HOLD 用于安全静止，
    // RETURN 用于单次返航，MOVE 统一承接外部运动指令。
    enum class State : uint8_t {
        INIT = sunray_msgs::UGVControlFSMState::FSM_INIT,
        HOLD = sunray_msgs::UGVControlFSMState::FSM_HOLD,
        RETURN = sunray_msgs::UGVControlFSMState::FSM_RETURN,
        MOVE = sunray_msgs::UGVControlFSMState::FSM_MOVE,
    };

    void load_config();
    void init_subscribers();
    void init_publishers();

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void odom_status_callback(const sunray_msgs::OdomStatus::ConstPtr& msg);
    void control_cmd_callback(const sunray_msgs::UGVControlCMD::ConstPtr& msg);
    void control_timer_callback(const ros::TimerEvent&);

    bool is_command_fresh(const sunray_msgs::UGVControlCMD& cmd, const ros::Time& now) const;
    bool is_motion_command(const sunray_msgs::UGVControlCMD& cmd) const;
    bool odom_is_valid_locked(const ros::Time& now) const;

    void enter_hold(const std::string& reason);
    void publish_fsm_state();
    void publish_zero_cmd();
    sunray_msgs::UGVControlCMD make_hold_command() const;
    sunray_msgs::UGVControlCMD make_return_point_command() const;

    static double quaternion_to_yaw(const geometry_msgs::Quaternion& q);

    ros::NodeHandle nh_;
    std::string ugv_ns_;
    UGVFSMConfig config_;

    ros::Subscriber odom_sub_;
    ros::Subscriber odom_status_sub_;
    ros::Subscriber control_cmd_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher fsm_state_pub_;
    ros::Timer control_timer_;

    std::shared_ptr<UGVController> controller_;

    mutable std::mutex mutex_;
    State state_{State::INIT};
    nav_msgs::Odometry last_odom_;
    ros::Time last_odom_stamp_;
    bool odom_received_{false};
    bool odom_status_received_{false};
    bool odom_status_valid_{true};
    bool inside_geo_fence_{true};
    bool home_initialized_{false};
    // 运行时 home 点既可以来自 yaml，也可以来自首个有效里程计位置。
    HomeConfig home_runtime_;
    sunray_msgs::UGVControlCMD active_cmd_;
};

}  // namespace sunray_ugv_control
