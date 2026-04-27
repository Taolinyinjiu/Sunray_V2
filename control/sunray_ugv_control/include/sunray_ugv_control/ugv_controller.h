#pragma once

#include "sunray_ugv_control/ugv_control_utils.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <mutex>
#include <ros/ros.h>
#include <string>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVControllerState.h>

namespace sunray_ugv_control {

struct PIDGains {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
};

// 从 yaml 读取并汇总后的控制器运行参数。
struct UGVControllerConfig {
    int ugv_type{sunray_msgs::UGVControllerState::MECANUM};
    double state_pub_frequency{100.0};

    PIDGains point_x;
    PIDGains point_y;
    PIDGains point_yaw;
    PIDGains vel_x;
    PIDGains vel_y;
    PIDGains vel_yaw;

    double lateral_to_yaw_gain{1.0};
    double max_linear_x{1.0};
    double max_linear_y{1.0};
    double max_angular_z{1.0};
    double goal_pos_tolerance{0.15};
    double goal_yaw_tolerance{0.20};
};

// 控制器所需的最小平面运动状态。
struct UGVKinematicState {
    geometry_msgs::Point position;
    geometry_msgs::Vector3 velocity;
    double yaw{0.0};
};

class UGVController {
  public:
    UGVController(ros::NodeHandle& nh,
                  const UGVControllerConfig& config,
                  const std::string& state_topic);

    void set_current_state(const UGVKinematicState& state);

    geometry_msgs::Twist hold();
    geometry_msgs::Twist move_point(const sunray_msgs::UGVControlCMD& cmd);
    geometry_msgs::Twist move_velocity(const sunray_msgs::UGVControlCMD& cmd);
    geometry_msgs::Twist move_velocity_body(const sunray_msgs::UGVControlCMD& cmd);

    bool reached_point(const sunray_msgs::UGVControlCMD& cmd) const;
    sunray_msgs::UGVControllerState get_status_snapshot() const;

  private:
    class PIDAxis {
      public:
        explicit PIDAxis(const PIDGains& gains = PIDGains{});

        void reset();
        double update(double error, double dt);

      private:
        PIDGains gains_;
        double integral_{0.0};
        double last_error_{0.0};
        bool initialized_{false};
    };

    void set_mode(uint8_t control_mode);
    double compute_dt();
    geometry_msgs::Twist apply_platform_limits(const geometry_msgs::Twist& raw_cmd,
                                               bool direct_body_command) const;
    sunray_msgs::UGVControllerState build_status_message(const ros::Time& stamp) const;
    void status_timer_callback(const ros::TimerEvent&);

    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    ros::Timer state_timer_;

    UGVControllerConfig config_;
    mutable std::mutex mutex_;

    UGVKinematicState current_state_;
    // 缓存最近一次期望状态，保证 100Hz 状态话题能反映当前正在跟踪的目标。
    geometry_msgs::Point desired_pos_;
    geometry_msgs::Vector3 desired_vel_;
    geometry_msgs::Vector3 desired_linear_;
    geometry_msgs::Vector3 desired_angular_;
    double desired_yaw_{0.0};
    geometry_msgs::Twist last_cmd_;
    uint8_t control_mode_{sunray_msgs::UGVControllerState::CONTROL_HOLD};
    ros::Time last_update_stamp_;

    PIDAxis point_x_pid_;
    PIDAxis point_y_pid_;
    PIDAxis point_yaw_pid_;
    PIDAxis vel_x_pid_;
    PIDAxis vel_y_pid_;
    PIDAxis vel_yaw_pid_;
};

}  // namespace sunray_ugv_control
