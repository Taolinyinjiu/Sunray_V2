#pragma once

#include "controller/controller_interface.hpp"
#include "mavros_helper/mavros_helper.hpp"
#include "utils/arrival_helper.hpp"
#include "utils/reference_limit_helper.hpp"
#include "utils/quintic_curve.hpp"
#include <Eigen/Dense>
#include <atomic>
#include <mutex>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <string>

class Raptor_Controller : public Controller_Interface {
  public:
    explicit Raptor_Controller(ros::NodeHandle& nh);
    ~Raptor_Controller() override = default;

    bool init() override;
    bool is_ready() override;

    void set_current_odom(const control_common::UAVStateEstimate& odom) override;
    void set_position_mode() override;

    bool takeoff(double relative_takeoff_height, double max_takeoff_velocity) override;
    bool land(bool land_type, double max_land_velocity) override;
    bool set_hover_point(control_common::UAVStateEstimate current_odom) override;
    bool hover() override;
    bool emergency_kill() override;
    bool move_point(controller_data_types::TargetPoint_t point) override;
    bool move_velocity(controller_data_types::TargetVelocity_t velocity) override;
    bool move_trajectory(controller_data_types::TargetTrajectoryPoint_t trajpoint) override;
    bool move_point_body(controller_data_types::TargetBodyPoint_t point) override;
    bool move_velocity_body(controller_data_types::TargetBodyVelocity_t velocity) override;
    bool move_point_wgs84(geographic_msgs::GeoPoint point) override;

    bool is_takeoff_complete() override;
    bool is_land_complete() override;
    bool is_point_complete() override;

    void pub_controller_state() override;
    void printf_logs(uint8_t log_level) override;

  private:
    struct LogSnapshot {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        bool controller_ready{false};
        bool takeoff_complete{false};
        bool land_complete{false};
        bool point_complete{false};
        bool has_uav_odometry{false};
        bool can_fuse{false};

        control_common::Mavros_State px4_state{};
        control_common::UAVStateEstimate odom{};
        control_common::Mavros_SetpointLocal px4_local_target{};
        control_common::Mavros_SetpointLocal last_setpoint{};
        controller_data_types::TargetTrajectoryPoint_t desired_state{};
        Eigen::Vector3d hover_point{Eigen::Vector3d::Zero()};
        double hover_yaw{0.0};
    };

    void load_and_validate_config_or_throw();
    void ensure_fusion_param_ready_or_throw();
    double compute_move_point_curve_maxvel(const Eigen::Vector3d& start_position,
                                           const Eigen::Vector3d& goal_position) const;
    bool move_point_impl(controller_data_types::TargetPoint_t point,
                         bool preserve_body_point_context);
    double update_limited_yaw_target(double target_yaw, const ros::Time& now);
    void warn_if_trajectory_exceeds_limits(
        const controller_data_types::TargetTrajectoryPoint_t& trajpoint) const;
    bool publish_trajectory_setpoint(
        const controller_data_types::TargetTrajectoryPoint_t& trajpoint,
        bool preserve_point_context);
    void reset_point_motion_context();
    void clear_cached_setpoint();
    void cache_local_setpoint(const control_common::Mavros_SetpointLocal& setpoint);
    controller_data_types::TargetTrajectoryPoint_t make_hold_desired_state(
        const control_common::UAVStateEstimate& odom) const;
    void publish_hold_setpoint(double yaw);
    bool ensure_raptor_mode(const ros::Time& now, double yaw);
    void update_log_snapshot();
    LogSnapshot get_log_snapshot() const;
    void pub_px4_state_timer_cb(const ros::TimerEvent&);
    void pub_vision_fuse_timer_cb(const ros::TimerEvent&);

    std::string config_yamlfile_path_;
    std::string uav_ns_;

    int fuse_odom_type{0};
    double fuse_odom_frequency{0.0};
    Eigen::Vector3d max_move_velocity_{Eigen::Vector3d::Ones()};
    double max_yaw_rate_rad_s_{0.5};
    arrival_helper::Config arrival_judge_config_{};
    arrival_helper::Config takeoff_arrival_config_{};

    ros::NodeHandle nh_;
    MavrosHelper mavros_helper_;

    std::atomic<bool> controller_ready_{false};

    double ground_height_ref_{0.0};
    double takeoff_yaw_{0.0};
    double land_yaw_{0.0};
    ros::Time last_checkout_raptor_time_{ros::Time(0)};
    curve::QuinticCurve quint_curve_;
    curve::QuinticCurve move_point_curve_;
    ros::Time start_land_time_{ros::Time(0)};
    ros::Time land_touchground_time_{ros::Time(0)};

    arrival_helper::State takeoff_arrival_state_{};
    arrival_helper::State point_arrival_state_{};
    reference_limit_helper::YawReferenceState yaw_reference_state_{};
    bool point_target_initialized_{false};
    bool body_point_target_initialized_{false};

    control_common::Mavros_SetpointLocal last_setpoint_{};
    controller_data_types::TargetPoint_t last_point_{};
    controller_data_types::TargetBodyPoint_t last_point_body_{};
    Eigen::Vector3d hover_point_{Eigen::Vector3d::Zero()};
    double hover_yaw_{0.0};

    std::atomic<bool> takeoff_complete_{false};
    std::atomic<bool> land_complete_{false};
    std::atomic<bool> point_complete_{false};

    control_common::UAVStateEstimate uav_odometry_;
    std::atomic<bool> has_uav_odometry_{false};
    std::atomic<bool> can_fuse_{false};
    ros::Time last_odom_timestamp_{ros::Time(0)};

    controller_data_types::TargetTrajectoryPoint_t desired_state_{};

    ros::Publisher controller_state_pub_;
    ros::Timer pub_px4_state_timer_;
    ros::Timer pub_vision_pose_timer_;

    mutable std::mutex log_snapshot_mutex_;
    LogSnapshot log_snapshot_{};
};
