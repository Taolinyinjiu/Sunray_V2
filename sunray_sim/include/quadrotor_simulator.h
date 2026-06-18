#ifndef SUNRAY_SIM_QUADROTOR_SIMULATOR_H
#define SUNRAY_SIM_QUADROTOR_SIMULATOR_H

#include "imu_model.h"
#include "quadrotor_dynamics.h"

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32MultiArray.h>

#include <memory>
#include <string>

namespace sunray_sim
{
class QuadrotorSimulator
{
public:
    QuadrotorSimulator(ros::NodeHandle& nh,
                       const std::string& agent_name,
                       int agent_id,
                       const Eigen::Vector3d& init_pos,
                       double init_yaw);
    void printStatus() const;

private:
    void motorRpmCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void updateTimerCallback(const ros::TimerEvent& event);
    void publishOdometry(const DroneState& state);
    void publishImu(const DroneState& state, const ros::Time& stamp);
    void publishNavSatFix(const DroneState& state, const ros::Time& stamp);
    Eigen::Vector3d computeBodySpecificForce(const DroneState& state, double dt) const;
    Eigen::Vector3d computeBodyAngularAcceleration(const DroneState& state, double dt) const;

    ros::NodeHandle nh_;
    std::string agent_prefix_;
    std::string agent_frame_prefix_;
    std::string global_frame_id_{"map"};
    std::string base_frame_id_;
    std::unique_ptr<QuadrotorDynamics> quadrotor_;
    sunray_imu_sim::ImuModel imu_model_;
    DroneInput input_;
    DroneState previous_state_;
    bool has_previous_state_{false};
    bool has_motor_rpm_cmd_{false};
    ros::Time last_motor_rpm_msg_time_;

    ros::Subscriber motor_rpm_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher navsat_pub_;
    ros::Timer update_timer_;

    DynamicParams dynamic_params_;
    double dynamics_update_rate_{500.0};
    double dt_{0.002};
    double cmd_timeout_{0.1};
    double sim_time_{0.0};
};
}  // namespace sunray_sim

#endif
