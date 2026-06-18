#ifndef SUNRAY_SIM_UGV_SIMULATOR_H
#define SUNRAY_SIM_UGV_SIMULATOR_H

#include "imu_model.h"
#include "ugv_dynamics.h"

#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <string>

namespace sunray_sim
{
class UgvSimulator
{
public:
    UgvSimulator(ros::NodeHandle& nh,
                 const std::string& agent_name,
                 int agent_id,
                 const Eigen::Vector3d& init_pos,
                 double init_yaw);

    void printStatus() const;

private:
    UgvDynamicParams loadDynamicParams() const;
    sunray_imu_sim::ImuModelConfig loadImuConfig() const;

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);
    void publishOdometry(const ros::Time& stamp);
    void publishImu(const ros::Time& stamp, double dt);

    static geometry_msgs::Quaternion yawToQuaternion(double yaw);

    ros::NodeHandle nh_;
    std::string agent_prefix_;
    std::string agent_frame_prefix_;
    std::string global_frame_id_{"map"};
    std::string base_frame_id_;
    std::string imu_frame_id_;
    std::string cmd_topic_;
    std::string odom_topic_;
    std::string imu_topic_;

    UgvDynamicParams dynamic_params_;
    UgvDynamics dynamics_;
    sunray_imu_sim::ImuModel imu_model_;
    double sim_time_{0.0};

    geometry_msgs::Twist last_cmd_;
    ros::Time last_cmd_time_{0};
    bool has_received_cmd_{false};

    ros::Subscriber cmd_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher imu_pub_;
    ros::Timer update_timer_;
    ros::Time last_update_time_;
    double publish_rate_{100.0};
};
}  // namespace sunray_sim

#endif
