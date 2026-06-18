#ifndef SUNRAY_SIM_LOCAL_MID360_SIMULATOR_H
#define SUNRAY_SIM_LOCAL_MID360_SIMULATOR_H

#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>
#include <vector>

namespace sunray_sim
{
class LocalMid360Simulator
{
public:
    LocalMid360Simulator(ros::NodeHandle& nh,
                         pcl::PointCloud<pcl::PointXYZI>::ConstPtr global_map,
                         const std::string& agent_name,
                         int agent_id);
    void printStatus() const;

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void renderTimerCallback(const ros::TimerEvent& event);

    static double clampValue(double value, double min_value, double max_value);
    static double wrap360(double degrees);

    ros::NodeHandle nh_;
    std::string agent_prefix_;
    std::string agent_frame_prefix_;
    std::string lidar_type_{"mid360"};
    std::string global_frame_id_{"map"};
    std::string sensor_frame_id_;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr global_map_;
    pcl::KdTreeFLANN<pcl::PointXYZI> map_kdtree_;

    ros::Subscriber odom_sub_;
    ros::Publisher cloud_world_frame_pub_;
    ros::Publisher cloud_sensor_frame_pub_;
    ros::Timer render_timer_;
    mutable tf2_ros::TransformBroadcaster tf_broadcaster_;

    nav_msgs::Odometry odom_;
    bool has_odom_{false};

    int is_360lidar_{1};
    double sensing_horizon_{15.0};
    double sensing_rate_{10.0};
    double polar_resolution_{0.2};
    double yaw_fov_{360.0};
    double vertical_fov_{90.0};
    double min_raylength_{1.0};
    Eigen::Vector3d sensor_offset_body_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d sensor_rpy_deg_{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d sensor_rotation_body_{Eigen::Matrix3d::Identity()};

    bool has_render_stats_{false};
    double last_render_time_sec_{0.0};
    std::size_t last_render_input_points_{0};
    std::size_t last_render_output_points_{0};
};
}  // namespace sunray_sim

#endif
