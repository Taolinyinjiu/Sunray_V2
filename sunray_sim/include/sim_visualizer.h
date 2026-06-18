#ifndef SUNRAY_SIM_SIM_VISUALIZER_H
#define SUNRAY_SIM_SIM_VISUALIZER_H

#include <Eigen/Dense>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

namespace sunray_sim
{
class SimVisualizer
{
public:
    SimVisualizer(ros::NodeHandle& nh, const std::string& agent_name, int agent_id);

    void printStatus() const;

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void rpmCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void collisionCallback(const std_msgs::Bool::ConstPtr& msg);
    void publishTimerCallback(const ros::TimerEvent& event);
    void publishMarkers(const ros::Time& stamp);
    void appendUavMarkers(visualization_msgs::MarkerArray& markers,
                          const ros::Time& stamp,
                          const Eigen::Vector3d& position,
                          const Eigen::Matrix3d& body_to_world) const;
    void appendUgvMarkers(visualization_msgs::MarkerArray& markers,
                          const ros::Time& stamp,
                          const Eigen::Vector3d& position,
                          const Eigen::Matrix3d& body_to_world) const;
    void appendCommonMarkers(visualization_msgs::MarkerArray& markers,
                             const ros::Time& stamp,
                             const Eigen::Vector3d& position) const;

    visualization_msgs::Marker makeMarker(int id, const std::string& ns, int type, const ros::Time& stamp) const;
    geometry_msgs::Point makePoint(const Eigen::Vector3d& point) const;
    Eigen::Matrix3d odomRotation() const;

    ros::NodeHandle nh_;
    std::string agent_prefix_;
    std::string agent_name_;
    bool is_ugv_{false};
    std::string global_frame_id_{"map"};
    std::string marker_topic_;

    ros::Subscriber odom_sub_;
    ros::Subscriber rpm_sub_;
    ros::Subscriber mavros_state_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber collision_sub_;
    ros::Publisher marker_pub_;
    ros::Timer publish_timer_;

    nav_msgs::Odometry latest_odom_;
    mavros_msgs::State latest_mavros_state_;
    std_msgs::Float32MultiArray latest_rpm_;
    ros::Time last_odom_time_;
    ros::Time last_rpm_time_;
    ros::Time last_mavros_state_time_;
    ros::Time last_cloud_time_;
    ros::Time last_collision_time_;
    bool has_odom_{false};
    bool has_rpm_{false};
    bool has_mavros_state_{false};
    bool in_collision_{false};
    uint32_t latest_cloud_points_{0};

    std::vector<Eigen::Vector3d> path_points_;
    Eigen::Vector3d last_path_point_{Eigen::Vector3d::Zero()};
    bool has_path_point_{false};

    double publish_rate_{10.0};
    double marker_scale_{1.0};
    double path_min_interval_{0.15};
    int path_max_points_{500};
    bool show_velocity_arrow_{true};
    bool show_status_text_{true};
};
}  // namespace sunray_sim

#endif
