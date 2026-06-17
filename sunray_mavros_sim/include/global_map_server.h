#ifndef SUNRAY_MAVROS_SIM_GLOBAL_MAP_SERVER_H
#define SUNRAY_MAVROS_SIM_GLOBAL_MAP_SERVER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

namespace sunray_mavros_sim
{
class GlobalMapServer
{
public:
    explicit GlobalMapServer(ros::NodeHandle& nh);

    bool ready() const { return ready_; }
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud() const { return cloud_; }
    std::string topic() const { return global_map_topic_; }
    void printStatus() const;

private:
    void publishTimerCallback(const ros::TimerEvent& event);
    bool loadMap();
    void addBoundary(pcl::PointCloud<pcl::PointXYZI>& cloud) const;
    void publishMap(const ros::Time& stamp);

    ros::NodeHandle nh_;
    ros::Publisher global_map_pub_;
    ros::Timer publish_timer_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
    sensor_msgs::PointCloud2 map_msg_;

    bool ready_{false};
    std::string map_name_;
    std::string global_map_topic_{"/map_generator/global_cloud"};
    std::string global_frame_id_{"map"};
    int add_boundary_{0};
    double downsample_res_{0.1};
    double map_offset_x_{0.0};
    double map_offset_y_{0.0};
    double map_offset_z_{0.0};
    double map_publish_rate_{1.0};
};
}  // namespace sunray_mavros_sim

#endif
