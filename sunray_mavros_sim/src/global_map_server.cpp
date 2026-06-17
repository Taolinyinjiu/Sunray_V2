#include "global_map_server.h"

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace sunray_mavros_sim
{
namespace
{
constexpr int kDefaultIntensity = 199;
constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiTitle = "\033[1;34m";
constexpr const char* kAnsiGood = "\033[1;32m";
constexpr const char* kAnsiWarn = "\033[1;33m";

std::string defaultMapPath()
{
    const std::string package_path = ros::package::getPath("sunray_mavros_sim");
    if (!package_path.empty())
    {
        return package_path + "/resource/small_forest01cutoff.pcd";
    }
    return std::string();
}
}  // namespace

GlobalMapServer::GlobalMapServer(ros::NodeHandle& nh) : nh_(nh), cloud_(new pcl::PointCloud<pcl::PointXYZI>)
{
    nh_.param<std::string>("map_name", map_name_, defaultMapPath());
    nh_.param<std::string>("global_map_topic", global_map_topic_, global_map_topic_);
    if (!nh_.getParam("global_frame_id", global_frame_id_))
    {
        nh_.param<std::string>("map_frame", global_frame_id_, global_frame_id_);
    }
    nh_.param<int>("add_boundary", add_boundary_, 0);
    nh_.param<double>("downsample_res", downsample_res_, 0.1);
    nh_.param<double>("map_offset_x", map_offset_x_, 0.0);
    nh_.param<double>("map_offset_y", map_offset_y_, 0.0);
    nh_.param<double>("map_offset_z", map_offset_z_, 0.0);
    nh_.param<double>("map_publish_rate", map_publish_rate_, 1.0);
    map_publish_rate_ = std::max(0.1, map_publish_rate_);

    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(global_map_topic_, 1, true);
    ready_ = loadMap();
    if (ready_)
    {
        publishMap(ros::Time::now());
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / map_publish_rate_),
                                         &GlobalMapServer::publishTimerCallback,
                                         this);
    }
}

bool GlobalMapServer::loadMap()
{
    if (map_name_.empty())
    {
        ROS_ERROR("[sunray_mavros_sim] map_name is empty");
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_name_, raw_cloud) < 0)
    {
        ROS_ERROR("[sunray_mavros_sim] cannot read PCD map: %s", map_name_.c_str());
        return false;
    }

    cloud_->clear();
    cloud_->reserve(raw_cloud.size());
    for (const auto& pt : raw_cloud.points)
    {
        pcl::PointXYZI out;
        out.x = pt.x;
        out.y = pt.y;
        out.z = pt.z;
        out.x += map_offset_x_;
        out.y += map_offset_y_;
        out.z += map_offset_z_;
        out.intensity = kDefaultIntensity;
        cloud_->push_back(out);
    }

    if (add_boundary_ == 1)
    {
        addBoundary(*cloud_);
    }

    if (downsample_res_ > 1.0e-4)
    {
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setLeafSize(downsample_res_, downsample_res_, downsample_res_);
        voxel.setInputCloud(cloud_);
        pcl::PointCloud<pcl::PointXYZI> filtered;
        voxel.filter(filtered);
        *cloud_ = filtered;
    }

    pcl::PointXYZI min_pt;
    pcl::PointXYZI max_pt;
    pcl::getMinMax3D(*cloud_, min_pt, max_pt);
    ROS_INFO("[sunray_mavros_sim] loaded map %s, points=%zu, bound x=(%.2f, %.2f), y=(%.2f, %.2f), z=(%.2f, %.2f)",
             map_name_.c_str(),
             cloud_->size(),
             min_pt.x,
             max_pt.x,
             min_pt.y,
             max_pt.y,
             min_pt.z,
             max_pt.z);

    pcl::toROSMsg(*cloud_, map_msg_);
    map_msg_.header.frame_id = global_frame_id_;
    return true;
}

void GlobalMapServer::addBoundary(pcl::PointCloud<pcl::PointXYZI>& cloud) const
{
    if (cloud.empty())
    {
        return;
    }

    pcl::PointXYZI min_pt;
    pcl::PointXYZI max_pt;
    pcl::getMinMax3D(cloud, min_pt, max_pt);

    const float step = 0.1f;
    const float min_x = std::floor(min_pt.x) - 1.0f;
    const float max_x = std::ceil(max_pt.x) + 1.0f;
    const float min_y = std::floor(min_pt.y) - 1.0f;
    const float max_y = std::ceil(max_pt.y) + 1.0f;
    const float min_z = std::floor(min_pt.z) - 1.0f;
    const float max_z = std::ceil(max_pt.z) + 1.0f;

    auto push_point = [&cloud](float x, float y, float z) {
        pcl::PointXYZI pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = kDefaultIntensity;
        cloud.push_back(pt);
    };

    for (float x = min_x; x <= max_x; x += step)
    {
        for (float y = min_y; y <= max_y; y += step)
        {
            push_point(x, y, min_z);
            push_point(x, y, max_z);
        }
    }
    for (float z = min_z; z <= max_z; z += step)
    {
        for (float x = min_x; x <= max_x; x += step)
        {
            push_point(x, min_y, z);
            push_point(x, max_y, z);
        }
        for (float y = min_y; y <= max_y; y += step)
        {
            push_point(min_x, y, z);
            push_point(max_x, y, z);
        }
    }
}

void GlobalMapServer::publishTimerCallback(const ros::TimerEvent&)
{
    publishMap(ros::Time::now());
}

void GlobalMapServer::publishMap(const ros::Time& stamp)
{
    if (!ready_)
    {
        return;
    }
    map_msg_.header.stamp = stamp;
    global_map_pub_.publish(map_msg_);
}

void GlobalMapServer::printStatus() const
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    ss << kAnsiTitle << "=================== global_map_server ===================" << kAnsiReset << "\n";
    ss << kAnsiGood << " 基本状态 " << kAnsiReset
       << "地图状态 = " << (ready_ ? kAnsiGood : kAnsiWarn) << (ready_ ? "正常" : "异常") << kAnsiReset
       << "  点数 = " << (cloud_ ? cloud_->size() : 0)
       << "  frame = " << global_frame_id_
       << "  发布频率 = " << map_publish_rate_ << " Hz"
       << "\n";

    ss << kAnsiGood << " 输入数据 " << kAnsiReset
       << "PCD文件 -> " << map_name_
       << "\n";

    ss << kAnsiGood << " 发布话题 " << kAnsiReset
       << "全局点云 -> " << global_map_topic_
       << "\n";

    ss << kAnsiGood << " 地图处理 " << kAnsiReset
       << "降采样 = " << downsample_res_ << " m"
       << "  边界补点 = " << (add_boundary_ == 1 ? "开启" : "关闭")
       << "  offset = (" << map_offset_x_ << ", " << map_offset_y_ << ", " << map_offset_z_ << ") m";

    std::cout << ss.str() << std::endl;
}
}  // namespace sunray_mavros_sim
