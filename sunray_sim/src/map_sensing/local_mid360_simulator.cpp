#include "local_mid360_simulator.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace sunray_sim
{
namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;
constexpr int kDefaultIntensity = 199;
constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiTitle = "\033[1;36m";
constexpr const char* kAnsiGood = "\033[1;32m";
constexpr const char* kAnsiWarn = "\033[1;33m";
constexpr const char* kAnsiBad = "\033[1;31m";
constexpr double kRenderTimeWarnSec = 0.1;

Eigen::Quaterniond odomQuaternion(const nav_msgs::Odometry& odom)
{
    Eigen::Quaterniond q;
    q.x() = odom.pose.pose.orientation.x;
    q.y() = odom.pose.pose.orientation.y;
    q.z() = odom.pose.pose.orientation.z;
    q.w() = odom.pose.pose.orientation.w;
    return q.normalized();
}

Eigen::Vector3d odomPosition(const nav_msgs::Odometry& odom)
{
    return Eigen::Vector3d(odom.pose.pose.position.x,
                           odom.pose.pose.position.y,
                           odom.pose.pose.position.z);
}

Eigen::Matrix3d rpyToRotationMatrix(const double roll_rad,
                                    const double pitch_rad,
                                    const double yaw_rad)
{
    const Eigen::AngleAxisd roll(roll_rad, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(pitch_rad, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(yaw_rad, Eigen::Vector3d::UnitZ());
    return (yaw * pitch * roll).toRotationMatrix();
}

}  // namespace

LocalMid360Simulator::LocalMid360Simulator(ros::NodeHandle& nh,
                                           pcl::PointCloud<pcl::PointXYZI>::ConstPtr global_map,
                                           const std::string& agent_name,
                                           const int agent_id)
    : nh_(nh), agent_prefix_("/" + agent_name + std::to_string(std::max(agent_id, 1))), global_map_(std::move(global_map))
{
    agent_frame_prefix_ = agent_name + std::to_string(std::max(agent_id, 1));
    nh_.param<std::string>("lidar_type", lidar_type_, lidar_type_);
    nh_.param<std::string>("global_frame_id", global_frame_id_, global_frame_id_);
    sensor_frame_id_ = agent_frame_prefix_ + "/sensor";
    nh_.param<std::string>("sensor_frame_id", sensor_frame_id_, sensor_frame_id_);
    if (sensor_frame_id_.empty())
    {
        sensor_frame_id_ = agent_frame_prefix_ + "/sensor";
    }
    nh_.param<int>("is_360lidar", is_360lidar_, 1);
    nh_.param<double>("sensing_horizon", sensing_horizon_, 15.0);
    nh_.param<double>("sensing_rate", sensing_rate_, 10.0);
    nh_.param<double>("polar_resolution", polar_resolution_, 0.2);
    nh_.param<double>("yaw_fov", yaw_fov_, 360.0);
    nh_.param<double>("vertical_fov", vertical_fov_, 90.0);
    nh_.param<double>("min_raylength", min_raylength_, 1.0);
    nh_.param<double>("sensor_offset_x", sensor_offset_body_.x(), 0.0);
    nh_.param<double>("sensor_offset_y", sensor_offset_body_.y(), 0.0);
    nh_.param<double>("sensor_offset_z", sensor_offset_body_.z(), 0.0);
    nh_.param<double>("sensor_roll", sensor_rpy_deg_.x(), 0.0);
    nh_.param<double>("sensor_pitch", sensor_rpy_deg_.y(), 0.0);
    nh_.param<double>("sensor_yaw", sensor_rpy_deg_.z(), 0.0);
    nh_.param<bool>("collision_check/enable", collision_check_enable_, true);
    nh_.param<double>("collision_check/radius", collision_radius_, 0.15);
    nh_.param<bool>("collision_check/z_filter_enable", collision_z_filter_enable_, true);
    nh_.param<double>("collision_check/z_margin", collision_z_margin_, 0.0);

    sensing_horizon_ = std::max(0.1, sensing_horizon_);
    sensing_rate_ = std::max(0.1, sensing_rate_);
    polar_resolution_ = std::max(0.05, polar_resolution_);
    yaw_fov_ = clampValue(yaw_fov_, 1.0, 360.0);
    vertical_fov_ = clampValue(vertical_fov_, 1.0, 179.0);
    collision_radius_ = std::max(0.01, collision_radius_);
    collision_z_margin_ = std::max(0.0, collision_z_margin_);
    sensor_rotation_body_ = rpyToRotationMatrix(sensor_rpy_deg_.x() * kDegToRad,
                                                sensor_rpy_deg_.y() * kDegToRad,
                                                sensor_rpy_deg_.z() * kDegToRad);

    if (global_map_ && !global_map_->empty())
    {
        map_kdtree_.setInputCloud(global_map_);
    }

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(agent_prefix_ + "/sunray_sim/odom",
                                                  50,
                                                  &LocalMid360Simulator::odomCallback,
                                                  this);
    cloud_world_frame_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(agent_prefix_ + "/sunray_sim/cloud_world_frame", 10);
    cloud_sensor_frame_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(agent_prefix_ + "/sunray_sim/cloud_sensor_frame", 10);
    collision_pub_ = nh_.advertise<std_msgs::Bool>(agent_prefix_ + "/sunray_sim/collision", 10);

    render_timer_ = nh_.createTimer(ros::Duration(1.0 / sensing_rate_),
                                    &LocalMid360Simulator::renderTimerCallback,
                                    this);

    ROS_INFO("[sunray_sim] local %s simulator started for %s, map_points=%zu",
             lidar_type_.c_str(),
             agent_prefix_.c_str(),
             global_map_ ? global_map_->size() : 0);
}

void LocalMid360Simulator::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_ = *msg;
    has_odom_ = true;
}

double LocalMid360Simulator::clampValue(const double value, const double min_value, const double max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

double LocalMid360Simulator::wrap360(const double degrees)
{
    double wrapped = std::fmod(degrees, 360.0);
    if (wrapped < 0.0)
    {
        wrapped += 360.0;
    }
    return wrapped;
}

void LocalMid360Simulator::updateCollisionState(const Eigen::Vector3d& body_pos, const ros::Time& stamp)
{
    if (!collision_check_enable_)
    {
        if (in_collision_)
        {
            in_collision_ = false;
            std_msgs::Bool collision_msg;
            collision_msg.data = false;
            collision_pub_.publish(collision_msg);
        }
        return;
    }

    pcl::PointXYZI search_point;
    search_point.x = body_pos.x();
    search_point.y = body_pos.y();
    search_point.z = body_pos.z();

    std::vector<int> indices;
    std::vector<float> distances;
    map_kdtree_.radiusSearch(search_point, collision_radius_, indices, distances);

    bool detected = false;
    double nearest_distance = std::numeric_limits<double>::infinity();
    Eigen::Vector3d nearest_point = Eigen::Vector3d::Zero();
    const double z_range = collision_radius_ + collision_z_margin_;
    for (std::size_t i = 0; i < indices.size(); ++i)
    {
        const auto& map_pt = global_map_->points[indices[i]];
        if (collision_z_filter_enable_ && std::fabs(static_cast<double>(map_pt.z) - body_pos.z()) > z_range)
        {
            continue;
        }

        const double distance = std::sqrt(static_cast<double>(distances[i]));
        if (distance < nearest_distance)
        {
            nearest_distance = distance;
            nearest_point = Eigen::Vector3d(map_pt.x, map_pt.y, map_pt.z);
        }
        detected = true;
    }

    if (detected && !in_collision_)
    {
        ++collision_count_;
        nearest_collision_distance_ = nearest_distance;
        nearest_collision_point_ = nearest_point;
        ROS_ERROR("[sunray_sim] %s collision detected, count=%lu, distance=%.3f m, vehicle=(%.2f, %.2f, %.2f), obstacle=(%.2f, %.2f, %.2f)",
                  agent_prefix_.c_str(),
                  static_cast<unsigned long>(collision_count_),
                  nearest_collision_distance_,
                  body_pos.x(),
                  body_pos.y(),
                  body_pos.z(),
                  nearest_collision_point_.x(),
                  nearest_collision_point_.y(),
                  nearest_collision_point_.z());
    }
    else if (detected)
    {
        nearest_collision_distance_ = nearest_distance;
        nearest_collision_point_ = nearest_point;
    }

    in_collision_ = detected;
    std_msgs::Bool collision_msg;
    collision_msg.data = in_collision_;
    collision_pub_.publish(collision_msg);
}

void LocalMid360Simulator::renderTimerCallback(const ros::TimerEvent&)
{
    if (!has_odom_ || !global_map_ || global_map_->empty())
    {
        return;
    }

    const ros::WallTime render_begin = ros::WallTime::now();

    const Eigen::Vector3d body_pos = odomPosition(odom_);
    const Eigen::Quaterniond body_q = odomQuaternion(odom_);
    const Eigen::Matrix3d body_rot = body_q.toRotationMatrix();
    const Eigen::Vector3d sensor_pos = body_pos + body_rot * sensor_offset_body_;
    const Eigen::Matrix3d sensor_rot = body_rot * sensor_rotation_body_;
    const Eigen::Quaterniond sensor_q(sensor_rot);
    // 局部 MID360 按自己的 sensing_rate 定时输出。这里使用当前渲染时刻作为
    // 点云和 sensor TF 时间戳，避免在 odom 没有更新时重复发布同一个 TF stamp。
    const ros::Time stamp = ros::Time::now();

    updateCollisionState(body_pos, stamp);

    pcl::PointXYZI search_point;
    search_point.x = sensor_pos.x();
    search_point.y = sensor_pos.y();
    search_point.z = sensor_pos.z();

    std::vector<int> indices;
    std::vector<float> distances;
    map_kdtree_.radiusSearch(search_point, sensing_horizon_, indices, distances);

    const double effective_yaw_fov = (is_360lidar_ == 1) ? 360.0 : yaw_fov_;
    const int width = std::max(1, static_cast<int>(std::ceil(effective_yaw_fov / polar_resolution_)));
    const int height = std::max(1, static_cast<int>(std::ceil(vertical_fov_ / polar_resolution_)));
    std::vector<float> ranges(static_cast<std::size_t>(width * height),
                              std::numeric_limits<float>::infinity());

    for (const int index : indices)
    {
        const auto& map_pt = global_map_->points[index];
        const Eigen::Vector3d world_pt(map_pt.x, map_pt.y, map_pt.z);
        const Eigen::Vector3d sensor_pt = sensor_rot.transpose() * (world_pt - sensor_pos);
        const double range = sensor_pt.norm();
        if (range < min_raylength_ || range > sensing_horizon_)
        {
            continue;
        }

        const double yaw_deg = std::atan2(sensor_pt.y(), sensor_pt.x()) * kRadToDeg;
        const double elevation_deg =
            std::atan2(sensor_pt.z(), std::hypot(sensor_pt.x(), sensor_pt.y())) * kRadToDeg;
        if (std::fabs(elevation_deg) > vertical_fov_ * 0.5)
        {
            continue;
        }
        if (is_360lidar_ != 1 && std::fabs(yaw_deg) > yaw_fov_ * 0.5)
        {
            continue;
        }

        const double yaw_bin_deg = (is_360lidar_ == 1) ? wrap360(yaw_deg) : (yaw_deg + yaw_fov_ * 0.5);
        int col = static_cast<int>(std::floor(yaw_bin_deg / polar_resolution_));
        int row = static_cast<int>(std::floor((elevation_deg + vertical_fov_ * 0.5) / polar_resolution_));
        col = std::max(0, std::min(width - 1, col));
        row = std::max(0, std::min(height - 1, row));
        const std::size_t bin = static_cast<std::size_t>(row * width + col);
        if (range < ranges[bin])
        {
            ranges[bin] = static_cast<float>(range);
        }
    }

    pcl::PointCloud<pcl::PointXYZI> world_cloud;
    pcl::PointCloud<pcl::PointXYZI> sensor_cloud;
    world_cloud.reserve(ranges.size());
    sensor_cloud.reserve(ranges.size());

    for (int row = 0; row < height; ++row)
    {
        for (int col = 0; col < width; ++col)
        {
            const float range = ranges[static_cast<std::size_t>(row * width + col)];
            if (!std::isfinite(range))
            {
                continue;
            }

            double yaw_deg = (col + 0.5) * polar_resolution_;
            if (is_360lidar_ != 1)
            {
                yaw_deg -= yaw_fov_ * 0.5;
            }
            const double elevation_deg = (row + 0.5) * polar_resolution_ - vertical_fov_ * 0.5;
            const double yaw = yaw_deg * kDegToRad;
            const double elevation = elevation_deg * kDegToRad;
            const Eigen::Vector3d local(range * std::cos(elevation) * std::cos(yaw),
                                        range * std::cos(elevation) * std::sin(yaw),
                                        range * std::sin(elevation));
            const Eigen::Vector3d world = sensor_rot * local + sensor_pos;

            pcl::PointXYZI sensor_pt;
            sensor_pt.x = local.x();
            sensor_pt.y = local.y();
            sensor_pt.z = local.z();
            sensor_pt.intensity = kDefaultIntensity;
            sensor_cloud.push_back(sensor_pt);

            pcl::PointXYZI world_pt;
            world_pt.x = world.x();
            world_pt.y = world.y();
            world_pt.z = world.z();
            world_pt.intensity = kDefaultIntensity;
            world_cloud.push_back(world_pt);
        }
    }

    sensor_msgs::PointCloud2 world_msg;
    pcl::toROSMsg(world_cloud, world_msg);
    world_msg.header.stamp = stamp;
    world_msg.header.frame_id = global_frame_id_;
    cloud_world_frame_pub_.publish(world_msg);

    sensor_msgs::PointCloud2 sensor_msg;
    pcl::toROSMsg(sensor_cloud, sensor_msg);
    sensor_msg.header.stamp = stamp;
    sensor_msg.header.frame_id = sensor_frame_id_;
    cloud_sensor_frame_pub_.publish(sensor_msg);

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = global_frame_id_;
    transform.child_frame_id = sensor_frame_id_;
    transform.transform.translation.x = sensor_pos.x();
    transform.transform.translation.y = sensor_pos.y();
    transform.transform.translation.z = sensor_pos.z();
    transform.transform.rotation.x = sensor_q.x();
    transform.transform.rotation.y = sensor_q.y();
    transform.transform.rotation.z = sensor_q.z();
    transform.transform.rotation.w = sensor_q.w();
    tf_broadcaster_.sendTransform(transform);

    last_render_time_sec_ = (ros::WallTime::now() - render_begin).toSec();
    last_render_input_points_ = indices.size();
    last_render_output_points_ = world_cloud.size();
    has_render_stats_ = true;
}

void LocalMid360Simulator::printStatus() const
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    ss << kAnsiTitle << "=================== local_mid360_simulator [" << agent_prefix_
       << "] ===================" << kAnsiReset << "\n";

    ss << kAnsiGood << " 基本状态 " << kAnsiReset
       << "雷达类型 = " << lidar_type_
       << "  odom状态 = " << (has_odom_ ? kAnsiGood : kAnsiWarn) << (has_odom_ ? "正常" : "等待") << kAnsiReset
       << "  全局frame = " << global_frame_id_
       << "  地图点数 = " << (global_map_ ? global_map_->size() : 0)
       << "  发布频率 = " << sensing_rate_ << " Hz"
       << "\n";

    ss << kAnsiGood << " 雷达参数 " << kAnsiReset
       << "量程 = " << sensing_horizon_ << " m"
       << "  最小距离 = " << min_raylength_ << " m"
       << "  水平视场 = " << ((is_360lidar_ == 1) ? 360.0 : yaw_fov_) << " deg"
       << "  垂直视场 = " << vertical_fov_ << " deg"
       << "  分辨率 = " << polar_resolution_ << " deg"
       << "\n";

    ss << kAnsiGood << " 安装外参 " << kAnsiReset
       << "offset_body = (" << sensor_offset_body_.x() << ", "
       << sensor_offset_body_.y() << ", " << sensor_offset_body_.z() << ") m"
       << "  rpy_body = (" << sensor_rpy_deg_.x() << ", "
       << sensor_rpy_deg_.y() << ", " << sensor_rpy_deg_.z() << ") deg"
       << "\n";

    ss << kAnsiGood << " 渲染统计 " << kAnsiReset;
    if (has_render_stats_)
    {
        const bool render_slow = last_render_time_sec_ > kRenderTimeWarnSec;
        const char* render_color = render_slow ? kAnsiBad : kAnsiGood;
        ss << "耗时 = " << render_color << std::setprecision(3) << last_render_time_sec_ << " s" << kAnsiReset
           << std::setprecision(2)
           << "  输入点数 = " << last_render_input_points_
           << "  输出点数 = " << last_render_output_points_
           << "  阈值 = " << kRenderTimeWarnSec << " s";
    }
    else
    {
        ss << kAnsiWarn << "等待首帧渲染" << kAnsiReset
           << "  输入点数 = 0"
           << "  输出点数 = 0"
           << "  阈值 = " << kRenderTimeWarnSec << " s";
    }
    ss << "\n";

    ss << kAnsiGood << " 碰撞检测 " << kAnsiReset
       << "状态 = ";
    if (!collision_check_enable_)
    {
        ss << kAnsiWarn << "关闭" << kAnsiReset;
    }
    else
    {
        ss << (in_collision_ ? kAnsiBad : kAnsiGood)
           << (in_collision_ ? "碰撞" : "正常") << kAnsiReset
           << "  碰撞次数 = " << collision_count_
           << "  半径 = " << collision_radius_ << " m"
           << "  z过滤 = " << (collision_z_filter_enable_ ? "开启" : "关闭");
        if (in_collision_)
        {
            ss << "  最近距离 = " << nearest_collision_distance_ << " m";
        }
    }
    ss << "\n";

    ss << kAnsiGood << " 订阅话题 " << kAnsiReset
       << "里程计 -> " << agent_prefix_ << "/sunray_sim/odom"
       << "\n";

    ss << kAnsiGood << " 发布话题 " << kAnsiReset
       << "全局系点云 -> " << agent_prefix_ << "/sunray_sim/cloud_world_frame"
       << "\n"
       << "          传感器系点云 -> " << agent_prefix_ << "/sunray_sim/cloud_sensor_frame"
       << "\n"
       << "          碰撞状态 -> " << agent_prefix_ << "/sunray_sim/collision"
       << "\n"
       << "          TF -> " << global_frame_id_ << " -> " << sensor_frame_id_;

    std::cout << ss.str() << std::endl;
}
}  // namespace sunray_sim
