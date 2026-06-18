#include "sim_visualizer.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace sunray_sim
{
namespace
{
constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiTitle = "\033[1;36m";
constexpr const char* kAnsiGood = "\033[1;32m";
constexpr const char* kAnsiBad = "\033[1;31m";

void setMarkerColor(visualization_msgs::Marker& marker,
                    const double r,
                    const double g,
                    const double b,
                    const double a)
{
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

double vectorNorm(const geometry_msgs::Vector3& value)
{
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}
}  // namespace

SimVisualizer::SimVisualizer(ros::NodeHandle& nh,
                             const std::string& agent_name,
                             const int agent_id)
    : nh_(nh),
      agent_prefix_("/" + agent_name + std::to_string(std::max(agent_id, 1))),
      agent_name_(agent_name)
{
    std::string agent_name_lower = agent_name_;
    std::transform(agent_name_lower.begin(), agent_name_lower.end(), agent_name_lower.begin(), ::tolower);
    is_ugv_ = agent_name_lower.find("ugv") == 0;

    nh_.param<std::string>("global_frame_id", global_frame_id_, global_frame_id_);
    nh_.param<double>("visualizer/publish_rate", publish_rate_, 10.0);
    publish_rate_ = std::max(0.1, publish_rate_);
    nh_.param<double>("visualizer/marker_scale", marker_scale_, 1.0);
    marker_scale_ = std::max(0.1, marker_scale_);
    nh_.param<int>("visualizer/path_max_points", path_max_points_, 500);
    path_max_points_ = std::max(2, path_max_points_);
    nh_.param<double>("visualizer/path_min_interval", path_min_interval_, 0.15);
    path_min_interval_ = std::max(0.0, path_min_interval_);
    nh_.param<bool>("visualizer/show_velocity_arrow", show_velocity_arrow_, true);
    nh_.param<bool>("visualizer/show_status_text", show_status_text_, true);
    nh_.param<std::string>("visualizer/marker_topic",
                           marker_topic_,
                           agent_prefix_ + "/sunray_sim/visualization");
    if (marker_topic_.empty())
    {
        marker_topic_ = agent_prefix_ + "/sunray_sim/visualization";
    }

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        agent_prefix_ + "/sunray_sim/odom", 20, &SimVisualizer::odomCallback, this);
    if (!is_ugv_)
    {
        rpm_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>(
            agent_prefix_ + "/sunray_sim/cmd_RPM", 20, &SimVisualizer::rpmCallback, this);
        mavros_state_sub_ = nh_.subscribe<mavros_msgs::State>(
            agent_prefix_ + "/mavros/state", 20, &SimVisualizer::mavrosStateCallback, this);
    }
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        agent_prefix_ + "/sunray_sim/cloud_world_frame", 5, &SimVisualizer::cloudCallback, this);
    collision_sub_ = nh_.subscribe<std_msgs::Bool>(
        agent_prefix_ + "/sunray_sim/collision", 10, &SimVisualizer::collisionCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);

    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                     &SimVisualizer::publishTimerCallback,
                                     this);

    ROS_INFO("[sunray_sim] visualizer started for %s, marker_topic=%s",
             agent_prefix_.c_str(),
             marker_topic_.c_str());
}

void SimVisualizer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    latest_odom_ = *msg;
    last_odom_time_ = ros::Time::now();
    has_odom_ = true;

    const Eigen::Vector3d position(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
    if (!has_path_point_ || (position - last_path_point_).norm() >= path_min_interval_)
    {
        path_points_.push_back(position);
        last_path_point_ = position;
        has_path_point_ = true;
        while (static_cast<int>(path_points_.size()) > path_max_points_)
        {
            path_points_.erase(path_points_.begin());
        }
    }
}

void SimVisualizer::rpmCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    latest_rpm_ = *msg;
    last_rpm_time_ = ros::Time::now();
    has_rpm_ = true;
}

void SimVisualizer::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    latest_mavros_state_ = *msg;
    last_mavros_state_time_ = ros::Time::now();
    has_mavros_state_ = true;
}

void SimVisualizer::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    latest_cloud_points_ = msg->width * msg->height;
    last_cloud_time_ = ros::Time::now();
}

void SimVisualizer::collisionCallback(const std_msgs::Bool::ConstPtr& msg)
{
    in_collision_ = msg->data;
    last_collision_time_ = ros::Time::now();
}

void SimVisualizer::publishTimerCallback(const ros::TimerEvent&)
{
    publishMarkers(ros::Time::now());
}

geometry_msgs::Point SimVisualizer::makePoint(const Eigen::Vector3d& point) const
{
    geometry_msgs::Point ros_point;
    ros_point.x = point.x();
    ros_point.y = point.y();
    ros_point.z = point.z();
    return ros_point;
}

visualization_msgs::Marker SimVisualizer::makeMarker(const int id,
                                                     const std::string& ns,
                                                     const int type,
                                                     const ros::Time& stamp) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = has_odom_ && !latest_odom_.header.frame_id.empty()
                                 ? latest_odom_.header.frame_id
                                 : global_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(std::max(0.2, 2.0 / publish_rate_));
    marker.pose.orientation.w = 1.0;
    return marker;
}

Eigen::Matrix3d SimVisualizer::odomRotation() const
{
    const geometry_msgs::Quaternion& q_msg = latest_odom_.pose.pose.orientation;
    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    if (q.norm() < 1.0e-6)
    {
        return Eigen::Matrix3d::Identity();
    }
    return q.normalized().toRotationMatrix();
}

void SimVisualizer::publishMarkers(const ros::Time& stamp)
{
    if (!has_odom_)
    {
        return;
    }

    const Eigen::Vector3d position(latest_odom_.pose.pose.position.x,
                                   latest_odom_.pose.pose.position.y,
                                   latest_odom_.pose.pose.position.z);
    const Eigen::Matrix3d body_to_world = odomRotation();

    visualization_msgs::MarkerArray markers;

    if (is_ugv_)
    {
        appendUgvMarkers(markers, stamp, position, body_to_world);
    }
    else
    {
        appendUavMarkers(markers, stamp, position, body_to_world);
    }
    appendCommonMarkers(markers, stamp, position);

    marker_pub_.publish(markers);
}

void SimVisualizer::appendUavMarkers(visualization_msgs::MarkerArray& markers,
                                     const ros::Time& stamp,
                                     const Eigen::Vector3d& position,
                                     const Eigen::Matrix3d& body_to_world) const
{
    const double scale = marker_scale_;
    // UAV marker 按 200 mm 轴距建模。这里的 wheelbase 是对角电机间距，
    // 因此中心到任意电机的距离为 wheelbase / 2。
    const double wheelbase = 0.20 * scale;
    const double arm = wheelbase * 0.5;
    const double rotor_radius = 0.026 * scale;
    const double rotor_height = 0.010 * scale;
    const double body_radius = 0.030 * scale;
    const double arrow_length = 0.16 * scale;
    const double x_arm_offset = arm / std::sqrt(2.0);

    visualization_msgs::Marker body = makeMarker(0, "uav_body", visualization_msgs::Marker::SPHERE, stamp);
    body.pose.position = makePoint(position);
    body.pose.orientation = latest_odom_.pose.pose.orientation;
    body.scale.x = body_radius * 2.5;
    body.scale.y = body_radius * 1.7;
    body.scale.z = body_radius * 0.8;
    setMarkerColor(body, 0.08, 0.28, 0.95, 1.0);
    markers.markers.push_back(body);

    visualization_msgs::Marker arms = makeMarker(1, "uav_x_frame", visualization_msgs::Marker::LINE_LIST, stamp);
    arms.scale.x = 0.012 * scale;
    setMarkerColor(arms, 0.92, 0.95, 1.00, 1.0);
    const Eigen::Vector3d front_right = body_to_world * Eigen::Vector3d(x_arm_offset, -x_arm_offset, 0.0);
    const Eigen::Vector3d rear_left = body_to_world * Eigen::Vector3d(-x_arm_offset, x_arm_offset, 0.0);
    const Eigen::Vector3d front_left = body_to_world * Eigen::Vector3d(x_arm_offset, x_arm_offset, 0.0);
    const Eigen::Vector3d rear_right = body_to_world * Eigen::Vector3d(-x_arm_offset, -x_arm_offset, 0.0);
    arms.points.push_back(makePoint(position + front_right));
    arms.points.push_back(makePoint(position + rear_left));
    arms.points.push_back(makePoint(position + front_left));
    arms.points.push_back(makePoint(position + rear_right));
    markers.markers.push_back(arms);

    const std::array<Eigen::Vector3d, 4> rotor_offsets = {
        Eigen::Vector3d(x_arm_offset, -x_arm_offset, 0.0),
        Eigen::Vector3d(x_arm_offset, x_arm_offset, 0.0),
        Eigen::Vector3d(-x_arm_offset, x_arm_offset, 0.0),
        Eigen::Vector3d(-x_arm_offset, -x_arm_offset, 0.0)};
    for (std::size_t i = 0; i < rotor_offsets.size(); ++i)
    {
        visualization_msgs::Marker rotor =
            makeMarker(static_cast<int>(10 + i), "uav_rotors", visualization_msgs::Marker::CYLINDER, stamp);
        rotor.pose.position = makePoint(position + body_to_world * rotor_offsets[i]);
        rotor.pose.orientation = latest_odom_.pose.pose.orientation;
        rotor.scale.x = rotor_radius * 2.0;
        rotor.scale.y = rotor_radius * 2.0;
        rotor.scale.z = rotor_height;
        if (rotor_offsets[i].x() > 0.0)
        {
            setMarkerColor(rotor, 0.08, 0.95, 0.38, 1.0);
        }
        else
        {
            setMarkerColor(rotor, 1.00, 0.14, 0.10, 1.0);
        }
        markers.markers.push_back(rotor);
    }

    visualization_msgs::Marker heading =
        makeMarker(20, "uav_heading", visualization_msgs::Marker::ARROW, stamp);
    heading.points.push_back(makePoint(position));
    heading.points.push_back(makePoint(position + body_to_world * Eigen::Vector3d(arrow_length, 0.0, 0.0)));
    heading.scale.x = 0.012 * scale;
    heading.scale.y = 0.030 * scale;
    heading.scale.z = 0.030 * scale;
    setMarkerColor(heading, 0.10, 0.95, 0.35, 1.0);
    markers.markers.push_back(heading);
}

void SimVisualizer::appendUgvMarkers(visualization_msgs::MarkerArray& markers,
                                     const ros::Time& stamp,
                                     const Eigen::Vector3d& position,
                                     const Eigen::Matrix3d& body_to_world) const
{
    const double scale = marker_scale_;
    const double body_length = 0.25 * scale;
    const double body_width = 0.13 * scale;
    const double body_height = 0.06 * scale;
    const double wheel_length = 0.07 * scale;
    const double wheel_width = 0.022 * scale;
    const double wheel_height = 0.05 * scale;
    const double arrow_length = 0.19 * scale;
    const Eigen::Vector3d body_center = position + Eigen::Vector3d(0.0, 0.0, body_height * 0.5);

    visualization_msgs::Marker body = makeMarker(0, "ugv_body", visualization_msgs::Marker::CUBE, stamp);
    body.pose.position = makePoint(body_center);
    body.pose.orientation = latest_odom_.pose.pose.orientation;
    body.scale.x = body_length;
    body.scale.y = body_width;
    body.scale.z = body_height;
    setMarkerColor(body, 0.08, 0.28, 0.95, 1.0);
    markers.markers.push_back(body);

    visualization_msgs::Marker cabin = makeMarker(1, "ugv_cabin", visualization_msgs::Marker::CUBE, stamp);
    cabin.pose.position = makePoint(position + body_to_world * Eigen::Vector3d(0.03 * scale, 0.0, 0.0) +
                                    Eigen::Vector3d(0.0, 0.0, body_height + 0.03 * scale));
    cabin.pose.orientation = latest_odom_.pose.pose.orientation;
    cabin.scale.x = 0.11 * scale;
    cabin.scale.y = 0.11 * scale;
    cabin.scale.z = 0.06 * scale;
    setMarkerColor(cabin, 0.12, 0.62, 0.95, 1.0);
    markers.markers.push_back(cabin);

    const std::array<Eigen::Vector3d, 4> wheel_offsets = {
        Eigen::Vector3d(0.08 * scale, -0.085 * scale, wheel_height * 0.5),
        Eigen::Vector3d(0.08 * scale, 0.085 * scale, wheel_height * 0.5),
        Eigen::Vector3d(-0.08 * scale, -0.085 * scale, wheel_height * 0.5),
        Eigen::Vector3d(-0.08 * scale, 0.085 * scale, wheel_height * 0.5)};
    for (std::size_t i = 0; i < wheel_offsets.size(); ++i)
    {
        visualization_msgs::Marker wheel =
            makeMarker(static_cast<int>(10 + i), "ugv_wheels", visualization_msgs::Marker::CUBE, stamp);
        wheel.pose.position = makePoint(position + body_to_world * wheel_offsets[i]);
        wheel.pose.orientation = latest_odom_.pose.pose.orientation;
        wheel.scale.x = wheel_length;
        wheel.scale.y = wheel_width;
        wheel.scale.z = wheel_height;
        setMarkerColor(wheel, 0.03, 0.04, 0.05, 1.0);
        markers.markers.push_back(wheel);
    }

    visualization_msgs::Marker front =
        makeMarker(20, "ugv_heading", visualization_msgs::Marker::ARROW, stamp);
    const Eigen::Vector3d arrow_start = position + Eigen::Vector3d(0.0, 0.0, body_height + 0.06 * scale);
    front.points.push_back(makePoint(arrow_start));
    front.points.push_back(makePoint(arrow_start + body_to_world * Eigen::Vector3d(arrow_length, 0.0, 0.0)));
    front.scale.x = 0.014 * scale;
    front.scale.y = 0.038 * scale;
    front.scale.z = 0.038 * scale;
    setMarkerColor(front, 0.10, 0.95, 0.35, 1.0);
    markers.markers.push_back(front);

    visualization_msgs::Marker front_bar =
        makeMarker(22, "ugv_front_bar", visualization_msgs::Marker::CUBE, stamp);
    front_bar.pose.position = makePoint(position + body_to_world * Eigen::Vector3d(0.13 * scale, 0.0, body_height * 0.62));
    front_bar.pose.orientation = latest_odom_.pose.pose.orientation;
    front_bar.scale.x = 0.011 * scale;
    front_bar.scale.y = body_width * 0.92;
    front_bar.scale.z = 0.019 * scale;
    setMarkerColor(front_bar, 0.08, 0.95, 0.38, 1.0);
    markers.markers.push_back(front_bar);
}

void SimVisualizer::appendCommonMarkers(visualization_msgs::MarkerArray& markers,
                                        const ros::Time& stamp,
                                        const Eigen::Vector3d& position) const
{
    const double scale = marker_scale_;

    if (show_velocity_arrow_)
    {
        const geometry_msgs::Vector3& velocity = latest_odom_.twist.twist.linear;
        const Eigen::Vector3d vel(velocity.x, velocity.y, velocity.z);
        if (vel.norm() > 0.05)
        {
            visualization_msgs::Marker vel_arrow =
                makeMarker(21, is_ugv_ ? "ugv_velocity" : "uav_velocity", visualization_msgs::Marker::ARROW, stamp);
            vel_arrow.points.push_back(makePoint(position));
            vel_arrow.points.push_back(makePoint(position + vel.normalized() * std::min(vel.norm(), 2.0) * 0.35 * scale));
            vel_arrow.scale.x = 0.030 * scale;
            vel_arrow.scale.y = 0.075 * scale;
            vel_arrow.scale.z = 0.075 * scale;
            setMarkerColor(vel_arrow, 0.10, 0.78, 1.00, 1.0);
            markers.markers.push_back(vel_arrow);
        }
    }

    if (path_points_.size() >= 2)
    {
        visualization_msgs::Marker path = makeMarker(30, is_ugv_ ? "ugv_path" : "uav_path", visualization_msgs::Marker::LINE_STRIP, stamp);
        path.scale.x = 0.030 * scale;
        setMarkerColor(path, 1.00, 0.82, 0.12, 1.0);
        path.points.reserve(path_points_.size());
        for (const Eigen::Vector3d& point : path_points_)
        {
            path.points.push_back(makePoint(point));
        }
        markers.markers.push_back(path);
    }

    if (show_status_text_)
    {
        visualization_msgs::Marker text =
            makeMarker(40, is_ugv_ ? "ugv_status_text" : "uav_status_text", visualization_msgs::Marker::TEXT_VIEW_FACING, stamp);
        text.pose.position = makePoint(position + Eigen::Vector3d(0.0, 0.0, (is_ugv_ ? 0.22 : 0.30) * scale));
        text.scale.z = (is_ugv_ ? 0.08 : 0.10) * scale;
        std::ostringstream text_stream;
        text_stream << std::fixed << std::setprecision(2)
                    << agent_prefix_ << "  "
                    << "pos(" << position.x() << ", " << position.y() << ", " << position.z() << ")"
                    << "  v=" << vectorNorm(latest_odom_.twist.twist.linear) << "m/s";
        text.text = text_stream.str();
        setMarkerColor(text, 0.05, 1.00, 0.30, 1.0);
        markers.markers.push_back(text);
    }

    if (in_collision_)
    {
        visualization_msgs::Marker collision_text =
            makeMarker(41, is_ugv_ ? "ugv_collision_text" : "uav_collision_text", visualization_msgs::Marker::TEXT_VIEW_FACING, stamp);
        collision_text.pose.position =
            makePoint(position + Eigen::Vector3d(0.0, 0.0, (is_ugv_ ? 0.33 : 0.48) * scale));
        collision_text.scale.z = (is_ugv_ ? 0.11 : 0.16) * scale;
        collision_text.text = "碰撞";
        setMarkerColor(collision_text, 1.00, 0.05, 0.02, 1.0);
        markers.markers.push_back(collision_text);
    }
}

void SimVisualizer::printStatus() const
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << kAnsiTitle << "=================== sim_visualizer [" << agent_prefix_
       << "] ===================" << kAnsiReset << "\n";
    ss << kAnsiGood << " 发布话题 " << kAnsiReset
       << "MarkerArray -> " << marker_topic_;
    if (in_collision_)
    {
        ss << "\n" << kAnsiBad << " 碰撞状态 " << kAnsiReset << "检测到碰撞";
    }

    std::cout << ss.str() << std::endl;
}
}  // namespace sunray_sim
