/*
本程序功能：
    1. 订阅 localization_fusion 发布的 OdomState
    2. 仅根据 OdomState 中的 odom 与 TF 快照发布 RViz MarkerArray
    3. 显示 world/global/local/base 坐标系、local/global odom、速度、轨迹和定位状态
*/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <sunray_msgs/OdomState.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <sstream>
#include <string>

namespace
{
ros::Publisher g_marker_pub;

std::string g_frame_id = "world";
double g_world_axis_length = 1.2;
double g_tf_axis_length = 0.45;
double g_axis_text_height = 0.16;
double g_odom_text_height = 0.18;
int g_trail_length = 50;

std::deque<geometry_msgs::Point> g_local_trail;
std::deque<geometry_msgs::Point> g_global_trail;

geometry_msgs::Point makePoint(const double x, const double y, const double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std_msgs::ColorRGBA makeColor(const double r, const double g, const double b, const double a)
{
    std_msgs::ColorRGBA color;
    color.r = static_cast<float>(r);
    color.g = static_cast<float>(g);
    color.b = static_cast<float>(b);
    color.a = static_cast<float>(a);
    return color;
}

visualization_msgs::Marker makeMarker(const int id, const std::string &ns, const int type)
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = g_frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.3);
    marker.pose.orientation.w = 1.0;
    return marker;
}

bool validQuaternion(const geometry_msgs::Quaternion &q)
{
    const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
           std::isfinite(q.w) && norm_sq > 1e-6;
}

bool validTransform(const geometry_msgs::TransformStamped &tf)
{
    const geometry_msgs::Vector3 &t = tf.transform.translation;
    return !tf.header.frame_id.empty() && !tf.child_frame_id.empty() &&
           std::isfinite(t.x) && std::isfinite(t.y) && std::isfinite(t.z) &&
           validQuaternion(tf.transform.rotation);
}

tf2::Transform identityTf()
{
    tf2::Transform tf;
    tf.setIdentity();
    return tf;
}

tf2::Transform toTf2OrIdentity(const geometry_msgs::TransformStamped &msg)
{
    if (!validTransform(msg))
    {
        return identityTf();
    }
    tf2::Transform tf;
    tf2::fromMsg(msg.transform, tf);
    return tf;
}

geometry_msgs::Point originOf(const tf2::Transform &tf)
{
    const tf2::Vector3 &o = tf.getOrigin();
    return makePoint(o.x(), o.y(), o.z());
}

geometry_msgs::Point axisEnd(const tf2::Transform &tf, const tf2::Vector3 &axis, const double length)
{
    const tf2::Vector3 end = tf.getOrigin() + tf.getBasis() * axis * length;
    return makePoint(end.x(), end.y(), end.z());
}

std::string frameLabel(const std::string &frame_id, const std::string &fallback)
{
    return frame_id.empty() ? fallback : frame_id;
}

std::string sourceName(const uint8_t source)
{
    switch (source)
    {
    case sunray_msgs::OdomState::VIOBOT:
        return "VIOBOT";
    case sunray_msgs::OdomState::MOCAP:
        return "MOCAP";
    case sunray_msgs::OdomState::VINS:
        return "VINS";
    case sunray_msgs::OdomState::GAZEBO:
        return "GAZEBO";
    case sunray_msgs::OdomState::GAZEBO_ARUCO:
        return "GAZEBO_ARUCO";
    case sunray_msgs::OdomState::PENGYU_SIM:
        return "PENGYU_SIM";
    case sunray_msgs::OdomState::FASTLIO_EFK:
        return "FASTLIO_EKF";
    default:
        return "UNKNOWN";
    }
}

void appendWorldAxis(visualization_msgs::MarkerArray &markers, int &marker_id)
{
    visualization_msgs::Marker origin = makeMarker(marker_id++, "fusion_world_origin", visualization_msgs::Marker::SPHERE);
    origin.pose.position = makePoint(0.0, 0.0, 0.0);
    origin.scale.x = 0.08;
    origin.scale.y = 0.08;
    origin.scale.z = 0.08;
    origin.color = makeColor(1.0, 1.0, 1.0, 0.95);
    markers.markers.push_back(origin);

    const tf2::Transform world_tf = identityTf();
    const std::string label = "world";

    visualization_msgs::Marker x_axis = makeMarker(marker_id++, "fusion_world_axis", visualization_msgs::Marker::ARROW);
    x_axis.points.push_back(originOf(world_tf));
    x_axis.points.push_back(axisEnd(world_tf, tf2::Vector3(1.0, 0.0, 0.0), g_world_axis_length));
    x_axis.scale.x = 0.035;
    x_axis.scale.y = 0.075;
    x_axis.scale.z = 0.10;
    x_axis.color = makeColor(0.95, 0.18, 0.18, 0.95);
    markers.markers.push_back(x_axis);

    visualization_msgs::Marker y_axis = x_axis;
    y_axis.id = marker_id++;
    y_axis.points[1] = axisEnd(world_tf, tf2::Vector3(0.0, 1.0, 0.0), g_world_axis_length);
    y_axis.color = makeColor(0.12, 0.85, 0.22, 0.95);
    markers.markers.push_back(y_axis);

    visualization_msgs::Marker z_axis = x_axis;
    z_axis.id = marker_id++;
    z_axis.points[1] = axisEnd(world_tf, tf2::Vector3(0.0, 0.0, 1.0), g_world_axis_length);
    z_axis.color = makeColor(0.18, 0.45, 1.0, 0.95);
    markers.markers.push_back(z_axis);

    visualization_msgs::Marker text = makeMarker(marker_id++, "fusion_world_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    text.pose.position = makePoint(0.0, 0.0, 0.18);
    text.scale.z = g_axis_text_height;
    text.color = makeColor(1.0, 1.0, 1.0, 1.0);
    text.text = label + "(O)";
    markers.markers.push_back(text);
}

void appendFrameAxis(visualization_msgs::MarkerArray &markers,
                     int &marker_id,
                     const tf2::Transform &tf,
                     const std::string &label,
                     const double axis_length,
                     const double z_text_offset)
{
    visualization_msgs::Marker x_axis = makeMarker(marker_id++, "fusion_tf_axis", visualization_msgs::Marker::ARROW);
    x_axis.points.push_back(originOf(tf));
    x_axis.points.push_back(axisEnd(tf, tf2::Vector3(1.0, 0.0, 0.0), axis_length));
    x_axis.scale.x = 0.025;
    x_axis.scale.y = 0.055;
    x_axis.scale.z = 0.08;
    x_axis.color = makeColor(0.95, 0.18, 0.18, 0.9);
    markers.markers.push_back(x_axis);

    visualization_msgs::Marker y_axis = x_axis;
    y_axis.id = marker_id++;
    y_axis.points[1] = axisEnd(tf, tf2::Vector3(0.0, 1.0, 0.0), axis_length);
    y_axis.color = makeColor(0.12, 0.85, 0.22, 0.9);
    markers.markers.push_back(y_axis);

    visualization_msgs::Marker z_axis = x_axis;
    z_axis.id = marker_id++;
    z_axis.points[1] = axisEnd(tf, tf2::Vector3(0.0, 0.0, 1.0), axis_length);
    z_axis.color = makeColor(0.18, 0.45, 1.0, 0.9);
    markers.markers.push_back(z_axis);

    visualization_msgs::Marker text = makeMarker(marker_id++, "fusion_tf_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    text.pose.position = originOf(tf);
    text.pose.position.z += z_text_offset;
    text.scale.z = g_axis_text_height;
    text.color = makeColor(0.95, 0.95, 0.95, 1.0);
    text.text = label;
    markers.markers.push_back(text);
}

void appendTfTreeLine(visualization_msgs::MarkerArray &markers,
                      int &marker_id,
                      const tf2::Transform &world_to_global,
                      const tf2::Transform &world_to_local,
                      const tf2::Transform &world_to_base,
                      const bool has_base)
{
    visualization_msgs::Marker line = makeMarker(marker_id++, "fusion_tf_tree", visualization_msgs::Marker::LINE_STRIP);
    line.scale.x = 0.018;
    line.color = makeColor(0.9, 0.9, 0.9, 0.55);
    line.points.push_back(makePoint(0.0, 0.0, 0.0));
    line.points.push_back(originOf(world_to_global));
    line.points.push_back(originOf(world_to_local));
    if (has_base)
    {
        line.points.push_back(originOf(world_to_base));
    }
    markers.markers.push_back(line);
}

void pushTrail(std::deque<geometry_msgs::Point> &trail, const geometry_msgs::Point &point)
{
    trail.push_back(point);
    while (static_cast<int>(trail.size()) > std::max(2, g_trail_length))
    {
        trail.pop_front();
    }
}

void appendOdomMarkers(visualization_msgs::MarkerArray &markers,
                       int &marker_id,
                       const nav_msgs::Odometry &odom,
                       const std::string &label,
                       const std_msgs::ColorRGBA &color,
                       std::deque<geometry_msgs::Point> &trail)
{
    if (odom.header.stamp.isZero())
    {
        return;
    }

    const geometry_msgs::Point &pos = odom.pose.pose.position;
    pushTrail(trail, pos);

    visualization_msgs::Marker body = makeMarker(marker_id++, "fusion_odom_body", visualization_msgs::Marker::SPHERE);
    body.pose.position = pos;
    body.scale.x = 0.16;
    body.scale.y = 0.16;
    body.scale.z = 0.16;
    body.color = color;
    markers.markers.push_back(body);

    visualization_msgs::Marker text = makeMarker(marker_id++, "fusion_odom_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    text.pose.position = pos;
    text.pose.position.z += 0.35;
    text.scale.z = g_odom_text_height;
    text.color = color;
    std::ostringstream text_ss;
    text_ss << label << "\n"
            << odom.header.frame_id << " -> "
            << (odom.child_frame_id.empty() ? "base" : odom.child_frame_id);
    text.text = text_ss.str();
    markers.markers.push_back(text);

    const geometry_msgs::Vector3 &vel = odom.twist.twist.linear;
    const double speed_sq = vel.x * vel.x + vel.y * vel.y + vel.z * vel.z;
    if (speed_sq > 1e-6)
    {
        visualization_msgs::Marker arrow = makeMarker(marker_id++, "fusion_odom_velocity", visualization_msgs::Marker::ARROW);
        arrow.points.push_back(pos);
        arrow.points.push_back(makePoint(pos.x + vel.x, pos.y + vel.y, pos.z + vel.z));
        arrow.scale.x = 0.035;
        arrow.scale.y = 0.08;
        arrow.scale.z = 0.10;
        arrow.color = makeColor(color.r, color.g, color.b, 0.95);
        markers.markers.push_back(arrow);
    }

    visualization_msgs::Marker vel_text = makeMarker(marker_id++, "fusion_odom_velocity_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    vel_text.pose.position = pos;
    vel_text.pose.position.z += 0.58;
    vel_text.scale.z = 0.14;
    vel_text.color = makeColor(color.r, color.g, color.b, 1.0);
    std::ostringstream vel_ss;
    vel_ss << "v=(" << std::fixed << std::setprecision(2)
           << vel.x << ", " << vel.y << ", " << vel.z << ") m/s";
    vel_text.text = vel_ss.str();
    markers.markers.push_back(vel_text);

    if (trail.size() >= 2)
    {
        visualization_msgs::Marker trail_marker = makeMarker(marker_id++, "fusion_odom_trail", visualization_msgs::Marker::LINE_STRIP);
        trail_marker.scale.x = 0.018;
        trail_marker.color = makeColor(color.r, color.g, color.b, 0.75);
        trail_marker.points.assign(trail.begin(), trail.end());
        markers.markers.push_back(trail_marker);
    }
}

void appendStatusText(visualization_msgs::MarkerArray &markers,
                      int &marker_id,
                      const sunray_msgs::OdomState &state,
                      const tf2::Transform &world_to_base,
                      const bool has_base)
{
    geometry_msgs::Point anchor = has_base ? originOf(world_to_base) : makePoint(0.0, 0.0, 0.0);
    visualization_msgs::Marker text = makeMarker(marker_id++, "fusion_status_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    text.pose.position = anchor;
    text.pose.position.z += has_base ? 0.85 : 0.55;
    text.scale.z = 0.18;
    text.color = state.odometry_valid ? makeColor(0.70, 1.0, 0.70, 1.0)
                                      : makeColor(1.0, 0.25, 0.20, 1.0);

    std::ostringstream ss;
    ss << "source=" << sourceName(state.external_source)
       << "\nodom=" << (state.odometry_valid ? "OK" : "BAD")
       << "  relocal=" << (state.relocalization_valid ? "OK" : "BAD")
       << "  hz=" << std::fixed << std::setprecision(1) << state.odometry_update_hz;
    text.text = ss.str();
    markers.markers.push_back(text);
}

void stateCallback(const sunray_msgs::OdomState::ConstPtr &msg)
{
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 1;
    appendWorldAxis(markers, marker_id);

    const bool has_world_to_global = validTransform(msg->world_to_global_tf);
    const bool has_global_to_local = validTransform(msg->global_to_local_tf);
    const bool has_local_to_base = validTransform(msg->local_to_base_tf);

    const tf2::Transform world_to_global = toTf2OrIdentity(msg->world_to_global_tf);
    const tf2::Transform global_to_local = toTf2OrIdentity(msg->global_to_local_tf);
    const tf2::Transform local_to_base = toTf2OrIdentity(msg->local_to_base_tf);
    const tf2::Transform world_to_local = world_to_global * global_to_local;
    const tf2::Transform world_to_base = world_to_local * local_to_base;

    if (has_world_to_global)
    {
        appendFrameAxis(markers,
                        marker_id,
                        world_to_global,
                        frameLabel(msg->world_to_global_tf.child_frame_id, "sunray_global"),
                        g_tf_axis_length,
                        0.18);
    }
    if (has_global_to_local)
    {
        appendFrameAxis(markers,
                        marker_id,
                        world_to_local,
                        frameLabel(msg->global_to_local_tf.child_frame_id, "sunray_local"),
                        g_tf_axis_length,
                        0.22);
    }
    if (has_local_to_base)
    {
        appendFrameAxis(markers,
                        marker_id,
                        world_to_base,
                        frameLabel(msg->local_to_base_tf.child_frame_id, "base_link"),
                        g_tf_axis_length,
                        0.26);
    }
    appendTfTreeLine(markers, marker_id, world_to_global, world_to_local, world_to_base, has_local_to_base);

    appendOdomMarkers(markers,
                      marker_id,
                      msg->local_odom,
                      "local_odom",
                      makeColor(0.10, 0.85, 1.0, 0.90),
                      g_local_trail);
    appendOdomMarkers(markers,
                      marker_id,
                      msg->global_odom,
                      "global_odom",
                      makeColor(1.0, 0.68, 0.18, 0.90),
                      g_global_trail);

    appendStatusText(markers, marker_id, *msg, world_to_base, has_local_to_base);
    g_marker_pub.publish(markers);
}

}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_visualization_localization_fusion_node");
    ros::NodeHandle nh("~");

    std::string agent_name = "uav";
    int agent_id = 1;
    std::string state_topic;
    std::string marker_topic;

    nh.param("agent_name", agent_name, agent_name);
    nh.param("agent_id", agent_id, agent_id);
    nh.param("frame_id", g_frame_id, g_frame_id);
    nh.param("world_axis_length", g_world_axis_length, g_world_axis_length);
    nh.param("tf_axis_length", g_tf_axis_length, g_tf_axis_length);
    nh.param("axis_text_height", g_axis_text_height, g_axis_text_height);
    nh.param("odom_text_height", g_odom_text_height, g_odom_text_height);
    nh.param("trail_length", g_trail_length, g_trail_length);
    nh.param("state_topic", state_topic,
             "/" + agent_name + std::to_string(agent_id) + "/sunray/localization/odom_state");
    nh.param("marker_topic", marker_topic,
             "/" + agent_name + std::to_string(agent_id) + "/sunray/localization/rviz_markers");

    g_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(marker_topic, 10);
    ros::Subscriber state_sub = nh.subscribe(state_topic, 20, stateCallback);

    ROS_INFO("rviz_visualization_localization_fusion_node subscribe: %s, publish: %s",
             state_topic.c_str(),
             marker_topic.c_str());

    ros::spin();
    return 0;
}
