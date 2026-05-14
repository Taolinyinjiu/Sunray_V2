/*
本程序功能：
    1、订阅 UGVControlState
    2、发布 RViz MarkerArray，显示无人车、速度、ID/FSM、目标点和地理围栏
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/UGVControlState.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace
{
ros::Publisher g_marker_pub;
std::string g_frame_id = "world";
std::string g_mesh_resource = "package://sunray_ugv_control/meshes/ugv_vehicle.dae";
double g_mesh_scale = 1.15;  // 0.28 m * 1.15 ~= 0.32 m
double g_world_axis_length = 1.0;
double g_world_axis_text_height = 0.18;

geometry_msgs::Quaternion yawToQuat(const double yaw)
{
    geometry_msgs::Quaternion q;
    q.w = std::cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    return q;
}

double yawFromOdom(const nav_msgs::Odometry &odom)
{
    const auto &q = odom.pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::string fsmName(const uint8_t state)
{
    switch (state)
    {
    case sunray_msgs::UGVControlState::FSM_INIT:
        return "INIT";
    case sunray_msgs::UGVControlState::FSM_HOLD:
        return "HOLD";
    case sunray_msgs::UGVControlState::FSM_RETURN:
        return "RETURN";
    case sunray_msgs::UGVControlState::FSM_MOVE:
        return "MOVE";
    default:
        return "UNKNOWN";
    }
}

visualization_msgs::Marker makeBaseMarker(const sunray_msgs::UGVControlState &state,
                                          const int id,
                                          const std::string &ns,
                                          const int type)
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
    (void)state;
    return marker;
}

visualization_msgs::Marker makeStaticMarker(const int id,
                                            const std::string &ns,
                                            const int type)
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

void appendWorldMarkers(visualization_msgs::MarkerArray &markers)
{
    visualization_msgs::Marker origin =
        makeStaticMarker(1, "world_origin", visualization_msgs::Marker::SPHERE);
    origin.pose.position.x = 0.0;
    origin.pose.position.y = 0.0;
    origin.pose.position.z = 0.03;
    origin.scale.x = 0.08;
    origin.scale.y = 0.08;
    origin.scale.z = 0.08;
    origin.color.r = 1.0;
    origin.color.g = 1.0;
    origin.color.b = 1.0;
    origin.color.a = 0.95;
    markers.markers.push_back(origin);

    visualization_msgs::Marker x_axis =
        makeStaticMarker(2, "world_axis", visualization_msgs::Marker::ARROW);
    x_axis.points.resize(2);
    x_axis.points[0].x = 0.0;
    x_axis.points[0].y = 0.0;
    x_axis.points[0].z = 0.02;
    x_axis.points[1].x = g_world_axis_length;
    x_axis.points[1].y = 0.0;
    x_axis.points[1].z = 0.02;
    x_axis.scale.x = 0.03;
    x_axis.scale.y = 0.06;
    x_axis.scale.z = 0.09;
    x_axis.color.r = 0.95;
    x_axis.color.g = 0.18;
    x_axis.color.b = 0.18;
    x_axis.color.a = 0.95;
    markers.markers.push_back(x_axis);

    visualization_msgs::Marker y_axis =
        makeStaticMarker(3, "world_axis", visualization_msgs::Marker::ARROW);
    y_axis.points.resize(2);
    y_axis.points[0].x = 0.0;
    y_axis.points[0].y = 0.0;
    y_axis.points[0].z = 0.02;
    y_axis.points[1].x = 0.0;
    y_axis.points[1].y = g_world_axis_length;
    y_axis.points[1].z = 0.02;
    y_axis.scale.x = 0.03;
    y_axis.scale.y = 0.06;
    y_axis.scale.z = 0.09;
    y_axis.color.r = 0.12;
    y_axis.color.g = 0.85;
    y_axis.color.b = 0.22;
    y_axis.color.a = 0.95;
    markers.markers.push_back(y_axis);

    visualization_msgs::Marker origin_text =
        makeStaticMarker(4, "world_axis_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    origin_text.pose.position.x = 0.0;
    origin_text.pose.position.y = 0.0;
    origin_text.pose.position.z = 0.18;
    origin_text.scale.z = g_world_axis_text_height;
    origin_text.color.r = 1.0;
    origin_text.color.g = 1.0;
    origin_text.color.b = 1.0;
    origin_text.color.a = 1.0;
    origin_text.text = "world(O)";
    markers.markers.push_back(origin_text);

    visualization_msgs::Marker x_text =
        makeStaticMarker(5, "world_axis_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    x_text.pose.position.x = g_world_axis_length + 0.08;
    x_text.pose.position.y = 0.0;
    x_text.pose.position.z = 0.10;
    x_text.scale.z = g_world_axis_text_height;
    x_text.color.r = 0.95;
    x_text.color.g = 0.18;
    x_text.color.b = 0.18;
    x_text.color.a = 1.0;
    x_text.text = "+X";
    markers.markers.push_back(x_text);

    visualization_msgs::Marker y_text =
        makeStaticMarker(6, "world_axis_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    y_text.pose.position.x = 0.0;
    y_text.pose.position.y = g_world_axis_length + 0.08;
    y_text.pose.position.z = 0.10;
    y_text.scale.z = g_world_axis_text_height;
    y_text.color.r = 0.12;
    y_text.color.g = 0.85;
    y_text.color.b = 0.22;
    y_text.color.a = 1.0;
    y_text.text = "+Y";
    markers.markers.push_back(y_text);
}

void stateCallback(const sunray_msgs::UGVControlState::ConstPtr &msg)
{
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    appendWorldMarkers(markers);

    if (!msg->odom_valid)
    {
        g_marker_pub.publish(markers);
        return;
    }

    const int base_id = static_cast<int>(msg->agent_id) * 100;
    const geometry_msgs::Point &pos = msg->self_odom.pose.pose.position;
    const double yaw = yawFromOdom(msg->self_odom);

    visualization_msgs::Marker body =
        makeBaseMarker(*msg, base_id + 1, "ugv_body", visualization_msgs::Marker::MESH_RESOURCE);
    body.pose.position = pos;
    body.pose.position.z += 0.02;
    body.pose.orientation = yawToQuat(yaw);
    body.scale.x = g_mesh_scale;
    body.scale.y = g_mesh_scale;
    body.scale.z = g_mesh_scale;
    body.mesh_resource = g_mesh_resource;
    body.mesh_use_embedded_materials = true;
    body.color.r = 1.0;
    body.color.g = 1.0;
    body.color.b = 1.0;
    body.color.a = 0.9;
    markers.markers.push_back(body);

    visualization_msgs::Marker text = makeBaseMarker(*msg, base_id + 2, "ugv_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    text.pose.position = pos;
    text.pose.position.z += 0.65;
    text.scale.z = 0.25;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    std::ostringstream ss;
    ss << "/" << msg->agent_name << static_cast<int>(msg->agent_id) << "  " << fsmName(msg->fsm_state);
    text.text = ss.str();
    markers.markers.push_back(text);

    visualization_msgs::Marker heading =
        makeBaseMarker(*msg, base_id + 3, "ugv_heading", visualization_msgs::Marker::ARROW);
    heading.points.resize(2);
    heading.points[0] = pos;
    heading.points[0].z += 0.18;
    heading.points[1] = heading.points[0];
    heading.points[1].x += 0.35 * std::cos(yaw);
    heading.points[1].y += 0.35 * std::sin(yaw);
    heading.scale.x = 0.035;
    heading.scale.y = 0.07;
    heading.scale.z = 0.10;
    heading.color.r = 1.0;
    heading.color.g = 0.25;
    heading.color.b = 0.25;
    heading.color.a = 0.95;
    markers.markers.push_back(heading);

    visualization_msgs::Marker vel = makeBaseMarker(*msg, base_id + 4, "ugv_velocity", visualization_msgs::Marker::ARROW);
    vel.points.resize(2);
    vel.points[0] = pos;
    vel.points[0].z += 0.25;
    vel.points[1] = vel.points[0];
    vel.points[1].x += msg->controller_cmd_vel.linear.x * std::cos(yaw) * 0.8 -
                       msg->controller_cmd_vel.linear.y * std::sin(yaw) * 0.8;
    vel.points[1].y += msg->controller_cmd_vel.linear.x * std::sin(yaw) * 0.8 +
                       msg->controller_cmd_vel.linear.y * std::cos(yaw) * 0.8;
    vel.scale.x = 0.04;
    vel.scale.y = 0.08;
    vel.scale.z = 0.12;
    vel.color.r = 0.2;
    vel.color.g = 0.6;
    vel.color.b = 1.0;
    vel.color.a = 0.9;
    markers.markers.push_back(vel);

    visualization_msgs::Marker vel_text =
        makeBaseMarker(*msg, base_id + 5, "ugv_velocity_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    vel_text.pose.position = pos;
    vel_text.pose.position.z += 0.42;
    vel_text.scale.z = 0.14;
    vel_text.color.r = 0.35;
    vel_text.color.g = 0.82;
    vel_text.color.b = 1.0;
    vel_text.color.a = 1.0;
    {
        std::ostringstream ss;
        ss << "vx=" << std::fixed << std::setprecision(2) << msg->controller_cmd_vel.linear.x
           << "  vy=" << msg->controller_cmd_vel.linear.y
           << "  wz=" << msg->controller_cmd_vel.angular.z;
        vel_text.text = ss.str();
    }
    markers.markers.push_back(vel_text);

    if (msg->target_valid)
    {
        visualization_msgs::Marker target =
            makeBaseMarker(*msg, base_id + 6, "ugv_target", visualization_msgs::Marker::SPHERE);
        target.pose.position = msg->target_pos;
        target.pose.position.z += 0.08;
        target.scale.x = 0.22;
        target.scale.y = 0.22;
        target.scale.z = 0.16;
        target.color.r = 1.0;
        target.color.g = 0.75;
        target.color.b = 0.1;
        target.color.a = 0.9;
        markers.markers.push_back(target);

        visualization_msgs::Marker target_text =
            makeBaseMarker(*msg, base_id + 7, "ugv_target_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
        target_text.pose.position = msg->target_pos;
        target_text.pose.position.z += 0.28;
        target_text.scale.z = 0.14;
        target_text.color.r = 1.0;
        target_text.color.g = 0.82;
        target_text.color.b = 0.18;
        target_text.color.a = 1.0;
        {
            std::ostringstream ss;
            ss << "goal("
               << std::fixed << std::setprecision(2)
               << msg->target_pos.x << ", " << msg->target_pos.y << ", "
               << msg->target_yaw << ")";
            target_text.text = ss.str();
        }
        markers.markers.push_back(target_text);
    }

    visualization_msgs::Marker fence =
        makeBaseMarker(*msg, base_id + 8, "ugv_geo_fence", visualization_msgs::Marker::CUBE);
    fence.pose.position.x = 0.5 * (msg->geo_fence_min.x + msg->geo_fence_max.x);
    fence.pose.position.y = 0.5 * (msg->geo_fence_min.y + msg->geo_fence_max.y);
    fence.pose.position.z = 0.02;
    fence.scale.x = std::max(0.01, msg->geo_fence_max.x - msg->geo_fence_min.x);
    fence.scale.y = std::max(0.01, msg->geo_fence_max.y - msg->geo_fence_min.y);
    fence.scale.z = 0.02;
    fence.color.r = msg->inside_geo_fence ? 0.1 : 1.0;
    fence.color.g = msg->inside_geo_fence ? 0.6 : 0.1;
    fence.color.b = 0.2;
    fence.color.a = 0.12;
    markers.markers.push_back(fence);

    visualization_msgs::Marker fence_min_text =
        makeBaseMarker(*msg, base_id + 9, "ugv_geo_fence_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    fence_min_text.pose.position = msg->geo_fence_min;
    fence_min_text.pose.position.z += 0.12;
    fence_min_text.scale.z = 0.12;
    fence_min_text.color.r = 0.95;
    fence_min_text.color.g = 0.95;
    fence_min_text.color.b = 0.95;
    fence_min_text.color.a = 0.95;
    {
        std::ostringstream ss;
        ss << "min(" << std::fixed << std::setprecision(1)
           << msg->geo_fence_min.x << ", " << msg->geo_fence_min.y << ")";
        fence_min_text.text = ss.str();
    }
    markers.markers.push_back(fence_min_text);

    visualization_msgs::Marker fence_max_text =
        makeBaseMarker(*msg, base_id + 10, "ugv_geo_fence_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    fence_max_text.pose.position = msg->geo_fence_max;
    fence_max_text.pose.position.z += 0.12;
    fence_max_text.scale.z = 0.12;
    fence_max_text.color.r = 0.95;
    fence_max_text.color.g = 0.95;
    fence_max_text.color.b = 0.95;
    fence_max_text.color.a = 0.95;
    {
        std::ostringstream ss;
        ss << "max(" << std::fixed << std::setprecision(1)
           << msg->geo_fence_max.x << ", " << msg->geo_fence_max.y << ")";
        fence_max_text.text = ss.str();
    }
    markers.markers.push_back(fence_max_text);

    g_marker_pub.publish(markers);
}
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_visualization_ugv_control_node");
    ros::NodeHandle nh("~");

    std::string agent_name = "ugv";
    int agent_id = 1;
    std::string state_topic;
    std::string marker_topic;
    nh.param("agent_name", agent_name, agent_name);
    nh.param("agent_id", agent_id, agent_id);
    nh.param("frame_id", g_frame_id, g_frame_id);
    nh.param("mesh_resource", g_mesh_resource, g_mesh_resource);
    nh.param("mesh_scale", g_mesh_scale, g_mesh_scale);
    nh.param("world_axis_length", g_world_axis_length, g_world_axis_length);
    nh.param("world_axis_text_height", g_world_axis_text_height, g_world_axis_text_height);
    nh.param("state_topic", state_topic,
             "/" + agent_name + std::to_string(agent_id) + "/sunray/ugv_control/control_state");
    nh.param("marker_topic", marker_topic,
             "/" + agent_name + std::to_string(agent_id) + "/sunray/ugv_control/rviz_markers");

    g_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(marker_topic, 10);
    ros::Subscriber state_sub = nh.subscribe(state_topic, 20, stateCallback);
    ROS_INFO("rviz_visualization_ugv_control_node subscribe: %s, publish: %s",
             state_topic.c_str(),
             marker_topic.c_str());

    ros::spin();
    return 0;
}
