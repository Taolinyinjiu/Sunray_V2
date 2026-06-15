/*
本程序功能：
    1、订阅 UAVControlState
    2、发布 RViz MarkerArray，显示无人机、速度、ID/FSM、目标点、轨迹和底层控制输出
    3、仅依赖 uav_control/control_state，不额外订阅 odom
*/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlState.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <sstream>
#include <string>

namespace
{
constexpr double kRadToDeg = 57.29577951308232;

ros::Publisher g_marker_pub;
std::string g_frame_id = "world";
std::string g_mesh_resource = "package://sunray_swarm_control/utils/meshes/uav.dae";
double g_mesh_scale = 1.0;
double g_world_axis_length = 1.2;
double g_world_axis_text_height = 0.18;
int g_trail_length = 50;
std::deque<geometry_msgs::Point> g_trail;

geometry_msgs::Point makePoint(const double x, const double y, const double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
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
    case sunray_msgs::UAVControlState::OFF:
        return "OFF";
    case sunray_msgs::UAVControlState::INIT:
        return "INIT";
    case sunray_msgs::UAVControlState::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVControlState::HOVER:
        return "HOVER";
    case sunray_msgs::UAVControlState::RETURN:
        return "RETURN";
    case sunray_msgs::UAVControlState::LAND:
        return "LAND";
    case sunray_msgs::UAVControlState::MOVE:
        return "MOVE";
    case sunray_msgs::UAVControlState::EMERGENCY_KILL:
        return "KILL";
    default:
        return "UNKNOWN";
    }
}

std::string cmdName(const uint8_t cmd)
{
    switch (cmd)
    {
    case sunray_msgs::UAVControlCMD::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::UAVControlCMD::LAND:
        return "LAND";
    case sunray_msgs::UAVControlCMD::RETURN:
        return "RETURN";
    case sunray_msgs::UAVControlCMD::KILL:
        return "KILL";
    case sunray_msgs::UAVControlCMD::HOVER:
        return "HOVER";
    case sunray_msgs::UAVControlCMD::MOVE_POINT:
        return "MOVE_POINT";
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY:
        return "MOVE_VELOCITY";
    case sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY:
        return "MOVE_TRAJECTORY";
    case sunray_msgs::UAVControlCMD::MOVE_POINT_BODY:
        return "MOVE_POINT_BODY";
    case sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY:
        return "MOVE_VELOCITY_BODY";
    case sunray_msgs::UAVControlCMD::MOVE_POINT_WGS84:
        return "MOVE_POINT_WGS84";
    default:
        return "UNDEFINE";
    }
}

std::string outputTypeName(const uint8_t output_type)
{
    switch (output_type)
    {
    case sunray_msgs::UAVControlState::OUTPUT_NONE:
        return "NONE";
    case sunray_msgs::UAVControlState::OUTPUT_POSITION_TARGET:
        return "POSITION_TARGET";
    case sunray_msgs::UAVControlState::OUTPUT_ATTITUDE_TARGET:
        return "ATTITUDE_TARGET";
    default:
        return "UNKNOWN";
    }
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

void appendWorldMarkers(visualization_msgs::MarkerArray &markers)
{
    visualization_msgs::Marker origin = makeMarker(1, "world_origin", visualization_msgs::Marker::SPHERE);
    origin.pose.position.z = 0.03;
    origin.scale.x = 0.08;
    origin.scale.y = 0.08;
    origin.scale.z = 0.08;
    origin.color.r = 1.0;
    origin.color.g = 1.0;
    origin.color.b = 1.0;
    origin.color.a = 0.95;
    markers.markers.push_back(origin);

    visualization_msgs::Marker x_axis = makeMarker(2, "world_axis", visualization_msgs::Marker::ARROW);
    x_axis.points.push_back(makePoint(0.0, 0.0, 0.02));
    x_axis.points.push_back(makePoint(g_world_axis_length, 0.0, 0.02));
    x_axis.scale.x = 0.03;
    x_axis.scale.y = 0.07;
    x_axis.scale.z = 0.10;
    x_axis.color.r = 0.95;
    x_axis.color.g = 0.18;
    x_axis.color.b = 0.18;
    x_axis.color.a = 0.95;
    markers.markers.push_back(x_axis);

    visualization_msgs::Marker y_axis = makeMarker(3, "world_axis", visualization_msgs::Marker::ARROW);
    y_axis.points.push_back(makePoint(0.0, 0.0, 0.02));
    y_axis.points.push_back(makePoint(0.0, g_world_axis_length, 0.02));
    y_axis.scale = x_axis.scale;
    y_axis.color.r = 0.12;
    y_axis.color.g = 0.85;
    y_axis.color.b = 0.22;
    y_axis.color.a = 0.95;
    markers.markers.push_back(y_axis);

    visualization_msgs::Marker z_axis = makeMarker(4, "world_axis", visualization_msgs::Marker::ARROW);
    z_axis.points.push_back(makePoint(0.0, 0.0, 0.02));
    z_axis.points.push_back(makePoint(0.0, 0.0, g_world_axis_length));
    z_axis.scale = x_axis.scale;
    z_axis.color.r = 0.18;
    z_axis.color.g = 0.45;
    z_axis.color.b = 1.0;
    z_axis.color.a = 0.95;
    markers.markers.push_back(z_axis);

    visualization_msgs::Marker origin_text =
        makeMarker(5, "world_axis_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    origin_text.pose.position = makePoint(0.0, 0.0, 0.20);
    origin_text.scale.z = g_world_axis_text_height;
    origin_text.color.r = 1.0;
    origin_text.color.g = 1.0;
    origin_text.color.b = 1.0;
    origin_text.color.a = 1.0;
    origin_text.text = "world(O)";
    markers.markers.push_back(origin_text);

    visualization_msgs::Marker x_text =
        makeMarker(6, "world_axis_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    x_text.pose.position = makePoint(g_world_axis_length + 0.10, 0.0, 0.10);
    x_text.scale.z = g_world_axis_text_height;
    x_text.color = x_axis.color;
    x_text.text = "+X";
    markers.markers.push_back(x_text);

    visualization_msgs::Marker y_text =
        makeMarker(7, "world_axis_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    y_text.pose.position = makePoint(0.0, g_world_axis_length + 0.10, 0.10);
    y_text.scale.z = g_world_axis_text_height;
    y_text.color = y_axis.color;
    y_text.text = "+Y";
    markers.markers.push_back(y_text);

    visualization_msgs::Marker z_text =
        makeMarker(8, "world_axis_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    z_text.pose.position = makePoint(0.0, 0.0, g_world_axis_length + 0.12);
    z_text.scale.z = g_world_axis_text_height;
    z_text.color = z_axis.color;
    z_text.text = "+Z";
    markers.markers.push_back(z_text);
}

bool commandGoalPoint(const sunray_msgs::UAVControlState &state, geometry_msgs::Point &goal, std::string &label)
{
    const sunray_msgs::UAVControlCMD &cmd = state.last_cmd;
    const nav_msgs::Odometry &odom = state.self_odom;
    switch (cmd.control_cmd)
    {
    case sunray_msgs::UAVControlCMD::MOVE_POINT:
    case sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY:
        goal.x = cmd.desired_pos.x;
        goal.y = cmd.desired_pos.y;
        goal.z = cmd.desired_pos.z;
        label = cmdName(cmd.control_cmd);
        return true;
    case sunray_msgs::UAVControlCMD::MOVE_POINT_BODY:
    {
        const double yaw = yawFromOdom(odom);
        const double bx = cmd.desired_body_xy_pos.x;
        const double by = cmd.desired_body_xy_pos.y;
        goal.x = odom.pose.pose.position.x + bx * std::cos(yaw) - by * std::sin(yaw);
        goal.y = odom.pose.pose.position.y + bx * std::sin(yaw) + by * std::cos(yaw);
        goal.z = cmd.fixed_height;
        label = "MOVE_POINT_BODY";
        return true;
    }
    case sunray_msgs::UAVControlCMD::RETURN:
        goal.x = state.home_point.x;
        goal.y = state.home_point.y;
        goal.z = state.home_point.z;
        label = "RETURN";
        return true;
    default:
        break;
    }

    if (state.controller_output_type == sunray_msgs::UAVControlState::OUTPUT_POSITION_TARGET)
    {
        const uint16_t mask = state.position_target.type_mask;
        const bool has_position = ((mask & mavros_msgs::PositionTarget::IGNORE_PX) == 0) &&
                                  ((mask & mavros_msgs::PositionTarget::IGNORE_PY) == 0) &&
                                  ((mask & mavros_msgs::PositionTarget::IGNORE_PZ) == 0);
        if (has_position)
        {
            goal = state.position_target.position;
            label = "POSITION_TARGET";
            return true;
        }
    }

    return false;
}

void appendControllerOutputText(const sunray_msgs::UAVControlState &state,
                                const geometry_msgs::Point &pos,
                                const int base_id,
                                visualization_msgs::MarkerArray &markers)
{
    visualization_msgs::Marker text =
        makeMarker(base_id + 9, "uav_controller_output_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    text.pose.position = pos;
    text.pose.position.z += 0.90;
    text.scale.z = 0.18;
    text.color.r = 0.75;
    text.color.g = 0.92;
    text.color.b = 1.0;
    text.color.a = 1.0;

    std::ostringstream ss;
    ss << "out=" << outputTypeName(state.controller_output_type);
    if (state.controller_output_type == sunray_msgs::UAVControlState::OUTPUT_ATTITUDE_TARGET)
    {
        ss << "  thrust=" << std::fixed << std::setprecision(2) << state.attitude_target.thrust
           << "  wz=" << state.attitude_target.body_rate.z * kRadToDeg << "deg/s";
    }
    else if (state.controller_output_type == sunray_msgs::UAVControlState::OUTPUT_POSITION_TARGET)
    {
        ss << "  yaw=" << std::fixed << std::setprecision(1)
           << state.position_target.yaw * kRadToDeg << "deg";
    }
    text.text = ss.str();
    markers.markers.push_back(text);
}

void stateCallback(const sunray_msgs::UAVControlState::ConstPtr &msg)
{
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    appendWorldMarkers(markers);

    if (!msg->odometry_valid || msg->self_odom.header.stamp.isZero())
    {
        g_marker_pub.publish(markers);
        return;
    }

    const int base_id = static_cast<int>(msg->agent_id) * 100;
    const geometry_msgs::Point &pos = msg->self_odom.pose.pose.position;
    const double yaw = yawFromOdom(msg->self_odom);

    g_trail.push_back(pos);
    while (static_cast<int>(g_trail.size()) > std::max(2, g_trail_length))
    {
        g_trail.pop_front();
    }

    visualization_msgs::Marker body = makeMarker(base_id + 1, "uav_body", visualization_msgs::Marker::MESH_RESOURCE);
    body.pose = msg->self_odom.pose.pose;
    body.scale.x = g_mesh_scale;
    body.scale.y = g_mesh_scale;
    body.scale.z = g_mesh_scale;
    body.mesh_resource = g_mesh_resource;
    body.mesh_use_embedded_materials = true;
    body.color.r = 0.20;
    body.color.g = 0.75;
    body.color.b = 1.0;
    body.color.a = 0.92;
    markers.markers.push_back(body);

    visualization_msgs::Marker text =
        makeMarker(base_id + 2, "uav_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    text.pose.position = pos;
    text.pose.position.z += 0.55;
    text.scale.z = 0.25;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 1.0;
    std::ostringstream name_ss;
    name_ss << "/" << msg->agent_name << static_cast<int>(msg->agent_id) << "  " << fsmName(msg->control_state);
    text.text = name_ss.str();
    markers.markers.push_back(text);

    visualization_msgs::Marker heading = makeMarker(base_id + 3, "uav_heading", visualization_msgs::Marker::ARROW);
    heading.points.push_back(makePoint(pos.x, pos.y, pos.z + 0.10));
    heading.points.push_back(makePoint(pos.x + 0.65 * std::cos(yaw),
                                       pos.y + 0.65 * std::sin(yaw),
                                       pos.z + 0.10));
    heading.scale.x = 0.04;
    heading.scale.y = 0.09;
    heading.scale.z = 0.12;
    heading.color.r = 1.0;
    heading.color.g = 0.25;
    heading.color.b = 0.20;
    heading.color.a = 0.95;
    markers.markers.push_back(heading);

    const geometry_msgs::Vector3 &vel_msg = msg->self_odom.twist.twist.linear;
    const double speed_sq = vel_msg.x * vel_msg.x + vel_msg.y * vel_msg.y + vel_msg.z * vel_msg.z;
    if (speed_sq > 1e-6)
    {
        visualization_msgs::Marker vel = makeMarker(base_id + 4, "uav_velocity", visualization_msgs::Marker::ARROW);
        vel.points.push_back(makePoint(pos.x, pos.y, pos.z + 0.18));
        vel.points.push_back(makePoint(pos.x + vel_msg.x,
                                       pos.y + vel_msg.y,
                                       pos.z + 0.18 + vel_msg.z));
        vel.scale.x = 0.04;
        vel.scale.y = 0.09;
        vel.scale.z = 0.12;
        vel.color.r = 0.10;
        vel.color.g = 0.55;
        vel.color.b = 1.0;
        vel.color.a = 0.95;
        markers.markers.push_back(vel);
    }

    visualization_msgs::Marker vel_text =
        makeMarker(base_id + 5, "uav_velocity_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
    vel_text.pose.position = pos;
    vel_text.pose.position.z += 0.74;
    vel_text.scale.z = 0.16;
    vel_text.color.r = 0.45;
    vel_text.color.g = 0.85;
    vel_text.color.b = 1.0;
    vel_text.color.a = 1.0;
    {
        std::ostringstream ss;
        ss << "v=(" << std::fixed << std::setprecision(2)
           << vel_msg.x << ", " << vel_msg.y << ", " << vel_msg.z << ") m/s";
        vel_text.text = ss.str();
    }
    markers.markers.push_back(vel_text);

    if (g_trail.size() >= 2)
    {
        visualization_msgs::Marker trail = makeMarker(base_id + 6, "uav_trail", visualization_msgs::Marker::LINE_STRIP);
        trail.scale.x = 0.018;
        trail.color.r = 0.10;
        trail.color.g = 0.95;
        trail.color.b = 0.95;
        trail.color.a = 0.85;
        trail.points.assign(g_trail.begin(), g_trail.end());
        markers.markers.push_back(trail);
    }

    geometry_msgs::Point goal;
    std::string goal_label;
    if (commandGoalPoint(*msg, goal, goal_label))
    {
        visualization_msgs::Marker target = makeMarker(base_id + 7, "uav_target", visualization_msgs::Marker::SPHERE);
        target.pose.position = goal;
        target.scale.x = 0.14;
        target.scale.y = 0.14;
        target.scale.z = 0.14;
        target.color.r = 1.0;
        target.color.g = 0.80;
        target.color.b = 0.15;
        target.color.a = 0.95;
        markers.markers.push_back(target);

        visualization_msgs::Marker target_text =
            makeMarker(base_id + 8, "uav_target_text", visualization_msgs::Marker::TEXT_VIEW_FACING);
        target_text.pose.position = goal;
        target_text.pose.position.z += 0.28;
        target_text.scale.z = 0.18;
        target_text.color.r = 1.0;
        target_text.color.g = 0.92;
        target_text.color.b = 0.35;
        target_text.color.a = 1.0;
        std::ostringstream ss;
        ss << goal_label << "\n("
           << std::fixed << std::setprecision(2)
           << goal.x << ", " << goal.y << ", " << goal.z << ")";
        target_text.text = ss.str();
        markers.markers.push_back(target_text);
    }

    appendControllerOutputText(*msg, pos, base_id, markers);
    g_marker_pub.publish(markers);
}
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_visualization_uav_control_node");
    ros::NodeHandle nh("~");

    std::string agent_name = "uav";
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
    nh.param("trail_length", g_trail_length, g_trail_length);
    nh.param("state_topic", state_topic,
             "/" + agent_name + std::to_string(agent_id) + "/sunray/uav_control/control_state");
    nh.param("marker_topic", marker_topic,
             "/" + agent_name + std::to_string(agent_id) + "/sunray/uav_control/rviz_markers");

    g_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(marker_topic, 10);
    ros::Subscriber state_sub = nh.subscribe(state_topic, 20, stateCallback);
    ROS_INFO("rviz_visualization_uav_control_node subscribe: %s, publish: %s",
             state_topic.c_str(),
             marker_topic.c_str());

    ros::spin();
    return 0;
}
