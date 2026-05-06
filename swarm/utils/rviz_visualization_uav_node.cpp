/*
本文件功能：
    1、订阅 UAVSwarmState，缓存每架无人机的集群状态
    2、发布 RViz MarkerArray，显示无人机 mesh、速度、ID、轨迹、目标点和当前任务
*/
#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <sunray_msgs/Formation.h>
#include <sunray_msgs/UAVSwarmCMD.h>
#include <sunray_msgs/UAVSwarmState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace
{

struct AgentVisualState
{
    sunray_msgs::UAVSwarmState state{};
    ros::Time receive_time{0.0};
    std::deque<geometry_msgs::Point> trail{};
};

class RvizVisualizationUavNode
{
  public:
    RvizVisualizationUavNode()
        : nh_(), private_nh_("~")
    {
        private_nh_.param("swarm_state_topic", swarm_state_topic_, std::string("/sunray/swarm/uav_swarm_state"));
        private_nh_.param("agent_name", agent_name_, std::string("uav"));
        private_nh_.param("marker_topic", marker_topic_, std::string("/sunray/swarm/uav_rviz_markers"));
        private_nh_.param("frame_id", frame_id_, std::string("world"));
        private_nh_.param("mesh_resource", mesh_resource_,
                          std::string("package://sunray_swarm_control/utils/meshes/uav.dae"));
        private_nh_.param("publish_hz", publish_hz_, 10.0);
        private_nh_.param("stale_timeout", stale_timeout_, 1.0);
        private_nh_.param("trail_size", trail_size_, 50);
        private_nh_.param("mesh_scale", mesh_scale_, 1.0);
        private_nh_.param("velocity_scale", velocity_scale_, 1.0);
        private_nh_.param("text_height", text_height_, 0.35);
        private_nh_.param("static_obstacle_height", static_obstacle_height_, 0.10);
        private_nh_.param("static_obstacle_alpha", static_obstacle_alpha_, 0.42);

        bool static_obstacles_enabled = false;
        private_nh_.param("static_obstacles/enabled", static_obstacles_enabled, false);
        if (static_obstacles_enabled)
        {
            private_nh_.getParam("static_obstacles/x", static_obstacle_x_);
            private_nh_.getParam("static_obstacles/y", static_obstacle_y_);
            private_nh_.getParam("static_obstacles/radius", static_obstacle_radius_);

            if (static_obstacle_x_.size() != static_obstacle_y_.size() ||
                static_obstacle_x_.size() != static_obstacle_radius_.size())
            {
                ROS_WARN("static_obstacles param size mismatch: x=%zu y=%zu radius=%zu",
                         static_obstacle_x_.size(),
                         static_obstacle_y_.size(),
                         static_obstacle_radius_.size());
                static_obstacle_x_.clear();
                static_obstacle_y_.clear();
                static_obstacle_radius_.clear();
            }
        }

        trail_size_ = std::max(1, trail_size_);
        publish_hz_ = std::max(1.0, publish_hz_);
        static_obstacle_height_ = std::max(0.02, static_obstacle_height_);
        static_obstacle_alpha_ = std::max(0.05, std::min(static_obstacle_alpha_, 1.0));

        swarm_state_sub_ =
            nh_.subscribe(swarm_state_topic_, 100, &RvizVisualizationUavNode::swarmStateCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_hz_),
                                         &RvizVisualizationUavNode::publishTimerCallback, this);

        ROS_INFO("rviz_visualization_uav_node ready: state_topic=%s marker_topic=%s mesh=%s",
                 swarm_state_topic_.c_str(),
                 marker_topic_.c_str(),
                 mesh_resource_.c_str());
        ROS_INFO("rviz static obstacles: %zu", static_obstacle_x_.size());
    }

  private:
    void swarmStateCallback(const sunray_msgs::UAVSwarmState::ConstPtr &msg)
    {
        if (msg->agent_id == 0)
        {
            return;
        }

        AgentVisualState &agent = agents_[static_cast<int>(msg->agent_id)];
        agent.state = *msg;
        agent.receive_time = ros::Time::now();

        if (msg->self_odom_ready)
        {
            const geometry_msgs::Point &pos = msg->self_odom.pose.pose.position;
            if (agent.trail.empty() || distance(agent.trail.back(), pos) > 1e-3)
            {
                agent.trail.push_back(pos);
                while (agent.trail.size() > static_cast<size_t>(trail_size_))
                {
                    agent.trail.pop_front();
                }
            }
        }
    }

    void publishTimerCallback(const ros::TimerEvent &)
    {
        const ros::Time now = ros::Time::now();
        visualization_msgs::MarkerArray marker_array;

        appendWorldOriginMarkers(now, marker_array);
        appendStaticObstacleMarkers(now, marker_array);

        for (auto &item : agents_)
        {
            const int agent_id = item.first;
            const AgentVisualState &agent = item.second;
            const bool online = (now - agent.receive_time).toSec() <= stale_timeout_;

            if (!agent.state.self_odom_ready || !online)
            {
                appendDeleteMarkers(agent_id, marker_array);
                continue;
            }

            appendAgentMarkers(agent_id, agent, now, marker_array);
        }

        marker_pub_.publish(marker_array);
    }

    void appendWorldOriginMarkers(const ros::Time &stamp, visualization_msgs::MarkerArray &marker_array) const
    {
        geometry_msgs::Point origin;
        origin.x = 0.0;
        origin.y = 0.0;
        origin.z = 0.0;

        visualization_msgs::Marker origin_sphere = baseOriginMarker(0, stamp);
        origin_sphere.type = visualization_msgs::Marker::SPHERE;
        origin_sphere.pose.position = origin;
        origin_sphere.scale.x = 0.18;
        origin_sphere.scale.y = 0.18;
        origin_sphere.scale.z = 0.18;
        origin_sphere.color = makeColor(1.0, 1.0, 1.0, 1.0);
        marker_array.markers.push_back(origin_sphere);

        appendAxisMarker(1, stamp, origin, 1.2, 0.0, 0.0, makeColor(1.0, 0.1, 0.1, 1.0), marker_array);
        appendAxisMarker(2, stamp, origin, 0.0, 1.2, 0.0, makeColor(0.1, 1.0, 0.1, 1.0), marker_array);
        appendAxisMarker(3, stamp, origin, 0.0, 0.0, 1.2, makeColor(0.2, 0.45, 1.0, 1.0), marker_array);

        visualization_msgs::Marker origin_text = baseOriginMarker(4, stamp);
        origin_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        origin_text.pose.position = origin;
        origin_text.pose.position.z = 0.35;
        origin_text.scale.z = text_height_ * 0.75;
        origin_text.color = makeColor(1.0, 1.0, 1.0, 1.0);
        origin_text.text = "world origin (0,0,0)";
        marker_array.markers.push_back(origin_text);
    }

    void appendStaticObstacleMarkers(const ros::Time &stamp, visualization_msgs::MarkerArray &marker_array) const
    {
        for (size_t i = 0; i < static_obstacle_x_.size(); ++i)
        {
            if (static_obstacle_radius_[i] <= 0.0)
            {
                continue;
            }

            visualization_msgs::Marker cylinder = baseStaticObstacleMarker(static_cast<int>(i * 2), stamp);
            cylinder.type = visualization_msgs::Marker::CYLINDER;
            cylinder.pose.position.x = static_obstacle_x_[i];
            cylinder.pose.position.y = static_obstacle_y_[i];
            cylinder.pose.position.z = static_obstacle_height_ * 0.5;
            cylinder.scale.x = static_obstacle_radius_[i] * 2.0;
            cylinder.scale.y = static_obstacle_radius_[i] * 2.0;
            cylinder.scale.z = static_obstacle_height_;
            cylinder.color = makeColor(1.0, 0.20, 0.05, static_obstacle_alpha_);
            marker_array.markers.push_back(cylinder);

            visualization_msgs::Marker text = baseStaticObstacleMarker(static_cast<int>(i * 2 + 1), stamp);
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.pose.position.x = static_obstacle_x_[i];
            text.pose.position.y = static_obstacle_y_[i];
            text.pose.position.z = static_obstacle_height_ + 0.35;
            text.scale.z = text_height_ * 0.70;
            text.color = makeColor(1.0, 0.45, 0.20, 1.0);

            std::ostringstream ss;
            ss << "obs" << (i + 1) << " r=" << std::fixed << std::setprecision(2) << static_obstacle_radius_[i];
            text.text = ss.str();
            marker_array.markers.push_back(text);
        }
    }

    void appendAxisMarker(const int marker_id,
                          const ros::Time &stamp,
                          const geometry_msgs::Point &origin,
                          const double dx,
                          const double dy,
                          const double dz,
                          const std_msgs::ColorRGBA &color,
                          visualization_msgs::MarkerArray &marker_array) const
    {
        visualization_msgs::Marker axis = baseOriginMarker(marker_id, stamp);
        axis.type = visualization_msgs::Marker::ARROW;
        axis.points.push_back(origin);
        geometry_msgs::Point end = origin;
        end.x += dx;
        end.y += dy;
        end.z += dz;
        axis.points.push_back(end);
        axis.scale.x = 0.035;
        axis.scale.y = 0.11;
        axis.scale.z = 0.16;
        axis.color = color;
        marker_array.markers.push_back(axis);
    }

    void appendAgentMarkers(const int agent_id,
                            const AgentVisualState &agent,
                            const ros::Time &stamp,
                            visualization_msgs::MarkerArray &marker_array) const
    {
        const sunray_msgs::UAVSwarmState &state = agent.state;
        const geometry_msgs::Point &pos = state.self_odom.pose.pose.position;
        const std_msgs::ColorRGBA color = colorForAgent(agent_id, 1.0);

        visualization_msgs::Marker mesh = baseMarker(agent_id, 0, stamp);
        mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
        mesh.mesh_resource = mesh_resource_;
        mesh.mesh_use_embedded_materials = true;
        mesh.pose = state.self_odom.pose.pose;
        mesh.scale.x = mesh_scale_;
        mesh.scale.y = mesh_scale_;
        mesh.scale.z = mesh_scale_;
        mesh.color = color;
        marker_array.markers.push_back(mesh);

        visualization_msgs::Marker velocity = baseMarker(agent_id, 1, stamp);
        velocity.type = visualization_msgs::Marker::ARROW;
        velocity.points.push_back(pos);
        geometry_msgs::Point vel_end = pos;
        vel_end.x += state.self_odom.twist.twist.linear.x * velocity_scale_;
        vel_end.y += state.self_odom.twist.twist.linear.y * velocity_scale_;
        vel_end.z += state.self_odom.twist.twist.linear.z * velocity_scale_;
        velocity.points.push_back(vel_end);
        velocity.scale.x = 0.04;
        velocity.scale.y = 0.12;
        velocity.scale.z = 0.18;
        velocity.color = colorForAgent(agent_id, 0.9);
        marker_array.markers.push_back(velocity);

        visualization_msgs::Marker id_text = baseMarker(agent_id, 2, stamp);
        id_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        id_text.pose.position = pos;
        id_text.pose.position.z += 0.75;
        id_text.scale.z = text_height_;
        id_text.color = makeColor(1.0, 1.0, 1.0, 1.0);
        id_text.text = agent_name_ + std::to_string(agent_id);
        marker_array.markers.push_back(id_text);

        visualization_msgs::Marker task_text = baseMarker(agent_id, 3, stamp);
        task_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        task_text.pose.position = pos;
        task_text.pose.position.z += 1.15;
        task_text.scale.z = text_height_ * 0.78;
        task_text.color = makeColor(0.2, 0.95, 1.0, 1.0);
        task_text.text = taskText(state);
        marker_array.markers.push_back(task_text);

        if (agent.trail.size() >= 2)
        {
            visualization_msgs::Marker trail = baseMarker(agent_id, 4, stamp);
            trail.type = visualization_msgs::Marker::LINE_STRIP;
            trail.scale.x = 0.013;
            trail.color = colorForAgent(agent_id, 0.85);
            for (const geometry_msgs::Point &point : agent.trail)
            {
                trail.points.push_back(point);
            }
            marker_array.markers.push_back(trail);
        }
        else
        {
            appendDeleteMarker(agent_id, 4, stamp, marker_array);
        }

        if (state.target_valid)
        {
            appendTargetMarkers(agent_id, state, stamp, marker_array);
        }
        else
        {
            appendDeleteMarker(agent_id, 5, stamp, marker_array);
            appendDeleteMarker(agent_id, 6, stamp, marker_array);
            appendDeleteMarker(agent_id, 7, stamp, marker_array);
        }
    }

    void appendTargetMarkers(const int agent_id,
                             const sunray_msgs::UAVSwarmState &state,
                             const ros::Time &stamp,
                             visualization_msgs::MarkerArray &marker_array) const
    {
        visualization_msgs::Marker target_sphere = baseMarker(agent_id, 5, stamp);
        target_sphere.type = visualization_msgs::Marker::SPHERE;
        target_sphere.pose.position = state.target_pos;
        target_sphere.scale.x = 0.14;
        target_sphere.scale.y = 0.14;
        target_sphere.scale.z = 0.14;
        target_sphere.color = makeColor(1.0, 0.75, 0.1, 0.95);
        marker_array.markers.push_back(target_sphere);

        visualization_msgs::Marker target_text = baseMarker(agent_id, 7, stamp);
        target_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        target_text.pose.position = state.target_pos;
        target_text.pose.position.z += 0.45;
        target_text.scale.z = text_height_ * 0.72;
        target_text.color = makeColor(1.0, 0.9, 0.35, 1.0);
        target_text.text = "goal " + agent_name_ + std::to_string(agent_id);
        marker_array.markers.push_back(target_text);
    }

    visualization_msgs::Marker baseMarker(const int agent_id, const int local_id, const ros::Time &stamp) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = stamp;
        marker.ns = agent_name_ + "_" + std::to_string(agent_id);
        marker.id = local_id;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(1.0 / publish_hz_ * 3.0);
        return marker;
    }

    visualization_msgs::Marker baseOriginMarker(const int local_id, const ros::Time &stamp) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = stamp;
        marker.ns = "world_origin";
        marker.id = local_id;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0.0);
        return marker;
    }

    visualization_msgs::Marker baseStaticObstacleMarker(const int local_id, const ros::Time &stamp) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = stamp;
        marker.ns = "static_obstacles";
        marker.id = local_id;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0.0);
        return marker;
    }

    void appendDeleteMarkers(const int agent_id, visualization_msgs::MarkerArray &marker_array) const
    {
        const ros::Time now = ros::Time::now();
        for (int marker_id = 0; marker_id <= 7; ++marker_id)
        {
            appendDeleteMarker(agent_id, marker_id, now, marker_array);
        }
    }

    void appendDeleteMarker(const int agent_id,
                            const int marker_id,
                            const ros::Time &stamp,
                            visualization_msgs::MarkerArray &marker_array) const
    {
        visualization_msgs::Marker marker = baseMarker(agent_id, marker_id, stamp);
        marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(marker);
    }

    static std_msgs::ColorRGBA makeColor(const double r, const double g, const double b, const double a)
    {
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(r);
        color.g = static_cast<float>(g);
        color.b = static_cast<float>(b);
        color.a = static_cast<float>(a);
        return color;
    }

    static std_msgs::ColorRGBA colorForAgent(const int agent_id, const double alpha)
    {
        static const double palette[][3] = {
            {0.10, 0.55, 1.00},
            {0.10, 0.85, 0.45},
            {1.00, 0.55, 0.10},
            {0.90, 0.25, 0.30},
            {0.70, 0.35, 1.00},
            {0.95, 0.85, 0.20},
            {0.20, 0.90, 0.90},
            {1.00, 0.35, 0.75},
            {0.55, 0.90, 0.15},
            {0.55, 0.55, 0.55},
        };
        const size_t idx = static_cast<size_t>(std::max(0, agent_id - 1)) % (sizeof(palette) / sizeof(palette[0]));
        return makeColor(palette[idx][0], palette[idx][1], palette[idx][2], alpha);
    }

    static double distance(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    static const char *swarmStateName(const uint8_t state)
    {
        switch (state)
        {
        case sunray_msgs::UAVSwarmState::INIT:
            return "INIT";
        case sunray_msgs::UAVSwarmState::TAKEOFF:
            return "TAKEOFF";
        case sunray_msgs::UAVSwarmState::LAND:
            return "LAND";
        case sunray_msgs::UAVSwarmState::RETURN_HOME:
            return "RETURN_HOME";
        case sunray_msgs::UAVSwarmState::ARRIVED:
            return "ARRIVED";
        case sunray_msgs::UAVSwarmState::SWARM_STATIC_FORMATION:
            return "STATIC_FORMATION";
        case sunray_msgs::UAVSwarmState::SWARM_DYNAMIC_FORMATION:
            return "DYNAMIC_FORMATION";
        case sunray_msgs::UAVSwarmState::SWARM_DYNAMIC_FORMATION_PREPARE:
            return "DYNAMIC_PREPARE";
        default:
            return "UNKNOWN";
        }
    }

    static const char *swarmCmdName(const uint8_t cmd)
    {
        switch (cmd)
        {
        case sunray_msgs::UAVSwarmCMD::SWARM_TAKEOFF:
            return "TAKEOFF";
        case sunray_msgs::UAVSwarmCMD::SWARM_LAND:
            return "LAND";
        case sunray_msgs::UAVSwarmCMD::SWARM_HOVER:
            return "HOVER";
        case sunray_msgs::UAVSwarmCMD::SWARM_RETURN:
            return "RETURN";
        case sunray_msgs::UAVSwarmCMD::SWARM_FORMATION:
            return "FORMATION";
        default:
            return "UNDEFINE";
        }
    }

    static const char *formationName(const uint8_t formation_type)
    {
        switch (formation_type)
        {
        case sunray_msgs::Formation::STATIC_KEEP_FORMATION:
            return "STATIC_KEEP";
        case sunray_msgs::Formation::STATIC_FORMATION_LINE:
            return "STATIC_LINE";
        case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
            return "STATIC_POLYGON";
        case sunray_msgs::Formation::STATIC_FORMATION_RANDOM:
            return "STATIC_RANDOM";
        case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
            return "STATIC_CUSTOM";
        case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
            return "DYNAMIC_RING";
        case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
            return "DYNAMIC_POLYGON";
        case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
            return "DYNAMIC_LEMNISCATE";
        default:
            return "UNKNOWN_FORMATION";
        }
    }

    static std::string taskText(const sunray_msgs::UAVSwarmState &state)
    {
        std::ostringstream ss;
        ss << swarmStateName(state.fsm_state) << " / " << swarmCmdName(state.swarm_cmd.swarm_cmd);
        if (state.swarm_cmd.swarm_cmd == sunray_msgs::UAVSwarmCMD::SWARM_FORMATION)
        {
            ss << " / " << formationName(state.swarm_cmd.formation_cmd.formation_type);
        }
        return ss.str();
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber swarm_state_sub_;
    ros::Publisher marker_pub_;
    ros::Timer publish_timer_;

    std::string swarm_state_topic_;
    std::string agent_name_{"uav"};
    std::string marker_topic_;
    std::string frame_id_;
    std::string mesh_resource_;
    double publish_hz_{10.0};
    double stale_timeout_{1.0};
    int trail_size_{50};
    double mesh_scale_{1.0};
    double velocity_scale_{1.0};
    double text_height_{0.35};
    double static_obstacle_height_{0.10};
    double static_obstacle_alpha_{0.42};
    std::vector<double> static_obstacle_x_{};
    std::vector<double> static_obstacle_y_{};
    std::vector<double> static_obstacle_radius_{};
    std::map<int, AgentVisualState> agents_;
};

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_visualization_uav_node");
    RvizVisualizationUavNode node;
    ros::spin();
    return 0;
}
