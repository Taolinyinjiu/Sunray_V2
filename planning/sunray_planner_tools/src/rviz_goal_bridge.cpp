#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sunray_msgs/PlanningWaypoint.h>
#include <sunray_msgs/UAVPlanningCMD.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "string_uav_namespace_utils.hpp"
#include "sunray_log.hpp"

namespace {

double quaternion_to_yaw(const geometry_msgs::Quaternion& orientation) {
    const double siny_cosp =
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    const double cosy_cosp =
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

bool is_supported_frame(const std::string& frame_id,
                        const std::string& local_frame_id,
                        const std::string& world_frame_id,
                        const std::string& global_frame_id) {
    return frame_id == local_frame_id || frame_id == world_frame_id ||
           frame_id == global_frame_id;
}

std::string load_required_global_string_param_or_throw(ros::NodeHandle& nh,
                                                       const std::string& param_name) {
    std::string value;
    if (!nh.getParam(param_name, value)) {
        throw std::runtime_error("missing param " + param_name);
    }
    if (value.empty()) {
        throw std::runtime_error(param_name + " cannot be empty");
    }
    return value;
}

int load_required_global_int_param_or_throw(ros::NodeHandle& nh,
                                            const std::string& param_name) {
    int value = 0;
    if (!nh.getParam(param_name, value)) {
        throw std::runtime_error("missing param " + param_name);
    }
    return value;
}

std::string load_uav_namespace_or_throw(ros::NodeHandle& nh) {
    const std::string uav_name = load_required_global_string_param_or_throw(nh, "/uav_name");
    const int uav_id = load_required_global_int_param_or_throw(nh, "/uav_id");
    if (uav_id <= 0) {
        throw std::runtime_error("/uav_id cannot <= 0");
    }
    return sunray_common::normalize_uav_ns(uav_name + std::to_string(uav_id));
}

std::string make_planning_cmd_topic(const std::string& uav_ns) {
    return uav_ns + "/sunray/planning_cmd";
}

struct BridgeConfig {
    std::string rviz_goal_topic{"/move_base_simple/goal"};
    std::string local_frame_id{"sunray_local"};
    std::string world_frame_id{"world"};
    std::string global_frame_id{"sunray_global"};
    double tf_lookup_timeout_sec{0.1};
    bool accept_empty_frame_id_as_target_frame{false};
    bool drop_on_tf_failure{true};
    double goal_height{0.6};
    double default_hold_time{0.0};
    double default_yaw{0.0};
    bool use_goal_yaw{true};
    uint8_t control_cmd{sunray_msgs::UAVPlanningCMD::PLANNING_LOCAL};
    uint8_t cmd_source{sunray_msgs::UAVPlanningCMD::SUNRAY_STATION};
};

struct BridgeContext {
    BridgeConfig config;
    std::string planning_cmd_topic;
    ros::Publisher planning_cmd_pub;
    tf2_ros::Buffer tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
};

bool transform_goal_to_local_frame(const geometry_msgs::PoseStamped& input_goal,
                                   geometry_msgs::PoseStamped& output_goal,
                                   const BridgeContext& context) {
    output_goal = input_goal;

    std::string input_frame = input_goal.header.frame_id;
    if (input_frame.empty()) {
        if (!context.config.accept_empty_frame_id_as_target_frame) {
            SUNRAY_WARN(
                "[sunray_planner_tools] pose goal bridge rejected goal from {}: empty frame_id",
                context.config.rviz_goal_topic);
            return false;
        }
        input_frame = context.config.local_frame_id;
        output_goal.header.frame_id = input_frame;
    }

    if (!is_supported_frame(input_frame,
                            context.config.local_frame_id,
                            context.config.world_frame_id,
                            context.config.global_frame_id)) {
        SUNRAY_WARN("[sunray_planner_tools] pose goal bridge rejected unsupported frame_id={} from {}, supported=[{},{},{}]",
                    input_frame,
                    context.config.rviz_goal_topic,
                    context.config.local_frame_id,
                    context.config.world_frame_id,
                    context.config.global_frame_id);
        return false;
    }

    if (input_frame == context.config.local_frame_id) {
        return true;
    }

    const ros::Time transform_time =
        output_goal.header.stamp.isZero() ? ros::Time(0) : output_goal.header.stamp;
    const ros::Duration timeout(context.config.tf_lookup_timeout_sec);

    if (!context.tf_buffer.canTransform(
            context.config.local_frame_id, input_frame, transform_time, timeout)) {
        SUNRAY_WARN("[sunray_planner_tools] pose goal bridge cannot transform goal from {} to {} within {:.3f}s, drop this goal",
                    input_frame,
                    context.config.local_frame_id,
                    context.config.tf_lookup_timeout_sec);
        return !context.config.drop_on_tf_failure;
    }

    try {
        context.tf_buffer.transform(
            output_goal, output_goal, context.config.local_frame_id, timeout);
        return true;
    } catch (const tf2::TransformException& ex) {
        SUNRAY_WARN("[sunray_planner_tools] pose goal bridge transform failed: {} -> {} error={}",
                    input_frame,
                    context.config.local_frame_id,
                    ex.what());
        return !context.config.drop_on_tf_failure;
    }
}

void handle_goal(const geometry_msgs::PoseStamped::ConstPtr& msg, BridgeContext* context) {
    if (!msg || context == nullptr) {
        return;
    }

    geometry_msgs::PoseStamped transformed_goal;
    if (!transform_goal_to_local_frame(*msg, transformed_goal, *context)) {
        return;
    }

    sunray_msgs::UAVPlanningCMD planning_cmd_msg;
    planning_cmd_msg.header = transformed_goal.header;
    planning_cmd_msg.header.frame_id = context->config.local_frame_id;
    if (planning_cmd_msg.header.stamp.isZero()) {
        planning_cmd_msg.header.stamp = ros::Time::now();
    }
    planning_cmd_msg.control_cmd = context->config.control_cmd;
    planning_cmd_msg.cmd_source = context->config.cmd_source;

    sunray_msgs::PlanningWaypoint waypoint;
    waypoint.position = transformed_goal.pose.position;
    waypoint.position.z = context->config.goal_height;
    waypoint.yaw = context->config.use_goal_yaw
                       ? quaternion_to_yaw(transformed_goal.pose.orientation)
                       : context->config.default_yaw;
    waypoint.hold_time = static_cast<float>(context->config.default_hold_time);

    planning_cmd_msg.waypoints.push_back(waypoint);
    context->planning_cmd_pub.publish(planning_cmd_msg);
    SUNRAY_INFO(
        "[sunray_planner_tools] goal bridge forwarded goal: {} -> {} pos=({:.3f}, {:.3f}, {:.3f}) yaw={:.3f} topic={}",
        msg->header.frame_id.empty() ? std::string("<empty>") : msg->header.frame_id,
        context->config.local_frame_id,
        waypoint.position.x,
        waypoint.position.y,
        waypoint.position.z,
        waypoint.yaw,
        context->planning_cmd_topic);
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "rviz_goal_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    try {
        BridgeContext context;
        private_nh.getParam("rviz_goal_topic", context.config.rviz_goal_topic);

        const std::string uav_ns = load_uav_namespace_or_throw(nh);
        context.planning_cmd_topic = make_planning_cmd_topic(uav_ns);
        context.tf_listener.reset(new tf2_ros::TransformListener(context.tf_buffer));
        context.planning_cmd_pub =
            nh.advertise<sunray_msgs::UAVPlanningCMD>(context.planning_cmd_topic, 10);

        ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
            context.config.rviz_goal_topic,
            10,
            boost::bind(&handle_goal, _1, &context));

        SUNRAY_INFO("[sunray_planner_tools] pose goal bridge: {} -> {} target_frame={} supported_frames=[{},{},{}] tf_timeout={}s",
                    context.config.rviz_goal_topic,
                    context.planning_cmd_topic,
                    context.config.local_frame_id,
                    context.config.local_frame_id,
                    context.config.world_frame_id,
                    context.config.global_frame_id,
                    context.config.tf_lookup_timeout_sec);

        ros::spin();
        return 0;
    } catch (const std::exception& ex) {
        ROS_ERROR("[sunray_planner_tools] failed to start rviz_goal_bridge: %s",
                  ex.what());
        return 1;
    }
}
