#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sunray_msgs/PlanningWaypoint.h>
#include <sunray_msgs/UAVPlanningCMD.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "agent_key_helper.hpp"
#include "eigen_helper.hpp"
#include "sunray_log.hpp"

namespace {

constexpr char kWorldFrameId[] = "world";
constexpr char kLocalPlanFrameMode[] = "local";
constexpr char kGlobalPlanFrameMode[] = "global";
constexpr char kDefaultLocalFrameTemplate[] = "${agent_key}/sunray_local";
constexpr char kDefaultGlobalFrameTemplate[] = "${agent_key}/sunray_global";
constexpr char kDefaultLocalOdomTopicTemplate[] = "${agent_key}/sunray/localization/local_odom";

enum class GoalPlanMode {
    Local,
    Global
};

struct BridgeConfig {
    std::string rviz_topic{"/move_base_simple/goal"};
    std::string local_frame_id{kDefaultLocalFrameTemplate};
    std::string global_frame_id{kDefaultGlobalFrameTemplate};
    std::string world_goal_plan_frame{kGlobalPlanFrameMode};
    bool use_agent_height{true};
    double fixed_goal_height{0.6};
    std::string agent_odom_topic{kDefaultLocalOdomTopicTemplate};
    double tf_lookup_timeout_sec{0.1};
    double default_hold_time{0.0};
    std::string plan_cmd_source{"RVIZ"};
};

struct GoalCommand {
    geometry_msgs::PoseStamped goal;
    GoalPlanMode plan_mode{GoalPlanMode::Local};
};

struct BridgeContext {
    BridgeConfig config;
    std::string agent_key;
    std::string planning_cmd_topic;
    ros::Publisher planning_cmd_pub;
    tf2_ros::Buffer tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    nav_msgs::Odometry latest_agent_odom;
    bool has_agent_odom{false};
};

std::string strip_leading_slash(std::string value) {
    while (!value.empty() && value.front() == '/') {
        value.erase(value.begin());
    }
    return value;
}

std::string make_planning_cmd_topic(const std::string& agent_key) {
    return agent_key + "/sunray/uav_planning/planning_cmd";
}

std::string plan_mode_to_string(const GoalPlanMode plan_mode) {
    return plan_mode == GoalPlanMode::Global ? kGlobalPlanFrameMode : kLocalPlanFrameMode;
}

uint8_t plan_mode_to_plan_cmd(const GoalPlanMode plan_mode) {
    return plan_mode == GoalPlanMode::Global ? sunray_msgs::UAVPlanningCMD::PLAN_GLOBAL_GOAL
                                             : sunray_msgs::UAVPlanningCMD::PLAN_LOCAL_GOAL;
}

const std::string& plan_mode_to_frame_id(const GoalPlanMode plan_mode,
                                         const BridgeContext& context) {
    return plan_mode == GoalPlanMode::Global ? context.config.global_frame_id
                                             : context.config.local_frame_id;
}

GoalPlanMode parse_world_goal_plan_mode(const std::string& raw_mode) {
    if (raw_mode == kGlobalPlanFrameMode) {
        return GoalPlanMode::Global;
    }
    if (raw_mode == kLocalPlanFrameMode) {
        return GoalPlanMode::Local;
    }
    throw std::runtime_error("invalid world_goal_plan_frame: " + raw_mode +
                             ", expected 'global' or 'local'");
}

bool transform_pose_to_target_frame(const geometry_msgs::PoseStamped& input_pose,
                                    const std::string& target_frame_id,
                                    const std::string& source_tag,
                                    geometry_msgs::PoseStamped& output_pose,
                                    const BridgeContext& context) {
    output_pose = input_pose;

    const std::string input_frame = strip_leading_slash(input_pose.header.frame_id);
    const std::string normalized_target_frame = strip_leading_slash(target_frame_id);
    if (input_frame.empty()) {
        SUNRAY_WARN("[sunray_planner_tools] {} rejected: empty frame_id", source_tag);
        return false;
    }
    if (normalized_target_frame.empty()) {
        SUNRAY_WARN("[sunray_planner_tools] {} rejected: empty target_frame_id", source_tag);
        return false;
    }
    output_pose.header.frame_id = input_frame;

    if (input_frame == normalized_target_frame) {
        return true;
    }

    const ros::Time transform_time =
        input_pose.header.stamp.isZero() ? ros::Time(0) : input_pose.header.stamp;
    const ros::Duration timeout(context.config.tf_lookup_timeout_sec);

    if (!context.tf_buffer.canTransform(
            normalized_target_frame, input_frame, transform_time, timeout)) {
        SUNRAY_WARN("[sunray_planner_tools] {} cannot transform {} -> {} within {:.3f}s",
                    source_tag,
                    input_frame,
                    normalized_target_frame,
                    context.config.tf_lookup_timeout_sec);
        return false;
    }

    try {
        context.tf_buffer.transform(output_pose, output_pose, normalized_target_frame, timeout);
        return true;
    } catch (const tf2::TransformException& ex) {
        SUNRAY_WARN("[sunray_planner_tools] {} transform failed: {} -> {} error={}",
                    source_tag,
                    input_frame,
                    normalized_target_frame,
                    ex.what());
        return false;
    }
}

bool resolve_goal_command(const geometry_msgs::PoseStamped& input_goal,
                          GoalCommand& goal_command,
                          const BridgeContext& context) {
    const std::string input_frame = strip_leading_slash(input_goal.header.frame_id);
    if (input_frame.empty()) {
        SUNRAY_WARN("[sunray_planner_tools] goal bridge rejected goal from {}: empty frame_id",
                    context.config.rviz_topic);
        return false;
    }

    geometry_msgs::PoseStamped normalized_goal = input_goal;
    normalized_goal.header.frame_id = input_frame;

    if (input_frame == context.config.local_frame_id) {
        goal_command.goal = normalized_goal;
        goal_command.plan_mode = GoalPlanMode::Local;
        return true;
    }

    if (input_frame == context.config.global_frame_id) {
        goal_command.goal = normalized_goal;
        goal_command.plan_mode = GoalPlanMode::Global;
        return true;
    }

    if (input_frame != kWorldFrameId) {
        SUNRAY_WARN("[sunray_planner_tools] goal bridge rejected unsupported frame_id={} from {}, supported=[{},{},{}]",
                    input_frame,
                    context.config.rviz_topic,
                    context.config.local_frame_id,
                    context.config.global_frame_id,
                    kWorldFrameId);
        return false;
    }

    goal_command.plan_mode = parse_world_goal_plan_mode(context.config.world_goal_plan_frame);
    const std::string& target_frame_id = plan_mode_to_frame_id(goal_command.plan_mode, context);
    if (!transform_pose_to_target_frame(
            normalized_goal, target_frame_id, "world goal", goal_command.goal, context)) {
        return false;
    }
    return true;
}

bool resolve_goal_height(const GoalPlanMode plan_mode,
                         const BridgeContext& context,
                         double& goal_height) {
    if (!context.config.use_agent_height) {
        goal_height = context.config.fixed_goal_height;
        return true;
    }

    if (!context.has_agent_odom) {
        SUNRAY_WARN("[sunray_planner_tools] goal bridge cannot use agent height because odom has not been received yet from {}",
                    context.config.agent_odom_topic);
        return false;
    }

    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header = context.latest_agent_odom.header;
    odom_pose.pose = context.latest_agent_odom.pose.pose;

    geometry_msgs::PoseStamped transformed_odom_pose;
    const std::string& target_frame_id = plan_mode_to_frame_id(plan_mode, context);
    if (!transform_pose_to_target_frame(
            odom_pose, target_frame_id, "agent odom height", transformed_odom_pose, context)) {
        return false;
    }

    goal_height = transformed_odom_pose.pose.position.z;
    return true;
}

void handle_agent_odom(const nav_msgs::Odometry::ConstPtr& msg, BridgeContext* context) {
    if (!msg || context == nullptr) {
        return;
    }
    context->latest_agent_odom = *msg;
    context->has_agent_odom = true;
}

void handle_goal(const geometry_msgs::PoseStamped::ConstPtr& msg, BridgeContext* context) {
    if (!msg || context == nullptr) {
        return;
    }

    GoalCommand goal_command;
    if (!resolve_goal_command(*msg, goal_command, *context)) {
        return;
    }

    double goal_height = 0.0;
    if (!resolve_goal_height(goal_command.plan_mode, *context, goal_height)) {
        return;
    }

    sunray_msgs::UAVPlanningCMD planning_cmd_msg;
    planning_cmd_msg.header = goal_command.goal.header;
    planning_cmd_msg.header.frame_id = plan_mode_to_frame_id(goal_command.plan_mode, *context);
    if (planning_cmd_msg.header.stamp.isZero()) {
        planning_cmd_msg.header.stamp = ros::Time::now();
    }
    planning_cmd_msg.plan_cmd = plan_mode_to_plan_cmd(goal_command.plan_mode);
    planning_cmd_msg.plan_cmd_source = context->config.plan_cmd_source;

    sunray_msgs::PlanningWaypoint waypoint;
    waypoint.position = goal_command.goal.pose.position;
    waypoint.position.z = goal_height;
    const Eigen::Quaterniond goal_orientation(goal_command.goal.pose.orientation.w,
                                              goal_command.goal.pose.orientation.x,
                                              goal_command.goal.pose.orientation.y,
                                              goal_command.goal.pose.orientation.z);
    waypoint.yaw = eigen_helper::get_yaw_from_orientation(goal_orientation);
    waypoint.hold_time = static_cast<float>(context->config.default_hold_time);

    planning_cmd_msg.waypoints.push_back(waypoint);
    context->planning_cmd_pub.publish(planning_cmd_msg);
    SUNRAY_INFO("[sunray_planner_tools] goal bridge forwarded goal: {} -> {} cmd={} pos=({:.3f}, {:.3f}, {:.3f}) yaw={:.3f} topic={}",
                msg->header.frame_id,
                planning_cmd_msg.header.frame_id,
                plan_mode_to_string(goal_command.plan_mode),
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
        private_nh.param("rviz_topic", context.config.rviz_topic, context.config.rviz_topic);
        private_nh.param(
            "sunray_local_frame", context.config.local_frame_id, context.config.local_frame_id);
        private_nh.param("sunray_global_frame",
                         context.config.global_frame_id,
                         context.config.global_frame_id);
        private_nh.param("world_goal_plan_frame",
                         context.config.world_goal_plan_frame,
                         context.config.world_goal_plan_frame);
        private_nh.param(
            "use_agent_height", context.config.use_agent_height, context.config.use_agent_height);
        private_nh.param("fixed_goal_height",
                         context.config.fixed_goal_height,
                         context.config.fixed_goal_height);
        private_nh.param(
            "agent_odom_topic", context.config.agent_odom_topic, context.config.agent_odom_topic);

        bool use_private_agent_key = false;
        private_nh.param("use_private_agent_key", use_private_agent_key, false);
        context.agent_key = use_private_agent_key ? sunray_common::get_agent_key_from_private()
                                                  : sunray_common::get_agent_key_from_global();

        context.config.local_frame_id =
            strip_leading_slash(
                sunray_common::replace_agent_key(context.config.local_frame_id, context.agent_key));
        context.config.global_frame_id =
            strip_leading_slash(sunray_common::replace_agent_key(context.config.global_frame_id,
                                                                 context.agent_key));
        context.config.agent_odom_topic =
            sunray_common::replace_agent_key(context.config.agent_odom_topic, context.agent_key);

        if (context.config.local_frame_id.empty() || context.config.global_frame_id.empty()) {
            throw std::runtime_error("expanded sunray frame id is empty");
        }
        if (context.config.agent_odom_topic.empty()) {
            throw std::runtime_error("expanded agent odom topic is empty");
        }

        parse_world_goal_plan_mode(context.config.world_goal_plan_frame);

        context.planning_cmd_topic = make_planning_cmd_topic(context.agent_key);
        context.tf_listener.reset(new tf2_ros::TransformListener(context.tf_buffer));
        context.planning_cmd_pub =
            nh.advertise<sunray_msgs::UAVPlanningCMD>(context.planning_cmd_topic, 10);

        ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
            context.config.rviz_topic,
            10,
            boost::bind(&handle_goal, _1, &context));

        ros::Subscriber agent_odom_sub;
        if (context.config.use_agent_height) {
            agent_odom_sub = nh.subscribe<nav_msgs::Odometry>(
                context.config.agent_odom_topic,
                10,
                boost::bind(&handle_agent_odom, _1, &context));
        }

        SUNRAY_INFO("[sunray_planner_tools] pose goal bridge: rviz_topic={} planning_cmd_topic={} local_frame={} global_frame={} world_goal_plan_frame={} use_agent_height={} fixed_goal_height={} odom_topic={}",
                    context.config.rviz_topic,
                    context.planning_cmd_topic,
                    context.config.local_frame_id,
                    context.config.global_frame_id,
                    context.config.world_goal_plan_frame,
                    context.config.use_agent_height,
                    context.config.fixed_goal_height,
                    context.config.agent_odom_topic);

        ros::spin();
        return 0;
    } catch (const std::exception& ex) {
        ROS_ERROR("[sunray_planner_tools] failed to start rviz_goal_bridge: %s", ex.what());
        return 1;
    }
}
