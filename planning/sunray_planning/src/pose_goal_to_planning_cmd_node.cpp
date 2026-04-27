#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sunray_msgs/PlanningWaypoint.h>
#include <sunray_msgs/UAVPlanningCMD.h>

#include "sunray_log.hpp"

namespace {
double quaternion_to_yaw(const geometry_msgs::Quaternion& orientation) {
    const double siny_cosp =
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    const double cosy_cosp =
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::string parent_directory(const std::string& path) {
    const std::size_t pos = path.find_last_of('/');
    return (pos == std::string::npos) ? "" : path.substr(0, pos);
}

std::string sanitize_path_token(std::string token) {
    std::replace(token.begin(), token.end(), '/', '_');
    return token.empty() ? "bridge" : token;
}

std::string make_startup_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_r(&now_time, &tm_now);

    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << "_" << std::setfill('0') << std::setw(3)
        << milliseconds.count();
    return oss.str();
}

int control_cmd_from_legacy_planning_frame(const int planning_frame) {
    switch (planning_frame) {
    case 1:
        return static_cast<int>(sunray_msgs::UAVPlanningCMD::PLANNING_LOCAL);
    case 2:
        return static_cast<int>(sunray_msgs::UAVPlanningCMD::PLANNING_GLOBAL);
    default:
        return static_cast<int>(sunray_msgs::UAVPlanningCMD::PLANNING_LOCAL);
    }
}

int control_cmd_from_legacy_exec_mode(const int exec_mode) {
    switch (exec_mode) {
    case 2:
        return static_cast<int>(sunray_msgs::UAVPlanningCMD::PLANNING_LOCAL);
    case 3:
        return static_cast<int>(sunray_msgs::UAVPlanningCMD::PLANNING_GLOBAL);
    default:
        return static_cast<int>(sunray_msgs::UAVPlanningCMD::PLANNING_LOCAL);
    }
}

class PoseGoalToPlanningCmdBridge {
  public:
    PoseGoalToPlanningCmdBridge() : nh_(), private_nh_("~") {
        private_nh_.param("log_save", log_save_, false);
        init_logger();
        private_nh_.param("goal_sub_topic",
                          goal_sub_topic_,
                          std::string("/move_base_simple/goal"));
        private_nh_.param("planning_cmd_pub_topic",
                          planning_cmd_pub_topic_,
                          std::string("/uav1/sunray/planning_cmd"));
        if (!private_nh_.getParam("control_cmd", control_cmd_)) {
            int legacy_planning_frame = 0;
            if (private_nh_.getParam("planning_frame", legacy_planning_frame)) {
                control_cmd_ = control_cmd_from_legacy_planning_frame(legacy_planning_frame);
                SUNRAY_WARN("[sunray_planning] pose goal bridge param '~planning_frame' is deprecated, use '~control_cmd' instead");
            } else {
                int legacy_exec_mode = 0;
                if (private_nh_.getParam("exec_mode", legacy_exec_mode)) {
                    control_cmd_ = control_cmd_from_legacy_exec_mode(legacy_exec_mode);
                    SUNRAY_WARN("[sunray_planning] pose goal bridge param '~exec_mode' is deprecated, use '~control_cmd' instead");
                } else {
                    control_cmd_ =
                        static_cast<int>(sunray_msgs::UAVPlanningCMD::PLANNING_LOCAL);
                }
            }
        }
        private_nh_.param("cmd_source",
                          cmd_source_,
                          static_cast<int>(sunray_msgs::UAVPlanningCMD::SUNRAY_STATION));
        private_nh_.param("goal_height", goal_height_, 0.6);
        private_nh_.param("default_hold_time", default_hold_time_, 0.0);
        private_nh_.param("default_yaw", default_yaw_, 0.0);
        private_nh_.param("use_goal_yaw", use_goal_yaw_, true);

        goal_sub_ = nh_.subscribe(
            goal_sub_topic_, 10, &PoseGoalToPlanningCmdBridge::goal_callback, this);
        planning_cmd_pub_ =
            nh_.advertise<sunray_msgs::UAVPlanningCMD>(planning_cmd_pub_topic_, 10);

        SUNRAY_INFO("[sunray_planning] pose goal bridge: {} -> {} control_cmd={} goal_height={}",
                    goal_sub_topic_,
                    planning_cmd_pub_topic_,
                    control_cmd_,
                    goal_height_);
    }

  private:
    void init_logger() {
        SunrayLogConfig cfg;
        cfg.name = "sunray_planning_bridge_" + sanitize_path_token(ros::this_node::getName());
        cfg.console_level = SunrayLogLevel::info;
        cfg.file_level = SunrayLogLevel::trace;
        cfg.async = false;

        if (log_save_) {
            log_file_path_ = make_log_file_path();
            cfg.file_path = log_file_path_;
        }

        SunrayLogger::instance().Init(cfg);
        auto logger = SunrayLogger::instance().Get();
        if (logger && !logger->sinks().empty()) {
            logger->sinks().front()->set_pattern("%v");
        }

        SUNRAY_INFO("[sunray_planning] pose goal bridge logger initialized. log_save={} file_path={}",
                    log_save_,
                    log_save_ ? log_file_path_ : "disabled");
    }

    std::string make_log_file_path() const {
        const std::string package_path = ros::package::getPath("sunray_planning");
        if (package_path.empty()) {
            throw std::runtime_error("failed to locate package path for sunray_planning");
        }

        const std::string planning_root = parent_directory(package_path);
        const std::string repo_root = parent_directory(planning_root);
        if (repo_root.empty()) {
            throw std::runtime_error("failed to infer repository root from package path: " +
                                     package_path);
        }

        return repo_root + "/log/sunray_planning/" +
               sanitize_path_token(ros::this_node::getName()) + "_" + make_startup_timestamp() +
               ".log";
    }

    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!msg) {
            return;
        }

        sunray_msgs::UAVPlanningCMD planning_cmd_msg;
        planning_cmd_msg.header = msg->header;
        if (planning_cmd_msg.header.stamp.isZero()) {
            planning_cmd_msg.header.stamp = ros::Time::now();
        }
        planning_cmd_msg.control_cmd = static_cast<uint8_t>(control_cmd_);
        planning_cmd_msg.cmd_source = static_cast<uint8_t>(cmd_source_);

        sunray_msgs::PlanningWaypoint waypoint;
        waypoint.position = msg->pose.position;
        waypoint.position.z = goal_height_;
        waypoint.yaw = use_goal_yaw_ ? quaternion_to_yaw(msg->pose.orientation) : default_yaw_;
        waypoint.hold_time = static_cast<float>(default_hold_time_);

        planning_cmd_msg.waypoints.push_back(waypoint);
        planning_cmd_pub_.publish(planning_cmd_msg);
        SUNRAY_INFO(
            "[sunray_planning] goal bridge forwarded goal: topic={} pos=({:.3f}, {:.3f}, {:.3f}) yaw={:.3f} -> {}",
            goal_sub_topic_,
            waypoint.position.x,
            waypoint.position.y,
            waypoint.position.z,
            waypoint.yaw,
            planning_cmd_pub_topic_);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber goal_sub_;
    ros::Publisher planning_cmd_pub_;

    std::string goal_sub_topic_;
    std::string planning_cmd_pub_topic_;
    int control_cmd_{sunray_msgs::UAVPlanningCMD::PLANNING_LOCAL};
    int cmd_source_{sunray_msgs::UAVPlanningCMD::SUNRAY_STATION};
    double goal_height_{0.6};
    double default_hold_time_{0.0};
    double default_yaw_{0.0};
    bool use_goal_yaw_{true};
    bool log_save_{false};
    std::string log_file_path_;
};
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_goal_to_planning_cmd_node");
    PoseGoalToPlanningCmdBridge bridge;
    ros::spin();
    return 0;
}
