#include "statemachine/sunray_fsm.hpp"
#include "utils/orientation_utils.hpp"
#include <iomanip>
#include <sstream>

namespace {

const char* bool_to_string(bool value) {
    return value ? "true" : "false";
}

const char* controller_type_to_string(uint8_t type) {
    switch (type) {
    case 0:
        return "PX4_OriginController";
    case 1:
        return "Geometric_Controller";
    case 2:
        return "Raptor_Controller";
    default:
        return "UnknownController";
    }
}

const char* sunray_state_to_string(sunray_fsm::SunrayState state) {
    switch (state) {
    case sunray_fsm::SunrayState::OFF:
        return "OFF";
    case sunray_fsm::SunrayState::INIT:
        return "INIT";
    case sunray_fsm::SunrayState::TAKEOFF:
        return "TAKEOFF";
    case sunray_fsm::SunrayState::HOVER:
        return "HOVER";
    case sunray_fsm::SunrayState::RETURN:
        return "RETURN";
    case sunray_fsm::SunrayState::LAND:
        return "LAND";
    case sunray_fsm::SunrayState::MOVE:
        return "MOVE";
    case sunray_fsm::SunrayState::EMERGENCY_KILL:
        return "EMERGENCY_KILL";
    default:
        return "UNKNOWN_STATE";
    }
}

const char* cmd_source_to_string(control_common::UavControlCmd::CmdSource source) {
    switch (source) {
    case control_common::UavControlCmd::CmdSource::UNDEFINE:
        return "UNDEFINE";
    case control_common::UavControlCmd::CmdSource::SUNRAY_STATION:
        return "SUNRAY_STATION";
    case control_common::UavControlCmd::CmdSource::RC_CONTROLLER:
        return "RC_CONTROLLER";
    case control_common::UavControlCmd::CmdSource::TERMINAL:
        return "TERMINAL";
    case control_common::UavControlCmd::CmdSource::CONTROL_CMD:
        return "CONTROL_CMD";
    default:
        return "UNKNOWN_SOURCE";
    }
}

const char* control_cmd_to_string(control_common::UavControlCmd::ControlCmd cmd) {
    switch (cmd) {
    case control_common::UavControlCmd::ControlCmd::UNDEFINE:
        return "UNDEFINE";
    case control_common::UavControlCmd::ControlCmd::TAKEOFF:
        return "TAKEOFF";
    case control_common::UavControlCmd::ControlCmd::LAND:
        return "LAND";
    case control_common::UavControlCmd::ControlCmd::RETURN:
        return "RETURN";
    case control_common::UavControlCmd::ControlCmd::KILL:
        return "KILL";
    case control_common::UavControlCmd::ControlCmd::HOVER:
        return "HOVER";
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT:
        return "MOVE_POINT";
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY:
        return "MOVE_VELOCITY";
    case control_common::UavControlCmd::ControlCmd::MOVE_TRAJECTORY:
        return "MOVE_TRAJECTORY";
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_BODY:
        return "MOVE_POINT_BODY";
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY_BODY:
        return "MOVE_VELOCITY_BODY";
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_WGS84:
        return "MOVE_POINT_WGS84";
    default:
        return "UNKNOWN_CMD";
    }
}

const char* yaw_mode_to_string(control_common::UavControlCmd::YawMode yaw_mode) {
    switch (yaw_mode) {
    case control_common::UavControlCmd::YawMode::KEEP_YAW:
        return "KEEP_YAW";
    case control_common::UavControlCmd::YawMode::SET_YAW:
        return "SET_YAW";
    case control_common::UavControlCmd::YawMode::SET_YAWRATE:
        return "SET_YAWRATE";
    default:
        return "UNKNOWN_YAW_MODE";
    }
}

std::string format_vec3(const Eigen::Vector3d& value, int precision = 3) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << "(" << value.x() << ", " << value.y()
        << ", " << value.z() << ")";
    return oss.str();
}

std::string format_vec2(const Eigen::Vector2d& value, int precision = 3) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << "(" << value.x() << ", " << value.y()
        << ")";
    return oss.str();
}

std::string format_control_target(const control_common::UavControlCmd& cmd) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    switch (cmd.control_cmd) {
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT:
        oss << "target_pos=" << format_vec3(cmd.position);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY:
        oss << "target_vel=" << format_vec3(cmd.velocity);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_TRAJECTORY:
        oss << "traj_pos=" << format_vec3(cmd.position) << " traj_vel=" << format_vec3(cmd.velocity)
            << " traj_acc=" << format_vec3(cmd.acceleration);
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_BODY:
        oss << "target_body_xy=" << format_vec2(cmd.body_position_xy)
            << " fixed_height=" << cmd.fixed_height;
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_VELOCITY_BODY:
        oss << "target_body_vel_xy=" << format_vec2(cmd.body_velocity_xy)
            << " fixed_height=" << cmd.fixed_height;
        break;
    case control_common::UavControlCmd::ControlCmd::MOVE_POINT_WGS84:
        oss << "target_wgs84=(" << cmd.wgs84_position.latitude << ", "
            << cmd.wgs84_position.longitude << ", " << cmd.wgs84_position.altitude << ")";
        break;
    default:
        oss << "target=n/a";
        break;
    }

    if (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAW) {
        oss << " yaw=" << cmd.yaw;
    } else if (cmd.yaw_mode == control_common::UavControlCmd::YawMode::SET_YAWRATE) {
        oss << " yaw_rate=" << cmd.yaw_rate;
    }

    return oss.str();
}

}  // namespace

void Sunray_FSM::show_static_info() {
    ROS_INFO("========== Sunray_FSM Static Info [%s] ==========", uav_ns_.c_str());
    ROS_INFO("controller=%s  odom_topic=%s",
             controller_type_to_string(fsm_config_.basic_param.controller_types),
             fsm_config_.basic_param.odom_topic_name.c_str());
    ROS_INFO("rate: controller=%.1f Hz  supervisor=%.1f Hz  fuse_odom[type=%u, freq=%.1f Hz]",
             fsm_config_.basic_param.controller_update_frequency,
             fsm_config_.basic_param.supervisor_update_frequency,
             fsm_config_.basic_param.fuse_odom_type,
             fsm_config_.basic_param.fuse_odom_frequency);
    ROS_INFO("vehicle: mass=%.2f kg  gravity=%.3f  hover_thrust=%.3f",
             fsm_config_.basic_param.mass_kg,
             fsm_config_.basic_param.gravity,
             fsm_config_.basic_param.hover_thrust_percent);
    ROS_INFO("takeoff: height=%.2f m  max_vel=%.2f m/s | land: type=%u  max_vel=%.2f m/s",
             fsm_config_.takeoff_land_param.takeoff_relative_height,
             fsm_config_.takeoff_land_param.takeoff_max_velocity,
             fsm_config_.takeoff_land_param.land_type,
             fsm_config_.takeoff_land_param.land_max_velocity);
    ROS_INFO("arrival judge: stable_time=%.2f s  pos_err=%.3f m  vel_err=%.3f m/s",
             fsm_config_.arrival_judge_param.judge_stabile_time_s,
             fsm_config_.arrival_judge_param.pos_stabile_err_m,
             fsm_config_.arrival_judge_param.vel_stabile_err_mps);
    ROS_INFO("timeout: odom=%.2f s  mavros=%.2f s  station=%.2f s",
             fsm_config_.msg_timeout_param.local_odometry,
             fsm_config_.msg_timeout_param.mavros_connect,
             fsm_config_.msg_timeout_param.sunray_station);
    ROS_INFO("protect: takeoff_with_code=%s  control_with_no_rc=%s  arm_with_code=%s  check_flip=%s  tilt_max=%.1f deg",
             bool_to_string(fsm_config_.protect_param.takeoff_with_code),
             bool_to_string(fsm_config_.protect_param.control_with_no_rc),
             bool_to_string(fsm_config_.protect_param.arm_with_code),
             bool_to_string(fsm_config_.protect_param.check_flip),
             fsm_config_.protect_param.tilt_angle_max);
    ROS_INFO("velocity limit: xyz=%s m/s  rc_xyz=%s m/s  yaw_rate=%.3f rad/s",
             format_vec3(fsm_config_.velocity_param.max_velocity).c_str(),
             format_vec3(fsm_config_.velocity_param.max_velocity_with_rc).c_str(),
             fsm_config_.velocity_param.yaw_rate);
    ROS_INFO("local fence: x=[%.2f, %.2f]  y=[%.2f, %.2f]  z=[%.2f, %.2f]",
             fsm_config_.local_fence_param.x_min,
             fsm_config_.local_fence_param.x_max,
             fsm_config_.local_fence_param.y_min,
             fsm_config_.local_fence_param.y_max,
             fsm_config_.local_fence_param.z_min,
             fsm_config_.local_fence_param.z_max);
    ROS_INFO("==================================================");
}

void Sunray_FSM::show_logs() {
    static ros::Time last_log_time = ros::Time(0);
    const ros::Time now = ros::Time::now();
    if (last_log_time != ros::Time(0) && (now - last_log_time).toSec() < 1.0) {
        return;
    }
    last_log_time = now;

    sunray_fsm::SunrayState state_snapshot;
    control_common::UAVStateEstimate odom_snapshot;
    control_common::UavControlCmd cmd_snapshot;
    std::size_t queue_size = 0;

    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        state_snapshot = fsm_state_;
    }
    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        odom_snapshot = last_odometry_;
    }
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        cmd_snapshot = last_control_cmd_;
    }
    {
        std::lock_guard<std::mutex> lk(event_mutex_);
        queue_size = fsm_event_queue_.size();
    }

    const double now_s = now.toSec();
    const double odom_age_s =
        (odom_snapshot.timestamp == ros::Time(0)) ? -1.0 : now_s - odom_snapshot.timestamp.toSec();
    const double cmd_age_s =
        (cmd_snapshot.timestamp == ros::Time(0)) ? -1.0 : now_s - cmd_snapshot.timestamp.toSec();
    const double yaw_deg = quaternion_to_yaw_rad(odom_snapshot.orientation) * 180.0 / M_PI;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "\n[Sunray_FSM][" << uav_ns_ << "] state=" << sunray_state_to_string(state_snapshot)
        << " cmd=" << control_cmd_to_string(cmd_snapshot.control_cmd)
        << " src=" << cmd_source_to_string(cmd_snapshot.cmd_source)
        << " yaw_mode=" << yaw_mode_to_string(cmd_snapshot.yaw_mode) << " queue=" << queue_size
        << "\n  flags: controller_ready=" << bool_to_string(controller_ready_)
        << " allow_takeoff=" << bool_to_string(allow_takeoff_)
        << " control_msg_lost=" << bool_to_string(control_msg_lost_)
        << " rc_connected=" << bool_to_string(rc_connected)
        << " is_flip=" << bool_to_string(is_flip_) << " is_fence=" << bool_to_string(is_fence_)
        << "\n  odom: pos=" << format_vec3(odom_snapshot.position)
        << " vel=" << format_vec3(odom_snapshot.velocity) << " yaw_deg=" << yaw_deg
        << " odom_age=" << odom_age_s << " s"
        << "\n  home: " << format_vec3(home_point_) << "  cmd_age=" << cmd_age_s
        << " s  " << format_control_target(cmd_snapshot);

    ROS_INFO_STREAM(oss.str());
    if (sunray_controller_) {
        sunray_controller_->printf_logs();
    }
}
