#include "mavros_helper/mavros_helper.hpp"
#include "agent_key_helper.hpp"
#include "eigen_helper.hpp"
#include <stdexcept>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <sunray_msgs/Px4State.h>

#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>

namespace {
// 此处为RAPTOR项目遗留，RAPTOR将会作为sunray_extra而非正式主线部分
constexpr uint8_t kPx4CustomMainModeAuto = 4;
constexpr uint8_t kPx4CustomSubModeExternal1 = 11;

uint32_t px4_external_mode_custom_mode(uint8_t external_mode_index) {
    if (external_mode_index < 1 || external_mode_index > 8) {
        return 0;
    }

    const uint8_t custom_sub_mode =
        static_cast<uint8_t>(kPx4CustomSubModeExternal1 + external_mode_index - 1);
    return (static_cast<uint32_t>(custom_sub_mode) << 24) |
           (static_cast<uint32_t>(kPx4CustomMainModeAuto) << 16);
}

std::string px4_external_mode_string(uint8_t external_mode_index) {
    return std::to_string(px4_external_mode_custom_mode(external_mode_index));
}

}  // namespace

MavrosHelper::MavrosHelper() {
    // 从节点私有命名空间中读取参数use_private_agent_key决定当前智能体标识符
    ros::NodeHandle private_nh("~");
    bool use_private_agent_key;
    private_nh.param("use_private_agent_key", use_private_agent_key, false);
    if (use_private_agent_key) {
        uav_ns_ = sunray_common::get_agent_key_from_private();
    } else {
        uav_ns_ = sunray_common::get_agent_key_from_global();
    }
}

void MavrosHelper::init() {
    // clang-format off
    state_sub_ = nh_.subscribe(uav_ns_ + "/mavros/state",
                                10,
                                &MavrosHelper::mavros_state_callback,
                                this);
    extended_state_sub_ = nh_.subscribe(uav_ns_ + "/mavros/extended_state",
                                        10,
                                        &MavrosHelper::mavros_externdedstate_callback,
                                        this);
    sys_sub_ = nh_.subscribe(uav_ns_ + "/mavros/sys_status",
                            10,
                            &MavrosHelper::mavros_sys_callback,
                            this);
    estimator_sub_ = nh_.subscribe(uav_ns_ + "/mavros/estimator_status",
                                    10,
                                    &MavrosHelper::mavros_estimator_callback,
                                    this);
    local_odom_sub_ = nh_.subscribe(uav_ns_ + "/mavros/local_position/odom",
                                    10,
                                    &MavrosHelper::mavros_localodom_callback,
                                    this);
    imu_sub_ =
        nh_.subscribe(uav_ns_ + "/mavros/imu/data", 10, &MavrosHelper::mavros_imu_callback, this);
    setpoint_local_sub_ = nh_.subscribe(uav_ns_ + "/mavros/setpoint_raw/target_local",
                                        10,
                                        &MavrosHelper::mavros_setpoint_local_callback,
                                        this);
    setpoint_attitude_sub_ = nh_.subscribe(uav_ns_ + "/mavros/setpoint_raw/target_attitude",
                                           10,
                                           &MavrosHelper::mavros_setpoint_attitude_callback,
                                           this);
    gps_raw_sub_ = nh_.subscribe(uav_ns_ + "/mavros/gpsstatus/gps1/raw",
                                  10,
                                  &MavrosHelper::mavros_gps_raw_callback,
                                  this);
    rc_in_sub_ = nh_.subscribe(uav_ns_ + "/mavros/rc/in",
                                10,
                                &MavrosHelper::mavros_rc_in_callback,
                                this);
    // clang-format on

    vision_pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(uav_ns_ + "/mavros/vision_pose/pose", 10);
    vision_odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(uav_ns_ + "/mavros/odometry/in", 10);
    setpoint_local_pub_ =
        nh_.advertise<mavros_msgs::PositionTarget>(uav_ns_ + "/mavros/setpoint_raw/local", 10);
    setpoint_attitude_pub_ =
        nh_.advertise<mavros_msgs::AttitudeTarget>(uav_ns_ + "/mavros/setpoint_raw/attitude", 10);
    px4_state_pub_ = nh_.advertise<sunray_msgs::Px4State>(uav_ns_ + "/sunray/px4_state", 10);

    px4_arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(uav_ns_ + "/mavros/cmd/arming");
    px4_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(uav_ns_ + "/mavros/set_mode");
    px4_cmdlong_client_ =
        nh_.serviceClient<mavros_msgs::CommandLong>(uav_ns_ + "/mavros/cmd/command");
    
    param_set_client_ =
      nh_.serviceClient<mavros_msgs::ParamSet>(uav_ns_ + "/mavros/param/set");
    param_get_client_ =
      nh_.serviceClient<mavros_msgs::ParamGet>(uav_ns_ + "/mavros/param/get");

    raptor_cmode_ = "CMODE(" + px4_external_mode_string(1) + ")";

    if (!vision_pose_pub_ || !vision_odometry_pub_ || !setpoint_local_pub_ ||
        !setpoint_attitude_pub_ || !px4_state_pub_) {
        throw std::runtime_error("[mavros_helper] Failed to create one or more publishers.");
    }
    if (!px4_arm_client_ || !px4_mode_client_ || !px4_cmdlong_client_ || !param_get_client_ || !param_set_client_) {
        throw std::runtime_error("[mavros_helper] Failed to create one or more service clients.");
    }
    if (state_sub_.getTopic().empty() || estimator_sub_.getTopic().empty() ||
        local_odom_sub_.getTopic().empty()) {
        throw std::runtime_error("[mavros_helper] Failed to create one or more subscribers.");
    }
    param_initialized_ = true;
}

control_common::Mavros_State MavrosHelper::get_state() const {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return mavros_state_data_;
}

control_common::Mavros_Estimator MavrosHelper::get_estimator_status() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return mavros_estimator_data_;
}

control_common::UAVStateEstimate MavrosHelper::get_odometry() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return mavros_odometry_data_;
}

control_common::Mavros_Pose MavrosHelper::get_local_pose() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return mavros_local_pose_data_;
}

control_common::Mavros_Velocity MavrosHelper::get_local_velocity() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return mavros_local_vel_data_;
}

control_common::Mavros_IMU MavrosHelper::get_imu_data() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return mavros_imu_data_;
}

Eigen::Quaterniond MavrosHelper::get_attitude_quat() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return mavros_attitude_data_;
}

Eigen::Vector3d MavrosHelper::get_attitude_euler_rad() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return eigen_helper::get_euler_from_orientation(mavros_attitude_data_);
}

Eigen::Vector3d MavrosHelper::get_attitude_euler_deg() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return eigen_helper::get_euler_from_orientation(mavros_attitude_data_) * (180.0 / M_PI);
}

double MavrosHelper::get_yaw_rad() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return eigen_helper::get_yaw_from_orientation(mavros_attitude_data_);
}

double MavrosHelper::get_yaw_deg() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return eigen_helper::get_yaw_from_orientation(mavros_attitude_data_) * 180.0 / M_PI;
}

control_common::Mavros_SetpointLocal MavrosHelper::get_target_local() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return mavros_setpoint_local_data_;
}

control_common::Mavros_SetpointAttitude MavrosHelper::get_target_attitude() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
    return mavros_setpoint_attitude_data_;
}

bool MavrosHelper::set_param_raw(const char* name, int value) {
    if (!param_initialized_) {
        return false;
    }

    mavros_msgs::ParamSet srv;
    srv.request.param_id = name;
    srv.request.value.integer = value;
    return param_set_client_.call(srv) && srv.response.success;
}

bool MavrosHelper::set_param_raw(const char* name, double value) {
    if (!param_initialized_) {
        return false;
    }

    mavros_msgs::ParamSet srv;
    srv.request.param_id = name;
    srv.request.value.real = static_cast<float>(value);
    return param_set_client_.call(srv) && srv.response.success;
}

bool MavrosHelper::read_param_raw(const char* name, int* value) {
    if (!param_initialized_ || value == nullptr) {
        return false;
    }

    mavros_msgs::ParamGet srv;
    srv.request.param_id = name;
    if (!param_get_client_.call(srv) || !srv.response.success) {
        return false;
    }

    *value = static_cast<int>(srv.response.value.integer);
    return true;
}

bool MavrosHelper::read_param_raw(const char* name, double* value) {
    if (!param_initialized_ || value == nullptr) {
        return false;
    }

    mavros_msgs::ParamGet srv;
    srv.request.param_id = name;
    if (!param_get_client_.call(srv) || !srv.response.success) {
        return false;
    }

    *value = static_cast<double>(srv.response.value.real);
    return true;
}

void MavrosHelper::set_vision_fuse_type(int fuse_type) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    if (fuse_type == 1) {
        fuse_vision_type_ = control_common::VisionFuseType::Vision_pose;
    } else if (fuse_type == 2) {
        fuse_vision_type_ = control_common::VisionFuseType::Odometry;
    } else {
        ROS_WARN("[mavros_helper] Invalid vision fuse type: %d. Expected 1 (Vision_pose) or 2 "
                 "(Odometry).",
                 fuse_type);
    }
}

bool MavrosHelper::pub_vision_pose(const control_common::UAVStateEstimate& uav_state) {
    control_common::VisionFuseType fuse_type;
    {
        std::unique_lock<std::shared_mutex> lock(data_mutex_);
        external_odometry_data_ = uav_state;
        fuse_type = fuse_vision_type_;
    }
    const ros::Time stamp = uav_state.timestamp.isZero() ? ros::Time::now() : uav_state.timestamp;

    if (fuse_type == control_common::VisionFuseType::Vision_pose) {
        geometry_msgs::PoseStamped vision_pose_data;
        vision_pose_data.header.stamp = stamp;
        vision_pose_data.header.frame_id = "odom";
        vision_pose_data.pose.position = eigen_helper::to_ros_point(uav_state.position);
        vision_pose_data.pose.orientation = eigen_helper::to_ros_quaternion(uav_state.orientation);
        vision_pose_pub_.publish(vision_pose_data);
        return true;
    }

    if (fuse_type == control_common::VisionFuseType::Odometry) {
        nav_msgs::Odometry vision_odometry_data;
        vision_odometry_data.header.stamp = stamp;
        vision_odometry_data.header.frame_id = "odom";
        vision_odometry_data.child_frame_id = "base_link";
        vision_odometry_data.pose.pose.position = eigen_helper::to_ros_point(uav_state.position);
        vision_odometry_data.pose.pose.orientation =
            eigen_helper::to_ros_quaternion(uav_state.orientation);
        vision_odometry_data.twist.twist.linear = eigen_helper::to_ros_vector3(uav_state.velocity);
        vision_odometry_data.twist.twist.angular = eigen_helper::to_ros_vector3(uav_state.bodyrate);
        vision_odometry_pub_.publish(vision_odometry_data);
        return true;
    }

    ROS_WARN_THROTTLE(
        1.0, "[mavros_helper] vision fuse type is undefined, skip publishing external odometry.");
    return false;
}

bool MavrosHelper::set_px4_mode(control_common::FlightMode flight_mode) {
    mavros_msgs::SetMode px4_mode;
    px4_mode.request.base_mode = 0;
    switch (flight_mode) {
    case control_common::FlightMode::Manual:
        px4_mode.request.custom_mode = "MANUAL";
        break;
    case control_common::FlightMode::Acro:
        px4_mode.request.custom_mode = "ACRO";
        break;
    case control_common::FlightMode::Altctl:
        px4_mode.request.custom_mode = "ALTCTL";
        break;
    case control_common::FlightMode::Posctl:
        px4_mode.request.custom_mode = "POSCTL";
        break;
    case control_common::FlightMode::Offboard:
        px4_mode.request.custom_mode = "OFFBOARD";
        break;
    case control_common::FlightMode::Stabilized:
        px4_mode.request.custom_mode = "STABILIZED";
        break;
    case control_common::FlightMode::Rattitude:
        px4_mode.request.custom_mode = "RATTITUDE";
        break;
    case control_common::FlightMode::AutoMission:
        px4_mode.request.custom_mode = "AUTO.MISSION";
        break;
    case control_common::FlightMode::AutoLoiter:
        px4_mode.request.custom_mode = "AUTO.LOITER";
        break;
    case control_common::FlightMode::AutoRtl:
        px4_mode.request.custom_mode = "AUTO.RTL";
        break;
    case control_common::FlightMode::AutoLand:
        px4_mode.request.custom_mode = "AUTO.LAND";
        break;
    case control_common::FlightMode::AutoRtgs:
        px4_mode.request.custom_mode = "AUTO.RTGS";
        break;
    case control_common::FlightMode::AutoReady:
        px4_mode.request.custom_mode = "AUTO.READY";
        break;
    case control_common::FlightMode::AutoTakeoff:
        px4_mode.request.custom_mode = "AUTO.TAKEOFF";
        break;
    case control_common::FlightMode::Raptor:
        return set_px4_external_mode(1);
    case control_common::FlightMode::Undefined:
        return false;
    default:
        return false;
    }
    return px4_mode_client_.call(px4_mode) && px4_mode.response.mode_sent;
}

bool MavrosHelper::set_px4_external_mode(uint8_t external_mode_index) {
    const uint32_t custom_mode = px4_external_mode_custom_mode(external_mode_index);
    if (custom_mode == 0) {
        return false;
    }

    mavros_msgs::SetMode px4_mode;
    px4_mode.request.base_mode = 0;
    px4_mode.request.custom_mode = std::to_string(custom_mode);
    return px4_mode_client_.call(px4_mode) && px4_mode.response.mode_sent;
}

bool MavrosHelper::set_arm(bool arm_state) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm_state;
    return px4_arm_client_.call(arm_cmd) && arm_cmd.response.success;
}

bool MavrosHelper::emergency_kill() {
    mavros_msgs::CommandLong srv;
    srv.request.broadcast = false;
    srv.request.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
    srv.request.confirmation = 0;
    srv.request.param1 = 0.0;      // disarm
    srv.request.param2 = 21196.0;  // force disarm magic number

    if (!px4_cmdlong_client_.exists()) {
        ROS_WARN("[mavros_helper] cmd_long service not ready for emergency_kill.");
        return false;
    }

    const bool call_ok = px4_cmdlong_client_.call(srv);
    if (!call_ok) {
        ROS_ERROR("[mavros_helper] emergency_kill call failed.");
        return false;
    }

    if (!srv.response.success) {
        ROS_ERROR("[mavros_helper] emergency_kill rejected by PX4. result=%d",
                  static_cast<int>(srv.response.result));
        return false;
    }

    ROS_ERROR("[mavros_helper] emergency_kill accepted.");
    return true;
}

bool MavrosHelper::reboot_px4() {
    mavros_msgs::CommandLong srv;
    srv.request.broadcast = false;
    srv.request.command = 246;  // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    srv.request.confirmation = 0;
    srv.request.param1 = 1.0;  // reboot autopilot
    srv.request.param2 = 0.0;
    srv.request.param3 = 0.0;
    srv.request.param4 = 0.0;
    srv.request.param5 = 0.0;
    srv.request.param6 = 0.0;
    srv.request.param7 = 0.0;

    if (!px4_cmdlong_client_.exists()) {
        ROS_WARN("[mavros_helper] cmd_long service not ready for reboot.");
        return false;
    }

    const bool call_ok = px4_cmdlong_client_.call(srv);
    if (!call_ok) {
        ROS_ERROR("[mavros_helper] reboot_px4 call failed.");
        return false;
    }

    if (!srv.response.success) {
        ROS_ERROR("[mavros_helper] reboot_px4 rejected by PX4. result=%d",
                  static_cast<int>(srv.response.result));
        return false;
    }

    ROS_WARN("[mavros_helper] reboot_px4 accepted. FCU will restart.");
    return true;
}

bool MavrosHelper::pub_local_setpoint(const control_common::Mavros_SetpointLocal& setpoint_local) {
    mavros_msgs::PositionTarget position_target_msg;
    position_target_msg.header.stamp = ros::Time::now();

    switch (setpoint_local.frame) {
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned:
        position_target_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        break;
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Offset_Ned:
        position_target_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_OFFSET_NED;
        break;
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Ned:
        position_target_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        break;
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Offset_Ned:
        position_target_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;
        break;
    }

    position_target_msg.type_mask = setpoint_local.mask;
    position_target_msg.position.x = setpoint_local.position.x();
    position_target_msg.position.y = setpoint_local.position.y();
    position_target_msg.position.z = setpoint_local.position.z();
    position_target_msg.velocity.x = setpoint_local.velocity.x();
    position_target_msg.velocity.y = setpoint_local.velocity.y();
    position_target_msg.velocity.z = setpoint_local.velocity.z();
    position_target_msg.acceleration_or_force.x = setpoint_local.accel_or_force.x();
    position_target_msg.acceleration_or_force.y = setpoint_local.accel_or_force.y();
    position_target_msg.acceleration_or_force.z = setpoint_local.accel_or_force.z();
    position_target_msg.yaw = setpoint_local.yaw;
    position_target_msg.yaw_rate = setpoint_local.yaw_rate;

    {
        std::unique_lock<std::shared_mutex> lock(data_mutex_);
        mavros_setpoint_local_data_ = setpoint_local;
        mavros_setpoint_local_data_.timestamp = position_target_msg.header.stamp;
        mavros_setpoint_local_data_.valid = true;
    }

    setpoint_local_pub_.publish(position_target_msg);
    return true;
}

bool MavrosHelper::pub_attitude_setpoint(
    const control_common::Mavros_SetpointAttitude& setpoint_attitude) {
    mavros_msgs::AttitudeTarget attitude_target_msg;
    attitude_target_msg.header.stamp = ros::Time::now();
    attitude_target_msg.type_mask = setpoint_attitude.mask;
    attitude_target_msg.orientation.x = setpoint_attitude.orientation.x();
    attitude_target_msg.orientation.y = setpoint_attitude.orientation.y();
    attitude_target_msg.orientation.z = setpoint_attitude.orientation.z();
    attitude_target_msg.orientation.w = setpoint_attitude.orientation.w();
    attitude_target_msg.body_rate.x = setpoint_attitude.body_rate.x();
    attitude_target_msg.body_rate.y = setpoint_attitude.body_rate.y();
    attitude_target_msg.body_rate.z = setpoint_attitude.body_rate.z();
    attitude_target_msg.thrust = setpoint_attitude.thrust;

    {
        std::unique_lock<std::shared_mutex> lock(data_mutex_);
        mavros_setpoint_attitude_data_ = setpoint_attitude;
        mavros_setpoint_attitude_data_.timestamp = attitude_target_msg.header.stamp;
        mavros_setpoint_attitude_data_.valid = true;
    }

    setpoint_attitude_pub_.publish(attitude_target_msg);
    return true;
}

bool MavrosHelper::pub_px4_state() {
    sunray_msgs::Px4State px4_state_msg;
    px4_state_msg.header.stamp = ros::Time::now();

    {
        std::shared_lock<std::shared_mutex> lock(data_mutex_);

        px4_state_msg.connected = mavros_state_data_.connected;
        px4_state_msg.rc_available = mavros_state_data_.rc_input;
        px4_state_msg.armed = mavros_state_data_.armed;
        px4_state_msg.flight_mode = static_cast<uint8_t>(mavros_state_data_.flight_mode);
        px4_state_msg.system_status = mavros_state_data_.system_status;
        px4_state_msg.rc_channels = mavros_rc_data_.channels;
        px4_state_msg.rc_rssi = mavros_rc_data_.rssi;
        px4_state_msg.landed_state = static_cast<uint8_t>(mavros_state_data_.landed_state);
        px4_state_msg.battery_voltage_v = mavros_state_data_.voltage;
        px4_state_msg.battery_current_a = mavros_state_data_.current;
        px4_state_msg.battery_percentage = mavros_state_data_.percent;
        px4_state_msg.fcu_load = mavros_state_data_.system_load;

        px4_state_msg.external_pose.position.x = external_odometry_data_.position.x();
        px4_state_msg.external_pose.position.y = external_odometry_data_.position.y();
        px4_state_msg.external_pose.position.z = external_odometry_data_.position.z();
        px4_state_msg.external_pose.orientation.w = external_odometry_data_.orientation.w();
        px4_state_msg.external_pose.orientation.x = external_odometry_data_.orientation.x();
        px4_state_msg.external_pose.orientation.y = external_odometry_data_.orientation.y();
        px4_state_msg.external_pose.orientation.z = external_odometry_data_.orientation.z();
        px4_state_msg.external_velocity.linear.x = external_odometry_data_.velocity.x();
        px4_state_msg.external_velocity.linear.y = external_odometry_data_.velocity.y();
        px4_state_msg.external_velocity.linear.z = external_odometry_data_.velocity.z();
        px4_state_msg.external_velocity.angular.x = external_odometry_data_.bodyrate.x();
        px4_state_msg.external_velocity.angular.y = external_odometry_data_.bodyrate.y();
        px4_state_msg.external_velocity.angular.z = external_odometry_data_.bodyrate.z();

        px4_state_msg.local_pose.position.x = mavros_odometry_data_.position.x();
        px4_state_msg.local_pose.position.y = mavros_odometry_data_.position.y();
        px4_state_msg.local_pose.position.z = mavros_odometry_data_.position.z();
        px4_state_msg.local_pose.orientation.w = mavros_odometry_data_.orientation.w();
        px4_state_msg.local_pose.orientation.x = mavros_odometry_data_.orientation.x();
        px4_state_msg.local_pose.orientation.y = mavros_odometry_data_.orientation.y();
        px4_state_msg.local_pose.orientation.z = mavros_odometry_data_.orientation.z();
        px4_state_msg.local_velocity.linear.x = mavros_odometry_data_.velocity.x();
        px4_state_msg.local_velocity.linear.y = mavros_odometry_data_.velocity.y();
        px4_state_msg.local_velocity.linear.z = mavros_odometry_data_.velocity.z();
        px4_state_msg.local_velocity.angular.x = mavros_odometry_data_.bodyrate.x();
        px4_state_msg.local_velocity.angular.y = mavros_odometry_data_.bodyrate.y();
        px4_state_msg.local_velocity.angular.z = mavros_odometry_data_.bodyrate.z();

        px4_state_msg.setpoint_coordinate_frame =
            static_cast<uint8_t>(mavros_setpoint_local_data_.frame);
        px4_state_msg.setpoint_local_type_mask = mavros_setpoint_local_data_.mask;
        px4_state_msg.pos_setpoint.x = mavros_setpoint_local_data_.position.x();
        px4_state_msg.pos_setpoint.y = mavros_setpoint_local_data_.position.y();
        px4_state_msg.pos_setpoint.z = mavros_setpoint_local_data_.position.z();
        px4_state_msg.vel_setpoint.x = mavros_setpoint_local_data_.velocity.x();
        px4_state_msg.vel_setpoint.y = mavros_setpoint_local_data_.velocity.y();
        px4_state_msg.vel_setpoint.z = mavros_setpoint_local_data_.velocity.z();
        px4_state_msg.acc_setpoint.x = mavros_setpoint_local_data_.accel_or_force.x();
        px4_state_msg.acc_setpoint.y = mavros_setpoint_local_data_.accel_or_force.y();
        px4_state_msg.acc_setpoint.z = mavros_setpoint_local_data_.accel_or_force.z();
        px4_state_msg.yaw_setpoint = mavros_setpoint_local_data_.yaw;
        px4_state_msg.yaw_rate_setpoint = mavros_setpoint_local_data_.yaw_rate;

        px4_state_msg.setpoint_att_type_mask = mavros_setpoint_attitude_data_.mask;
        px4_state_msg.orientation_setpoint.w = mavros_setpoint_attitude_data_.orientation.w();
        px4_state_msg.orientation_setpoint.x = mavros_setpoint_attitude_data_.orientation.x();
        px4_state_msg.orientation_setpoint.y = mavros_setpoint_attitude_data_.orientation.y();
        px4_state_msg.orientation_setpoint.z = mavros_setpoint_attitude_data_.orientation.z();
        px4_state_msg.body_rate_setpoint.x = mavros_setpoint_attitude_data_.body_rate.x();
        px4_state_msg.body_rate_setpoint.y = mavros_setpoint_attitude_data_.body_rate.y();
        px4_state_msg.body_rate_setpoint.z = mavros_setpoint_attitude_data_.body_rate.z();
        px4_state_msg.thrust_setpoint = mavros_setpoint_attitude_data_.thrust;

        px4_state_msg.satellites = mavros_gps_.satellites;
        px4_state_msg.gps_status = mavros_gps_.gps_status;
        px4_state_msg.gps_service = mavros_gps_.gps_service;
        px4_state_msg.latitude = mavros_gps_.latitude;
        px4_state_msg.longitude = mavros_gps_.longitude;
        px4_state_msg.altitude = mavros_gps_.altitude;
        px4_state_msg.latitude_raw = mavros_gps_.latitude_raw;
        px4_state_msg.longitude_raw = mavros_gps_.longitude_raw;
        px4_state_msg.altitude_amsl = mavros_gps_.altitude_amsl;
    }

    px4_state_pub_.publish(px4_state_msg);
    return true;
}

bool MavrosHelper::is_ready() {
    const ros::Time now_time = ros::Time::now();
    const ros::Duration timeout(1.5);
    bool ready = true;

    std::unique_lock<std::shared_mutex> lock(data_mutex_);

    mavros_state_data_.valid = (now_time - mavros_state_data_.timestamp) <= timeout;
    ready = ready && mavros_state_data_.valid;

    mavros_estimator_data_.valid = (now_time - mavros_estimator_data_.timestamp) <= timeout;
    ready = ready && mavros_estimator_data_.valid;

    ready = ready && ((now_time - mavros_odometry_data_.timestamp) <= timeout);

    lock.unlock();

    const bool control_handle_ready = vision_pose_pub_ && vision_odometry_pub_ &&
                                      setpoint_local_pub_ && setpoint_attitude_pub_ &&
                                      px4_arm_client_ && px4_mode_client_ && px4_cmdlong_client_;
    const bool param_handle_ready = !param_initialized_ || (param_set_client_ && param_get_client_);
    ready = ready && control_handle_ready && param_handle_ready;

    mavros_ready.store(ready);
    return ready;
}

/*----------------------------------mavros回调函数---------------------------*/
void MavrosHelper::mavros_state_callback(const mavros_msgs::State& msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    mavros_state_data_.update_from(msg, raptor_cmode_);
}

void MavrosHelper::mavros_externdedstate_callback(const mavros_msgs::ExtendedState& msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    mavros_state_data_.update_from(msg);
}

void MavrosHelper::mavros_sys_callback(const mavros_msgs::SysStatus& msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    mavros_state_data_.update_from(msg);
}

void MavrosHelper::mavros_estimator_callback(const mavros_msgs::EstimatorStatus& msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    mavros_estimator_data_ = control_common::Mavros_Estimator(msg);
}

void MavrosHelper::mavros_localodom_callback(const nav_msgs::Odometry& msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    mavros_odometry_data_ = control_common::UAVStateEstimate(msg);
    mavros_attitude_data_ = mavros_odometry_data_.orientation;
    mavros_local_pose_data_.position = mavros_odometry_data_.position;
    mavros_local_pose_data_.orientation = mavros_odometry_data_.orientation;
    mavros_local_vel_data_.linear = mavros_odometry_data_.velocity;
    mavros_local_vel_data_.angular = mavros_odometry_data_.bodyrate;
}

void MavrosHelper::mavros_imu_callback(const sensor_msgs::Imu& msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    mavros_imu_data_ = control_common::Mavros_IMU(msg);
}

void MavrosHelper::mavros_setpoint_local_callback(const mavros_msgs::PositionTarget& msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    mavros_setpoint_local_data_ = control_common::Mavros_SetpointLocal(msg);
}

void MavrosHelper::mavros_setpoint_attitude_callback(const mavros_msgs::AttitudeTarget& msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    mavros_setpoint_attitude_data_ = control_common::Mavros_SetpointAttitude(msg);
}

void MavrosHelper::mavros_gps_raw_callback(const mavros_msgs::GPSRAW& msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    mavros_gps_ = control_common::Mavros_GPS(msg);
}

void MavrosHelper::mavros_rc_in_callback(const mavros_msgs::RCIn& msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    mavros_rc_data_ = control_common::Mavros_RC(msg);
}
