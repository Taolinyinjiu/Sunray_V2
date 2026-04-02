#include "mavros_helper/mavros_helper.hpp"
#include "string_uav_namespace_utils.hpp"
#include <stdexcept>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <sunray_msgs/Px4State.h>
#include <controller/utils.hpp>
// 构造函数
MavrosHelper::MavrosHelper(ros::NodeHandle& nh) {
    // 缓存句柄
    nh_ = nh;
    // 使用全局参数构造uav_ns_
    std::string uav_name;
    int uav_id;
    if (!nh.getParam("/uav_name", uav_name)) {
        // 抛出异常
        throw std::runtime_error("missing param /uav_name");
    }
    if (!nh.getParam("/uav_id", uav_id)) {
        // 读取失败，抛出异常
        throw std::runtime_error("missing param /uav_id");
    }
    // 拼接uav_ns
    uav_ns_ = uav_name + std::to_string(uav_id);
    // 标准化
    uav_ns_ = sunray_common::normalize_uav_ns(uav_ns_);
    // clang-format on
}

bool MavrosHelper::init(MavrosHelper_ConfigList config_list) {
    // 本来设计了一个表，然后使用迭代器遍历的，但是想了一下，可读性没有直接用if高，所以还是用if吧
    // 首先还是要缓存一下这个config_list，这样做的好处是，提高结构体中的valid语义
    config_cache_ = config_list;
    // if手动遍历，判断状态，然后初始化订阅者
    // 由于代码格式化的原因，下面的初始化语句有的会是一行，有的又会拆成很多行，因此禁止插件格式化，并手动排版提高可读性
    // clang-format off
    if (config_list.state == true) {
        state_sub_ =nh_.subscribe(uav_ns_ + "/mavros/state",
                                10,
                                &MavrosHelper::mavros_state_callback,
                                this);
        extended_state_sub_ = nh_.subscribe(uav_ns_ + "/mavros/extended_state",
                                        10,
                                        &MavrosHelper::mavros_externdedstate_callback,
                                        this);
        sys_sub_ = nh_.subscribe(uav_ns_+"/mavros/sys_status",
                                10,
                                &MavrosHelper::mavros_sys_callback,
                                this);
    }
    if (config_list.ekf2_status == true) {
        estimator_sub_ = nh_.subscribe(uav_ns_ + "/mavros/estimator_status",
                                    10,
                                    &MavrosHelper::mavros_estimator_callback,
                                    this);
    }
    if(config_list.local_odom == true){
        local_odom_sub_ = nh_.subscribe(uav_ns_+"/mavros/local_position/odom",
                                    10,
                                    &MavrosHelper::mavros_localodom_callback,
                                    this);
    }
    if(config_list.uav_target_state == true){
        setpoint_local_sub_ = nh_.subscribe(uav_ns_+"/mavros/setpoint_raw/target_local",
                                        10,
                                        &MavrosHelper::mavros_setpoint_local_callback,
                                        this);
        setpoint_attitude_sub_ = nh_.subscribe(uav_ns_+"/mavros/setpoint_raw/target_attitude",
                                        10,
                                        &MavrosHelper::mavros_setpoint_attitude_callback,
                                        this);
    }
    // 接下来是发布
    if(config_list.uav_control_handle){
        // 首先是vision_pose
        vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(uav_ns_+"/mavros/vision_pose/pose",10);
        vision_odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(uav_ns_+"/mavros/odometry/in",10);
        // 然后是setpoint
        setpoint_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(uav_ns_+"/mavros/setpoint_raw/local",10);
        setpoint_attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(uav_ns_+"/mavros/setpoint_raw/attitude",10);
        // 然后是服务端
        px4_arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(uav_ns_+"/mavros/cmd/arming");
        px4_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(uav_ns_ + "/mavros/set_mode");
        px4_cmdlong_client_ = nh_.serviceClient<mavros_msgs::CommandLong>(uav_ns_ + "/mavros/cmd/command");
        // 最后是PX4State
        px4_state_pub_ = nh_.advertise<sunray_msgs::Px4State>(uav_ns_ + "/sunray/px4_state", 10);
    }
    return true;
}
// clang-format on
control_common::Mavros_State MavrosHelper::get_state() {
    // 检查数据是否有效？由于函数的返回类型是一个结构体，因此我无法在这里说，无效不返回之类的，
    // 同时，函数也无法说同名函数不同返回类型之类的，so这里的最后决定是
    // 直接返回缓存，如果后面测试发现需要添加锁，就再加上锁
    // 数据的有效性通过结构体中的valid字段决定
    return mavros_state_data_;
}

control_common::Mavros_Estimator MavrosHelper::get_estimator_status() {
    return mavros_estimator_data_;
}

control_common::UAVStateEstimate MavrosHelper::get_odometry() {
    return mavros_odometry_data_;
}

control_common::Mavros_Pose MavrosHelper::get_local_pose() {
    return mavros_local_pose_data_;
}

control_common::Mavros_Velocity MavrosHelper::get_local_velocity() {
    return mavros_local_vel_data_;
}

Eigen::Quaterniond MavrosHelper::get_attitude_quat() {
    return mavros_attitude_data_;
}

Eigen::Vector3d MavrosHelper::get_attitude_eluer_rad() {
    // 计算的顺序为roll pitch yaw
    auto euler = mavros_attitude_data_.toRotationMatrix().eulerAngles(0, 1, 2);
    return euler;
}

Eigen::Vector3d MavrosHelper::get_attitude_eluer_deg() {
    // 计算的顺序为roll pitch yaw
    auto euler = mavros_attitude_data_.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector3d degress = euler * (180.0 / M_PI);
    return degress;
}

double MavrosHelper::get_yaw_rad() {
    Eigen::Quaterniond& q = mavros_attitude_data_;  // 引用类成员，简化下面的运算
    double yaw =
        atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    return yaw;
}

double MavrosHelper::get_yaw_deg() {
    Eigen::Quaterniond& q = mavros_attitude_data_;  // 引用类成员，简化下面的运算
    double yaw =
        atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    double yaw_deg = yaw * 180.0 / M_PI;
    return yaw_deg;
}

control_common::Mavros_SetpointLocal MavrosHelper::get_target_local() {
    return mavros_setpoint_local_data_;
}

control_common::Mavros_SetpointAttitude MavrosHelper::get_target_attitude() {
    return mavros_setpoint_attitude_data_;
}

void MavrosHelper::set_vision_fuse_type(int fuse_type) {
    if (fuse_type == 1) {
        fuse_vision_type_ = control_common::VisionFuseType::Vision_pose;
    } else if (fuse_type == 2) {
        fuse_vision_type_ = control_common::VisionFuseType::Odometry;
    }
}

bool MavrosHelper::pub_vision_pose(control_common::UAVStateEstimate uav_state) {
    // 先将数据拷贝一份到类成员，给后面px4state使用
    external_odometry_data_ = uav_state;
    // 首先判断是否有运动控制权限
    if (config_cache_.uav_control_handle == false) {
        return false;
    }
    // 根据fuse_odom_type 构造类型
    if (fuse_vision_type_ == control_common::VisionFuseType::Vision_pose) {
        geometry_msgs::PoseStamped vision_pose_data;
        // 填充数据
        vision_pose_data.header.stamp = ros::Time::now();
        vision_pose_data.pose.position.x = uav_state.position.x();
        vision_pose_data.pose.position.y = uav_state.position.y();
        vision_pose_data.pose.position.z = uav_state.position.z();
        vision_pose_data.pose.orientation.x = uav_state.orientation.x();
        vision_pose_data.pose.orientation.y = uav_state.orientation.y();
        vision_pose_data.pose.orientation.z = uav_state.orientation.z();
        vision_pose_data.pose.orientation.w = uav_state.orientation.w();
        // 发布
        vision_pose_pub_.publish(vision_pose_data);
    }  // TODO: 实现odometry通道的融合
    return true;
}

bool MavrosHelper::set_px4_mode(control_common::FlightMode flight_mode) {
    // 首先依旧判断是否有运动控制权限
    if (config_cache_.uav_control_handle == false) {
        return false;
    }
    // 服务使用的数据类型
    mavros_msgs::SetMode px4_mode;
    px4_mode.request.base_mode = 0;
    // 根据flight_mode映射px4_mode
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
    case control_common::FlightMode::Undefined:  // 检查是否传入了一个不合理的值
        return false;
    default:
        return false;
    }
    return px4_mode_client_.call(px4_mode) && px4_mode.response.mode_sent;
}

bool MavrosHelper::set_arm(bool arm_state) {
    // 首先依旧判断是否有运动控制权限
    if (config_cache_.uav_control_handle == false) {
        return false;
    }
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
        ROS_WARN("[mavros_helper] cmd_long service not ready.");
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

bool MavrosHelper::pub_local_setpoint(control_common::Mavros_SetpointLocal setpoint_local) {
    // 首先依旧判断是否有运动控制权限
    if (config_cache_.uav_control_handle == false) {
        return false;
    }
    // 构造数据
    mavros_msgs::PositionTarget position_target_msg;
    // 时间戳
    position_target_msg.header.stamp = ros::Time::now();
    // 坐标系映射
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
    // 掩码
    position_target_msg.type_mask = setpoint_local.mask;
    // 位置
    position_target_msg.position.x = setpoint_local.position.x();
    position_target_msg.position.y = setpoint_local.position.y();
    position_target_msg.position.z = setpoint_local.position.z();
    // 速度
    position_target_msg.velocity.x = setpoint_local.velocity.x();
    position_target_msg.velocity.y = setpoint_local.velocity.y();
    position_target_msg.velocity.z = setpoint_local.velocity.z();
    // 加速度
    position_target_msg.acceleration_or_force.x = setpoint_local.accel_or_force.x();
    position_target_msg.acceleration_or_force.y = setpoint_local.accel_or_force.y();
    position_target_msg.acceleration_or_force.z = setpoint_local.accel_or_force.z();
    // 偏航角
    position_target_msg.yaw = setpoint_local.yaw;
    position_target_msg.yaw_rate = setpoint_local.yaw_rate;
    // 发布数据
    setpoint_local_pub_.publish(position_target_msg);
    return true;
}

bool MavrosHelper::pub_attitude_setpoint(
    control_common::Mavros_SetpointAttitude setpoint_attitude) {
    // 首先依旧判断是否有运动控制权限
    if (config_cache_.uav_control_handle == false) {
        return false;
    }
    // 构造数据
    mavros_msgs::AttitudeTarget attitude_target_msg;
    // 时间戳
    attitude_target_msg.header.stamp = ros::Time::now();
    // 掩码
    attitude_target_msg.type_mask = setpoint_attitude.mask;
    // 四元数姿态
    attitude_target_msg.orientation.x = setpoint_attitude.orientation.x();
    attitude_target_msg.orientation.y = setpoint_attitude.orientation.y();
    attitude_target_msg.orientation.z = setpoint_attitude.orientation.z();
    attitude_target_msg.orientation.w = setpoint_attitude.orientation.w();
    // 角速度
    attitude_target_msg.body_rate.x = setpoint_attitude.body_rate.x();
    attitude_target_msg.body_rate.y = setpoint_attitude.body_rate.y();
    attitude_target_msg.body_rate.z = setpoint_attitude.body_rate.z();
    // 推力
    attitude_target_msg.thrust = setpoint_attitude.thrust;
    // 发布
    setpoint_attitude_pub_.publish(attitude_target_msg);
    return true;
}

bool MavrosHelper::pub_px4_state() {
    // 首先依旧判断是否有运动控制权限
    if (config_cache_.uav_control_handle == false) {
        return false;
    }
    // 发布数据相关定时器,首先发布Px4State
    sunray_msgs::Px4State px4_state_msg;
    // 填充时间戳
    px4_state_msg.header.stamp = ros::Time::now();
    // 这部分从mavros/state获取数据
    px4_state_msg.connected = mavros_state_data_.connected;
    px4_state_msg.rc_available = mavros_state_data_.rc_input;
    px4_state_msg.armed = mavros_state_data_.armed;
    // 由于flight_mode与mavros_msgs中一一对应，这里使用类型转换来填充
    px4_state_msg.flight_mode = static_cast<uint8_t>(mavros_state_data_.flight_mode);
    px4_state_msg.flight_mode_name = flightmode_to_string(mavros_state_data_.flight_mode);
    px4_state_msg.system_status = mavros_state_data_.system_status;
    // 这部分从mavros/extended_state获取数据
    px4_state_msg.landed_state = static_cast<uint8_t>(mavros_state_data_.landed_state);
    px4_state_msg.landed_state_name = landed_to_string(mavros_state_data_.landed_state);
    // 这部分从mavros/sys_status获取数据
    px4_state_msg.battery_voltage_v = mavros_state_data_.voltage;
    px4_state_msg.battery_current_a = mavros_state_data_.current;
    px4_state_msg.battery_percentage = mavros_state_data_.percent;
    px4_state_msg.fcu_load = mavros_state_data_.system_load;

    // -------------------------------填充外部里程计-------------------------------
    px4_state_msg.external_pose.position.x = external_odometry_data_.position.x();
    px4_state_msg.external_pose.position.y = external_odometry_data_.position.y();
    px4_state_msg.external_pose.position.z = external_odometry_data_.position.z();
    // 填充外部里程计姿态
    px4_state_msg.external_pose.orientation.w = external_odometry_data_.orientation.w();
    px4_state_msg.external_pose.orientation.x = external_odometry_data_.orientation.x();
    px4_state_msg.external_pose.orientation.y = external_odometry_data_.orientation.y();
    px4_state_msg.external_pose.orientation.z = external_odometry_data_.orientation.z();
    // 填充外部里程计速度
    px4_state_msg.external_velocity.linear.x = external_odometry_data_.velocity.x();
    px4_state_msg.external_velocity.linear.y = external_odometry_data_.velocity.y();
    px4_state_msg.external_velocity.linear.z = external_odometry_data_.velocity.z();
    px4_state_msg.external_velocity.angular.x = external_odometry_data_.bodyrate.x();
    px4_state_msg.external_velocity.angular.y = external_odometry_data_.bodyrate.y();
    px4_state_msg.external_velocity.angular.z = external_odometry_data_.bodyrate.z();

    // -------------------------------填充local里程计-------------------------------
    px4_state_msg.local_pose.position.x = mavros_odometry_data_.position.x();
    px4_state_msg.local_pose.position.y = mavros_odometry_data_.position.y();
    px4_state_msg.local_pose.position.z = mavros_odometry_data_.position.z();
    // 填充外部里程计姿态
    px4_state_msg.local_pose.orientation.w = mavros_odometry_data_.orientation.w();
    px4_state_msg.local_pose.orientation.x = mavros_odometry_data_.orientation.x();
    px4_state_msg.local_pose.orientation.y = mavros_odometry_data_.orientation.y();
    px4_state_msg.local_pose.orientation.z = mavros_odometry_data_.orientation.z();
    // 填充外部里程计速度
    px4_state_msg.local_velocity.linear.x = mavros_odometry_data_.velocity.x();
    px4_state_msg.local_velocity.linear.y = mavros_odometry_data_.velocity.y();
    px4_state_msg.local_velocity.linear.z = mavros_odometry_data_.velocity.z();
    px4_state_msg.local_velocity.angular.x = mavros_odometry_data_.bodyrate.x();
    px4_state_msg.local_velocity.angular.y = mavros_odometry_data_.bodyrate.y();
    px4_state_msg.local_velocity.angular.z = mavros_odometry_data_.bodyrate.z();

    // -------------------------------填充setpoint local-------------------------------
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

    // -------------------------------填充setpoint attitude-------------------------------
    px4_state_msg.setpoint_att_type_mask = mavros_setpoint_attitude_data_.mask;
    px4_state_msg.orientation_setpoint.w = mavros_setpoint_attitude_data_.orientation.w();
    px4_state_msg.orientation_setpoint.x = mavros_setpoint_attitude_data_.orientation.x();
    px4_state_msg.orientation_setpoint.y = mavros_setpoint_attitude_data_.orientation.y();
    px4_state_msg.orientation_setpoint.z = mavros_setpoint_attitude_data_.orientation.z();
    px4_state_msg.body_rate_setpoint.x = mavros_setpoint_attitude_data_.body_rate.x();
    px4_state_msg.body_rate_setpoint.y = mavros_setpoint_attitude_data_.body_rate.y();
    px4_state_msg.body_rate_setpoint.z = mavros_setpoint_attitude_data_.body_rate.z();
    px4_state_msg.thrust_setpoint = mavros_setpoint_attitude_data_.thrust;
    // -------------------------------填充Global Position-------------------------------
    px4_state_msg.satellites = mavros_gps.satellites;
    px4_state_msg.gps_status = mavros_gps.gps_status;
    px4_state_msg.gps_service = mavros_gps.gps_service;
    px4_state_msg.latitude = mavros_gps.latitude;
    px4_state_msg.longitude = mavros_gps.longitude;
    px4_state_msg.altitude = mavros_gps.altitude;
    px4_state_msg.latitude_raw = mavros_gps.latitude_raw;
    px4_state_msg.longitude_raw = mavros_gps.longitude_raw;
    px4_state_msg.altitude_amsl = mavros_gps.altitude_amsl;
    px4_state_pub_.publish(px4_state_msg);
    return true;
}

/*----------------------------------mavros回调函数---------------------------*/
void MavrosHelper::mavros_state_callback(const mavros_msgs::State& msg) {
    mavros_state_data_.timestamp = msg.header.stamp;
    mavros_state_data_.connected = msg.connected;
    mavros_state_data_.armed = msg.armed;
    mavros_state_data_.rc_input = msg.manual_input;
    mavros_state_data_.system_status = msg.system_status;
    // 匹配模式
    if (msg.mode == mavros_msgs::State::MODE_PX4_MANUAL) {
        mavros_state_data_.flight_mode = control_common::FlightMode::Manual;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_ACRO) {
        mavros_state_data_.flight_mode = control_common::FlightMode::Acro;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_ALTITUDE) {
        mavros_state_data_.flight_mode = control_common::FlightMode::Altctl;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_POSITION) {
        mavros_state_data_.flight_mode = control_common::FlightMode::Posctl;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_OFFBOARD) {
        mavros_state_data_.flight_mode = control_common::FlightMode::Offboard;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_STABILIZED) {
        mavros_state_data_.flight_mode = control_common::FlightMode::Stabilized;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_RATTITUDE) {
        mavros_state_data_.flight_mode = control_common::FlightMode::Rattitude;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_MISSION) {
        mavros_state_data_.flight_mode = control_common::FlightMode::AutoMission;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_LOITER) {
        mavros_state_data_.flight_mode = control_common::FlightMode::AutoLoiter;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_RTL) {
        mavros_state_data_.flight_mode = control_common::FlightMode::AutoRtl;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_LAND) {
        mavros_state_data_.flight_mode = control_common::FlightMode::AutoLand;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_RTGS) {
        mavros_state_data_.flight_mode = control_common::FlightMode::AutoRtgs;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_READY) {
        mavros_state_data_.flight_mode = control_common::FlightMode::AutoReady;
    } else if (msg.mode == mavros_msgs::State::MODE_PX4_TAKEOFF) {
        mavros_state_data_.flight_mode = control_common::FlightMode::AutoTakeoff;
    } else {
        mavros_state_data_.flight_mode = control_common::FlightMode::Undefined;
    }
}

void MavrosHelper::mavros_externdedstate_callback(const mavros_msgs::ExtendedState& msg) {
    mavros_state_data_.timestamp = msg.header.stamp;
    // 匹配降落检测器状态
    switch (msg.landed_state) {
    case mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED:
        mavros_state_data_.landed_state = control_common::LandedState::Undefined;
        break;
    case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND:
        mavros_state_data_.landed_state = control_common::LandedState::OnGround;
        break;
    case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR:
        mavros_state_data_.landed_state = control_common::LandedState::InAir;
        break;
    case mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF:
        mavros_state_data_.landed_state = control_common::LandedState::Takeoff;
        break;
    case mavros_msgs::ExtendedState::LANDED_STATE_LANDING:
        mavros_state_data_.landed_state = control_common::LandedState::Landing;
        break;
    default:
        mavros_state_data_.landed_state = control_common::LandedState::Undefined;
        break;
    }
}

void MavrosHelper::mavros_sys_callback(const mavros_msgs::SysStatus& msg) {
    mavros_state_data_.timestamp = msg.header.stamp;
    // 这里我们将原始的数据转换为float，从而得到更好的效果
    mavros_state_data_.system_load = static_cast<float>(msg.load) / 1000.0f;
    if (msg.voltage_battery != UINT16_MAX) {  // UINT16_MAX 表示无效值
        mavros_state_data_.voltage = static_cast<float>(msg.voltage_battery) / 1000.0f;
    }
    if (msg.current_battery != -1) {  // -1 表示无效值
        mavros_state_data_.current = static_cast<float>(msg.current_battery) / 1000.0f;
    }
    if (msg.battery_remaining != -1) {  // -1 表示无效值
        mavros_state_data_.percent = static_cast<float>(msg.battery_remaining) / 100.0f;
    }
}

void MavrosHelper::mavros_estimator_callback(const mavros_msgs::EstimatorStatus& msg) {
    // 这个结构体似乎无法完全的表示vision_pose与EKF2之间的融合关系，后续尝试将px4内部的uorb消息拿出来试试
    mavros_estimator_data_.timestamp = msg.header.stamp;

    mavros_estimator_data_.attitude_valid = msg.attitude_status_flag;

    // 将 EKF 的速度/位置状态压缩到当前抽象结构中。。
    mavros_estimator_data_.local_hroiz_valid =
        msg.velocity_horiz_status_flag && msg.pos_horiz_rel_status_flag;
    mavros_estimator_data_.local_vertical_valid = msg.velocity_vert_status_flag &&
                                                  msg.pos_vert_abs_status_flag &&
                                                  msg.pos_vert_agl_status_flag;

    mavros_estimator_data_.global_hroiz_valid =
        msg.pos_horiz_abs_status_flag && msg.pred_pos_horiz_abs_status_flag;
    mavros_estimator_data_.global_vertical_valid = msg.pos_vert_abs_status_flag;

    mavros_estimator_data_.gps_error = msg.gps_glitch_status_flag;
    mavros_estimator_data_.acc_error = msg.accel_error_status_flag;
    mavros_estimator_data_.valid = true;
}

void MavrosHelper::mavros_localodom_callback(const nav_msgs::Odometry& msg) {
    // 这里很简单，直接丢进去
    mavros_odometry_data_ = control_common::UAVStateEstimate(msg);
    mavros_attitude_data_ = mavros_odometry_data_.orientation;
    // 提取位姿
    mavros_local_pose_data_.position = mavros_odometry_data_.position;
    mavros_local_pose_data_.orientation = mavros_odometry_data_.orientation;
    // 提取速度
    mavros_local_vel_data_.linear = mavros_odometry_data_.velocity;
    mavros_local_vel_data_.angular = mavros_odometry_data_.bodyrate;
}

void MavrosHelper::mavros_setpoint_local_callback(const mavros_msgs::PositionTarget& msg) {
    // 填时间戳
    mavros_setpoint_local_data_.timestamp = msg.header.stamp;
    // 转换坐标系
    switch (msg.coordinate_frame) {
    case mavros_msgs::PositionTarget::FRAME_LOCAL_NED:
        mavros_setpoint_local_data_.frame =
            control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
        break;
    case mavros_msgs::PositionTarget::FRAME_LOCAL_OFFSET_NED:
        mavros_setpoint_local_data_.frame =
            control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Offset_Ned;
        break;
    case mavros_msgs::PositionTarget::FRAME_BODY_NED:
        mavros_setpoint_local_data_.frame =
            control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Ned;
        break;
    case mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED:
        mavros_setpoint_local_data_.frame =
            control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Offset_Ned;
        break;
    default:
        break;
    }
    // 提取掩码
    mavros_setpoint_local_data_.mask = msg.type_mask;
    // 目标位置
    mavros_setpoint_local_data_.position.x() = msg.position.x;
    mavros_setpoint_local_data_.position.y() = msg.position.y;
    mavros_setpoint_local_data_.position.z() = msg.position.z;
    // 目标速度
    mavros_setpoint_local_data_.velocity.x() = msg.velocity.x;
    mavros_setpoint_local_data_.velocity.y() = msg.velocity.y;
    mavros_setpoint_local_data_.velocity.z() = msg.velocity.z;
    // 目标加速度
    mavros_setpoint_local_data_.accel_or_force.x() = msg.acceleration_or_force.x;
    mavros_setpoint_local_data_.accel_or_force.y() = msg.acceleration_or_force.y;
    mavros_setpoint_local_data_.accel_or_force.z() = msg.acceleration_or_force.z;
    // 目标偏航角
    mavros_setpoint_local_data_.yaw = msg.yaw;
    mavros_setpoint_local_data_.yaw_rate = msg.yaw_rate;
}

void MavrosHelper::mavros_setpoint_attitude_callback(const mavros_msgs::AttitudeTarget& msg) {
    // 填时间戳
    mavros_setpoint_attitude_data_.timestamp = msg.header.stamp;
    // 掩码
    mavros_setpoint_attitude_data_.mask = msg.type_mask;
    // 目标姿态
    mavros_setpoint_attitude_data_.orientation.x() = msg.orientation.x;
    mavros_setpoint_attitude_data_.orientation.y() = msg.orientation.y;
    mavros_setpoint_attitude_data_.orientation.z() = msg.orientation.z;
    mavros_setpoint_attitude_data_.orientation.w() = msg.orientation.w;
    // 目标角速度
    mavros_setpoint_attitude_data_.body_rate.x() = msg.body_rate.x;
    mavros_setpoint_attitude_data_.body_rate.y() = msg.body_rate.y;
    mavros_setpoint_attitude_data_.body_rate.z() = msg.body_rate.z;
    // 推力
    mavros_setpoint_attitude_data_.thrust = msg.thrust;
}

bool MavrosHelper::is_ready() {
    // 首先得到当前时间戳
    const ros::Time now_time = ros::Time::now();
    const ros::Duration timeout(0.5);
    // 根据缓存的 config_list 检查对应数据是否超时。
    // 对于带有 valid 字段的缓存，这里同步刷新 valid；
    // 对于 UAVStateEstimate，只参与 ready 判定，不额外写入 valid。
    bool ready = true;

    if (config_cache_.state == true) {
        mavros_state_data_.valid = (now_time - mavros_state_data_.timestamp) <= timeout;
        ready = ready && mavros_state_data_.valid;
    }

    if (config_cache_.ekf2_status == true) {
        mavros_estimator_data_.valid = (now_time - mavros_estimator_data_.timestamp) <= timeout;
        ready = ready && mavros_estimator_data_.valid;
    }

    if (config_cache_.local_odom == true) {
        ready = ready && ((now_time - mavros_odometry_data_.timestamp) <= timeout);
    }

    if (config_cache_.uav_target_state == true) {
        mavros_setpoint_local_data_.valid =
            (now_time - mavros_setpoint_local_data_.timestamp) <= timeout;
        mavros_setpoint_attitude_data_.valid =
            (now_time - mavros_setpoint_attitude_data_.timestamp) <= timeout;
        ready = ready && mavros_setpoint_local_data_.valid && mavros_setpoint_attitude_data_.valid;
    }

    // 控制句柄属于“是否成功初始化接口”，不是时间新鲜度问题。
    // 如果配置要求控制句柄，则检查发布者和 service client 是否有效。
    if (config_cache_.uav_control_handle == true) {
        const bool control_handle_ready = vision_pose_pub_ && setpoint_local_pub_ &&
                                          setpoint_attitude_pub_ && px4_arm_client_ &&
                                          px4_mode_client_;
        ready = ready && control_handle_ready;
    }

    mavros_ready = ready;
    return mavros_ready;
}
