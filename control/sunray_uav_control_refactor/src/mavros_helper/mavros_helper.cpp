#include "mavros_helper/mavros_helper.hpp"
#include <stdexcept>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

// 构造函数
MavrosHelper::MavrosHelper(ros::NodeHandle nh, MavrosHelper_ConfigList config_list) {
    // 使用全局参数构造uav_ns_
    std::string uav_ns;
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
    uav_ns = uav_name + std::to_string(uav_id);
    // 本来设计了一个表，然后使用迭代器遍历的，但是想了一下，可读性没有直接用if高，所以还是用if吧
    // 首先还是要缓存一下这个config_list，这样做的好处是，提高结构体中的valid语义
    config_cache_ = config_list;
    // if手动遍历，判断状态，然后初始化订阅者
    // 由于代码格式化的原因，下面的初始化语句有的会是一行，有的又会拆成很多行，因此禁止插件格式化，并手动排版提高可读性
    // clang-format off
    if (config_list.state == true) {
        state_sub_ =nh.subscribe(uav_ns + "/mavros/state",
                                 10,
                                 &MavrosHelper::mavros_state_callback,
                                 this);
        extended_state_sub_ = nh.subscribe(uav_ns + "/mavros/extended_state",
                                           10,
                                           &MavrosHelper::mavros_externdedstate_callback,
                                           this);
        sys_sub_ = nh.subscribe(uav_ns+"/mavros/sys_status",
                                10,
                                &MavrosHelper::mavros_sys_callback,
                                this);
    }
    if (config_list.ekf2_status == true) {
        estimator_sub_ = nh.subscribe(uav_ns + "/mavros/estimator_status",
                                    10,
                                    &MavrosHelper::mavros_estimator_callback,
                                    this);
    }
    if(config_list.local_odom == true){
        local_odom_sub_ = nh.subscribe(uav_ns+"/mavros/local_position/odom",
                                       10,
                                       &MavrosHelper::mavros_localodom_callback,
                                       this);
    }
    if(config_list.uav_target_state == true){
        setpoint_local_sub_ = nh.subscribe(uav_ns+"/mavros/setpoint_raw/target_local",
                                           10,
                                           &MavrosHelper::mavros_setpoint_local_callback,
                                           this);
        setpoint_attitude_sub_ = nh.subscribe(uav_ns+"/mavros/setpoint_raw/target_attitude",
                                           10,
                                           &MavrosHelper::mavros_setpoint_attitude_callback,
                                           this);
    }
    // 接下来是发布
    if(config_list.uav_control_handle){
        // 首先是vision_pose
        vision_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(uav_ns+"/mavros/vision_pose/pose",10);
        // 然后是setpoint
        setpoint_local_pub_ = nh.advertise<mavros_msgs::PositionTarget>(uav_ns+"/mavros/setpoint_raw/local",10);
        setpoint_attitude_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(uav_ns+"/mavros/setpoint_raw/attitude",10);
        // 然后是服务端
        px4_arm_client_ = nh.serviceClient<mavros_msgs::CommandBool>(uav_ns+"/mavros/cmd/arming");
        px4_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(uav_ns + "/mavros/set_mode");
    }
    // clang-format on
}

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

bool MavrosHelper::pub_vision_pose(control_common::UAVStateEstimate uav_state) {
    // 首先判断是否有运动控制权限
    if (config_cache_.uav_control_handle == false) {
        return false;
    }
    // 构造类型
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
    attitude_target_msg.orientation.x = setpoint_attitude.attitude.x();
    attitude_target_msg.orientation.y = setpoint_attitude.attitude.y();
    attitude_target_msg.orientation.z = setpoint_attitude.attitude.z();
    attitude_target_msg.orientation.w = setpoint_attitude.attitude.w();
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
    mavros_setpoint_attitude_data_.attitude.x() = msg.orientation.x;
    mavros_setpoint_attitude_data_.attitude.y() = msg.orientation.y;
    mavros_setpoint_attitude_data_.attitude.z() = msg.orientation.z;
    mavros_setpoint_attitude_data_.attitude.w() = msg.orientation.w;
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
