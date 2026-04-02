#include "controller/px4_origin_controller.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "control_data_types/mavros_helper_data_types.hpp"
#include "string_uav_namespace_utils.hpp"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>  // 引入Yaml-cpp库，用于读取yaml文件
#include <sunray_msgs/UAVControllerState.h>
#include <px4_param_manager/px4_param_types.h>
#include <px4_param_manager/px4_param_decode.h>

// 构造函数，读取参数
PX4_OriginController::PX4_OriginController(ros::NodeHandle& nh)
    : nh_(nh), mavros_helper_(nh_), mavros_param_(nh_) {
    // 读取节点名
    std::string node_name = ros::this_node::getName();
    // 构造私有节点句柄，用于读取节点私有参数,controller主要读取yaml路径
    ros::NodeHandle private_nh_("~");
    if (private_nh_.getParam("config_yamlfile_path", config_yamlfile_path_)) {
        if (config_yamlfile_path_.empty()) {  // 路径为空，抛出异常
            throw std::runtime_error("yaml_path connot be empty");
        }
    } else {  // 读取失败，抛出异常
        throw std::runtime_error("missing param" + node_name + "/config_yamlfile_path");
    }
    // 读取全局参数
    std::string uav_name;
    int uav_id;
    if (nh_.getParam("/uav_name", uav_name)) {
        if (uav_name.empty()) {  // 如果路径参数为空，也抛出异常
            throw std::runtime_error("uav_name connot be empty");
        }
    } else {
        throw std::runtime_error("missing param /uav_name");
    }
    if (!nh_.getParam("/uav_id", uav_id)) {
        throw std::runtime_error("missing param /uav_id");
    }
    // 拼接uav_ns
    uav_ns_ = uav_name + std::to_string(uav_id);
    // 标准化
    uav_ns_ = sunray_common::normalize_uav_ns(uav_ns_);
}

bool PX4_OriginController::init() {
    // 加载参数并校验参数
    load_and_validate_config_or_throw();
    // 配置读取完毕，初始化mavros_helper
    MavrosHelper_ConfigList config_list(true);
    if (!mavros_helper_.init(config_list)) {
        throw std::runtime_error("mavros_helper init failed");
    }
    if (config_param_.fuse_odom_type != 0) {  // 融合外部里程计到px4，设置对应参数,注册定时器
        // 检查外部里程计融合相关参数
        ensure_fusion_param_ready_or_throw();  // 该函数检查失败会抛出异常
        mavros_helper_.set_vision_fuse_type(config_param_.fuse_odom_type);
        // 创建定时器
        pub_vision_pose_timer_ =
            nh_.createTimer(ros::Duration(1.0 / config_param_.fuse_odom_frequency),
                            &PX4_OriginController::pub_vision_fuse_timer_cb,
                            this);
    }
    // 初始化话题发布者
    controller_state_pub_ =
        nh_.advertise<sunray_msgs::UAVControllerState>(uav_ns_ + "/sunray/controller_state", 10);
    // 初始化发布px4_state数据的定时器
    pub_px4_state_timer = nh_.createTimer(ros::Duration(1.0 / pub_px4_state_freq_),
                                          &PX4_OriginController::pub_px4_state_timer_cb,
                                          this);
    return true;
}

bool PX4_OriginController::is_ready() {
    control_common::Mavros_State mavros_state = mavros_helper_.get_state();
    px4_mode_ = mavros_state.flight_mode;
    px4_land_ = mavros_state.landed_state;
    px4_arm_ = mavros_state.armed;
    bool mavros_ready = mavros_helper_.is_ready();
    if (px4_land_ == control_common::LandedState::OnGround && px4_arm_ == false &&
        has_uav_odometry_ == true && mavros_ready == true) {
        controller_ready_ = true;
        return true;
    } else {
        return false;
    }
}

void PX4_OriginController::set_current_odom(const control_common::UAVStateEstimate& odom) {
    uav_odometry_ = odom;
    has_uav_odometry_ = true;
}

// -------------运动相关接口------------

bool PX4_OriginController::takeoff(double relative_takeoff_height, double max_takeoff_velocity) {
    // 如何设计呢？起始这里的问题是，我们如何触发？
    // 实现思路为这样，我们不断的触发这个函数直到达到预设的起飞高度，也就是这样
    // ------sunray_fsm--------
    // while(controller_update_freq){
    //   switch(control_cmd){
    //       controller.takeoff();//状态机根据控制器的频率持续调用
    //   }
    // }
    // 首先如果控制器状态未就绪，拒绝起飞，返回false
    // 本函数在确定到达起飞位置后，返回true，其他情况均返回false，控制器依据返回true切换为hover状态
    if (land_complete_ == true) {
        land_complete_ = false;
    }
    if (controller_ready_ == false) {
        return false;
    }
    if (takeoff_complete_ == true) {
        control_common::Mavros_SetpointLocal current_setpoint;
        current_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
        current_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreVx |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreVy |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreVz |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreAfz |
                                control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
        control_common::Mavros_Pose current_odom = mavros_helper_.get_local_pose();
        current_setpoint.position = current_odom.position;
        current_setpoint.yaw = mavros_helper_.get_yaw_rad();
        mavros_helper_.pub_local_setpoint(last_setpoint_);

        ROS_INFO("satrt_position x : %f,y : %f,z : %f",
                 current_setpoint.position.x(),
                 current_setpoint.position.y(),
                 current_setpoint.position.z());
        return true;
    }
    ros::Time now = ros::Time::now();
    // 通过五次项曲线类，确定是否为第一次进入takeoff函数
    if (!quint_curve_.is_ready()) {
        // 首次进入五次项曲线，注入参数
        // 起点参数为当前里程计值，速度为0
        quint_curve_.set_start_trajpoint(uav_odometry_.position, Eigen::Vector3d::Zero());
        // 终点参数为当前里程计+z轴相对期望高度，速度为0
        quint_curve_.set_end_trajpoint(uav_odometry_.position +
                                           Eigen::Vector3d(0, 0, relative_takeoff_height),
                                       Eigen::Vector3d::Zero());
        // 根据起飞过程最大速度反推时间
        quint_curve_.set_curve_maxvel(max_takeoff_velocity);
        // quint_curve并不显式的设置开始运动的时间，我们以第一次调用get_result的时刻为开始运动的时间
        // 我们需要考虑无人机是否需要一点时间来进行加速，也就是说从电机桨叶不转动到转动的过程，是先不计算数据的
        // 顺便记录一下初始时刻的yaw角和地面高度
        takeoff_yaw_ = mavros_helper_.get_yaw_rad();
        takeoff_ground_height = uav_odometry_.position.z();
    }
    control_common::Mavros_State px4_state = mavros_helper_.get_state();
    if (px4_state.flight_mode != control_common::FlightMode::Offboard) {
        // 默认的模式应该是position模式
        // 首先我们需要切换为offboard模式，切换模式需要发送至少2Hz的控制指令，因此我们先设置控制指令为零速度指令
        control_common::Mavros_SetpointLocal setpoint_cmd;
        // 设置坐标系
        setpoint_cmd.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
        // 掩码这里先忽略位置与加速度和yaw_rate，使用位运算
        setpoint_cmd.mask = control_common::Mavros_SetpointLocal::Mask::IgnorePx |
                            control_common::Mavros_SetpointLocal::Mask::IgnorePy |
                            control_common::Mavros_SetpointLocal::Mask::IgnorePz |
                            control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                            control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                            control_common::Mavros_SetpointLocal::Mask::IgnoreAfz |
                            control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
        // 速度置零
        setpoint_cmd.velocity = Eigen::Vector3d::Zero();
        // 保持yaw角不变
        setpoint_cmd.yaw = takeoff_yaw_;
        // 发送
        mavros_helper_.pub_local_setpoint(setpoint_cmd);
        // // 开始进入 offboard 检查窗口
        if (start_checkout_offboard_time_ == ros::Time(0)) {
            // 记录开始触发的时间
            start_checkout_offboard_time_ = now;
            last_checkout_offboard_time_ = ros::Time(0);
        }
        // 每 0.3s 请求一次 Offboard
        if (last_checkout_offboard_time_ == ros::Time(0) ||
            (now - last_checkout_offboard_time_).toSec() >= 0.3) {
            mavros_helper_.set_px4_mode(control_common::FlightMode::Offboard);
            last_checkout_offboard_time_ = now;
        }
        // 超过 3s 仍未进入 Offboard，走失败处理
        if ((now - start_checkout_offboard_time_).toSec() > 3) {
            // TODO: 如何处理超过3s也无法切换为offboard的情况
        }
    }
    // 成功切换到offboard模式
    if (px4_state.flight_mode == control_common::FlightMode::Offboard) {
        // 清除切换offboard的上下文
        start_checkout_offboard_time_ = ros::Time(0);
        last_checkout_offboard_time_ = ros::Time(0);
        // 如果飞控没有解锁，则尝试解锁
        if (px4_state.armed == false) {
            mavros_helper_.set_arm(true);
        } else {  // 飞控当前处于解锁状态
            // 注意到起飞阶段由于无人机在地面，与在空中的动力学分析不一致，起飞阶段setpoint具有严重的滞后
            // 这里尝试一种起飞方式，先通过速度控制，实现离地，然后切换为轨迹控制
            if (last_arm_time_ == ros::Time(0)) {
                control_common::Mavros_SetpointLocal setpoint_cmd;
                // 设置坐标系
                // setpoint_cmd.frame =
                //     control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
                // 掩码这里忽略z轴位置，三轴加速度和yaw_rate，使用位运算
                setpoint_cmd.mask = control_common::Mavros_SetpointLocal::Mask::IgnorePz |
                                    control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                                    control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                                    control_common::Mavros_SetpointLocal::Mask::IgnoreAfz |
                                    control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
                // 设置xy轴的位置
                Eigen::Vector3d start_pos = quint_curve_.get_start_position();
                setpoint_cmd.position = start_pos;
                // z轴速度设置为0.1
                setpoint_cmd.velocity = Eigen::Vector3d(0, 0, 0.1);
                // 设置yaw角为初始时刻yaw角
                setpoint_cmd.yaw = takeoff_yaw_;
                // 发送
                mavros_helper_.pub_local_setpoint(setpoint_cmd);
                // 设置轨迹点
                ROS_INFO("uav_odometry z : %f", uav_odometry_.position.z());
                ROS_INFO("takeoff_ground_height : %f", takeoff_ground_height);
                if (uav_odometry_.position.z() - takeoff_ground_height > 0.05) {
                    last_arm_time_ = now;
                    Eigen::Vector3d renew_pos = start_pos;
                    renew_pos.z() = uav_odometry_.position.z();
                    quint_curve_.set_start_trajpoint(renew_pos, Eigen::Vector3d(0, 0, 0.1));
                } else {
                    return false;
                }
            }
            // 得到五次项曲线输出
            curve::QuinticCurveState curve_result;
            curve_result = quint_curve_.get_result();
            // 使用五次项输出填充setpoint_cmd
            control_common::Mavros_SetpointLocal setpoint_cmd;
            // setpoint_cmd.frame =
            // control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
            setpoint_cmd.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
            setpoint_cmd.position = curve_result.position;
            setpoint_cmd.velocity = curve_result.velocity;
            setpoint_cmd.accel_or_force = curve_result.acceleration;
            setpoint_cmd.yaw = takeoff_yaw_;
            // 发布setpotin_cmd
            mavros_helper_.pub_local_setpoint(setpoint_cmd);
            // 缓存状态
            last_setpoint_ = setpoint_cmd;
            // 在这里判断是否到达期望的起飞目标点
            Eigen::Vector3d pos_err_vec = uav_odometry_.position - quint_curve_.get_end_position();
            double pos_err = pos_err_vec.norm();  // 推荐：位置误差标量

            Eigen::Vector3d vel_err_vec = uav_odometry_.velocity - Eigen::Vector3d::Zero();
            double vel_err = vel_err_vec.norm();  // 推荐：速度误差标量
            ROS_INFO("pos_err : %f", pos_err);
            ROS_INFO("vel_err : %f", vel_err);
            if (pos_err < 0.3 && vel_err < 0.15) {
                if (start_checkout_takeoff_success_time_ == ros::Time(0)) {
                    start_checkout_takeoff_success_time_ = now;
                }
                if ((now - start_checkout_takeoff_success_time_).toSec() >
                    takeoff_success_keep_time_s) {
                    takeoff_complete_ = true;
                    // 清理上下文
                    start_checkout_offboard_time_ = ros::Time(0);
                    last_checkout_offboard_time_ = ros::Time(0);
                    last_arm_time_ = ros::Time(0);
                    start_checkout_takeoff_success_time_ = ros::Time(0);
                    quint_curve_.clear();
                    return true;
                }
            } else {
                start_checkout_takeoff_success_time_ = ros::Time(0);
            }
        }
    }
    return false;
}

bool PX4_OriginController::is_takeoff_complete() {
    return takeoff_complete_;
}

// 仅测试
bool PX4_OriginController::land(bool land_type, double max_land_velocity) {
    if (land_type == 1) {
        // 切换为px4的auto land模式
        mavros_helper_.set_px4_mode(control_common::FlightMode::AutoLand);
        bool land_state =
            mavros_helper_.get_state().landed_state == control_common::LandedState::OnGround;
        return land_state;
    }
    ros::Time now = ros::Time::now();
    if (land_complete_ == true) {
        return true;
    }
    // 第一次进入land函数，先初始化用到的变量
    if (start_land_time_ == ros::Time(0)) {
        // 清除掉五次项曲线的参数，然后重新填入
        quint_curve_.clear();

        // 使用当前setpoint的输出position部分为轨迹的起点，保证setpoint的连续性
        control_common::Mavros_SetpointLocal last_setpoint;
        last_setpoint = mavros_helper_.get_target_local();
        Eigen::Vector3d last_setpoint_position = last_setpoint.position;
        quint_curve_.set_start_trajpoint(last_setpoint_position, Eigen::Vector3d::Zero());
        // 使用当前位置的xy和地面高度+2作为轨迹的终点,速度使用0.1
        Eigen::Vector3d land_position = Eigen::Vector3d(
            uav_odometry_.position.x(), uav_odometry_.position.y(), takeoff_ground_height + 0.2);
        Eigen::Vector3d land_vel = Eigen::Vector3d(0, 0, -0.2);
        // 设置轨迹的终点参数
        quint_curve_.set_end_trajpoint(land_position, land_vel);
        // 使用最大降落速度求解时间
        quint_curve_.set_curve_maxvel(max_land_velocity);
        // 记录当前的yaw角
        land_yaw_ = mavros_helper_.get_yaw_rad();
        // 更新时间戳
        start_land_time_ = now;
        ROS_INFO("satrt_position x : %f,y : %f,z : %f",
                 last_setpoint.position.x(),
                 last_setpoint.position.y(),
                 last_setpoint.position.z());
        ROS_INFO("land_position x : %f,y : %f,z : %f",
                 land_position.x(),
                 land_position.y(),
                 land_position.z());
    }
    if (land_near_ground_ == false) {
        // 得到五次项曲线输出
        curve::QuinticCurveState curve_result;
        curve_result = quint_curve_.get_result();
        // 使用五次项输出填充setpoint_cmd
        control_common::Mavros_SetpointLocal setpoint_cmd;
        // setpoint_cmd.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
        setpoint_cmd.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
        setpoint_cmd.position = curve_result.position;
        setpoint_cmd.velocity = curve_result.velocity;
        setpoint_cmd.accel_or_force = curve_result.acceleration;
        // yaw角设置为降落触发时的yaw角
        setpoint_cmd.yaw = land_yaw_;
        // 发布setpotin_cmd
        mavros_helper_.pub_local_setpoint(setpoint_cmd);
    }
    // 检查轨迹是否执行到目标点，也就是近地段
    bool near_ground = (uav_odometry_.position.z() - quint_curve_.get_end_position().z() < 0.05);
    bool velocity_low = (uav_odometry_.velocity.norm() < 0.15);
    control_common::LandedState px4_landed = mavros_helper_.get_state().landed_state;
    if (near_ground && velocity_low) {
        land_near_ground_ = true;
    }
    // 轨迹结束后，设置为锁xy然后持续下降
    if (land_near_ground_ == true) {
        control_common::Mavros_SetpointLocal setpoint_cmd;
        // setpoint_cmd.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
        setpoint_cmd.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate |
                            control_common::Mavros_SetpointLocal::Mask::IgnorePz;
        curve::QuinticCurveState curve_result;
        curve_result = quint_curve_.get_result();
        setpoint_cmd.position = curve_result.position;
        setpoint_cmd.velocity = curve_result.velocity;

        setpoint_cmd.yaw = land_yaw_;
        // 持续检查是否接触地面
        velocity_low = std::abs(uav_odometry_.velocity.z()) < 0.1;
        if (velocity_low == true) {
            setpoint_cmd.velocity.z() -= 0.1;
        }
        bool px4_land = px4_landed == control_common::LandedState::OnGround;
        if (velocity_low && px4_land) {  // 考虑传感器漂移，这里用||逻辑
            if (land_touchground_time_ == ros::Time(0)) {
                land_touchground_time_ = now;
            }
            if ((now - land_touchground_time_).toSec() > 1.0) {
                mavros_helper_.set_arm(false);  // 上锁
                if (mavros_helper_.get_state().armed == false) {
                    // 上锁成功，清理上下文
                    land_complete_ = true;
                    takeoff_complete_ = false;
                    quint_curve_.clear();
                    start_land_time_ = ros::Time(0);
                    land_near_ground_ = false;
                    land_touchground_time_ = ros::Time(0);
                }
                return land_complete_;
            }
        } else {
            land_touchground_time_ = ros::Time(0);
        }
        mavros_helper_.pub_local_setpoint(setpoint_cmd);
    }
    return false;
}

bool PX4_OriginController::is_land_complete() {
    return land_complete_;
}

bool PX4_OriginController::hover() {
    // hover 阶段，我们认为使用上一次的setpoint中的position值用于悬停
    control_common::Mavros_SetpointLocal hover_setpoint;
    hover_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    hover_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreVx |
                          control_common::Mavros_SetpointLocal::Mask::IgnoreVy |
                          control_common::Mavros_SetpointLocal::Mask::IgnoreVz |
                          control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                          control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                          control_common::Mavros_SetpointLocal::Mask::IgnoreAfz |
                          control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
    control_common::Mavros_SetpointLocal current_setpoint;
    current_setpoint = mavros_helper_.get_target_local();
    hover_setpoint.position = current_setpoint.position;
    hover_setpoint.yaw = current_setpoint.yaw;
    mavros_helper_.pub_local_setpoint(hover_setpoint);
    // 是否应当分析误差来确定返回值为true or false呢
    return true;
}

bool PX4_OriginController::emergency_kill() {
    return mavros_helper_.emergency_kill();
}

bool PX4_OriginController::move_point(controller_data_types::TargetPoint_t point) {
    // 记录当前时间戳
    ros::Time now = ros::Time::now();
    // 如果传入的point没有对位置进行赋值
    if (!point.position.has_value()) {
        return false;
    }
    // 新目标判据，使用==会由于数值漂移导致异常情况(虽然很少出现)
    constexpr double kNewTargetPosEps = 1e-3;
    // 新目标状态
    bool is_new_target = false;
    // 检查是否为新目标
    if (!last_point_.position.has_value()) {
        // 缓存没有值，为新目标
        is_new_target = true;
    } else {
        const double dp = (point.position.value() - last_point_.position.value()).norm();
        // 位置误差大于常量误差，为新值
        if (dp > kNewTargetPosEps) {
            is_new_target = true;
        }
    }
    // 如果为新目标，则清除一些上下文参数
    if (is_new_target) {
        move_point_arrive_state_ = false;
        start_move_arrive_time_ = ros::Time(0);
        last_point_ = point;
    }
    control_common::Mavros_SetpointLocal send_setpoint;
    send_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    send_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnoreVx |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreVy |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreVz |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreAfz |
                         control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
    send_setpoint.position = point.position.value();
    if (point.yaw.has_value()) {
        send_setpoint.yaw = point.yaw.value();
    } else {
        send_setpoint.yaw = last_setpoint_.yaw;
    }
    mavros_helper_.pub_local_setpoint(send_setpoint);
    // 更新缓存
    last_setpoint_ = send_setpoint;
    // 检查误差
    Eigen::Vector3d pos_err_vec = uav_odometry_.position - point.position.value();
    double pos_err = pos_err_vec.norm();  // 推荐：位置误差标量

    Eigen::Vector3d vel_err_vec = uav_odometry_.velocity - Eigen::Vector3d::Zero();
    double vel_err = vel_err_vec.norm();  // 推荐：速度误差标量
    ROS_INFO("pos_err : %f", pos_err);
    ROS_INFO("vel_err : %f", vel_err);
    if (pos_err < 0.15 && vel_err < 0.15) {
        if (start_move_arrive_time_ == ros::Time(0)) {
            start_move_arrive_time_ = now;
        }
        if ((now - start_move_arrive_time_).toSec() > 0.5) {
            move_point_arrive_state_ = true;
        }
    } else {
        start_move_arrive_time_ = ros::Time(0);
    }
    return move_point_arrive_state_;
}
bool PX4_OriginController::move_velocity(controller_data_types::TargetVelocity_t velocity) {
    // 请注意，velocity是一个比较危险的接口，我们会默认返回true
    control_common::Mavros_SetpointLocal velocity_setpoint;
    velocity_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    velocity_setpoint.mask = control_common::Mavros_SetpointLocal::Mask::IgnorePx |
                             control_common::Mavros_SetpointLocal::Mask::IgnorePy |
                             control_common::Mavros_SetpointLocal::Mask::IgnorePz |
                             control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                             control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                             control_common::Mavros_SetpointLocal::Mask::IgnoreAfz;
    velocity_setpoint.velocity = velocity.velocity.value();
    if (velocity.yaw.has_value()) {
        velocity_setpoint.yaw = velocity.yaw.value();
    } else {
        velocity_setpoint.mask |= control_common::Mavros_SetpointLocal::Mask::IgnoreYaw;
    }
    if (velocity.yaw_rate.has_value()) {
        velocity_setpoint.yaw_rate = velocity.yaw_rate.value();
    } else {
        velocity_setpoint.mask |= control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
    }
    mavros_helper_.pub_local_setpoint(velocity_setpoint);
    return true;
}
bool PX4_OriginController::move_trajectory(
    controller_data_types::TargetTrajectoryPoint_t trajpoint) {
    // 轨迹接口传递有值的字段，没有值的字段设置为掩码
    control_common::Mavros_SetpointLocal trajpoint_setpoint;
    trajpoint_setpoint.frame = control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned;
    trajpoint_setpoint.position = trajpoint.position;
    if (trajpoint.velocity.has_value()) {
        trajpoint_setpoint.velocity = trajpoint.velocity.value();
    } else {
        trajpoint_setpoint.mask |= control_common::Mavros_SetpointLocal::Mask::IgnoreVx |
                                   control_common::Mavros_SetpointLocal::Mask::IgnoreVy |
                                   control_common::Mavros_SetpointLocal::Mask::IgnoreVz;
    }
    if (trajpoint.acceleration.has_value()) {
        trajpoint_setpoint.accel_or_force = trajpoint.acceleration.value();
    } else {
        trajpoint_setpoint.mask |= control_common::Mavros_SetpointLocal::Mask::IgnoreAfx |
                                   control_common::Mavros_SetpointLocal::Mask::IgnoreAfy |
                                   control_common::Mavros_SetpointLocal::Mask::IgnoreAfz;
    }
    if (trajpoint.yaw.has_value()) {
        trajpoint_setpoint.yaw = trajpoint.yaw.value();
    } else {
        trajpoint_setpoint.mask |= control_common::Mavros_SetpointLocal::Mask::IgnoreYaw;
    }
    if (trajpoint.yaw_rate.has_value()) {
        trajpoint_setpoint.yaw_rate = trajpoint.yaw_rate.value();
    } else {
        trajpoint_setpoint.mask |= control_common::Mavros_SetpointLocal::Mask::IgnoreYawRate;
    }
    // 我们如何设置轨迹的反馈呢？这里的问题是，拿到了轨迹的反馈，有什么用呢？
    // 轨迹是一个高频变化的值，因此拿到轨迹的反馈，其实没有什么用，so
    // 返回的是true表示接受到了o.o
    return true;
}
// BUG: body系的函数存在一点小问题
bool PX4_OriginController::move_point_body(controller_data_types::TargetPoint_t point) {
    // 首选检查传入的point是否有值
    if (!point.position.has_value()) {
        return false;
    }
    constexpr double kPosEps = 1e-3;
    constexpr double kYawEps = 1e-3;

    bool is_new_body_target = false;
    if (!last_point_body_.position.has_value()) {
        is_new_body_target = true;
    } else {
        double dp = (point.position.value() - last_point_body_.position.value()).norm();
        if (dp > kPosEps)
            is_new_body_target = true;

        if (!is_new_body_target && point.yaw.has_value() && last_point_body_.yaw.has_value()) {
            if (std::abs(point.yaw.value() - last_point_body_.yaw.value()) > kYawEps) {
                is_new_body_target = true;
            }
        } else if (!is_new_body_target &&
                   (point.yaw.has_value() != last_point_body_.yaw.has_value())) {
            is_new_body_target = true;
        }
    }
    if (is_new_body_target) {
        // 得到当前的yaw角
        const double yaw = mavros_helper_.get_yaw_rad();
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        // 做body系到local系的转换
        const Eigen::Vector3d p_b = point.position.value();
        Eigen::Vector3d delta_w;
        delta_w.x() = c * p_b.x() - s * p_b.y();
        delta_w.y() = s * p_b.x() + c * p_b.y();
        // z轴不进行转换
        delta_w.z() = p_b.z();
        // 目标点 = 当前点 + 增量点
        controller_data_types::TargetPoint_t world_point;
        world_point.position = uav_odometry_.position + delta_w;

        // yaw语义：若给了yaw，按“相对机体yaw增量”处理；否则保持当前朝向
        if (point.yaw.has_value()) {
            world_point.yaw = yaw + point.yaw.value();
        } else {
            world_point.yaw = yaw;
        }
        last_point_ = world_point;
        last_point_body_ = point;
    }
    // 复用已有move_point逻辑（到点判定/缓存/停留判定）
    return move_point(last_point_);
}
bool PX4_OriginController::move_velocity_body(controller_data_types::TargetVelocity_t velocity) {
    // 输入检查
    if (!velocity.velocity.has_value()) {
        return false;
    }
    // 读取当前的yaw角
    const double yaw = mavros_helper_.get_yaw_rad();
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    // 转换输入的body velocity -> world velocity
    const Eigen::Vector3d v_b = velocity.velocity.value();
    Eigen::Vector3d v_w;
    v_w.x() = c * v_b.x() - s * v_b.y();
    v_w.y() = s * v_b.x() + c * v_b.y();
    v_w.z() = v_b.z();
    // 转换为world下的速度
    controller_data_types::TargetVelocity_t world_vel;
    world_vel.velocity = v_w;
    // yaw语义：
    // - 若输入yaw有值：按“相对机体yaw增量”解释
    // - 否则保持当前yaw
    if (velocity.yaw.has_value()) {
        world_vel.yaw = yaw + velocity.yaw.value();
    } else {
        world_vel.yaw = yaw;
    }
    return move_velocity(world_vel);
}
// WGS84不知道怎么测试，先放一边
bool PX4_OriginController::move_point_wgs84(controller_data_types::TargetPoint_t point) {
    return false;
}
// -------------起降状态查询接口------------

void PX4_OriginController::pub_controller_state() {
    return;
}
// clang-format off
void PX4_OriginController::load_and_validate_config_or_throw() {
    // 首先在构造函数中我们已经判断了config_yamlfile_path_非空,因此这里不再判断
    YAML::Node root;  // 构造一个YAML文件的根节点
    // 由于读取的过程可能引发异常，因此使用try语法
    try {
        root = YAML::LoadFile(config_yamlfile_path_); // 从指定的路径中读取yaml文件并解析为YAML::Node
    } catch (const YAML::Exception& e) {  // 如果解析的过程中发生错误，捕捉异常
        throw std::runtime_error("Failed to load yaml file '" + config_yamlfile_path_ + ":" + e.what());
    }
    // 顺利读取，取出字段basic_param的部分
    const YAML::Node basic_param = root["basic_param"];
    // 如果basic_param为空，或者不是键值对的形式，则抛出异常
    if (!basic_param || !basic_param.IsMap()) {
        throw std::runtime_error("the yaml file '" + config_yamlfile_path_ + "' is missing a valid basic_param map");
    }
    // 由于OriginalController需要的参数不多，我们直接拿字段读
    if (!basic_param["fuse_odom_type"]) {
        throw std::runtime_error("miss param 'fuse_odom_type'");
    } else {
        config_param_.fuse_odom_type = basic_param["fuse_odom_type"].as<int>();
    }
    if (!basic_param["fuse_odom_frequency"]) {
        throw std::runtime_error("miss param 'fuse_odom_frequency'");
    } else {
        config_param_.fuse_odom_frequency = basic_param["fuse_odom_frequency"].as<double>();
    }
    // 检查一下参数是否正常
    if(config_param_.fuse_odom_type != 0 && config_param_.fuse_odom_type != 1 && config_param_.fuse_odom_type != 2 ){
        throw std::runtime_error("param 'fuse_odom_type' value must be 0,1,2");
    }
    // 限制融合的频率
    config_param_.fuse_odom_frequency = std::max(10.0, config_param_.fuse_odom_frequency);
    config_param_.fuse_odom_frequency = std::min(200.0, config_param_.fuse_odom_frequency);
}
// clang-format on

void PX4_OriginController::ensure_fusion_param_ready_or_throw() {
    if (config_param_.fuse_odom_type == 0) {
        return;  // 不做视觉融合时无需检查
    }
    // 构造lambda表达式简化后续重读
    auto check_param = [this]() -> bool {
        px4_param_decode::EKF2_EV_CTRL ev_ctrl_read;
        px4_param_decode::EKF2_HGT_REF hgt_ref_read;
        mavros_param_.read_param(&ev_ctrl_read);
        mavros_param_.read_param(&hgt_ref_read);

        bool ev_ok = ev_ctrl_read.enable_horizontal_position() &&
                     ev_ctrl_read.enable_vertical_position() && ev_ctrl_read.enable_yaw();
        bool hgt_ok = hgt_ref_read.is_vision();
        return ev_ok && hgt_ok;
    };
    // 先读一次，如果满足直接结束
    if (check_param()) {
        return;
    }
    // 不满足，写入目标参数
    px4_param_types::EKF2_EV_CTRL ev_ctrl_write;
    ev_ctrl_write.enable_Horizontalposition();
    ev_ctrl_write.enable_Verticalposition();
    ev_ctrl_write.enable_Yaw();
    mavros_param_.set_param(ev_ctrl_write);

    px4_param_types::EKF2_HGT_REF hgt_ref_write;
    hgt_ref_write.enable_vision();
    mavros_param_.set_param(hgt_ref_write);

    // TODO: 这里需要重启EKF2

    // 重读确认
    const int max_retry = 5;
    const double retry_interval_sec = 0.2;
    for (int i = 0; i < max_retry; ++i) {
        ros::Duration(retry_interval_sec).sleep();
        if (check_param()) {
            return;
        }
    }

    // 修改失败，重试无效，抛出异常
    throw std::runtime_error("Failed to apply fusion params after retries: "
                             "require EKF2_EV_CTRL(hpos,vpos,yaw)=on and EKF2_HGT_REF=vision");
}

bool PX4_OriginController::check_px4_basic_state() {
    // px4的基础状态，指的是什么呢？
    // 1. 处于position模式(这点是我们需要确认的，默认是position模式，遥控器也是双回中)
    // 2. 处于未解锁模式
    // 3. 着地检测器处于地面
    const control_common::Mavros_State st = mavros_helper_.get_state();

    const bool mode_ok = (st.flight_mode == control_common::FlightMode::Posctl);
    const bool land_ok = (st.landed_state == control_common::LandedState::OnGround);
    const bool arm_ok = (st.armed == false);

    return mode_ok && land_ok && arm_ok;
}

bool PX4_OriginController::check_mavros_stream_ready() {
    return mavros_helper_.is_ready();
}

bool PX4_OriginController::check_odom_freshness() {
    return ((ros::Time::now() - uav_odometry_.timestamp).toSec() < 0.15);
}

bool PX4_OriginController::check_odom_for_fusion(control_common::UAVStateEstimate& fuse_odom) {
    // 1. 时间戳有效
    if (fuse_odom.timestamp.isZero()) {
        return false;
    }
    // 2.新鲜度
    if (!check_odom_freshness()) {
        return false;
    }
    // 3, 数值有界
    auto finite3 = [](double a, double b, double c) {
        return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
    };
    if (!finite3(fuse_odom.position.x(), fuse_odom.position.y(), fuse_odom.position.z()))
        return false;
    if (!finite3(fuse_odom.velocity.x(), fuse_odom.velocity.y(), fuse_odom.velocity.z()))
        return false;
    if (!finite3(fuse_odom.bodyrate.x(), fuse_odom.bodyrate.y(), fuse_odom.bodyrate.z()))
        return false;
    // 4. 四元数范数检查
    const double qn = std::sqrt(fuse_odom.orientation.w() * fuse_odom.orientation.w() +
                                fuse_odom.orientation.x() * fuse_odom.orientation.x() +
                                fuse_odom.orientation.y() * fuse_odom.orientation.y() +
                                fuse_odom.orientation.z() * fuse_odom.orientation.z());

    if (qn < 1e-6)
        return false;  // 退化
    if (std::fabs(qn - 1.0) > 0.2)
        return false;  // 偏离过大
    // 到这里都ok
    return true;
}

void PX4_OriginController::pub_px4_state_timer_cb(const ros::TimerEvent&) {
    mavros_helper_.pub_px4_state();
}

void PX4_OriginController::pub_vision_fuse_timer_cb(const ros::TimerEvent&) {
    if (has_uav_odometry_) {
        mavros_helper_.pub_vision_pose(uav_odometry_);
    }
}
