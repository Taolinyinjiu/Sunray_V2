#include "controller/px4_linear_attitude_controller.hpp"
#include "string_uav_namespace_utils.hpp"
#include <sunray_msgs/UAVControllerState.h>
#include <ros/ros.h>
#include "px4_param_manager/px4_param_manager.h"
#include "px4_param_manager/px4_param_decode.h"
#include "px4_param_manager/px4_param_types.h"
#include <yaml-cpp/yaml.h>  // 引入Yaml-cpp库，用于读取yaml文件
// 构造函数
PX4_LinearAttitude_Controller::PX4_LinearAttitude_Controller(ros::NodeHandle& nh)
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
// ------------生命周期相关----------------
bool PX4_LinearAttitude_Controller::init() {
    // 加载参数并校验参数
    load_and_validate_config_or_throw();
    // 将参数加载到linear_control
    controller.load_param(linear_controller_param_);
    // 配置读取完毕，初始化mavros_helper
    MavrosHelper_ConfigList config_list(true);
    if (!mavros_helper_.init(config_list)) {
        throw std::runtime_error("mavros_helper init failed");
    }
    if (fuse_odom_type != 0) {  // 融合外部里程计到px4，设置对应参数,注册定时器
        // 检查外部里程计融合相关参数
        ensure_fusion_param_ready_or_throw();  // 该函数检查失败会抛出异常
        mavros_helper_.set_vision_fuse_type(fuse_odom_type);
        // 创建定时器
        pub_vision_pose_timer_ =
            nh_.createTimer(ros::Duration(1.0 / fuse_odom_frequency),
                            &PX4_LinearAttitude_Controller::pub_vision_fuse_timer_cb,
                            this);
    }
    // 初始化话题发布者
    controller_state_pub_ =
        nh_.advertise<sunray_msgs::UAVControllerState>(uav_ns_ + "/sunray/controller_state", 10);
    // 初始化发布px4_state数据的定时器
    pub_px4_state_timer = nh_.createTimer(ros::Duration(1.0 / pub_px4_state_freq_),
                                          &PX4_LinearAttitude_Controller::pub_px4_state_timer_cb,
                                          this);
    return true;
}
bool PX4_LinearAttitude_Controller::is_ready() {
    ros::Time now = ros::Time::now();
    // 首先判断里程计是否超时
    if (uav_odometry_.timestamp == ros::Time(0)) {
        // 没有接受到里程计数据，返回false
        ROS_INFO("odom msg lost");
        return false;
    } else if ((now - uav_odometry_.timestamp).toSec() > 0.5) {
        // 里程计超时
        ROS_INFO("odom msg timeout");
        return false;
    }
    control_common::Mavros_State mavros_state = mavros_helper_.get_state();
    if (mavros_state.connected != true) {
        // px4未正常连接，判断为超时
        ROS_INFO("mavros not connect");
        return false;
    }
    // 判断mavros_helper提取的各项数据，是否满足要求
    bool mavros_ready = mavros_helper_.is_ready();
    if (!mavros_ready) {
        ROS_INFO("mavros helper not ready");
        return false;
    }

    controller_ready_ = true;
    return true;
}
// -------------状态注入---------------
void PX4_LinearAttitude_Controller::set_current_odom(const control_common::UAVStateEstimate& odom) {
    uav_odometry_ = odom;
    has_uav_odometry_ = true;
}
// -------------运动相关接口------------
void PX4_LinearAttitude_Controller::on_ground_keep_setpoint() {
    // 这里我们也不做判断，只要调用这个接口，我们就输出需要的setpoint
    control_common::Mavros_SetpointAttitude current_setpoint;
    current_setpoint.thrust = 0.0;
    mavros_helper_.pub_attitude_setpoint(current_setpoint);
}
bool PX4_LinearAttitude_Controller::takeoff(double relative_takeoff_height,
                                            double max_takeoff_velocity) {
    // 如果当前已经降落过一次，则清除降落的标志位
    if (land_complete_ == true) {
        land_complete_ = false;
    }
    // 如果当前控制器未就绪，则return
    if (controller_ready_ == false) {
        return false;
    }
    // 如果起飞已经成功，但还是在调用本函数，则输出为在当前点悬停
    if (takeoff_complete_ == true) {
        return hover();
    }
    // 运行到这里，说明进入了起飞阶段
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
        control_common::Mavros_SetpointAttitude setpoint_cmd;
        // 速度置零
        setpoint_cmd.thrust = 0.0;
        // 保持yaw角不变
        setpoint_cmd.orientation = Eigen::Quaterniond::Identity();
        // 发送
        mavros_helper_.pub_attitude_setpoint(setpoint_cmd);
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
            // 这里尝试一种起飞方式，使用get_rotor_speed_up_des函数，得到起飞怠速，近似离地
            if (last_arm_time_ == ros::Time(0)) {
                last_arm_time_ = now;
            }
            if ((now - last_arm_time_).toSec() < motors_speedup_time_) {
                control_common::Mavros_SetpointAttitude setpoint;
                controller_data_types::TargetTrajectoryPoint_t linear_input;
                Linear_AttitudeControl_Output_t output;
                linear_input = get_rotor_speed_up_des(now);

                control_common::Mavros_IMU imu_data = mavros_helper_.get_imu_data();
                output = controller.calculateControl(linear_input, uav_odometry_, imu_data, false);
                setpoint.orientation = output.orientation;
                setpoint.thrust = output.thrust;
                mavros_helper_.pub_attitude_setpoint(setpoint);

                // 更新缓存
                last_setpoint_ = setpoint;
                return false;
            } else {  // 怠速阶段结束，开始准备起飞
                // 得到五次项曲线输出
                curve::QuinticCurveState curve_result;
                curve_result = quint_curve_.get_result();
                // 使用五次项输出作为LInearControl的输入
                controller_data_types::TargetTrajectoryPoint_t linear_input_state;
                Linear_AttitudeControl_Output_t output;
                linear_input_state.position = curve_result.position;
                linear_input_state.velocity = curve_result.velocity;
                linear_input_state.acceleration = curve_result.acceleration;
                linear_input_state.yaw = takeoff_yaw_;  // yaw角使用之前锁存的yaw
                control_common::Mavros_IMU imu_data = mavros_helper_.get_imu_data();
                output =
                    controller.calculateControl(linear_input_state, uav_odometry_, imu_data, false);
                control_common::Mavros_SetpointAttitude setpoint;
                setpoint.orientation = output.orientation;
                setpoint.thrust = output.thrust;

                // 发布输出
                mavros_helper_.pub_attitude_setpoint(setpoint);
                // 更新缓存
                last_setpoint_ = setpoint;
                // 到位检查
                // 在这里判断是否到达期望的起飞目标点
                Eigen::Vector3d pos_err_vec =
                    uav_odometry_.position - quint_curve_.get_end_position();
                double pos_err = pos_err_vec.norm();  // 推荐：位置误差标量

                Eigen::Vector3d vel_err_vec = uav_odometry_.velocity - Eigen::Vector3d::Zero();
                double vel_err = vel_err_vec.norm();  // 推荐：速度误差标量
                // ROS_INFO("pos_err : %f", pos_err);
                // ROS_INFO("vel_err : %f", vel_err);
                if (pos_err < 0.3 && vel_err < 0.15) {
                    if (start_checkout_takeoff_success_time_ == ros::Time(0)) {
                        start_checkout_takeoff_success_time_ = now;
                    }
                    if ((now - start_checkout_takeoff_success_time_).toSec() >
                        takeoff_success_keep_time_s) {
                        takeoff_complete_ = true;
                        hover_point = quint_curve_.get_start_position();
                        hover_point.z() += relative_takeoff_height;
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
    }
    return false;
}

void applyThrustShaping(double& thrust, bool near_ground, bool landed) {
    // 1. 确定当前的推力上限
    // 如果已经触地，给一个极小的维持压力；如果只是靠近地面，给一个略低于悬停的上限
    double thrust_cap = landed ? 0.05 : (near_ground ? 0.31 : 1.0);

    // 2. 核心限幅：确保推力在 [最小怠速, 动态上限] 之间
    thrust = std::clamp(thrust, 0.02, thrust_cap);
};

bool PX4_LinearAttitude_Controller::land(bool land_type, double max_land_velocity) {
    // 首先，我们要求检查land_type ，当land_type==1切换为px4的auto_land
    if (land_type == 1) {
        // 切换为px4的auto land模式
        mavros_helper_.set_px4_mode(control_common::FlightMode::AutoLand);
        bool land_state =
            mavros_helper_.get_state().landed_state == control_common::LandedState::OnGround;
        return land_state;
    }
    ros::Time now = ros::Time::now();
    // 其次我们检查land是否已经成功(本函数会被反复调用)
    if (land_complete_ == true) {
        return true;
    }
    // 最后我们检查是否为第一次进入本函数
    if (landing_time_ == ros::Time(0)) {
        landing_time_ = now;
        Eigen::Vector3d landing_start_point_ =
            uav_odometry_.position;  // 使用当前里程计作为降落开始的点，锁定xy的位置
        Eigen::Vector3d landing_end_point_ = uav_odometry_.position;
        landing_end_point_.z() =
            takeoff_ground_height - 0.1;  // 在这里减去0.2以防止传感器漂移产生的z轴偏差
        Eigen::Vector3d landing_end_vel = Eigen::Vector3d(0, 0, -0.1);
        land_yaw_ = mavros_helper_.get_yaw_rad();  // 获取当前的yaw角
        quint_curve_.clear();                      // 清除曲线参数
        quint_curve_.set_start_trajpoint(landing_start_point_,
                                         Eigen::Vector3d::Zero());  // 设置起点为当前位置，速度为零
        quint_curve_.set_end_trajpoint(landing_end_point_, landing_end_vel);  // 设置终点参数
        quint_curve_.set_curve_maxvel(max_land_velocity);  // 根据最大速度反推时间
    }
    // 首先我们讨论这样一个问题，我们要一个怎样的下降？
    // 1. 匀速下降(恒定速度)
    // 2. 非匀速的下降，速度平滑变化的下降，在离地的时候速度正好到0(曲线规划)
    // 假设我们使用后者，我们就需要在上面的函数中，对曲线参数进行初始化
    bool near_ground = uav_odometry_.position.z() <= takeoff_ground_height + 0.05;
    bool landed_by_velocity = false;
    const bool velocity_low = std::abs(uav_odometry_.velocity.x()) < 0.1 &&
                              std::abs(uav_odometry_.velocity.y()) < 0.1 &&
                              std::abs(uav_odometry_.velocity.z()) < 0.1;
    if (near_ground && velocity_low) {
        if (start_land_time_ == ros::Time(0)) {
            start_land_time_ = now;
        } else {
            if ((now - start_land_time_).toSec() > 2.0) {
                landed_by_velocity = true;
            }
        }
    } else {
        start_land_time_ = ros::Time(0);
        landed_by_velocity = false;
    }
    // 获取五次项曲线的输出
    control_common::LandedState px4_land_state = mavros_helper_.get_state().landed_state;
    bool px4_land_status_ = px4_land_state == control_common::LandedState::OnGround;
    const bool landed_detected = px4_land_status_ || landed_by_velocity;
    controller_data_types::TargetTrajectoryPoint_t linear_input_state;
    Linear_AttitudeControl_Output_t output;
    curve::QuinticCurveState curve_result;
    curve_result = quint_curve_.get_result();
    linear_input_state.position = curve_result.position;
    linear_input_state.velocity = curve_result.velocity;
    linear_input_state.acceleration = curve_result.acceleration;
    linear_input_state.yaw = takeoff_yaw_;  // yaw角使用之前锁存的yaw
    control_common::Mavros_IMU imu_data = mavros_helper_.get_imu_data();
    output = controller.calculateControl(linear_input_state, uav_odometry_, imu_data, false);
    control_common::Mavros_SetpointAttitude setpoint;
    setpoint.orientation = output.orientation;
    setpoint.thrust = output.thrust;

    // 对推力进行整形
    applyThrustShaping(setpoint.thrust, near_ground, landed_by_velocity);
    std::cout << "----------------------------------" << std::endl;
    std::cout << "setpoint thrust: " << setpoint.thrust << std::endl;
    std::cout << "velocity_low: " << velocity_low << std::endl;
    std::cout << "landed_by_velocity: " << landed_by_velocity << std::endl;
    std::cout << "px4_land_status_: " << px4_land_status_ << std::endl;
    if (landed_detected && landed_by_velocity) {  // 考虑传感器漂移，这里用||逻辑
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
                landing_time_ = ros::Time(0);
            }
            return land_complete_;
        }
    } else {
        land_touchground_time_ = ros::Time(0);
    }
    mavros_helper_.pub_attitude_setpoint(setpoint);
    last_setpoint_ = setpoint;
    return false;
}

// bool PX4_LinearAttitude_Controller::land(bool land_type, double max_land_velocity) {
//     if (land_type == 1) {
//         // 切换为px4的auto land模式
//         mavros_helper_.set_px4_mode(control_common::FlightMode::AutoLand);
//         bool land_state =
//             mavros_helper_.get_state().landed_state == control_common::LandedState::OnGround;
//         return land_state;
//     }
//     ros::Time now = ros::Time::now();
//     if (land_complete_ == true) {
//         return true;
//     }
//     //
//     降落的基本思路是，要求以一个稳定的，小的速度到达地面，这样能够保证imu数据是稳定的，基于imu映射的推力也是稳定的
//     //
//     所以我们使用递归时间降落，也就是，记录进入land函数的时间戳，然后构造一个kp用于递减当前的pos z
//     if (landing_time_ == ros::Time(0)) {
//         landing_time_ = now;
//         land_point_ = uav_odometry_.position;
//         land_yaw_ = mavros_helper_.get_yaw_rad();
//     }
//     controller_data_types::TargetTrajectoryPoint_t linear_input_state;
//     linear_input_state.position = land_point_;

//     linear_input_state.position.z() -= 0.5 * (now - landing_time_).toSec();

//     linear_input_state.velocity.z() = -0.1;
//     linear_input_state.yaw = land_yaw_;
//     Linear_AttitudeControl_Output_t output;
//     control_common::Mavros_IMU imu_data = mavros_helper_.get_imu_data();
//     output = controller.calculateControl(linear_input_state, uav_odometry_, imu_data, false);
//     control_common::Mavros_SetpointAttitude setpoint;
//     setpoint.orientation = output.orientation;
//     setpoint.thrust = output.thrust;
//     // 发布输出
//     mavros_helper_.pub_attitude_setpoint(setpoint);
//     // 更新缓存
//     last_setpoint_ = setpoint;
//     // 到位检查
//     bool velocity_low = (uav_odometry_.velocity.norm() < 0.15);
//     control_common::LandedState px4_landed = mavros_helper_.get_state().landed_state;
//     bool px4_land = px4_landed == control_common::LandedState::OnGround;
//     if (velocity_low && px4_land) {  // 考虑传感器漂移，这里用||逻辑
//         if (land_touchground_time_ == ros::Time(0)) {
//             land_touchground_time_ = now;
//         }
//         if ((now - land_touchground_time_).toSec() > 1.0) {
//             mavros_helper_.set_arm(false);  // 上锁
//             if (mavros_helper_.get_state().armed == false) {
//                 // 上锁成功，清理上下文
//                 land_complete_ = true;
//                 takeoff_complete_ = false;
//                 quint_curve_.clear();
//                 start_land_time_ = ros::Time(0);
//                 land_near_ground_ = false;
//                 land_touchground_time_ = ros::Time(0);
//                 landing_time_ = ros::Time(0);
//             }
//             return land_complete_;
//         }
//     } else {
//         land_touchground_time_ = ros::Time(0);
//     }
//     return false;
// }
bool PX4_LinearAttitude_Controller::hover() {
    // hover模式，设定当前里程计位置为期望位置，速度加速度加加速度置零
    controller_data_types::TargetTrajectoryPoint_t linear_input_state;
    linear_input_state.position = hover_point;
    control_common::Mavros_IMU imu_data = mavros_helper_.get_imu_data();
    Linear_AttitudeControl_Output_t output;
    output = controller.calculateControl(linear_input_state, uav_odometry_, imu_data, true);
    control_common::Mavros_SetpointAttitude setpoint;
    setpoint.orientation = output.orientation;
    setpoint.thrust = output.thrust;
    mavros_helper_.pub_attitude_setpoint(setpoint);

    controller.estimateThrustModel(imu_data.accelection);
    return true;
}
bool PX4_LinearAttitude_Controller::emergency_kill() {
    return false;
}
bool PX4_LinearAttitude_Controller::move_point(controller_data_types::TargetPoint_t point) {
    // 记录当前时间戳
    ros::Time now = ros::Time::now();

    // 新目标判据，使用==会由于数值漂移导致异常情况(虽然很少出现)
    constexpr double kNewTargetPosEps = 1e-3;
    // 新目标状态
    bool is_new_target = false;
    // 检查是否为新目标
    const double dp = (point.position - last_point_.position).norm();
    // 位置误差大于常量误差，为新值
    if (dp > kNewTargetPosEps) {
        is_new_target = true;
    }
    // 如果为新目标，则清除一些上下文参数
    if (is_new_target) {
        move_point_arrive_state_ = false;
        start_move_arrive_time_ = ros::Time(0);
        last_point_ = point;
    }

    controller_data_types::TargetTrajectoryPoint_t linear_input_state;
    linear_input_state.position = point.position;
    linear_input_state.yaw = point.yaw;
    control_common::Mavros_IMU imu_data = mavros_helper_.get_imu_data();
    Linear_AttitudeControl_Output_t output;
    output = controller.calculateControl(linear_input_state, uav_odometry_, imu_data);
    control_common::Mavros_SetpointAttitude setpoint;
    setpoint.orientation = output.orientation;
    setpoint.thrust = output.thrust;
    mavros_helper_.pub_attitude_setpoint(setpoint);
    // 更新缓存
    last_setpoint_ = setpoint;
    // 检查误差
    Eigen::Vector3d pos_err_vec = uav_odometry_.position - point.position;
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
            point_complete_ = true;
            hover_point = last_point_.position;
        }
    } else {
        start_move_arrive_time_ = ros::Time(0);
        point_complete_ = false;
    }
    return move_point_arrive_state_;
}
bool PX4_LinearAttitude_Controller::move_velocity(
    controller_data_types::TargetVelocity_t velocity) {
    return false;
}
bool PX4_LinearAttitude_Controller::move_trajectory(
    controller_data_types::TargetTrajectoryPoint_t trajpoint) {
    return false;
}
bool PX4_LinearAttitude_Controller::move_point_body(controller_data_types::TargetPoint_t point) {
    return false;
}
bool PX4_LinearAttitude_Controller::move_velocity_body(
    controller_data_types::TargetVelocity_t velocity) {
    return false;
}
bool PX4_LinearAttitude_Controller::move_point_wgs84(geographic_msgs::GeoPoint point) {
    return false;
}
// ---------------------运动状态查询接口-----------------------
bool PX4_LinearAttitude_Controller::is_takeoff_complete() {
    return takeoff_complete_;
}
bool PX4_LinearAttitude_Controller::is_point_complete() {
    return point_complete_;
}
bool PX4_LinearAttitude_Controller::is_land_complete() {
    return land_complete_;
}
// ----------------------控制器状态话题更新函数-----------------
void PX4_LinearAttitude_Controller::pub_controller_state() {
    return;
}

// -----------------------private部分函数-----------------
void PX4_LinearAttitude_Controller::load_and_validate_config_or_throw() {
    // 首先在构造函数中我们已经判断了config_yamlfile_path_非空,因此这里不再判断
    YAML::Node root;  // 构造一个YAML文件的根节点
    // 由于读取的过程可能引发异常，因此使用try语法
    try {
        root =
            YAML::LoadFile(config_yamlfile_path_);  // 从指定的路径中读取yaml文件并解析为YAML::Node
    } catch (const YAML::Exception& e) {  // 如果解析的过程中发生错误，捕捉异常
        throw std::runtime_error("Failed to load yaml file '" + config_yamlfile_path_ + ":" +
                                 e.what());
    }
    // 顺利读取，取出字段basic_param的部分
    const YAML::Node basic_param = root["basic_param"];
    // 如果basic_param为空，或者不是键值对的形式，则抛出异常
    if (!basic_param || !basic_param.IsMap()) {
        throw std::runtime_error("the yaml file '" + config_yamlfile_path_ +
                                 "' is missing a valid basic_param map");
    }
    // 由于OriginalController需要的参数不多，我们直接拿字段读
    if (!basic_param["fuse_odom_type"]) {
        throw std::runtime_error("miss param 'fuse_odom_type'");
    } else {
        fuse_odom_type = basic_param["fuse_odom_type"].as<int>();
    }
    if (!basic_param["fuse_odom_frequency"]) {
        throw std::runtime_error("miss param 'fuse_odom_frequency'");
    } else {
        fuse_odom_frequency = basic_param["fuse_odom_frequency"].as<double>();
    }
    // 检查一下参数是否正常
    if (fuse_odom_type != 0 && fuse_odom_type != 1 && fuse_odom_type != 2) {
        throw std::runtime_error("param 'fuse_odom_type' value must be 0,1,2");
    }
    // 限制融合的频率
    fuse_odom_frequency = std::max(10.0, fuse_odom_frequency);
    fuse_odom_frequency = std::min(200.0, fuse_odom_frequency);
    // 目前没有使用到uav的质量，后续可能会有吧
    if (!basic_param["gravity"]) {
        throw std::runtime_error("miss param 'gravity'");
    } else {
        linear_controller_param_.gravity = basic_param["gravity"].as<double>();
    }
    // 构造linear_attitude_controller_param字段对应节点
    const YAML::Node linear_param = root["linear_attitude_controller_param"];
    if (!linear_param["output_type"]) {
        throw std::runtime_error("miss param 'output_type'");
    } else {
        linear_controller_param_.output_type = linear_param["output_type"].as<int>();
    }
    // 构造子节点访问hover_percentage
    const YAML::Node thrust_model = linear_param["thrust_model"];
    if (!thrust_model["accurate_thrust_model"]) {
        throw std::runtime_error("miss param 'accurate_thrust_model'");
    } else {
        linear_controller_param_.accurate_thrust_model =
            thrust_model["accurate_thrust_model"].as<bool>();
    }
    if (!thrust_model["hover_percentage"]) {
        throw std::runtime_error("miss param 'hover_percentage'");
    } else {
        linear_controller_param_.hover_percentage = thrust_model["hover_percentage"].as<double>();
    }
    // 构造增益节点
    const YAML::Node gain_param = linear_param["gain"];
    if (gain_param) {
        if (gain_param["pos_kp"] && gain_param["pos_kp"].IsSequence()) {
            linear_controller_param_.pos_gain.Kx = gain_param["pos_kp"][0].as<double>();
            linear_controller_param_.pos_gain.Ky = gain_param["pos_kp"][1].as<double>();
            linear_controller_param_.pos_gain.Kz = gain_param["pos_kp"][2].as<double>();
        } else {
            throw std::runtime_error("miss param 'gain_param pos_kp'");
        }
        if (gain_param["vel_kp"] && gain_param["vel_kp"].IsSequence()) {
            linear_controller_param_.vel_gain.Kx = gain_param["vel_kp"][0].as<double>();
            linear_controller_param_.vel_gain.Ky = gain_param["vel_kp"][1].as<double>();
            linear_controller_param_.vel_gain.Kz = gain_param["vel_kp"][2].as<double>();
        } else {
            throw std::runtime_error("miss param 'gain_param vel_kp'");
        }
    } else {
        throw std::runtime_error("miss param 'gain_param'");
    }
}
void PX4_LinearAttitude_Controller::ensure_fusion_param_ready_or_throw() {
    if (fuse_odom_type == 0) {
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
bool PX4_LinearAttitude_Controller::check_px4_basic_state() {
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
bool PX4_LinearAttitude_Controller::check_mavros_stream_ready() {
    return mavros_helper_.is_ready();
}
bool PX4_LinearAttitude_Controller::check_odom_freshness() {
    return ((ros::Time::now() - uav_odometry_.timestamp).toSec() < 0.15);
}
bool PX4_LinearAttitude_Controller::check_odom_for_fusion(
    control_common::UAVStateEstimate& fuse_odom) {
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
void PX4_LinearAttitude_Controller::pub_px4_state_timer_cb(const ros::TimerEvent&) {
    mavros_helper_.pub_px4_state();
}
void PX4_LinearAttitude_Controller::pub_vision_fuse_timer_cb(const ros::TimerEvent&) {
    if (has_uav_odometry_) {
        mavros_helper_.pub_vision_pose(uav_odometry_);
    }
}

controller_data_types::TargetTrajectoryPoint_t
PX4_LinearAttitude_Controller::get_rotor_speed_up_des(const ros::Time now) {
    double delta_t = (now - last_arm_time_).toSec();
    double des_a_z =
        exp((delta_t - 3.0) * 6.0) * 7.0 - 7.0;  // Parameters 6.0 and 7.0 are just heuristic values
                                                 // which result in a saticfactory curve.
    if (des_a_z > 0.1) {
        ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
        des_a_z = 0.0;
    }

    controller_data_types::TargetTrajectoryPoint_t des;
    Eigen::Vector3d start_pos = quint_curve_.get_start_position();
    des.position = start_pos;
    des.velocity = Eigen::Vector3d::Zero();
    des.acceleration = Eigen::Vector3d(0, 0, des_a_z);
    des.jerk = Eigen::Vector3d::Zero();
    des.yaw = takeoff_yaw_;
    des.yaw_rate = 0.0;

    return des;
}
