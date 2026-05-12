/**
 * @file circle_trajectory.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> move circle by trajectory -> return
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Surnay无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物，
 * 运行结果：
 * sunray系列无人机在当前位置起飞，使用轨迹控制命令move_trajectory，
 * 沿局部坐标系圆形轨迹飞行一圈，最后返航
 * [补充说明]:
 * 如需修改起飞降落的相关参数，请修改control文件夹中
 * sunray_uav_control文件夹下config文件夹中的sunray_control_config.yaml中对应参数
 *
 */

// ros_msg_utils头文件，包含了大部分情况下需要的头文件
#include <ros_msg_utils.h>

// 定义一些全局变量，后续使用
std::string node_name;
std::string agent_key;
sunray_msgs::UAVControlState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
nav_msgs::Odometry uav_odom;

// 全局使用的控制命令发布者
ros::Publisher control_cmd_pub;

// 圆形轨迹参数
Eigen::Vector3d circle_center = Eigen::Vector3d::Zero();
constexpr double kPi = 3.14159265358979323846;
constexpr double kCircleRadius = 1.0;                                  // 半径
constexpr double kCircleAngularSpeed = 0.2;                           // 角速度
constexpr double kCircleDuration = 2.0 * kPi / kCircleAngularSpeed;   // 一圈轨迹总时间
constexpr double kTransitSpeed = 0.3;                                 // 切入圆起点的速度
constexpr double kControlDt = 0.02;

struct TrajectoryReference {
    Eigen::Vector3d desired_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_vel{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_acc{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_jerk{Eigen::Vector3d::Zero()};
};

double compute_line_duration(const Eigen::Vector3d& start_point,
                             const Eigen::Vector3d& end_point,
                             const double speed) {
    const double distance = (end_point - start_point).norm();
    if (speed <= 1e-6 || distance <= 1e-6) {
        return kControlDt;
    }
    const double duration = distance / speed;
    return duration > kControlDt ? duration : kControlDt;
}

TrajectoryReference make_hold_reference(const Eigen::Vector3d& desired_pos) {
    TrajectoryReference reference;
    reference.desired_pos = desired_pos;
    return reference;
}

TrajectoryReference sample_line_reference(double elapsed_time,
                                          const Eigen::Vector3d& start_point,
                                          const Eigen::Vector3d& end_point) {
    TrajectoryReference reference;
    const Eigen::Vector3d delta = end_point - start_point;
    const double distance = delta.norm();
    if (distance <= 1e-6) {
        reference.desired_pos = end_point;
        return reference;
    }

    const double duration = compute_line_duration(start_point, end_point, kTransitSpeed);
    double clamped_time = elapsed_time;
    if (clamped_time < 0.0) {
        clamped_time = 0.0;
    }
    if (clamped_time > duration) {
        clamped_time = duration;
    }

    const Eigen::Vector3d direction = delta / distance;
    const double alpha = clamped_time / duration;

    reference.desired_pos = start_point + alpha * delta;
    if (clamped_time >= duration) {
        reference.desired_pos = end_point;
        reference.desired_vel = Eigen::Vector3d::Zero();
    } else {
        reference.desired_vel = direction * kTransitSpeed;
    }
    return reference;
}

TrajectoryReference sample_circle_reference(double elapsed_time, const Eigen::Vector3d& center_point) {
    const double angle = kCircleAngularSpeed * elapsed_time;
    const double angular_speed_sq = kCircleAngularSpeed * kCircleAngularSpeed;
    const double angular_speed_cube = angular_speed_sq * kCircleAngularSpeed;

    TrajectoryReference reference;
    reference.desired_pos = center_point;
    reference.desired_pos.x() += kCircleRadius * std::cos(angle);
    reference.desired_pos.y() += kCircleRadius * std::sin(angle);

    reference.desired_vel.x() = -kCircleRadius * kCircleAngularSpeed * std::sin(angle);
    reference.desired_vel.y() = kCircleRadius * kCircleAngularSpeed * std::cos(angle);

    reference.desired_acc.x() = -kCircleRadius * angular_speed_sq * std::cos(angle);
    reference.desired_acc.y() = -kCircleRadius * angular_speed_sq * std::sin(angle);

    reference.desired_jerk.x() = kCircleRadius * angular_speed_cube * std::sin(angle);
    reference.desired_jerk.y() = -kCircleRadius * angular_speed_cube * std::cos(angle);
    return reference;
}

void publish_trajectory_command(const TrajectoryReference& reference) {
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY;
    uav_cmd.desired_pos.x = reference.desired_pos.x();
    uav_cmd.desired_pos.y = reference.desired_pos.y();
    uav_cmd.desired_pos.z = reference.desired_pos.z();
    uav_cmd.desired_vel.x = reference.desired_vel.x();
    uav_cmd.desired_vel.y = reference.desired_vel.y();
    uav_cmd.desired_vel.z = reference.desired_vel.z();
    uav_cmd.desired_acc.x = reference.desired_acc.x();
    uav_cmd.desired_acc.y = reference.desired_acc.y();
    uav_cmd.desired_acc.z = reference.desired_acc.z();
    uav_cmd.desired_jerk.x = reference.desired_jerk.x();
    uav_cmd.desired_jerk.y = reference.desired_jerk.y();
    uav_cmd.desired_jerk.z = reference.desired_jerk.z();
    uav_cmd.desired_yaw = 0.0;
    uav_cmd.desired_yaw_rate = 0.0;
    uav_cmd.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;
    control_cmd_pub.publish(uav_cmd);
}

// 到达判定参数
Eigen::Vector3d arrived_error(0.15, 0.15, 0.15);
double keep_time = 1.0;
ros::Time arrived_time(0);

// 判断是否到达目标点
bool check_arrived(const Eigen::Vector3d& desired_pos, const nav_msgs::Odometry& odom) {
    bool arrive_state = false;
    Eigen::Vector3d err_pos{Eigen::Vector3d::Zero()};
    err_pos.x() = std::abs(desired_pos.x() - odom.pose.pose.position.x);
    err_pos.y() = std::abs(desired_pos.y() - odom.pose.pose.position.y);
    err_pos.z() = std::abs(desired_pos.z() - odom.pose.pose.position.z);
    if ((err_pos.x() < arrived_error.x()) && (err_pos.y() < arrived_error.y()) &&
        (err_pos.z() < arrived_error.z())) {
        if (arrived_time.isZero()) {
            arrived_time = ros::Time::now();
        }
    } else {
        arrived_time = ros::Time(0);
    }
    if (!arrived_time.isZero()) {
        if ((ros::Time::now() - arrived_time).toSec() > keep_time) {
            arrive_state = true;
        }
    }
    return arrive_state;
};

// 退出信号捕获函数
void mySignalHandler(int sig) {
    std::cout << "sunray_uav_example [circle_trajectory] node exit..." << std::endl;
    ros::shutdown();
    exit(EXIT_SUCCESS);  // 或者使用 exit(0)
}

// 无人机状态回调函数
void uav_state_callback(const sunray_msgs::UAVControlState::ConstPtr& msg) {
    uav_state = *msg;
}

// 无人机里程计回调函数
void uav_odom_callback(const nav_msgs::OdometryConstPtr& msg) {
    uav_odom = *msg;
}

// 主函数
int main(int argc, char** argv) {
    // ros节点初始化
    ros::init(argc, argv, "circle_trajectory_node");

    // 创建全局句柄与私有句柄
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 注册退出信号捕获函数
    signal(SIGINT, mySignalHandler);

    bool use_private_agent_key = false;
    private_nh.param("use_private_agent_key", use_private_agent_key, false);
    agent_key = use_private_agent_key
        ? sunray_common::get_agent_key_from_private()
        : sunray_common::get_agent_key_from_global();
    // 读取完成，开始初始化ros订阅者与发布者
    // [订阅] 无人机状态
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVControlState>(
        agent_key + "/sunray/uav_control/control_state", 10, uav_state_callback);
    // [订阅] 无人机当前里程计
    ros::Subscriber uav_odom_sub = nh.subscribe<nav_msgs::Odometry>(
        agent_key + "/sunray/localization/local_odom", 10, uav_odom_callback);
    // [发布] 无人机控制指令
    control_cmd_pub =
        nh.advertise<sunray_msgs::UAVControlCMD>(agent_key + "/sunray/uav_control/control_cmd", 1);

    // 进入主循环
    int times = 0;
    while (ros::ok() && (uav_state.control_state != sunray_msgs::UAVControlState::INIT)) {
        // 首先判断当前无人机控制状态
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            ROS_ERROR("uav control state can't init success ...");
    }
    // 清理循环变量
    times = 0;
    // 初始化成功，发布起飞
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd_source = sunray_msgs::UAVControlCMD::EXAMPLE_DEMO;
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::TAKEOFF;
    control_cmd_pub.publish(uav_cmd);
    // 判断是否完成起飞
    // 当uav_state从INIT变成HOVER时，认为起飞完成
    while (ros::ok() &&
           (uav_state.control_state != sunray_msgs::UAVControlState::HOVER)) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            ROS_INFO("uav is takeoffing and wait for enter hover ");
    }
    // 清理循环变量
    times = 0;
    // 完成起飞后，从当前位置生成圆轨迹
    ROS_INFO("uav takeoff successfully and now track circle trajectory");
    circle_center.x() = uav_odom.pose.pose.position.x;
    circle_center.y() = uav_odom.pose.pose.position.y;
    circle_center.z() = uav_odom.pose.pose.position.z;

    Eigen::Vector3d current_hover_point = circle_center;
    Eigen::Vector3d start_point = circle_center;
    start_point.x() += kCircleRadius;

    // 先切入到圆轨迹起点
    const ros::Time transit_start_time = ros::Time::now();
    const double transit_duration =
        compute_line_duration(current_hover_point, start_point, kTransitSpeed);
    bool transit_finished = false;
    arrived_time = ros::Time(0);

    while (ros::ok()) {
        const double elapsed_time = (ros::Time::now() - transit_start_time).toSec();
        if (!transit_finished) {
            const TrajectoryReference reference =
                sample_line_reference(elapsed_time, current_hover_point, start_point);
            publish_trajectory_command(reference);
            if (elapsed_time >= transit_duration) {
                transit_finished = true;
                arrived_time = ros::Time(0);
                ROS_INFO("uav reached circle start point and now track circle trajectory");
            }
        } else {
            publish_trajectory_command(make_hold_reference(start_point));
            if (check_arrived(start_point, uav_odom)) {
                break;
            }
        }
        ros::spinOnce();
        ros::Duration(kControlDt).sleep();
    }

    // 此处进入轨迹控制追踪圆形轨迹，控制频率设定为20ms一次，也就是50Hz
    bool circle_finished = false;
    const ros::Time trajectory_start_time = ros::Time::now();
    arrived_time = ros::Time(0);

    while (ros::ok()) {
        const double elapsed_time = (ros::Time::now() - trajectory_start_time).toSec();
        if (!circle_finished) {
            const TrajectoryReference reference = sample_circle_reference(elapsed_time, circle_center);
            publish_trajectory_command(reference);
            if (elapsed_time >= kCircleDuration) {
                circle_finished = true;
                arrived_time = ros::Time(0);
                ROS_INFO("circle trajectory completed, now stabilize at start point");
            }
        } else {
            publish_trajectory_command(make_hold_reference(start_point));
            if (check_arrived(start_point, uav_odom)) {
                ROS_INFO("circle trajectory example finished and now to return ");
                break;
            }
        }
        ros::spinOnce();
        ros::Duration(kControlDt).sleep();
    }

    // 发布返航命令
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::RETURN;
    control_cmd_pub.publish(uav_cmd);
    // 一次publish需要ros的spin才能实现，所以这里需要进行一次spin，如果觉得这样不保险，可以选择while循环查询进入到RETURN后再退出
    ros::spinOnce();
    // RETURN可以在sunray_uav_config.yaml文件中被配置为自动降落或切入悬停，如果配置文件中设置为切入悬停，则会导致本节点会持续悬停

    // 直接结束节点或等待成功降落？
    ROS_INFO("uav is enter return mode and [circle_trajectory] demo finished,quit !");
    return 0;
}
