/**
 * @file circle_velocity_yaw.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> move circle by velocity with tangent yaw -> return
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Sunray无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物
 * 运行结果：
 * sunray系列无人机在当前位置起飞，使用惯性系速度控制命令move_velocity，
 * 沿局部坐标系圆形轨迹飞行一圈，先对准切线方向，再通过绝对 yaw 持续保持机头沿圆轨迹切线方向，最后返航
 * [补充说明]:
 * 如需修改起飞降落的相关参数，请修改control文件夹中
 * sunray_uav_control/config/sunray_control_base.yaml 以及
 * config/airframes/<airframe_type>.yaml 中对应参数
 *
 */

#include <ros_msg_utils.h>

#include <cmath>

std::string node_name;
std::string agent_key;
sunray_msgs::UAVControlState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
nav_msgs::Odometry uav_odom;
bool has_odom = false;

ros::Publisher control_cmd_pub;

Eigen::Vector3d circle_center = Eigen::Vector3d::Zero();
constexpr double kPi = 3.14159265358979323846;
constexpr double kCircleRadius = 1.0;
constexpr double kCircleAngularSpeed = 0.2;
constexpr double kCircleDuration = 2.0 * kPi / kCircleAngularSpeed;
constexpr double kControlDt = 0.02;

float kp = 0.3;
float max_vel = 0.5;
float min_vel = -0.5;

struct CircleReference {
    Eigen::Vector3d desired_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_vel_ff{Eigen::Vector3d::Zero()};
    double desired_yaw{0.0};
};

double compute_yaw_from_velocity(const Eigen::Vector3d& velocity,
                                 double fallback_yaw = 0.0) {
    const double xy_speed = std::hypot(velocity.x(), velocity.y());
    if (xy_speed < 1e-6) {
        return fallback_yaw;
    }
    return std::atan2(velocity.y(), velocity.x());
}

void velocity_controller_update(const Eigen::Vector3d& desired_pos,
                                const Eigen::Vector3d& desired_vel_ff,
                                double desired_yaw,
                                const nav_msgs::Odometry& odom) {
    Eigen::Vector3d err_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_cmd = desired_vel_ff;
    err_pos.x() = desired_pos.x() - odom.pose.pose.position.x;
    err_pos.y() = desired_pos.y() - odom.pose.pose.position.y;
    err_pos.z() = desired_pos.z() - odom.pose.pose.position.z;

    velocity_cmd.x() += err_pos.x() * kp;
    velocity_cmd.y() += err_pos.y() * kp;
    velocity_cmd.z() += err_pos.z() * kp;

    for (int i = 0; i < 3; ++i) {
        if (velocity_cmd[i] > max_vel)
            velocity_cmd[i] = max_vel;
        if (velocity_cmd[i] < min_vel)
            velocity_cmd[i] = min_vel;
    }

    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_VELOCITY;
    uav_cmd.desired_vel.x = velocity_cmd.x();
    uav_cmd.desired_vel.y = velocity_cmd.y();
    uav_cmd.desired_vel.z = velocity_cmd.z();
    uav_cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    uav_cmd.desired_yaw = desired_yaw;
    uav_cmd.desired_yaw_rate = 0.0;
    control_cmd_pub.publish(uav_cmd);
}

void velocity_controller_update_with_target_yaw(const Eigen::Vector3d& desired_pos,
                                                const Eigen::Vector3d& desired_vel_ff,
                                                double desired_yaw,
                                                const nav_msgs::Odometry& odom) {
    Eigen::Vector3d err_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_cmd = desired_vel_ff;
    err_pos.x() = desired_pos.x() - odom.pose.pose.position.x;
    err_pos.y() = desired_pos.y() - odom.pose.pose.position.y;
    err_pos.z() = desired_pos.z() - odom.pose.pose.position.z;

    velocity_cmd.x() += err_pos.x() * kp;
    velocity_cmd.y() += err_pos.y() * kp;
    velocity_cmd.z() += err_pos.z() * kp;

    for (int i = 0; i < 3; ++i) {
        if (velocity_cmd[i] > max_vel)
            velocity_cmd[i] = max_vel;
        if (velocity_cmd[i] < min_vel)
            velocity_cmd[i] = min_vel;
    }

    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_VELOCITY;
    uav_cmd.desired_vel.x = velocity_cmd.x();
    uav_cmd.desired_vel.y = velocity_cmd.y();
    uav_cmd.desired_vel.z = velocity_cmd.z();
    uav_cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    uav_cmd.desired_yaw = desired_yaw;
    uav_cmd.desired_yaw_rate = 0.0;
    control_cmd_pub.publish(uav_cmd);
}

Eigen::Vector3d arrived_error(0.15, 0.15, 0.15);
double keep_time = 1.0;
ros::Time arrived_time(0);

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

CircleReference sample_circle_reference(double elapsed_time, const Eigen::Vector3d& circle_center) {
    const double angle = kCircleAngularSpeed * elapsed_time;
    CircleReference reference;
    reference.desired_pos = circle_center;
    reference.desired_pos.x() += kCircleRadius * std::cos(angle);
    reference.desired_pos.y() += kCircleRadius * std::sin(angle);
    reference.desired_vel_ff.x() = -kCircleRadius * kCircleAngularSpeed * std::sin(angle);
    reference.desired_vel_ff.y() = kCircleRadius * kCircleAngularSpeed * std::cos(angle);
    reference.desired_vel_ff.z() = 0.0;
    reference.desired_yaw = compute_yaw_from_velocity(reference.desired_vel_ff);
    return reference;
}

void mySignalHandler(int sig) {
    std::cout << "sunray_uav_example [circle_velocity_yaw] node exit..." << std::endl;
    ros::shutdown();
    exit(EXIT_SUCCESS);
}

void uav_state_callback(const sunray_msgs::UAVControlState::ConstPtr& msg) {
    uav_state = *msg;
}

bool is_odom_valid(const nav_msgs::Odometry& odom) {
    return !odom.header.stamp.isZero() && std::isfinite(odom.pose.pose.position.x) &&
           std::isfinite(odom.pose.pose.position.y) &&
           std::isfinite(odom.pose.pose.position.z);
}

void uav_odom_callback(const nav_msgs::OdometryConstPtr& msg) {
    uav_odom = *msg;
    has_odom = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_velocity_yaw_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    signal(SIGINT, mySignalHandler);

    bool use_private_agent_key = false;
    private_nh.param("use_private_agent_key", use_private_agent_key, false);
    agent_key = use_private_agent_key
        ? sunray_common::get_agent_key_from_private()
        : sunray_common::get_agent_key_from_global();

    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVControlState>(
        agent_key + "/sunray/uav_control/control_state", 10, uav_state_callback);
    ros::Subscriber uav_odom_sub = nh.subscribe<nav_msgs::Odometry>(
        agent_key + "/sunray/localization/local_odom", 10, uav_odom_callback);
    control_cmd_pub =
        nh.advertise<sunray_msgs::UAVControlCMD>(agent_key + "/sunray/uav_control/control_cmd", 1);

    int times = 0;
    while (ros::ok() && (uav_state.control_state != sunray_msgs::UAVControlState::INIT)) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            ROS_ERROR("uav control state has not entered INIT yet...");
    }

    times = 0;
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd_source = sunray_msgs::UAVControlCMD::EXAMPLE_DEMO;
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::TAKEOFF;
    uav_cmd.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;
    uav_cmd.desired_yaw = 0.0;
    uav_cmd.desired_yaw_rate = 0.0;
    control_cmd_pub.publish(uav_cmd);

    while (ros::ok() &&
           (uav_state.control_state != sunray_msgs::UAVControlState::HOVER)) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            ROS_INFO("uav is taking off, waiting for HOVER...");
    }

    times = 0;
    // 圆心和闭环速度都依赖当前里程计，进入任务前先等待有效 local_odom
    while (ros::ok() && (!has_odom || !is_odom_valid(uav_odom))) {
        ROS_WARN_THROTTLE(1.0, "waiting for valid local odom before circle velocity yaw task...");
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("uav takeoff succeeded, moving to circle start point");
    circle_center.x() = uav_odom.pose.pose.position.x;
    circle_center.y() = uav_odom.pose.pose.position.y;
    circle_center.z() = uav_odom.pose.pose.position.z;

    Eigen::Vector3d temp_aim;
    temp_aim.x() = uav_odom.pose.pose.position.x + kCircleRadius;
    temp_aim.y() = uav_odom.pose.pose.position.y;
    temp_aim.z() = uav_odom.pose.pose.position.z;
    const double start_tangent_yaw = compute_yaw_from_velocity(
        Eigen::Vector3d(0.0, kCircleRadius * kCircleAngularSpeed, 0.0));

    ros::Rate transit_rate(1.0 / kControlDt);
    while (ros::ok()) {
        velocity_controller_update(temp_aim, Eigen::Vector3d::Zero(), start_tangent_yaw, uav_odom);
        if (check_arrived(temp_aim, uav_odom)) {
            break;
        }
        ros::spinOnce();
        transit_rate.sleep();
    }

    bool circle_finished = false;
    Eigen::Vector3d start_point;
    start_point.x() = uav_odom.pose.pose.position.x;
    start_point.y() = uav_odom.pose.pose.position.y;
    start_point.z() = uav_odom.pose.pose.position.z;
    const ros::Time trajectory_start_time = ros::Time::now();
    ros::Rate control_rate(1.0 / kControlDt);

    while (ros::ok()) {
        const double elapsed_time = (ros::Time::now() - trajectory_start_time).toSec();
        if (!circle_finished) {
            const CircleReference reference = sample_circle_reference(elapsed_time, circle_center);
            velocity_controller_update_with_target_yaw(
                reference.desired_pos, reference.desired_vel_ff, reference.desired_yaw, uav_odom);
            if (elapsed_time >= kCircleDuration) {
                circle_finished = true;
                arrived_time = ros::Time(0);
                ROS_INFO("circle trajectory completed, now stabilize at start point with tangent yaw");
            }
        } else {
            velocity_controller_update(start_point, Eigen::Vector3d::Zero(), start_tangent_yaw, uav_odom);
            if (check_arrived(start_point, uav_odom)) {
                ROS_INFO("circle yaw example finished, now sending RETURN");
                break;
            }
        }
        ros::spinOnce();
        control_rate.sleep();
    }

    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::RETURN;
    uav_cmd.yaw_mode = sunray_msgs::UAVControlCMD::KEEP_YAW;
    control_cmd_pub.publish(uav_cmd);
    ros::spinOnce();

    ROS_INFO("sent RETURN command and [circle_velocity_yaw] demo finished, quit!");
    return 0;
}
