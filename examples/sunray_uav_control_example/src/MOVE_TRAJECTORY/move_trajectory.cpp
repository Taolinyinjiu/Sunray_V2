/**
 * @file move_trajectory.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> move trajectory -> return
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Surnay无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物，
 * 运行结果：
 * sunray系列无人机在当前位置起飞，使用轨迹控制命令move_trajectory，
 * 依次追踪预定义的四个矩形角点轨迹，最后返航
 * [补充说明]:
 * 如需修改起飞降落的相关参数，请修改control文件夹中
 * sunray_uav_control文件夹下config文件夹中的sunray_control_config.yaml中对应参数
 *
 */

// ros_msg_utils头文件，包含了大部分情况下需要的头文件
#include <ros_msg_utils.h>

// 定义一些全局变量，后续使用
std::string node_name;
std::string agent_name;
int agent_id;
std::string agent_ns;  // agent命名空间，使用agent_name与agent_id构造
// 其实这里我也想使用agent前缀，而不是uav前缀，但是考虑到uav和ugv没有统一，因此还是先使用了uav，看后续能不能尝试统一为agent
sunray_msgs::UAVControlFSMState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
nav_msgs::Odometry uav_odom;

// 全局使用的控制命令发布者
ros::Publisher control_cmd_pub;

// 多个目标点,这里是矩形的四个角点
Eigen::Vector3d Point_1(1, 1, 0.6);
Eigen::Vector3d Point_2(1, -1, 0.6);
Eigen::Vector3d Point_3(-1, -1, 0.6);
Eigen::Vector3d Point_4(-1, 1, 0.6);
// 目标点容器
std::vector<Eigen::Vector3d> Expect_Points;

constexpr double kLineSpeed = 0.3;  // 直线段期望速度
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

    const double duration = compute_line_duration(start_point, end_point, kLineSpeed);
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
        reference.desired_vel = direction * kLineSpeed;
    }
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

// 计算当前是否到达目标点
/* 参数要求为 1. 当前期望目标点
            2. 当前实际位置(里程计数据)
*/
Eigen::Vector3d arrived_error{Eigen::Vector3d(0.15, 0.15, 0.15)};
double keep_time{1.0};
ros::Time arrived_time{ros::Time(0)};

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
    std::cout << "sunray_uav_example [move_trajectory] node exit..." << std::endl;
    ros::shutdown();
    exit(EXIT_SUCCESS);  // 或者使用 exit(0)
}

// 无人机状态回调函数
void uav_state_callback(const sunray_msgs::UAVControlFSMState::ConstPtr& msg) {
    uav_state = *msg;
}

// 无人机里程计回调函数
void uav_odom_callback(const nav_msgs::OdometryConstPtr& msg) {
    uav_odom = *msg;
}

// 主函数
int main(int argc, char** argv) {
    // ros节点初始化
    ros::init(argc, argv, "move_trajectory_node");

    // 创建全局句柄与私有句柄
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 注册退出信号捕获函数
    signal(SIGINT, mySignalHandler);

    // 首先读取节点私有的uav_name与uav_id
    private_nh.getParam("agent_name", agent_name);
    if (agent_name.empty()) {
        // 如果agent_name为空，说明需要读取全局参数
        nh.getParam("agent_name", agent_name);
        nh.getParam("agent_id", agent_id);
    } else {
        // 如果从节点私有参数空间读取的agent_name不为空，则继续读取agent_id
        private_nh.getParam("agent_id", agent_id);
    }
    agent_ns = agent_name + std::to_string(agent_id);
    agent_ns = sunray_common::normalize_uav_ns(agent_ns);  // 标准化命名空间
    // 读取完成，开始初始化ros订阅者与发布者
    // [订阅] 无人机状态
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVControlFSMState>(
        agent_ns + "/sunray/fsm/state", 10, uav_state_callback);
    // [订阅] 无人机当前里程计
    ros::Subscriber uav_odom_sub = nh.subscribe<nav_msgs::Odometry>(
        agent_ns + "/sunray/localization/local_odom", 10, uav_odom_callback);
    // [发布] 无人机控制指令
    control_cmd_pub =
        nh.advertise<sunray_msgs::UAVControlCMD>(agent_ns + "/sunray/uav_control_cmd", 1);

    // 填充多航点容器
    Expect_Points.push_back(Point_1);
    Expect_Points.push_back(Point_2);
    Expect_Points.push_back(Point_3);
    Expect_Points.push_back(Point_4);

    // 进入主循环
    int times = 0;
    while (ros::ok() && (uav_state.sunray_fsm_state != sunray_msgs::UAVControlFSMState::FSM_INIT)) {
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
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::TAKEOFF;
    control_cmd_pub.publish(uav_cmd);
    // 判断是否完成起飞
    // 当uav_state从INIT变成HOVER时，认为起飞完成
    while (ros::ok() &&
           (uav_state.sunray_fsm_state != sunray_msgs::UAVControlFSMState::FSM_HOVER)) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            ROS_INFO("uav is takeoffing and wait for enter hover ");
    }
    // 清理循环变量
    times = 0;
    // 完成起飞后，开始轨迹追踪
    ROS_INFO("uav takeoff successfully and now move by trajectory");
    ROS_INFO("the first point is x: %f,y:%f,z:%f",
             Expect_Points[0].x(),
             Expect_Points[0].y(),
             Expect_Points[0].z());

    Eigen::Vector3d segment_start_point;
    segment_start_point.x() = uav_odom.pose.pose.position.x;
    segment_start_point.y() = uav_odom.pose.pose.position.y;
    segment_start_point.z() = uav_odom.pose.pose.position.z;

    for (std::size_t i = 0; ros::ok() && i < Expect_Points.size(); ++i) {
        const Eigen::Vector3d segment_end_point = Expect_Points[i];
        const ros::Time segment_start_time = ros::Time::now();
        const double segment_duration =
            compute_line_duration(segment_start_point, segment_end_point, kLineSpeed);
        bool segment_finished = false;
        arrived_time = ros::Time(0);

        while (ros::ok()) {
            const double elapsed_time = (ros::Time::now() - segment_start_time).toSec();
            if (!segment_finished) {
                const TrajectoryReference reference =
                    sample_line_reference(elapsed_time, segment_start_point, segment_end_point);
                publish_trajectory_command(reference);
                if (elapsed_time >= segment_duration) {
                    segment_finished = true;
                    arrived_time = ros::Time(0);
                    ROS_INFO("trajectory segment %u completed, now stabilize at target point",
                             static_cast<unsigned>(i + 1));
                }
            } else {
                publish_trajectory_command(make_hold_reference(segment_end_point));
                if (check_arrived(segment_end_point, uav_odom)) {
                    break;
                }
            }
            ros::spinOnce();
            ros::Duration(kControlDt).sleep();
        }

        if (!ros::ok()) {
            break;
        }

        segment_start_point = segment_end_point;
        if (i + 1 < Expect_Points.size()) {
            ROS_INFO("the next point is x: %f,y:%f,z:%f",
                     Expect_Points[i + 1].x(),
                     Expect_Points[i + 1].y(),
                     Expect_Points[i + 1].z());
        } else {
            ROS_INFO("move_trajectory end and now to return ");
        }
    }

    // 发布返航命令
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::RETURN;
    control_cmd_pub.publish(uav_cmd);
    // 一次publish需要ros的spin才能实现，所以这里需要进行一次spin，如果觉得这样不保险，可以选择while循环查询进入到RETURN后再退出
    ros::spinOnce();
    // RETURN可以在sunray_uav_config.yaml文件中被配置为自动降落或切入悬停，如果配置文件中设置为切入悬停，则会导致本节点会持续悬停

    // 直接结束节点或等待成功降落？
    ROS_INFO("uav is enter return mode and [move_trajectory] demo finished,quit !");
    return 0;
}
