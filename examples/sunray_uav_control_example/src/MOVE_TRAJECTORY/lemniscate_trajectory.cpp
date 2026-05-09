/**
 * @file lemniscate_trajectory.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> move lemniscate by trajectory -> return
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Surnay无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物，
 * 运行结果：
 * sunray系列无人机在当前位置起飞，使用轨迹控制命令move_trajectory，
 * 沿局部坐标系8字轨迹飞行一圈，最后返航
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

// 双纽线（lemniscate）轨迹参数
Eigen::Vector3d lemniscate_center = Eigen::Vector3d::Zero();
constexpr double kPi = 3.14159265358979323846;
constexpr double kLemniscateXSpan = 1;               // x方向峰峰值
constexpr double kLemniscateYSpan = 2;               // y方向峰峰值
constexpr double kLemniscateSpeed = 0.2;             // 沿轨迹的期望速度
constexpr double kLemniscatePhaseEnd = 2.0 * kPi;    // 轨迹闭合一圈的相位
constexpr double kLemniscateStartPhase = 1.5 * kPi;  // 从中心交叉点出发
constexpr double kLemniscatePhaseDirection = -1.0;   // 默认先朝右下分支运动
constexpr double kControlDt = 0.02;

struct LemniscateReference {
    Eigen::Vector3d desired_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_vel{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_acc{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_jerk{Eigen::Vector3d::Zero()};
    double phase_rate{0.0};
};

struct LemniscateKinematics {
    Eigen::Vector3d desired_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_vel{Eigen::Vector3d::Zero()};
    double phase_rate{0.0};
};

void publish_trajectory_command(const Eigen::Vector3d& desired_pos,
                                const Eigen::Vector3d& desired_vel,
                                const Eigen::Vector3d& desired_acc,
                                const Eigen::Vector3d& desired_jerk) {
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY;
    uav_cmd.desired_pos.x = desired_pos.x();
    uav_cmd.desired_pos.y = desired_pos.y();
    uav_cmd.desired_pos.z = desired_pos.z();
    uav_cmd.desired_vel.x = desired_vel.x();
    uav_cmd.desired_vel.y = desired_vel.y();
    uav_cmd.desired_vel.z = desired_vel.z();
    uav_cmd.desired_acc.x = desired_acc.x();
    uav_cmd.desired_acc.y = desired_acc.y();
    uav_cmd.desired_acc.z = desired_acc.z();
    uav_cmd.desired_jerk.x = desired_jerk.x();
    uav_cmd.desired_jerk.y = desired_jerk.y();
    uav_cmd.desired_jerk.z = desired_jerk.z();
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

LemniscateKinematics sample_lemniscate_kinematics(double phase,
                                                  const Eigen::Vector3d& center_point,
                                                  const double phase_direction) {
    const double sin_phase = std::sin(phase);
    const double cos_phase = std::cos(phase);
    const double denom = 1.0 + sin_phase * sin_phase;
    const double denom_sq = denom * denom;

    const double x_scale = std::sqrt(2.0) * kLemniscateXSpan;
    const double y_scale = 0.5 * kLemniscateYSpan;
    const double x_rel = x_scale * sin_phase * cos_phase / denom;
    const double y_rel = y_scale * cos_phase / denom;

    const double dx_dphase =
        x_scale *
        (((cos_phase * cos_phase - sin_phase * sin_phase) * denom) -
         ((sin_phase * cos_phase) * (2.0 * sin_phase * cos_phase))) /
        denom_sq;
    const double dy_dphase =
        y_scale * (((-sin_phase) * denom) - (cos_phase * (2.0 * sin_phase * cos_phase))) /
        denom_sq;

    LemniscateKinematics sample;
    sample.desired_pos = center_point;
    sample.desired_pos.x() += x_rel;
    sample.desired_pos.y() += y_rel;

    const double tangent_norm = std::hypot(dx_dphase, dy_dphase);
    if (tangent_norm > 1e-6) {
        sample.phase_rate = kLemniscateSpeed / tangent_norm;
    } else {
        sample.phase_rate = 0.0;
    }

    const double signed_phase_rate = phase_direction * sample.phase_rate;
    sample.desired_vel.x() = dx_dphase * signed_phase_rate;
    sample.desired_vel.y() = dy_dphase * signed_phase_rate;
    return sample;
}

LemniscateReference sample_lemniscate_reference(double phase,
                                                const Eigen::Vector3d& center_point) {
    const LemniscateKinematics current =
        sample_lemniscate_kinematics(phase, center_point, kLemniscatePhaseDirection);
    const double phase_step = kLemniscatePhaseDirection * current.phase_rate * kControlDt;
    const LemniscateKinematics next =
        sample_lemniscate_kinematics(phase + phase_step, center_point, kLemniscatePhaseDirection);
    const LemniscateKinematics prev =
        sample_lemniscate_kinematics(phase - phase_step, center_point, kLemniscatePhaseDirection);

    LemniscateReference reference;
    reference.desired_pos = current.desired_pos;
    reference.desired_vel = current.desired_vel;
    reference.desired_acc = (next.desired_vel - prev.desired_vel) / (2.0 * kControlDt);
    reference.desired_jerk =
        (next.desired_vel - 2.0 * current.desired_vel + prev.desired_vel) /
        (kControlDt * kControlDt);
    reference.phase_rate = current.phase_rate;
    return reference;
}

// 退出信号捕获函数
void mySignalHandler(int sig) {
    std::cout << "sunray_uav_example [lemniscate_trajectory] node exit..." << std::endl;
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
    ros::init(argc, argv, "lemniscate_trajectory_node");

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
    // 完成起飞后，直接从当前位置作为双纽线中心开始轨迹追踪
    ROS_INFO("uav takeoff successfully and now track lemniscate trajectory");
    lemniscate_center.x() = uav_odom.pose.pose.position.x;
    lemniscate_center.y() = uav_odom.pose.pose.position.y;
    lemniscate_center.z() = uav_odom.pose.pose.position.z;

    bool lemniscate_finished = false;
    const Eigen::Vector3d center_point = lemniscate_center;
    double lemniscate_phase = kLemniscateStartPhase;
    double lemniscate_phase_travel = 0.0;
    arrived_time = ros::Time(0);

    while (ros::ok()) {
        if (!lemniscate_finished) {
            const LemniscateReference reference =
                sample_lemniscate_reference(lemniscate_phase, lemniscate_center);
            publish_trajectory_command(reference.desired_pos,
                                       reference.desired_vel,
                                       reference.desired_acc,
                                       reference.desired_jerk);
            lemniscate_phase += kLemniscatePhaseDirection * reference.phase_rate * kControlDt;
            lemniscate_phase_travel += reference.phase_rate * kControlDt;
            if (lemniscate_phase_travel >= kLemniscatePhaseEnd) {
                lemniscate_finished = true;
                arrived_time = ros::Time(0);
                ROS_INFO("lemniscate trajectory completed, now stabilize at center point");
            }
        } else {
            publish_trajectory_command(center_point,
                                       Eigen::Vector3d::Zero(),
                                       Eigen::Vector3d::Zero(),
                                       Eigen::Vector3d::Zero());
            if (check_arrived(center_point, uav_odom)) {
                ROS_INFO("lemniscate trajectory example finished and now to return ");
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
    ROS_INFO("uav is enter return mode and [lemniscate_trajectory] demo finished,quit !");
    return 0;
}
