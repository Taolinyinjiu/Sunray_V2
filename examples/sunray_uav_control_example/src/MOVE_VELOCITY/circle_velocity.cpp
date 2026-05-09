/**
 * @file circle_velocity.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> move circle by velocity -> return
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Surnay无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物，
 * 运行结果：
 * sunray系列无人机在当前位置起飞，使用惯性系速度控制命令move_velocity，
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
std::string agent_name;
int agent_id;
std::string agent_ns;  // agent命名空间，使用agent_name与agent_id构造
// 其实这里我也想使用agent前缀，而不是uav前缀，但是考虑到uav和ugv没有统一，因此还是先使用了uav，看后续能不能尝试统一为agent
sunray_msgs::UAVControlFSMState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;
nav_msgs::Odometry uav_odom;

// 全局使用的控制命令发布者
ros::Publisher control_cmd_pub;

// 圆形轨迹参数
Eigen::Vector3d circle_center = Eigen::Vector3d::Zero();
constexpr double kPi = 3.14159265358979323846;
constexpr double kCircleRadius = 1.0;                                // 半径
constexpr double kCircleAngularSpeed = 0.2;                         // 角速度
constexpr double kCircleDuration = 2.0 * kPi / kCircleAngularSpeed;  // 这段圆形轨迹总共花费的时间
constexpr double kControlDt = 0.02;                                  // 速度解算频率 20ms ~ 50Hz

// 反馈比例控制器 比例参数
float kp = 0.3;
// 限制最大和最小速度
float max_vel = 0.5;
float min_vel = -0.5;

// 圆形轨迹输出参考
struct CircleReference {
    Eigen::Vector3d desired_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_vel_ff{Eigen::Vector3d::Zero()};
};

// 控制器函数体实现
/* 参数要求为 1. 当前期望目标点
            2. 当前期望前馈速度
            3. 当前实际位置(里程计数据)
*/
void velocity_controller_update(const Eigen::Vector3d& desired_pos,
                                const Eigen::Vector3d& desired_vel_ff,
                                const nav_msgs::Odometry& odom) {
    // 1. 期望与实际做差，得到三轴误差，这里做差的顺序决定了kp的正负性
    // 我们用期望与实际做差，假设实际为(0,0)
    // 期望为(1,1)则做差结果为(1,1),按照惯性系的定义，此时无人机应该向左前飞，期望速度为正，因此kp也为正
    Eigen::Vector3d err_pos{Eigen::Vector3d::Zero()};
    // 由于是对圆形轨迹进行追踪，因此我们引入了速度作为前馈项
    Eigen::Vector3d velocity_cmd = desired_vel_ff;
    // 注意，单纯的比例反馈器实际上并不需要历史信息
    err_pos.x() = desired_pos.x() - odom.pose.pose.position.x;
    err_pos.y() = desired_pos.y() - odom.pose.pose.position.y;
    err_pos.z() = desired_pos.z() - odom.pose.pose.position.z;
    // 计算输出：前馈速度负责沿圆周运动，P项负责纠偏
    velocity_cmd.x() += err_pos.x() * kp;
    velocity_cmd.y() += err_pos.y() * kp;
    velocity_cmd.z() += err_pos.z() * kp;
    // 对输出进行限制幅度
    for (int i = 0; i < 3; ++i) {
        if (velocity_cmd[i] > max_vel)
            velocity_cmd[i] = max_vel;
        if (velocity_cmd[i] < min_vel)
            velocity_cmd[i] = min_vel;
    }

    // 发布命令
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_VELOCITY;
    uav_cmd.desired_vel.x = velocity_cmd.x();
    uav_cmd.desired_vel.y = velocity_cmd.y();
    uav_cmd.desired_vel.z = velocity_cmd.z();
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

// 圆形轨迹采样，输出圆上一点及对应切向前馈速度
CircleReference sample_circle_reference(double elapsed_time, const Eigen::Vector3d& circle_center) {
    const double angle = kCircleAngularSpeed * elapsed_time;
    CircleReference reference;
    reference.desired_pos = circle_center;
    reference.desired_pos.x() += kCircleRadius * std::cos(angle);
    reference.desired_pos.y() += kCircleRadius * std::sin(angle);
    reference.desired_vel_ff.x() = -kCircleRadius * kCircleAngularSpeed * std::sin(angle);
    reference.desired_vel_ff.y() = kCircleRadius * kCircleAngularSpeed * std::cos(angle);
    reference.desired_vel_ff.z() = 0.0;
    return reference;
}

// 退出信号捕获函数
void mySignalHandler(int sig) {
    std::cout << "sunray_uav_example [circle_velocity] node exit..." << std::endl;
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
    ros::init(argc, argv, "circle_velocity_node");

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
    // 完成起飞后，发布move_point命令，移动到目标点
    ROS_INFO("uav takeoff successfully and now move to defined points");
    // 设置当前位置为圆心
    circle_center.x() = uav_odom.pose.pose.position.x;
    circle_center.y() = uav_odom.pose.pose.position.y;
    circle_center.z() = uav_odom.pose.pose.position.z;

    // 向前移动到圆形上的一点，假设当前为(0 0 0.6)则移动到（r 0 0.6）并从此处开始进行一次绕圆运动
    Eigen::Vector3d temp_aim;
    temp_aim.x() = uav_odom.pose.pose.position.x + kCircleRadius;
    temp_aim.y() = uav_odom.pose.pose.position.y;
    temp_aim.z() = uav_odom.pose.pose.position.z;

    // 让无人机运动到前方圆半径处
    while (ros::ok()) {
        // 内部分为两部分，一部分为速度控制器更新，计算输出，另一部分为计算当前是否到达航点
        // 1. 速度控制器更新
        velocity_controller_update(temp_aim, Eigen::Vector3d::Zero(), uav_odom);
        // 2. 检测当前是否达到目标点
        if (check_arrived(temp_aim, uav_odom)) {
            break;
        }
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }

    // 此处进入速度控制追踪圆形轨迹，控制频率设定为20ms一次，也就是50Hz
    bool circle_finished = false;
    // 圆形轨迹上开始运动的点
    Eigen::Vector3d start_point;
    start_point.x() = uav_odom.pose.pose.position.x;
    start_point.y() = uav_odom.pose.pose.position.y;
    start_point.z() = uav_odom.pose.pose.position.z;
    // 轨迹开始生成的时间
    const ros::Time trajectory_start_time = ros::Time::now();

    while (ros::ok()) {
        // 得到轨迹进度时间，用当前时间戳减去开始时间戳
        const double elapsed_time = (ros::Time::now() - trajectory_start_time).toSec();
        // 如果圆形轨迹还未结束
        if (!circle_finished) {
            // 使用采样函数，生成圆形轨迹上的期望位置与切向前馈速度
            const CircleReference reference = sample_circle_reference(elapsed_time, circle_center);
            // 速度控制器进行追踪
            velocity_controller_update(reference.desired_pos, reference.desired_vel_ff, uav_odom);
            // 如果轨迹进度时间大于了计算的时间
            if (elapsed_time >= kCircleDuration) {
                // 认为轨迹生成结束，追踪最后一个目标点
                circle_finished = true;
                arrived_time = ros::Time(0);
                ROS_INFO("circle trajectory completed, now stabilize at start point");
            }
        } else {
            // 如果圆形轨迹完成了，应当移动到开始运动的点，然后再进行return
            velocity_controller_update(start_point, Eigen::Vector3d::Zero(), uav_odom);
            if (check_arrived(start_point, uav_odom)) {
                ROS_INFO("circle example finished and now to return ");
                break;
            }
        }
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    // 发布返航命令
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::RETURN;
    control_cmd_pub.publish(uav_cmd);
    // 一次publish需要ros的spin才能实现，所以这里需要进行一次spin，如果觉得这样不保险，可以选择while循环查询进入到RETURN后再退出
    ros::spinOnce();
    // RETURN可以在sunray_uav_config.yaml文件中被配置为自动降落或切入悬停，如果配置文件中设置为切入悬停，则会导致本节点会持续悬停

    // 直接结束节点或等待成功降落？
    ROS_INFO("uav is enter return mode and [circle_velocity] demo finished,quit !");
    return 0;
}
