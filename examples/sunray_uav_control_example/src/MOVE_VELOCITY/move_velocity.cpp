/**
 * @file move_velocity.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> move_velocity -> return
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Surnay无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物，
 * 运行结果：
 * sunray系列无人机在当前位置起飞，使用惯性系速度控制命令move_velocity，移动到预定义的四个矩形角点，最后返航
 * 设计一个简单地反馈比例控制器，只有一个参数Kp，输入为当前位置和期望位置，输出为速度控制命令
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

// 反馈比例控制器 比例参数
float kp = 0.3;
// 限制最大和最小速度
float max_vel = 0.5;
float min_vel = -0.5;

// 控制器函数体实现
/* 参数要求为 1. 当前期望目标点
            2.  当前实际位置(里程计数据)
*/
void velocity_controller_update(Eigen::Vector3d desired_pos, const nav_msgs::Odometry& odom) {
    // 1. 期望与实际做差，得到三轴误差，这里做差的顺序决定了kp的zhegnfu
    // 我们用期望与实际做差，假设实际为(0,0)
    // 期望为(1,1)则做差结果为(1,1),按照惯性系的定义，此时无人机应该向左前飞，期望速度为正，因此kp也为正
    Eigen::Vector3d err_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_cmd{Eigen::Vector3d::Zero()};
    // 注意，单纯的比例反馈器实际上并不需要历史信息
    err_pos.x() = desired_pos.x() - odom.pose.pose.position.x;
    err_pos.y() = desired_pos.y() - odom.pose.pose.position.y;
    err_pos.z() = desired_pos.z() - odom.pose.pose.position.z;
    // 计算输出
    velocity_cmd.x() = err_pos.x() * kp;
    velocity_cmd.y() = err_pos.y() * kp;
    velocity_cmd.z() = err_pos.z() * kp;
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
// 计算当前是否到达目标点
/* 参数要求为 1. 当前期望目标点
            2.  当前实际位置(里程计数据)
*/
Eigen::Vector3d arrived_error{
    Eigen::Vector3d(0.15, 0.15, 0.15)};  // 定义三轴误差小于0.15时，认为到达目标点
double keep_time{1.0};                   // 在目标点应保持稳定1s认为到达
ros::Time arrived_time{ros::Time(0)};
bool check_arrived(Eigen::Vector3d desired_pos, const nav_msgs::Odometry& odom) {
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
    std::cout << "sunray_uav_example [move_velocity] node exit..." << std::endl;
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
    ros::init(argc, argv, "move_velocity_node");

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
    // 完成起飞后，发布move_point命令，移动到目标点
    ROS_INFO("uav takeoff successfully and now move to defined points");
    ROS_INFO("the first point is x: %f,y:%f,z:%f",
             Expect_Points[0].x(),
             Expect_Points[0].y(),
             Expect_Points[0].z());
    // 此处进入速度控制模式，控制频率设定为20ms一次，也就是50Hz
    while (ros::ok()) {
        // 进入航点循环，循环次数设定为航点容器的大小
        for (uint i = 0; i < Expect_Points.size();) {
            // 内部分为两部分，一部分为速度控制器更新，计算输出，另一部分为计算当前是否到达航点
            // 1. 速度控制器更新
            velocity_controller_update(Expect_Points[i], uav_odom);
            // 2. 检测当前是否达到目标点
            if (check_arrived(Expect_Points[i], uav_odom)) {
                i++;
                if(i < Expect_Points.size()){
                ROS_INFO("the next point is x: %f,y:%f,z:%f",
                         Expect_Points[i].x(),
                         Expect_Points[i].y(),
                         Expect_Points[i].z());
                }else{
                    ROS_INFO("move_velocity end and now to return ");
                }
            }
            ros::spinOnce();
            ros::Duration(0.02).sleep();
        }
        break;
    }

    // 发布返航命令
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::RETURN;
    control_cmd_pub.publish(uav_cmd);
    // 一次publish需要ros的spin才能实现，所以这里需要进行一次spin，如果觉得这样不保险，可以选择while循环查询进入到RETURN后再退出
    ros::spinOnce();
    // RETURN可以在sunray_uav_config.yaml文件中被配置为自动降落或切入悬停，如果配置文件中设置为切入悬停，则会导致本节点会持续悬停

    // 直接结束节点或等待成功降落？
    ROS_INFO("uav is enter land mode and [move_velocity] demo finished,quit !");
    return 0;
}
