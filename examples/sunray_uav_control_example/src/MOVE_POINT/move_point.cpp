/**
 * @file move_point.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> hover -> land
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Surnay无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物
 * [启动顺序]：为了保持example示例的简洁，我们并没有完整的启动流程保护措施，
 * 运行结果：
 * sunray系列无人机在当前位置起飞，达到指定高度后悬停10s，然后降落
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

// 期望的目标点
#define Point_X 1.0
#define Point_Y 0.0
#define Point_Z 0.6

// 退出信号捕获函数
void mySignalHandler(int sig) {
    std::cout << "sunray_uav_example [move_point] node exit..." << std::endl;

    ros::shutdown();
    exit(EXIT_SUCCESS);  // 或者使用 exit(0)
}

// 无人机状态回调函数
void uav_state_callback(const sunray_msgs::UAVControlFSMState::ConstPtr& msg) {
    uav_state = *msg;
}

// 主函数
int main(int argc, char** argv) {
    // ros节点初始化
    ros::init(argc, argv, "move_point_node");

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
    // [发布] 无人机控制指令
    ros::Publisher control_cmd_pub =
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
    while (ros::ok() && (uav_state.sunray_fsm_state != sunray_msgs::UAVControlFSMState::FSM_HOVER)) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            ROS_INFO("uav is takeoffing and wait for enter hover ");
    }
    // 清理循环变量
    times = 0;
    // 完成起飞后，发布move_point命令，移动到目标点
    ROS_INFO("uav takeoff successfully and now move to Point(%f,%f,%f) " ,Point_X,Point_Y,Point_Z);
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_POINT;
    uav_cmd.desired_pos.x = Point_X;
    uav_cmd.desired_pos.y = Point_Y;
    uav_cmd.desired_pos.z = Point_Z;
    control_cmd_pub.publish(uav_cmd);
    // 首先判断是否会成功切换为move_point
    while (ros::ok() && (uav_state.sunray_fsm_state != sunray_msgs::UAVControlFSMState::FSM_MOVE)) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        // 如果超过5s还没有进入到move模式，那么我们认为可能是system_check模块认为当前无法进行运动，需要检查日志
        // 发布降落命令
        if (times++ > 5){
            ROS_ERROR("Error : sunray control module can't checkout move state,please check sunray log");
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::LAND;
            control_cmd_pub.publish(uav_cmd);
            break;
        }
    }
    // 清理循环变量
    times = 0;
    // 判断是否到达目标点
    while (ros::ok() && (uav_state.sunray_fsm_state != sunray_msgs::UAVControlFSMState::FSM_HOVER)) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            ROS_INFO("uav is moving and wait for enter hover ");
    }
    // 清理循环变量
    times = 0;

    // 日志打印，到达目标点
    ROS_INFO("uav is moved point successfully ,and then enter land state");
    // 发布降落命令
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::LAND;
    control_cmd_pub.publish(uav_cmd);

    // 直接结束节点或等待成功降落？
    ROS_INFO("uav is enter land mode and [move_point] demo finished,quit !");
    return 0;
}
