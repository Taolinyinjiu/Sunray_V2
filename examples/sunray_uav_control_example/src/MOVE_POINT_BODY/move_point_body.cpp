/**
 * @file move_point_body.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> move_point[body] -> return
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Sunray无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物，
 * 运行结果：
 * sunray系列无人机在当前位置起飞，使用机体系控制命令move_point_body移动到指定的目标点，然后返航降落
 * [补充说明]:
 * 如需修改起飞降落的相关参数，请修改control文件夹中
 * sunray_uav_control/config/sunray_control_base.yaml 以及
 * config/airframes/<airframe_type>.yaml 中对应参数
 *
 */

// ros_msg_utils头文件，包含了大部分情况下需要的头文件
#include <ros_msg_utils.h>

// 定义一些全局变量，后续使用
std::string node_name;
std::string agent_key;
sunray_msgs::UAVControlState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;

// 期望的目标点 [机体系]
#define Point_X 1.0
#define Point_Y 0.0
#define Point_Z 0.6

// 退出信号捕获函数
void mySignalHandler(int sig) {
    std::cout << "sunray_uav_example [move_point_body] node exit..." << std::endl;

    ros::shutdown();
    exit(EXIT_SUCCESS);  // 或者使用 exit(0)
}

// 无人机状态回调函数
void uav_state_callback(const sunray_msgs::UAVControlState::ConstPtr& msg) {
    uav_state = *msg;
}

// 主函数
int main(int argc, char** argv) {
    // ros节点初始化
    ros::init(argc, argv, "move_point_body_node");

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
    // [发布] 无人机控制指令
    ros::Publisher control_cmd_pub =
        nh.advertise<sunray_msgs::UAVControlCMD>(agent_key + "/sunray/uav_control/control_cmd", 1);
    // 进入主循环
    int times = 0;
    while (ros::ok() && (uav_state.control_state != sunray_msgs::UAVControlState::INIT)) {
        // 首先判断当前无人机控制状态
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            ROS_ERROR("uav control state has not entered INIT yet...");
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
    while (ros::ok() && (uav_state.control_state != sunray_msgs::UAVControlState::HOVER)) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            ROS_INFO("uav is taking off, waiting for HOVER...");
    }
    // 清理循环变量
    times = 0;
    // 完成起飞后，发布 MOVE_POINT_BODY 命令，移动到机体系相对目标点
    ROS_INFO("uav takeoff succeeded, moving to body-frame point (%f, %f, %f)",
             Point_X, Point_Y, Point_Z);
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_POINT_BODY;
    uav_cmd.desired_pos.x = Point_X;
    uav_cmd.desired_pos.y = Point_Y;
    uav_cmd.desired_pos.z = Point_Z;
    control_cmd_pub.publish(uav_cmd);
    // 首先判断是否会成功切换为 MOVE
    while (ros::ok() && (uav_state.control_state != sunray_msgs::UAVControlState::MOVE)) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        // 如果超过5s还没有进入 MOVE，可能是 system_check 模块认为当前无法运动，需要检查日志
        // 发布降落命令
        if (times++ > 5){
            ROS_ERROR("sunray control module failed to enter MOVE state, please check Sunray log");
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::LAND;
            control_cmd_pub.publish(uav_cmd);
            break;
        }
    }
    // 清理循环变量
    times = 0;
    // 判断是否到达目标点
    while (ros::ok() && (uav_state.control_state != sunray_msgs::UAVControlState::HOVER)) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            ROS_INFO("uav is moving, waiting for HOVER...");
    }
    // 清理循环变量
    times = 0;

    // 日志打印，到达目标点
    ROS_INFO("uav reached body-frame target point, now sending LAND");
    // 发布降落命令
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::LAND;
    control_cmd_pub.publish(uav_cmd);

    // 直接结束节点或等待成功降落？
    ROS_INFO("sent LAND command and [move_point_body] demo finished, quit!");
    return 0;
}
