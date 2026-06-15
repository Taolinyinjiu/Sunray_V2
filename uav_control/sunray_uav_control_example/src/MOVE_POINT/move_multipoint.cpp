/**
 * @file move_multipoint.cpp
 * @brief Sunray单个无人机示例系列 - takeoff -> hover -> move k point -> land
 * 运行要求：
 * [仿真环境]：要求Gazebo仿真环境只存在一台Sunray无人机
 * [真实环境]：要求无人机周围无阻拦运动的障碍物
 * 运行结果：
 * sunray系列无人机在当前位置起飞，达到指定高度后悬停5s，然后移动到指定位置悬停5s，随后移动到下一个指定位置悬停5s，依次类推，最后降落在第k个位置的下方
 *
 * 教学重点：
 * 演示如何按顺序发送多个 MOVE_POINT 命令；每个位置点命令只需发布一次，
 * 到达并重新进入 HOVER 后再发送下一个目标点。
 */
// ros_msg_utils头文件，包含了大部分情况下需要的头文件
#include <ros_msg_utils.h>

// 定义一些全局变量，后续使用
std::string node_name;
std::string agent_key;
sunray_msgs::UAVControlState uav_state;
sunray_msgs::UAVControlCMD uav_cmd;

// 多个目标点
Eigen::Vector3d Point_1(1, 1, 0.6);
Eigen::Vector3d Point_2(1, -1, 0.6);
Eigen::Vector3d Point_3(-1, -1, 0.6);
Eigen::Vector3d Point_4(-1, 1, 0.6);
// 目标点容器
std::vector<Eigen::Vector3d> Expect_Points;

// 退出信号捕获函数
void mySignalHandler(int sig) {
    std::cout << "sunray_uav_example [move_multipoint] node exit..." << std::endl;
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
    ros::init(argc, argv, "move_multipoint_node");

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
    // 填充多航点容器
    Expect_Points.push_back(Point_1);
    Expect_Points.push_back(Point_2);
    Expect_Points.push_back(Point_3);
    Expect_Points.push_back(Point_4);
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
    while (ros::ok() &&
           (uav_state.control_state != sunray_msgs::UAVControlState::HOVER)) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            ROS_INFO("uav is taking off, waiting for HOVER...");
    }
    // 清理循环变量
    times = 0;
    // 完成起飞后，按顺序发布 MOVE_POINT 命令
    ROS_INFO(
        "uav takeoff succeeded, moving to defined points");

    for (int i = 0; i < Expect_Points.size(); i++) {
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_POINT;
        uav_cmd.desired_pos.x = Expect_Points[i].x();
        uav_cmd.desired_pos.y = Expect_Points[i].y();
        uav_cmd.desired_pos.z = Expect_Points[i].z();
        control_cmd_pub.publish(uav_cmd);
        ROS_INFO("uav will move to point x: %f, y: %f, z: %f",
                 Expect_Points[i].x(), Expect_Points[i].y(), Expect_Points[i].z());
        // 首先判断是否会成功切换为 MOVE
        while (ros::ok() &&
               (uav_state.control_state != sunray_msgs::UAVControlState::MOVE)) {
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            // 如果超过5s还没有进入 MOVE，可能是 system_check 模块认为当前无法运动，需要检查日志
            // 发布降落命令
            if (times++ > 5) {
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
        while (ros::ok() &&
               (uav_state.control_state != sunray_msgs::UAVControlState::HOVER)) {
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            if (times++ > 5)
                ROS_INFO("uav is moving, waiting for HOVER...");
        }
        // 清理循环变量
        times = 0;

        // 日志打印，到达目标点
        ROS_INFO("uav reached current target point");
    }
    // 发布返航命令
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.control_cmd = sunray_msgs::UAVControlCMD::RETURN;
    control_cmd_pub.publish(uav_cmd);
    // 一次publish需要ros的spin才能实现，所以这里需要进行一次spin，如果觉得这样不保险，可以选择while循环查询进入到RETURN后再退出
    ros::spinOnce();
    // RETURN 可以通过 sunray_uav_control/config/sunray_control_base.yaml 中 basic_param.return_with_land
    // 配置为自动降落或切入悬停，config/airframes/<airframe_type>.yaml 也可以覆盖该参数

    // 直接结束节点或等待成功降落？
    ROS_INFO("sent RETURN command and [move_multipoint] demo finished, quit!");
    return 0;
}
