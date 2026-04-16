#include "statemachine/sunray_fsm.hpp"
#include <atomic>
#include <csignal>  // 捕获中断信号头文件
// 这里有一个全局变量，用来指示节点的while循环什么时候停止
std::atomic<bool> stop_request{false};

// 捕获中断信号
void mySigintHandler(int /*sig*/) {
    stop_request.store(true, std::memory_order_relaxed);
}

// main函数
int main(int argc, char** argv) {
    // 设置日志
    // ...

    // 初始化ros节点
    ros::init(argc, argv, "uav_control_node");
    // 获取全局节点句柄
    ros::NodeHandle nh;

    // 设置中断信号处理函数
    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    // 实例化Sunray_FSN
    Sunray_FSM sunray_fsm(nh);
    // 初始化FSM
    sunray_fsm.init();

    // 回调独立线程：里程计/控制指令等可按到达频率处理（例如200Hz）
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 获取状态机更新频率
    double update_hz = sunray_fsm.get_update_frequency();
    if (update_hz <= 0.0) {
        ROS_FATAL("invalid supervisor update frequency: %.3f Hz", update_hz);
        ros::shutdown();
        return 1;
    }
    ros::Rate rate(update_hz);
    while (ros::ok() && !stop_request) {
        // 状态机更新
        sunray_fsm.process();
        // 日志打印？
        rate.sleep();
    }
    // 打印退出日志
    ROS_INFO("uav control node shutdown now");
    spinner.stop();
    ros::shutdown();
    return 0;
}
