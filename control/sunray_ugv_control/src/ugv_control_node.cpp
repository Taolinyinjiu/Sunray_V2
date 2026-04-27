#include "sunray_ugv_control/ugv_control_fsm.h"
#include <atomic>
#include <csignal>

namespace {

std::atomic<bool> stop_requested{false};

void sigint_handler(int) {
    stop_requested.store(true, std::memory_order_relaxed);
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "ugv_control_node");
    ros::NodeHandle nh;

    signal(SIGINT, sigint_handler);

    sunray_ugv_control::UGVControlFSM fsm(nh);
    fsm.init();

    // AsyncSpinner 保证话题回调持续响应，主线程则专注运行低频监督循环。
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate rate(fsm.get_update_frequency());
    while (ros::ok() && !stop_requested.load(std::memory_order_relaxed)) {
        fsm.process();
        rate.sleep();
    }

    spinner.stop();
    ros::shutdown();
    return 0;
}
