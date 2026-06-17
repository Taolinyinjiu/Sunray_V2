/** @file @brief yunlink_ros_bridge_tui_node 入口。 */
#include "bridge_tui_node.hpp"

/// 初始化 TUI 节点并进入 ROS spin。
int main(int argc, char** argv) {
    ros::init(argc, argv, "yunlink_ros_bridge_tui_node");
    BridgeTuiNode node;
    ros::spin();
    return 0;
}
