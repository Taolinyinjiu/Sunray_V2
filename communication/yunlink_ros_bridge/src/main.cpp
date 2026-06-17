/** @file @brief yunlink_ros_bridge_node 入口。 */
#include "bridge_node.hpp"

/// 初始化 bridge 节点并进入 ROS spin。
int main(int argc, char** argv) {
    ros::init(argc, argv, "yunlink_ros_bridge_node");
    YunlinkRosBridgeNode node;
    ros::spin();
    return 0;
}
