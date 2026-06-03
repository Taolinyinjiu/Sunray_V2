#include "bridge_node.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "yunlink_ros_bridge_node");
    YunlinkRosBridgeNode node;
    ros::spin();
    return 0;
}
