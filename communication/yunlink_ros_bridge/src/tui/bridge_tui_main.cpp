#include "bridge_tui_node.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "yunlink_ros_bridge_tui_node");
    BridgeTuiNode node;
    ros::spin();
    return 0;
}
