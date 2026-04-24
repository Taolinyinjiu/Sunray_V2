#include "planning_fsm.hpp"
#include "sunray_log.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "sunray_planning_node");
    ros::NodeHandle nh;

    try {
        PlanningFSM planning_fsm(nh);
        planning_fsm.init();
        ros::spin();
    } catch (const std::exception& e) {
        SUNRAY_CRITICAL("[sunray_planning] initialization failed: {}", e.what());
        return 1;
    }

    return 0;
}
