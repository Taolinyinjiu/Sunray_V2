#include "bridge_node.hpp"

#include <cmath>

#include "bridge_mapping.hpp"

void YunlinkRosBridgeNode::onTakeoffCommand(
    const yunlink::InboundCommandView<yunlink::TakeoffCommand>& view) {
    const auto command_msg = mapTakeoffCommand(view, ros::Time::now());
    sunray_msgs::UAVControlCMD cmd = mapControlCmd(command_msg);
    publishControlCmd("takeoff",
                      cmd,
                      command_msg.meta,
                      "relative_height_m=" + std::to_string(command_msg.relative_height_m) +
                          " max_velocity_mps=" + std::to_string(command_msg.max_velocity_mps));
}

void YunlinkRosBridgeNode::onLandCommand(
    const yunlink::InboundCommandView<yunlink::LandCommand>& view) {
    const auto command_msg = mapLandCommand(view, ros::Time::now());
    sunray_msgs::UAVControlCMD cmd = mapControlCmd(command_msg);
    publishControlCmd("land",
                      cmd,
                      command_msg.meta,
                      "max_velocity_mps=" + std::to_string(command_msg.max_velocity_mps));
}

void YunlinkRosBridgeNode::onReturnCommand(
    const yunlink::InboundCommandView<yunlink::ReturnCommand>& view) {
    const auto command_msg = mapReturnCommand(view, ros::Time::now());
    sunray_msgs::UAVControlCMD cmd = mapControlCmd(command_msg);
    publishControlCmd("return",
                      cmd,
                      command_msg.meta,
                      "loiter_before_return_s=" + std::to_string(command_msg.loiter_before_return_s));
}

void YunlinkRosBridgeNode::onGotoCommand(
    const yunlink::InboundCommandView<yunlink::GotoCommand>& view) {
    const auto command_msg = mapGotoCommand(view, ros::Time::now());
    sunray_msgs::UAVControlCMD cmd = mapControlCmd(command_msg);
    publishControlCmd("goto",
                      cmd,
                      command_msg.meta,
                      "x=" + std::to_string(command_msg.x_m) +
                          " y=" + std::to_string(command_msg.y_m) +
                          " z=" + std::to_string(command_msg.z_m) +
                          " yaw=" + std::to_string(command_msg.yaw_rad));
}

void YunlinkRosBridgeNode::onVelocitySetpointCommand(
    const yunlink::InboundCommandView<yunlink::VelocitySetpointCommand>& view) {
    const auto command_msg = mapVelocitySetpointCommand(view, ros::Time::now());

    std::string detail = "body_frame=" + std::string(command_msg.body_frame ? "true" : "false") +
                         " vx=" + std::to_string(command_msg.vx_mps) +
                         " vy=" + std::to_string(command_msg.vy_mps) +
                         " vz=" + std::to_string(command_msg.vz_mps) +
                         " yaw_rate=" + std::to_string(command_msg.yaw_rate_radps);

    float fixed_height = 0.0F;
    if (view.payload.body_frame) {
        bool has_height = false;
        fixed_height = latestPx4Height(&has_height);
        if (!has_height) {
            ROS_WARN_THROTTLE(5.0,
                              "yunlink_ros_bridge has not received px4_state yet, body velocity fallback fixed_height=0");
        }
        if (std::fabs(command_msg.vz_mps) > 1e-4F) {
            ROS_WARN_THROTTLE(5.0,
                              "yunlink_ros_bridge body velocity ignores vz_mps=%f and keeps fixed_height=%f",
                              command_msg.vz_mps,
                              fixed_height);
        }
        detail += " fixed_height=" + std::to_string(fixed_height);
    }
    sunray_msgs::UAVControlCMD cmd = mapControlCmd(command_msg, fixed_height);

    publishControlCmd("velocity_setpoint",
                      cmd,
                      command_msg.meta,
                      detail);
}
