#include "bridge_node.hpp"

#include <cmath>

void YunlinkRosBridgeNode::onTakeoffCommand(
    const yunlink::InboundCommandView<yunlink::TakeoffCommand>& view) {
    sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(sunray_msgs::UAVControlCMD::TAKEOFF);
    publishControlCmd("takeoff",
                      cmd,
                      view.inbound.envelope.session_id,
                      view.inbound.envelope.correlation_id,
                      view.inbound.envelope.message_id,
                      "relative_height_m=" + std::to_string(view.payload.relative_height_m) +
                          " max_velocity_mps=" + std::to_string(view.payload.max_velocity_mps));
}

void YunlinkRosBridgeNode::onLandCommand(
    const yunlink::InboundCommandView<yunlink::LandCommand>& view) {
    sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(sunray_msgs::UAVControlCMD::LAND);
    publishControlCmd("land",
                      cmd,
                      view.inbound.envelope.session_id,
                      view.inbound.envelope.correlation_id,
                      view.inbound.envelope.message_id,
                      "max_velocity_mps=" + std::to_string(view.payload.max_velocity_mps));
}

void YunlinkRosBridgeNode::onReturnCommand(
    const yunlink::InboundCommandView<yunlink::ReturnCommand>& view) {
    sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(sunray_msgs::UAVControlCMD::RETURN);
    publishControlCmd("return",
                      cmd,
                      view.inbound.envelope.session_id,
                      view.inbound.envelope.correlation_id,
                      view.inbound.envelope.message_id,
                      "loiter_before_return_s=" + std::to_string(view.payload.loiter_before_return_s));
}

void YunlinkRosBridgeNode::onGotoCommand(
    const yunlink::InboundCommandView<yunlink::GotoCommand>& view) {
    sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(sunray_msgs::UAVControlCMD::MOVE_POINT);
    cmd.desired_pos.x = view.payload.x_m;
    cmd.desired_pos.y = view.payload.y_m;
    cmd.desired_pos.z = view.payload.z_m;
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    cmd.desired_yaw = view.payload.yaw_rad;
    publishControlCmd("goto",
                      cmd,
                      view.inbound.envelope.session_id,
                      view.inbound.envelope.correlation_id,
                      view.inbound.envelope.message_id,
                      "x=" + std::to_string(view.payload.x_m) +
                          " y=" + std::to_string(view.payload.y_m) +
                          " z=" + std::to_string(view.payload.z_m) +
                          " yaw=" + std::to_string(view.payload.yaw_rad));
}

void YunlinkRosBridgeNode::onVelocitySetpointCommand(
    const yunlink::InboundCommandView<yunlink::VelocitySetpointCommand>& view) {
    sunray_msgs::UAVControlCMD cmd = makeBaseControlCmd(
        view.payload.body_frame ? sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY
                                : sunray_msgs::UAVControlCMD::MOVE_VELOCITY);
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAWRATE;
    cmd.desired_yaw_rate = view.payload.yaw_rate_radps;

    std::string detail = "body_frame=" + std::string(view.payload.body_frame ? "true" : "false") +
                         " vx=" + std::to_string(view.payload.vx_mps) +
                         " vy=" + std::to_string(view.payload.vy_mps) +
                         " vz=" + std::to_string(view.payload.vz_mps) +
                         " yaw_rate=" + std::to_string(view.payload.yaw_rate_radps);

    if (view.payload.body_frame) {
        bool has_height = false;
        cmd.desired_body_xy_vel.x = view.payload.vx_mps;
        cmd.desired_body_xy_vel.y = view.payload.vy_mps;
        cmd.fixed_height = latestPx4Height(&has_height);
        if (!has_height) {
            ROS_WARN_THROTTLE(5.0,
                              "yunlink_ros_bridge has not received px4_state yet, body velocity fallback fixed_height=0");
        }
        if (std::fabs(view.payload.vz_mps) > 1e-4F) {
            ROS_WARN_THROTTLE(5.0,
                              "yunlink_ros_bridge body velocity ignores vz_mps=%f and keeps fixed_height=%f",
                              view.payload.vz_mps,
                              cmd.fixed_height);
        }
        detail += " fixed_height=" + std::to_string(cmd.fixed_height);
    } else {
        cmd.desired_vel.x = view.payload.vx_mps;
        cmd.desired_vel.y = view.payload.vy_mps;
        cmd.desired_vel.z = view.payload.vz_mps;
    }

    publishControlCmd("velocity_setpoint",
                      cmd,
                      view.inbound.envelope.session_id,
                      view.inbound.envelope.correlation_id,
                      view.inbound.envelope.message_id,
                      detail);
}
