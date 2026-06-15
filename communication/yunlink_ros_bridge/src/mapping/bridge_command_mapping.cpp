#include "bridge_mapping.hpp"

namespace {

std_msgs::Header makeHeader(const ros::Time& stamp) {
    std_msgs::Header header;
    header.stamp = stamp;
    return header;
}

sunray_msgs::UAVControlCMD makeControlCmd(const std_msgs::Header& header, uint8_t control_cmd) {
    sunray_msgs::UAVControlCMD out;
    out.header = header;
    out.cmd_source = sunray_msgs::UAVControlCMD::SUNRAY_STATION;
    out.control_cmd = control_cmd;
    return out;
}

}  // namespace

yunlink_msgs::CommandMeta
mapCommandMeta(uint64_t session_id, uint64_t message_id, uint64_t correlation_id) {
    yunlink_msgs::CommandMeta out{};
    out.session_id = session_id;
    out.message_id = message_id;
    out.correlation_id = correlation_id;
    return out;
}

sunray_msgs::UAVControlCMD applyTrackingToken(const sunray_msgs::UAVControlCMD& msg,
                                              uint64_t tracking_token) {
    sunray_msgs::UAVControlCMD out = msg;
    out.tracking_token = tracking_token;
    return out;
}

yunlink_msgs::TakeoffCommand
mapTakeoffCommand(const yunlink::InboundCommandView<yunlink::TakeoffCommand>& view,
                  const ros::Time& stamp) {
    yunlink_msgs::TakeoffCommand out;
    out.header = makeHeader(stamp);
    out.meta = mapCommandMeta(view.inbound.envelope.session_id,
                              view.inbound.envelope.message_id,
                              view.inbound.envelope.correlation_id);
    out.relative_height_m = view.payload.relative_height_m;
    out.max_velocity_mps = view.payload.max_velocity_mps;
    return out;
}

yunlink_msgs::LandCommand mapLandCommand(const yunlink::InboundCommandView<yunlink::LandCommand>& view,
                                         const ros::Time& stamp) {
    yunlink_msgs::LandCommand out;
    out.header = makeHeader(stamp);
    out.meta = mapCommandMeta(view.inbound.envelope.session_id,
                              view.inbound.envelope.message_id,
                              view.inbound.envelope.correlation_id);
    out.max_velocity_mps = view.payload.max_velocity_mps;
    return out;
}

yunlink_msgs::ReturnCommand
mapReturnCommand(const yunlink::InboundCommandView<yunlink::ReturnCommand>& view,
                 const ros::Time& stamp) {
    yunlink_msgs::ReturnCommand out;
    out.header = makeHeader(stamp);
    out.meta = mapCommandMeta(view.inbound.envelope.session_id,
                              view.inbound.envelope.message_id,
                              view.inbound.envelope.correlation_id);
    out.loiter_before_return_s = view.payload.loiter_before_return_s;
    return out;
}

yunlink_msgs::GotoCommand mapGotoCommand(const yunlink::InboundCommandView<yunlink::GotoCommand>& view,
                                         const ros::Time& stamp) {
    yunlink_msgs::GotoCommand out;
    out.header = makeHeader(stamp);
    out.meta = mapCommandMeta(view.inbound.envelope.session_id,
                              view.inbound.envelope.message_id,
                              view.inbound.envelope.correlation_id);
    out.x_m = view.payload.x_m;
    out.y_m = view.payload.y_m;
    out.z_m = view.payload.z_m;
    out.yaw_rad = view.payload.yaw_rad;
    return out;
}

yunlink_msgs::VelocitySetpointCommand
mapVelocitySetpointCommand(const yunlink::InboundCommandView<yunlink::VelocitySetpointCommand>& view,
                           const ros::Time& stamp) {
    yunlink_msgs::VelocitySetpointCommand out;
    out.header = makeHeader(stamp);
    out.meta = mapCommandMeta(view.inbound.envelope.session_id,
                              view.inbound.envelope.message_id,
                              view.inbound.envelope.correlation_id);
    out.vx_mps = view.payload.vx_mps;
    out.vy_mps = view.payload.vy_mps;
    out.vz_mps = view.payload.vz_mps;
    out.yaw_rate_radps = view.payload.yaw_rate_radps;
    out.body_frame = view.payload.body_frame;
    return out;
}

sunray_msgs::UAVControlCMD mapControlCmd(const yunlink_msgs::TakeoffCommand& msg) {
    auto out = makeControlCmd(msg.header, sunray_msgs::UAVControlCMD::TAKEOFF);
    out.takeoff_relative_height = msg.relative_height_m;
    out.takeoff_max_velocity = msg.max_velocity_mps;
    return out;
}

sunray_msgs::UAVControlCMD mapControlCmd(const yunlink_msgs::LandCommand& msg) {
    auto out = makeControlCmd(msg.header, sunray_msgs::UAVControlCMD::LAND);
    out.land_max_velocity = msg.max_velocity_mps;
    return out;
}

sunray_msgs::UAVControlCMD mapControlCmd(const yunlink_msgs::ReturnCommand& msg) {
    auto out = makeControlCmd(msg.header, sunray_msgs::UAVControlCMD::RETURN);
    return out;
}

sunray_msgs::UAVControlCMD mapControlCmd(const yunlink_msgs::GotoCommand& msg) {
    auto out = makeControlCmd(msg.header, sunray_msgs::UAVControlCMD::MOVE_POINT);
    out.desired_pos.x = msg.x_m;
    out.desired_pos.y = msg.y_m;
    out.desired_pos.z = msg.z_m;
    out.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    out.desired_yaw = msg.yaw_rad;
    return out;
}

sunray_msgs::UAVControlCMD mapControlCmd(const yunlink_msgs::VelocitySetpointCommand& msg,
                                         float fixed_height_m) {
    auto out = makeControlCmd(msg.header,
                              msg.body_frame ? sunray_msgs::UAVControlCMD::MOVE_VELOCITY_BODY
                                             : sunray_msgs::UAVControlCMD::MOVE_VELOCITY);
    out.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAWRATE;
    out.desired_yaw_rate = msg.yaw_rate_radps;
    if (msg.body_frame) {
        out.desired_body_xy_vel.x = msg.vx_mps;
        out.desired_body_xy_vel.y = msg.vy_mps;
        out.fixed_height = fixed_height_m;
    } else {
        out.desired_vel.x = msg.vx_mps;
        out.desired_vel.y = msg.vy_mps;
        out.desired_vel.z = msg.vz_mps;
    }
    return out;
}

yunlink_msgs::CommandExecutionStatus mapLocalCommandExecutionStatusMsg(
    const sunray_msgs::UAVControlCommandStatus& msg,
    const yunlink_msgs::CommandMeta& meta) {
    yunlink_msgs::CommandExecutionStatus out;
    out.header = msg.header;
    out.agent_name = msg.agent_name;
    out.agent_id = msg.agent_id;
    out.meta = meta;
    out.command_kind = msg.command_kind;
    out.execution_state = msg.execution_state;
    out.progress_percent = msg.progress_percent;
    out.active = msg.active;
    out.terminal = msg.terminal;
    out.success = msg.success;
    out.result_code = msg.result_code;
    out.detail = msg.detail;
    out.control_state = msg.control_state;
    out.px4_landed_state = msg.px4_landed_state;
    out.ready_for_takeoff = msg.ready_for_takeoff;
    out.ready_for_land = msg.ready_for_land;
    out.busy_reason = msg.busy_reason;
    return out;
}

yunlink::CommandExecutionStatusSnapshot
mapCommandExecutionStatus(const yunlink_msgs::CommandExecutionStatus& msg) {
    yunlink::CommandExecutionStatusSnapshot out{};
    out.header = mapHeader(msg.header);
    out.agent_name = msg.agent_name;
    out.agent_id = msg.agent_id;
    out.session_id = msg.meta.session_id;
    out.command_message_id = msg.meta.message_id;
    out.command_correlation_id = msg.meta.correlation_id;
    out.command_kind = static_cast<yunlink::CommandKind>(msg.command_kind);
    out.execution_state = msg.execution_state;
    out.progress_percent = msg.progress_percent;
    out.active = msg.active;
    out.terminal = msg.terminal;
    out.success = msg.success;
    out.result_code = msg.result_code;
    out.detail = msg.detail;
    out.control_state = msg.control_state;
    out.px4_landed_state = msg.px4_landed_state;
    out.ready_for_takeoff = msg.ready_for_takeoff;
    out.ready_for_land = msg.ready_for_land;
    out.busy_reason = msg.busy_reason;
    return out;
}
