/** @file @brief bridge ROS topic 和 YunLink command 订阅初始化实现。 */
#include "bridge_node.hpp"

#include <algorithm>

/// 建立 ROS topic 订阅/发布和 YunLink command/system-service 订阅。
void YunlinkRosBridgeNode::setupSubscribers() {
    int control_cmd_queue_size = 20;
    // 多种 YunLink 飞行命令共用同一个 Sunray control_cmd publisher，
    // 因此取所有命令通道配置中的最大 queue_size。
    for (const char* channel : {"takeoff_command",
                                "land_command",
                                "return_command",
                                "goto_command",
                                "velocity_setpoint_command",
                                "takeoff",
                                "land",
                                "return",
                                "goto",
                                "velocity_setpoint"}) {
        control_cmd_queue_size =
            std::max(control_cmd_queue_size,
                     queueSize(qos_channels_.yunlink_to_ros, channel, control_cmd_queue_size));
    }
    control_cmd_pub_ =
        nh_.advertise<sunray_msgs::UAVControlCMD>(params_.control_cmd_topic, control_cmd_queue_size);
    monitor_diagnostic_pub_ =
        nh_.advertise<diagnostic_msgs::DiagnosticArray>(params_.monitor_diagnostic_topic, 1, true);

    const auto hints_for = [this](const char* channel) {
        ros::TransportHints hints;
        if (tcpNoDelay(qos_channels_.ros_to_yunlink, channel, true)) {
            hints.tcpNoDelay();
        }
        return hints;
    };

    // 状态 snapshot 只关心最新值，不需要积压历史消息。
    local_odom_sub_ = nh_.subscribe(params_.local_odom_topic,
                                    queueSize(qos_channels_.ros_to_yunlink, "local_odom", 1),
                                    &YunlinkRosBridgeNode::onLocalOdom,
                                    this,
                                    hints_for("local_odom"));
    odom_state_sub_ = nh_.subscribe(params_.odom_state_topic,
                                    queueSize(qos_channels_.ros_to_yunlink, "odom_state", 1),
                                    &YunlinkRosBridgeNode::onOdomState,
                                    this,
                                    hints_for("odom_state"));
    control_cmd_sub_ = nh_.subscribe(params_.control_cmd_topic,
                                     queueSize(qos_channels_.ros_to_yunlink, "uav_control_cmd", 1),
                                     &YunlinkRosBridgeNode::onControlCmd,
                                     this,
                                     hints_for("uav_control_cmd"));
    control_state_sub_ = nh_.subscribe(params_.control_state_topic,
                                       queueSize(qos_channels_.ros_to_yunlink, "control_state", 1),
                                       &YunlinkRosBridgeNode::onControlState,
                                       this,
                                       hints_for("control_state"));
    command_execution_status_sub_ = nh_.subscribe(params_.command_execution_status_topic,
                                                  queueSize(qos_channels_.ros_to_yunlink,
                                                            "command_execution_status",
                                                            1),
                                                  &YunlinkRosBridgeNode::onCommandExecutionStatus,
                                                  this,
                                                  hints_for("command_execution_status"));
    px4_state_sub_ = nh_.subscribe(params_.px4_state_topic,
                                   queueSize(qos_channels_.ros_to_yunlink, "px4_state", 1),
                                   &YunlinkRosBridgeNode::onPx4State,
                                   this,
                                   hints_for("px4_state"));
    if (params_.enable_runtime_diagnostics && !params_.external_odom_topic.empty()) {
        external_odom_diag_sub_ = nh_.subscribe(params_.external_odom_topic,
                                                queueSize(qos_channels_.ros_to_yunlink,
                                                          "external_odom",
                                                          1),
                                                &YunlinkRosBridgeNode::onExternalOdomDiagnostic,
                                                this,
                                                hints_for("external_odom"));
    }
    if (params_.enable_runtime_diagnostics && !params_.global_odom_topic.empty()) {
        global_odom_diag_sub_ = nh_.subscribe(params_.global_odom_topic,
                                              queueSize(qos_channels_.ros_to_yunlink,
                                                        "global_odom",
                                                        1),
                                              &YunlinkRosBridgeNode::onGlobalOdomDiagnostic,
                                              this,
                                              hints_for("global_odom"));
    }

    takeoff_command_sub_token_ = runtime_.command_subscriber().subscribe_takeoff(
        [this](const yunlink::InboundCommandView<yunlink::TakeoffCommand>& view) {
            onTakeoffCommand(view);
        });
    land_command_sub_token_ = runtime_.command_subscriber().subscribe_land(
        [this](const yunlink::InboundCommandView<yunlink::LandCommand>& view) {
            onLandCommand(view);
        });
    return_command_sub_token_ = runtime_.command_subscriber().subscribe_return(
        [this](const yunlink::InboundCommandView<yunlink::ReturnCommand>& view) {
            onReturnCommand(view);
        });
    goto_command_sub_token_ = runtime_.command_subscriber().subscribe_goto(
        [this](const yunlink::InboundCommandView<yunlink::GotoCommand>& view) {
            onGotoCommand(view);
        });
    velocity_setpoint_command_sub_token_ =
        runtime_.command_subscriber().subscribe_velocity_setpoint(
            [this](const yunlink::InboundCommandView<yunlink::VelocitySetpointCommand>& view) {
                onVelocitySetpointCommand(view);
            });

    if (params_.enable_system_services) {
        feature_list_request_sub_token_ =
            runtime_.system_service_subscriber().subscribe_feature_list_requests(
                [this](const yunlink::InboundSystemServiceRequestView<yunlink::FeatureListRequest>& view) {
                    onFeatureListRequest(view);
                });
        feature_get_request_sub_token_ =
            runtime_.system_service_subscriber().subscribe_feature_get_requests(
                [this](const yunlink::InboundSystemServiceRequestView<yunlink::FeatureGetRequest>& view) {
                    onFeatureGetRequest(view);
                });
        feature_start_request_sub_token_ =
            runtime_.system_service_subscriber().subscribe_feature_start_requests(
                [this](const yunlink::InboundSystemServiceRequestView<yunlink::FeatureStartRequest>& view) {
                    onFeatureStartRequest(view);
                });
        feature_stop_request_sub_token_ =
            runtime_.system_service_subscriber().subscribe_feature_stop_requests(
                [this](const yunlink::InboundSystemServiceRequestView<yunlink::FeatureStopRequest>& view) {
                    onFeatureStopRequest(view);
                });
    }

    ROS_INFO("yunlink_ros_bridge advertise control_cmd: %s queue_size=%d",
             params_.control_cmd_topic.c_str(),
             control_cmd_queue_size);
    ROS_INFO("yunlink_ros_bridge subscribe local_odom: %s queue_size=%d tcp_no_delay=%s",
             params_.local_odom_topic.c_str(),
             queueSize(qos_channels_.ros_to_yunlink, "local_odom", 1),
             tcpNoDelay(qos_channels_.ros_to_yunlink, "local_odom", true) ? "true" : "false");
    ROS_INFO("yunlink_ros_bridge subscribe odom_state: %s queue_size=%d tcp_no_delay=%s",
             params_.odom_state_topic.c_str(),
             queueSize(qos_channels_.ros_to_yunlink, "odom_state", 1),
             tcpNoDelay(qos_channels_.ros_to_yunlink, "odom_state", true) ? "true" : "false");
    ROS_INFO("yunlink_ros_bridge subscribe control_cmd: %s queue_size=%d tcp_no_delay=%s",
             params_.control_cmd_topic.c_str(),
             queueSize(qos_channels_.ros_to_yunlink, "uav_control_cmd", 1),
             tcpNoDelay(qos_channels_.ros_to_yunlink, "uav_control_cmd", true) ? "true" : "false");
    ROS_INFO("yunlink_ros_bridge subscribe control_state: %s queue_size=%d tcp_no_delay=%s",
             params_.control_state_topic.c_str(),
             queueSize(qos_channels_.ros_to_yunlink, "control_state", 1),
             tcpNoDelay(qos_channels_.ros_to_yunlink, "control_state", true) ? "true" : "false");
    ROS_INFO("yunlink_ros_bridge subscribe command_execution_status: %s",
             params_.command_execution_status_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe px4_state: %s queue_size=%d tcp_no_delay=%s",
             params_.px4_state_topic.c_str(),
             queueSize(qos_channels_.ros_to_yunlink, "px4_state", 1),
             tcpNoDelay(qos_channels_.ros_to_yunlink, "px4_state", true) ? "true" : "false");
    if (params_.enable_runtime_diagnostics) {
        ROS_INFO("yunlink_ros_bridge diagnostic external_odom: %s",
                 params_.external_odom_topic.empty() ? "<empty>" : params_.external_odom_topic.c_str());
        ROS_INFO("yunlink_ros_bridge diagnostic global_odom: %s",
                 params_.global_odom_topic.empty() ? "<empty>" : params_.global_odom_topic.c_str());
    }
    ROS_INFO("yunlink_ros_bridge monitor diagnostics topic: %s",
             params_.monitor_diagnostic_topic.c_str());
    ROS_INFO("yunlink_ros_bridge subscribe yunlink commands: takeoff land return goto velocity_setpoint");
    if (params_.enable_system_services) {
        ROS_INFO("yunlink_ros_bridge subscribe yunlink system services: feature_list feature_get feature_start feature_stop");
    }
}
