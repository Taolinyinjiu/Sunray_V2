#pragma once

#include <cstdint>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <string>
#include <thread>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sunray_msgs/GetFeatures.h>
#include <sunray_msgs/ListFeatures.h>
#include <sunray_msgs/OdomState.h>
#include <sunray_msgs/Px4State.h>
#include <sunray_msgs/StartFeature.h>
#include <sunray_msgs/StopFeature.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVCommandExecutionStatus.h>
#include <sunray_msgs/UAVControlState.h>
#include <yunlink/runtime/runtime.hpp>

#include "discovery/bridge_endpoint_discovery.hpp"

struct BridgeParams {
    std::string local_odom_topic{"/uav1/sunray/localization/local_odom"};
    std::string odom_state_topic{"/uav1/sunray/localization/odom_state"};
    std::string control_state_topic{"/uav1/sunray/uav_control/control_state"};
    std::string command_execution_status_topic{"/uav1/sunray/uav_control/command_execution_status"};
    std::string control_cmd_topic{"/uav1/sunray/uav_control/control_cmd"};
    std::string px4_state_topic{"/uav1/sunray/px4_state"};
    std::string external_odom_topic;
    std::string global_odom_topic{"/uav1/sunray/localization/global_odom"};
    bool enable_system_services{true};
    bool enable_runtime_diagnostics{true};
    std::string sunray_system_ns{"/sunray_system"};
    double system_service_timeout_sec{3.0};
    double diagnostic_publish_rate_hz{2.0};
    int diagnostic_stale_timeout_ms{1000};
    std::string monitor_diagnostic_topic{"/yunlink_ros_bridge/monitor_diagnostics"};
    bool default_start_with_terminal{false};
    std::string remote_ip;
    std::string shared_secret{"yunlink-default-secret"};
    std::string node_name{"yunlink_ros_bridge_node"};
    int remote_tcp_port{0};
    int udp_bind_port{9696};
    int udp_target_port{9898};
    int tcp_listen_port{9696};
    int agent_id{1};
    bool enable_endpoint_discovery{true};
    int discovery_port{9966};
    std::string discovery_target_ip{"255.255.255.255"};
    int discovery_period_ms{1000};
    std::string endpoint_name_prefix{"yundrone_uav"};
    std::string endpoint_id_file;
};

class YunlinkRosBridgeNode {
  public:
    YunlinkRosBridgeNode();
    ~YunlinkRosBridgeNode();

    void onLocalOdom(const nav_msgs::Odometry::ConstPtr& msg);
    void onOdomState(const sunray_msgs::OdomState::ConstPtr& msg);
    void onControlCmd(const sunray_msgs::UAVControlCMD::ConstPtr& msg);
    void onControlState(const sunray_msgs::UAVControlState::ConstPtr& msg);
    void onCommandExecutionStatus(const sunray_msgs::UAVCommandExecutionStatus::ConstPtr& msg);
    void onPx4State(const sunray_msgs::Px4State::ConstPtr& msg);
    void onExternalOdomDiagnostic(const nav_msgs::Odometry::ConstPtr& msg);
    void onGlobalOdomDiagnostic(const nav_msgs::Odometry::ConstPtr& msg);
    void onDiagnosticTimer(const ros::TimerEvent&);

    void onTakeoffCommand(const yunlink::InboundCommandView<yunlink::TakeoffCommand>& view);
    void onLandCommand(const yunlink::InboundCommandView<yunlink::LandCommand>& view);
    void onReturnCommand(const yunlink::InboundCommandView<yunlink::ReturnCommand>& view);
    void onGotoCommand(const yunlink::InboundCommandView<yunlink::GotoCommand>& view);
    void onVelocitySetpointCommand(
        const yunlink::InboundCommandView<yunlink::VelocitySetpointCommand>& view);
    void onFeatureListRequest(
        const yunlink::InboundSystemServiceRequestView<yunlink::FeatureListRequest>& view);
    void onFeatureGetRequest(
        const yunlink::InboundSystemServiceRequestView<yunlink::FeatureGetRequest>& view);
    void onFeatureStartRequest(
        const yunlink::InboundSystemServiceRequestView<yunlink::FeatureStartRequest>& view);
    void onFeatureStopRequest(
        const yunlink::InboundSystemServiceRequestView<yunlink::FeatureStopRequest>& view);

  private:
  public:
    enum class BridgeFlowDirection {
        kRosToYunlink,
        kYunlinkToRos,
    };

  private:
    struct TopicDiagnosticRuntime {
        std::string key;
        std::string label;
        std::string topic;
        bool configured{false};
        bool has_message{false};
        uint64_t message_count{0};
        uint32_t publisher_count{0};
        double hz{0.0};
        ros::Time last_receive_time{0.0};
        ros::Time previous_receive_time{0.0};
        ros::Time last_event_log_time{0.0};
        std::string detail;
    };

    struct FlowDiagnosticRuntime {
        std::string key;
        std::string label;
        bool configured{true};
        bool stale_by_age{false};
        bool has_message{false};
        uint64_t message_count{0};
        double hz{0.0};
        ros::Time last_receive_time{0.0};
        ros::Time previous_receive_time{0.0};
        std::string detail;
    };

    struct BridgeEvent {
        ros::Time time{0.0};
        BridgeFlowDirection direction{BridgeFlowDirection::kRosToYunlink};
        std::string key;
        std::string detail;
    };

    enum class SystemServiceJobKind {
        kFeatureList,
        kFeatureGet,
        kFeatureStart,
        kFeatureStop,
    };

    struct SystemServiceJob {
        SystemServiceJobKind kind{SystemServiceJobKind::kFeatureList};
        yunlink::EnvelopeEvent inbound;
        yunlink::FeatureGetRequest feature_get;
        yunlink::FeatureStartRequest feature_start;
        yunlink::FeatureStopRequest feature_stop;
    };

    void loadParams();
    void startRuntime();
    void setupSubscribers();
    void setupDiagnosticState();
    void setupMonitorState();
    void setupEndpointDiscovery();
    void setupReconnectTimer();
    void setupSystemServiceClients();
    void startSystemServiceWorker();
    void stopSystemServiceWorker();
    void onReconnectTimer(const ros::TimerEvent&);
    void onEndpointDiscoveryTimer(const ros::TimerEvent&);
    void publishRuntimeDiagnosticSnapshot();
    void publishMonitorDiagnosticArray(const ros::Time& now);
    void refreshTopicPublisherCounts();
    void systemServiceWorkerLoop();
    void enqueueSystemServiceJob(SystemServiceJob job);
    bool waitForService(ros::ServiceClient& client, const char* name) const;
    void handleFeatureListJob(const SystemServiceJob& job);
    void handleFeatureGetJob(const SystemServiceJob& job);
    void handleFeatureStartJob(const SystemServiceJob& job);
    void handleFeatureStopJob(const SystemServiceJob& job);

    static uint16_t clampPort(int port);
    bool ensurePeerReady();
    yunlink::TargetSelector targetSelector() const;
    float latestPx4Height(bool* has_height = nullptr) const;
    sunray_msgs::UAVControlCMD makeBaseControlCmd(uint8_t control_cmd) const;
    void publishControlCmd(const char* name,
                           const sunray_msgs::UAVControlCMD& cmd,
                           uint64_t session_id,
                           uint64_t correlation_id,
                           uint64_t message_id,
                           const std::string& detail = std::string());
    void recordRosToYunlinkEvent(TopicDiagnosticRuntime* runtime,
                                 const ros::Time& receive_time,
                                 const std::string& detail = std::string(),
                                 bool log_event = true);
    void recordYunlinkToRosEvent(const char* key, const std::string& detail);
    void appendRecentEventUnlocked(BridgeFlowDirection direction,
                                   const std::string& key,
                                   const std::string& detail,
                                   const ros::Time& event_time);
    void updateTopicDiagnostic(TopicDiagnosticRuntime* runtime, const ros::Time& receive_time);
    yunlink::SunrayTopicDiagnosticSnapshot
    makeTopicDiagnosticSnapshot(const TopicDiagnosticRuntime& runtime, const ros::Time& now) const;
    static std::string statusForTopicDiagnostic(const TopicDiagnosticRuntime& runtime,
                                                const ros::Time& now,
                                                int stale_timeout_ms);
    static std::string statusForMonitorFlow(bool configured,
                                            bool has_message,
                                            uint32_t publisher_count,
                                            const ros::Time& last_receive_time,
                                            const ros::Time& now,
                                            int stale_timeout_ms,
                                            bool require_publisher,
                                            bool stale_by_age);

    template <typename PublishFn, typename PayloadT>
    void publishSnapshot(const char* name, PublishFn publish_fn, const PayloadT& payload) {
        if (!ensurePeerReady()) {
            return;
        }

        const auto ec = (runtime_.*publish_fn)(peer_id_, targetSelector(), payload, session_id_);
        if (ec != yunlink::ErrorCode::kOk) {
            peer_ready_ = false;
            ROS_WARN_THROTTLE(5.0,
                              "yunlink_ros_bridge publish %s failed, ec=%u",
                              name,
                              static_cast<unsigned>(ec));
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber local_odom_sub_;
    ros::Subscriber odom_state_sub_;
    ros::Subscriber control_cmd_sub_;
    ros::Subscriber control_state_sub_;
    ros::Subscriber command_execution_status_sub_;
    ros::Subscriber px4_state_sub_;
    ros::Subscriber external_odom_diag_sub_;
    ros::Subscriber global_odom_diag_sub_;
    ros::Publisher control_cmd_pub_;
    ros::Publisher monitor_diagnostic_pub_;
    ros::Timer reconnect_timer_;
    ros::Timer diagnostic_timer_;
    ros::Timer endpoint_discovery_timer_;
    ros::ServiceClient list_features_client_;
    ros::ServiceClient get_features_client_;
    ros::ServiceClient start_feature_client_;
    ros::ServiceClient stop_feature_client_;
    yunlink::Runtime runtime_;
    BridgeEndpointDiscovery endpoint_discovery_;
    BridgeParams params_;
    std::string peer_id_;
    size_t takeoff_command_sub_token_{0};
    size_t land_command_sub_token_{0};
    size_t return_command_sub_token_{0};
    size_t goto_command_sub_token_{0};
    size_t velocity_setpoint_command_sub_token_{0};
    size_t feature_list_request_sub_token_{0};
    size_t feature_get_request_sub_token_{0};
    size_t feature_start_request_sub_token_{0};
    size_t feature_stop_request_sub_token_{0};
    uint64_t session_id_{0};
    bool peer_ready_{false};
    mutable std::mutex px4_state_mu_;
    mutable std::mutex diag_mu_;
    mutable std::mutex system_service_mu_;
    std::condition_variable system_service_cv_;
    std::deque<SystemServiceJob> system_service_jobs_;
    std::thread system_service_worker_;
    bool stop_system_service_worker_{false};
    float latest_px4_height_m_{0.0F};
    bool has_px4_height_{false};
    TopicDiagnosticRuntime external_odom_diag_;
    TopicDiagnosticRuntime odom_state_diag_;
    TopicDiagnosticRuntime local_odom_diag_;
    TopicDiagnosticRuntime global_odom_diag_;
    TopicDiagnosticRuntime uav_control_cmd_diag_;
    TopicDiagnosticRuntime uav_control_state_diag_;
    TopicDiagnosticRuntime px4_state_diag_;
    FlowDiagnosticRuntime takeoff_diag_;
    FlowDiagnosticRuntime land_diag_;
    FlowDiagnosticRuntime return_diag_;
    FlowDiagnosticRuntime goto_diag_;
    FlowDiagnosticRuntime velocity_setpoint_diag_;
    FlowDiagnosticRuntime feature_list_diag_;
    FlowDiagnosticRuntime feature_get_diag_;
    FlowDiagnosticRuntime feature_start_diag_;
    FlowDiagnosticRuntime feature_stop_diag_;
    std::deque<BridgeEvent> recent_events_;
};
