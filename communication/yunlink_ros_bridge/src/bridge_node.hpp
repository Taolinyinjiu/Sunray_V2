/** @file @brief YunLink ROS bridge 主节点接口与运行时状态定义。 */
#pragma once

#include <cstdint>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <map>
#include <string>
#include <thread>
#include <vector>

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
#include <yunlink/core/semantic/message_ids.hpp>
#include <yunlink/runtime/runtime.hpp>

#include "bridge_runtime_types.hpp"
#include "discovery/bridge_endpoint_discovery.hpp"

/** @brief 负责 YunLink runtime 与 Sunray ROS topic/service 之间的双向适配。 */
class YunlinkRosBridgeNode {
  public:
    YunlinkRosBridgeNode();  ///< @brief 构造并启动 bridge runtime、ROS 订阅发布和诊断定时器。
    ~YunlinkRosBridgeNode(); ///< @brief 停止后台线程、端点发现和 YunLink runtime。

    void onLocalOdom(const nav_msgs::Odometry::ConstPtr& msg); ///< @brief 转发本地里程计 snapshot。 @param msg ROS 本地里程计消息。
    void onOdomState(const sunray_msgs::OdomState::ConstPtr& msg); ///< @brief 转发定位状态 snapshot。 @param msg Sunray 定位状态消息。
    void onControlCmd(const sunray_msgs::UAVControlCMD::ConstPtr& msg); ///< @brief 转发控制命令 snapshot。 @param msg Sunray 控制命令消息。
    void onControlState(const sunray_msgs::UAVControlState::ConstPtr& msg); ///< @brief 转发控制状态 snapshot。 @param msg Sunray 控制状态消息。
    void onCommandExecutionStatus(const sunray_msgs::UAVCommandExecutionStatus::ConstPtr& msg); ///< @brief 转发命令执行状态 snapshot。 @param msg 控制侧发布的命令执行状态。
    void onPx4State(const sunray_msgs::Px4State::ConstPtr& msg); ///< @brief 转发 PX4 状态 snapshot 并缓存当前高度。 @param msg Sunray PX4 状态消息。
    void onExternalOdomDiagnostic(const nav_msgs::Odometry::ConstPtr& msg); ///< @brief 记录外部里程计诊断输入。 @param msg 外部里程计消息。
    void onGlobalOdomDiagnostic(const nav_msgs::Odometry::ConstPtr& msg); ///< @brief 记录全局里程计诊断输入。 @param msg 全局里程计消息。
    void onDiagnosticTimer(const ros::TimerEvent& event); ///< @brief 周期发布 bridge 运行诊断。 @param event ROS 定时器事件。

    void onTakeoffCommand(const yunlink::InboundCommandView<yunlink::TakeoffCommand>& view); ///< @brief 将 YunLink 起飞命令转成 Sunray 控制命令。 @param view 入站起飞命令视图。
    void onLandCommand(const yunlink::InboundCommandView<yunlink::LandCommand>& view); ///< @brief 将 YunLink 降落命令转成 Sunray 控制命令。 @param view 入站降落命令视图。
    void onReturnCommand(const yunlink::InboundCommandView<yunlink::ReturnCommand>& view); ///< @brief 将 YunLink 返航命令转成 Sunray 控制命令。 @param view 入站返航命令视图。
    void onGotoCommand(const yunlink::InboundCommandView<yunlink::GotoCommand>& view); ///< @brief 将 YunLink 定点命令转成 Sunray 控制命令。 @param view 入站定点命令视图。
    void onVelocitySetpointCommand(const yunlink::InboundCommandView<yunlink::VelocitySetpointCommand>& view); ///< @brief 将 YunLink 速度设定命令转成 Sunray 控制命令。 @param view 入站速度设定命令视图。
    void onFeatureListRequest(const yunlink::InboundSystemServiceRequestView<yunlink::FeatureListRequest>& view); ///< @brief 转发功能列表请求到 sunray_system。 @param view 入站功能列表请求视图。
    void onFeatureGetRequest(const yunlink::InboundSystemServiceRequestView<yunlink::FeatureGetRequest>& view); ///< @brief 转发功能查询请求到 sunray_system。 @param view 入站功能查询请求视图。
    void onFeatureStartRequest(const yunlink::InboundSystemServiceRequestView<yunlink::FeatureStartRequest>& view); ///< @brief 转发功能启动请求到 sunray_system。 @param view 入站功能启动请求视图。
    void onFeatureStopRequest(const yunlink::InboundSystemServiceRequestView<yunlink::FeatureStopRequest>& view); ///< @brief 转发功能停止请求到 sunray_system。 @param view 入站功能停止请求视图。

  private:
  public:
    /** @brief bridge 诊断事件的转发方向。 */
    enum class BridgeFlowDirection { kRosToYunlink, kYunlinkToRos };

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
        std::string last_status;
        std::string last_transition;
        ros::Time last_transition_time{0.0};
        uint64_t publish_fail_count{0};
        double expected_min_hz{0.0};
        bool sparse{false};
    };

    struct FlowDiagnosticRuntime {
        std::string key;
        std::string label;
        bool configured{true};
        bool stale_by_age{false};
        bool has_message{false};
        uint64_t message_count{0};
        uint64_t publish_count{0};
        uint64_t fail_count{0};
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

    enum class SystemServiceJobKind { kFeatureList, kFeatureGet, kFeatureStart, kFeatureStop };

    struct SystemServiceJob {
        SystemServiceJobKind kind{SystemServiceJobKind::kFeatureList};
        yunlink::EnvelopeEvent inbound;
        yunlink::FeatureGetRequest feature_get;
        yunlink::FeatureStartRequest feature_start;
        yunlink::FeatureStopRequest feature_stop;
    };

    void loadParams();
    bool loadQosParams();
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
    bool waitForService(ros::ServiceClient& client, const char* name, double timeout_sec) const;
    double serviceTimeoutSec(const char* channel) const;
    void handleFeatureListJob(const SystemServiceJob& job);
    void handleFeatureGetJob(const SystemServiceJob& job);
    void handleFeatureStartJob(const SystemServiceJob& job);
    void handleFeatureStopJob(const SystemServiceJob& job);

    static uint16_t clampPort(int port);
    static std::string normalizeQosToken(const std::string& value);
    static const char* qosProfileName(yunlink::QosProfile profile);
    static const char* transportName(yunlink::TransportPreference transport);
    yunlink::RuntimeQosPolicy makeRuntimeQosPolicy() const;
    std::vector<yunlink::RuntimeQosChannelPolicy> makeRuntimeQosChannelPolicies() const;
    bool validateQosChannels(std::string* error) const;
    int queueSize(const std::map<std::string, BridgeChannelQosConfig>& group,
                  const char* channel,
                  int fallback) const;
    bool tcpNoDelay(const std::map<std::string, BridgeChannelQosConfig>& group,
                    const char* channel,
                    bool fallback) const;
    double publishHz(const std::map<std::string, BridgeChannelQosConfig>& group,
                     const char* channel,
                     double fallback) const;
    BridgeChannelQosConfig channelQos(const std::map<std::string, BridgeChannelQosConfig>& group,
                                      const char* channel) const;
    bool shouldPublishRosToYunlink(const char* channel, const ros::Time& now);
    void logQosConfig() const;
    bool ensurePeerReady();
    yunlink::TargetSelector targetSelector() const;
    float latestPx4Height(bool* has_height = nullptr) const;
    sunray_msgs::UAVControlCMD makeBaseControlCmd(uint8_t control_cmd) const;
    void publishControlCmd(const char* name, const sunray_msgs::UAVControlCMD& cmd,
                           uint64_t session_id, uint64_t correlation_id, uint64_t message_id,
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
    void recordPublishFailureUnlocked(const std::string& direction,
                                      const std::string& key,
                                      uint32_t error_code,
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
    void recordRosToYunlinkPublishResult(const char* name, yunlink::ErrorCode ec);

    template <typename PublishFn, typename PayloadT>
    void publishSnapshot(const char* name, PublishFn publish_fn, const PayloadT& payload) {
        if (!ensurePeerReady()) {
            return;
        }

        const auto ec = (runtime_.*publish_fn)(peer_id_, targetSelector(), payload, session_id_);
        recordRosToYunlinkPublishResult(name, ec);
        if (ec != yunlink::ErrorCode::kOk) {
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
    BridgeQosChannels qos_channels_;
    std::string peer_id_;
    size_t takeoff_command_sub_token_{0}, land_command_sub_token_{0}, return_command_sub_token_{0};
    size_t goto_command_sub_token_{0}, velocity_setpoint_command_sub_token_{0};
    size_t feature_list_request_sub_token_{0}, feature_get_request_sub_token_{0};
    size_t feature_start_request_sub_token_{0}, feature_stop_request_sub_token_{0};
    uint64_t session_id_{0};
    bool peer_ready_{false};
    mutable std::mutex px4_state_mu_;
    mutable std::mutex diag_mu_;
    mutable std::mutex system_service_mu_;
    std::condition_variable system_service_cv_;
    std::deque<SystemServiceJob> system_service_jobs_;
    std::thread system_service_worker_;
    bool stop_system_service_worker_{false};
    bool runtime_started_{false};
    uint64_t connect_attempt_count_{0}, session_lost_count_{0};
    uint64_t ros_to_yunlink_publish_count_{0}, ros_to_yunlink_fail_count_{0};
    uint64_t yunlink_to_ros_command_count_{0}, yunlink_to_ros_publish_count_{0};
    uint64_t yunlink_to_ros_fail_count_{0};
    std::string last_connect_error_;
    std::string last_session_error_;
    std::string last_publish_error_;
    std::string last_fail_direction_;
    std::string last_fail_key_;
    uint32_t last_fail_error_code_{0};
    std::string last_fail_detail_;
    ros::Time last_error_time_{0.0};
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
    std::map<std::string, ros::Time> last_ros_to_yunlink_publish_time_;
};
