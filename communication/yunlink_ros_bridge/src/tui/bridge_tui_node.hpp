#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>

class BridgeTuiNode {
  public:
    BridgeTuiNode();

  private:
    struct FlowRow {
        std::string key;
        std::string label;
        std::string status{"WAIT_MESSAGE"};
        std::string topic{"-"};
        std::string detail;
        double hz{0.0};
        uint64_t age_ms{0};
        uint64_t count{0};
        uint32_t publisher_count{0};
        bool configured{true};
        bool has_message{false};
    };

    struct ConnectionState {
        std::string status{"WAIT"};
        std::string udp_bind_port{"-"};
        std::string udp_target_port{"-"};
        std::string tcp_listen_port{"-"};
        std::string remote_ip{"-"};
        std::string remote_tcp_port{"-"};
        std::string peer_id{"WAIT"};
        std::string session_id{"WAIT"};
    };

    struct EventRow {
        std::string direction;
        std::string key;
        std::string detail;
        double stamp_sec{0.0};
        uint64_t age_ms{0};
    };

    void loadParams();
    void setupDefaults();
    void setupRos();
    void onDiagnostics(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);
    void onPrintTimer(const ros::TimerEvent&);
    std::string buildPanel() const;
    void resetSnapshotLocked();
    void updateFlowRow(FlowRow* row, const diagnostic_msgs::DiagnosticStatus& status);
    static std::string findValue(const diagnostic_msgs::DiagnosticStatus& status, const char* key);
    static std::string formatAge(uint64_t age_ms, bool has_message);
    static std::string formatHz(double hz);
    static std::string formatStamp(double stamp_sec);
    static std::string colorStatus(const std::string& status);
    static std::string colorText(const std::string& text, const char* color);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber diagnostic_sub_;
    ros::Timer print_timer_;
    mutable std::mutex mu_;
    std::string diagnostic_topic_{"/yunlink_ros_bridge/monitor_diagnostics"};
    double print_hz_{2.0};
    int stale_timeout_ms_{1000};
    ros::Time last_snapshot_time_{0.0};
    std::string agent_key_{"/uav?"};
    ConnectionState connection_;
    std::map<std::string, FlowRow> ros_to_yunlink_;
    std::map<std::string, FlowRow> yunlink_to_ros_;
    std::vector<EventRow> recent_events_;
};
