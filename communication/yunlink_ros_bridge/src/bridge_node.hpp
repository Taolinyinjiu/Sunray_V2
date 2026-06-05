#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sunray_msgs/OdomState.h>
#include <sunray_msgs/Px4State.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlState.h>
#include <yunlink/runtime/runtime.hpp>

struct BridgeParams {
    std::string local_odom_topic{"/uav1/sunray/localization/local_odom"};
    std::string odom_state_topic{"/uav1/sunray/localization/odom_state"};
    std::string control_state_topic{"/uav1/sunray/uav_control/control_state"};
    std::string control_cmd_topic{"/uav1/sunray/uav_control/control_cmd"};
    std::string mavros_state_topic{"/uav1/mavros/state"};
    std::string px4_state_topic{"/uav1/sunray/px4_state"};
    std::string remote_ip;
    std::string shared_secret{"yunlink-default-secret"};
    std::string node_name{"yunlink_ros_bridge_node"};
    int remote_tcp_port{0};
    int udp_bind_port{9696};
    int udp_target_port{9898};
    int tcp_listen_port{9696};
    int agent_id{1};
};

class YunlinkRosBridgeNode {
  public:
    YunlinkRosBridgeNode();
    ~YunlinkRosBridgeNode();

    void onLocalOdom(const nav_msgs::Odometry::ConstPtr& msg);
    void onOdomState(const sunray_msgs::OdomState::ConstPtr& msg);
    void onControlState(const sunray_msgs::UAVControlState::ConstPtr& msg);
    void onMavrosState(const mavros_msgs::State::ConstPtr& msg);
    void onPx4State(const sunray_msgs::Px4State::ConstPtr& msg);

    void onTakeoffCommand(const yunlink::InboundCommandView<yunlink::TakeoffCommand>& view);
    void onLandCommand(const yunlink::InboundCommandView<yunlink::LandCommand>& view);
    void onReturnCommand(const yunlink::InboundCommandView<yunlink::ReturnCommand>& view);
    void onGotoCommand(const yunlink::InboundCommandView<yunlink::GotoCommand>& view);
    void onVelocitySetpointCommand(
        const yunlink::InboundCommandView<yunlink::VelocitySetpointCommand>& view);

  private:
    void loadParams();
    void startRuntime();
    void setupSubscribers();
    void setupReconnectTimer();
    void onReconnectTimer(const ros::TimerEvent&);

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
    ros::Subscriber control_state_sub_;
    ros::Subscriber mavros_state_sub_;
    ros::Subscriber px4_state_sub_;
    ros::Publisher control_cmd_pub_;
    ros::Timer reconnect_timer_;
    yunlink::Runtime runtime_;
    BridgeParams params_;
    std::string peer_id_;
    size_t takeoff_command_sub_token_{0};
    size_t land_command_sub_token_{0};
    size_t return_command_sub_token_{0};
    size_t goto_command_sub_token_{0};
    size_t velocity_setpoint_command_sub_token_{0};
    uint64_t session_id_{0};
    bool peer_ready_{false};
    mutable std::mutex px4_state_mu_;
    float latest_px4_height_m_{0.0F};
    bool has_px4_height_{false};
};
