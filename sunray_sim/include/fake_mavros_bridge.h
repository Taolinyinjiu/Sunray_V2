#ifndef SUNRAY_SIM_FAKE_MAVROS_BRIDGE_H
#define SUNRAY_SIM_FAKE_MAVROS_BRIDGE_H

#include <ros/ros.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SysStatus.h>
#include <mavros_msgs/GPSRAW.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <unordered_map>

namespace sunray_sim
{
class FakeMavrosBridge {
public:
    FakeMavrosBridge(ros::NodeHandle& nh, const std::string& uav_name);
    void printStatus() const;

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void navsatCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void localSetpointCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void attitudeSetpointCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
    void publishTimerCallback(const ros::TimerEvent& event);

    bool setModeService(mavros_msgs::SetMode::Request& req,
                        mavros_msgs::SetMode::Response& res);
    bool armingService(mavros_msgs::CommandBool::Request& req,
                       mavros_msgs::CommandBool::Response& res);
    bool commandLongService(mavros_msgs::CommandLong::Request& req,
                            mavros_msgs::CommandLong::Response& res);
    bool paramGetService(mavros_msgs::ParamGet::Request& req,
                         mavros_msgs::ParamGet::Response& res);
    bool paramSetService(mavros_msgs::ParamSet::Request& req,
                         mavros_msgs::ParamSet::Response& res);

    void publishLocalPositionOutputs(const ros::Time& stamp);
    void publishGlobalPositionOutputs(const ros::Time& stamp);
    uint8_t estimateLandedState() const;
    static mavros_msgs::ParamValue makeIntegerParamValue(int64_t value);
    static mavros_msgs::ParamValue makeRealParamValue(double value);
    std::string buildStatusPanel() const;

    ros::NodeHandle nh_;
    std::string uav_name_;

    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber navsat_sub_;
    ros::Subscriber local_setpoint_sub_;
    ros::Subscriber attitude_setpoint_sub_;

    ros::Publisher state_pub_;
    ros::Publisher extended_state_pub_;
    ros::Publisher sys_status_pub_;
    ros::Publisher estimator_status_pub_;
    ros::Publisher local_odom_pub_;
    ros::Publisher local_pose_pub_;
    ros::Publisher local_velocity_pub_;
    ros::Publisher global_position_pub_;
    ros::Publisher gps_raw_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher target_local_pub_;
    ros::Publisher target_attitude_pub_;

    ros::ServiceServer set_mode_srv_;
    ros::ServiceServer arming_srv_;
    ros::ServiceServer command_long_srv_;
    ros::ServiceServer param_get_srv_;
    ros::ServiceServer param_set_srv_;
    ros::Timer publish_timer_;

    nav_msgs::Odometry latest_odom_;
    sensor_msgs::Imu latest_imu_;
    sensor_msgs::NavSatFix latest_navsat_;
    mavros_msgs::PositionTarget latest_local_target_;
    mavros_msgs::AttitudeTarget latest_attitude_target_;
    ros::Time last_local_target_time_;
    ros::Time last_attitude_target_time_;

    bool has_odom_{false};
    bool has_imu_{false};
    bool has_navsat_{false};
    bool has_local_target_{false};
    bool has_attitude_target_{false};

    bool connected_{true};
    bool armed_{true};
    bool manual_input_{true};
    std::string current_mode_{"OFFBOARD"};
    uint8_t system_status_{4};
    std::unordered_map<std::string, mavros_msgs::ParamValue> params_;

    double mavros_publish_rate_hz_{50.0};
    double on_ground_height_threshold_m_{0.05};
    double on_ground_velocity_threshold_mps_{0.10};
    float battery_voltage_v_{15.2f};
    float battery_current_a_{0.0f};
    float battery_remaining_{0.9f};
    uint16_t system_load_raw_{150};
};
}  // namespace sunray_sim

#endif  // SUNRAY_SIM_FAKE_MAVROS_BRIDGE_H
