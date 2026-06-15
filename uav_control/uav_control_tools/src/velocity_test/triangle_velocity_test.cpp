#include <ros/ros.h>
#include <signal.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <sunray_msgs/Px4State.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlState.h>

namespace {

struct Point3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

sunray_msgs::Px4State g_px4_state;
sunray_msgs::UAVControlState g_fsm_state;
bool g_has_px4_state = false;
bool g_has_fsm_state = false;

double clamp_abs(double value, double limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void sigint_handler(int) {
    ros::shutdown();
}

void px4_state_callback(const sunray_msgs::Px4State::ConstPtr& msg) {
    g_px4_state = *msg;
    g_has_px4_state = true;
}

void fsm_state_callback(const sunray_msgs::UAVControlState::ConstPtr& msg) {
    g_fsm_state = *msg;
    g_has_fsm_state = true;
}

sunray_msgs::UAVControlCMD make_cmd(uint8_t control_cmd, double fixed_yaw_rad) {
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd_source = sunray_msgs::UAVControlCMD::EXAMPLE_DEMO;
    cmd.control_cmd = control_cmd;
    cmd.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;
    cmd.desired_yaw = fixed_yaw_rad;
    cmd.desired_yaw_rate = 0.0;
    return cmd;
}

void publish_for_duration(ros::Publisher& pub,
                          const sunray_msgs::UAVControlCMD& base_cmd,
                          double duration_s,
                          double rate_hz) {
    ros::Rate rate(rate_hz);
    const ros::Time end_time = ros::Time::now() + ros::Duration(duration_s);
    while (ros::ok() && ros::Time::now() < end_time) {
        sunray_msgs::UAVControlCMD cmd = base_cmd;
        cmd.header.stamp = ros::Time::now();
        pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
}

bool wait_until_fsm_state(uint8_t target_state, double timeout_s, double rate_hz) {
    ros::Rate rate(rate_hz);
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_s);
    while (ros::ok() && ros::Time::now() < deadline) {
        ros::spinOnce();
        if (g_has_fsm_state && g_fsm_state.control_state == target_state) {
            return true;
        }
        rate.sleep();
    }
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "triangle_velocity_test");
    ros::NodeHandle nh("~");
    signal(SIGINT, sigint_handler);

    int uav_id = 1;
    std::string uav_name = "uav";
    double rate_hz = 20.0;
    double ready_hold_s = 3.0;
    double prestart_hover_s = 3.0;
    double post_hover_s = 3.0;
    double triangle_radius = 1.0;
    int num_points_per_edge = 20;
    double k_p_xy = 1.0;
    double k_p_z = 0.5;
    double max_vel_xy = 1.0;
    double max_vel_z = 0.6;
    double pos_tol_xy = 0.15;
    double pos_tol_z = 0.15;
    double fixed_yaw_rad = 0.0;
    double takeoff_wait_timeout_s = 30.0;
    double land_wait_timeout_s = 30.0;

    nh.param("uav_id", uav_id, 1);
    nh.param("uav_name", uav_name, std::string("uav"));
    nh.param("rate_hz", rate_hz, 20.0);
    nh.param("ready_hold_s", ready_hold_s, 3.0);
    nh.param("prestart_hover_s", prestart_hover_s, 3.0);
    nh.param("post_hover_s", post_hover_s, 3.0);
    nh.param("triangle_radius", triangle_radius, 1.0);
    nh.param("num_points_per_edge", num_points_per_edge, 20);
    nh.param("k_p_xy", k_p_xy, 1.0);
    nh.param("k_p_z", k_p_z, 0.5);
    nh.param("max_vel_xy", max_vel_xy, 1.0);
    nh.param("max_vel_z", max_vel_z, 0.6);
    nh.param("pos_tol_xy", pos_tol_xy, 0.15);
    nh.param("pos_tol_z", pos_tol_z, 0.15);
    nh.param("fixed_yaw_rad", fixed_yaw_rad, 0.0);
    nh.param("takeoff_wait_timeout_s", takeoff_wait_timeout_s, 30.0);
    nh.param("land_wait_timeout_s", land_wait_timeout_s, 30.0);

    const std::string uav_ns = "/" + uav_name + std::to_string(uav_id);
    const std::string cmd_topic = uav_ns + "/sunray/uav_control/control_cmd";
    const std::string fsm_topic = uav_ns + "/sunray/uav_control/control_state";
    const std::string px4_state_topic = uav_ns + "/sunray/px4_state";

    ros::Subscriber fsm_sub =
        nh.subscribe<sunray_msgs::UAVControlState>(fsm_topic, 10, fsm_state_callback);
    ros::Subscriber px4_state_sub =
        nh.subscribe<sunray_msgs::Px4State>(px4_state_topic, 10, px4_state_callback);
    ros::Publisher cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(cmd_topic, 10);

    ros::Rate rate(rate_hz);
    ROS_INFO("triangle_velocity_test started for %s", uav_ns.c_str());
    ROS_INFO("waiting for px4 state and fsm state...");

    while (ros::ok() && (!g_has_px4_state || !g_has_fsm_state || !g_px4_state.connected)) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("system connected, waiting for FSM INIT...");
    if (!wait_until_fsm_state(sunray_msgs::UAVControlState::INIT,
                              takeoff_wait_timeout_s,
                              rate_hz)) {
        ROS_ERROR("timeout waiting for FSM INIT");
        return 1;
    }

    ROS_INFO("FSM INIT reached, hold %.1fs before takeoff", ready_hold_s);
    ros::Duration(ready_hold_s).sleep();

    ROS_INFO("sending TAKEOFF");
    publish_for_duration(cmd_pub,
                         make_cmd(sunray_msgs::UAVControlCMD::TAKEOFF, fixed_yaw_rad),
                         0.8,
                         rate_hz);

    ROS_INFO("waiting for FSM HOVER after takeoff...");
    if (!wait_until_fsm_state(sunray_msgs::UAVControlState::HOVER,
                              takeoff_wait_timeout_s,
                              rate_hz)) {
        ROS_ERROR("timeout waiting for FSM HOVER after takeoff");
        return 1;
    }

    ROS_INFO("hover settled, wait %.1fs before triangle", prestart_hover_s);
    ros::Duration(prestart_hover_s).sleep();
    ros::spinOnce();

    const double center_x = g_px4_state.local_pose.position.x;
    const double center_y = g_px4_state.local_pose.position.y;
    const double height = g_px4_state.local_pose.position.z;

    std::vector<Point3> vertices = {
        {center_x + triangle_radius, center_y, height},
        {center_x - triangle_radius * std::cos(M_PI / 3.0),
         center_y + triangle_radius * std::sin(M_PI / 3.0),
         height},
        {center_x - triangle_radius * std::cos(M_PI / 3.0),
         center_y - triangle_radius * std::sin(M_PI / 3.0),
         height},
    };

    ROS_INFO("triangle start: center=(%.2f, %.2f, %.2f), radius=%.2f",
             center_x,
             center_y,
             height,
             triangle_radius);

    for (int edge = 0; ros::ok() && edge < 3; ++edge) {
        const Point3& start = vertices[edge];
        const Point3& end = vertices[(edge + 1) % 3];

        for (int i = 0; ros::ok() && i <= num_points_per_edge; ++i) {
            const double alpha = static_cast<double>(i) /
                                 static_cast<double>(std::max(1, num_points_per_edge));
            const Point3 target{
                start.x + alpha * (end.x - start.x),
                start.y + alpha * (end.y - start.y),
                start.z + alpha * (end.z - start.z),
            };

            while (ros::ok()) {
                ros::spinOnce();

                const double x = g_px4_state.local_pose.position.x;
                const double y = g_px4_state.local_pose.position.y;
                const double z = g_px4_state.local_pose.position.z;

                const double dx = target.x - x;
                const double dy = target.y - y;
                const double dz = target.z - z;

                sunray_msgs::UAVControlCMD cmd =
                    make_cmd(sunray_msgs::UAVControlCMD::MOVE_VELOCITY, fixed_yaw_rad);
                cmd.desired_vel.x = clamp_abs(k_p_xy * dx, max_vel_xy);
                cmd.desired_vel.y = clamp_abs(k_p_xy * dy, max_vel_xy);
                cmd.desired_vel.z = clamp_abs(k_p_z * dz, max_vel_z);
                cmd_pub.publish(cmd);

                const bool reached_xy = std::fabs(dx) < pos_tol_xy && std::fabs(dy) < pos_tol_xy;
                const bool reached_z = std::fabs(dz) < pos_tol_z;
                if (reached_xy && reached_z) {
                    break;
                }

                rate.sleep();
            }
        }
    }

    ROS_INFO("triangle finished, sending HOVER");
    publish_for_duration(cmd_pub,
                         make_cmd(sunray_msgs::UAVControlCMD::HOVER, fixed_yaw_rad),
                         post_hover_s,
                         rate_hz);

    ROS_INFO("sending LAND");
    publish_for_duration(
        cmd_pub, make_cmd(sunray_msgs::UAVControlCMD::LAND, fixed_yaw_rad), 0.8, rate_hz);

    ROS_INFO("waiting for FSM INIT after landing...");
    if (!wait_until_fsm_state(
            sunray_msgs::UAVControlState::INIT, land_wait_timeout_s, rate_hz)) {
        ROS_WARN("landing completion wait timeout");
        return 1;
    }

    ROS_INFO("triangle_velocity_test finished");
    return 0;
}
