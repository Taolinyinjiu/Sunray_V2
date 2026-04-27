#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>

#include "sunray_ugv_control/ugv_controller.h"

namespace sunray_ugv_control {
namespace {

UGVControllerConfig make_velocity_config() {
    UGVControllerConfig config;
    config.ugv_type = sunray_msgs::UGVControllerState::MECANUM;
    config.state_pub_frequency = 10.0;
    config.vel_x = PIDGains{1.0, 0.0, 0.0};
    config.vel_y = PIDGains{1.0, 0.0, 0.0};
    config.vel_yaw = PIDGains{1.0, 0.0, 0.0};
    config.max_linear_x = 10.0;
    config.max_linear_y = 10.0;
    config.max_angular_z = 10.0;
    return config;
}

TEST(UGVControllerTest, VelocityControlKeepsDesiredVelocityWhenTrackingErrorIsZero) {
    ros::NodeHandle nh;
    UGVController controller(nh, make_velocity_config(), "/ugv_controller_test/state");

    UGVKinematicState state;
    state.velocity.x = 1.2;
    state.velocity.y = -0.4;
    state.yaw = 0.0;
    controller.set_current_state(state);

    sunray_msgs::UGVControlCMD cmd;
    cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_VELOCITY;
    cmd.desired_vel.x = 1.2;
    cmd.desired_vel.y = -0.4;
    cmd.desired_yaw = 0.0;

    const geometry_msgs::Twist output = controller.move_velocity(cmd);

    EXPECT_NEAR(output.linear.x, 1.2, 1e-9);
    EXPECT_NEAR(output.linear.y, -0.4, 1e-9);
    EXPECT_NEAR(output.angular.z, 0.0, 1e-9);
}

}  // namespace
}  // namespace sunray_ugv_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "ugv_controller_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
