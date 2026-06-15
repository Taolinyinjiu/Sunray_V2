#include <gtest/gtest.h>

#include "utils/body_frame_reference_helper.hpp"

namespace {

TEST(BodyFrameReferenceHelperTest, BodyPointYawIsAbsolute) {
  control_common::UAVStateEstimate odom;
  odom.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  odom.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));

  controller_data_types::TargetBodyPoint_t body_point;
  body_point.position_xy = Eigen::Vector2d(2.0, 0.0);
  body_point.fixed_height = 4.0;
  body_point.yaw = 0.25;

  const auto world_point = body_frame_reference_helper::to_world_point(odom, body_point);
  EXPECT_NEAR(world_point.position.x(), 1.0 + 2.0 * std::cos(1.0), 1e-9);
  EXPECT_NEAR(world_point.position.y(), 2.0 + 2.0 * std::sin(1.0), 1e-9);
  EXPECT_DOUBLE_EQ(world_point.position.z(), 4.0);
  EXPECT_DOUBLE_EQ(world_point.yaw, 0.25);
}

TEST(BodyFrameReferenceHelperTest, BodyVelocityYawIsPassedThrough) {
  control_common::UAVStateEstimate odom;
  odom.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitZ()));

  controller_data_types::TargetBodyVelocity_t body_velocity;
  body_velocity.velocity_xy = Eigen::Vector2d(1.0, 2.0);
  body_velocity.fixed_height = 5.0;
  body_velocity.yaw = -0.75;
  body_velocity.yaw_rate = 0.3;

  const auto world_velocity =
      body_frame_reference_helper::to_world_velocity(odom, body_velocity);
  EXPECT_DOUBLE_EQ(world_velocity.yaw, -0.75);
  EXPECT_DOUBLE_EQ(world_velocity.yaw_rate, 0.3);
  EXPECT_DOUBLE_EQ(world_velocity.fixed_height, 5.0);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
