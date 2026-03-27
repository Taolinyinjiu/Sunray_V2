/**
 * @file controller_output_types_compile_test.cpp
 * @brief 
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-16
 * @version 0.1
 * 
 */
#include <gtest/gtest.h>

#include "controller_data_types/controller_output_types.hpp"

namespace controller_data_types {
namespace {

TEST(ControllerOutputTypesTest, LocalSetpointDefaultsAreUsable) {
  LocalSetpointOutput output;

  EXPECT_EQ(output.coordinate_frame, LocalSetpointFrame::kLocalNed);
  EXPECT_EQ(output.type_mask, 0u);
  EXPECT_TRUE(output.position.isZero());
  EXPECT_TRUE(output.velocity.isZero());
  EXPECT_TRUE(output.acceleration_or_force.isZero());
  EXPECT_DOUBLE_EQ(output.yaw, 0.0);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0.0);
}

TEST(ControllerOutputTypesTest, AttitudeSetpointDefaultsAreUsable) {
  AttitudeSetpointOutput output;

  EXPECT_EQ(output.type_mask, 0u);
  EXPECT_TRUE(output.attitude.coeffs().isApprox(
      Eigen::Quaterniond::Identity().coeffs()));
  EXPECT_TRUE(output.body_rate.isZero());
  EXPECT_DOUBLE_EQ(output.thrust, 0.0);
}

}  // namespace
}  // namespace controller_data_types

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
