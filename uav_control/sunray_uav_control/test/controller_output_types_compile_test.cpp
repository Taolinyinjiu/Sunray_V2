/**
 * @file controller_output_types_compile_test.cpp
 * @brief 
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-16
 * @version 0.1
 * 
 */
#include <gtest/gtest.h>

#include "control_data_types/controller_desired_types.hpp"
namespace controller_data_types {
namespace {

TEST(ControllerDesiredTypesTest, TargetTrajectoryDefaultsAreZeroInitialized) {
  TargetTrajectoryPoint_t point;

  EXPECT_TRUE(point.position.isZero());
  EXPECT_TRUE(point.velocity.isZero());
  EXPECT_TRUE(point.acceleration.isZero());
  EXPECT_TRUE(point.jerk.isZero());
  EXPECT_DOUBLE_EQ(point.yaw, 0.0);
  EXPECT_DOUBLE_EQ(point.yaw_rate, 0.0);
}

}  // namespace
}  // namespace controller_data_types

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
