#include <gtest/gtest.h>

#include "px4_bridge/px4_param_types.h"

TEST(Px4ParamTypesTest, Ekf2EvCtrlMaskOps) {
  px4_param_types::EKF2_EV_CTRL ctrl;
  EXPECT_EQ(ctrl.toInt(), 0);

  ctrl.enable_Horizontalposition();
  ctrl.enable_Yaw();
  EXPECT_TRUE(ctrl.has_Horizontalposition());
  EXPECT_TRUE(ctrl.has_Yaw());
  EXPECT_FALSE(ctrl.has_Velocity());
  EXPECT_EQ(ctrl.toInt(), (1 << 0) | (1 << 3));

  ctrl.disable_Horizontalposition();
  EXPECT_FALSE(ctrl.has_Horizontalposition());
  EXPECT_EQ(ctrl.toInt(), (1 << 3));
}

TEST(Px4ParamTypesTest, Ekf2HgtRefSingleChoice) {
  px4_param_types::EKF2_HGT_REF ref;
  EXPECT_TRUE(ref.is_Baro());
  EXPECT_EQ(ref.toInt(), 0);

  ref.set_Gnss();
  EXPECT_TRUE(ref.is_Gnss());
  EXPECT_EQ(ref.toInt(), 1);

  ref.setRaw(3);
  EXPECT_TRUE(ref.is_Vision());
  EXPECT_EQ(ref.toInt(), 3);
}

TEST(Px4ParamTypesTest, FloatParamTypeBasics) {
  px4_param_types::EKF2_EV_DELAY ev_delay;
  ev_delay.set_ms(12.5f);
  EXPECT_FLOAT_EQ(ev_delay.milliseconds(), 12.5f);
  EXPECT_FLOAT_EQ(ev_delay.toFloat(), 12.5f);

  px4_param_types::EKF2_GPS_DELAY gps_delay;
  gps_delay.set_ms(110.0f);
  EXPECT_FLOAT_EQ(gps_delay.milliseconds(), 110.0f);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

