#include <gtest/gtest.h>

#include "px4_bridge/px4_data_reader.h"

TEST(Px4DataReaderTest, ReaderOptionsEnableDisableAll) {
  ReaderOptions options;
  options.enable_all();

  EXPECT_TRUE(options.read_system_state);
  EXPECT_TRUE(options.read_ekf2_state);
  EXPECT_TRUE(options.read_flow_state);
  EXPECT_TRUE(options.read_local_pose);
  EXPECT_TRUE(options.read_local_velocity);
  EXPECT_TRUE(options.read_body_pose);
  EXPECT_TRUE(options.read_body_velocity);

  options.disable_all();
  EXPECT_FALSE(options.read_system_state);
  EXPECT_FALSE(options.read_ekf2_state);
  EXPECT_FALSE(options.read_flow_state);
  EXPECT_FALSE(options.read_local_pose);
  EXPECT_FALSE(options.read_local_velocity);
  EXPECT_FALSE(options.read_body_pose);
  EXPECT_FALSE(options.read_body_velocity);
}

TEST(Px4DataReaderTest, FlightModeFromStringMapping) {
  EXPECT_EQ(PX4_DataReader::flight_mode_from_string("OFFBOARD"),
            px4_data_types::FlightMode::kOffboard);
  EXPECT_EQ(PX4_DataReader::flight_mode_from_string("POSCTL"),
            px4_data_types::FlightMode::kPosctl);
  EXPECT_EQ(PX4_DataReader::flight_mode_from_string("AUTO.LAND"),
            px4_data_types::FlightMode::kAutoLand);
  EXPECT_EQ(PX4_DataReader::flight_mode_from_string("UNKNOWN_MODE"),
            px4_data_types::FlightMode::kUndefined);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

