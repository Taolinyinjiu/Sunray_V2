#include <gtest/gtest.h>

#include "mavros_msgs/ParamGet.h"
#include "px4_bridge/px4_data_reader.h"
#include "ros/ros.h"

#include <map>
#include <string>

namespace {

bool fake_param_get_cb(mavros_msgs::ParamGet::Request &req,
                       mavros_msgs::ParamGet::Response &res) {
  static const std::map<std::string, int> int_params = {
      {"EKF2_HGT_REF", 1},   {"EKF2_EV_CTRL", 9},   {"EKF2_MAG_TYPE", 5},
      {"EKF2_MAG_CHECK", 3}, {"EKF2_GPS_CTRL", 7},  {"EKF2_GPS_CHECK", 11},
  };
  static const std::map<std::string, float> float_params = {
      {"EKF2_EV_DELAY", 12.5f},
      {"EKF2_GPS_DELAY", 110.0f},
      {"MC_ROLLRATE_P", 0.10f},
      {"MC_ROLLRATE_I", 0.20f},
      {"MC_ROLLRATE_D", 0.30f},
      {"MC_PITCHRATE_P", 0.40f},
      {"MC_PITCHRATE_I", 0.50f},
      {"MC_PITCHRATE_D", 0.60f},
      {"MC_YAWRATE_P", 0.70f},
      {"MC_YAWRATE_I", 0.80f},
      {"MC_YAWRATE_D", 0.90f},
      {"MPC_XY_VEL_P_ACC", 1.10f},
      {"MPC_XY_VEL_I_ACC", 1.20f},
      {"MPC_XY_VEL_D_ACC", 1.30f},
      {"MPC_Z_VEL_P_ACC", 2.10f},
      {"MPC_Z_VEL_I_ACC", 2.20f},
      {"MPC_Z_VEL_D_ACC", 2.30f},
      {"MPC_XY_P", 3.10f},
      {"MPC_Z_P", 3.20f},
  };

  auto it_int = int_params.find(req.param_id);
  if (it_int != int_params.end()) {
    res.success = true;
    res.value.integer = it_int->second;
    res.value.real = static_cast<float>(it_int->second);
    return true;
  }

  auto it_float = float_params.find(req.param_id);
  if (it_float != float_params.end()) {
    res.success = true;
    res.value.real = it_float->second;
    res.value.integer = static_cast<int64_t>(it_float->second);
    return true;
  }

  res.success = false;
  res.value.integer = 0;
  res.value.real = 0.0f;
  return true;
}

} // namespace

TEST(Px4DataReaderIntegrationTest, FetchEkf2ParamsFromService) {
  ros::NodeHandle nh;
  nh.setParam("uav_id", 1);
  nh.setParam("uav_name", std::string("uav"));

  ros::ServiceServer srv =
      nh.advertiseService("/uav1/mavros/param/get", fake_param_get_cb);
  ASSERT_TRUE(static_cast<bool>(srv));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ReaderOptions options;
  options.disable_all();
  PX4_DataReader reader(nh, options);

  auto hgt = reader.fetch_ekf2_hgt_ref();
  auto ev_ctrl = reader.fetch_ekf2_ev_ctrl();
  auto ev_delay = reader.fetch_ekf2_ev_delay();
  auto mag_type = reader.fetch_ekf2_mag_type();
  auto mag_check = reader.fetch_ekf2_mag_check();
  auto gps_ctrl = reader.fetch_ekf2_gps_ctrl();
  auto gps_check = reader.fetch_ekf2_gps_check();
  auto gps_delay = reader.fetch_ekf2_gps_delay();

  EXPECT_EQ(hgt.toInt(), 1);
  EXPECT_EQ(ev_ctrl.toInt(), 9);
  EXPECT_FLOAT_EQ(ev_delay.toFloat(), 12.5f);
  EXPECT_EQ(mag_type.toInt(), 5);
  EXPECT_EQ(mag_check.toInt(), 3);
  EXPECT_EQ(gps_ctrl.toInt(), 7);
  EXPECT_EQ(gps_check.toInt(), 11);
  EXPECT_FLOAT_EQ(gps_delay.toFloat(), 110.0f);
}

TEST(Px4DataReaderIntegrationTest, FetchControllerParamsFromService) {
  ros::NodeHandle nh;
  nh.setParam("uav_id", 1);
  nh.setParam("uav_name", std::string("uav"));

  ros::ServiceServer srv =
      nh.advertiseService("/uav1/mavros/param/get", fake_param_get_cb);
  ASSERT_TRUE(static_cast<bool>(srv));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ReaderOptions options;
  options.disable_all();
  PX4_DataReader reader(nh, options);

  auto rate = reader.fetch_px4_rate_pid();
  auto vel = reader.fetch_px4_velocity_pid();
  auto pos = reader.fetch_px4_position_p();

  EXPECT_FLOAT_EQ(rate.roll.p.value, 0.10f);
  EXPECT_FLOAT_EQ(rate.pitch.i.value, 0.50f);
  EXPECT_FLOAT_EQ(rate.yaw.d.value, 0.90f);

  EXPECT_FLOAT_EQ(vel.xy.p_acc.value, 1.10f);
  EXPECT_FLOAT_EQ(vel.z.i_acc.value, 2.20f);

  EXPECT_FLOAT_EQ(pos.xy_p.value, 3.10f);
  EXPECT_FLOAT_EQ(pos.z_p.value, 3.20f);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "px4_data_reader_param_fetch_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

