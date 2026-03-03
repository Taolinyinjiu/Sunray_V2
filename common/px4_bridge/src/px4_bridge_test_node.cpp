#include "mavros_msgs/ParamGet.h"
#include "px4_bridge/px4_data_reader.h"
#include "ros/ros.h"

#include <cmath>
#include <cstdint>
#include <string>

namespace {

bool get_param_int(ros::ServiceClient &client, const std::string &name, int &out) {
  mavros_msgs::ParamGet srv;
  srv.request.param_id = name;
  if (!client.call(srv) || !srv.response.success) {
    return false;
  }
  out = static_cast<int>(srv.response.value.integer);
  return true;
}

bool get_param_float(ros::ServiceClient &client, const std::string &name, float &out) {
  mavros_msgs::ParamGet srv;
  srv.request.param_id = name;
  if (!client.call(srv) || !srv.response.success) {
    return false;
  }
  out = static_cast<float>(srv.response.value.real);
  return true;
}

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "px4_bridge_test_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int uav_id = 1;
  std::string uav_name = "uav";
  double test_hz = 1.0;
  pnh.param("uav_id", uav_id, uav_id);
  pnh.param("uav_name", uav_name, uav_name);
  pnh.param("test_hz", test_hz, test_hz);

  nh.setParam("uav_id", uav_id);
  nh.setParam("uav_name", uav_name);

  const std::string mavros_ns = "/" + uav_name + std::to_string(uav_id) + "/mavros";
  ros::ServiceClient param_get_client =
      nh.serviceClient<mavros_msgs::ParamGet>(mavros_ns + "/param/get");
  if (!param_get_client.waitForExistence(ros::Duration(3.0))) {
    ROS_ERROR("ParamGet service unavailable: %s/param/get", mavros_ns.c_str());
    return 1;
  }

  ReaderOptions options;
  options.disable_all();
  PX4_DataReader reader(nh, options);

  ROS_INFO("px4_bridge_test_node started at %.2f Hz, target namespace=%s",
           test_hz, mavros_ns.c_str());

  ros::Rate rate(test_hz);
  const float kFloatTol = 1e-3f;
  while (ros::ok()) {
    const auto hgt = reader.fetch_ekf2_hgt_ref();
    const auto ev_ctrl = reader.fetch_ekf2_ev_ctrl();
    const auto ev_delay = reader.fetch_ekf2_ev_delay();
    const auto gps_delay = reader.fetch_ekf2_gps_delay();

    int hgt_raw = 0;
    int ev_ctrl_raw = 0;
    float ev_delay_raw = 0.0f;
    float gps_delay_raw = 0.0f;

    bool ok = true;
    ok &= get_param_int(param_get_client, "EKF2_HGT_REF", hgt_raw);
    ok &= get_param_int(param_get_client, "EKF2_EV_CTRL", ev_ctrl_raw);
    ok &= get_param_float(param_get_client, "EKF2_EV_DELAY", ev_delay_raw);
    ok &= get_param_float(param_get_client, "EKF2_GPS_DELAY", gps_delay_raw);

    if (!ok) {
      ROS_WARN_THROTTLE(2.0, "[bridge-test] Failed to fetch one or more params from MAVROS.");
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    const bool match_hgt = (hgt.toInt() == hgt_raw);
    const bool match_ev_ctrl = (ev_ctrl.toInt() == ev_ctrl_raw);
    const bool match_ev_delay = (std::fabs(ev_delay.toFloat() - ev_delay_raw) <= kFloatTol);
    const bool match_gps_delay =
        (std::fabs(gps_delay.toFloat() - gps_delay_raw) <= kFloatTol);

    const bool all_match = match_hgt && match_ev_ctrl && match_ev_delay && match_gps_delay;
    if (all_match) {
      ROS_INFO_THROTTLE(2.0,
                        "[bridge-test][PASS] HGT_REF=%d EV_CTRL=%d EV_DELAY=%.3f GPS_DELAY=%.3f",
                        hgt.toInt(), ev_ctrl.toInt(), ev_delay.toFloat(),
                        gps_delay.toFloat());
    } else {
      ROS_ERROR_THROTTLE(
          2.0,
          "[bridge-test][FAIL] reader(hgt=%d,ev_ctrl=%d,ev_delay=%.3f,gps_delay=%.3f) "
          "!= mavros(hgt=%d,ev_ctrl=%d,ev_delay=%.3f,gps_delay=%.3f)",
          hgt.toInt(), ev_ctrl.toInt(), ev_delay.toFloat(), gps_delay.toFloat(), hgt_raw,
          ev_ctrl_raw, ev_delay_raw, gps_delay_raw);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

