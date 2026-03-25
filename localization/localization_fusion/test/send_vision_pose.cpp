#include <ros/ros.h>

#include <boost/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <nav_msgs/Odometry.h>
#include <sunray_msgs/OdomStatus.h>

#include <algorithm>
#include <mutex>
#include <string>

namespace {

struct RuntimeConfig {
  std::string uav_ns{};
  std::string mavros_ns{"/mavros"};
  std::string local_odom_topic{"${uav_ns}/sunray/localization/local_odom"};
  std::string odom_status_topic{"${uav_ns}/sunray/localization/odom_status"};
  double publish_rate_hz{100.0};
  double init_wait_timeout_s{5.0};
  double param_retry_interval_s{1.0};
  bool auto_set_ekf2_ev_ctrl{false};
  bool exit_on_param_error{true};
};

struct SharedState {
  std::mutex mutex;

  bool has_local_odom{false};
  nav_msgs::Odometry latest_local_odom{};

  bool has_odom_status{false};
  sunray_msgs::OdomStatus latest_odom_status{};

  int checked_source_id{-1};
  bool param_check_ok{false};
  int last_param_check_source_id{-1};
  ros::Time last_param_check_attempt{};
};

struct SourceRule {
  int source_id{-1};
  const char *source_name{""};
  int expected_ekf2_ev_ctrl{15};
};

enum class ParamCheckResult : uint8_t {
  kReady = 0,
  kRetryLater = 1,
  kFailed = 2,
};

std::string trim_leading_slash(const std::string &value) {
  if (!value.empty() && value.front() == '/') {
    return value.substr(1);
  }
  return value;
}

std::string trim_trailing_slash(const std::string &value) {
  if (!value.empty() && value.back() == '/') {
    return value.substr(0, value.size() - 1);
  }
  return value;
}

std::string ensure_leading_slash(const std::string &value) {
  if (value.empty()) {
    return value;
  }
  if (value.front() == '/') {
    return value;
  }
  return "/" + value;
}

std::string replace_uav_ns(const std::string &value, const std::string &uav_ns) {
  const std::string key = "${uav_ns}";
  const std::size_t pos = value.find(key);
  if (pos == std::string::npos) {
    return ensure_leading_slash(value);
  }

  std::string replaced = value;
  const std::string ns = uav_ns.empty() ? "" : ensure_leading_slash(uav_ns);
  replaced.replace(pos, key.size(), ns);
  return ensure_leading_slash(replaced);
}

std::string resolve_uav_namespace(ros::NodeHandle &nh) {
  std::string key;
  std::string uav_ns;
  if (nh.searchParam("uav_ns", key) && nh.getParam(key, uav_ns) &&
      !uav_ns.empty()) {
    return trim_leading_slash(trim_trailing_slash(uav_ns));
  }

  std::string uav_name;
  int uav_id = 0;
  bool ok_name = false;
  bool ok_id = false;
  if (nh.searchParam("uav_name", key)) {
    ok_name = nh.getParam(key, uav_name) && !uav_name.empty();
  }
  if (nh.searchParam("uav_id", key)) {
    ok_id = nh.getParam(key, uav_id);
  }
  if (ok_name && ok_id) {
    return trim_leading_slash(uav_name + std::to_string(uav_id));
  }

  return "";
}

bool lookup_source_rule(const int source_id, SourceRule *rule) {
  if (rule == nullptr) {
    return false;
  }

  switch (source_id) {
  case sunray_msgs::OdomStatus::VIOBOT:
    *rule = SourceRule{source_id, "VIOBOT", 15};
    return true;
  case sunray_msgs::OdomStatus::MOCAP:
    *rule = SourceRule{source_id, "MOCAP", 15};
    return true;
  case sunray_msgs::OdomStatus::VINS:
    *rule = SourceRule{source_id, "VINS", 15};
    return true;
  case sunray_msgs::OdomStatus::GAZEBO:
    *rule = SourceRule{source_id, "GAZEBO", 15};
    return true;
  case sunray_msgs::OdomStatus::GAZEBO_ARUCO:
    *rule = SourceRule{source_id, "GAZEBO_ARUCO", 15};
    return true;
  default:
    return false;
  }
}

bool read_param_int(ros::ServiceClient *client, const std::string &name,
                    int *value) {
  if (client == nullptr || value == nullptr) {
    return false;
  }
  mavros_msgs::ParamGet srv;
  srv.request.param_id = name;
  if (!client->call(srv) || !srv.response.success) {
    return false;
  }
  *value = static_cast<int>(srv.response.value.integer);
  return true;
}

bool set_param_int(ros::ServiceClient *client, const std::string &name,
                   const int value) {
  if (client == nullptr) {
    return false;
  }
  mavros_msgs::ParamSet srv;
  srv.request.param_id = name;
  srv.request.value.integer = value;
  if (!client->call(srv) || !srv.response.success) {
    return false;
  }
  return true;
}

bool wait_for_ready(const ros::Duration &timeout, SharedState *state) {
  if (state == nullptr) {
    return false;
  }

  const ros::Time start = ros::Time::now();
  ros::Rate rate(50.0);
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->has_local_odom && state->has_odom_status) {
        return true;
      }
    }

    if ((ros::Time::now() - start) >= timeout) {
      return false;
    }

    ros::spinOnce();
    rate.sleep();
  }
  return false;
}

ParamCheckResult ensure_source_param_ready(const RuntimeConfig *config,
                                          SharedState *state,
                                          ros::ServiceClient *param_get_client,
                                          ros::ServiceClient *param_set_client,
                                          const bool force_check) {
  if (config == nullptr || state == nullptr || param_get_client == nullptr ||
      param_set_client == nullptr) {
    return ParamCheckResult::kFailed;
  }

  int source_id = -1;
  std::string source_name;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (!state->has_odom_status) {
      return ParamCheckResult::kRetryLater;
    }
    source_id = static_cast<int>(state->latest_odom_status.external_source_id);
    source_name = state->latest_odom_status.external_source_name;
    if (state->param_check_ok && state->checked_source_id == source_id) {
      return ParamCheckResult::kReady;
    }
  }

  const ros::Time now = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (!force_check && state->last_param_check_source_id == source_id &&
        !state->last_param_check_attempt.isZero() &&
        (now - state->last_param_check_attempt).toSec() <
            config->param_retry_interval_s) {
      return ParamCheckResult::kRetryLater;
    }
    state->last_param_check_source_id = source_id;
    state->last_param_check_attempt = now;
  }

  SourceRule rule;
  if (!lookup_source_rule(source_id, &rule)) {
    ROS_ERROR("[send_vision_pose] unknown external_source_id=%d name='%s'",
              source_id, source_name.c_str());
    std::lock_guard<std::mutex> lock(state->mutex);
    state->param_check_ok = false;
    return ParamCheckResult::kFailed;
  }

  const std::string get_service = config->mavros_ns + "/param/get";
  if (!ros::service::waitForService(get_service, ros::Duration(2.0))) {
    ROS_ERROR("[send_vision_pose] service not available: %s",
              get_service.c_str());
    std::lock_guard<std::mutex> lock(state->mutex);
    state->param_check_ok = false;
    return ParamCheckResult::kFailed;
  }

  int actual_ev_ctrl = 0;
  if (!read_param_int(param_get_client, "EKF2_EV_CTRL", &actual_ev_ctrl)) {
    ROS_ERROR("[send_vision_pose] read PX4 param failed: EKF2_EV_CTRL");
    std::lock_guard<std::mutex> lock(state->mutex);
    state->param_check_ok = false;
    return ParamCheckResult::kFailed;
  }

  if (actual_ev_ctrl != rule.expected_ekf2_ev_ctrl) {
    ROS_WARN("[send_vision_pose] EKF2_EV_CTRL mismatch for source=%s(%d): "
             "expected=%d actual=%d",
             rule.source_name, rule.source_id, rule.expected_ekf2_ev_ctrl,
             actual_ev_ctrl);

    if (!config->auto_set_ekf2_ev_ctrl) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->param_check_ok = false;
      return ParamCheckResult::kFailed;
    }

    const std::string set_service = config->mavros_ns + "/param/set";
    if (!ros::service::waitForService(set_service, ros::Duration(2.0))) {
      ROS_ERROR("[send_vision_pose] service not available: %s",
                set_service.c_str());
      std::lock_guard<std::mutex> lock(state->mutex);
      state->param_check_ok = false;
      return ParamCheckResult::kFailed;
    }

    if (!set_param_int(param_set_client, "EKF2_EV_CTRL",
                       rule.expected_ekf2_ev_ctrl)) {
      ROS_ERROR("[send_vision_pose] set PX4 param failed: EKF2_EV_CTRL");
      std::lock_guard<std::mutex> lock(state->mutex);
      state->param_check_ok = false;
      return ParamCheckResult::kFailed;
    }

    int verify_ev_ctrl = 0;
    if (!read_param_int(param_get_client, "EKF2_EV_CTRL", &verify_ev_ctrl) ||
        verify_ev_ctrl != rule.expected_ekf2_ev_ctrl) {
      ROS_ERROR("[send_vision_pose] verify PX4 param failed: EKF2_EV_CTRL "
                "expected=%d actual=%d",
                rule.expected_ekf2_ev_ctrl, verify_ev_ctrl);
      std::lock_guard<std::mutex> lock(state->mutex);
      state->param_check_ok = false;
      return ParamCheckResult::kFailed;
    }

    ROS_INFO("[send_vision_pose] set PX4 param ok: EKF2_EV_CTRL=%d",
             rule.expected_ekf2_ev_ctrl);
  }

  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->checked_source_id = source_id;
    state->param_check_ok = true;
  }

  ROS_INFO("[send_vision_pose] source ready: %s(%d), EKF2_EV_CTRL=%d",
           rule.source_name, rule.source_id, rule.expected_ekf2_ev_ctrl);
  return ParamCheckResult::kReady;
}

void local_odom_callback(const nav_msgs::OdometryConstPtr &msg,
                         SharedState *state) {
  if (msg == nullptr || state == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(state->mutex);
  state->latest_local_odom = *msg;
  state->has_local_odom = true;
}

void odom_status_callback(const sunray_msgs::OdomStatusConstPtr &msg,
                          SharedState *state) {
  if (msg == nullptr || state == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(state->mutex);
  state->latest_odom_status = *msg;
  state->has_odom_status = true;
  if (state->checked_source_id !=
      static_cast<int>(state->latest_odom_status.external_source_id)) {
    state->param_check_ok = false;
  }
}

void publish_timer_callback(const ros::TimerEvent & /*event*/,
                            const RuntimeConfig *config, SharedState *state,
                            ros::Publisher *vision_pose_pub,
                            ros::ServiceClient *param_get_client,
                            ros::ServiceClient *param_set_client) {
  if (config == nullptr || state == nullptr || vision_pose_pub == nullptr) {
    return;
  }

  const ParamCheckResult param_result =
      ensure_source_param_ready(config, state, param_get_client,
                                param_set_client, false);
  if (param_result != ParamCheckResult::kReady) {
    if (param_result == ParamCheckResult::kFailed &&
        config->exit_on_param_error) {
      ROS_ERROR_THROTTLE(1.0,
                         "[send_vision_pose] PX4 param check failed, shutting "
                         "down node.");
      ros::shutdown();
    }
    return;
  }

  nav_msgs::Odometry latest_local_odom;
  sunray_msgs::OdomStatus latest_odom_status;
  bool has_local_odom = false;
  bool has_odom_status = false;
  bool param_check_ok = false;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    latest_local_odom = state->latest_local_odom;
    latest_odom_status = state->latest_odom_status;
    has_local_odom = state->has_local_odom;
    has_odom_status = state->has_odom_status;
    param_check_ok = state->param_check_ok;
  }

  if (!has_local_odom || !has_odom_status || !param_check_ok) {
    return;
  }
  if (!latest_odom_status.has_odometry) {
    ROS_WARN_THROTTLE(1.0,
                      "[send_vision_pose] skip publish: has_odometry=false");
    return;
  }
  if (latest_odom_status.odom_timeout) {
    ROS_WARN_THROTTLE(1.0,
                      "[send_vision_pose] skip publish: odom_timeout=true");
    return;
  }

  geometry_msgs::PoseStamped vision_pose_msg;
  vision_pose_msg.header = latest_local_odom.header;
  if (vision_pose_msg.header.stamp.isZero()) {
    vision_pose_msg.header.stamp = ros::Time::now();
  }
  if (vision_pose_msg.header.frame_id.empty() &&
      !latest_odom_status.local_frame_id.empty()) {
    vision_pose_msg.header.frame_id = latest_odom_status.local_frame_id;
  }
  vision_pose_msg.pose = latest_local_odom.pose.pose;
  vision_pose_pub->publish(vision_pose_msg);
}

RuntimeConfig load_runtime_config(ros::NodeHandle &nh,
                                  ros::NodeHandle &private_nh) {
  RuntimeConfig config;
  config.uav_ns = resolve_uav_namespace(nh);

  private_nh.param<std::string>("local_odom_topic", config.local_odom_topic,
                                config.local_odom_topic);
  private_nh.param<std::string>("odom_status_topic", config.odom_status_topic,
                                config.odom_status_topic);
  private_nh.param<double>("publish_rate_hz", config.publish_rate_hz,
                           config.publish_rate_hz);
  private_nh.param<double>("init_wait_timeout_s", config.init_wait_timeout_s,
                           config.init_wait_timeout_s);
  private_nh.param<double>("param_retry_interval_s",
                           config.param_retry_interval_s,
                           config.param_retry_interval_s);
  private_nh.param<bool>("auto_set_ekf2_ev_ctrl", config.auto_set_ekf2_ev_ctrl,
                         config.auto_set_ekf2_ev_ctrl);
  private_nh.param<bool>("exit_on_param_error", config.exit_on_param_error,
                         config.exit_on_param_error);

  std::string mavros_ns_param = "__AUTO__";
  private_nh.param<std::string>("mavros_ns", mavros_ns_param, mavros_ns_param);

  config.publish_rate_hz = std::max(1.0, config.publish_rate_hz);
  config.init_wait_timeout_s = std::max(0.1, config.init_wait_timeout_s);
  config.param_retry_interval_s = std::max(0.1, config.param_retry_interval_s);

  config.local_odom_topic =
      replace_uav_ns(config.local_odom_topic, config.uav_ns);
  config.odom_status_topic =
      replace_uav_ns(config.odom_status_topic, config.uav_ns);

  mavros_ns_param = trim_trailing_slash(mavros_ns_param);
  if (!mavros_ns_param.empty() && mavros_ns_param != "__AUTO__") {
    config.mavros_ns = ensure_leading_slash(mavros_ns_param);
  } else if (!config.uav_ns.empty()) {
    config.mavros_ns = "/" + trim_trailing_slash(config.uav_ns) + "/mavros";
  } else {
    config.mavros_ns = "/mavros";
  }

  return config;
}

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "send_vision_pose");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  RuntimeConfig config = load_runtime_config(nh, private_nh);
  SharedState state;

  ros::Publisher vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
      config.mavros_ns + "/vision_pose/pose", 20);

  ros::ServiceClient param_get_client =
      nh.serviceClient<mavros_msgs::ParamGet>(config.mavros_ns + "/param/get");
  ros::ServiceClient param_set_client =
      nh.serviceClient<mavros_msgs::ParamSet>(config.mavros_ns + "/param/set");

  ros::Subscriber local_odom_sub = nh.subscribe<nav_msgs::Odometry>(
      config.local_odom_topic, 50, boost::bind(&local_odom_callback, _1, &state));
  ros::Subscriber odom_status_sub = nh.subscribe<sunray_msgs::OdomStatus>(
      config.odom_status_topic, 10,
      boost::bind(&odom_status_callback, _1, &state));

  ROS_INFO("[send_vision_pose] wait first messages: local_odom='%s' "
           "odom_status='%s' mavros_ns='%s'",
           config.local_odom_topic.c_str(), config.odom_status_topic.c_str(),
           config.mavros_ns.c_str());

  if (!wait_for_ready(ros::Duration(config.init_wait_timeout_s), &state)) {
    ROS_ERROR("[send_vision_pose] timeout waiting for first local_odom and "
              "odom_status messages.");
    return 1;
  }

  if (ensure_source_param_ready(&config, &state, &param_get_client,
                                &param_set_client, true) !=
          ParamCheckResult::kReady &&
      config.exit_on_param_error) {
    ROS_ERROR("[send_vision_pose] init param check failed.");
    return 1;
  }

  ros::Timer publish_timer = nh.createTimer(
      ros::Duration(1.0 / config.publish_rate_hz),
      boost::bind(&publish_timer_callback, _1, &config, &state,
                  &vision_pose_pub, &param_get_client, &param_set_client));

  ROS_INFO("[send_vision_pose] init ok, publish_rate=%.1fHz",
           config.publish_rate_hz);
  ros::spin();

  (void)local_odom_sub;
  (void)odom_status_sub;
  (void)publish_timer;
  return 0;
}
