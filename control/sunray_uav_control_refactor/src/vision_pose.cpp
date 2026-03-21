#include "vision_pose/vision_pose.hpp"

#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>

#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>

namespace {

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

// Resolve uav namespace from params:
// 1) uav_ns
// 2) uav_name + uav_id
// If not found, return empty string.
std::string resolve_uav_namespace(ros::NodeHandle &nh) {
  std::string key;
  std::string uav_ns;
  if (nh.searchParam("uav_ns", key) && nh.getParam(key, uav_ns) &&
      !uav_ns.empty()) {
    return trim_leading_slash(uav_ns);
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

bool approx_equal(double a, double b) {
  const double diff = std::fabs(a - b);
  const double scale = std::max(1.0, std::max(std::fabs(a), std::fabs(b)));
  return diff <= (1e-4 + 1e-3 * scale);
}

} // namespace

VisionPose::VisionPose(ros::NodeHandle &nh) : nh_(nh), private_nh_("~") {}

bool VisionPose::Init() {
  if (!LoadRuntimeParams()) {
    ROS_ERROR("[vision_pose] failed to load runtime params.");
    return false;
  }
  if (!LoadSourceConfigs()) {
    ROS_ERROR("[vision_pose] failed to load source configs.");
    return false;
  }
  if (!SetupMavrosClientsAndPublishers()) {
    ROS_ERROR("[vision_pose] failed to setup MAVROS clients/publishers.");
    return false;
  }

  SetupSubscribers();

  if (!WaitForReady(ros::Duration(init_wait_timeout_s_))) {
    ROS_ERROR("[vision_pose] timeout waiting for first odom/status messages.");
    return false;
  }

  int initial_source_id = -1;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_odom_status_) {
      initial_source_id = static_cast<int>(latest_odom_status_.external_source);
    }
  }
  if (initial_source_id < 0) {
    ROS_ERROR("[vision_pose] invalid initial source_id=%d", initial_source_id);
    return false;
  }

  if (!ApplySourceConfig(initial_source_id)) {
    return false;
  }

  SetupPublishTimer();
  ROS_INFO("[vision_pose] init ok: mavros_ns=%s source_id=%d fuse_type=%d",
           mavros_ns_.c_str(), active_source_id_, active_fuse_type_);
  return true;
}

void VisionPose::Spin() { ros::spin(); }

bool VisionPose::LoadRuntimeParams() {
  private_nh_.param<std::string>("local_odom_topic", local_odom_topic_,
                                 local_odom_topic_);
  private_nh_.param<std::string>("odom_status_topic", odom_status_topic_,
                                 odom_status_topic_);

  private_nh_.param<double>("publish_rate_hz", publish_rate_hz_, publish_rate_hz_);
  private_nh_.param<double>("odom_timeout_s", odom_timeout_s_, odom_timeout_s_);
  private_nh_.param<double>("init_wait_timeout_s", init_wait_timeout_s_,
                            init_wait_timeout_s_);

  private_nh_.param<bool>("auto_set_px4_params", auto_set_px4_params_,
                          auto_set_px4_params_);
  private_nh_.param<bool>("strict_param_check", strict_param_check_,
                          strict_param_check_);
  private_nh_.param<bool>("allow_source_switch", allow_source_switch_,
                          allow_source_switch_);

  publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
  odom_timeout_s_ = std::max(0.0, odom_timeout_s_);
  init_wait_timeout_s_ = std::max(0.1, init_wait_timeout_s_);

  std::string mavros_ns_param = "__AUTO__";
  private_nh_.param<std::string>("mavros_ns", mavros_ns_param, mavros_ns_param);
  mavros_ns_param = trim_trailing_slash(mavros_ns_param);
  if (!mavros_ns_param.empty() && mavros_ns_param != "__AUTO__") {
    mavros_ns_ = ensure_leading_slash(mavros_ns_param);
    return true;
  }

  const std::string uav_ns = resolve_uav_namespace(nh_);
  if (uav_ns.empty()) {
    ROS_WARN("[vision_pose] cannot resolve uav namespace, fallback to /mavros");
    mavros_ns_ = "/mavros";
  } else {
    mavros_ns_ = "/" + trim_trailing_slash(uav_ns) + "/mavros";
  }
  return true;
}

bool VisionPose::LoadSourceConfigs() {
  XmlRpc::XmlRpcValue sources_list;
  if (!private_nh_.getParam("sources_list", sources_list)) {
    ROS_ERROR("[vision_pose] missing param: ~sources_list");
    return false;
  }
  if (sources_list.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("[vision_pose] ~sources_list must be a map.");
    return false;
  }

  sources_.clear();

  for (auto it = sources_list.begin(); it != sources_list.end(); ++it) {
    const std::string source_name = static_cast<std::string>(it->first);
    XmlRpc::XmlRpcValue source_cfg = it->second;

    SourceConfig cfg;
    cfg.source_name = source_name;

    if (source_cfg.hasMember("source_id") &&
        source_cfg["source_id"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      cfg.source_id = static_cast<int>(source_cfg["source_id"]);
    } else {
      ROS_ERROR("[vision_pose] source '%s' missing valid source_id.",
                source_name.c_str());
      return false;
    }

    cfg.fuse_type = 0;
    if (source_cfg.hasMember("fuse_type") &&
        source_cfg["fuse_type"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      cfg.fuse_type = static_cast<int>(source_cfg["fuse_type"]);
    }

    if (source_cfg.hasMember("params")) {
      auto params = source_cfg["params"];
      if (params.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("[vision_pose] source '%s' params must be a list.",
                  source_name.c_str());
        return false;
      }
      for (int i = 0; i < params.size(); ++i) {
        auto p = params[i];
        if (p.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
          ROS_ERROR("[vision_pose] source '%s' params[%d] must be a map.",
                    source_name.c_str(), i);
          return false;
        }

        ParamSpec spec;
        if (p.hasMember("name") &&
            p["name"].getType() == XmlRpc::XmlRpcValue::TypeString) {
          spec.name = static_cast<std::string>(p["name"]);
        } else {
          ROS_ERROR("[vision_pose] source '%s' params[%d] missing name.",
                    source_name.c_str(), i);
          return false;
        }

        std::string type_str;
        if (p.hasMember("type") &&
            p["type"].getType() == XmlRpc::XmlRpcValue::TypeString) {
          type_str = static_cast<std::string>(p["type"]);
        } else {
          ROS_ERROR("[vision_pose] source '%s' param '%s' missing type.",
                    source_name.c_str(), spec.name.c_str());
          return false;
        }

        if (type_str == "int") {
          spec.type = ParamType::kInt;
          if (!p.hasMember("value")) {
            ROS_ERROR("[vision_pose] source '%s' param '%s' missing value.",
                      source_name.c_str(), spec.name.c_str());
            return false;
          }
          if (p["value"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            spec.int_value = static_cast<int>(p["value"]);
          } else if (p["value"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            spec.int_value = static_cast<int>(static_cast<double>(p["value"]));
          } else {
            ROS_ERROR("[vision_pose] source '%s' param '%s' int value type invalid.",
                      source_name.c_str(), spec.name.c_str());
            return false;
          }
        } else if (type_str == "float" || type_str == "double") {
          spec.type = ParamType::kFloat;
          if (!p.hasMember("value")) {
            ROS_ERROR("[vision_pose] source '%s' param '%s' missing value.",
                      source_name.c_str(), spec.name.c_str());
            return false;
          }
          if (p["value"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            spec.float_value = static_cast<double>(p["value"]);
          } else if (p["value"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            spec.float_value = static_cast<double>(static_cast<int>(p["value"]));
          } else {
            ROS_ERROR("[vision_pose] source '%s' param '%s' float value type invalid.",
                      source_name.c_str(), spec.name.c_str());
            return false;
          }
        } else {
          ROS_ERROR("[vision_pose] source '%s' param '%s' unsupported type='%s'",
                    source_name.c_str(), spec.name.c_str(), type_str.c_str());
          return false;
        }

        cfg.params.push_back(spec);
      }
    }

    sources_[cfg.source_id] = cfg;
  }

  return !sources_.empty();
}

bool VisionPose::SetupMavrosClientsAndPublishers() {
  vision_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(mavros_ns_ + "/vision_pose/pose", 20);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(mavros_ns_ + "/odometry/in", 20);

  param_get_client_ = nh_.serviceClient<mavros_msgs::ParamGet>(mavros_ns_ + "/param/get");
  param_set_client_ = nh_.serviceClient<mavros_msgs::ParamSet>(mavros_ns_ + "/param/set");
  return true;
}

void VisionPose::SetupSubscribers() {
  local_odom_sub_ = nh_.subscribe(local_odom_topic_, 50, &VisionPose::LocalOdomCallback,
                                  this);
  odom_status_sub_ = nh_.subscribe(odom_status_topic_, 10, &VisionPose::OdomStatusCallback,
                                   this);
}

void VisionPose::SetupPublishTimer() {
  const double hz = std::max(1.0, publish_rate_hz_);
  publish_timer_ = nh_.createTimer(ros::Duration(1.0 / hz), &VisionPose::OnPublishTimer,
                                   this);
}

bool VisionPose::WaitForReady(const ros::Duration &timeout) {
  const ros::Time start = ros::Time::now();
  ros::Rate rate(50.0);

  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_local_odom_ && has_odom_status_) {
        return true;
      }
    }
    if ((ros::Time::now() - start) >= timeout) {
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }
  return false;
}

const VisionPose::SourceConfig *VisionPose::FindSource(int source_id) const {
  auto it = sources_.find(source_id);
  if (it == sources_.end()) {
    return nullptr;
  }
  return &(it->second);
}

bool VisionPose::ApplySourceConfig(int source_id) {
  const SourceConfig *cfg = FindSource(source_id);
  if (cfg == nullptr) {
    ROS_ERROR("[vision_pose] source_id=%d not found in sources_list.", source_id);
    Disable_fuse();
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_cfg_ = cfg;
    active_source_id_ = cfg->source_id;
    active_fuse_type_ = cfg->fuse_type;
    fuse_enabled_ = false;
  }

  const bool ok = check_param(*cfg);
  if (!ok) {
    Disable_fuse();
    if (strict_param_check_) {
      return false;
    }
  } else {
    std::lock_guard<std::mutex> lock(mutex_);
    fuse_enabled_ = true;
  }
  return true;
}

bool VisionPose::read_param_int(const std::string &name, int *out_value) {
  if (out_value == nullptr) {
    return false;
  }
  mavros_msgs::ParamGet srv;
  srv.request.param_id = name;
  if (!param_get_client_.call(srv) || !srv.response.success) {
    return false;
  }
  *out_value = static_cast<int>(srv.response.value.integer);
  return true;
}

bool VisionPose::read_param_float(const std::string &name, double *out_value) {
  if (out_value == nullptr) {
    return false;
  }
  mavros_msgs::ParamGet srv;
  srv.request.param_id = name;
  if (!param_get_client_.call(srv) || !srv.response.success) {
    return false;
  }
  *out_value = static_cast<double>(srv.response.value.real);
  return true;
}

bool VisionPose::set_param_int(const std::string &name, int value) {
  mavros_msgs::ParamSet srv;
  srv.request.param_id = name;
  srv.request.value.integer = value;
  if (!param_set_client_.call(srv) || !srv.response.success) {
    return false;
  }
  return true;
}

bool VisionPose::set_param_float(const std::string &name, double value) {
  mavros_msgs::ParamSet srv;
  srv.request.param_id = name;
  srv.request.value.real = static_cast<float>(value);
  if (!param_set_client_.call(srv) || !srv.response.success) {
    return false;
  }
  return true;
}

bool VisionPose::check_param(const SourceConfig &cfg) {
  if (cfg.params.empty()) {
    ROS_WARN("[vision_pose] source_id=%d has empty px4 params list.", cfg.source_id);
    return true;
  }

  // Ensure services exist before calling.
  const std::string get_service = mavros_ns_ + "/param/get";
  const std::string set_service = mavros_ns_ + "/param/set";
  if (!ros::service::waitForService(get_service, ros::Duration(2.0))) {
    ROS_ERROR("[vision_pose] service not available: %s", get_service.c_str());
    return false;
  }
  if (auto_set_px4_params_ &&
      !ros::service::waitForService(set_service, ros::Duration(2.0))) {
    ROS_ERROR("[vision_pose] service not available: %s", set_service.c_str());
    return false;
  }

  for (const auto &spec : cfg.params) {
    if (spec.name.empty()) {
      continue;
    }
    if (spec.type == ParamType::kInt) {
      int actual = 0;
      if (!read_param_int(spec.name, &actual)) {
        ROS_ERROR("[vision_pose] read param failed: %s", spec.name.c_str());
        return false;
      }
      if (actual != spec.int_value) {
        ROS_WARN("[vision_pose] px4 param mismatch: %s expected=%d actual=%d",
                 spec.name.c_str(), spec.int_value, actual);
        if (auto_set_px4_params_) {
          if (!set_param_int(spec.name, spec.int_value)) {
            ROS_ERROR("[vision_pose] set param failed: %s", spec.name.c_str());
            return false;
          }
          int verify = 0;
          if (!read_param_int(spec.name, &verify) || verify != spec.int_value) {
            ROS_ERROR("[vision_pose] verify param failed: %s expected=%d actual=%d",
                      spec.name.c_str(), spec.int_value, verify);
            return false;
          }
          ROS_INFO("[vision_pose] set param ok: %s=%d", spec.name.c_str(),
                   spec.int_value);
        } else if (strict_param_check_) {
          return false;
        }
      }
    } else if (spec.type == ParamType::kFloat) {
      double actual = 0.0;
      if (!read_param_float(spec.name, &actual)) {
        ROS_ERROR("[vision_pose] read param failed: %s", spec.name.c_str());
        return false;
      }
      if (!approx_equal(actual, spec.float_value)) {
        ROS_WARN("[vision_pose] px4 param mismatch: %s expected=%.6f actual=%.6f",
                 spec.name.c_str(), spec.float_value, actual);
        if (auto_set_px4_params_) {
          if (!set_param_float(spec.name, spec.float_value)) {
            ROS_ERROR("[vision_pose] set param failed: %s", spec.name.c_str());
            return false;
          }
          double verify = 0.0;
          if (!read_param_float(spec.name, &verify) ||
              !approx_equal(verify, spec.float_value)) {
            ROS_ERROR("[vision_pose] verify param failed: %s expected=%.6f actual=%.6f",
                      spec.name.c_str(), spec.float_value, verify);
            return false;
          }
          ROS_INFO("[vision_pose] set param ok: %s=%.6f", spec.name.c_str(),
                   spec.float_value);
        } else if (strict_param_check_) {
          return false;
        }
      }
    } else {
      ROS_ERROR("[vision_pose] unsupported param type for %s: %s",
                spec.name.c_str(), "<unknown>");
      return false;
    }
  }

  return true;
}

void VisionPose::Disable_fuse() {
  std::lock_guard<std::mutex> lock(mutex_);
  fuse_enabled_ = false;
}

void VisionPose::LocalOdomCallback(const nav_msgs::OdometryConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_local_odom_ = *msg;
  has_local_odom_ = true;
  latest_local_odom_rx_time_ = ros::Time::now();
}

void VisionPose::OdomStatusCallback(const sunray_msgs::OdomStatusConstPtr &msg) {
  if (!msg) {
    return;
  }
  const int source_id = static_cast<int>(msg->external_source);
  int active_source_id_snapshot = -1;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_status_ = *msg;
    has_odom_status_ = true;
    latest_odom_status_rx_time_ = ros::Time::now();
    active_source_id_snapshot = active_source_id_;

    if (allow_source_switch_) {
      if (active_source_id_ < 0) {
        pending_source_id_ = source_id;
      } else if (source_id != active_source_id_) {
        pending_source_id_ = source_id;
      }
    }
  }

  if (!allow_source_switch_ && active_source_id_snapshot >= 0 &&
      source_id != active_source_id_snapshot) {
    ROS_WARN_THROTTLE(1.0,
                      "[vision_pose] external_source changed (%d -> %d) but "
                      "allow_source_switch=false",
                      active_source_id_snapshot, source_id);
  }
}

void VisionPose::OnPublishTimer(const ros::TimerEvent & /*e*/) {
  PublishOnce(ros::Time::now());
}

void VisionPose::PublishOnce(const ros::Time &now) {
  int source_to_apply = -1;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_source_id_ >= 0 && pending_source_id_ == active_source_id_) {
      pending_source_id_ = -1;
    }
    if (pending_source_id_ >= 0 && pending_source_id_ != active_source_id_) {
      source_to_apply = pending_source_id_;
      pending_source_id_ = -1;
    }
  }
  if (source_to_apply >= 0) {
    if (!ApplySourceConfig(source_to_apply)) {
      ROS_WARN_THROTTLE(1.0,
                        "[vision_pose] source switch apply failed, disable fuse.");
    } else {
      int source_id_snapshot = -1;
      int fuse_type_snapshot = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        source_id_snapshot = active_source_id_;
        fuse_type_snapshot = active_fuse_type_;
      }
      ROS_INFO("[vision_pose] switched source_id=%d fuse_type=%d",
               source_id_snapshot, fuse_type_snapshot);
    }
  }

  bool fuse_enabled = false;
  bool has_odom = false;
  bool has_status = false;
  nav_msgs::Odometry odom;
  sunray_msgs::OdomStatus status;
  ros::Time odom_rx_time;
  int fuse_type = 0;
  const SourceConfig *cfg = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fuse_enabled = fuse_enabled_;
    has_odom = has_local_odom_;
    has_status = has_odom_status_;
    odom = latest_local_odom_;
    status = latest_odom_status_;
    odom_rx_time = latest_local_odom_rx_time_;
    fuse_type = active_fuse_type_;
    cfg = active_cfg_;
  }

  if (!fuse_enabled || !has_odom || !has_status || cfg == nullptr) {
    return;
  }

  if (!status.odom_valid) {
    ROS_WARN_THROTTLE(1.0, "[vision_pose] skip publish: odom_valid=false");
    return;
  }

  const ros::Time stamp = odom.header.stamp.isZero() ? odom_rx_time : odom.header.stamp;
  const double age_s = (now - stamp).toSec();
  if (age_s < 0.0 || age_s > odom_timeout_s_) {
    ROS_WARN_THROTTLE(1.0,
                      "[vision_pose] skip publish: odom timeout age=%.3f "
                      "timeout=%.3f",
                      age_s, odom_timeout_s_);
    return;
  }

  if (fuse_type == 0) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = odom.header;
    if (pose_msg.header.stamp.isZero()) {
      pose_msg.header.stamp = now;
    }
    pose_msg.pose = odom.pose.pose;
    vision_pose_pub_.publish(pose_msg);
  } else if (fuse_type == 1) {
    nav_msgs::Odometry odom_msg = odom;
    if (odom_msg.header.stamp.isZero()) {
      odom_msg.header.stamp = now;
    }
    if (odom_msg.child_frame_id.empty()) {
      odom_msg.child_frame_id = "body";
    }
    odom_pub_.publish(odom_msg);
  } else {
    ROS_WARN_THROTTLE(1.0, "[vision_pose] unknown fuse_type=%d", fuse_type);
  }
}
