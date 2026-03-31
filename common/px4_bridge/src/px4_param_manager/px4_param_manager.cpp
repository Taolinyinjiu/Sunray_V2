/**
 * @file px4_param_manager.cpp
 * @brief PX4 参数读写管理器实现。
 */

#include "px4_param_manager/px4_param_manager.h"

#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <ros/ros.h>

#include <string>

PX4_ParamManager::PX4_ParamManager(ros::NodeHandle nh) {
  int uav_id = 1;
  std::string uav_name = "uav";

  nh.param("uav_id", uav_id, uav_id);
  nh.param("uav_name", uav_name, uav_name);

  const std::string mavros_ns =
      "/" + uav_name + std::to_string(uav_id) + "/mavros";

  param_set_client_ =
      nh.serviceClient<mavros_msgs::ParamSet>(mavros_ns + "/param/set");
  param_get_client_ =
      nh.serviceClient<mavros_msgs::ParamGet>(mavros_ns + "/param/get");

  initialized_ = true;
}

bool PX4_ParamManager::set_param_raw(const char *name, int value) {
  if (!initialized_) {
    return false;
  }

  mavros_msgs::ParamSet srv;
  srv.request.param_id = name;
  srv.request.value.integer = value;
  return param_set_client_.call(srv) && srv.response.success;
}

bool PX4_ParamManager::set_param_raw(const char *name, double value) {
  if (!initialized_) {
    return false;
  }

  mavros_msgs::ParamSet srv;
  srv.request.param_id = name;
  srv.request.value.real = static_cast<float>(value);
  return param_set_client_.call(srv) && srv.response.success;
}

bool PX4_ParamManager::read_param_raw(const char *name, int *value) {
  if (!initialized_ || value == nullptr) {
    return false;
  }

  mavros_msgs::ParamGet srv;
  srv.request.param_id = name;
  if (!param_get_client_.call(srv) || !srv.response.success) {
    return false;
  }

  *value = static_cast<int>(srv.response.value.integer);
  return true;
}

bool PX4_ParamManager::read_param_raw(const char *name, double *value) {
  if (!initialized_ || value == nullptr) {
    return false;
  }

  mavros_msgs::ParamGet srv;
  srv.request.param_id = name;
  if (!param_get_client_.call(srv) || !srv.response.success) {
    return false;
  }

  *value = static_cast<double>(srv.response.value.real);
  return true;
}

