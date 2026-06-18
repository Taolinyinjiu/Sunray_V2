#pragma once

#include "statemachine/sunray_fsm_param.hpp"
#include <yaml-cpp/yaml.h>
// 导入基本参数
void loadBasicParam(const YAML::Node& node, sunray_fsm::basic_param_t& param);
// 导入ROS消息超时参数
void loadMsgTimeoutParam(const YAML::Node& node, sunray_fsm::msg_timeout_param_t& param);
// 导入起飞降落参数
void loadTakeoffLandParam(const YAML::Node& node, sunray_fsm::takeoff_land_param_t& param);
// 导入电子围栏与地理围栏参数
void loadLocalFenceParam(const YAML::Node& node, sunray_fsm::local_fence_param_t& param);
// 导入飞行速度参数
void loadVelocityParam(const YAML::Node& node, sunray_fsm::velocity_param_t& param);
// 导入里程计滤波参数
void loadOdomFilterParam(const YAML::Node& node, control_common::OdomKalmanFilterParam_t& param);
