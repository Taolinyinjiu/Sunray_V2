#pragma once

#include "statemachine/sunray_fsm_param.hpp"
#include <yaml-cpp/yaml.h>

void loadBasicParam(const YAML::Node& node, sunray_fsm::basic_param_t& param);
void loadProtectParam(const YAML::Node& node, sunray_fsm::protect_param_t& param);
void loadMsgTimeoutParam(const YAML::Node& node, sunray_fsm::msg_timeout_param_t& param);
void loadTakeoffLandParam(const YAML::Node& node, sunray_fsm::takeoff_land_param_t& param);
void loadMissionErrorParam(const YAML::Node& node, sunray_fsm::mission_error_param_t& param);
void loadLocalFenceParam(const YAML::Node& node, sunray_fsm::local_fence_param_t& param);
void loadVelocityParam(const YAML::Node& node, sunray_fsm::velocity_param_t& param);
