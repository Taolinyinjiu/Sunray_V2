#pragma once

#include <string>

#include "planner_datatypes.hpp"

PlannerRuntimeConfig load_selected_planner_config(const std::string& config_yamlfile_path,
                                                  const std::string& planner_type_filter,
                                                  int planner_id_filter,
                                                  const std::string& uav_ns);
