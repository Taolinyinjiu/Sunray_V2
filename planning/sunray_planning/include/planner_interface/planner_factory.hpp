#pragma once

#include <memory>

#include "planner_interface.hpp"

std::unique_ptr<PlannerInterface> create_planner_interface(const PlannerRuntimeConfig& config);
