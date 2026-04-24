#include "planner_interface/planner_factory.hpp"

#include <stdexcept>

#include "planner_interface/diff_planner.hpp"
#include "planner_interface/ego_planner.hpp"
#include "planner_interface/super_planner.hpp"

std::unique_ptr<PlannerInterface> create_planner_interface(const PlannerRuntimeConfig& config) {
    switch (planner_type_from_string(config.planner_type)) {
    case PlannerType::EGO:
        return std::unique_ptr<PlannerInterface>(new EgoPlanner());
    case PlannerType::DIFF:
        return std::unique_ptr<PlannerInterface>(new DiffPlanner());
    case PlannerType::SUPER:
        return std::unique_ptr<PlannerInterface>(new SuperPlannerInterface());
    case PlannerType::FUEL:
        throw std::runtime_error("planner type 'fuel' is not supported by sunray_planning first-stage adapter");
    case PlannerType::UNDEFINE:
    default:
        throw std::runtime_error("unsupported planner type '" + config.planner_type + "'");
    }
}
