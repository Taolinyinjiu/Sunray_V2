/*
ORCA formation stress test.

This ROS node does not publish or subscribe. It reads the same ORCA / field /
goal parameters used by swarm_control_uav_sim, generates formation targets
through the formation class, and drives ORCA exactly through its public API.
*/
#include "ORCA.h"
#include "formation.h"

#include <ros/ros.h>
#include <sunray_msgs/Formation.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct Point2D
{
    double x{0.0};
    double y{0.0};
};

struct AgentState
{
    Point2D pos{};
    Point2D vel{};
    Point2D goal{};
    double z{1.5};
    double yaw{0.0};
    bool reached{false};
    double low_speed_time{0.0};
};

struct CircleObstacle
{
    double x{0.0};
    double y{0.0};
    double radius{0.0};
};

struct OrcaParams
{
    float neighbor_dist{2.0F};
    float time_horizon{2.0F};
    float radius{0.2F};
    float max_speed{1.0F};
    float time_step{0.05F};
    int max_neighbors{-1};
};

struct FieldParams
{
    double x_min{-20.0};
    double x_max{20.0};
    double y_min{-20.0};
    double y_max{20.0};
    double z_min{0.0};
    double z_max{3.0};
};

struct TestParams
{
    OrcaParams orca{};
    FieldParams field{};
    double goal_tolerance{0.1};
    double stuck_time_threshold{3.0};
    int transition_case_count{100};
    int print_failed_detail{1};
};

enum class FormationKind
{
    Keep,
    Line,
    Polygon,
    Random,
    Custom,
    DynamicRing,
    DynamicPolygon,
    DynamicLemniscate
};

struct FormationPhase
{
    std::string name{};
    sunray_msgs::Formation cmd{};
    double max_time{20.0};
    bool require_reached{true};
    bool expect_invalid_goal{false};
    double tracking_tolerance{0.9};
};

struct TestCase
{
    std::string name{};
    int agent_num{0};
    std::vector<AgentState> agents{};
    std::vector<FormationPhase> phases{};
    std::vector<CircleObstacle> obstacles{};
};

struct PhaseResult
{
    bool pass{false};
    bool invalid_goal{false};
    bool collision{false};
    int reached_num{0};
    int stuck_num{0};
    double finish_time{0.0};
    double min_pair_distance{std::numeric_limits<double>::infinity()};
    double min_obstacle_clearance{std::numeric_limits<double>::infinity()};
    double final_avg_goal_distance{0.0};
};

struct CaseResult
{
    std::string name{};
    int agent_num{0};
    bool pass{false};
    std::vector<PhaseResult> phase_results{};
};

double distance2D(const Point2D &a, const Point2D &b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double normalizeYaw(const double yaw)
{
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

std::string kindName(const FormationKind kind)
{
    switch (kind)
    {
    case FormationKind::Keep:
        return "KEEP";
    case FormationKind::Line:
        return "LINE";
    case FormationKind::Polygon:
        return "POLYGON";
    case FormationKind::Random:
        return "RANDOM";
    case FormationKind::Custom:
        return "CUSTOM";
    case FormationKind::DynamicRing:
        return "DYN_RING";
    case FormationKind::DynamicPolygon:
        return "DYN_POLYGON";
    case FormationKind::DynamicLemniscate:
        return "LEMNISCATE";
    }
    return "UNKNOWN";
}

double averageGoalDistance(const std::vector<AgentState> &agents)
{
    if (agents.empty())
    {
        return 0.0;
    }

    double sum = 0.0;
    for (const AgentState &agent : agents)
    {
        sum += distance2D(agent.pos, agent.goal);
    }
    return sum / static_cast<double>(agents.size());
}

double minPairDistance(const std::vector<AgentState> &agents)
{
    double min_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < agents.size(); ++i)
    {
        for (size_t j = i + 1; j < agents.size(); ++j)
        {
            min_distance = std::min(min_distance, distance2D(agents[i].pos, agents[j].pos));
        }
    }
    return min_distance;
}

double minObstacleClearance(const std::vector<AgentState> &agents,
                            const std::vector<CircleObstacle> &obstacles,
                            const double agent_radius)
{
    if (obstacles.empty())
    {
        return std::numeric_limits<double>::infinity();
    }

    double min_clearance = std::numeric_limits<double>::infinity();
    for (const AgentState &agent : agents)
    {
        for (const CircleObstacle &obstacle : obstacles)
        {
            const Point2D obstacle_center{obstacle.x, obstacle.y};
            const double clearance = distance2D(agent.pos, obstacle_center) - obstacle.radius - agent_radius;
            min_clearance = std::min(min_clearance, clearance);
        }
    }
    return min_clearance;
}

int countReached(const std::vector<AgentState> &agents)
{
    return static_cast<int>(std::count_if(agents.begin(), agents.end(), [](const AgentState &agent) {
        return agent.reached;
    }));
}

int countStuck(const std::vector<AgentState> &agents, const double stuck_time_threshold)
{
    return static_cast<int>(std::count_if(agents.begin(), agents.end(), [stuck_time_threshold](const AgentState &agent) {
        return !agent.reached && agent.low_speed_time >= stuck_time_threshold;
    }));
}

void setLeader(sunray_msgs::Formation &cmd,
               const double x,
               const double y,
               const double z = 1.5,
               const double yaw = 0.0)
{
    cmd.leader_pos.x = x;
    cmd.leader_pos.y = y;
    cmd.leader_pos.z = z;
    cmd.leader_yaw = static_cast<float>(yaw);
}

double staticPhaseTime(const FormationKind kind)
{
    return (kind == FormationKind::Random) ? 70.0 : 24.0;
}

double dynamicPhaseTime()
{
    return 28.0;
}

void fillCustomOffsets(sunray_msgs::Formation &cmd, const int agent_num, const int variant)
{
    cmd.custom_offsets_pos.clear();
    cmd.custom_offsets_yaw.clear();
    cmd.custom_offsets_pos.reserve(static_cast<size_t>(agent_num));
    cmd.custom_offsets_yaw.reserve(static_cast<size_t>(agent_num));

    const double safe_spacing = 0.75 + 0.05 * static_cast<double>(variant % 3);
    for (int i = 0; i < agent_num; ++i)
    {
        geometry_msgs::Point offset;
        if (variant % 3 == 0)
        {
            const double center = 0.5 * static_cast<double>(agent_num - 1);
            offset.x = (static_cast<double>(i) - center) * safe_spacing;
            offset.y = ((i % 2) == 0) ? 0.35 : -0.35;
        }
        else if (variant % 3 == 1)
        {
            const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(agent_num);
            const double radius = 0.75 + 0.06 * static_cast<double>(agent_num);
            offset.x = radius * std::cos(angle);
            offset.y = 0.65 * radius * std::sin(angle);
        }
        else
        {
            const int row = i / 3;
            const int col = i % 3;
            offset.x = (static_cast<double>(col) - 1.0) * safe_spacing;
            offset.y = (static_cast<double>(row) - 0.5 * static_cast<double>((agent_num - 1) / 3)) * safe_spacing;
        }
        offset.z = 0.0;
        cmd.custom_offsets_pos.push_back(offset);
        cmd.custom_offsets_yaw.push_back(0.0F);
    }
}

FormationPhase makePhase(const FormationKind kind,
                         const int agent_num,
                         const int variant,
                         const double leader_x,
                         const double leader_y)
{
    FormationPhase phase;
    phase.name = kindName(kind);
    phase.cmd.header.stamp = ros::Time(1000 + variant, 1000 * variant + agent_num);
    setLeader(phase.cmd, leader_x, leader_y);

    const double line_spacing = 0.65 + 0.05 * static_cast<double>((variant + agent_num) % 4);
    const double polygon_spacing = 0.75 + 0.05 * static_cast<double>((variant + 2 * agent_num) % 4);
    const double ring_radius = std::max(0.75, 0.12 * static_cast<double>(agent_num) + 0.35);

    switch (kind)
    {
    case FormationKind::Keep:
        phase.cmd.formation_type = sunray_msgs::Formation::STATIC_KEEP_FORMATION;
        phase.max_time = staticPhaseTime(kind);
        phase.require_reached = true;
        break;

    case FormationKind::Line:
        phase.cmd.formation_type = sunray_msgs::Formation::STATIC_FORMATION_LINE;
        phase.cmd.static_line_spacing = static_cast<float>(line_spacing);
        phase.cmd.static_line_angle = static_cast<float>((variant * 37 + agent_num * 11) % 180);
        phase.max_time = staticPhaseTime(kind);
        phase.require_reached = true;
        break;

    case FormationKind::Polygon:
        phase.cmd.formation_type = sunray_msgs::Formation::STATIC_FORMATION_POLYGON;
        phase.cmd.static_polygon_spacing = static_cast<float>(polygon_spacing);
        phase.max_time = staticPhaseTime(kind);
        phase.require_reached = true;
        break;

    case FormationKind::Random:
        phase.cmd.formation_type = sunray_msgs::Formation::STATIC_FORMATION_RANDOM;
        phase.max_time = staticPhaseTime(kind);
        phase.require_reached = true;
        break;

    case FormationKind::Custom:
        phase.cmd.formation_type = sunray_msgs::Formation::STATIC_FORMATION_CUSTOM;
        fillCustomOffsets(phase.cmd, agent_num, variant);
        phase.max_time = staticPhaseTime(kind);
        phase.require_reached = true;
        break;

    case FormationKind::DynamicRing:
        phase.cmd.formation_type = sunray_msgs::Formation::DYNAMIC_FORMATION_RING;
        phase.cmd.dynamic_time = static_cast<float>(dynamicPhaseTime());
        phase.cmd.dynamic_ring_radius = static_cast<float>(ring_radius);
        phase.cmd.dynamic_ring_move_speed = ((variant % 2) == 0) ? 0.25F : -0.25F;
        phase.max_time = dynamicPhaseTime();
        phase.require_reached = false;
        phase.tracking_tolerance = 1.0;
        break;

    case FormationKind::DynamicPolygon:
        phase.cmd.formation_type = sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON;
        phase.cmd.dynamic_time = static_cast<float>(dynamicPhaseTime());
        phase.cmd.dynamic_polygon_spacing = static_cast<float>(polygon_spacing);
        phase.cmd.dynamic_polygon_move_speed = ((variant % 2) == 0) ? 0.22F : -0.22F;
        phase.max_time = dynamicPhaseTime();
        phase.require_reached = false;
        phase.tracking_tolerance = 1.0;
        break;

    case FormationKind::DynamicLemniscate:
        phase.cmd.formation_type = sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE;
        phase.cmd.dynamic_time = static_cast<float>(dynamicPhaseTime());
        phase.cmd.dynamic_lemniscate_x_radius = static_cast<float>(1.4 + 0.08 * static_cast<double>(agent_num));
        phase.cmd.dynamic_lemniscate_y_radius = static_cast<float>(1.0 + 0.06 * static_cast<double>(agent_num));
        phase.cmd.dynamic_lemniscate_move_speed = ((variant % 2) == 0) ? 0.22F : -0.22F;
        phase.max_time = dynamicPhaseTime();
        phase.require_reached = false;
        phase.tracking_tolerance = 1.1;
        break;
    }
    return phase;
}

std::vector<AgentState> makeCircleAgents(const int agent_num, const double center_x, const double center_y)
{
    std::vector<AgentState> agents;
    agents.reserve(static_cast<size_t>(agent_num));
    const double radius = 1.2 + 0.12 * static_cast<double>(agent_num);
    for (int i = 0; i < agent_num; ++i)
    {
        const double phase = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(agent_num);
        AgentState agent;
        agent.pos.x = center_x + radius * std::cos(phase);
        agent.pos.y = center_y + radius * std::sin(phase);
        agents.push_back(agent);
    }
    return agents;
}

std::vector<AgentState> makeLineAgents(const int agent_num,
                                       const double center_x,
                                       const double center_y,
                                       const double spacing,
                                       const double angle)
{
    std::vector<AgentState> agents;
    agents.reserve(static_cast<size_t>(agent_num));
    const double center = 0.5 * static_cast<double>(agent_num - 1);
    for (int i = 0; i < agent_num; ++i)
    {
        const double distance = (static_cast<double>(i) - center) * spacing;
        AgentState agent;
        agent.pos.x = center_x + distance * std::cos(angle);
        agent.pos.y = center_y + distance * std::sin(angle);
        agents.push_back(agent);
    }
    return agents;
}

bool captureKeepFormationIfNeeded(swarm_formation::formation &formation,
                                  const FormationPhase &phase,
                                  const std::vector<AgentState> &agents)
{
    if (phase.cmd.formation_type != sunray_msgs::Formation::STATIC_KEEP_FORMATION)
    {
        return true;
    }

    std::vector<double> pos_x;
    std::vector<double> pos_y;
    std::vector<double> pos_z;
    std::vector<double> yaw;
    pos_x.reserve(agents.size());
    pos_y.reserve(agents.size());
    pos_z.reserve(agents.size());
    yaw.reserve(agents.size());

    for (const AgentState &agent : agents)
    {
        pos_x.push_back(agent.pos.x);
        pos_y.push_back(agent.pos.y);
        pos_z.push_back(agent.z);
        yaw.push_back(agent.yaw);
    }

    return formation.CaptureKeepFormation(pos_x.data(), pos_y.data(), pos_z.data(), yaw.data(),
                                          static_cast<int>(agents.size()));
}

PhaseResult runPhase(TestCase &test_case,
                     const FormationPhase &phase,
                     const TestParams &params,
                     swarm_formation::formation &formation)
{
    PhaseResult result;
    if (!captureKeepFormationIfNeeded(formation, phase, test_case.agents))
    {
        result.invalid_goal = true;
        return result;
    }

    std::vector<orca_swarm::ORCA> solvers(static_cast<size_t>(test_case.agent_num));
    for (orca_swarm::ORCA &solver : solvers)
    {
        solver.init(test_case.agent_num,
                    params.orca.neighbor_dist,
                    params.orca.time_horizon,
                    params.orca.radius,
                    params.orca.max_speed,
                    params.orca.time_step,
                    params.orca.max_neighbors);

        for (const CircleObstacle &obstacle : test_case.obstacles)
        {
            if (!solver.addCircleObstacle(obstacle.x, obstacle.y, obstacle.radius))
            {
                result.invalid_goal = true;
                return result;
            }
        }
    }

    const double dt = static_cast<double>(params.orca.time_step);
    const double collision_distance = 2.0 * static_cast<double>(params.orca.radius);

    for (double phase_time = 0.0; phase_time <= phase.max_time + 1e-9; phase_time += dt)
    {
        bool invalid_goal = false;
        for (int i = 0; i < test_case.agent_num; ++i)
        {
            AgentState &agent = test_case.agents[static_cast<size_t>(i)];
            double goal_x = 0.0;
            double goal_y = 0.0;
            double goal_z = 0.0;
            double goal_yaw = 0.0;
            if (!formation.GetFormationGoal(phase.cmd, i + 1, phase_time, goal_x, goal_y, goal_z, goal_yaw))
            {
                invalid_goal = true;
                continue;
            }

            agent.goal = Point2D{goal_x, goal_y};
            agent.z = goal_z;
            agent.yaw = normalizeYaw(goal_yaw);
            agent.reached = distance2D(agent.pos, agent.goal) <= params.goal_tolerance;
        }

        result.min_pair_distance = std::min(result.min_pair_distance, minPairDistance(test_case.agents));
        result.min_obstacle_clearance =
            std::min(result.min_obstacle_clearance,
                     minObstacleClearance(test_case.agents, test_case.obstacles, params.orca.radius));
        if (result.min_pair_distance < collision_distance - 1e-4)
        {
            result.collision = true;
        }
        if (result.min_obstacle_clearance < -1e-4)
        {
            result.collision = true;
        }
        result.invalid_goal = result.invalid_goal || invalid_goal;

        if (!ros::ok() || result.invalid_goal)
        {
            result.finish_time = phase_time;
            break;
        }

        if (phase.require_reached && countReached(test_case.agents) == test_case.agent_num)
        {
            result.finish_time = phase_time;
            break;
        }

        std::vector<Point2D> cmd_velocities(static_cast<size_t>(test_case.agent_num));
        for (int self_idx = 0; self_idx < test_case.agent_num; ++self_idx)
        {
            orca_swarm::ORCA &solver = solvers[static_cast<size_t>(self_idx)];
            for (int agent_idx = 0; agent_idx < test_case.agent_num; ++agent_idx)
            {
                const AgentState &agent = test_case.agents[static_cast<size_t>(agent_idx)];
                solver.setAgentState(agent_idx, agent.pos.x, agent.pos.y, agent.vel.x, agent.vel.y);
                solver.setAgentGoal(agent_idx, agent.goal.x, agent.goal.y);
            }

            double vx = 0.0;
            double vy = 0.0;
            if (!solver.GetOrcaVelCmd(self_idx, vx, vy))
            {
                result.invalid_goal = true;
            }
            cmd_velocities[static_cast<size_t>(self_idx)] = Point2D{vx, vy};
        }

        for (int i = 0; i < test_case.agent_num; ++i)
        {
            AgentState &agent = test_case.agents[static_cast<size_t>(i)];
            const Point2D cmd = cmd_velocities[static_cast<size_t>(i)];
            const double speed = std::sqrt(cmd.x * cmd.x + cmd.y * cmd.y);
            const double goal_distance = distance2D(agent.pos, agent.goal);

            if (!agent.reached && speed < 0.02 && goal_distance > params.goal_tolerance)
            {
                agent.low_speed_time += dt;
            }
            else
            {
                agent.low_speed_time = 0.0;
            }

            agent.vel = cmd;
            agent.pos.x += agent.vel.x * dt;
            agent.pos.y += agent.vel.y * dt;
        }

        result.finish_time = phase_time;
    }

    for (AgentState &agent : test_case.agents)
    {
        agent.reached = distance2D(agent.pos, agent.goal) <= params.goal_tolerance;
    }

    result.reached_num = countReached(test_case.agents);
    result.stuck_num = countStuck(test_case.agents, params.stuck_time_threshold);
    result.final_avg_goal_distance = averageGoalDistance(test_case.agents);
    const bool static_ok = !phase.require_reached || result.reached_num == test_case.agent_num;
    const bool dynamic_ok = phase.require_reached || result.final_avg_goal_distance <= phase.tracking_tolerance;
    if (phase.expect_invalid_goal)
    {
        result.pass = result.invalid_goal && !result.collision;
    }
    else
    {
        result.pass = static_ok && dynamic_ok && !result.invalid_goal && !result.collision && result.stuck_num == 0;
    }
    return result;
}

CaseResult runCase(TestCase test_case, const TestParams &params)
{
    swarm_formation::formation formation;
    formation.init(test_case.agent_num,
                   params.orca.radius,
                   params.orca.max_speed,
                   params.field.x_min,
                   params.field.x_max,
                   params.field.y_min,
                   params.field.y_max,
                   params.field.z_min,
                   params.field.z_max);

    for (const CircleObstacle &obstacle : test_case.obstacles)
    {
        formation.addCircleObstacle(obstacle.x, obstacle.y, obstacle.radius);
    }

    CaseResult result;
    result.name = test_case.name;
    result.agent_num = test_case.agent_num;
    result.pass = true;
    for (const FormationPhase &phase : test_case.phases)
    {
        PhaseResult phase_result = runPhase(test_case, phase, params, formation);
        result.pass = result.pass && phase_result.pass;
        result.phase_results.push_back(phase_result);
    }
    return result;
}

TestCase makeTransitionCase(const int case_index, const FormationKind from_kind, const FormationKind to_kind)
{
    const int agent_num = 3 + (case_index % 8);
    const double initial_x = -3.0 + 0.4 * static_cast<double>(case_index % 5);
    const double initial_y = -2.0 + 0.3 * static_cast<double>(case_index % 7);
    const double leader_a_x = -1.5 + 0.25 * static_cast<double>(case_index % 4);
    const double leader_a_y = 1.2 - 0.20 * static_cast<double>(case_index % 5);
    const double leader_b_x = 1.6 - 0.20 * static_cast<double>(case_index % 6);
    const double leader_b_y = -1.2 + 0.25 * static_cast<double>(case_index % 4);

    std::ostringstream name;
    name << "transition_" << std::setw(3) << std::setfill('0') << case_index
         << "_" << kindName(from_kind) << "_to_" << kindName(to_kind)
         << "_N" << agent_num;

    TestCase test_case;
    test_case.name = name.str();
    test_case.agent_num = agent_num;
    test_case.agents = makeCircleAgents(agent_num, initial_x, initial_y);
    test_case.phases.push_back(makePhase(from_kind, agent_num, case_index, leader_a_x, leader_a_y));
    test_case.phases.push_back(makePhase(to_kind, agent_num, case_index + 1000, leader_b_x, leader_b_y));
    return test_case;
}

TestCase makeHeadOnCustomCase(const int case_index, const int agent_num)
{
    TestCase test_case;
    test_case.name = "custom_head_on_swap_N" + std::to_string(agent_num) + "_" + std::to_string(case_index);
    test_case.agent_num = agent_num;
    test_case.agents = makeLineAgents(agent_num, -3.0, 0.0, 0.65, 0.0);

    FormationPhase phase;
    phase.name = "CUSTOM_HEAD_ON_SWAP";
    phase.cmd.header.stamp = ros::Time(3000 + case_index, 0);
    phase.cmd.formation_type = sunray_msgs::Formation::STATIC_FORMATION_CUSTOM;
    setLeader(phase.cmd, 3.0, 0.0);
    phase.max_time = 35.0;
    phase.require_reached = true;
    fillCustomOffsets(phase.cmd, agent_num, 0);
    std::reverse(phase.cmd.custom_offsets_pos.begin(), phase.cmd.custom_offsets_pos.end());
    test_case.phases.push_back(phase);
    return test_case;
}

TestCase makeCircleSwapCustomCase(const int case_index, const int agent_num)
{
    TestCase test_case;
    test_case.name = "custom_circle_swap_N" + std::to_string(agent_num) + "_" + std::to_string(case_index);
    test_case.agent_num = agent_num;
    test_case.agents = makeCircleAgents(agent_num, 0.0, 0.0);

    FormationPhase phase;
    phase.name = "CUSTOM_CIRCLE_OPPOSITE";
    phase.cmd.header.stamp = ros::Time(4000 + case_index, 0);
    phase.cmd.formation_type = sunray_msgs::Formation::STATIC_FORMATION_CUSTOM;
    setLeader(phase.cmd, 0.0, 0.0);
    phase.max_time = 40.0;
    phase.require_reached = true;

    phase.cmd.custom_offsets_pos.clear();
    phase.cmd.custom_offsets_yaw.clear();
    for (int i = 0; i < agent_num; ++i)
    {
        const double phase_angle = 2.0 * kPi * static_cast<double>((i + agent_num / 2) % agent_num) /
                                   static_cast<double>(agent_num);
        geometry_msgs::Point offset;
        offset.x = 1.2 * std::cos(phase_angle);
        offset.y = 1.2 * std::sin(phase_angle);
        offset.z = 0.0;
        phase.cmd.custom_offsets_pos.push_back(offset);
        phase.cmd.custom_offsets_yaw.push_back(0.0F);
    }
    test_case.phases.push_back(phase);
    return test_case;
}

std::vector<CircleObstacle> makeFixedObstacleSet()
{
    return {
        {2.0, 0.0, 0.8},
        {-2.0, 1.5, 0.8},
        {0.0, -2.0, 0.8},
    };
}

TestCase makeObstacleCrossCustomCase(const int case_index, const int agent_num, const double y_center)
{
    TestCase test_case;
    test_case.name = "obstacle_cross_custom_N" + std::to_string(agent_num) + "_" + std::to_string(case_index);
    test_case.agent_num = agent_num;
    test_case.agents = makeLineAgents(agent_num, -5.0, y_center, 0.7, 0.5 * kPi);
    test_case.obstacles = makeFixedObstacleSet();

    FormationPhase phase;
    phase.name = "CUSTOM_CROSS_FIXED_OBSTACLES";
    phase.cmd.header.stamp = ros::Time(5000 + case_index, 0);
    phase.cmd.formation_type = sunray_msgs::Formation::STATIC_FORMATION_CUSTOM;
    setLeader(phase.cmd, 5.0, y_center);
    phase.max_time = 55.0;
    phase.require_reached = true;

    const double center = 0.5 * static_cast<double>(agent_num - 1);
    for (int i = 0; i < agent_num; ++i)
    {
        geometry_msgs::Point offset;
        offset.x = 0.0;
        offset.y = (static_cast<double>(i) - center) * 0.7;
        offset.z = 0.0;
        phase.cmd.custom_offsets_pos.push_back(offset);
        phase.cmd.custom_offsets_yaw.push_back(0.0F);
    }

    test_case.phases.push_back(phase);
    return test_case;
}

TestCase makeObstacleGoalRejectCase(const int case_index)
{
    TestCase test_case;
    test_case.name = "obstacle_goal_reject_" + std::to_string(case_index);
    test_case.agent_num = 3;
    test_case.agents = makeCircleAgents(test_case.agent_num, -4.0, -4.0);
    test_case.obstacles = makeFixedObstacleSet();

    FormationPhase phase;
    phase.name = "GOAL_INSIDE_OBSTACLE_SHOULD_REJECT";
    phase.cmd.header.stamp = ros::Time(6000 + case_index, 0);
    phase.cmd.formation_type = sunray_msgs::Formation::STATIC_FORMATION_CUSTOM;
    setLeader(phase.cmd, 2.0, 0.0);
    phase.max_time = 1.0;
    phase.require_reached = true;
    phase.expect_invalid_goal = true;
    phase.tracking_tolerance = 0.0;

    for (int i = 0; i < test_case.agent_num; ++i)
    {
        geometry_msgs::Point offset;
        offset.x = 0.0;
        offset.y = 0.0;
        offset.z = 0.0;
        phase.cmd.custom_offsets_pos.push_back(offset);
        phase.cmd.custom_offsets_yaw.push_back(0.0F);
    }

    test_case.phases.push_back(phase);
    return test_case;
}

std::vector<TestCase> makeTestCases(const int transition_case_count)
{
    const std::vector<FormationKind> kinds = {
        FormationKind::Keep,
        FormationKind::Line,
        FormationKind::Polygon,
        FormationKind::Random,
        FormationKind::Custom,
        FormationKind::DynamicRing,
        FormationKind::DynamicPolygon,
        FormationKind::DynamicLemniscate,
    };

    std::vector<TestCase> cases;
    cases.reserve(static_cast<size_t>(transition_case_count + 16));

    int case_index = 0;
    while (static_cast<int>(cases.size()) < transition_case_count)
    {
        for (FormationKind from_kind : kinds)
        {
            for (FormationKind to_kind : kinds)
            {
                if (static_cast<int>(cases.size()) >= transition_case_count)
                {
                    break;
                }
                cases.push_back(makeTransitionCase(case_index++, from_kind, to_kind));
            }
        }
    }

    for (int agent_num = 3; agent_num <= 10; ++agent_num)
    {
        cases.push_back(makeHeadOnCustomCase(case_index++, agent_num));
        cases.push_back(makeCircleSwapCustomCase(case_index++, agent_num));
    }

    cases.push_back(makeObstacleCrossCustomCase(case_index++, 3, 0.0));
    cases.push_back(makeObstacleCrossCustomCase(case_index++, 6, 0.0));
    cases.push_back(makeObstacleCrossCustomCase(case_index++, 10, 0.0));
    cases.push_back(makeObstacleCrossCustomCase(case_index++, 6, 1.5));
    cases.push_back(makeObstacleGoalRejectCase(case_index++));
    return cases;
}

TestParams loadParams(ros::NodeHandle &nh)
{
    TestParams params;
    double control_loop_hz = 20.0;
    nh.param("control_loop_hz", control_loop_hz, 20.0);
    nh.param("goal_xy_tolerance", params.goal_tolerance, 0.1);
    nh.param("orca/neighbor_dist", params.orca.neighbor_dist, 2.0F);
    nh.param("orca/time_horizon", params.orca.time_horizon, 2.0F);
    nh.param("orca/radius", params.orca.radius, 0.2F);
    nh.param("orca/max_speed", params.orca.max_speed, 1.0F);
    nh.param("orca/max_neighbors", params.orca.max_neighbors, -1);
    nh.param("field/x_min", params.field.x_min, -20.0);
    nh.param("field/x_max", params.field.x_max, 20.0);
    nh.param("field/y_min", params.field.y_min, -20.0);
    nh.param("field/y_max", params.field.y_max, 20.0);
    nh.param("field/z_min", params.field.z_min, 0.0);
    nh.param("field/z_max", params.field.z_max, 3.0);
    nh.param("orca_test/transition_case_count", params.transition_case_count, 100);
    nh.param("orca_test/print_failed_detail", params.print_failed_detail, 1);

    if (control_loop_hz > 0.0)
    {
        params.orca.time_step = static_cast<float>(1.0 / control_loop_hz);
    }
    return params;
}

void printConfig(const TestParams &params, const int case_count)
{
    std::cout << "========== ORCA FORMATION TEST ==========\n";
    std::cout << "case_count=" << case_count
              << "  radius=" << params.orca.radius
              << " m  max_speed=" << params.orca.max_speed
              << " m/s  neighbor_dist=" << params.orca.neighbor_dist
              << " m  time_horizon=" << params.orca.time_horizon
              << " s  time_step=" << params.orca.time_step
              << " s  goal_tolerance=" << params.goal_tolerance << " m\n";
    std::cout << "field=[" << params.field.x_min << ", " << params.field.x_max
              << "] x [" << params.field.y_min << ", " << params.field.y_max
              << "] x [" << params.field.z_min << ", " << params.field.z_max << "] m\n";
}

void printCaseResult(const int index, const CaseResult &result, const bool print_detail)
{
    int pass_phase_num = 0;
    double min_pair = std::numeric_limits<double>::infinity();
    double min_obstacle_clearance = std::numeric_limits<double>::infinity();
    double max_avg_goal = 0.0;
    for (const PhaseResult &phase_result : result.phase_results)
    {
        if (phase_result.pass)
        {
            ++pass_phase_num;
        }
        min_pair = std::min(min_pair, phase_result.min_pair_distance);
        min_obstacle_clearance = std::min(min_obstacle_clearance, phase_result.min_obstacle_clearance);
        max_avg_goal = std::max(max_avg_goal, phase_result.final_avg_goal_distance);
    }

    std::cout << std::fixed << std::setprecision(2)
              << "[" << std::setw(3) << index << "] "
              << std::left << std::setw(56) << result.name
              << " N=" << std::setw(2) << result.agent_num
              << " phases=" << pass_phase_num << "/" << result.phase_results.size()
              << " min_pair=" << std::setw(5) << min_pair
              << " min_obs_clear=" << std::setw(7)
              << (std::isinf(min_obstacle_clearance) ? 999.0 : min_obstacle_clearance)
              << " max_avg_goal=" << std::setw(6) << max_avg_goal
              << " " << (result.pass ? "PASS" : "CHECK") << "\n";

    if (!result.pass && print_detail)
    {
        for (size_t i = 0; i < result.phase_results.size(); ++i)
        {
            const PhaseResult &phase = result.phase_results[i];
            std::cout << "      phase[" << i << "]"
                      << " pass=" << (phase.pass ? "yes" : "no")
                      << " reached=" << phase.reached_num
                      << " collision=" << (phase.collision ? "yes" : "no")
                      << " invalid_goal=" << (phase.invalid_goal ? "yes" : "no")
                      << " stuck=" << phase.stuck_num
                      << " min_pair=" << phase.min_pair_distance
                      << " min_obs_clear=" << phase.min_obstacle_clearance
                      << " avg_goal=" << phase.final_avg_goal_distance
                      << " finish_time=" << phase.finish_time << "\n";
        }
    }
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orca_test");
    ros::NodeHandle nh("~");

    const TestParams params = loadParams(nh);
    const std::vector<TestCase> cases = makeTestCases(std::max(100, params.transition_case_count));
    printConfig(params, static_cast<int>(cases.size()));

    int pass_case_num = 0;
    int pass_phase_num = 0;
    int total_phase_num = 0;
    double global_min_pair = std::numeric_limits<double>::infinity();
    double global_min_obstacle_clearance = std::numeric_limits<double>::infinity();
    std::vector<CaseResult> failed_results;

    for (size_t i = 0; i < cases.size(); ++i)
    {
        const CaseResult result = runCase(cases[i], params);
        if (result.pass)
        {
            ++pass_case_num;
        }
        else
        {
            failed_results.push_back(result);
        }

        for (const PhaseResult &phase_result : result.phase_results)
        {
            ++total_phase_num;
            if (phase_result.pass)
            {
                ++pass_phase_num;
            }
            global_min_pair = std::min(global_min_pair, phase_result.min_pair_distance);
            global_min_obstacle_clearance =
                std::min(global_min_obstacle_clearance, phase_result.min_obstacle_clearance);
        }

        printCaseResult(static_cast<int>(i + 1), result, params.print_failed_detail != 0);
    }

    std::cout << "\n================ ORCA TEST SUMMARY ================\n";
    std::cout << "cases:  " << pass_case_num << "/" << cases.size() << " passed\n";
    std::cout << "phases: " << pass_phase_num << "/" << total_phase_num << " passed\n";
    std::cout << "global_min_pair_distance=" << std::fixed << std::setprecision(3)
              << global_min_pair << " m\n";
    if (!std::isinf(global_min_obstacle_clearance))
    {
        std::cout << "global_min_obstacle_clearance=" << std::fixed << std::setprecision(3)
                  << global_min_obstacle_clearance << " m\n";
    }

    if (!failed_results.empty())
    {
        std::cout << "failed_cases:\n";
        for (const CaseResult &result : failed_results)
        {
            std::cout << "  " << result.name << "\n";
        }
    }

    return failed_results.empty() ? 0 : 1;
}
