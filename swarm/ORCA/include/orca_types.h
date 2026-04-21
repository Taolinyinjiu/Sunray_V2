#pragma once

#include <cstdint>

namespace orca_swarm
{

struct AgentState
{
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    double timestamp{0.0};
};

struct OrcaSetupCmd
{
    enum Cmd : uint8_t
    {
        GOAL = 1,
        GOAL_RUN = 2,
        RUN = 3,
        STOP = 4
    };

    uint8_t cmd{GOAL};
    float desired_pos[3]{0.0f, 0.0f, 0.0f};
    float desired_yaw{0.0f};
};

struct OrcaOutput
{
    enum State : uint8_t
    {
        INIT = 0,
        RUN = 1,
        ARRIVED = 2,
        STOP = 3
    };

    uint8_t state{INIT};
    float linear[3]{0.0f, 0.0f, 0.0f};
    float angular[3]{0.0f, 0.0f, 0.0f};
    float goal_pos[3]{0.0f, 0.0f, 0.0f};
    float goal_yaw{0.0f};
    double timestamp{0.0};
};

} // namespace orca_swarm
