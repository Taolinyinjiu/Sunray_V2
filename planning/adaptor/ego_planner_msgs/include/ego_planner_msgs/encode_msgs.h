#pragma once

#include <stdint.h>
#include <vector>
#include <ego_planner_msgs/SO3Command.h>
#include <ego_planner_msgs/TRPYCommand.h>
#include <ego_planner_msgs/Gains.h>

namespace ego_planner_msgs
{

    void encodeSO3Command(const ego_planner_msgs::SO3Command &so3_command,
                          std::vector<uint8_t> &output);
    void encodeTRPYCommand(const ego_planner_msgs::TRPYCommand &trpy_command,
                           std::vector<uint8_t> &output);

    void encodePPRGains(const ego_planner_msgs::Gains &gains,
                        std::vector<uint8_t> &output);
}
