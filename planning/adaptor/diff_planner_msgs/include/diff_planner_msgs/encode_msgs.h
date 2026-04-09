#pragma once

#include <stdint.h>
#include <vector>
#include <diff_planner_msgs/SO3Command.h>
#include <diff_planner_msgs/TRPYCommand.h>
#include <diff_planner_msgs/Gains.h>

namespace diff_planner_msgs
{

void encodeSO3Command(const diff_planner_msgs::SO3Command &so3_command,
                      std::vector<uint8_t> &output);
void encodeTRPYCommand(const diff_planner_msgs::TRPYCommand &trpy_command,
                       std::vector<uint8_t> &output);

void encodePPRGains(const diff_planner_msgs::Gains &gains,
                    std::vector<uint8_t> &output);
}
