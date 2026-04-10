#pragma once

#include <stdint.h>
#include <vector>
#include <ego_planner_msgs/OutputData.h>
#include <ego_planner_msgs/StatusData.h>
#include <ego_planner_msgs/PPROutputData.h>

namespace ego_planner_msgs
{

    bool decodeOutputData(const std::vector<uint8_t> &data,
                          ego_planner_msgs::OutputData &output);

    bool decodeStatusData(const std::vector<uint8_t> &data,
                          ego_planner_msgs::StatusData &status);

    bool decodePPROutputData(const std::vector<uint8_t> &data,
                             ego_planner_msgs::PPROutputData &output);
}

