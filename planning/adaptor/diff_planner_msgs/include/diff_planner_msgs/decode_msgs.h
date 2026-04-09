#pragma once

#include <stdint.h>
#include <vector>
#include <diff_planner_msgs/OutputData.h>
#include <diff_planner_msgs/StatusData.h>
#include <diff_planner_msgs/PPROutputData.h>

namespace diff_planner_msgs
{

bool decodeOutputData(const std::vector<uint8_t> &data,
                      diff_planner_msgs::OutputData &output);

bool decodeStatusData(const std::vector<uint8_t> &data,
                      diff_planner_msgs::StatusData &status);

bool decodePPROutputData(const std::vector<uint8_t> &data,
                         diff_planner_msgs::PPROutputData &output);
}
