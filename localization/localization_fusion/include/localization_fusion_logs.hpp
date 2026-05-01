#pragma once

#include <string>

#include <nav_msgs/Odometry.h>

#include "localization_fusion_types.hpp"

struct LocalizationFusionLogSnapshot {
    std::string uav_ns;
    SourceConfig selected_source;

    std::string odometry_sub_topic;
    std::string relocalization_sub_topic;
    std::string local_pub_topic;
    std::string global_pub_topic;
    std::string odom_status_pub_topic;

    bool has_odometry_data{false};
    bool has_relocalization_data{false};
    bool relocalization_data_valid{false};
    bool odometry_data_timeout{false};

    double odometry_input_rate_hz{0.0};
    double relocalization_input_rate_hz{0.0};

    nav_msgs::Odometry last_odometry_data;
    nav_msgs::Odometry last_relocalization_data;
};

void init_localization_fusion_logger(const std::string& uav_ns,
                                     bool log_save,
                                     std::string& log_file_path);

std::string make_localization_fusion_log_file_path();

void write_localization_fusion_panel(const LocalizationFusionLogSnapshot& snapshot);
