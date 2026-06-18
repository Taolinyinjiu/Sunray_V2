#include "single_ugv_simulator.h"

#include <algorithm>

namespace sunray_sim
{
SingleUgvSimulator::SingleUgvSimulator(ros::NodeHandle& nh,
                                       pcl::PointCloud<pcl::PointXYZI>::ConstPtr global_map,
                                       const bool enable_sensing,
                                       const std::string& agent_name,
                                       const int agent_id)
    : nh_(nh),
      agent_prefix_("/" + agent_name + std::to_string(std::max(agent_id, 1))),
      enable_sensing_(enable_sensing)
{
    const std::string vehicle_name = agent_name + std::to_string(std::max(agent_id, 1));
    const std::string vehicle_param_prefix = "vehicles/" + vehicle_name + "/";
    double init_x = 0.0;
    double init_y = 0.0;
    nh_.param<double>(vehicle_param_prefix + "init_x", init_x, 0.0);
    nh_.param<double>(vehicle_param_prefix + "init_y", init_y, 0.0);
    nh_.param<double>(vehicle_param_prefix + "init_yaw", init_yaw_, 0.0);
    init_pos_ = Eigen::Vector3d(init_x, init_y, 0.0);

    if (enable_sensing_ && global_map)
    {
        local_mid360_.reset(new LocalMid360Simulator(nh_, global_map, agent_name, agent_id));
    }
    ugv_.reset(new UgvSimulator(nh_, agent_name, agent_id, init_pos_, init_yaw_));

    ROS_INFO("[single_ugv_simulator] started for %s init_param=%s init_pos=(%.2f, %.2f) init_yaw=%.2f rad",
             agent_prefix_.c_str(),
             vehicle_param_prefix.c_str(),
             init_pos_.x(),
             init_pos_.y(),
             init_yaw_);
}

void SingleUgvSimulator::printStatus() const
{
    if (local_mid360_)
    {
        local_mid360_->printStatus();
    }
    ugv_->printStatus();
}
}  // namespace sunray_sim
