#include "single_uav_simulator.h"

#include <algorithm>

namespace sunray_sim
{
SingleUavSimulator::SingleUavSimulator(ros::NodeHandle& nh,
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
    double init_z = 0.0;
    nh_.param<double>(vehicle_param_prefix + "init_x", init_x, 0.0);
    nh_.param<double>(vehicle_param_prefix + "init_y", init_y, 0.0);
    nh_.param<double>(vehicle_param_prefix + "init_z", init_z, 0.0);
    nh_.param<double>(vehicle_param_prefix + "init_yaw", init_yaw_, 0.0);
    init_pos_ = Eigen::Vector3d(init_x, init_y, init_z);

    if (enable_sensing_ && global_map)
    {
        local_mid360_.reset(new LocalMid360Simulator(nh_, global_map, agent_name, agent_id));
    }
    quadrotor_.reset(new UavPlant(nh_, agent_name, agent_id, init_pos_, init_yaw_));
    px4_mavros_.reset(new Px4MavrosSim(nh_, agent_prefix_));

    ROS_INFO("[single_uav_simulator] started for %s init_param=%s init_pos=(%.2f, %.2f, %.2f) init_yaw=%.2f rad",
             agent_prefix_.c_str(),
             vehicle_param_prefix.c_str(),
             init_pos_.x(),
             init_pos_.y(),
             init_pos_.z(),
             init_yaw_);
}

void SingleUavSimulator::printStatus() const
{
    if (local_mid360_)
    {
        local_mid360_->printStatus();
    }
    quadrotor_->printStatus();
    px4_mavros_->printStatus();
}
}  // namespace sunray_sim
