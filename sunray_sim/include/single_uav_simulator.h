#ifndef SUNRAY_SIM_SINGLE_UAV_SIMULATOR_H
#define SUNRAY_SIM_SINGLE_UAV_SIMULATOR_H

#include "local_mid360_simulator.h"
#include "px4_mavros_sim.h"
#include "uav_plant.h"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <memory>
#include <string>

namespace sunray_sim
{
class SingleUavSimulator
{
public:
    SingleUavSimulator(ros::NodeHandle& nh,
                       pcl::PointCloud<pcl::PointXYZI>::ConstPtr global_map,
                       bool enable_sensing,
                       const std::string& agent_name,
                       int agent_id);

    void printStatus() const;

private:
    ros::NodeHandle nh_;
    std::string agent_prefix_;
    bool enable_sensing_{true};
    Eigen::Vector3d init_pos_{Eigen::Vector3d::Zero()};
    double init_yaw_{0.0};

    std::unique_ptr<LocalMid360Simulator> local_mid360_;
    std::unique_ptr<UavPlant> quadrotor_;
    std::unique_ptr<Px4MavrosSim> px4_mavros_;
};
}  // namespace sunray_sim

#endif
