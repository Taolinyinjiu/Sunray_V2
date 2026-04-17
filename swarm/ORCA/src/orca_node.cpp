// 中文说明：ORCA 节点入口，初始化引擎与 IO
#include "orca_engine.h"
#include "orca_io.h"
#include <ros/ros.h>

// ORCA 节点入口
int main(int argc, char **argv)
{
    ros::init(argc, argv, "orca_node");
    ros::NodeHandle nh("~");

    int agent_id = 1;
    int agent_num = 1;
    std::string agent_name = "uav";
    nh.param<int>("agent_id", agent_id, 1);
    nh.param<int>("agent_num", agent_num, 1);
    nh.param<std::string>("agent_name", agent_name, std::string("uav"));

    orca_swarm::OrcaParams params;
    nh.param<float>("orca_params/neighborDist", params.neighbor_dist, 1.5f);
    nh.param<float>("orca_params/timeHorizon", params.time_horizon, 2.0f);
    nh.param<float>("orca_params/timeHorizonObst", params.time_horizon_obst, 2.0f);
    nh.param<float>("orca_params/radius", params.radius, 0.3f);
    nh.param<float>("orca_params/maxSpeed", params.max_speed, 0.5f);
    nh.param<float>("orca_params/time_step", params.time_step, 0.1f);

    orca_swarm::GeoFence fence;
    nh.param<float>("geo_fence/min_x", fence.min_x, -5.0f);
    nh.param<float>("geo_fence/max_x", fence.max_x, 5.0f);
    nh.param<float>("geo_fence/min_y", fence.min_y, -5.0f);
    nh.param<float>("geo_fence/max_y", fence.max_y, 5.0f);

    // 初始化 ORCA 引擎与 IO
    orca_swarm::OrcaEngine engine;
    engine.init(agent_id, agent_num, params, fence);

    orca_swarm::OrcaIO io;
    io.init(nh, agent_id, agent_num, agent_name, &engine);

    ros::Rate rate(20.0);
    while (ros::ok())
    {
        ros::spinOnce();
        sunray_msgs::OrcaCmd cmd;
        engine.step(cmd);
        io.publishCmd(cmd);
        io.publishFenceMarkers(engine.fenceMarkers());
        rate.sleep();
    }

    return 0;
}
