#include "global_map_server.h"
#include "sim_visualizer.h"
#include "single_uav_simulator.h"
#include "single_ugv_simulator.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <XmlRpcValue.h>

namespace
{
std::vector<int> loadAgentIds(ros::NodeHandle& nh, const std::string& param_name)
{
    std::vector<int> agent_ids;
    XmlRpc::XmlRpcValue agent_ids_param;
    if (nh.getParam(param_name, agent_ids_param))
    {
        if (agent_ids_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("[sunray_mavros_sim] %s must be a YAML list, for example: %s: [1, 2]",
                      param_name.c_str(),
                      param_name.c_str());
            return agent_ids;
        }

        for (int i = 0; i < agent_ids_param.size(); ++i)
        {
            if (agent_ids_param[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_WARN("[sunray_mavros_sim] ignore non-integer %s[%d]", param_name.c_str(), i);
                continue;
            }
            agent_ids.push_back(std::max(1, static_cast<int>(agent_ids_param[i])));
        }
    }

    if (agent_ids.empty())
    {
        ROS_ERROR("[sunray_mavros_sim] %s is empty, please set at least one vehicle id", param_name.c_str());
        return agent_ids;
    }

    std::sort(agent_ids.begin(), agent_ids.end());
    agent_ids.erase(std::unique(agent_ids.begin(), agent_ids.end()), agent_ids.end());
    return agent_ids;
}

std::vector<int> loadAgentIdsWithFallback(ros::NodeHandle& nh,
                                          const std::string& param_name,
                                          const std::string& fallback_param_name)
{
    if (nh.hasParam(param_name))
    {
        return loadAgentIds(nh, param_name);
    }
    return loadAgentIds(nh, fallback_param_name);
}

std::string joinAgentPrefixes(const std::string& agent_name, const std::vector<int>& agent_ids)
{
    std::ostringstream ss;
    for (std::size_t i = 0; i < agent_ids.size(); ++i)
    {
        if (i > 0)
        {
            ss << ", ";
        }
        ss << "/" << agent_name << agent_ids[i];
    }
    return ss.str();
}
}  // namespace

int main(int argc, char** argv)
{
    // 这个进程是 sunray_mavros_sim 的唯一运行入口。所有功能模块都在同一个
    // ROS node 内创建，launch 时不再分别启动地图、局部雷达、动力学、PX4
    // 控制转换和 fake MAVROS 多个 node。
    ros::init(argc, argv, "sunray_mavros_sim_node");
    ros::NodeHandle nh("~");

    // uav/agent_name + uav/agent_ids 决定无人机命名空间；
    // ugv/agent_name + ugv/agent_ids 决定无人车命名空间。
    // 为兼容旧配置，uav 会回退读取顶层 agent_name / agent_ids。
    bool enable_uav = true;
    bool enable_ugv = false;
    nh.param<bool>("uav/enable", enable_uav, true);
    nh.param<bool>("ugv/enable", enable_ugv, false);

    std::string uav_agent_name = "uav";
    std::string ugv_agent_name = "ugv";
    nh.param<std::string>("agent_name", uav_agent_name, std::string("uav"));
    nh.param<std::string>("uav/agent_name", uav_agent_name, uav_agent_name);
    nh.param<std::string>("ugv/agent_name", ugv_agent_name, std::string("ugv"));

    const std::vector<int> uav_agent_ids = enable_uav
                                               ? loadAgentIdsWithFallback(nh, "uav/agent_ids", "agent_ids")
                                               : std::vector<int>();
    const std::vector<int> ugv_agent_ids = enable_ugv
                                               ? loadAgentIds(nh, "ugv/agent_ids")
                                               : std::vector<int>();
    if ((enable_uav && uav_agent_ids.empty()) || (enable_ugv && ugv_agent_ids.empty()))
    {
        return 1;
    }
    if (!enable_uav && !enable_ugv)
    {
        ROS_ERROR("[sunray_mavros_sim] both uav/enable and ugv/enable are false");
        return 1;
    }
    const std::string uav_agent_prefixes = joinAgentPrefixes(uav_agent_name, uav_agent_ids);
    const std::string ugv_agent_prefixes = joinAgentPrefixes(ugv_agent_name, ugv_agent_ids);

    // global_frame_id 由各模块直接从同一个私有参数读取，用来统一全局地图、
    // 局部点云、TF、odom 和 navsat 的父坐标系。默认配置为 map，便于对接
    // 常见规划/建图模块；如果需要沿用 world，只改 YAML 中这一处即可。
    std::string global_frame_id = "map";
    nh.param<std::string>("global_frame_id", global_frame_id, std::string("map"));

    // enable_sensing 控制“地图 + 局部 MID360”感知链路是否启动。
    // false 时不创建 GlobalMapServer 和 LocalMid360Simulator，只保留动力学、
    // PX4 setpoint 转换和 fake MAVROS，适合只需要 odom/imu/MAVROS 接口的场景。
    bool enable_sensing = true;
    nh.param<bool>("enable_sensing", enable_sensing, true);

    // enable_visualizer 控制 RViz 可视化 MarkerArray 是否启动。
    // 可视化模块只订阅本包已经发布的话题，不参与动力学和传感器计算；
    // 关闭后不会影响 odom、IMU、点云或 fake MAVROS 输出。
    bool enable_visualizer = true;
    nh.param<bool>("visualizer/enable", enable_visualizer, true);

    // 统一状态打印参数。各功能模块只提供 printStatus() 接口，不再各自创建
    // 状态打印定时器，避免终端输出分散、频率不一致。
    bool enable_status_print = true;
    double status_print_hz = 1.0;
    nh.param<bool>("enable_status_print", enable_status_print, true);
    nh.param<double>("status_print_hz", status_print_hz, 1.0);
    status_print_hz = std::max(0.1, status_print_hz);

    // 1. 可选感知链路：
    //    enable_sensing=true 时创建全局地图服务，读取 PCD 并周期发布全局点云；
    //    地图点云指针随后传给每个 SingleUavSimulator，用于创建局部 MID360 仿真。
    std::unique_ptr<sunray_mavros_sim::GlobalMapServer> map_server;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr global_map;
    if (enable_sensing)
    {
        map_server.reset(new sunray_mavros_sim::GlobalMapServer(nh));
        if (!map_server->ready())
        {
            ROS_ERROR("[sunray_mavros_sim] failed to initialize global map, node will exit");
            return 1;
        }
        global_map = map_server->cloud();
    }

    // 2. 无人机/无人车仿真管理：
    //    每个 agent_id 创建一个 SingleUavSimulator。SingleUavSimulator 内部创建
    //    并管理该无人机的动力学、PX4/MAVROS 控制转换和 fake MAVROS；
    //    enable_sensing=true 时额外创建该无人机自己的局部 MID360。
    //    初始位置由 SingleUavSimulator 按 vehicles/<agent>/init_* 读取。
    std::vector<std::unique_ptr<sunray_mavros_sim::SingleUavSimulator>> uav_simulators;
    std::vector<std::unique_ptr<sunray_mavros_sim::SingleUgvSimulator>> ugv_simulators;
    std::vector<std::unique_ptr<sunray_mavros_sim::SimVisualizer>> visualizers;
    uav_simulators.reserve(uav_agent_ids.size());
    ugv_simulators.reserve(ugv_agent_ids.size());
    visualizers.reserve(uav_agent_ids.size() + ugv_agent_ids.size());
    for (const int agent_id : uav_agent_ids)
    {
        uav_simulators.emplace_back(new sunray_mavros_sim::SingleUavSimulator(nh,
                                                                              global_map,
                                                                              enable_sensing,
                                                                              uav_agent_name,
                                                                              agent_id));

        // 3. 统一 RViz 可视化：
        //    每架无人机创建一个 SimVisualizer，订阅自己的 odom、cmd_RPM、
        //    MAVROS state 和局部点云状态，然后发布一个 MarkerArray 话题。
        if (enable_visualizer)
        {
            visualizers.emplace_back(new sunray_mavros_sim::SimVisualizer(nh, uav_agent_name, agent_id));
        }
    }
    for (const int agent_id : ugv_agent_ids)
    {
        ugv_simulators.emplace_back(new sunray_mavros_sim::SingleUgvSimulator(nh,
                                                                              global_map,
                                                                              enable_sensing,
                                                                              ugv_agent_name,
                                                                              agent_id));
        if (enable_visualizer)
        {
            visualizers.emplace_back(new sunray_mavros_sim::SimVisualizer(nh, ugv_agent_name, agent_id));
        }
    }

    ROS_INFO("[sunray_mavros_sim] integrated node started for uav=[%s], ugv=[%s], global_frame_id=%s",
             uav_agent_prefixes.c_str(),
             ugv_agent_prefixes.c_str(),
             global_frame_id.c_str());
    if (enable_sensing)
    {
        ROS_INFO("[sunray_mavros_sim] sensing outputs: %s, /<agent>/sunray_mavros_sim/cloud_world_frame, /<agent>/sunray_mavros_sim/cloud_sensor_frame",
                 map_server->topic().c_str());
    }
    else
    {
        ROS_INFO("[sunray_mavros_sim] sensing disabled, global map and local MID360 are not started");
    }
    ROS_INFO("[sunray_mavros_sim] dynamics outputs: /<agent>/sunray_mavros_sim/odom, /<agent>/sunray_mavros_sim/imu");
    ROS_INFO("[sunray_mavros_sim] MAVROS facade: /<uav>/mavros/*");
    if (!visualizers.empty())
    {
        ROS_INFO("[sunray_mavros_sim] visualization output: /<agent>/sunray_mavros_sim/visualization");
    }

    ros::Timer status_timer;
    if (enable_status_print)
    {
        // 主节点统一打印关键模块的当前状态：
        //   1. enable_sensing=true 时打印 GlobalMapServer；
        //   2. SingleUavSimulator：打印局部雷达（如启用）、动力学、
        //      PX4 控制转换和 fake MAVROS 的状态；
        //   3. SimVisualizer：打印 RViz 可视化输入/输出状态。
        status_timer = nh.createTimer(
            ros::Duration(1.0 / status_print_hz),
            [&](const ros::TimerEvent&) {
                std::cout << "\033[1;44;37m"
                          << "=================== sunray_mavros_sim [uav=" << uav_simulators.size()
                          << ", ugv=" << ugv_simulators.size()
                          << "] ==================="
                          << "\033[0m" << std::endl;
                if (map_server)
                {
                    map_server->printStatus();
                }
                for (const auto& uav_simulator : uav_simulators)
                {
                    uav_simulator->printStatus();
                }
                for (const auto& ugv_simulator : ugv_simulators)
                {
                    ugv_simulator->printStatus();
                }
                for (const auto& visualizer : visualizers)
                {
                    visualizer->printStatus();
                }
            });
    }

    // 所有模块都依赖 ROS callback/timer 驱动，这里保持一个 spin 即可。
    ros::spin();
    return 0;
}
