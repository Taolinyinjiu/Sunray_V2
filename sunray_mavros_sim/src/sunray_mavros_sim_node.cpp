#include "fake_mavros_bridge.h"
#include "global_map_server.h"
#include "local_mid360_simulator.h"
#include "px4_control_sim.h"
#include "quadrotor_simulator.h"

#include <algorithm>
#include <iostream>
#include <memory>

int main(int argc, char** argv)
{
    // 这个进程是 sunray_mavros_sim 的唯一运行入口。所有功能模块都在同一个
    // ROS node 内创建，launch 时不再分别启动地图、局部雷达、动力学、PX4
    // 控制转换和 fake MAVROS 多个 node。
    ros::init(argc, argv, "sunray_mavros_sim_node");
    ros::NodeHandle nh("~");

    // agent_name/agent_id 决定单机命名空间，实际话题前缀由下面的 agent_prefix 拼出。
    // 本包内部无人机仿真话题统一放在 /<agent>/sunray_mavros_sim 下；
    // fake MAVROS 话题仍保持 /<agent>/mavros 下，方便上层代码按 MAVROS 接口接入。
    std::string agent_name = "uav";
    int agent_id = 1;
    nh.param<std::string>("agent_name", agent_name, std::string("uav"));
    nh.param<int>("agent_id", agent_id, 1);
    agent_id = std::max(agent_id, 1);
    const std::string agent_prefix = "/" + agent_name + std::to_string(agent_id);

    // global_frame_id 由各模块直接从同一个私有参数读取，用来统一全局地图、
    // 局部点云、TF、odom 和 navsat 的父坐标系。默认配置为 map，便于对接
    // 常见规划/建图模块；如果需要沿用 world，只改 YAML 中这一处即可。
    std::string global_frame_id = "map";
    nh.param<std::string>("global_frame_id", global_frame_id, std::string("map"));

    // 统一状态打印参数。各功能模块只提供 printStatus() 接口，不再各自创建
    // 状态打印定时器，避免终端输出分散、频率不一致。
    bool enable_status_print = true;
    double status_print_hz = 1.0;
    nh.param<bool>("enable_status_print", enable_status_print, true);
    nh.param<double>("status_print_hz", status_print_hz, 1.0);
    status_print_hz = std::max(0.1, status_print_hz);

    // 1. 全局地图服务：
    //    - 读取 PCD 文件；
    //    - 做可选坐标转换、偏移、边界补充和体素降采样；
    //    - 周期发布 /map_generator/global_cloud；
    //    - 向局部雷达仿真模块提供内存中的全局点云指针。
    sunray_mavros_sim::GlobalMapServer map_server(nh);
    if (!map_server.ready())
    {
        ROS_ERROR("[sunray_mavros_sim] failed to initialize global map, node will exit");
        return 1;
    }

    // 2. 局部 MID360 风格点云：
    //    使用全局点云和当前 odom 生成局部扫描结果，输出全局坐标系点云、
    //    传感器坐标系点云和深度图。该模块订阅 odom，先创建也可以等待后续
    //    QuadrotorSimulator 发布 odom。
    sunray_mavros_sim::LocalMid360Simulator local_lidar(nh, map_server.cloud(), agent_name, agent_id);

    // 3. 动力学仿真：
    //    订阅 /<agent>/sunray_mavros_sim/cmd_RPM，积分四旋翼动力学，并发布
    //    odom、IMU 和 GNSS 假数据。没有电机指令或指令超时时会保持当前位置。
    sunray_mavros_sim::QuadrotorSimulator quadrotor_simulator(nh, agent_name, agent_id);

    // 4. PX4/MAVROS 控制转换：
    //    订阅 /<agent>/mavros/setpoint_raw/local 和
    //    /<agent>/mavros/setpoint_raw/attitude，将 MAVROS setpoint 转成四电机
    //    RPM，发布给上面的 QuadrotorSimulator。
    PX4_CONTROL_SIM px4_control(nh, agent_prefix);

    // 5. fake MAVROS：
    //    把本包内部 odom/imu 转发成 MAVROS 常用输出，同时提供 set_mode、
    //    arming、param/get、param/set 等常见服务，便于不启动真实 MAVROS 时调试。
    FakeMavrosBridge fake_mavros(nh, agent_prefix);

    ROS_INFO("[sunray_mavros_sim] integrated node started for %s, global_frame_id=%s",
             agent_prefix.c_str(),
             global_frame_id.c_str());
    ROS_INFO("[sunray_mavros_sim] outputs: %s, %s, %s, %s",
             map_server.topic().c_str(),
             (agent_prefix + "/sunray_mavros_sim/cloud_world_frame").c_str(),
             (agent_prefix + "/sunray_mavros_sim/odom").c_str(),
             (agent_prefix + "/sunray_mavros_sim/imu").c_str());
    ROS_INFO("[sunray_mavros_sim] MAVROS facade: %s/mavros/*", agent_prefix.c_str());

    ros::Timer status_timer;
    if (enable_status_print)
    {
        // 主节点统一打印 5 个关键模块的当前状态：
        //   1. GlobalMapServer：地图是否加载、点数、发布话题；
        //   2. LocalMid360Simulator：局部雷达是否收到 odom、当前位置；
        //   3. QuadrotorSimulator：动力学状态、电机指令、当前位置和速度；
        //   4. PX4_CONTROL_SIM：MAVROS setpoint、控制模式、当前 odom；
        //   5. FakeMavrosBridge：fake MAVROS 的订阅/发布话题和参数状态。
        status_timer = nh.createTimer(
            ros::Duration(1.0 / status_print_hz),
            [&](const ros::TimerEvent&) {
                std::cout << "\033[1;44;37m"
                          << "=================== sunray_mavros_sim [" << agent_prefix
                          << "] ==================="
                          << "\033[0m" << std::endl;
                map_server.printStatus();
                local_lidar.printStatus();
                quadrotor_simulator.printStatus();
                px4_control.printStatus();
                fake_mavros.printStatus();
            });
    }

    // 所有模块都依赖 ROS callback/timer 驱动，这里保持一个 spin 即可。
    ros::spin();
    return 0;
}
