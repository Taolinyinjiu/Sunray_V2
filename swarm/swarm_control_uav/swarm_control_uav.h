/*
本文件功能：
    1、定义无人机集群控制骨架类 Swarm_Control_UAV
    2、负责加载参数、订阅本机/其他无人机里程计、订阅集群控制指令、发布无人机控制指令
    3、集成 formation / ORCA 与集群状态机基础框架
*/
#pragma once

#include "ORCA.h"
#include "formation.h"

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlFSMState.h>
#include <sunray_msgs/UAVSwarmCMD.h>
#include <sunray_msgs/UAVSwarmState.h>

#include <string>
#include <vector>

namespace swarm_control
{

class Swarm_Control_UAV
{
  public:
    Swarm_Control_UAV(ros::NodeHandle &nh);
    ~Swarm_Control_UAV() = default;

  private:
    struct Params
    {
        int agent_id{1};                 // 本机编号，从 1 开始
        int swarm_num{1};              // 集群总数量
        std::string agent_name{"uav"};   // 无人机命名前缀
        double control_loop_hz{200.0}; // 主循环频率

        double goal_xy_tolerance{0.1};   // x-y 平面到达阈值
        double goal_z_tolerance{0.1};    // z 方向到达阈值
        double goal_yaw_tolerance{0.1};  // yaw 到达阈值
        double goal_z_kp{1.0};           // 高度速度控制比例系数
        double goal_z_vel_limit{0.8};    // 高度速度控制限幅
        double peer_odom_timeout{0.1};   // 邻居里程计超时阈值
        double dynamic_prepare_wait_time{2.0}; // 动态阵型全体就位后的等待时间

        double field_x_min{-100.0};      // 场地 X 最小值
        double field_x_max{100.0};       // 场地 X 最大值
        double field_y_min{-100.0};      // 场地 Y 最小值
        double field_y_max{100.0};       // 场地 Y 最大值
        double field_z_min{0.0};         // 场地 Z 最小值
        double field_z_max{10.0};        // 场地 Z 最大值

        double orca_neighbor_dist{5.0};  // ORCA 邻居搜索半径
        double orca_time_horizon{2.0};   // ORCA 预测时间窗口
        double orca_radius{0.6};         // ORCA 碰撞半径
        double orca_max_speed{1.0};      // ORCA 平面最大速度
        int orca_max_neighbors{-1};      // ORCA 最大邻居数

        bool static_obstacles_enabled{false};          // 是否启用圆形静态障碍物
        std::vector<double> static_obstacle_x{};       // 圆形静态障碍物圆心 X，单位：米
        std::vector<double> static_obstacle_y{};       // 圆形静态障碍物圆心 Y，单位：米
        std::vector<double> static_obstacle_radius{};  // 圆形静态障碍物半径，单位：米
    };
    Params params_{};

    struct OdomCache
    {
        nav_msgs::Odometry odom{};
        ros::Time receive_time{0.0};
        bool received{false};
    };

    struct GoalPoint
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double yaw{0.0};
        bool valid{false};
    };

    enum class HoverPointSource
    {
        CURRENT_ODOM,
        GOAL_POINT
    };

    void localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void uavFsmStateCallback(const sunray_msgs::UAVControlFSMState::ConstPtr &msg);
    void peerOdomCallback(const nav_msgs::Odometry::ConstPtr &msg, int agent_id);
    void swarmCmdCallback(const sunray_msgs::UAVSwarmCMD::ConstPtr &msg);
    void swarm_control_main_loop(const ros::TimerEvent &event);
    void swarmStatePubTimerCallback(const ros::TimerEvent &event);

    bool isValidAgentId(int agent_id) const;
    bool isReadyForSwarmCmd(const sunray_msgs::UAVSwarmCMD &cmd) const;
    bool hasLocalOdom() const;
    static double getYawFromOdom(const nav_msgs::Odometry &odom);
    static double normalizeYaw(double yaw);
    static double clamp(double value, double min_value, double max_value);
    static bool isDynamicFormationType(uint8_t formation_type);
    bool hasReachedGoal() const;
    bool getFormationGoalForAgent(const sunray_msgs::Formation &formation_cmd,
                                  int agent_id,
                                  double formation_time,
                                  GoalPoint &goal_point);
    bool hasAgentReachedGoal(int agent_id, const GoalPoint &goal_point) const;
    bool isAgentOdomReady(int agent_id, const ros::Time &now) const;
    bool areAllAgentsAtDynamicInitialGoal();
    void updateOrcaCommand();
    void fillGoalPointFromSelfOdom(GoalPoint &goal_point) const;
    void switchToArrived(HoverPointSource hover_point_source);

    ros::NodeHandle nh_;

    ros::Subscriber local_odom_sub_{};
    ros::Subscriber uav_fsm_state_sub_{};
    std::vector<ros::Subscriber> peer_odom_subs_{};
    ros::Subscriber swarm_cmd_sub_{};
    ros::Publisher control_cmd_pub_{};
    ros::Publisher swarm_state_pub_{};
    ros::Timer swarm_control_fsm_timer_{};
    ros::Timer swarm_state_pub_timer_{};

    sunray_msgs::UAVControlCMD uav_control_cmd_{};
    std::vector<OdomCache> odom_caches_{}; // 下标直接使用 agent_id，0 号位保留不用
    sunray_msgs::UAVControlFSMState uav_control_fsm_state_{};
    bool has_uav_fsm_state_{false};
    sunray_msgs::UAVSwarmCMD uav_swarm_cmd_{};
    sunray_msgs::UAVSwarmState uav_swarm_state_{};
    GoalPoint goal_point_{};
    GoalPoint hover_point_{};
    GoalPoint return_point_{};
    ros::Time dynamic_formation_start_time_{0.0};
    ros::Time dynamic_prepare_ready_time_{0.0};
    swarm_formation::formation formation_{};
    orca_swarm::ORCA orca_{};
};

} // namespace swarm_control
