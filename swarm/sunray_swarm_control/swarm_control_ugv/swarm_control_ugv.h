/*
本文件功能：
    1、定义无人车集群控制类 Swarm_Control_UGV
    2、负责加载参数、订阅本机/其他无人车里程计、订阅集群控制指令、发布无人车控制指令
    3、集成 formation / ORCA，实现 UGV 集群编队控制状态机
*/
#pragma once

#include "ORCA.h"
#include "formation.h"

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVControlState.h>
#include <sunray_msgs/UGVSwarmCMD.h>
#include <sunray_msgs/UGVSwarmState.h>

#include <string>
#include <vector>

namespace swarm_control
{

class Swarm_Control_UGV
{
  public:
    explicit Swarm_Control_UGV(ros::NodeHandle &nh);
    ~Swarm_Control_UGV() = default;

  private:
    struct Params
    {
        int agent_id{1};                 // 本机编号，从 1 开始
        int swarm_num{1};              // 集群总数量
        std::string agent_name{"ugv"};   // 无人车命名前缀
        double control_loop_hz{50.0};  // 主循环频率，单位：Hz
        double peer_odom_timeout{0.1}; // 邻居里程计超时阈值，单位：秒
        double dynamic_prepare_wait_time{2.0}; // 动态阵型全体就位后的等待时间，单位：秒

        double goal_xy_tolerance{0.1};  // XY 平面到达阈值，单位：米
        double goal_yaw_tolerance{0.1}; // yaw 到达阈值，单位：rad

        double field_x_min{-100.0}; // 场地 X 最小值，单位：米
        double field_x_max{100.0};  // 场地 X 最大值，单位：米
        double field_y_min{-100.0}; // 场地 Y 最小值，单位：米
        double field_y_max{100.0};  // 场地 Y 最大值，单位：米
        double field_z_min{-1.0};   // 场地 Z 最小值，单位：米；UGV通常为0附近
        double field_z_max{1.0};    // 场地 Z 最大值，单位：米；UGV通常为0附近

        double orca_neighbor_dist{2.0}; // ORCA 邻居搜索半径，单位：米
        double orca_time_horizon{2.0};  // ORCA 预测时间窗口，单位：秒
        double orca_radius{0.3};        // UGV 等效避碰半径，单位：米
        double orca_max_speed{0.8};     // ORCA 输出最大平面速度，单位：米/秒
        int orca_max_neighbors{-1};     // ORCA 最大邻居数，-1代表考虑所有邻居

        bool static_obstacles_enabled{false};         // 是否启用圆形静态障碍物
        std::vector<double> static_obstacle_x{};      // 圆形静态障碍物圆心 X，单位：米
        std::vector<double> static_obstacle_y{};      // 圆形静态障碍物圆心 Y，单位：米
        std::vector<double> static_obstacle_radius{}; // 圆形静态障碍物半径，单位：米
    };

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

    enum class HoldPointSource
    {
        CURRENT_ODOM,
        GOAL_POINT
    };

    void localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void ugvFsmStateCallback(const sunray_msgs::UGVControlState::ConstPtr &msg);
    void peerOdomCallback(const nav_msgs::Odometry::ConstPtr &msg, int agent_id);
    void swarmCmdCallback(const sunray_msgs::UGVSwarmCMD::ConstPtr &msg);
    void swarm_control_main_loop(const ros::TimerEvent &event);
    void swarmStatePubTimerCallback(const ros::TimerEvent &event);

    bool isValidAgentId(int agent_id) const;
    bool isReadyForSwarmCmd(const sunray_msgs::UGVSwarmCMD &cmd) const;
    bool hasLocalOdom() const;
    static double getYawFromOdom(const nav_msgs::Odometry &odom);
    static double normalizeYaw(double yaw);
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
    void switchToArrived(HoldPointSource hold_point_source);
    void publishHoldCommand();

    ros::NodeHandle nh_;
    Params params_{};

    ros::Subscriber local_odom_sub_{};
    ros::Subscriber ugv_fsm_state_sub_{};
    std::vector<ros::Subscriber> peer_odom_subs_{};
    ros::Subscriber swarm_cmd_sub_{};
    ros::Publisher control_cmd_pub_{};
    ros::Publisher swarm_state_pub_{};
    ros::Timer swarm_control_fsm_timer_{};
    ros::Timer swarm_state_pub_timer_{};

    sunray_msgs::UGVControlCMD ugv_control_cmd_{};
    std::vector<OdomCache> odom_caches_{}; // 下标直接使用 agent_id，0 号位保留不用
    sunray_msgs::UGVControlState ugv_control_state_{};
    bool has_ugv_fsm_state_{false};
    sunray_msgs::UGVSwarmCMD ugv_swarm_cmd_{};
    sunray_msgs::UGVSwarmState ugv_swarm_state_{};
    GoalPoint goal_point_{};
    GoalPoint hold_point_{};
    GoalPoint return_point_{};
    ros::Time dynamic_formation_start_time_{0.0};
    ros::Time dynamic_prepare_ready_time_{0.0};
    swarm_formation::formation formation_{};
    orca_swarm::ORCA orca_{};
};

} // namespace swarm_control
