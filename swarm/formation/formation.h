/*
本文件功能：
    1、定义基于 sunray_msgs/Formation 的阵型目标点生成类 formation
    2、对外仅暴露 GetFormationGoal(...) 统一入口
    3、内部完成阵型消息缓存与指定 agent 目标位姿计算
*/
#pragma once

#include <cstdint>
#include <vector>
#include <sunray_msgs/Formation.h>

namespace swarm_formation
{

class formation
{
  public:
    formation() = default;
    ~formation() = default;

    /*
    初始化阵型类：
        1、设置集群总人数
        2、读入 ORCA 避碰半径，用于判断阵型间距/边长是否满足基础安全距离
        3、读入 ORCA 最大速度，用于限制动态阵型速度上限
        4、读入场地 xyz 边界，用于判断最终阵型目标点是否在可飞区域内
        5、后续所有阵型几何参数都直接来自 Formation 消息本身
    */
    void init(int agent_num,
              double orca_radius,
              double orca_max_speed,
              double field_x_min,
              double field_x_max,
              double field_y_min,
              double field_y_max,
              double field_z_min,
              double field_z_max);

    /*
    对外统一入口：
        1、直接接收完整 Formation 消息
        2、formation_time 表示当前阵型已经运行的时间，静态阵型传 0 即可
        3、计算指定 agent 的目标位姿
    */
    bool GetFormationGoal(const sunray_msgs::Formation &formation_cmd,
                          int agent_id,
                          double formation_time,
                          double &target_x,
                          double &target_y,
                          double &target_z,
                          double &target_yaw);

    /*
    STATIC_KEEP_FORMATION 专用抓拍接口：
        1、外部传入当前全集群每个智能体的 x/y/z/yaw
        2、内部将当前位置关系转换为相对 offset 并缓存
        3、后续 GetFormationGoal(STATIC_KEEP_FORMATION, ...) 直接基于该缓存求目标点
    */
    bool CaptureKeepFormation(const double *pos_x,
                              const double *pos_y,
                              const double *pos_z,
                              const double *yaw,
                              int agent_num);

    /*
    添加圆形静态障碍物：
        1、formation 不负责避障，只负责目标点合法性检查；
        2、如果目标点落入 obstacle_radius + orca_radius_ 范围内，GetFormationGoal 返回 false；
        3、这里与 ORCA 中的圆障碍物配置保持一致，避免生成“不可到达/不安全”的阵型目标点。
    */
    bool addCircleObstacle(double x, double y, double radius);

    int agent_num() const
    {
        return agent_num_;
    }

  private:
    bool isValidAgentId(int agent_id) const;
    static bool isDynamicFormationType(uint8_t formation_type);

    bool computeStaticLineOffset(int agent_index,
                                 double spacing,
                                 double angle,
                                 double &offset_x,
                                 double &offset_y,
                                 double &offset_z,
                                 double &offset_yaw) const;
    bool computeStaticPolygonOffset(int agent_index,
                                    double spacing,
                                    double &offset_x,
                                    double &offset_y,
                                    double &offset_z,
                                    double &offset_yaw) const;
    bool computeRegularPolygonVertex(int vertex_index,
                                     int vertex_num,
                                     double spacing,
                                     double &offset_x,
                                     double &offset_y) const;
    bool computeStaticCustomOffset(int agent_index,
                                   double &offset_x,
                                   double &offset_y,
                                   double &offset_z,
                                   double &offset_yaw) const;
    bool computeKeepFormationOffset(int agent_index,
                                    double &offset_x,
                                    double &offset_y,
                                    double &offset_z,
                                    double &offset_yaw) const;
    bool computeStaticRandomOffset(int agent_index,
                                   double &offset_x,
                                   double &offset_y,
                                   double &offset_z,
                                   double &offset_yaw);
    bool generateStaticRandomOffsets(uint32_t seed);
    uint32_t makeStaticRandomSeed(const sunray_msgs::Formation &formation_cmd) const;
    double minSafeDistance() const;
    double maxSafeDistance() const;
    bool isSpacingInSafeRange(double spacing) const;
    bool areTargetDistancesInSafeRange(const std::vector<double> &target_x,
                                       const std::vector<double> &target_y,
                                       const std::vector<double> &target_z) const;
    bool isDynamicMoveSpeedValid(double move_speed) const;
    bool canFitDynamicPath(double max_abs_offset_x, double max_abs_offset_y) const;
    bool isPointSafeFromObstacles(double x, double y) const;
    bool computeDynamicRingOffset(int agent_index,
                                  double radius,
                                  double &offset_x,
                                  double &offset_y,
                                  double &offset_z,
                                  double &offset_yaw) const;
    bool computeDynamicPolygonOffset(int agent_index,
                                     double spacing,
                                     double &offset_x,
                                     double &offset_y,
                                     double &offset_z,
                                     double &offset_yaw) const;
    bool computeDynamicLemniscateOffset(int agent_index,
                                        double x_radius,
                                        double y_radius,
                                        double move_speed,
                                        double &offset_x,
                                        double &offset_y,
                                        double &offset_z,
                                        double &offset_yaw) const;

    struct CircleObstacle
    {
        double x{0.0};
        double y{0.0};
        double radius{0.0};
    };

    int agent_num_{0}; // 当前阵型类维护的集群总人数
    double orca_radius_{0.0}; // ORCA 避碰半径，阵型几何间距需满足 [minSafeDistance(), maxSafeDistance()]
    double orca_max_speed_{1.0}; // ORCA 最大平面速度，动态阵型速度上限不能超过该值
    double field_x_min_{-100.0}; // 场地 X 最小值，单位：米
    double field_x_max_{100.0};  // 场地 X 最大值，单位：米
    double field_y_min_{-100.0}; // 场地 Y 最小值，单位：米
    double field_y_max_{100.0};  // 场地 Y 最大值，单位：米
    double field_z_min_{0.0};    // 场地 Z 最小值，单位：米
    double field_z_max_{10.0};   // 场地 Z 最大值，单位：米
    std::vector<CircleObstacle> circle_obstacles_{}; // 圆形静态障碍物，用于检查目标点是否合法

    sunray_msgs::Formation current_formation_cmd_{};
    double current_formation_time_{0.0};
    bool has_current_formation_cmd_{false};
    std::vector<double> keep_offsets_x_{};
    std::vector<double> keep_offsets_y_{};
    std::vector<double> keep_offsets_z_{};
    std::vector<double> keep_offsets_yaw_{};
    bool has_keep_formation_snapshot_{false};
    std::vector<double> random_offsets_x_{};
    std::vector<double> random_offsets_y_{};
    std::vector<double> random_offsets_z_{};
    uint32_t random_offsets_seed_{0};
    bool has_random_offsets_{false};
};

} // namespace swarm_formation
