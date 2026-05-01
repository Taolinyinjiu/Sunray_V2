/*
本文件功能：
    1、实现基于 sunray_msgs/Formation 的阵型目标点生成类 formation
    2、对外通过 GetFormationGoal(...) 提供统一入口
    3、内部完成阵型消息缓存与指定 agent 目标位姿计算
*/
#include "formation.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <random>
#include <vector>

namespace swarm_formation
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMinDynamicMoveSpeed = 0.05;

double normalizeYaw(const double yaw)
{
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

void rotate2D(const double x, const double y, const double angle, double &rotated_x, double &rotated_y)
{
    const double cos_angle = std::cos(angle);
    const double sin_angle = std::sin(angle);
    rotated_x = cos_angle * x - sin_angle * y;
    rotated_y = sin_angle * x + cos_angle * y;
}

double computeOrbitYaw(const double offset_x, const double offset_y, const double omega)
{
    const double vx = -omega * offset_y;
    const double vy = omega * offset_x;
    return std::atan2(vy, vx);
}

} // namespace

void formation::init(const int agent_num,
                     const double orca_radius,
                     const double orca_max_speed,
                     const double field_x_min,
                     const double field_x_max,
                     const double field_y_min,
                     const double field_y_max,
                     const double field_z_min,
                     const double field_z_max)
{
    agent_num_ = std::max(0, agent_num);
    orca_radius_ = std::max(0.0, orca_radius);
    orca_max_speed_ = std::max(0.0, orca_max_speed);
    field_x_min_ = field_x_min;
    field_x_max_ = field_x_max;
    field_y_min_ = field_y_min;
    field_y_max_ = field_y_max;
    field_z_min_ = field_z_min;
    field_z_max_ = field_z_max;
}

bool formation::GetFormationGoal(const sunray_msgs::Formation &formation_cmd,
                                 const int agent_id,
                                 const double formation_time,
                                 double &target_x,
                                 double &target_y,
                                 double &target_z,
                                 double &target_yaw)
{
    current_formation_cmd_ = formation_cmd;
    current_formation_time_ = std::max(0.0, formation_time);
    has_current_formation_cmd_ = true;

    if (agent_num_ <= 0 || !isValidAgentId(agent_id))
    {
        return false;
    }

    if (isDynamicFormationType(current_formation_cmd_.formation_type) && current_formation_cmd_.dynamic_time <= 0.0f)
    {
        return false;
    }

    const int agent_index = agent_id - 1;
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;
    double offset_yaw = 0.0;

    switch (current_formation_cmd_.formation_type)
    {
    case sunray_msgs::Formation::STATIC_KEEP_FORMATION:
        if (!computeKeepFormationOffset(agent_index, offset_x, offset_y, offset_z, offset_yaw))
        {
            return false;
        }
        break;

    case sunray_msgs::Formation::STATIC_FORMATION_LINE:
        if (!computeStaticLineOffset(agent_index,
                                     current_formation_cmd_.static_line_spacing,
                                     current_formation_cmd_.static_line_angle,
                                     offset_x,
                                     offset_y,
                                     offset_z,
                                     offset_yaw))
        {
            return false;
        }
        break;

    case sunray_msgs::Formation::STATIC_FORMATION_POLYGON:
        if (!computeStaticPolygonOffset(agent_index, current_formation_cmd_.static_polygon_spacing, offset_x,
                                        offset_y, offset_z, offset_yaw))
        {
            return false;
        }
        break;

    case sunray_msgs::Formation::STATIC_FORMATION_RANDOM:
        if (!computeStaticRandomOffset(agent_index, offset_x, offset_y, offset_z, offset_yaw))
        {
            return false;
        }
        break;

    case sunray_msgs::Formation::STATIC_FORMATION_CUSTOM:
        if (!computeStaticCustomOffset(agent_index, offset_x, offset_y, offset_z, offset_yaw))
        {
            return false;
        }
        break;

    case sunray_msgs::Formation::DYNAMIC_FORMATION_RING:
        if (!computeDynamicRingOffset(agent_index, current_formation_cmd_.dynamic_ring_radius, offset_x, offset_y,
                                      offset_z, offset_yaw))
        {
            return false;
        }
        break;

    case sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON:
        if (!computeDynamicPolygonOffset(agent_index, current_formation_cmd_.dynamic_polygon_spacing, offset_x,
                                         offset_y, offset_z, offset_yaw))
        {
            return false;
        }
        break;

    case sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE:
        if (!computeDynamicLemniscateOffset(agent_index,
                                            current_formation_cmd_.dynamic_lemniscate_x_radius,
                                            current_formation_cmd_.dynamic_lemniscate_y_radius,
                                            current_formation_cmd_.dynamic_lemniscate_move_speed,
                                            offset_x,
                                            offset_y,
                                            offset_z,
                                            offset_yaw))
        {
            return false;
        }
        break;

    default:
        return false;
    }

    target_x = current_formation_cmd_.leader_pos.x + offset_x;
    target_y = current_formation_cmd_.leader_pos.y + offset_y;
    target_z = current_formation_cmd_.leader_pos.z + offset_z;
    target_yaw = normalizeYaw(current_formation_cmd_.leader_yaw + offset_yaw);

    if (target_x < field_x_min_ || target_x > field_x_max_ || target_y < field_y_min_ || target_y > field_y_max_ ||
        target_z < field_z_min_ || target_z > field_z_max_)
    {
        return false;
    }

    if (!isPointSafeFromObstacles(target_x, target_y))
    {
        return false;
    }

    return true;
}

bool formation::addCircleObstacle(const double x, const double y, const double radius)
{
    if (radius <= 0.0)
    {
        return false;
    }

    CircleObstacle obstacle;
    obstacle.x = x;
    obstacle.y = y;
    obstacle.radius = radius;
    circle_obstacles_.push_back(obstacle);
    return true;
}

bool formation::isValidAgentId(const int agent_id) const
{
    return agent_id >= 1 && agent_id <= agent_num_;
}

bool formation::isDynamicFormationType(const uint8_t formation_type)
{
    return formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_RING ||
           formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_POLYGON ||
           formation_type == sunray_msgs::Formation::DYNAMIC_FORMATION_LEMNISCATE;
}

bool formation::CaptureKeepFormation(const double *pos_x,
                                     const double *pos_y,
                                     const double *pos_z,
                                     const double *yaw,
                                     const int agent_num)
{
    if (agent_num_ <= 0 || agent_num != agent_num_ || pos_x == nullptr || pos_y == nullptr || pos_z == nullptr ||
        yaw == nullptr)
    {
        return false;
    }

    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
    double yaw_sin_sum = 0.0;
    double yaw_cos_sum = 0.0;

    for (int i = 0; i < agent_num_; ++i)
    {
        center_x += pos_x[i];
        center_y += pos_y[i];
        center_z += pos_z[i];
        yaw_sin_sum += std::sin(yaw[i]);
        yaw_cos_sum += std::cos(yaw[i]);
    }

    const double inv_agent_num = 1.0 / static_cast<double>(agent_num_);
    center_x *= inv_agent_num;
    center_y *= inv_agent_num;
    center_z *= inv_agent_num;
    const double center_yaw = std::atan2(yaw_sin_sum, yaw_cos_sum);

    keep_offsets_x_.assign(static_cast<size_t>(agent_num_), 0.0);
    keep_offsets_y_.assign(static_cast<size_t>(agent_num_), 0.0);
    keep_offsets_z_.assign(static_cast<size_t>(agent_num_), 0.0);
    keep_offsets_yaw_.assign(static_cast<size_t>(agent_num_), 0.0);

    for (int i = 0; i < agent_num_; ++i)
    {
        keep_offsets_x_[static_cast<size_t>(i)] = pos_x[i] - center_x;
        keep_offsets_y_[static_cast<size_t>(i)] = pos_y[i] - center_y;
        keep_offsets_z_[static_cast<size_t>(i)] = pos_z[i] - center_z;
        keep_offsets_yaw_[static_cast<size_t>(i)] = normalizeYaw(yaw[i] - center_yaw);
    }

    has_keep_formation_snapshot_ = true;
    return true;
}


bool formation::computeKeepFormationOffset(const int agent_index,
                                           double &offset_x,
                                           double &offset_y,
                                           double &offset_z,
                                           double &offset_yaw) const
{
    // STATIC_KEEP_FORMATION：
    // 1. offset 并不是来自某个规则阵型模板；
    // 2. 而是来自外部在命令触发瞬间抓拍的全集群当前相对位姿；
    // 3. 后续只要虚拟 leader 改变位置，就在保持该相对关系的前提下整体平移/偏航。
    if (!has_keep_formation_snapshot_ || keep_offsets_x_.size() != static_cast<size_t>(agent_num_) ||
        keep_offsets_y_.size() != static_cast<size_t>(agent_num_) ||
        keep_offsets_z_.size() != static_cast<size_t>(agent_num_) ||
        keep_offsets_yaw_.size() != static_cast<size_t>(agent_num_))
    {
        return false;
    }

    const size_t idx = static_cast<size_t>(agent_index);
    offset_x = keep_offsets_x_[idx];
    offset_y = keep_offsets_y_[idx];
    offset_z = keep_offsets_z_[idx];
    offset_yaw = keep_offsets_yaw_[idx];
    return true;
}

bool formation::computeStaticRandomOffset(const int agent_index,
                                          double &offset_x,
                                          double &offset_y,
                                          double &offset_z,
                                          double &offset_yaw)
{
    // 静态随机阵型：
    // 1. 随机生成的是世界坐标系下的最终目标点；
    // 2. 所有目标点必须位于场地边界向内收缩 2*orca_radius_ 后的安全范围内；
    // 3. 任意两个目标点之间的三维距离必须大于 2*orca_radius_ + 0.1；
    // 4. 生成后转成相对虚拟 leader 的 offset，供统一的 GetFormationGoal 出口使用。
    if (agent_num_ < 1 || agent_num_ > 10)
    {
        return false;
    }

    const uint32_t seed = makeStaticRandomSeed(current_formation_cmd_);
    if (!has_random_offsets_ || random_offsets_seed_ != seed ||
        random_offsets_x_.size() != static_cast<size_t>(agent_num_) ||
        random_offsets_y_.size() != static_cast<size_t>(agent_num_) ||
        random_offsets_z_.size() != static_cast<size_t>(agent_num_))
    {
        if (!generateStaticRandomOffsets(seed))
        {
            return false;
        }
    }

    const size_t idx = static_cast<size_t>(agent_index);
    offset_x = random_offsets_x_[idx];
    offset_y = random_offsets_y_[idx];
    offset_z = random_offsets_z_[idx];
    offset_yaw = 0.0;
    return true;
}

bool formation::generateStaticRandomOffsets(const uint32_t seed)
{
    const double safe_margin = 2.0 * orca_radius_;
    const double min_distance = 2.0 * orca_radius_ + 0.1;
    const double safe_x_min = field_x_min_ + safe_margin;
    const double safe_x_max = field_x_max_ - safe_margin;
    const double safe_y_min = field_y_min_ + safe_margin;
    const double safe_y_max = field_y_max_ - safe_margin;
    const double safe_z_min = field_z_min_ + safe_margin;
    const double safe_z_max = field_z_max_ - safe_margin;

    if (safe_x_min >= safe_x_max || safe_y_min >= safe_y_max || safe_z_min >= safe_z_max)
    {
        return false;
    }

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist_x(safe_x_min, safe_x_max);
    std::uniform_real_distribution<double> dist_y(safe_y_min, safe_y_max);
    std::uniform_real_distribution<double> dist_z(safe_z_min, safe_z_max);

    std::vector<double> target_x;
    std::vector<double> target_y;
    std::vector<double> target_z;
    target_x.reserve(static_cast<size_t>(agent_num_));
    target_y.reserve(static_cast<size_t>(agent_num_));
    target_z.reserve(static_cast<size_t>(agent_num_));

    constexpr int kMaxTryPerAgent = 2000;
    for (int i = 0; i < agent_num_; ++i)
    {
        bool accepted = false;
        for (int try_count = 0; try_count < kMaxTryPerAgent; ++try_count)
        {
            const double candidate_x = dist_x(rng);
            const double candidate_y = dist_y(rng);
            const double candidate_z = dist_z(rng);

            if (!isPointSafeFromObstacles(candidate_x, candidate_y))
            {
                continue;
            }

            bool too_close = false;
            for (size_t j = 0; j < target_x.size(); ++j)
            {
                const double dx = candidate_x - target_x[j];
                const double dy = candidate_y - target_y[j];
                const double dz = candidate_z - target_z[j];
                if (std::sqrt(dx * dx + dy * dy + dz * dz) <= min_distance)
                {
                    too_close = true;
                    break;
                }
            }

            if (too_close)
            {
                continue;
            }

            target_x.push_back(candidate_x);
            target_y.push_back(candidate_y);
            target_z.push_back(candidate_z);
            accepted = true;
            break;
        }

        if (!accepted)
        {
            has_random_offsets_ = false;
            return false;
        }
    }

    random_offsets_x_.assign(static_cast<size_t>(agent_num_), 0.0);
    random_offsets_y_.assign(static_cast<size_t>(agent_num_), 0.0);
    random_offsets_z_.assign(static_cast<size_t>(agent_num_), 0.0);
    for (int i = 0; i < agent_num_; ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        random_offsets_x_[idx] = target_x[idx] - current_formation_cmd_.leader_pos.x;
        random_offsets_y_[idx] = target_y[idx] - current_formation_cmd_.leader_pos.y;
        random_offsets_z_[idx] = target_z[idx] - current_formation_cmd_.leader_pos.z;
    }

    random_offsets_seed_ = seed;
    has_random_offsets_ = true;
    return true;
}

uint32_t formation::makeStaticRandomSeed(const sunray_msgs::Formation &formation_cmd) const
{
    uint32_t seed = 2166136261u;
    const auto mix = [&seed](const uint32_t value) {
        seed ^= value;
        seed *= 16777619u;
    };

    mix(static_cast<uint32_t>(agent_num_));
    mix(static_cast<uint32_t>(formation_cmd.header.stamp.sec));
    mix(static_cast<uint32_t>(formation_cmd.header.stamp.nsec));
    mix(static_cast<uint32_t>(std::llround(formation_cmd.leader_pos.x * 1000.0)));
    mix(static_cast<uint32_t>(std::llround(formation_cmd.leader_pos.y * 1000.0)));
    mix(static_cast<uint32_t>(std::llround(formation_cmd.leader_pos.z * 1000.0)));
    mix(static_cast<uint32_t>(std::llround(static_cast<double>(formation_cmd.leader_yaw) * 1000.0)));
    mix(static_cast<uint32_t>(std::llround(orca_radius_ * 1000.0)));
    return seed;
}

bool formation::computeStaticLineOffset(const int agent_index,
                                        const double spacing,
                                        const double angle,
                                        double &offset_x,
                                        double &offset_y,
                                        double &offset_z,
                                        double &offset_yaw) const
{
    // 静态直线阵型：
    // 1. 所有智能体在惯性系下排成一条直线；
    // 2. 整条队列以虚拟领机位置为几何中心；
    // 3. 相邻两个智能体之间的间距恒为 static_line_spacing；
    // 4. static_line_angle 表示直线方向与世界坐标系 X 轴的夹角，单位为 deg。
    //
    // 这里采用“按索引等间距展开”的规则：
    // 1. agent_index 从 0 开始递增；
    // 2. 若智能体数量为奇数，则会有一个智能体恰好位于虚拟领机位置；
    // 3. 若智能体数量为偶数，则虚拟领机位于中间两个智能体之间，队列对称分布。
    //
    // 例如：
    // 1. angle = 0 deg 时，直线沿 X 轴展开，等价横队；
    // 2. angle = 90 deg 时，直线沿 Y 轴展开，等价纵队；
    // 3. agent_num = 3 时，沿直线方向的距离为 {-1s, 0, +1s}。
    //
    // 其余自由度保持与虚拟领机一致，因此：
    // 1. offset_z = 0.0
    // 2. offset_yaw = 0.0

    // agent_num_ 限制范围：1-10
    if (agent_num_ < 1 || agent_num_ > 10)
    {
        return false;
    }

    // spacing 限制范围：下限为 ORCA 避碰半径*2+0.1，上限暂定 3.0 m
    const double min_spacing = 2.0 * orca_radius_ + 0.1;
    if (spacing < min_spacing || spacing > 3.0)
    {
        return false;
    }

    // 解算 offset
    const double center = 0.5 * static_cast<double>(agent_num_ - 1);
    const double line_distance = (static_cast<double>(agent_index) - center) * spacing;
    const double angle_rad = angle * kDegToRad;
    offset_x = line_distance * std::cos(angle_rad);
    offset_y = line_distance * std::sin(angle_rad);
    offset_z = 0.0;
    offset_yaw = 0.0;
    return true;
}

bool formation::computeStaticPolygonOffset(const int agent_index,
                                           const double spacing,
                                           double &offset_x,
                                           double &offset_y,
                                           double &offset_z,
                                           double &offset_yaw) const
{
    // 静态正多边形阵：
    // 1. 智能体数量 N 由 agent_num_ 决定，建议范围 3-10；
    // 2. 所有智能体位于正 N 边形的 N 个顶点；
    // 3. 虚拟领机位置位于正多边形几何中心；
    // 4. spacing 表示相邻两个智能体之间的边长。
    if (agent_num_ < 3 || agent_num_ > 10)
    {
        return false;
    }

    if (!computeRegularPolygonVertex(agent_index, agent_num_, spacing, offset_x, offset_y))
    {
        return false;
    }

    offset_z = 0.0;
    offset_yaw = 0.0;
    return true;
}

bool formation::computeRegularPolygonVertex(const int vertex_index,
                                            const int vertex_num,
                                            const double spacing,
                                            double &offset_x,
                                            double &offset_y) const
{
    if (vertex_num < 3 || vertex_index < 0 || vertex_index >= vertex_num)
    {
        return false;
    }

    // spacing 限制范围：下限为 ORCA 避碰半径*2+0.1，上限暂定 3.0 m
    const double min_spacing = 2.0 * orca_radius_ + 0.1;
    if (spacing < min_spacing || spacing > 3.0)
    {
        return false;
    }

    const double polygon_radius = spacing / (2.0 * std::sin(kPi / static_cast<double>(vertex_num)));
    const double angle = 0.5 * kPi + 2.0 * kPi * static_cast<double>(vertex_index) /
                                       static_cast<double>(vertex_num);
    offset_x = polygon_radius * std::cos(angle);
    offset_y = polygon_radius * std::sin(angle);
    return true;
}

bool formation::computeStaticCustomOffset(const int agent_index,
                                          double &offset_x,
                                          double &offset_y,
                                          double &offset_z,
                                          double &offset_yaw) const
{
    // 静态自定义阵型：
    // 1. 外部通过 custom_offsets_pos[i] 指定第 i+1 个智能体的位置偏移；
    // 2. 外部通过 custom_offsets_yaw[i] 指定第 i+1 个智能体的偏航角偏移；
    // 3. 本函数不推导阵型几何，只负责按 agent_index 直接读取对应配置。

    // 自定义偏移配置必须覆盖全集群
    if (current_formation_cmd_.custom_offsets_pos.size() != static_cast<size_t>(agent_num_) ||
        current_formation_cmd_.custom_offsets_yaw.size() != static_cast<size_t>(agent_num_))
    {
        return false;
    }

    // 解算 offset
    const size_t idx = static_cast<size_t>(agent_index);
    const geometry_msgs::Point &offset = current_formation_cmd_.custom_offsets_pos[idx];
    offset_x = offset.x;
    offset_y = offset.y;
    offset_z = offset.z;
    offset_yaw = current_formation_cmd_.custom_offsets_yaw[idx];
    return true;
}

double formation::minSafeDistance() const
{
    return 2.0 * orca_radius_ + 0.1;
}

bool formation::isDynamicMoveSpeedValid(const double move_speed) const
{
    const double abs_speed = std::abs(move_speed);
    return abs_speed >= kMinDynamicMoveSpeed && abs_speed <= orca_max_speed_;
}

bool formation::canFitDynamicPath(const double max_abs_offset_x, const double max_abs_offset_y) const
{
    // 动态轨迹会持续运行一段时间，因此不能只检查当前相位点；
    // 这里要求整条轨迹在 XY 平面上都能放进场地，且至少离边界 2*orca_radius_。
    const double safe_margin = 2.0 * orca_radius_;
    return current_formation_cmd_.leader_pos.x - max_abs_offset_x >= field_x_min_ + safe_margin &&
           current_formation_cmd_.leader_pos.x + max_abs_offset_x <= field_x_max_ - safe_margin &&
           current_formation_cmd_.leader_pos.y - max_abs_offset_y >= field_y_min_ + safe_margin &&
           current_formation_cmd_.leader_pos.y + max_abs_offset_y <= field_y_max_ - safe_margin &&
           current_formation_cmd_.leader_pos.z >= field_z_min_ + safe_margin &&
           current_formation_cmd_.leader_pos.z <= field_z_max_ - safe_margin;
}

bool formation::isPointSafeFromObstacles(const double x, const double y) const
{
    for (const CircleObstacle &obstacle : circle_obstacles_)
    {
        const double dx = x - obstacle.x;
        const double dy = y - obstacle.y;
        const double safe_distance = obstacle.radius + orca_radius_;
        if (dx * dx + dy * dy <= safe_distance * safe_distance)
        {
            return false;
        }
    }
    return true;
}

bool formation::computeDynamicRingOffset(const int agent_index,
                                         const double radius,
                                         double &offset_x,
                                         double &offset_y,
                                         double &offset_z,
                                         double &offset_yaw) const
{
    // 动态圆环阵：
    // 1. 所有智能体均匀分布在圆周上；
    // 2. leader_pos 为圆心；
    // 3. leader_yaw 为整个圆环的初始相位偏置；
    // 4. dynamic_ring_move_speed 为沿圆周的切向速度。
    if (agent_num_ < 2)
    {
        return false;
    }

    if (agent_index < 0 || agent_index >= agent_num_)
    {
        return false;
    }

    const double move_speed = static_cast<double>(current_formation_cmd_.dynamic_ring_move_speed);

    // 圆环半径：
    // 1. 必须大于 0；
    // 2. 相邻智能体弦长必须满足 ORCA 基础安全距离；
    // 3. 整个圆环必须能放进当前场地安全范围。
    const double adjacent_chord =
        2.0 * radius * std::sin(kPi / static_cast<double>(agent_num_));
    if (radius <= 0.0 || adjacent_chord < minSafeDistance() || !canFitDynamicPath(radius, radius))
    {
        return false;
    }

    if (!isDynamicMoveSpeedValid(move_speed))
    {
        return false;
    }

    const double omega = move_speed / radius;
    const double base_phase = 2.0 * kPi * static_cast<double>(agent_index) / static_cast<double>(agent_num_);
    const double phase = static_cast<double>(current_formation_cmd_.leader_yaw) + base_phase + omega * current_formation_time_;

    offset_x = radius * std::cos(phase);
    offset_y = radius * std::sin(phase);
    offset_z = 0.0;
    offset_yaw = (omega >= 0.0) ? (phase + 0.5 * kPi - static_cast<double>(current_formation_cmd_.leader_yaw))
                                : (phase - 0.5 * kPi - static_cast<double>(current_formation_cmd_.leader_yaw));
    return true;
}

bool formation::computeDynamicPolygonOffset(const int agent_index,
                                            const double spacing,
                                            double &offset_x,
                                            double &offset_y,
                                            double &offset_z,
                                            double &offset_yaw) const
{
    // 动态正多边形阵：
    // 1. 智能体数量 N 由 agent_num_ 决定，建议范围 3-10；
    // 2. 所有智能体沿正 N 边形周长循环运动；
    // 3. 不同智能体在周长参数上相差一个边长，因此会均匀分布在不同边段附近；
    // 4. 偏航角始终指向当前沿边运动的速度方向。
    if (agent_num_ < 3 || agent_num_ > 10 || agent_index < 0 || agent_index >= agent_num_)
    {
        return false;
    }

    const double move_speed = static_cast<double>(current_formation_cmd_.dynamic_polygon_move_speed);
    if (!isDynamicMoveSpeedValid(move_speed))
    {
        return false;
    }

    const double polygon_radius = spacing / (2.0 * std::sin(kPi / static_cast<double>(agent_num_)));
    if (spacing < minSafeDistance() || spacing > 3.0 || !canFitDynamicPath(polygon_radius, polygon_radius))
    {
        return false;
    }

    std::vector<double> vertex_x(static_cast<size_t>(agent_num_), 0.0);
    std::vector<double> vertex_y(static_cast<size_t>(agent_num_), 0.0);
    for (int i = 0; i < agent_num_; ++i)
    {
        if (!computeRegularPolygonVertex(i, agent_num_, spacing, vertex_x[static_cast<size_t>(i)],
                                         vertex_y[static_cast<size_t>(i)]))
        {
            return false;
        }
    }

    const double perimeter = static_cast<double>(agent_num_) * spacing;
    double perimeter_distance =
        static_cast<double>(agent_index) * spacing + move_speed * current_formation_time_;
    perimeter_distance = std::fmod(perimeter_distance, perimeter);
    if (perimeter_distance < 0.0)
    {
        perimeter_distance += perimeter;
    }

    int edge_index = static_cast<int>(std::floor(perimeter_distance / spacing));
    if (edge_index >= agent_num_)
    {
        edge_index = agent_num_ - 1;
    }

    const int next_index = (edge_index + 1) % agent_num_;
    const double local_distance = perimeter_distance - static_cast<double>(edge_index) * spacing;
    const double ratio = local_distance / spacing;
    const double start_x = vertex_x[static_cast<size_t>(edge_index)];
    const double start_y = vertex_y[static_cast<size_t>(edge_index)];
    const double end_x = vertex_x[static_cast<size_t>(next_index)];
    const double end_y = vertex_y[static_cast<size_t>(next_index)];
    const double edge_dx = end_x - start_x;
    const double edge_dy = end_y - start_y;

    offset_x = start_x + ratio * edge_dx;
    offset_y = start_y + ratio * edge_dy;
    offset_z = 0.0;
    offset_yaw = normalizeYaw(std::atan2(move_speed * edge_dy, move_speed * edge_dx) -
                              static_cast<double>(current_formation_cmd_.leader_yaw));
    return true;
}

bool formation::computeDynamicLemniscateOffset(const int agent_index,
                                               const double x_radius,
                                               const double y_radius,
                                               const double move_speed,
                                               double &offset_x,
                                               double &offset_y,
                                               double &offset_z,
                                               double &offset_yaw) const
{
    // 动态 Lemniscate / 8 字轨迹：
    // 1. leader_pos 为 8 字轨迹中心；
    // 2. leader_yaw 用作初始相位偏置，不旋转轨迹几何；
    // 3. 不同智能体通过相位错开，沿同一条 8 字轨迹运动；
    // 4. yaw 始终指向当前轨迹切线方向。
    if (agent_num_ < 1 || agent_num_ > 10 || agent_index < 0 || agent_index >= agent_num_)
    {
        return false;
    }

    const double y_extent = 0.5 * y_radius;
    if (x_radius < minSafeDistance() || y_radius < minSafeDistance() ||
        !isDynamicMoveSpeedValid(move_speed) || !canFitDynamicPath(x_radius, y_extent))
    {
        return false;
    }

    const double equivalent_radius = std::max(0.5 * (x_radius + y_radius), 1e-6);
    const double omega = move_speed / equivalent_radius;
    const double base_phase = 2.0 * kPi * static_cast<double>(agent_index) / static_cast<double>(agent_num_);
    const double phase = static_cast<double>(current_formation_cmd_.leader_yaw) + base_phase + omega * current_formation_time_;
    const double sin_phase = std::sin(phase);
    const double cos_phase = std::cos(phase);

    offset_x = x_radius * sin_phase;
    offset_y = y_radius * sin_phase * cos_phase;
    offset_z = 0.0;

    const double vx = x_radius * cos_phase * omega;
    const double vy = y_radius * (cos_phase * cos_phase - sin_phase * sin_phase) * omega;
    offset_yaw = normalizeYaw(std::atan2(vy, vx) - static_cast<double>(current_formation_cmd_.leader_yaw));
    return true;
}

} // namespace swarm_formation
