/*
本程序功能：
    1、实现 OffsetBasedPolicy::computeTarget 通用流程：查表 → 缩放 → 旋转 → 平移
    2、实现 RingPolicy：圆环阵型，follower 均匀分布在圆周上，归一化使相邻弦长=1
    3、实现 LinePolicy：一字横队，follower 交替分布在 leader 左右
    4、实现 ColumnPolicy：纵队，follower 依次排列在 leader 正后方
    5、实现 VFormationPolicy：V 形/雁阵，两翼向后张开 45° 夹角
    6、实现 WedgePolicy：楔形，两翼收拢约 27° 夹角
    7、实现 CustomPolicy：外部输入偏移量表，线程安全读写，不足时用 Ring 补齐
*/
#include "formation_policies.h"
#include <cmath>
#include <tf/transform_datatypes.h>

namespace swarm_control
{

// ========== 工具函数 ==========

namespace
{

int followerIndex(const FormationContext &ctx)
{
    return ctx.agent_id - 1;
}

std::vector<Offset2D> generateRingOffsets(int n)
{
    std::vector<Offset2D> offsets(n);
    double radius = 1.0;
    if (n >= 2)
    {
        radius = 1.0 / (2.0 * std::sin(M_PI / n));
    }
    for (int i = 0; i < n; ++i)
    {
        double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
        offsets[i] = {radius * std::cos(angle), radius * std::sin(angle)};
    }
    return offsets;
}

void centerOffsets(std::vector<Offset2D> &offsets)
{
    if (offsets.empty())
    {
        return;
    }
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const auto &offset : offsets)
    {
        sum_x += offset.x;
        sum_y += offset.y;
    }
    const double mean_x = sum_x / static_cast<double>(offsets.size());
    const double mean_y = sum_y / static_cast<double>(offsets.size());
    for (auto &offset : offsets)
    {
        offset.x -= mean_x;
        offset.y -= mean_y;
    }
}

} // namespace

// ========== OffsetBasedPolicy 通用 computeTarget ==========

bool OffsetBasedPolicy::computeTarget(const geometry_msgs::Pose &reference_pose, const FormationContext &ctx,
                                      geometry_msgs::Pose &target_pose) const
{
    int idx = followerIndex(ctx);
    if (idx < 0)
    {
        return false;
    }
    int followers = ctx.agent_num;
    if (followers <= 0)
    {
        return false;
    }

    // 1. 查表（归一化偏移，spacing=1）
    std::vector<Offset2D> offsets = generateOffsets(followers);
    if (idx >= static_cast<int>(offsets.size()))
    {
        return false;
    }

    // 2. 缩放
    double ox = offsets[idx].x * ctx.spacing;
    double oy = offsets[idx].y * ctx.spacing;

    // 3. 旋转到世界系（按虚拟参考中心朝向）
    double yaw = tf::getYaw(reference_pose.orientation);
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    double rx = c * ox - s * oy;
    double ry = s * ox + c * oy;

    // 4. 平移
    target_pose = reference_pose;
    target_pose.position.x += rx;
    target_pose.position.y += ry;
    return true;
}

// ========== Ring —— 圆环 ==========
//
//  所有 agent 均匀分布在以虚拟参考中心为圆心的圆周上。
//  归一化半径使得相邻 follower 的弦长 = 1（乘 spacing 后 = spacing）。
//
//       2
//     /   \
//   3       1
//     \   /
//       4
//
std::vector<Offset2D> RingPolicy::generateOffsets(int n) const
{
    return generateRingOffsets(n);
}

// ========== Line —— 一字横队 ==========
//
//  所有 agent 围绕虚拟参考中心横向连续分布，垂直于朝向方向。
//  编号 0 在左侧 y=+1，编号 1 在右侧 y=-1，编号 2 在 y=+2 ……
//
//   2  0  L  1  3
//
std::vector<Offset2D> LinePolicy::generateOffsets(int n) const
{
    std::vector<Offset2D> offsets(n);
    const double center = 0.5 * static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i)
    {
        offsets[i] = {0.0, static_cast<double>(i) - center};
    }
    return offsets;
}

// ========== Column —— 纵队 ==========
//
//  所有 agent 围绕虚拟参考中心纵向连续分布。
//
//   L
//   0
//   1
//   2
//
std::vector<Offset2D> ColumnPolicy::generateOffsets(int n) const
{
    std::vector<Offset2D> offsets(n);
    const double center = 0.5 * static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i)
    {
        offsets[i] = {center - static_cast<double>(i), 0.0};
    }
    return offsets;
}

// ========== V-shape —— V 形 / 雁阵 ==========
//
//  两翼向后张开，每翼与纵轴夹 45°。
//  编号交替分配到左右翼。
//
//         L
//       0   1
//     2       3
//   4           5
//
std::vector<Offset2D> VFormationPolicy::generateOffsets(int n) const
{
    std::vector<Offset2D> offsets(n);
    for (int i = 0; i < n; ++i)
    {
        int rank = i / 2 + 1;
        double sign = (i % 2 == 0) ? 1.0 : -1.0;
        offsets[i] = {-static_cast<double>(rank), sign * static_cast<double>(rank)};
    }
    centerOffsets(offsets);
    return offsets;
}

// ========== Wedge —— 楔形 ==========
//
//  类似 V 形但两翼更收拢（横向 = 纵向 × 0.5，夹角 ≈27°）。
//  适合前突探测场景。
//
//        L
//       0 1
//      2   3
//     4     5
//
std::vector<Offset2D> WedgePolicy::generateOffsets(int n) const
{
    std::vector<Offset2D> offsets(n);
    for (int i = 0; i < n; ++i)
    {
        int rank = i / 2 + 1;
        double sign = (i % 2 == 0) ? 1.0 : -1.0;
        offsets[i] = {-static_cast<double>(rank), sign * static_cast<double>(rank) * 0.5};
    }
    centerOffsets(offsets);
    return offsets;
}

void CustomPolicy::setOffsets(const std::vector<Offset2D> &offsets)
{
    std::lock_guard<std::mutex> lock(mutex_);
    offsets_ = offsets;
}

std::vector<Offset2D> CustomPolicy::generateOffsets(int n) const
{
    std::vector<Offset2D> offsets;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        offsets = offsets_;
    }
    if (static_cast<int>(offsets.size()) > n)
    {
        offsets.resize(n);
    }
    if (static_cast<int>(offsets.size()) < n)
    {
        std::vector<Offset2D> fallback = generateRingOffsets(n - static_cast<int>(offsets.size()));
        offsets.insert(offsets.end(), fallback.begin(), fallback.end());
    }
    return offsets;
}

} // namespace swarm_control
