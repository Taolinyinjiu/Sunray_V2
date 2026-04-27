/*
本程序功能：
    1、定义 FormationContext 结构体，包含 agent_id、agent_num、spacing 等编队上下文参数
    2、定义 Offset2D 结构体，表示归一化偏移量（leader 体坐标系，spacing=1 时的偏移）
    3、定义 FormationPolicy 抽象接口，声明 computeTarget() 虚函数供具体策略实现
    4、定义 OffsetBasedPolicy 基类，实现通用的 computeTarget 流程：查表 → 缩放 → 旋转 → 平移
    5、子类只需实现 generateOffsets() 返回归一化偏移量表即可
*/
#pragma once

#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>

namespace swarm_control
{

// 编队上下文参数
struct FormationContext
{
    int agent_id{1};
    int agent_num{1};
    double spacing{1.0};
};

// 2D 归一化偏移量（虚拟参考中心体坐标系，spacing=1 时的偏移）
struct Offset2D
{
    double x{0.0}; // 前方为正
    double y{0.0}; // 左方为正
};

// 编队策略接口
class FormationPolicy
{
  public:
    virtual ~FormationPolicy() = default;
    // 策略名称
    virtual std::string name() const = 0;
    // 计算目标点；返回 false 表示不产生目标（如 leader 自身）
    virtual bool computeTarget(const geometry_msgs::Pose &reference_pose, const FormationContext &ctx,
                               geometry_msgs::Pose &target_pose) const = 0;
};

// 偏移量表驱动的编队基类
// 子类只需实现 generateOffsets()，通用 computeTarget 负责 缩放→旋转→平移
class OffsetBasedPolicy : public FormationPolicy
{
  public:
    bool computeTarget(const geometry_msgs::Pose &reference_pose, const FormationContext &ctx,
                       geometry_msgs::Pose &target_pose) const override;

  protected:
    // 生成归一化偏移量表（spacing=1），offsets[i] 对应 follower i
    virtual std::vector<Offset2D> generateOffsets(int follower_count) const = 0;
};

} // namespace swarm_control
