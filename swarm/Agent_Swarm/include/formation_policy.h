// 中文说明：编队策略接口与上下文参数定义，偏移量表驱动基类
#pragma once

#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>

namespace agent_swarm
{

// 编队上下文参数
struct FormationContext
{
    int agent_id{1};
    int agent_num{1};
    int leader_id{1};
    double spacing{1.0};
};

// 2D 归一化偏移量（leader 体坐标系，spacing=1 时的偏移）
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
    virtual bool computeTarget(const geometry_msgs::Pose &leader_pose, const FormationContext &ctx,
                               geometry_msgs::Pose &target_pose) const = 0;
};

// 偏移量表驱动的编队基类
// 子类只需实现 generateOffsets()，通用 computeTarget 负责 缩放→旋转→平移
class OffsetBasedPolicy : public FormationPolicy
{
  public:
    bool computeTarget(const geometry_msgs::Pose &leader_pose, const FormationContext &ctx,
                       geometry_msgs::Pose &target_pose) const override;

  protected:
    // 生成归一化偏移量表（spacing=1），offsets[i] 对应 follower i
    virtual std::vector<Offset2D> generateOffsets(int follower_count) const = 0;
};

} // namespace agent_swarm
