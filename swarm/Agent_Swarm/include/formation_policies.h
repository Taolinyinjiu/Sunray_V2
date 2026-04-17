// 中文说明：编队策略集合声明，所有阵型均基于偏移量表
#pragma once

#include "formation_policy.h"
#include <mutex>

namespace agent_swarm
{

// 圆环阵型（leader 为圆心，follower 均匀分布在圆周上）
class RingPolicy final : public OffsetBasedPolicy
{
  public:
    std::string name() const override
    {
        return "ring";
    }

  protected:
    std::vector<Offset2D> generateOffsets(int follower_count) const override;
};

// 一字横队（follower 交替分布在 leader 左右两侧）
class LinePolicy final : public OffsetBasedPolicy
{
  public:
    std::string name() const override
    {
        return "line";
    }

  protected:
    std::vector<Offset2D> generateOffsets(int follower_count) const override;
};

// 纵队（follower 依次排列在 leader 身后）
class ColumnPolicy final : public OffsetBasedPolicy
{
  public:
    std::string name() const override
    {
        return "column";
    }

  protected:
    std::vector<Offset2D> generateOffsets(int follower_count) const override;
};

// V 形编队 / 雁阵（两翼向后张开，45° 夹角）
class VFormationPolicy final : public OffsetBasedPolicy
{
  public:
    std::string name() const override
    {
        return "v_shape";
    }

  protected:
    std::vector<Offset2D> generateOffsets(int follower_count) const override;
};

// 楔形编队 / 箭头形（两翼向后收拢，约 27° 夹角）
class WedgePolicy final : public OffsetBasedPolicy
{
  public:
    std::string name() const override
    {
        return "wedge";
    }

  protected:
    std::vector<Offset2D> generateOffsets(int follower_count) const override;
};

// 自定义阵型（外部输入偏移量表）
class CustomPolicy final : public OffsetBasedPolicy
{
  public:
    std::string name() const override
    {
        return "custom";
    }

    void setOffsets(const std::vector<Offset2D> &offsets);

  protected:
    std::vector<Offset2D> generateOffsets(int follower_count) const override;

  private:
    mutable std::mutex mutex_{};
    std::vector<Offset2D> offsets_{};
};

} // namespace agent_swarm
