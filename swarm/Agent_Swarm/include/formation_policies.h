/*
本程序功能：
    1、定义 6 种具体编队策略类：RingPolicy、LinePolicy、ColumnPolicy、VFormationPolicy、WedgePolicy、CustomPolicy
    2、每种策略继承 OffsetBasedPolicy，只需实现 generateOffsets() 返回归一化偏移量表
    3、Ring 圆环阵型：follower 均匀分布在以 leader 为圆心的圆周上
    4、Line 一字横队：follower 交替排列在 leader 左右两侧
    5、Column 纵队：follower 依次排列在 leader 正后方
    6、VFormation 雁阵：两翼向后张开 45° 夹角
    7、Wedge 楔形：两翼向后收拢约 27° 夹角
    8、Custom 自定义阵型：由外部（TUI）输入偏移量表，不足时用 Ring 补齐
*/
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
