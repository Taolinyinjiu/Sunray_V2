// 中文说明：围栏/障碍物构建接口与可视化
#pragma once

#include "RVO.h"

namespace orca_swarm
{

// 地理围栏参数
struct GeoFence
{
    float min_x{-5.0f};
    float max_x{5.0f};
    float min_y{-5.0f};
    float max_y{5.0f};
};

// 障碍物构建器
class ObstacleBuilder
{
  public:
    // 初始化围栏参数
    void init(const GeoFence &fence);
    // 应用到 RVO 仿真
    void apply(RVO::RVOSimulator *sim);

  private:
    GeoFence fence_{};
};

} // namespace orca_swarm
