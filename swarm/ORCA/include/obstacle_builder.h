/*
本程序功能：
    1、定义 GeoFence 结构体，描述 ORCA 仿真使用的矩形边界范围
    2、定义 ObstacleBuilder 类，将围栏参数转换为 RVO 可识别的静态障碍物
    3、为 ORCA 引擎提供统一的围栏初始化与障碍物注入接口
*/
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
