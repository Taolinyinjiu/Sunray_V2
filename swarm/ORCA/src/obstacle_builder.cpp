/*
本程序功能：
    1、实现 ObstacleBuilder::init，缓存 ORCA 仿真所需的矩形围栏参数
    2、实现 ObstacleBuilder::apply，将围栏四条边转换为 RVO 多边形障碍物
    3、在仿真器中注册并处理围栏障碍，供 ORCA 避障与越界约束使用
*/
#include "obstacle_builder.h"

namespace orca_swarm
{

void ObstacleBuilder::init(const GeoFence &fence)
{
    fence_ = fence;
}

void ObstacleBuilder::apply(RVO::RVOSimulator *sim)
{
    if (!sim)
    {
        return;
    }

    // 构造四条围栏
    std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;

    obstacle1.push_back(RVO::Vector2(fence_.max_x + 0.2f, fence_.max_y));
    obstacle1.push_back(RVO::Vector2(fence_.max_x, fence_.max_y));
    obstacle1.push_back(RVO::Vector2(fence_.max_x, fence_.min_y));
    obstacle1.push_back(RVO::Vector2(fence_.max_x + 0.2f, fence_.min_y));

    obstacle2.push_back(RVO::Vector2(fence_.max_x, fence_.max_y + 0.2f));
    obstacle2.push_back(RVO::Vector2(fence_.min_x, fence_.max_y + 0.2f));
    obstacle2.push_back(RVO::Vector2(fence_.min_x, fence_.max_y));
    obstacle2.push_back(RVO::Vector2(fence_.max_x, fence_.max_y));

    obstacle3.push_back(RVO::Vector2(fence_.min_x, fence_.max_y));
    obstacle3.push_back(RVO::Vector2(fence_.min_x - 0.2f, fence_.max_y));
    obstacle3.push_back(RVO::Vector2(fence_.min_x - 0.2f, fence_.min_y));
    obstacle3.push_back(RVO::Vector2(fence_.min_x, fence_.min_y));

    obstacle4.push_back(RVO::Vector2(fence_.max_x, fence_.min_y));
    obstacle4.push_back(RVO::Vector2(fence_.min_x, fence_.min_y));
    obstacle4.push_back(RVO::Vector2(fence_.min_x, fence_.min_y - 0.2f));
    obstacle4.push_back(RVO::Vector2(fence_.max_x, fence_.min_y - 0.2f));

    sim->addObstacle(obstacle1);
    sim->addObstacle(obstacle2);
    sim->addObstacle(obstacle3);
    sim->addObstacle(obstacle4);
    sim->processObstacles();
}

} // namespace orca_swarm
