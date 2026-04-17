// 中文说明：围栏/障碍物构建实现
#include "obstacle_builder.h"
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

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

    // 可视化围栏
    visualization_msgs::Marker fence_marker;
    fence_marker.header.frame_id = "world";
    fence_marker.header.stamp = ros::Time::now();
    fence_marker.ns = "fence";
    fence_marker.type = visualization_msgs::Marker::LINE_STRIP;
    fence_marker.action = visualization_msgs::Marker::ADD;
    fence_marker.scale.x = 0.1;
    fence_marker.color.r = 1.0;
    fence_marker.color.g = 0.0;
    fence_marker.color.b = 0.0;
    fence_marker.color.a = 1.0;
    fence_marker.pose.orientation.w = 1.0;

    geometry_msgs::Point p1, p2, p3, p4;
    p1.x = fence_.max_x + 0.2f;
    p1.y = fence_.max_y;
    p1.z = 1.0;
    p2.x = fence_.max_x;
    p2.y = fence_.max_y;
    p2.z = 1.0;
    p3.x = fence_.max_x;
    p3.y = fence_.min_y;
    p3.z = 1.0;
    p4.x = fence_.max_x + 0.2f;
    p4.y = fence_.min_y;
    p4.z = 1.0;
    fence_marker.points = {p1, p2, p3, p4, p1};
    fence_marker.id = 0;
    marker_array_.markers.push_back(fence_marker);

    p1.x = fence_.max_x;
    p1.y = fence_.max_y + 0.2f;
    p1.z = 1.0;
    p2.x = fence_.min_x;
    p2.y = fence_.max_y + 0.2f;
    p2.z = 1.0;
    p3.x = fence_.min_x;
    p3.y = fence_.max_y;
    p3.z = 1.0;
    p4.x = fence_.max_x;
    p4.y = fence_.max_y;
    p4.z = 1.0;
    fence_marker.points = {p1, p2, p3, p4, p1};
    fence_marker.id = 1;
    marker_array_.markers.push_back(fence_marker);

    p1.x = fence_.min_x;
    p1.y = fence_.max_y;
    p1.z = 1.0;
    p2.x = fence_.min_x - 0.2f;
    p2.y = fence_.max_y;
    p2.z = 1.0;
    p3.x = fence_.min_x - 0.2f;
    p3.y = fence_.min_y;
    p3.z = 1.0;
    p4.x = fence_.min_x;
    p4.y = fence_.min_y;
    p4.z = 1.0;
    fence_marker.points = {p1, p2, p3, p4, p1};
    fence_marker.id = 2;
    marker_array_.markers.push_back(fence_marker);

    p1.x = fence_.max_x;
    p1.y = fence_.min_y;
    p1.z = 1.0;
    p2.x = fence_.min_x;
    p2.y = fence_.min_y;
    p2.z = 1.0;
    p3.x = fence_.min_x;
    p3.y = fence_.min_y - 0.2f;
    p3.z = 1.0;
    p4.x = fence_.max_x;
    p4.y = fence_.min_y - 0.2f;
    p4.z = 1.0;
    fence_marker.points = {p1, p2, p3, p4, p1};
    fence_marker.id = 3;
    marker_array_.markers.push_back(fence_marker);
}

} // namespace orca_swarm
