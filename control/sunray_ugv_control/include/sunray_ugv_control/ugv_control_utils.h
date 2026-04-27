#pragma once

#include <algorithm>
#include <cmath>

namespace sunray_ugv_control {

// 控制器内部使用的轻量二维向量，避免为简单平面运算引入 Eigen 依赖。
struct Vector2 {
    double x{0.0};
    double y{0.0};
};

// 低频监督循环使用的局部坐标系轴对齐围栏。
struct GeoFence {
    double x_min{0.0};
    double x_max{0.0};
    double y_min{0.0};
    double y_max{0.0};
    double z_min{0.0};
    double z_max{0.0};
};

inline double clamp(const double value, const double min_value, const double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

inline double normalize_angle(double angle_rad) {
    while (angle_rad > M_PI) {
        angle_rad -= 2.0 * M_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += 2.0 * M_PI;
    }
    return angle_rad;
}

// 将 ENU 平面向量旋转到当前车体系。
inline Vector2 enu_to_body(const Vector2& enu, const double yaw_rad) {
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);

    Vector2 body;
    body.x = c * enu.x + s * enu.y;
    body.y = -s * enu.x + c * enu.y;
    return body;
}

// 将车体系平面向量旋转回 ENU 坐标系。
inline Vector2 body_to_enu(const Vector2& body, const double yaw_rad) {
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);

    Vector2 enu;
    enu.x = c * body.x - s * body.y;
    enu.y = s * body.x + c * body.y;
    return enu;
}

inline bool check_geo_fence(const double x,
                            const double y,
                            const double z,
                            const GeoFence& fence) {
    return x >= fence.x_min && x <= fence.x_max && y >= fence.y_min && y <= fence.y_max &&
           z >= fence.z_min && z <= fence.z_max;
}

}  // namespace sunray_ugv_control
