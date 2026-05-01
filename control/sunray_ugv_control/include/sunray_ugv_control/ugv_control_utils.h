#ifndef UGV_CONTROL_UTILS_H
#define UGV_CONTROL_UTILS_H

#include <Eigen/Eigen>

namespace sunray_ugv_control {

// ENU和body坐标系的二维转换
Eigen::Vector2d enu_to_body(const Eigen::Vector2d& enu, double yaw);

// 检查地理围栏
// 输入：当前位置、xyz范围
// 输出：BOOL
bool check_geo_fence(const Eigen::Vector3d& current_pos, const Eigen::Vector3d& fence_min, const Eigen::Vector3d& fence_max);

} // namespace sunray_ugv_control

#endif // UGV_CONTROL_UTILS_H