#include "sunray_ugv_control/ugv_control_utils.h"

namespace sunray_ugv_control {

// ENU和body坐标系的二维转换
Eigen::Vector2d enu_to_body(const Eigen::Vector2d& enu, double yaw) {
  Eigen::Matrix2d rotation;
  rotation << cos(yaw), -sin(yaw),
              sin(yaw), cos(yaw);
  return rotation * enu;
}

// 检查地理围栏
// 输入：当前位置、xyz范围
// 输出：BOOL
bool check_geo_fence(const Eigen::Vector3d& current_pos, const Eigen::Vector3d& fence_min, const Eigen::Vector3d& fence_max) {
  if (current_pos.x() < fence_min.x() || current_pos.x() > fence_max.x()) {
    return true;
  }
  if (current_pos.y() < fence_min.y() || current_pos.y() > fence_max.y()) {
    return true;
  }
  if (current_pos.z() < fence_min.z() || current_pos.z() > fence_max.z()) {
    return true;
  }
  return false;
}

} // namespace sunray_ugv_control