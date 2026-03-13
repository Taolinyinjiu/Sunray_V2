#pragma once
/**
        @brief 本文件主要用于各曲线生成器，构建标准的输出结构体
 */

#include <Eigen/Dense>

namespace uav_control {

struct Curve_Output {
	// 曲线是否正确生成
	bool curve_status;
  // 位置
  Eigen::Vector3d position;
  // 速度
  Eigen::Vector3d velocity;
  // 加速度
  Eigen::Vector3d acceleration;
};

}; // namespace uav_control