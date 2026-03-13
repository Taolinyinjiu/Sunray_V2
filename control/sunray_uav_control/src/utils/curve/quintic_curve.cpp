#include "utils/curve/quintic_curve.hpp"

#include <algorithm>
#include <cmath>

namespace uav_control {
namespace curve {
namespace {

/**
 * @brief 判断标量是否为有限值。
 * @param value 待检查标量。
 * @return true 表示不是 NaN/Inf。
 */
bool is_finite_scalar(double value) { return std::isfinite(value); }

/**
 * @brief 判断三维向量所有分量是否为有限值。
 * @param value 待检查向量。
 * @return true 表示所有分量均有限。
 */
bool is_finite_vector(const Eigen::Vector3d &value) { return value.allFinite(); }

/**
 * @brief 零端点速度/加速度 quintic 轨迹的速度峰值比例系数。
 *
 * @details
 * 对 `q(s)=10s^3-15s^4+6s^5`（`s=t/T`）有
 * `max |dq/ds| = 15/8`，故三维位移范数意义下：
 * `v_peak = (15/8) * ||Δp|| / T`。
 */
constexpr double kZeroVelAccQuinticPeakVelocityScale = 15.0 / 8.0;

} // namespace

/**
 * @brief 评估五次轨迹在给定时刻的运动状态。
 *
 * @details
 * 实现与头文件文档保持一致：输入时间先裁剪，再在裁剪时刻计算
 * `position/velocity/acceleration/jerk/snap`。
 */
QuinticCurveState evaluate_quintic_curve(
    const Eigen::Vector3d &start_position,
    const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &end_position, const Eigen::Vector3d &end_velocity,
    double start_time_s, double duration_s, double current_time_s) {
  QuinticCurveState output;
  if (!is_finite_scalar(start_time_s) || !is_finite_scalar(duration_s) ||
      !is_finite_scalar(current_time_s) || duration_s <= 0.0 ||
      !is_finite_vector(start_position) || !is_finite_vector(start_velocity) ||
      !is_finite_vector(end_position) || !is_finite_vector(end_velocity)) {
    return output;
  }

  const double elapsed_s = current_time_s - start_time_s;
  const double clamped_elapsed_s =
      std::max(0.0, std::min(duration_s, elapsed_s));
  const double t = clamped_elapsed_s;
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;

  const double T = duration_s;
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;

  const Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();
  const Eigen::Vector3d end_acc = Eigen::Vector3d::Zero();

  const Eigen::Vector3d c0 = start_position;
  const Eigen::Vector3d c1 = start_velocity;
  const Eigen::Vector3d c2 = 0.5 * start_acc;
  const Eigen::Vector3d delta_p = end_position - start_position;

  const Eigen::Vector3d c3 =
      (20.0 * delta_p - (8.0 * end_velocity + 12.0 * start_velocity) * T -
       (3.0 * start_acc - end_acc) * T2) /
      (2.0 * T3);
  const Eigen::Vector3d c4 =
      (30.0 * (-delta_p) + (14.0 * end_velocity + 16.0 * start_velocity) * T +
       (3.0 * start_acc - 2.0 * end_acc) * T2) /
      (2.0 * T4);
  const Eigen::Vector3d c5 =
      (12.0 * delta_p - (6.0 * end_velocity + 6.0 * start_velocity) * T -
       (start_acc - end_acc) * T2) /
      (2.0 * T5);

  output.position = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
  output.velocity =
      c1 + 2.0 * c2 * t + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4;
  output.acceleration =
      2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t2 + 20.0 * c5 * t3;
  output.jerk = 6.0 * c3 + 24.0 * c4 * t + 60.0 * c5 * t2;
  output.snap = 24.0 * c4 + 120.0 * c5 * t;
  output.elapsed_s = clamped_elapsed_s;
  output.normalized_time = clamped_elapsed_s / duration_s;
  output.clamped = (std::abs(clamped_elapsed_s - elapsed_s) > 1e-9);
  output.valid = is_finite_vector(output.position) &&
                 is_finite_vector(output.velocity) &&
                 is_finite_vector(output.acceleration) &&
                 is_finite_vector(output.jerk) &&
                 is_finite_vector(output.snap);
  return output;
}

/**
 * @brief 根据最大速度上限反解零边界 quintic 的最小运动时长。
 *
 * @details
 * 采用解析式：
 * `T_min = (15/8) * ||end - start|| / max_speed_mps`。
 */
QuinticCurveMinDuration solve_quintic_min_duration_from_max_speed(
    const Eigen::Vector3d &start_position, const Eigen::Vector3d &end_position,
    double max_speed_mps) {
  QuinticCurveMinDuration output;
  if (!is_finite_vector(start_position) || !is_finite_vector(end_position) ||
      !is_finite_scalar(max_speed_mps) || max_speed_mps <= 0.0) {
    return output;
  }

  const double displacement = (end_position - start_position).norm();
  if (!is_finite_scalar(displacement)) {
    return output;
  }

  output.min_duration_s =
      kZeroVelAccQuinticPeakVelocityScale * displacement / max_speed_mps;
  output.valid = is_finite_scalar(output.min_duration_s) &&
                 output.min_duration_s >= 0.0;
  if (!output.valid) {
    output.min_duration_s = 0.0;
  }
  return output;
}

} // namespace curve
} // namespace uav_control
