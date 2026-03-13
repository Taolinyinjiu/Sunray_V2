#pragma once

#include <Eigen/Dense>

namespace uav_control {
namespace curve {

/**
 * @file quintic_curve.hpp
 * @brief 五次多项式轨迹评估与时长反解接口。
 *
 * @details
 * 本文件提供两类能力：
 * - 在给定起止状态与时长下，评估五次曲线在任意时刻的位置、速度、加速度、jerk、snap；
 * - 在端点速度与端点加速度均为 0 的约束下，根据最大速度上限反推最小可行时长。
 *
 * 五次曲线模型定义为：
 * @f[
 * p(t)=c_0+c_1 t+c_2 t^2+c_3 t^3+c_4 t^4+c_5 t^5,\quad t\in[0,T]
 * @f]
 *
 * 其中本模块的默认边界条件为：
 * @f[
 * p(0)=p_0,\ v(0)=v_0,\ a(0)=0,\ p(T)=p_f,\ v(T)=v_f,\ a(T)=0
 * @f]
 *
 * 在该约束下每个轴均有唯一解，三维轨迹按轴独立求解后组合。
 */

/**
 * @brief 五次曲线在某一时刻的评估结果。
 *
 * @details
 * 该结构是 `evaluate_quintic_curve()` 的返回值容器。
 * 当 `valid=false` 时，除 `elapsed_s/normalized_time/clamped` 以外的状态字段
 * 不应作为控制输入使用。
 *
 * @note
 * `elapsed_s` 与 `normalized_time` 基于时间裁剪后的时刻计算：
 * - 若 `current_time_s < start_time_s`，按 `t=0` 输出；
 * - 若 `current_time_s > start_time_s + duration_s`，按 `t=T` 输出。
 */
struct QuinticCurveState {
  /** @brief 结果是否有效。仅当输入合法且全部输出为有限值时为 true。 */
  bool valid{false};
  /** @brief 是否发生时间裁剪。若当前时刻落在轨迹区间外则为 true。 */
  bool clamped{false};
  /** @brief 从起始时刻计算的裁剪后历时，单位 s，范围 `[0, duration_s]`。 */
  double elapsed_s{0.0};
  /** @brief 归一化时间 `elapsed_s / duration_s`，范围 `[0, 1]`。 */
  double normalized_time{0.0};
  /** @brief 位置 `p(t)`，单位 m。 */
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  /** @brief 速度 `v(t)=\dot p(t)`，单位 m/s。 */
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  /** @brief 加速度 `a(t)=\ddot p(t)`，单位 m/s^2。 */
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  /** @brief jerk `j(t)=\dddot p(t)`，单位 m/s^3。 */
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  /** @brief snap `s(t)=p^{(4)}(t)`，单位 m/s^4。 */
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
};

/**
 * @brief 最大速度约束下的最小时长反解结果。
 *
 * @details
 * 对于约束
 * @f[
 * v(0)=v(T)=0,\ a(0)=a(T)=0
 * @f]
 * 的 quintic smoothstep 轨迹，其速度峰值满足：
 * @f[
 * v_{peak}=\frac{15}{8}\frac{\|\Delta p\|}{T}
 * @f]
 * 因此最小可行时长：
 * @f[
 * T_{min}=\frac{15}{8}\frac{\|\Delta p\|}{v_{max}}
 * @f]
 * 本结构用于返回该反解结果及其有效性。
 */
struct QuinticCurveMinDuration {
  /** @brief 结果是否有效。输入非法或计算异常时为 false。 */
  bool valid{false};
  /** @brief 满足最大速度约束的最小运动时间，单位 s。 */
  double min_duration_s{0.0};
};

/**
 * @brief 评估五次曲线在指定时刻的状态。
 *
 * @details
 * 该接口按三轴独立求解 quintic 系数，再合并为三维状态输出。
 * 使用的约束为：
 * @f[
 * p(0)=p_0,\ v(0)=v_0,\ a(0)=0,\ p(T)=p_f,\ v(T)=v_f,\ a(T)=0
 * @f]
 *
 * 时间处理规则：
 * - 先计算 `elapsed = current_time_s - start_time_s`；
 * - 将 `elapsed` 裁剪到 `[0, duration_s]`；
 * - 在裁剪后的 `t` 上输出状态，并通过 `clamped` 标记是否发生裁剪。
 *
 * 输入合法性要求：
 * - `duration_s > 0`；
 * - 所有标量均为有限值；
 * - 所有向量分量均为有限值。
 *
 * @param start_position 起点位置 @f$p_0@f$，单位 m。
 * @param start_velocity 起点速度 @f$v_0@f$，单位 m/s。
 * @param end_position 终点位置 @f$p_f@f$，单位 m。
 * @param end_velocity 终点速度 @f$v_f@f$，单位 m/s。
 * @param start_time_s 轨迹起始时间戳，单位 s（统一时间基准）。
 * @param duration_s 轨迹总时长 @f$T@f$，单位 s，必须大于 0。
 * @param current_time_s 需要评估的时间戳，单位 s（统一时间基准）。
 * @return QuinticCurveState
 * 返回当前时刻的 `position/velocity/acceleration/jerk/snap` 以及时间信息。
 * 输入非法时返回 `valid=false`。
 *
 * @warning
 * 本函数默认端点加速度均为 0。若业务需要非零端点加速度，请使用独立接口。
 *
 * @code{.cpp}
 * const auto state = uav_control::curve::evaluate_quintic_curve(
 *     start_pos, start_vel, end_pos, end_vel, t0_s, duration_s, now_s);
 * if (state.valid) {
 *   // consume state.position / state.velocity ...
 * }
 * @endcode
 */
QuinticCurveState evaluate_quintic_curve(
    const Eigen::Vector3d &start_position,
    const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &end_position,
    const Eigen::Vector3d &end_velocity, double start_time_s,
    double duration_s, double current_time_s);

/**
 * @brief 在零端点速度/加速度条件下，根据最大速度约束反推最小时长。
 *
 * @details
 * 该函数用于“先给速度上限，再估计轨迹最短时长”的场景，假设：
 * @f[
 * v(0)=v(T)=0,\ a(0)=a(T)=0
 * @f]
 * 则最小时长由下式直接给出：
 * @f[
 * T_{min}=\frac{15}{8}\frac{\|end\_position-start\_position\|}{max\_speed\_mps}
 * @f]
 *
 * 其中位移范数为欧氏距离。若位移为 0，则 `min_duration_s=0`。
 *
 * @param start_position 起点位置，单位 m。
 * @param end_position 终点位置，单位 m。
 * @param max_speed_mps 速度上限，单位 m/s，必须大于 0 且有限。
 * @return QuinticCurveMinDuration
 * - `valid=true`：返回可用的 `min_duration_s`；
 * - `valid=false`：输入非法（如 `max_speed_mps<=0` 或存在 NaN/Inf）。
 */
QuinticCurveMinDuration solve_quintic_min_duration_from_max_speed(
    const Eigen::Vector3d &start_position, const Eigen::Vector3d &end_position,
    double max_speed_mps);

} // namespace curve
} // namespace uav_control
