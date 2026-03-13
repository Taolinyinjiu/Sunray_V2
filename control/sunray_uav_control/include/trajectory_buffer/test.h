// 仅设计草案，不直接改你现有代码
// 目标：基于你现有 TrajectoryPoint + Base_Controller 设计双缓冲轨迹管理

#pragma once
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <ros/time.h>
#include <Eigen/Dense>

#include "control_data_types/control_data_types.h"

namespace uav_control {

// -----------------------------
// 1) 轨迹容器（不改 TrajectoryPoint）
// -----------------------------
struct TrajectoryBundle {
  uint32_t trajectory_id{0};                  // 轨迹级ID（建议新增，避免依赖点内ID）
  ros::Time receive_time;                     // 接收时刻
  ros::Time effective_time;                   // 生效时刻（切换时机）
  std::vector<TrajectoryPoint> points;        // 按 time_frome_start 升序
};

// -----------------------------
// 2) 采样结果（给控制器 update 用）
// -----------------------------
struct TrajectorySample {
  bool valid{false};
  bool reached_end{false};

  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_acc{0.0};

  uint32_t trajectory_id{0};
};

// -----------------------------
// 3) 状态码（可映射你消息里的 trajectory_flag）
// -----------------------------
enum class TrajectoryBufferStatus : uint8_t {
  EMPTY = 0,
  READY = 1,
  COMPLETED = 3,
  ABORT = 4,
  ILLEGAL_START = 5,
  ILLEGAL_FINAL = 6,
  IMPOSSIBLE = 7
};

// -----------------------------
// 4) 双缓冲管理器
//    - 后台线程: push_pending()
//    - 前台控制环: update_active()/sample()
// -----------------------------
class TrajectoryBufferManager {
public:
  // 后台写入：提交一条待生效轨迹
  bool push_pending(TrajectoryBundle traj) {
    if (!validate_basic(traj)) {
      status_.store(TrajectoryBufferStatus::IMPOSSIBLE, std::memory_order_relaxed);
      return false;
    }

    // 轨迹ID递增约束（可按需求放宽）
    const uint32_t id = traj.trajectory_id;
    const uint32_t last = last_received_id_.load(std::memory_order_relaxed);
    if (id <= last) {
      return false;
    }

    {
      std::lock_guard<std::mutex> lk(pending_mtx_);
      pending_.emplace_back(std::make_shared<const TrajectoryBundle>(std::move(traj)));
    }
    last_received_id_.store(id, std::memory_order_relaxed);
    return true;
  }

  // 前台周期调用：到时切换 pending -> active
  void update_active(const ros::Time& now) {
    std::lock_guard<std::mutex> lk(pending_mtx_);
    while (!pending_.empty()) {
      const auto& cand = pending_.front();
      if (cand->effective_time > now) break;

      // 原子切换活动轨迹（前台无锁读）
      std::atomic_store(&active_, cand);
      active_id_.store(cand->trajectory_id, std::memory_order_relaxed);
      status_.store(TrajectoryBufferStatus::READY, std::memory_order_relaxed);
      pending_.pop_front();
    }
  }

  // 前台读取当前轨迹并按当前时刻采样
  TrajectorySample sample(const ros::Time& now) const {
    auto active = std::atomic_load(&active_);
    if (!active) {
      return {};
    }

    const auto& pts = active->points;
    if (pts.empty()) {
      return {};
    }

    const double t = (now - active->effective_time).toSec();
    if (t < 0.0) {
      return {};
    }

    // 末端保持
    const double t_end = pts.back().time_frome_start.toSec();
    if (t >= t_end) {
      TrajectorySample out = from_point(pts.back(), active->trajectory_id);
      out.valid = true;
      out.reached_end = true;
      return out;
    }

    // 找区间 [i, i+1]
    size_t i = 0;
    while (i + 1 < pts.size() && pts[i + 1].time_frome_start.toSec() < t) {
      ++i;
    }

    if (i + 1 >= pts.size()) {
      TrajectorySample out = from_point(pts.back(), active->trajectory_id);
      out.valid = true;
      return out;
    }

    const auto& p0 = pts[i];
    const auto& p1 = pts[i + 1];

    const double t0 = p0.time_frome_start.toSec();
    const double t1 = p1.time_frome_start.toSec();
    const double u = (t1 > t0) ? clamp01((t - t0) / (t1 - t0)) : 0.0;

    TrajectorySample out;
    out.valid = true;
    out.trajectory_id = active->trajectory_id;
    out.position = lerp(p0.position, p1.position, u);
    out.velocity = lerp(p0.velocity, p1.velocity, u);
    out.acceleration = lerp(p0.acceleration, p1.acceleration, u);
    out.jerk = lerp(p0.jerk, p1.jerk, u);
    out.snap = lerp(p0.snap, p1.snap, u);
    out.yaw = lerp_scalar(p0.yaw, p1.yaw, u);
    out.yaw_rate = lerp_scalar(p0.yaw_rate, p1.yaw_rate, u);
    out.yaw_acc = lerp_scalar(p0.yaw_acc, p1.yaw_acc, u);
    return out;
  }

  void mark_completed() { status_.store(TrajectoryBufferStatus::COMPLETED, std::memory_order_relaxed); }
  void mark_abort() { status_.store(TrajectoryBufferStatus::ABORT, std::memory_order_relaxed); }

  TrajectoryBufferStatus status() const { return status_.load(std::memory_order_relaxed); }
  uint32_t active_id() const { return active_id_.load(std::memory_order_relaxed); }

private:
  static bool validate_basic(const TrajectoryBundle& traj) {
    if (traj.points.empty()) return false;

    // 时间单调性检查
    double last_t = -1e9;
    for (const auto& p : traj.points) {
      const double t = p.time_frome_start.toSec();
      if (t < 0.0) return false;
      if (t < last_t) return false;
      last_t = t;
    }
    return true;
  }

  static TrajectorySample from_point(const TrajectoryPoint& p, uint32_t id) {
    TrajectorySample s;
    s.trajectory_id = id;
    s.position = p.position;
    s.velocity = p.velocity;
    s.acceleration = p.acceleration;
    s.jerk = p.jerk;
    s.snap = p.snap;
    s.yaw = p.yaw;
    s.yaw_rate = p.yaw_rate;
    s.yaw_acc = p.yaw_acc;
    return s;
  }

  static Eigen::Vector3d lerp(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double u) {
    return (1.0 - u) * a + u * b;
  }

  static double lerp_scalar(double a, double b, double u) {
    return (1.0 - u) * a + u * b;
  }

  static double clamp01(double x) {
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
  }

private:
  // pending队列（后台写、前台切换时访问）
  mutable std::mutex pending_mtx_;
  std::deque<std::shared_ptr<const TrajectoryBundle>> pending_;

  // active轨迹（前台高频无锁读）
  std::shared_ptr<const TrajectoryBundle> active_{nullptr};

  std::atomic<uint32_t> active_id_{0};
  std::atomic<uint32_t> last_received_id_{0};
  std::atomic<TrajectoryBufferStatus> status_{TrajectoryBufferStatus::EMPTY};
};

} // namespace uav_control