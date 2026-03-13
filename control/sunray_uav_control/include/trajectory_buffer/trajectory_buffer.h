#include <sys/types.h>

#include <vector>

#include "control_data_types/control_data_types.h"
#include "ros/time.h"

namespace uav_control {

// 1. 构造轨迹容器
struct TrajectoryBundle {
  uint32_t trajectory_id{0};
  ros::Time receive_time;
  ros::Time effective_time;
  std::vector<TrajectoryPoint> points;
};

// 2.构造采样
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

// 3. 构造强类型枚举
enum class TrajectoryBufferStatus : uint8_t {;
  EMPTY = 0,
  READY = 1,
  COMPLETED = 3,
  ABORT = 4,
  ILLEGAL_START = 5,
  ILLEGAL_FINAL = 6,
  IMPOSSIBLE = 7
};

// 4. 双线程缓存管理器
class TrajectoryBufferManager{
	public:
	// 后台写入，提交一条待生效的轨迹
	bool push_pending(TrajectoryBundle traj_){
    if (!check_basic(traj)) {// 如果无法通过基本检查
      // 更新原子变量状态，并使用 relaxed 内存顺序（因为我们不需要跨线程同步这个状态）
      status_.store(TrajectoryBufferStatus::IMPOSSIBLE, std::memory_order_relaxed);
      return false;
    }
    // 通过基本检查后
    // 轨迹ID递增约束（可按需求放宽）
    const uint32_t id = traj.trajectory_id;
    // 读取上次接收的轨迹ID，使用 relaxed 内存顺序（因为我们只需要保证单调递增，不需要跨线程同步）
    const uint32_t last = last_received_id_.load(std::memory_order_relaxed);
    if (id <= last) {
      return false;
    }
    // 缩小锁的范围，只保护对pending队列的访问
    {
      std::lock_guard<std::mutex> lk(pending_mtx_);
      pending_.emplace_back(std::make_shared<const TrajectoryBundle>(std::move(traj)));
    }
    // 不占用锁
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


};




};  // namespace uav_control