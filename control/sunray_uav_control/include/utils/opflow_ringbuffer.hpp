#pragma once
/**
 * @file opflow_ringbuffer.hpp
 * @brief 光流样本环形缓冲区
 *
 * @details
 * - 在高频回调线程中快速写入光流样本
 * - 在控制线程中读取最新值或窗口均值
 * - 通过短时间窗口降低毛刺对控制的影响
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "px4_manager/px4_data_types.h"

/**
 * @brief 光流环形缓冲区最小存储单元
 */
struct opflow_sample {
  /// 光流时间戳，单位 s
  double timestamp = 0.0;
  /// 光流质量，范围通常为 [0,255]
  uint8_t quality = 0;
  /// 估计的平面速度 x 分量，单位 m/s
  float vx_raw = 0.0f;
  /// 估计的平面速度 y 分量，单位 m/s
  float vy_raw = 0.0f;
  /// 离地高度，单位 m
  float distance = 0.0f;
  /// 本次积分时间，单位 s
  float dt_s = 0.0f;

  /**
   * @brief 默认构造函数（使用成员默认初值）
   */
  opflow_sample() = default;

  /**
   * @brief 从原始光流消息构造样本
   * @param opflow_raw PX4/MAVROS 光流原始数据
   */
  explicit opflow_sample(const px4_data::OpticalFlowRaw &opflow_raw);
};

inline opflow_sample::opflow_sample(const px4_data::OpticalFlowRaw &opflow_raw) {
  timestamp = opflow_raw.timestamp;
  quality = opflow_raw.quality;
  distance = opflow_raw.distance;
  dt_s = static_cast<float>(opflow_raw.integration_time_us * 1e-6);

  // 光流转速度：去旋转补偿 -> 除以 dt -> 乘高度
  if (dt_s > 1e-6f && distance > 0.0f) {
    const float flow_x = opflow_raw.integrated_x - opflow_raw.integrated_xgyro;
    const float flow_y = opflow_raw.integrated_y - opflow_raw.integrated_ygyro;
    vx_raw = (flow_x / dt_s) * distance;
    vy_raw = (flow_y / dt_s) * distance;
  }
};

/**
 * @brief 光流环形缓冲区（线程安全）
 */
class Opflow_Buffer {
public:
  /**
   * @brief 构造环形缓冲区
   * @param capacity 缓冲区容量（样本个数），必须大于 0
   */
  explicit Opflow_Buffer(size_t capacity = 32)
      : buffer_(capacity), capacity_(capacity) {
    if (capacity_ == 0) {
      throw std::invalid_argument("Opflow_Buffer capacity must be > 0");
    }
  }
  /**
   * @brief 写入原始光流数据
   * @param raw 原始光流数据
   * @return true 写入成功；false 数据不合法
   */
  bool push(const px4_data::OpticalFlowRaw &raw) { return push(opflow_sample(raw)); }

  /**
   * @brief 写入样本数据
   * @param sample 样本数据
   * @return true 写入成功；false 数据不合法
   *
   * @details
   * 满容量时覆盖最旧样本。
   */
  bool push(const opflow_sample &sample) {
    // 首先是对数据上锁
    std::lock_guard<std::mutex> lock(mtx_);
    // 如果光流数据无效，则不进行存储
    if (!isSampleValid(sample)) {
      return false;
    }
    // 写入头指针指向位置
    buffer_[head_] = sample;
    // 移动头指针
    head_ = (head_ + 1) % capacity_;
    // 如果环形缓冲区还没有满，则增加count
    if (count_ < capacity_) {
      ++count_;
    }
    return true;
  }

  /**
   * @brief 读取最新样本（不弹出）
   * @param out 输出样本
   * @return true 读取成功；false 缓冲区为空
   */
  bool latest(opflow_sample &out) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (count_ == 0) {
      return false;
    }
    const size_t latest_index = (head_ + capacity_ - 1) % capacity_;
    out = buffer_[latest_index];
    return true;
  }

  /**
   * @brief 返回按时间从旧到新的样本快照
   * @return 样本数组（旧 -> 新）
   */
  std::vector<opflow_sample> snapshot() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<opflow_sample> out;
    out.reserve(count_);
    for (size_t i = 0; i < count_; ++i) {
      out.push_back(buffer_[indexFromOldest(i)]);
    }
    return out;
  }

  /**
   * @brief 计算最近窗口样本均值
   * @param window 窗口长度（样本个数）
   * @param out 输出均值样本
   * @param min_quality 最小质量阈值，小于该阈值的样本将被忽略
   * @return true 计算成功；false 无有效样本
   */
  bool mean(size_t window, opflow_sample &out, uint8_t min_quality = 0) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (count_ == 0 || window == 0) {
      return false;
    }

    const size_t use_n = std::min(window, count_);
    double ts_sum = 0.0;
    float vx_sum = 0.0f;
    float vy_sum = 0.0f;
    float dist_sum = 0.0f;
    float dt_sum = 0.0f;
    uint32_t quality_sum = 0;
    size_t valid_n = 0;

    const size_t start = count_ - use_n;
    for (size_t i = start; i < count_; ++i) {
      const opflow_sample &s = buffer_[indexFromOldest(i)];
      if (s.quality < min_quality) {
        continue;
      }
      ts_sum += s.timestamp;
      vx_sum += s.vx_raw;
      vy_sum += s.vy_raw;
      dist_sum += s.distance;
      dt_sum += s.dt_s;
      quality_sum += s.quality;
      ++valid_n;
    }

    if (valid_n == 0) {
      return false;
    }

    out.timestamp = ts_sum / static_cast<double>(valid_n);
    out.vx_raw = vx_sum / static_cast<float>(valid_n);
    out.vy_raw = vy_sum / static_cast<float>(valid_n);
    out.distance = dist_sum / static_cast<float>(valid_n);
    out.dt_s = dt_sum / static_cast<float>(valid_n);
    out.quality = static_cast<uint8_t>(quality_sum / valid_n);
    return true;
  }

  /**
   * @brief 获取当前有效样本数
   * @return 有效样本数（范围：0 ~ capacity_）
   */
  size_t size() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return count_;
  }

  /**
   * @brief 获取缓冲区总容量
   * @return 容量
   */
  size_t capacity() const { return capacity_; }

  /**
   * @brief 判断缓冲区是否为空
   * @return true 为空；false 非空
   */
  bool empty() const { return size() == 0; }

  /**
   * @brief 清空缓冲区（不释放底层容量）
   */
  void clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    head_ = 0;
    count_ = 0;
  }

private:
  /**
   * @brief 样本合法性检查
   * @param s 输入样本
   * @return true 合法；false 不合法
   */
  static bool isSampleValid(const opflow_sample &s) {
    if (!std::isfinite(s.timestamp) || !std::isfinite(s.vx_raw) ||
        !std::isfinite(s.vy_raw) || !std::isfinite(s.distance) ||
        !std::isfinite(s.dt_s)) {
      return false;
    }
    if (s.dt_s <= 1e-6f || s.distance <= 0.0f) {
      return false;
    }
    return true;
  }

  /**
   * @brief 将“从最旧样本开始的偏移”映射到真实数组下标
   * @param offset 从最旧样本开始的偏移
   * @return 对应数组下标
   */
  size_t indexFromOldest(size_t offset) const {
    const size_t oldest = (head_ + capacity_ - count_) % capacity_;
    return (oldest + offset) % capacity_;
  }

  /// 底层存储容器
  std::vector<opflow_sample> buffer_;
  /// 始终指向下一次写入位置
  size_t head_ = 0;
  /// 当前有效样本数
  size_t count_ = 0;
  /// 环形缓冲区容量（构造后不变）
  const size_t capacity_;
  /// 互斥锁，保护多线程读写
  mutable std::mutex mtx_;
};
