/**
 * @file localization_fusion_utils.hpp
 * @brief localization_fusion 的辅助函数（与 types 分离）
 *
 * 使用说明：
 * 1) 先包含本头文件和 localization_fusion_types.hpp；
 * 2) 读取 yaml 字段后，使用 ParseLoopMode/ParsePublishMode 将字符串转为枚举；
 * 3) 使用 AddLoopIfMissing 维护 supported_loops，避免重复；
 * 4) 使用 HasLocalTopic/HasGlobalTopic/HasLoopTopic 判断 topic 是否可用；
 * 5) 打印日志时使用 ToString(LoopMode/PublishMode) 统一输出格式。
 *
 * 约定：
 * - 当 yaml 中 topic 写成 null 或空串时，解析后按空字符串处理；
 * - Parse* 函数遇到未知字符串会抛出 std::invalid_argument，建议在上层捕获并报错退出。
 */

#pragma once

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

#include "localization_fusion_types.hpp"

namespace localization_fusion_types {

// 将字符串转为小写，便于枚举解析时做大小写无关匹配。
inline std::string ToLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

// 解析 loop 字符串。
// 支持别名: disable/off, lidar_loop/lidar, aruco_loop/aruco
// 异常: 未识别输入会抛出 std::invalid_argument
inline LoopMode ParseLoopMode(const std::string& raw) {
  const std::string s = ToLower(raw);
  if (s == "disable" || s == "off") return LoopMode::DISABLE;
  if (s == "lidar_loop" || s == "lidar") return LoopMode::LIDAR_LOOP;
  if (s == "aruco_loop" || s == "aruco") return LoopMode::ARUCO_LOOP;
  throw std::invalid_argument("Unknown LoopMode: " + raw);
}

// 解析 publish_mode 字符串。
// 支持别名:
// - SINGLE_COPY: single_copy/single_mirror/direct_copy
// - DUAL_DIRECT: dual_direct
// - INIT_TRANSFORM: init_transform/dual_bootstrap
// 异常: 未识别输入会抛出 std::invalid_argument
inline PublishMode ParsePublishMode(const std::string& raw) {
  const std::string s = ToLower(raw);
  if (s == "single_copy" || s == "single_mirror" || s == "direct_copy") {
    return PublishMode::SINGLE_COPY;
  }
  if (s == "dual_direct") return PublishMode::DUAL_DIRECT;
  if (s == "init_transform" || s == "dual_bootstrap") {
    return PublishMode::INIT_TRANSFORM;
  }
  throw std::invalid_argument("Unknown PublishMode: " + raw);
}

// 枚举转字符串（用于日志打印和调试输出）。
inline const char* ToString(LoopMode mode) {
  switch (mode) {
    case LoopMode::DISABLE: return "disable";
    case LoopMode::LIDAR_LOOP: return "lidar_loop";
    case LoopMode::ARUCO_LOOP: return "aruco_loop";
    default: return "unknown";
  }
}

// 枚举转字符串（用于日志打印和调试输出）。
inline const char* ToString(PublishMode mode) {
  switch (mode) {
    case PublishMode::SINGLE_COPY: return "single_copy";
    case PublishMode::DUAL_DIRECT: return "dual_direct";
    case PublishMode::INIT_TRANSFORM: return "init_transform";
    default: return "unknown";
  }
}

// 检查某个 loop 是否在 capabilities 的支持列表中。
inline bool SupportsLoop(const Capabilities& caps, LoopMode mode) {
  return std::find(caps.supported_loops.begin(), caps.supported_loops.end(),
                   mode) != caps.supported_loops.end();
}

// 仅在不存在时添加 loop，避免 supported_loops 出现重复元素。
inline void AddLoopIfMissing(Capabilities& caps, LoopMode mode) {
  if (!SupportsLoop(caps, mode)) {
    caps.supported_loops.push_back(mode);
  }
}

// 判断 local topic 是否可用（非空串即视为可用）。
inline bool HasLocalTopic(const Topics& topics) {
  return !topics.local_topic.empty();
}

// 判断 global topic 是否可用（非空串即视为可用）。
inline bool HasGlobalTopic(const Topics& topics) {
  return !topics.global_topic.empty();
}

// 判断 loop topic 是否可用（非空串即视为可用）。
inline bool HasLoopTopic(const Topics& topics) {
  return !topics.loop_topic.empty();
}

} // namespace localization_fusion_types
