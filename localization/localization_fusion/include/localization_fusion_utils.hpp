/**
 * @file localization_fusion_utils.hpp
 * @brief localization_fusion 的辅助函数（与 types 分离）
 *
 * 使用说明：
 * 1) 先包含本头文件和 localization_fusion_types.hpp；
 * 2) 读取 yaml 字段后，使用 ParseRelocalizationMode/ParsePublishMode 将字符串转为枚举；
 * 3) 使用 AddRelocalizationIfMissing 维护 supported_relocalization，避免重复；
 * 4) 使用 HasLocalTopic/HasGlobalTopic/HasRelocalizationTopic 判断 topic 是否可用；
 * 5) 打印日志时使用 ToString(RelocalizationMode/PublishMode) 统一输出格式。
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

// 解析 relocalization 字符串。
// 支持: disable/lidar_relocalization/aruco_relocalization
// 异常: 未识别输入会抛出 std::invalid_argument
inline RelocalizationMode ParseRelocalizationMode(const std::string& raw) {
  const std::string s = ToLower(raw);
  if (s == "disable") return RelocalizationMode::DISABLE;
  if (s == "lidar_relocalization") return RelocalizationMode::LIDAR_RELOCALIZATION;
  if (s == "aruco_relocalization") return RelocalizationMode::ARUCO_RELOCALIZATION;
  throw std::invalid_argument("Unknown RelocalizationMode: " + raw);
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
inline const char* ToString(RelocalizationMode mode) {
  switch (mode) {
    case RelocalizationMode::DISABLE: return "disable";
    case RelocalizationMode::LIDAR_RELOCALIZATION: return "lidar_relocalization";
    case RelocalizationMode::ARUCO_RELOCALIZATION: return "aruco_relocalization";
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

// 检查某个 relocalization 是否在 capabilities 的支持列表中。
inline bool SupportsRelocalization(const Capabilities& caps, RelocalizationMode mode) {
  return std::find(caps.supported_relocalization.begin(),
                   caps.supported_relocalization.end(),
                   mode) != caps.supported_relocalization.end();
}

// 仅在不存在时添加 relocalization，避免 supported_relocalization 出现重复元素。
inline void AddRelocalizationIfMissing(Capabilities& caps, RelocalizationMode mode) {
  if (!SupportsRelocalization(caps, mode)) {
    caps.supported_relocalization.push_back(mode);
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

// 判断重定位 topic 是否可用（非空串即视为可用）。
inline bool HasRelocalizationTopic(const Topics& topics) {
  return !topics.relocalization_topic.empty();
}

} // namespace localization_fusion_types
