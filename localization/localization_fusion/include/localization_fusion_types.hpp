/**
 * @file localization_fusion_types.hpp
 * @author your name (you@domain.com)
 * @brief 本文件主要声明Localization_Fusion所使用的数据结构和枚举类型
 * @note 字符串解析/枚举转换/通用校验函数定义在 localization_fusion_utils.hpp 中
 * @version 0.1
 * @date 2026-03-19
 * @see https://yundrone.feishu.cn/wiki/NukDw1pCKiLD4rkhX5gcOGjWnAh
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <string>
#include <vector>

// clang-format off

namespace localization_fusion_types {

// 重定位模式，分别是失能，基于雷达的重定位，基于ARUCO的重定位
enum class RelocalizationMode {
  DISABLE = 0,
  LIDAR_RELOCALIZATION = 1,
  ARUCO_RELOCALIZATION = 2
};

// 输出模式（控制 local/global 的输出关系）
enum class PublishMode {
  // 单输入复制：local_odom = global_odom = single_input
  SINGLE_COPY = 0,
  // 双输入直通：local_odom = local_meas, global_odom = global_meas 最直观的表现就是动捕环境下实现 机+车 fastlio 集群
  DUAL_DIRECT = 1,
  // 双输入初始化变换：先初始化 T_map_odom，再 global_odom = T_map_odom * local_odom
  INIT_TRANSFORM = 2
};

// 定位源 能力描述，描述某个定位源的职责
struct Capabilities {
  bool provides_local{false}; // 该定位源能否提供local系下的位置估计
  bool provides_global{false}; // 该定位源能否提供global系下的位置估计
  std::vector<RelocalizationMode> supported_relocalization{}; // 该定位源能否支持重定位
  PublishMode publish_mode{PublishMode::SINGLE_COPY}; // 输出策略
};

// 构造话题配置结构体
struct Topics {
  std::string local_topic;      // 大多数定位源都会提供local系下的定位能力，这里表示local系下订阅话题
  std::string global_topic;     // 部分定位源会提供global系下定位能力，这里表示global系下订阅话题
  std::string relocalization_topic; // 该定位源重定位/回环数据输出话题
};

// 单个定位源的配置结构体
struct SourceConfig {
  int source_id{-1}; // 外部定位源id，对应yaml中的source_id
  std::string source_name;     // 该定位源的名称，可以打印到日志中
  Topics source_topics; // 配置该定位源的话题
  Capabilities capabilities; // 配置该定位源的职责
  double timeout_s{0.2}; // 配置超时时间
};

}
// clang-format on
