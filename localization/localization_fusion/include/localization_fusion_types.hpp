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

#include <cstdint>
#include <string>

// 定位源输入的类型
enum class LocalizationMode : uint8_t {
  LOCAL = 0,        // odometry_topic -> local odom
  GLOBAL = 1,       // odometry_topic -> global odom
  LOCAL_AND_GLOBAL = 2, // odometry_topic -> local odom,
                         // relocalization_topic -> global odom
  LOCAL_WITH_ARUCO = 3   // odometry_topic -> local odom,
                         // relocalization_topic -> relocalization
};

// 单个定位源的配置结构体
struct SourceConfig {
	std::string source_name; // 该定位源的名称，可以打印到日志中
  int source_id{-1};       // 外部定位源id，对应yaml中的source_id
  LocalizationMode localization_mode{LocalizationMode::LOCAL}; // 配置定位源的输入类型
	std::string odometry_topic{""};
	std::string relocalization_topic{""};
  double timeout_s{0.2};                       // 配置超时时间
};

// 读整个 yaml 文件，返回转译后的 source_id 对应的 config 结构体
SourceConfig load_config_from_yaml(const std::string &yaml_path,
                                   const int source_id,
                                   const std::string &uav_ns);
