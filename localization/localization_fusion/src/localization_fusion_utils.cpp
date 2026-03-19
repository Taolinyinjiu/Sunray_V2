/**
 * @file localization_fusion_utils.cpp
 * @author your name (you@domain.com)
 * @brief 本文件作为localization_fusion.hpp中参数加载/校验类函数的实现
 * @version 0.1
 * @date 2026-03-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "localization_fusion.hpp"

#include <exception>

#include <XmlRpcValue.h>

#include "localization_fusion_utils.hpp"

namespace localization_fusion {

// 加载运行时参数
bool LocalizationFusion::LoadRuntimeParams() {
  // 使用私有命名空间，读取source_id参数
	private_nh_.param<int>("source_id", selected_source_id_, -1);
	// 默认loop回环为失能状态
  std::string loop_str = "disable";
  // 使用私有命名空间，读取loop参数
	private_nh_.param<std::string>("loop", loop_str, "disable");
	// 由于字符串解析函数可能遇到用户拼写错误抛出异常，因此这里使用try进行
  try {
		// 将回环参数进行枚举转换
    selected_loop_mode_ = localization_fusion_types::ParseLoopMode(loop_str);
  } catch (const std::exception &e) {
    ROS_ERROR("[localization_fusion] Invalid loop param: %s", e.what());
    return false;
  }
	// 当读取到的selected_source_id_ 有效时输出true
  return selected_source_id_ >= 0;
}

// 加载定位源配置（~sources_list）
bool LocalizationFusion::LoadSourceConfigs() {
  // XmlRpc::XmlRpcValue 是 ROS 参数系统中的通用容器，可承载 map/array/string 等类型。
  XmlRpc::XmlRpcValue sources_list;

  // 读取私有参数 ~sources_list。
  // 该参数在launch文件中使用rosparam load 导入。
  if (!private_nh_.getParam("sources_list", sources_list)) {
    ROS_ERROR("[localization_fusion] Missing param: ~sources_list");
    return false;
  }

  // 期望 sources_list 是 map 结构（yaml 键值对），例如：
  // sources_list:
  //   VIOBOT: {...}
  //   MOCAP:  {...}
  if (sources_list.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("[localization_fusion] ~sources_list must be a map.");
    return false;
  }

  // 清空旧配置，避免重复加载时残留历史数据。
  sources_.clear();

  // 使用迭代器，遍历每个 source 节点（key 为 source_name，value 为 source_cfg）。
  for (auto it = sources_list.begin(); it != sources_list.end(); ++it) {
    // 先读取定位源的名字
		const std::string source_name = static_cast<std::string>(it->first);
    // 再构造配置键值对
		XmlRpc::XmlRpcValue source_cfg = it->second;

    // 先构造临时配置对象，解析完后再写入 sources_。
    SourceConfig cfg;
    cfg.source_name = source_name;

    // source_id 是必填项，且必须是 int。
    if (source_cfg.hasMember("source_id") &&
        source_cfg["source_id"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      cfg.source_id = static_cast<int>(source_cfg["source_id"]);
    } else {
      ROS_ERROR("[localization_fusion] source '%s' missing valid source_id.",
                source_name.c_str());
      return false;
    }

    // timeout_s 是可选项，允许 int/double 两种数值类型，但最后都会被转换为double类型使用
    if (source_cfg.hasMember("timeout_s") &&
        (source_cfg["timeout_s"].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
         source_cfg["timeout_s"].getType() == XmlRpc::XmlRpcValue::TypeInt)) {
      cfg.timeout_s = static_cast<double>(source_cfg["timeout_s"]);
    }

    // capabilities 是能力描述：
    // - provides_local / provides_global
    // - supported_loops
    // - publish_mode
    if (source_cfg.hasMember("capabilities")) {
      auto caps = source_cfg["capabilities"];

      if (caps.hasMember("provides_local") &&	
          caps["provides_local"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        cfg.capabilities.provides_local = static_cast<bool>(caps["provides_local"]);
      }
      if (caps.hasMember("provides_global") &&
          caps["provides_global"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        cfg.capabilities.provides_global = static_cast<bool>(caps["provides_global"]);
      }

      if (caps.hasMember("supported_loops") &&
          caps["supported_loops"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < caps["supported_loops"].size(); ++i) {
          // 非字符串元素直接跳过，避免类型污染。
          if (caps["supported_loops"][i].getType() != XmlRpc::XmlRpcValue::TypeString) {
            continue;
          }
          try {
            auto mode = localization_fusion_types::ParseLoopMode(
                static_cast<std::string>(caps["supported_loops"][i]));
            // 去重后再加入 supported_loops。
            localization_fusion_types::AddLoopIfMissing(cfg.capabilities, mode);
          } catch (const std::exception &e) {
            ROS_ERROR("[localization_fusion] source '%s' invalid loop: %s",
                      source_name.c_str(), e.what());
            return false;
          }
        }
      }

      if (caps.hasMember("publish_mode") &&
          caps["publish_mode"].getType() == XmlRpc::XmlRpcValue::TypeString) {
        try {
          cfg.capabilities.publish_mode = localization_fusion_types::ParsePublishMode(
              static_cast<std::string>(caps["publish_mode"]));
        } catch (const std::exception &e) {
          ROS_ERROR("[localization_fusion] source '%s' invalid publish_mode: %s",
                    source_name.c_str(), e.what());
          return false;
        }
      }
    }

    // source_topics 描述输入话题配置。
    // 若 yaml 中写 null，XmlRpc 中通常不是 TypeString，这里会保持默认空串。
    if (source_cfg.hasMember("source_topics")) {
      auto topics = source_cfg["source_topics"];
      if (topics.hasMember("local_topic") &&
          topics["local_topic"].getType() == XmlRpc::XmlRpcValue::TypeString) {
        cfg.source_topics.local_topic = static_cast<std::string>(topics["local_topic"]);
      }
      if (topics.hasMember("global_topic") &&
          topics["global_topic"].getType() == XmlRpc::XmlRpcValue::TypeString) {
        cfg.source_topics.global_topic = static_cast<std::string>(topics["global_topic"]);
      }
      if (topics.hasMember("loop_topic") &&
          topics["loop_topic"].getType() == XmlRpc::XmlRpcValue::TypeString) {
        cfg.source_topics.loop_topic = static_cast<std::string>(topics["loop_topic"]);
      }
    }

    // 以 source_id 为键写入注册表（后写入会覆盖同 id 旧值）。
    sources_[cfg.source_id] = cfg;
  }

  // 至少加载到 1 个 source 才算成功。
  return !sources_.empty();
}

// 在启动前检查 当前选择的 source + loop 组合是否符合Sunray项目准则、可正常运行
bool LocalizationFusion::ValidateMode(const SourceConfig &cfg, LoopMode loop_mode) const {
	// 检查当前的配置与launch文件中指定的回环模式是否匹配
  if (!localization_fusion_types::SupportsLoop(cfg.capabilities, loop_mode)) {
    ROS_ERROR("[localization_fusion] source_id=%d does not support loop=%s",
              cfg.source_id, localization_fusion_types::ToString(loop_mode));
    return false;
  }
	// 如果定位源既不支持local系与不支持global系，则报错
  if (!cfg.capabilities.provides_local && !cfg.capabilities.provides_global) {
    ROS_ERROR("[localization_fusion] source_id=%d provides neither local nor global.",
              cfg.source_id);
    return false;
  }
	// 如果提供了local系但是订阅的local_topics为空字符串，则报错
  if (cfg.capabilities.provides_local &&
      !localization_fusion_types::HasLocalTopic(cfg.source_topics)) {
    ROS_ERROR("[localization_fusion] source_id=%d requires local_topic.", cfg.source_id);
    return false;
  }

	// 如果提供了global系但是订阅的global_topics为空字符串，则报错
  if (cfg.capabilities.provides_global &&
      !localization_fusion_types::HasGlobalTopic(cfg.source_topics)) {
    ROS_ERROR("[localization_fusion] source_id=%d requires global_topic.", cfg.source_id);
    return false;
  }

  return true;
}

} // namespace localization_fusion
