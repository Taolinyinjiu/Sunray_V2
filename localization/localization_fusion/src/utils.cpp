/**
 * @file localization_fusion_utils.cpp
 * @brief localization_fusion 配置读取与校验辅助接口的实现
 */

#include "localization_fusion_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace localization_fusion_utils {
namespace {

const char *to_string(const InputType input_type) {
  switch (input_type) {
  case InputType::LOCAL:
    return "local";
  case InputType::GLOBAL:
    return "global";
  case InputType::LOCAL_AND_GLOBAL:
    return "local_and_global";
  case InputType::LOCAL_WITH_ARUCO:
    return "local_with_aruco";
  default:
    return "unknown";
  }
}

std::string normalize_uav_ns(std::string uav_ns) {
  if (uav_ns.empty()) {
    return uav_ns;
  }
  if (uav_ns.front() != '/') {
    uav_ns.insert(uav_ns.begin(), '/');
  }
  while (uav_ns.size() > 1 && uav_ns.back() == '/') {
    uav_ns.pop_back();
  }
  return uav_ns;
}

std::string replace_all(std::string text, const std::string &from,
                        const std::string &to) {
  std::size_t start_pos = 0;
  while ((start_pos = text.find(from, start_pos)) != std::string::npos) {
    text.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
  return text;
}

[[noreturn]] void throw_invalid_source(const std::string &source_name,
                                       const std::string &reason) {
  throw std::invalid_argument("Invalid source '" + source_name + "': " +
                              reason);
}

} // namespace

// 将localizaton_sources.yaml中的input_type从整型转化为InputType枚举类型
InputType parse_input_type(const int raw) {
  switch (raw) {
  case 0:
    return InputType::LOCAL;
  case 1:
    return InputType::GLOBAL;
  case 2:
    return InputType::LOCAL_AND_GLOBAL;
  case 3:
    return InputType::LOCAL_WITH_ARUCO;
  // 匹配不到则抛出异常
	default:
    throw std::invalid_argument("Unknown input_type integer: " +
                                std::to_string(raw));
  }
}

// 将订阅话题中的"${uav_ns}"转译为参数空间中的uav_name + uav_id
std::string resolve_topic(const std::string &raw_topic,
                          const std::string &uav_ns) {
  // 如果输入的原始字符串为空，则返回空
	if (raw_topic.empty()) {
    return "";
  }
	// 如果在输入的原始字符串中没有找到"${uav_ns}"则返回原始字符串
  if (raw_topic.find("${uav_ns}") == std::string::npos) {
    return raw_topic;
  }
	// 将uav_ns进行标准化，确保最后得到的是"/uav1"这样的格式
  const std::string normalized_uav_ns = normalize_uav_ns(uav_ns);
  // 如果标准化失败，就抛出异常，认为参数空间没有加载对应的参数
	if (normalized_uav_ns.empty()) {
    throw std::invalid_argument(
        "Topic contains '${uav_ns}' but resolved uav_ns is empty: " +
        raw_topic);
  }
	// 返回转译后的新串
  return replace_all(raw_topic, "${uav_ns}", normalized_uav_ns);
}

// 从yaml文件中加载参数
SourceMap load_config_from_yaml(const std::string &yaml_path,
                                const std::string &uav_ns) {
  // 如果yaml文件的路径为空，则抛出异常
	if (yaml_path.empty()) {
    throw std::invalid_argument("yaml_path cannot be empty.");
  }
	// 构造yaml 根节点
  YAML::Node root;
  try {
		// 从指定的路径中读取yaml文件，并解析为一个YAML::Node
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception &e) {	//如果解析的过程中出现错误，就抛出异常
    throw std::runtime_error("Failed to load yaml file '" + yaml_path +
                             "': " + e.what());
  }
	// 从yaml根节点root中，取出字段source_list的子节点
  const YAML::Node sources_list = root["sources_list"];
  // 如果sources_list为空，或者 sources_list不是键值对结构，则抛出异常
	if (!sources_list || !sources_list.IsMap()) {
    throw std::runtime_error("YAML file '" + yaml_path +
                             "' is missing a valid 'sources_list' map.");
  }

  SourceMap sources;
  std::unordered_set<int> used_source_ids;
	
  for (const auto &item : sources_list) {
    if (!item.first.IsScalar()) {
      throw std::runtime_error(
          "Each key in 'sources_list' must be a scalar source name.");
    }
    if (!item.second.IsMap()) {
      throw std::runtime_error("Source '" + item.first.as<std::string>() +
                               "' must be a map.");
		}

    SourceConfig cfg;
    cfg.source_name = item.first.as<std::string>();
    const YAML::Node source_node = item.second;

    if (!source_node["source_id"] || !source_node["source_id"].IsScalar()) {
      throw_invalid_source(cfg.source_name,
                           "missing scalar field 'source_id'.");
    }
    cfg.source_id = source_node["source_id"].as<int>();

    if (!used_source_ids.insert(cfg.source_id).second) {
      throw_invalid_source(cfg.source_name,
                           "duplicated source_id=" +
                               std::to_string(cfg.source_id) + ".");
    }

    if (!source_node["input_type"] || !source_node["input_type"].IsScalar()) {
      throw_invalid_source(cfg.source_name,
                           "missing scalar field 'input_type'.");
    }
    try {
      cfg.input_type = parse_input_type(source_node["input_type"].as<int>());
    } catch (const YAML::Exception &) {
      throw_invalid_source(cfg.source_name,
                           "field 'input_type' must be an integer.");
    }

    if (source_node["odometry_topic"]) {
      cfg.source_topics.odometry_topic =
          resolve_topic(source_node["odometry_topic"].as<std::string>(),
                        uav_ns);
    }
    if (source_node["relocalization_topic"]) {
      cfg.source_topics.relocalization_topic =
          resolve_topic(source_node["relocalization_topic"].as<std::string>(),
                        uav_ns);
    }
    if (source_node["timeout_s"]) {
      cfg.timeout_s = source_node["timeout_s"].as<double>();
    }

    check_source(cfg);
    sources.emplace(cfg.source_id, cfg);
  }

  check_sources_list(sources);
  return sources;
}

void check_source(const SourceConfig &cfg) {
  if (cfg.source_name.empty()) {
    throw std::invalid_argument("Source name cannot be empty.");
  }
  if (cfg.source_id < 0) {
    throw_invalid_source(cfg.source_name,
                         "source_id must be greater than or equal to 0.");
  }
  if (cfg.timeout_s <= 0.0) {
    throw_invalid_source(cfg.source_name, "timeout_s must be greater than 0.");
  }
  if (cfg.source_topics.odometry_topic.empty()) {
    throw_invalid_source(cfg.source_name, "odometry_topic cannot be empty.");
  }

  switch (cfg.input_type) {
  case InputType::LOCAL:
  case InputType::GLOBAL:
    if (!cfg.source_topics.relocalization_topic.empty()) {
      throw_invalid_source(cfg.source_name,
                           std::string("input_type='") +
                               to_string(cfg.input_type) +
                               "' should not configure relocalization_topic.");
    }
    break;
  case InputType::LOCAL_AND_GLOBAL:
  case InputType::LOCAL_WITH_ARUCO:
    if (cfg.source_topics.relocalization_topic.empty()) {
      throw_invalid_source(cfg.source_name,
                           std::string("input_type='") +
                               to_string(cfg.input_type) +
                               "' requires relocalization_topic.");
    }
    break;
  default:
    throw_invalid_source(cfg.source_name, "input_type is invalid.");
  }
}

void check_sources_list(const SourceMap &sources) {
  if (sources.empty()) {
    throw std::invalid_argument("sources_list cannot be empty.");
  }

  for (const auto &entry : sources) {
    const int source_id = entry.first;
    const SourceConfig &cfg = entry.second;
    if (source_id != cfg.source_id) {
      std::ostringstream oss;
      oss << "Source map key mismatch: key=" << source_id
          << " but cfg.source_id=" << cfg.source_id << ".";
      throw std::invalid_argument(oss.str());
    }
    check_source(cfg);
  }
}

} // namespace localization_fusion_utils
