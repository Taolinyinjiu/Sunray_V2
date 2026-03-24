#include "localization_fusion_types.hpp"
#include "string_uav_namespace_utils.hpp"
#include <stdexcept>       // 引入stdexcpet用于抛出异常
#include <yaml-cpp/yaml.h> // 引入Yaml-cpp库，用于读取yaml文件

// 这里我们只需要实现一个函数，就是从指定的路径中加载配置参数
SourceConfig load_config_from_yaml(const std::string &yaml_path,
                                   const int source_id,
                                   const std::string &uav_ns) {
  // 构建用于返回的结构体
  SourceConfig result_config;
  // 根据传入的yaml文件路径，查找对应的yaml文件
  // 首先检查yaml路径是否为空
  if (yaml_path.empty()) {
    // 为空则抛出异常
    throw std::invalid_argument("yaml_path connot be empty");
  } else if (source_id < 0) {
    throw std::invalid_argument("the source_id: " + std::to_string(source_id) +
                                ", it must >= 0");
  } else if (uav_ns.empty()) {
    throw std::invalid_argument("uav_ns connot be empty");
  }
  // 输入初步没有问题，尝试读取yaml文件
  // 构造一个YAML的根节点
  YAML::Node root;
  // 由于读取的过程可能引发异常，因此使用try语法
  try {
    // 从指定的路径中读取yaml文件并解析为YAML::Node
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception &e) { // 如果解析的过程中发生错误，捕捉到异常
    throw std::runtime_error("Failed to load yaml file '" + yaml_path + ":" +
                             e.what());
  }
  // 顺利读取，取出字段sources_list的部分
  const YAML::Node sources_list = root["sources_list"];
  // 如果sources_list为空，或者不是键值对的形式，则抛出异常
  if (!sources_list || !sources_list.IsMap()) {
    throw std::runtime_error("the yaml file '" + yaml_path +
                             "' is missing a valid sources_list map");
  }
  // 根据传入的source_id寻找我们对应的定位源配置
  // 首先，我们需要遍历source_list，找到source_id与传入的source_id一致的那一项，再取那一项的其他字段
  // 使用迭代器进行遍历
  for (const auto &item : sources_list) {
    // 首先得到定位源的名字
    const std::string source_name = item.first.as<std::string>();
    // 得到定位源对应的字段
    const YAML::Node source_node = item.second;
    // 如果定位源对应的字段中，source_id 与传入的id一致，则认为是我们需要的
    // 首先检查字段存在，再判断值相等,这样不会导致异常
    if (source_node["source_id"] &&
        source_node["source_id"].as<int>() == source_id) {
      if (source_node
              ["localization_mode"]) // 定位模式，分别有local,global,local_and_global,local_with_aruco，使用字符串判断
      {
        // 将yaml字段内容填充到temp_string
        std::string temp_string =
            source_node["localization_mode"].as<std::string>();
        if (temp_string == "local") {
          result_config.localization_mode = LocalizationMode::LOCAL;
        } else if (temp_string == "global") {
          result_config.localization_mode = LocalizationMode::GLOBAL;
        } else if (temp_string == "local_and_global") {
          result_config.localization_mode = LocalizationMode::LOCAL_AND_GLOBAL;
        } else if (temp_string == "local_with_aruco") {
          result_config.localization_mode = LocalizationMode::LOCAL_WITH_ARUCO;
        } else {
          // 运行到这里，说明一个都没匹配上，抛出异常
          throw std::runtime_error(
              "localization_mode in localization_sources.yaml need check");
        }
      }
      // 里程计对应话题
      if (source_node["odometry_topic"]) {
        std::string temp_string =
            source_node["odometry_topic"].as<std::string>();
        if (!temp_string.empty()) {
          result_config.odometry_topic = temp_string;
        } else {
          throw std::runtime_error(
              "the odometry_topic in localization_sources.yaml missing value");
        }
      }
      // 重定位对应话题,只有在启用了重定位的时候才需要读取
      if ((result_config.localization_mode ==
               LocalizationMode::LOCAL_AND_GLOBAL ||
           result_config.localization_mode ==
               LocalizationMode::LOCAL_WITH_ARUCO) &&
          source_node["relocalization_topic"]) {
        std::string temp_string =
            source_node["relocalization_topic"].as<std::string>();
        if (!temp_string.empty()) {
          result_config.relocalization_topic = temp_string;
        } else {
          throw std::runtime_error("the relocalization_topic in localization_sources.yaml missing value");
        }
      }
      // 最后填充名字与序号
      result_config.source_name = source_name;
      result_config.source_id = source_node["source_id"].as<int>();
    }
  }
  // 结束遍历，得到结果(-1为结构体默认值，如果为-1说明没有更新id，也就是说没有对应的定位源)
  if (result_config.source_id == -1) {
    // 如果是-1表示SourceConfig使用的是默认的值，因此需要抛出异常
    throw std::runtime_error("faild to load config");
  } else
    return result_config;
};
