#include "localization_fusion_types.hpp"
#include <stdexcept>        // 引入stdexcpet用于抛出异常
#include <yaml-cpp/yaml.h>  // 引入Yaml-cpp库，用于读取yaml文件

// clang-format off

// 这里我们只需要实现一个函数，就是从指定的路径中加载配置参数
SourceConfig load_config_from_yaml(const std::string& yaml_path,
                                   const int source_id) {
    // 构建用于返回的结构体
    SourceConfig result_config;
    // 根据传入的yaml文件路径，查找对应的yaml文件
    // 首先检查yaml路径是否为空
    if (yaml_path.empty()) {
        throw std::invalid_argument("yaml_path cannot be empty");
    } else if (source_id < 0) {
        throw std::invalid_argument("the source_id: " + std::to_string(source_id) + ", it must >= 0");
    }
    // 输入初步没有问题，尝试读取yaml文件
    // 构造一个YAML的根节点
    YAML::Node root;
    // 由于读取的过程可能引发异常，因此使用try语法
    try {
        // 从指定的路径中读取yaml文件并解析为YAML::Node
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {  // 如果解析的过程中发生错误，捕捉到异常
        throw std::runtime_error("Failed to load yaml file '" + yaml_path + ":" + e.what());
    }
    // 顺利读取，取出字段sources_list的部分
    const YAML::Node sources_list = root["sources_list"];
    // 如果sources_list为空，或者不是键值对的形式，则抛出异常
    if (!sources_list || !sources_list.IsMap()) {
        throw std::runtime_error("the yaml file '" + yaml_path + "' is missing a valid sources_list map");
    }
    // 根据传入的source_id寻找我们对应的定位源配置
    // 首先，我们需要遍历source_list，找到source_id与传入的source_id一致的那一项，再取那一项的其他字段
    // 使用迭代器进行遍历
    for (const auto& item : sources_list) {
        // 首先得到定位源的名字
        const std::string source_name = item.first.as<std::string>();
        // 得到定位源对应的字段
        const YAML::Node source_node = item.second;
        // 如果定位源对应的字段中，source_id 与传入的id一致，则认为是我们需要的
        // 首先检查字段存在，再判断值相等,这样不会导致异常
        if (source_node["source_id"] && source_node["source_id"].as<int>() == source_id) {
            // 里程计对应话题
            if (source_node["odometry_topic"]) {
                std::string temp_string = source_node["odometry_topic"].as<std::string>();
                if (!temp_string.empty()) {
                    result_config.odometry_topic = temp_string;
                } else {
                    throw std::runtime_error("the odometry_topic in localization_sources.yaml missing value");
                }
            }
            // 重定位对应话题，允许为空字符串，表示当前source未接入重定位输入
            if (source_node["relocalization_topic"]) {
                result_config.relocalization_topic =
                    source_node["relocalization_topic"].as<std::string>();
            }
            // 通信超时参数
            if (source_node["timeout_s"]) {
                result_config.timeout_s = source_node["timeout_s"].as<double>();
                if (result_config.timeout_s <= 0.0) {
                    throw std::runtime_error("the timeout_s in localization_sources.yaml must > 0");
                }
            }
            // 传感器坐标系到机体系的外参矩阵
            if (source_node["source_frame_to_base"]) {
                const YAML::Node tf_node = source_node["source_frame_to_base"];
                if (!tf_node.IsSequence() || tf_node.size() != 4) {
                    throw std::runtime_error("source_frame_to_base must be a 4x4 matrix");
                }
                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                for (std::size_t row = 0; row < 4; ++row) {
                    const YAML::Node row_node = tf_node[row];
                    if (!row_node.IsSequence() || row_node.size() != 4) {
                        throw std::runtime_error("source_frame_to_base must be a 4x4 matrix");
                    }
                    for (std::size_t col = 0; col < 4; ++col) {
                        T(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
                            row_node[col].as<double>();
                    }
                }
                result_config.source_frame_to_base = T;
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

// clang-format on
