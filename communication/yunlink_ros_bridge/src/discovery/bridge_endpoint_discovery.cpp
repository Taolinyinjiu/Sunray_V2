/** @file @brief bridge 端点发现广播和稳定 endpoint_id 管理实现。 */
#include "discovery/bridge_endpoint_discovery.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <random>

#include <ros/console.h>

#include "bridge_node.hpp"

namespace {

/// 返回当前墙钟毫秒时间，用于发现包启动时间戳。
uint64_t wall_time_ms() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

/// 写入可选错误字符串。
void set_error(std::string* error, const std::string& value) {
    if (error != nullptr) {
        *error = value;
    }
}

/// 根据配置声明 bridge 能力。
std::vector<std::string> default_capabilities(bool enable_system_services) {
    std::vector<std::string> capabilities{"state", "commands"};
    if (enable_system_services) {
        capabilities.push_back("system_service");
    }
    return capabilities;
}

}  // namespace

BridgeEndpointDiscovery::BridgeEndpointDiscovery() = default;

BridgeEndpointDiscovery::~BridgeEndpointDiscovery() {
    stop();
}

/// 根据 bridge 参数生成广播内容并启动 UDP endpoint advertiser。
bool BridgeEndpointDiscovery::configure(const BridgeParams& params, std::string* error) {
    stop();
    if (!params.enable_endpoint_discovery) {
        enabled_ = false;
        return true;
    }

    // endpoint_id 跨重启保持稳定，避免 bridge 进程重启后 monitor 侧设备条目反复变化。
    std::string endpoint_id;
    if (!load_or_create_endpoint_id(params.endpoint_id_file, &endpoint_id, error)) {
        return false;
    }

    advertisement_ = yunlink::EndpointAdvertisement{};
    advertisement_.endpoint_id = endpoint_id;
    advertisement_.display_name_prefix =
        params.endpoint_name_prefix.empty() ? std::string(yunlink::kDefaultEndpointNamePrefix)
                                            : params.endpoint_name_prefix;
    advertisement_.agent_type = "uav";
    advertisement_.agent_id = static_cast<uint32_t>(std::max(params.agent_id, 0));
    advertisement_.role = "vehicle";
    advertisement_.node_name = params.node_name;
    advertisement_.tcp_listen_port = static_cast<uint16_t>(std::max(params.tcp_listen_port, 0));
    advertisement_.udp_bind_port = static_cast<uint16_t>(std::max(params.udp_bind_port, 0));
    advertisement_.protocol_version = "0.1.0";
    advertisement_.capabilities = default_capabilities(params.enable_system_services);
    advertisement_.discovery_period_ms =
        static_cast<uint32_t>(std::max(params.discovery_period_ms, 100));
    started_at_ms_ = wall_time_ms();
    advertisement_.started_at_ms = started_at_ms_;
    advertisement_.display_name = yunlink::make_endpoint_display_name(
        advertisement_.display_name_prefix, advertisement_.agent_id, advertisement_.endpoint_id);

    yunlink::EndpointDiscoveryConfig config;
    config.discovery_port =
        static_cast<uint16_t>(std::max(params.discovery_port, 0));
    config.target_ip = params.discovery_target_ip;
    const auto ec = advertiser_.start(config);
    if (ec != yunlink::ErrorCode::kOk) {
        set_error(error,
                  "start endpoint advertiser failed ec=" +
                      std::to_string(static_cast<int>(ec)) + " target=" +
                      params.discovery_target_ip + ":" + std::to_string(params.discovery_port));
        return false;
    }

    enabled_ = true;
    sequence_ = 0;
    return true;
}

/// 停止 endpoint advertiser 并清空启用状态。
void BridgeEndpointDiscovery::stop() {
    advertiser_.stop();
    enabled_ = false;
    sequence_ = 0;
}

/// 返回端点发现是否处于启用状态。
bool BridgeEndpointDiscovery::enabled() const {
    return enabled_;
}

/// 广播一次 endpoint advertisement，供 monitor 发现 bridge。
void BridgeEndpointDiscovery::publish_once() {
    if (!enabled_) {
        return;
    }
    advertisement_.started_at_ms = started_at_ms_;
    advertisement_.sequence = ++sequence_;
    if (advertiser_.send(advertisement_) < 0) {
        ROS_WARN_THROTTLE(5.0,
                          "endpoint discovery send failed: %s",
                          advertiser_.last_error().c_str());
    }
}

/// 返回当前广播给 monitor 的显示名。
std::string BridgeEndpointDiscovery::display_name() const {
    return advertisement_.display_name;
}

/// 返回当前稳定 endpoint_id。
std::string BridgeEndpointDiscovery::endpoint_id() const {
    return advertisement_.endpoint_id;
}

/// 优先读取持久化 endpoint_id，缺失时生成并写回。
bool BridgeEndpointDiscovery::load_or_create_endpoint_id(const std::string& path,
                                                         std::string* endpoint_id,
                                                         std::string* error) {
    if (read_endpoint_id_file(path, endpoint_id, error)) {
        return true;
    }

    if (!ensure_parent_directory(path, error)) {
        return false;
    }

    const std::string generated = generate_endpoint_id();
    if (!write_endpoint_id_file(path, generated, error)) {
        return false;
    }
    if (endpoint_id != nullptr) {
        *endpoint_id = generated;
    }
    return true;
}

/// 生成短 endpoint_id，用于显示和发现时区分不同 bridge。
std::string BridgeEndpointDiscovery::generate_endpoint_id() {
    static constexpr char kAlphabet[] = "0123456789abcdefghijklmnopqrstuvwxyz";
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_int_distribution<int> distribution(0, 35);

    std::string out;
    out.reserve(5);
    for (int index = 0; index < 5; ++index) {
        out.push_back(kAlphabet[distribution(generator)]);
    }
    return out;
}

/// 确保持久化 endpoint_id 的父目录存在。
bool BridgeEndpointDiscovery::ensure_parent_directory(const std::string& path, std::string* error) {
    try {
        const std::filesystem::path file_path(path);
        const std::filesystem::path parent = file_path.parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }
        return true;
    } catch (const std::exception& ex) {
        set_error(error, std::string("create endpoint id parent dir failed: ") + ex.what());
        return false;
    }
}

/// 从文件读取并校验 endpoint_id。
bool BridgeEndpointDiscovery::read_endpoint_id_file(const std::string& path,
                                                    std::string* endpoint_id,
                                                    std::string* error) {
    std::ifstream input(path);
    if (!input.is_open()) {
        set_error(error, "endpoint id file not found");
        return false;
    }

    std::string value;
    std::getline(input, value);
    if (!yunlink::validate_endpoint_id(value)) {
        set_error(error, "endpoint id file is invalid");
        return false;
    }
    if (endpoint_id != nullptr) {
        *endpoint_id = value;
    }
    return true;
}

/// 将生成的 endpoint_id 写入文件，保证后续重启稳定。
bool BridgeEndpointDiscovery::write_endpoint_id_file(const std::string& path,
                                                     const std::string& endpoint_id,
                                                     std::string* error) {
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open()) {
        set_error(error, "open endpoint id file for write failed");
        return false;
    }
    output << endpoint_id;
    if (!output.good()) {
        set_error(error, "write endpoint id file failed");
        return false;
    }
    return true;
}
