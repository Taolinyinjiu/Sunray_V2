#include "discovery/bridge_endpoint_discovery.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>

#include <ros/console.h>

#include "bridge_node.hpp"

namespace {

uint64_t wall_time_ms() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

void set_error(std::string* error, const std::string& value) {
    if (error != nullptr) {
        *error = value;
    }
}

std::vector<std::string> default_capabilities(bool enable_system_services) {
    std::vector<std::string> capabilities{"state", "commands"};
    if (enable_system_services) {
        capabilities.push_back("system_service");
    }
    return capabilities;
}

bool parse_agent_id_text(const std::string& text, uint32_t* agent_id) {
    if (text.empty()) {
        return false;
    }
    for (char ch : text) {
        if (!std::isdigit(static_cast<unsigned char>(ch))) {
            return false;
        }
    }

    std::size_t parsed = 0;
    const unsigned long long value = std::stoull(text, &parsed, 10);
    if (parsed != text.size() || value == 0 ||
        value > static_cast<unsigned long long>(std::numeric_limits<uint32_t>::max())) {
        return false;
    }
    if (agent_id != nullptr) {
        *agent_id = static_cast<uint32_t>(value);
    }
    return true;
}

}  // namespace

BridgeEndpointDiscovery::BridgeEndpointDiscovery() = default;

BridgeEndpointDiscovery::~BridgeEndpointDiscovery() {
    stop();
}

bool BridgeEndpointDiscovery::resolve_agent_id(bool enable_auto_agent_id,
                                               int configured_agent_id,
                                               const std::string& path,
                                               uint32_t* resolved_agent_id,
                                               std::string* error) {
    return load_or_create_agent_id(
        enable_auto_agent_id, configured_agent_id, path, resolved_agent_id, error);
}

bool BridgeEndpointDiscovery::configure(const BridgeParams& params, std::string* error) {
    stop();
    if (!params.enable_endpoint_discovery) {
        enabled_ = false;
        return true;
    }

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

void BridgeEndpointDiscovery::stop() {
    advertiser_.stop();
    enabled_ = false;
    sequence_ = 0;
}

bool BridgeEndpointDiscovery::enabled() const {
    return enabled_;
}

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

std::string BridgeEndpointDiscovery::display_name() const {
    return advertisement_.display_name;
}

std::string BridgeEndpointDiscovery::endpoint_id() const {
    return advertisement_.endpoint_id;
}

bool BridgeEndpointDiscovery::load_or_create_agent_id(bool enable_auto_agent_id,
                                                      int configured_agent_id,
                                                      const std::string& path,
                                                      uint32_t* resolved_agent_id,
                                                      std::string* error) {
    if (!enable_auto_agent_id) {
        if (configured_agent_id < 0) {
            set_error(error, "manual agent id must be >= 0 when auto agent id is disabled");
            return false;
        }
        const uint32_t manual_agent_id = static_cast<uint32_t>(configured_agent_id);
        if (resolved_agent_id != nullptr) {
            *resolved_agent_id = manual_agent_id;
        }
        return true;
    }

    if (read_agent_id_file(path, resolved_agent_id, error)) {
        return true;
    }
    if (!ensure_parent_directory(path, error)) {
        return false;
    }

    uint32_t generated = 0;
    if (!allocate_next_agent_id(path, &generated, error)) {
        return false;
    }
    if (!write_agent_id_file(path, generated, error)) {
        return false;
    }
    if (resolved_agent_id != nullptr) {
        *resolved_agent_id = generated;
    }
    return true;
}

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

bool BridgeEndpointDiscovery::allocate_next_agent_id(const std::string& path,
                                                     uint32_t* agent_id,
                                                     std::string* error) {
    const std::string next_path = next_agent_id_path(path);
    uint32_t next_value = 0;

    std::ifstream input(next_path);
    if (input.is_open()) {
        std::string value;
        std::getline(input, value);
        if (!value.empty() && !parse_agent_id_text(value, &next_value)) {
            set_error(error, "next agent id file is invalid");
            return false;
        }
    }

    std::ofstream output(next_path, std::ios::trunc);
    if (!output.is_open()) {
        set_error(error, "open next agent id file for write failed");
        return false;
    }
    output << (next_value + 1U);
    if (!output.good()) {
        set_error(error, "write next agent id file failed");
        return false;
    }

    if (agent_id != nullptr) {
        *agent_id = next_value;
    }
    return true;
}

std::string BridgeEndpointDiscovery::next_agent_id_path(const std::string& path) {
    return path + ".next";
}

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

bool BridgeEndpointDiscovery::read_agent_id_file(const std::string& path,
                                                 uint32_t* agent_id,
                                                 std::string* error) {
    std::ifstream input(path);
    if (!input.is_open()) {
        set_error(error, "agent id file not found");
        return false;
    }

    std::string value;
    std::getline(input, value);
    if (!parse_agent_id_text(value, agent_id)) {
        set_error(error, "agent id file is invalid");
        return false;
    }
    return true;
}

bool BridgeEndpointDiscovery::write_agent_id_file(const std::string& path,
                                                  uint32_t agent_id,
                                                  std::string* error) {
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open()) {
        set_error(error, "open agent id file for write failed");
        return false;
    }
    output << agent_id;
    if (!output.good()) {
        set_error(error, "write agent id file failed");
        return false;
    }
    return true;
}

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
