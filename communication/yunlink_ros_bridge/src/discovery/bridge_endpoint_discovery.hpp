#pragma once

#include <cstdint>
#include <string>

#include <yunlink/discovery/endpoint_discovery.hpp>

class YunlinkRosBridgeNode;
struct BridgeParams;

class BridgeEndpointDiscovery {
  public:
    BridgeEndpointDiscovery();
    ~BridgeEndpointDiscovery();

    bool configure(const BridgeParams& params, std::string* error = nullptr);
    void stop();
    bool enabled() const;
    void publish_once();
    std::string display_name() const;
    std::string endpoint_id() const;

  private:
    static bool load_or_create_endpoint_id(const std::string& path,
                                           std::string* endpoint_id,
                                           std::string* error);
    static std::string generate_endpoint_id();
    static bool ensure_parent_directory(const std::string& path, std::string* error);
    static bool read_endpoint_id_file(const std::string& path,
                                      std::string* endpoint_id,
                                      std::string* error);
    static bool write_endpoint_id_file(const std::string& path,
                                       const std::string& endpoint_id,
                                       std::string* error);

    bool enabled_{false};
    uint64_t sequence_{0};
    uint64_t started_at_ms_{0};
    yunlink::EndpointAdvertisement advertisement_;
    yunlink::EndpointAdvertiser advertiser_;
};
