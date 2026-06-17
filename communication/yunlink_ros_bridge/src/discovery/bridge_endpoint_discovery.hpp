/** @file @brief YunLink bridge 端点发现封装接口。 */
#pragma once

#include <cstdint>
#include <string>

#include <yunlink/discovery/endpoint_discovery.hpp>

class YunlinkRosBridgeNode;
struct BridgeParams;

/** @brief 负责生成稳定 endpoint_id 并周期广播 bridge 端点信息。 */
class BridgeEndpointDiscovery {
  public:
    BridgeEndpointDiscovery(); ///< @brief 构造端点发现对象。
    ~BridgeEndpointDiscovery(); ///< @brief 停止广播并释放发现资源。

    /** @brief 根据 bridge 参数配置端点发现。 @param params bridge 运行参数。 @param error 可选错误信息输出。 @return 配置成功返回 true。 */
    bool configure(const BridgeParams& params, std::string* error = nullptr);
    void stop(); ///< @brief 停止端点广播。
    bool enabled() const; ///< @brief 查询端点发现是否启用。 @return 已启用返回 true。
    void publish_once(); ///< @brief 立即广播一次当前端点信息。
    std::string display_name() const; ///< @brief 获取对 monitor 展示的端点名称。 @return 端点显示名。
    std::string endpoint_id() const; ///< @brief 获取稳定端点 ID。 @return endpoint_id 字符串。

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
