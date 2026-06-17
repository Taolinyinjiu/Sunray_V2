/** @file @brief bridge system-service 客户端和后台 worker 生命周期实现。 */
#include "bridge_node.hpp"

/// 创建 sunray_system 的 ROS service client。
void YunlinkRosBridgeNode::setupSystemServiceClients() {
    if (!params_.enable_system_services) {
        return;
    }

    const std::string list_name = params_.sunray_system_ns + "/list_features";
    const std::string get_name = params_.sunray_system_ns + "/get_features";
    const std::string start_name = params_.sunray_system_ns + "/start_feature";
    const std::string stop_name = params_.sunray_system_ns + "/stop_feature";
    list_features_client_ = nh_.serviceClient<sunray_msgs::ListFeatures>(list_name, false);
    get_features_client_ = nh_.serviceClient<sunray_msgs::GetFeatures>(get_name, false);
    start_feature_client_ = nh_.serviceClient<sunray_msgs::StartFeature>(start_name, false);
    stop_feature_client_ = nh_.serviceClient<sunray_msgs::StopFeature>(stop_name, false);
}

/// 在启用 system-service 时启动后台 worker。
void YunlinkRosBridgeNode::startSystemServiceWorker() {
    if (!params_.enable_system_services) {
        return;
    }
    stop_system_service_worker_ = false;
    system_service_worker_ = std::thread(&YunlinkRosBridgeNode::systemServiceWorkerLoop, this);
}

/// 请求后台 worker 退出并等待线程结束。
void YunlinkRosBridgeNode::stopSystemServiceWorker() {
    {
        std::lock_guard<std::mutex> lock(system_service_mu_);
        stop_system_service_worker_ = true;
    }
    system_service_cv_.notify_all();
    if (system_service_worker_.joinable()) {
        system_service_worker_.join();
    }
}

/// 端点发现定时器入口。
void YunlinkRosBridgeNode::onEndpointDiscoveryTimer(const ros::TimerEvent&) {
    endpoint_discovery_.publish_once();
}
