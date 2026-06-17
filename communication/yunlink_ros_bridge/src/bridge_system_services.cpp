/** @file @brief YunLink system-service 请求到 sunray_system ROS service 的适配实现。 */
#include "bridge_node.hpp"

namespace {

/// 组合 sunray_system 命名空间和 service 后缀。
std::string service_name(const std::string& ns, const char* suffix) {
    return ns + "/" + suffix;
}

}  // namespace

/// 请求回调只入队任务，避免 ROS service 等待阻塞 runtime 回调路径。
void YunlinkRosBridgeNode::onFeatureListRequest(
    const yunlink::InboundSystemServiceRequestView<yunlink::FeatureListRequest>& view) {
    recordYunlinkToRosEvent("feature_list", "list_features");
    SystemServiceJob job;
    job.kind = SystemServiceJobKind::kFeatureList;
    job.inbound = view.inbound;
    enqueueSystemServiceJob(std::move(job));
}

/// 接收 YunLink feature_get 请求并放入后台 service 队列。
void YunlinkRosBridgeNode::onFeatureGetRequest(
    const yunlink::InboundSystemServiceRequestView<yunlink::FeatureGetRequest>& view) {
    recordYunlinkToRosEvent("feature_get", view.payload.feature_name);
    SystemServiceJob job;
    job.kind = SystemServiceJobKind::kFeatureGet;
    job.inbound = view.inbound;
    job.feature_get = view.payload;
    enqueueSystemServiceJob(std::move(job));
}

/// 接收 YunLink feature_start 请求并放入后台 service 队列。
void YunlinkRosBridgeNode::onFeatureStartRequest(
    const yunlink::InboundSystemServiceRequestView<yunlink::FeatureStartRequest>& view) {
    recordYunlinkToRosEvent("feature_start", view.payload.feature_name);
    SystemServiceJob job;
    job.kind = SystemServiceJobKind::kFeatureStart;
    job.inbound = view.inbound;
    job.feature_start = view.payload;
    enqueueSystemServiceJob(std::move(job));
}

/// 接收 YunLink feature_stop 请求并放入后台 service 队列。
void YunlinkRosBridgeNode::onFeatureStopRequest(
    const yunlink::InboundSystemServiceRequestView<yunlink::FeatureStopRequest>& view) {
    recordYunlinkToRosEvent("feature_stop", view.payload.feature_name);
    SystemServiceJob job;
    job.kind = SystemServiceJobKind::kFeatureStop;
    job.inbound = view.inbound;
    job.feature_stop = view.payload;
    enqueueSystemServiceJob(std::move(job));
}

/// 将 system-service 请求放入 worker 队列并唤醒后台线程。
void YunlinkRosBridgeNode::enqueueSystemServiceJob(SystemServiceJob job) {
    {
        std::lock_guard<std::mutex> lock(system_service_mu_);
        system_service_jobs_.push_back(std::move(job));
    }
    system_service_cv_.notify_one();
}

/// 串行处理 YunLink system-service 请求，避免阻塞 runtime 回调线程。
void YunlinkRosBridgeNode::systemServiceWorkerLoop() {
    while (true) {
        SystemServiceJob job;
        {
            std::unique_lock<std::mutex> lock(system_service_mu_);
            system_service_cv_.wait(lock, [this]() {
                return stop_system_service_worker_ || !system_service_jobs_.empty();
            });
            if (stop_system_service_worker_ && system_service_jobs_.empty()) {
                return;
            }
            job = std::move(system_service_jobs_.front());
            system_service_jobs_.pop_front();
        }

        // 每个 handler 只转换一组 request/response，feature 执行仍在 sunray_system。
        switch (job.kind) {
        case SystemServiceJobKind::kFeatureList:
            handleFeatureListJob(job);
            break;
        case SystemServiceJobKind::kFeatureGet:
            handleFeatureGetJob(job);
            break;
        case SystemServiceJobKind::kFeatureStart:
            handleFeatureStartJob(job);
            break;
        case SystemServiceJobKind::kFeatureStop:
            handleFeatureStopJob(job);
            break;
        }
    }
}

/// 按配置的超时策略等待 Sunray ROS service 可用。
bool YunlinkRosBridgeNode::waitForService(ros::ServiceClient& client, const char* name) const {
    if (params_.system_service_timeout_sec <= 0.0) {
        return client.exists();
    }
    if (client.waitForExistence(ros::Duration(params_.system_service_timeout_sec))) {
        return true;
    }
    ROS_WARN("yunlink_ros_bridge service unavailable: %s", name);
    return false;
}

/// 调用 sunray_system/list_features 并发布 YunLink 响应。
void YunlinkRosBridgeNode::handleFeatureListJob(const SystemServiceJob& job) {
    yunlink::FeatureListResponse response{};
    const std::string name = service_name(params_.sunray_system_ns, "list_features");

    if (!waitForService(list_features_client_, name.c_str())) {
        response.success = false;
        response.message = "ros-service-unavailable: " + name;
    } else {
        sunray_msgs::ListFeatures srv;
        if (!list_features_client_.call(srv)) {
            response.success = false;
            response.message = "ros-service-call-failed: " + name;
        } else {
            response.success = true;
            response.message = "ok";
            response.feature_names = srv.response.feature_names;
        }
    }

    const auto ec =
        runtime_.system_service_publisher().publish_feature_list_response(job.inbound, response);
    if (ec != yunlink::ErrorCode::kOk) {
        ROS_WARN("yunlink_ros_bridge publish feature list response failed, ec=%u",
                 static_cast<unsigned>(ec));
    }
}

/// 调用 sunray_system/get_features 并发布 YunLink 响应。
void YunlinkRosBridgeNode::handleFeatureGetJob(const SystemServiceJob& job) {
    yunlink::FeatureGetResponse response{};
    const std::string name = service_name(params_.sunray_system_ns, "get_features");

    if (!waitForService(get_features_client_, name.c_str())) {
        response.success = false;
        response.message = "ros-service-unavailable: " + name;
        response.name = job.feature_get.feature_name;
    } else {
        sunray_msgs::GetFeatures srv;
        srv.request.feature_name = job.feature_get.feature_name;
        if (!get_features_client_.call(srv)) {
            response.success = false;
            response.message = "ros-service-call-failed: " + name;
            response.name = job.feature_get.feature_name;
        } else {
            response.success = srv.response.success;
            response.message = srv.response.message;
            response.name = srv.response.name;
            response.group = srv.response.group;
            response.running = srv.response.running;
            response.description = srv.response.description;
            response.auto_start = srv.response.auto_start;
            response.depends_on = srv.response.depends_on;
            response.stop_timeout_sec = srv.response.stop_timeout_sec;
            response.start_preview_units = srv.response.start_preview_units;
            response.start_preview_commands = srv.response.start_preview_commands;
        }
    }

    const auto ec =
        runtime_.system_service_publisher().publish_feature_get_response(job.inbound, response);
    if (ec != yunlink::ErrorCode::kOk) {
        ROS_WARN("yunlink_ros_bridge publish feature get response failed, ec=%u",
                 static_cast<unsigned>(ec));
    }
}

/// 调用 sunray_system/start_feature 并发布 YunLink 响应。
void YunlinkRosBridgeNode::handleFeatureStartJob(const SystemServiceJob& job) {
    yunlink::FeatureStartResponse response{};
    const std::string name = service_name(params_.sunray_system_ns, "start_feature");

    if (!waitForService(start_feature_client_, name.c_str())) {
        response.success = false;
        response.message = "ros-service-unavailable: " + name;
        response.feature_name = job.feature_start.feature_name;
    } else {
        sunray_msgs::StartFeature srv;
        srv.request.feature_name = job.feature_start.feature_name;
        srv.request.override_args = job.feature_start.override_args;
        srv.request.restart_if_running = job.feature_start.restart_if_running;
        srv.request.start_with_terminal = job.feature_start.start_with_terminal;
        if (!start_feature_client_.call(srv)) {
            response.success = false;
            response.message = "ros-service-call-failed: " + name;
            response.feature_name = job.feature_start.feature_name;
        } else {
            response.success = srv.response.success;
            response.message = srv.response.message;
            response.feature_name = job.feature_start.feature_name;
        }
    }

    const auto ec =
        runtime_.system_service_publisher().publish_feature_start_response(job.inbound, response);
    if (ec != yunlink::ErrorCode::kOk) {
        ROS_WARN("yunlink_ros_bridge publish feature start response failed, ec=%u",
                 static_cast<unsigned>(ec));
    }
}

/// 调用 sunray_system/stop_feature 并发布 YunLink 响应。
void YunlinkRosBridgeNode::handleFeatureStopJob(const SystemServiceJob& job) {
    yunlink::FeatureStopResponse response{};
    const std::string name = service_name(params_.sunray_system_ns, "stop_feature");

    if (!waitForService(stop_feature_client_, name.c_str())) {
        response.success = false;
        response.message = "ros-service-unavailable: " + name;
        response.feature_name = job.feature_stop.feature_name;
    } else {
        sunray_msgs::StopFeature srv;
        srv.request.feature_name = job.feature_stop.feature_name;
        srv.request.force = job.feature_stop.force;
        if (!stop_feature_client_.call(srv)) {
            response.success = false;
            response.message = "ros-service-call-failed: " + name;
            response.feature_name = job.feature_stop.feature_name;
        } else {
            response.success = srv.response.success;
            response.message = srv.response.message;
            response.feature_name = job.feature_stop.feature_name;
        }
    }

    const auto ec =
        runtime_.system_service_publisher().publish_feature_stop_response(job.inbound, response);
    if (ec != yunlink::ErrorCode::kOk) {
        ROS_WARN("yunlink_ros_bridge publish feature stop response failed, ec=%u",
                 static_cast<unsigned>(ec));
    }
}
