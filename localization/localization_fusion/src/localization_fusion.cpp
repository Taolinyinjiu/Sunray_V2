#include "localization_fusion.hpp"
#include "sunray_log.hpp"
#include "string_uav_namespace_utils.hpp"
#include "uav_param_utils.hpp"
#include <chrono>
#include <cmath>
#include <ctime>
#include <Eigen/Dense>
#include <iomanip>
#include <ros/package.h>
#include <sstream>
#include <stdexcept>
#include <sunray_msgs/OdomStatus.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// clang-format on

namespace {

enum class PanelLogSeverity {
    INFO,
    WARN,
    ERROR
};

constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiLightCyan = "\033[1;36m";
constexpr const char* kAnsiGreen = "\033[32m";
constexpr const char* kAnsiRed = "\033[31m";

const char* localization_mode_to_string(LocalizationMode mode) {
    switch (mode) {
    case LocalizationMode::LOCAL:
        return "LOCAL";
    case LocalizationMode::GLOBAL:
        return "GLOBAL";
    case LocalizationMode::LOCAL_AND_GLOBAL:
        return "LOCAL_AND_GLOBAL";
    case LocalizationMode::LOCAL_WITH_ARUCO:
        return "LOCAL_WITH_ARUCO";
    default:
        return "UNKNOWN";
    }
}

bool needs_relocalization(LocalizationMode mode) {
    return mode == LocalizationMode::LOCAL_AND_GLOBAL ||
           mode == LocalizationMode::LOCAL_WITH_ARUCO;
}

std::string bool_to_cn(bool value, const std::string& true_text, const std::string& false_text) {
    return value ? true_text : false_text;
}

std::string format_scalar(double value, int precision = 3) {
    if (!std::isfinite(value)) {
        return "nan";
    }
    std::ostringstream oss;
    oss << std::showpos << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

std::string format_vec3(double x, double y, double z, int precision = 3) {
    std::ostringstream oss;
    oss << format_scalar(x, precision) << " " << format_scalar(y, precision) << " "
        << format_scalar(z, precision);
    return oss.str();
}

std::string format_rate_hz(double value, int precision = 2) {
    if (!std::isfinite(value) || value <= 0.0) {
        return "未知";
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

std::string colorize_console_text(const std::string& text, const char* color) {
    return std::string(color) + text + kAnsiReset;
}

std::string format_odom_rpy_deg(const nav_msgs::Odometry& odom, int precision = 2) {
    const geometry_msgs::Quaternion& q = odom.pose.pose.orientation;
    const bool quat_finite =
        std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
    if (!quat_finite) {
        return "nan nan nan";
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    if (tf_q.length2() < 1e-12) {
        return "nan nan nan";
    }
    tf_q.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    constexpr double kRadToDeg = 180.0 / M_PI;
    return format_vec3(roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg, precision);
}

std::string safe_topic(const std::string& runtime_topic, const std::string& fallback_topic) {
    return runtime_topic.empty() ? fallback_topic : runtime_topic;
}

std::string parent_directory(const std::string& path) {
    const std::size_t pos = path.find_last_of('/');
    return (pos == std::string::npos) ? "" : path.substr(0, pos);
}

std::string make_startup_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_r(&now_time, &tm_now);

    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << "_" << std::setfill('0') << std::setw(3)
        << milliseconds.count();
    return oss.str();
}

void update_input_rate(double& averaged_rate_hz,
                       std::deque<double>& sample_times_s,
                       const double sample_time_s,
                       const std::size_t window_size = 20) {
    if (!std::isfinite(sample_time_s)) {
        return;
    }

    if (!sample_times_s.empty() && sample_time_s <= sample_times_s.back()) {
        if (std::abs(sample_time_s - sample_times_s.back()) < 1e-6) {
            return;
        }
        sample_times_s.clear();
    }

    sample_times_s.push_back(sample_time_s);
    while (sample_times_s.size() > window_size) {
        sample_times_s.pop_front();
    }

    if (sample_times_s.size() < 2) {
        return;
    }

    const double duration_s = sample_times_s.back() - sample_times_s.front();
    if (duration_s > 1e-4) {
        averaged_rate_hz = static_cast<double>(sample_times_s.size() - 1) / duration_s;
    }
}

}  // namespace

// 由于我们在launch文件中将传入的参数设置为了节点私有参数，因此需要在这里构造私有句柄
LocalizationFusion::LocalizationFusion(ros::NodeHandle& nh) {
    // 缓存全局句柄
    nh_ = nh;
    // 读取节点名
    std::string node_name = ros::this_node::getName();
    // 构造私有节点句柄，用于读取节点私有参数
    ros::NodeHandle private_nh_("~");
    // 读取节点参数
    if (!private_nh_.getParam("source_id", selected_source_id_)) {
        // 读取失败，抛出异常
        throw std::runtime_error("missing param" + node_name + "/source_id");
    }
    if (!private_nh_.getParam("config_yamlfile_path", config_yamlfile_path_)) {
        // 读取失败，抛出异常
        throw std::runtime_error("missing param" + node_name + "/config_yamlfile_path");
    }
    if (!private_nh_.getParam("health_rate_hz", health_rate_hz_)) {
        // 读取失败，抛出异常
        throw std::runtime_error("missing param" + node_name + "/health_rate_hz");
    }
    if (!private_nh_.getParam("use_receive_time", use_receive_time_)) {
        // 读取失败，抛出异常
        throw std::runtime_error("missing param" + node_name + "/use_receive_time");
    }
    private_nh_.param("log_save", log_save_, false);
    // 优先读取节点私有参数中的 uav_name/uav_id，单机场景再回退到全局参数
    uav_ns_ = localization_fusion::load_uav_namespace_or_throw(nh_);
    init_logger();
}

void LocalizationFusion::init_logger() {
    SunrayLogConfig cfg;
    cfg.name = "localization_fusion_" + uav_ns_;
    cfg.console_level = SunrayLogLevel::info;
    cfg.file_level = SunrayLogLevel::trace;
    cfg.async = false;

    if (log_save_) {
        log_file_path_ = make_log_file_path();
        cfg.file_path = log_file_path_;
    }

    SunrayLogger::instance().Init(cfg);
    auto logger = SunrayLogger::instance().Get();
    if (logger && !logger->sinks().empty()) {
        // 终端日志不显示 [info]/[warn] 前缀，状态面板的颜色在 printf_terminal 中单独控制。
        logger->sinks().front()->set_pattern("%v");
    }
    SUNRAY_INFO("localization_fusion logger initialized. log_save={} file_path={}",
                log_save_,
                log_save_ ? log_file_path_ : "disabled");
}

std::string LocalizationFusion::make_log_file_path() const {
    const std::string package_path = ros::package::getPath("localization_fusion");
    if (package_path.empty()) {
        throw std::runtime_error("failed to locate package path for localization_fusion");
    }

    const std::string localization_root = parent_directory(package_path);
    const std::string repo_root = parent_directory(localization_root);
    if (repo_root.empty()) {
        throw std::runtime_error("failed to infer repository root from package path: " +
                                 package_path);
    }

    return repo_root + "/log/localization_fusion/" + make_startup_timestamp() + ".log";
}

bool LocalizationFusion::load_param() {
    // 读取参数
    selected_source_ = load_config_from_yaml(config_yamlfile_path_, selected_source_id_, uav_ns_);
    // 只要不是默认值，就说明被覆盖过值，字段名都经过校验，内容由localization_sources.yaml填充
    if (selected_source_.source_id != -1) {
        has_selected_source_ = true;
        return true;
    }
    return false;
}

bool LocalizationFusion::Init() {
    bool init_state = false;
    // 首先调用load_param()函数读取参数配置
    init_state = load_param();
    // 加载参数失败，结束
    if (init_state == false) {
        return init_state;
    }
    // 加载参数成功，假设参数都正常
    // 首先将输入输出的字符串中含有 "${uav_ns}"的部分进行转译

    // 输入话题
    selected_source_.odometry_topic =
        sunray_common::replace_uav_ns(selected_source_.odometry_topic, uav_ns_);
    // 检查，输入里程计话题不能为空
    if (selected_source_.odometry_topic.empty()) {
        throw std::runtime_error("selected source config the odometry topic missing value");
    }
    // 如果模式要求有重定位的输入，就转换一下
    if (selected_source_.localization_mode != LocalizationMode::LOCAL &&
        selected_source_.localization_mode != LocalizationMode::GLOBAL) {
        selected_source_.relocalization_topic =
            sunray_common::replace_uav_ns(selected_source_.relocalization_topic, uav_ns_);
        if (selected_source_.relocalization_topic.empty()) {
            throw std::runtime_error(
                "selected source config the relocalizaiton topic missing value");
        }
    }

    // 输出话题
    global_odometry_topic_ = sunray_common::replace_uav_ns(global_odometry_topic_, uav_ns_);
    local_odometry_topic_ = sunray_common::replace_uav_ns(local_odometry_topic_, uav_ns_);
    odom_status_topic_ = sunray_common::replace_uav_ns(odom_status_topic_, uav_ns_);
    // 检查，输出话题不能为空
    if (global_odometry_topic_.empty() || local_odometry_topic_.empty() ||
        odom_status_topic_.empty()) {
        throw std::runtime_error("localization fusion config has empty topic");
    }

    // 注册订阅者
    odometry_sub_ = nh_.subscribe(
        selected_source_.odometry_topic, 50, &LocalizationFusion::odometry_callback, this);
    if (selected_source_.localization_mode != LocalizationMode::LOCAL &&
        selected_source_.localization_mode != LocalizationMode::GLOBAL) {
        relocalization_sub_ = nh_.subscribe(selected_source_.relocalization_topic,
                                            50,
                                            &LocalizationFusion::relocalization_callback,
                                            this);
    }

    // 注册发布者
    local_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(local_odometry_topic_, 10);
    global_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(global_odometry_topic_, 10);
    odom_state_pub_ = nh_.advertise<sunray_msgs::OdomStatus>(odom_status_topic_, 10);

    // 注册定时器
    // 考虑到在构造函数读取参数时没有进行检查，这里构造之前检查一下，要求频率至少在1Hz
    health_rate_hz_ = std::max(1.0, health_rate_hz_);
    health_timer_ = nh_.createTimer(
        ros::Duration(1.0 / health_rate_hz_), &LocalizationFusion::healthtimer_callback, this);

    // 初始化tf数据
    // sunray_global -> sunray_local
    global_to_local_tf_.header.frame_id = global_frame_id_;
    global_to_local_tf_.child_frame_id = local_frame_id_;
    // 设置为原点重合
    global_to_local_tf_.transform.translation.x = 0.0;
    global_to_local_tf_.transform.translation.y = 0.0;
    global_to_local_tf_.transform.translation.z = 0.0;
    // 姿态设置为单位阵
    global_to_local_tf_.transform.rotation.x = 0.0;
    global_to_local_tf_.transform.rotation.y = 0.0;
    global_to_local_tf_.transform.rotation.z = 0.0;
    global_to_local_tf_.transform.rotation.w = 1.0;

    // 检查当前的重定位模式，如果是local和global模式，他们并不需要relocalization,因此只需要一次静态的tf
    if (selected_source_.localization_mode == LocalizationMode::LOCAL ||
        selected_source_.localization_mode == LocalizationMode::GLOBAL) {
        tf_static_broadcaster_.sendTransform(global_to_local_tf_);
    }

    // 返回初始化状态
    init_state = true;
    return init_state;
}

void LocalizationFusion::odometry_callback(const nav_msgs::OdometryConstPtr& msg) {
    update_input_rate(odometry_input_rate_hz_,
                      odometry_rate_samples_s_,
                      ros::WallTime::now().toSec());
    // 将里程计转换为sunray_local系下的数据进行发送
    nav_msgs::Odometry temp_msg = *msg;  // 首先解引用，拿到里程计的值
    last_odometry_data_ = temp_msg;
    has_odometry_data_ = true;
    // 改时间戳
    if (use_receive_time_) {
        temp_msg.header.stamp = ros::Time::now();
    }
    // 改frame
    temp_msg.header.frame_id = local_frame_id_;
    temp_msg.child_frame_id = base_frame_id_;
    // 发布
    local_odom_pub_.publish(temp_msg);
    if (selected_source_.localization_mode != LocalizationMode::LOCAL_AND_GLOBAL) {
        publish_global_odom_from_local(temp_msg);  // 根据tf构造输出
    }
    // 发布完了记录一下时间戳
    last_odometry_rx_time_ = temp_msg.header.stamp;
    last_odometry_data_ = temp_msg;
    // 同步发送TF
    broadcast_local_to_base_tf(temp_msg);
}

void LocalizationFusion::publish_global_odom_from_local(const nav_msgs::Odometry& msg) {
    nav_msgs::Odometry global_msg = msg;
    global_msg.header.frame_id = global_frame_id_;
    global_msg.child_frame_id = base_frame_id_;
    global_msg.header.stamp = msg.header.stamp;
    if (selected_source_.localization_mode != LocalizationMode::LOCAL_WITH_ARUCO) {
        // 直接发送，然后退出
        global_odom_pub_.publish(global_msg);
        return;
    }
    // 如果是aruco辅助的话，需要考虑当前的tf变换对是否被修改了已经
    tf2::Transform T_global_local;
    tf2::Transform T_local_base;

    // sunray_global -> sunray_local
    tf2::fromMsg(global_to_local_tf_.transform, T_global_local);
    // sunray_local -> base_link
    tf2::fromMsg(msg.pose.pose, T_local_base);
    // sunray_global -> base_link
    const tf2::Transform T_global_base = T_global_local * T_local_base;
    tf2::toMsg(T_global_base, global_msg.pose.pose);

    // TODO: 这里对速度的处理需要考虑
    global_msg.twist = msg.twist;

    global_odom_pub_.publish(global_msg);
}

void LocalizationFusion::relocalization_callback(const nav_msgs::OdometryConstPtr& msg) {
    update_input_rate(relocalization_input_rate_hz_,
                      relocalization_rate_samples_s_,
                      ros::WallTime::now().toSec());
    // 输出global系下的里程计，并根据差值重构tf
    nav_msgs::Odometry global_msg = *msg;
    // 首先要进行输入保护，由于tf是严格的计算过程，因此我们要先保证输入的数据是正常的
    // 1. 有界性：xyz不能是无限值
    // 2. 单位性：odometry使用四元数表示姿态，四元数需要是单位四元数
    // 提取位置与姿态
    Eigen::Vector3d position(global_msg.pose.pose.position.x,
                             global_msg.pose.pose.position.y,
                             global_msg.pose.pose.position.z);
    Eigen::Quaterniond quad(global_msg.pose.pose.orientation.w,
                            global_msg.pose.pose.orientation.x,
                            global_msg.pose.pose.orientation.y,
                            global_msg.pose.pose.orientation.z);

    bool odometry_finite =
        std::isfinite(position.x()) && std::isfinite(position.y()) && std::isfinite(position.z());
    if (!odometry_finite) {
        relocalization_data_valid_ = false;
        return;
    }
    bool odometry_quad_safe = std::isfinite(quad.x()) && std::isfinite(quad.y()) &&
                              std::isfinite(quad.z()) && std::isfinite(quad.w()) &&
                              std::abs(quad.x() * quad.x() + quad.y() * quad.y() +
                                       quad.z() * quad.z() + quad.w() * quad.w() - 1.0) < 1e-2;
    if (!odometry_quad_safe) {
        relocalization_data_valid_ = false;
        return;
    }
    // 本来我是想处理完如果输入有问题抛出异常的，但是考虑到飞行过程总如果出现问题抛出异常，会导致local数据中断发送，因此这里我选择的是置标志位，然后return
    relocalization_data_valid_ = true;
    last_relocalization_data_ = *msg;
    has_relocalization_data_ = true;
    if (selected_source_.localization_mode == LocalizationMode::LOCAL_AND_GLOBAL) {
        global_msg.header.frame_id = global_frame_id_;
        global_msg.child_frame_id = base_frame_id_;
        global_msg.header.stamp = msg->header.stamp;
        // 发布
        global_odom_pub_.publish(global_msg);
    }
    // 重构tf树
    // 先确认已经有 local odom，否则没法反推出 global -> local
    if (!has_odometry_data_) {
        return;
    }
    tf2::Transform T_global_base;
    tf2::Transform T_local_base;
    // base_link in sunray_global
    tf2::fromMsg(global_msg.pose.pose, T_global_base);

    // base_link in sunray_local
    tf2::fromMsg(last_odometry_data_.pose.pose, T_local_base);

    // 反推出 sunray_global -> sunray_local
    const tf2::Transform T_global_local = T_global_base * T_local_base.inverse();

    global_to_local_tf_.header.stamp = global_msg.header.stamp;
    global_to_local_tf_.header.frame_id = global_frame_id_;
    global_to_local_tf_.child_frame_id = local_frame_id_;
    global_to_local_tf_.transform = tf2::toMsg(T_global_local);

    last_relocalization_data_ = global_msg;
    has_relocalization_data_ = true;

    // 动态广播 global -> local
    if (selected_source_.localization_mode == LocalizationMode::LOCAL_AND_GLOBAL) {
        // 只有在lidar的高频里程计时，才选择在这里发布tf，因为aruco的频率不定，转移到healthtimer_callback更好
        tf_broadcaster_.sendTransform(global_to_local_tf_);
    }
}

void LocalizationFusion::healthtimer_callback(const ros::TimerEvent& e) {
    // 本函数主要负责
    // 1. 检查Localization_Fusion各变量状态
    // 2. 当使用aruco进行重定位时，在这里持续发送动态tf树
    // 3，检查通信链路是否超时，只检查odometry,不检查relocalization
    // 4. 填充sunray_msgs::OdomStatus消息并发布

    if (selected_source_.localization_mode == LocalizationMode::LOCAL_WITH_ARUCO) {
        // 默认值为原点重合
        global_to_local_tf_.header.stamp = ros::Time::now();
        tf_broadcaster_.sendTransform(global_to_local_tf_);
    }
    // 对odometry通信链路的检查需要先接受到数据
    if (has_odometry_data_) {
        // 在拥有数据的基础上，用当前时间和最新接收时间做差，如果大于配置结构体中的默认参数，则判断为超时
        if ((ros::Time::now() - last_odometry_rx_time_).toSec() > selected_source_.timeout_s) {
            odometry_data_timeout_ = true;
        } else {
            odometry_data_timeout_ = false;
        }
    }
    // 构建sunray_msgs::OdomStatus消息
    sunray_msgs::OdomStatus status_msgs;
    status_msgs.header.stamp = ros::Time::now();
    status_msgs.external_source_name = selected_source_.source_name;
    status_msgs.external_source_id = selected_source_id_;
    switch (selected_source_.localization_mode) {
    case LocalizationMode::LOCAL:
        status_msgs.localization_mode = sunray_msgs::OdomStatus::LOCAL;
        status_msgs.localization_mode_name = "LOCAL";
        break;
    case LocalizationMode::GLOBAL:
        status_msgs.localization_mode = sunray_msgs::OdomStatus::GLOBAL;
        status_msgs.localization_mode_name = "GLOBAL";
        break;
    case LocalizationMode::LOCAL_AND_GLOBAL:
        status_msgs.localization_mode = sunray_msgs::OdomStatus::LOCAL_AND_GLOBAL;
        status_msgs.localization_mode_name = "LOCAL_AND_GLOBAL";
        break;
    case LocalizationMode::LOCAL_WITH_ARUCO:
        status_msgs.localization_mode = sunray_msgs::OdomStatus::LOCAL_WITH_ARUCO;
        status_msgs.localization_mode_name = "LOCAL_WITH_ARUCO";
        break;
    }
    status_msgs.has_odometry = has_odometry_data_;
    status_msgs.has_relocalization = has_relocalization_data_;
    status_msgs.last_odometry_time = last_odometry_rx_time_;
    status_msgs.odom_timeout = odometry_data_timeout_;
    status_msgs.global_frame_id = global_frame_id_;
    status_msgs.local_frame_id = local_frame_id_;
    status_msgs.base_frame_id = base_frame_id_;
    status_msgs.relocalization_data_valid = relocalization_data_valid_;
    // 向外发布
    odom_state_pub_.publish(status_msgs);

    printf_terminal();
}

void LocalizationFusion::broadcast_local_to_base_tf(const nav_msgs::Odometry& local_odom) {
    // 填充帧头
    local_to_base_tf_.header.stamp = local_odom.header.stamp;
    local_to_base_tf_.header.frame_id = local_frame_id_;
    local_to_base_tf_.child_frame_id = base_frame_id_;
    // 填充xyz数据
    local_to_base_tf_.transform.translation.x = local_odom.pose.pose.position.x;
    local_to_base_tf_.transform.translation.y = local_odom.pose.pose.position.y;
    local_to_base_tf_.transform.translation.z = local_odom.pose.pose.position.z;
    // 填充姿态数据
    local_to_base_tf_.transform.rotation = local_odom.pose.pose.orientation;
    // 发布
    tf_broadcaster_.sendTransform(local_to_base_tf_);
}

void LocalizationFusion::Spin() {
    ros::spin();
}

void LocalizationFusion::printf_terminal(){
    static ros::Time last_print_time(0);
    const ros::Time now = ros::Time::now();
    if (last_print_time != ros::Time(0) && (now - last_print_time).toSec() < 1.0) {
        return;
    }
    last_print_time = now;

    PanelLogSeverity severity = PanelLogSeverity::INFO;
    std::string source_status = "正常";
    if (!has_odometry_data_) {
        source_status = "未收到里程计";
        severity = PanelLogSeverity::WARN;
    } else if (odometry_data_timeout_) {
        source_status = "里程计超时";
        severity = PanelLogSeverity::ERROR;
    } else if (selected_source_.localization_mode == LocalizationMode::LOCAL_AND_GLOBAL &&
               !has_relocalization_data_) {
        source_status = "等待全局里程计";
        severity = PanelLogSeverity::WARN;
    } else if (selected_source_.localization_mode == LocalizationMode::LOCAL_WITH_ARUCO &&
               !has_relocalization_data_) {
        source_status = "正常(未收到重定位)";
        severity = PanelLogSeverity::WARN;
    } else if (needs_relocalization(selected_source_.localization_mode) &&
               has_relocalization_data_ && !relocalization_data_valid_) {
        source_status = "重定位数据无效";
        severity = PanelLogSeverity::ERROR;
    }

    const std::string odom_sub_topic =
        safe_topic(odometry_sub_.getTopic(), selected_source_.odometry_topic);
    const std::string relocalization_sub_topic =
        safe_topic(relocalization_sub_.getTopic(), selected_source_.relocalization_topic);
    const std::string local_pub_topic =
        safe_topic(local_odom_pub_.getTopic(), local_odometry_topic_);
    const std::string global_pub_topic =
        safe_topic(global_odom_pub_.getTopic(), global_odometry_topic_);
    const std::string odom_status_pub_topic =
        safe_topic(odom_state_pub_.getTopic(), odom_status_topic_);

    const std::string odometry_rate_text = format_rate_hz(odometry_input_rate_hz_);
    const std::string relocalization_rate_text = format_rate_hz(relocalization_input_rate_hz_);
    const std::string odometry_rx_text = bool_to_cn(has_odometry_data_, "已收到", "未收到");
    const std::string odometry_timeout_text = bool_to_cn(odometry_data_timeout_, "超时", "正常");
    const std::string relocalization_rx_text =
        bool_to_cn(has_relocalization_data_, "已收到", "未收到");
    const std::string relocalization_valid_text =
        bool_to_cn(relocalization_data_valid_, "有效", "无效/未校验");

    const std::string console_source_status =
        colorize_console_text(source_status,
                              severity == PanelLogSeverity::INFO ? kAnsiGreen : kAnsiRed);
    const std::string console_odometry_rate =
        colorize_console_text(odometry_rate_text,
                              odometry_input_rate_hz_ > 0.0 ? kAnsiGreen : kAnsiRed);
    const std::string console_odometry_rx =
        colorize_console_text(odometry_rx_text, has_odometry_data_ ? kAnsiGreen : kAnsiRed);
    const std::string console_odometry_timeout =
        colorize_console_text(odometry_timeout_text, odometry_data_timeout_ ? kAnsiRed : kAnsiGreen);
    const std::string console_relocalization_rate =
        colorize_console_text(relocalization_rate_text,
                              relocalization_input_rate_hz_ > 0.0 ? kAnsiGreen : kAnsiRed);
    const std::string console_relocalization_rx = colorize_console_text(
        relocalization_rx_text, has_relocalization_data_ ? kAnsiGreen : kAnsiRed);
    const std::string console_relocalization_valid =
        colorize_console_text(relocalization_valid_text,
                              relocalization_data_valid_ ? kAnsiGreen : kAnsiRed);

    std::ostringstream body_oss;
    std::ostringstream console_body_oss;
    body_oss << std::fixed << std::setprecision(3);
    console_body_oss << std::fixed << std::setprecision(3);
    body_oss << " -------- 外部定位源\n";
    console_body_oss << " -------- 外部定位源\n";
    body_oss << " 外部定位源名称: [ " << selected_source_.source_name << " ]\n";
    console_body_oss << " 外部定位源名称: [ " << selected_source_.source_name << " ]\n";
    body_oss << " 外部定位源模式: [ "
             << localization_mode_to_string(selected_source_.localization_mode) << " ]\n";
    console_body_oss << " 外部定位源模式: [ "
                     << localization_mode_to_string(selected_source_.localization_mode) << " ]\n";
    body_oss << " 外部定位源状态: [ " << source_status << " ]\n";
    console_body_oss << " 外部定位源状态: [ " << console_source_status << " ]\n";

    body_oss << " -------- 当前订阅的里程计相关信息\n";
    console_body_oss << " -------- 当前订阅的里程计相关信息\n";
    body_oss << " odometry话题: " << odom_sub_topic << "\n";
    console_body_oss << " odometry话题: " << odom_sub_topic << "\n";
    body_oss << " odometry上游发布频率: " << odometry_rate_text << " [Hz]\n";
    console_body_oss << " odometry上游发布频率: " << console_odometry_rate << " [Hz]\n";
    body_oss << " odometry接收状态: [ " << odometry_rx_text << " ]\n";
    console_body_oss << " odometry接收状态: [ " << console_odometry_rx << " ]\n";
    body_oss << " odometry超时状态: [ " << odometry_timeout_text << " ]\n";
    console_body_oss << " odometry超时状态: [ " << console_odometry_timeout << " ]\n";
    if (needs_relocalization(selected_source_.localization_mode)) {
        body_oss << " relocalization话题: " << relocalization_sub_topic << "\n";
        console_body_oss << " relocalization话题: " << relocalization_sub_topic << "\n";
        body_oss << " relocalization上游发布频率: " << relocalization_rate_text << " [Hz]\n";
        console_body_oss << " relocalization上游发布频率: " << console_relocalization_rate
                         << " [Hz]\n";
        body_oss << " relocalization接收状态: [ " << relocalization_rx_text << " ]\n";
        console_body_oss << " relocalization接收状态: [ " << console_relocalization_rx << " ]\n";
        body_oss << " relocalization数据状态: [ " << relocalization_valid_text << " ]\n";
        console_body_oss << " relocalization数据状态: [ " << console_relocalization_valid
                         << " ]\n";
    }

    body_oss << " -------- 当前发布的里程计相关信息\n";
    console_body_oss << " -------- 当前发布的里程计相关信息\n";
    body_oss << " local_odom发布话题: " << local_pub_topic << "\n";
    console_body_oss << " local_odom发布话题: " << local_pub_topic << "\n";
    body_oss << " global_odom发布话题: " << global_pub_topic << "\n";
    console_body_oss << " global_odom发布话题: " << global_pub_topic << "\n";
    body_oss << " odom_status发布话题: " << odom_status_pub_topic << "\n";
    console_body_oss << " odom_status发布话题: " << odom_status_pub_topic << "\n";

    body_oss << " -------- 当前里程计数据\n";
    console_body_oss << " -------- 当前里程计数据\n";
    if (!has_odometry_data_) {
        body_oss << " 尚未收到里程计数据\n";
        console_body_oss << " 尚未收到里程计数据\n";
    } else {
        body_oss << " 时间戳: " << last_odometry_data_.header.stamp.toSec()
                 << " [s]  坐标系: [ " << last_odometry_data_.header.frame_id << " -> "
                 << last_odometry_data_.child_frame_id << " ]\n";
        console_body_oss << " 时间戳: " << last_odometry_data_.header.stamp.toSec()
                         << " [s]  坐标系: [ " << last_odometry_data_.header.frame_id << " -> "
                         << last_odometry_data_.child_frame_id << " ]\n";
        body_oss << " 位置[X Y Z]: "
                 << format_vec3(last_odometry_data_.pose.pose.position.x,
                                last_odometry_data_.pose.pose.position.y,
                                last_odometry_data_.pose.pose.position.z)
                 << " [m]\n";
        console_body_oss << " 位置[X Y Z]: "
                         << format_vec3(last_odometry_data_.pose.pose.position.x,
                                        last_odometry_data_.pose.pose.position.y,
                                        last_odometry_data_.pose.pose.position.z)
                         << " [m]\n";
        body_oss << " 速度[X Y Z]: "
                 << format_vec3(last_odometry_data_.twist.twist.linear.x,
                                last_odometry_data_.twist.twist.linear.y,
                                last_odometry_data_.twist.twist.linear.z)
                 << " [m/s]\n";
        console_body_oss << " 速度[X Y Z]: "
                         << format_vec3(last_odometry_data_.twist.twist.linear.x,
                                        last_odometry_data_.twist.twist.linear.y,
                                        last_odometry_data_.twist.twist.linear.z)
                         << " [m/s]\n";
        body_oss << " 姿态[RPY]: " << format_odom_rpy_deg(last_odometry_data_) << " [deg]\n";
        console_body_oss << " 姿态[RPY]: " << format_odom_rpy_deg(last_odometry_data_)
                         << " [deg]\n";
    }

    if (needs_relocalization(selected_source_.localization_mode) && has_relocalization_data_) {
        body_oss << " -------- 当前重定位数据\n";
        console_body_oss << " -------- 当前重定位数据\n";
        body_oss << " 时间戳: " << last_relocalization_data_.header.stamp.toSec()
                 << " [s]  坐标系: [ " << last_relocalization_data_.header.frame_id << " -> "
                 << last_relocalization_data_.child_frame_id << " ]\n";
        console_body_oss << " 时间戳: " << last_relocalization_data_.header.stamp.toSec()
                         << " [s]  坐标系: [ " << last_relocalization_data_.header.frame_id
                         << " -> " << last_relocalization_data_.child_frame_id << " ]\n";
        body_oss << " 位置[X Y Z]: "
                 << format_vec3(last_relocalization_data_.pose.pose.position.x,
                                last_relocalization_data_.pose.pose.position.y,
                                last_relocalization_data_.pose.pose.position.z)
                 << " [m]\n";
        console_body_oss << " 位置[X Y Z]: "
                         << format_vec3(last_relocalization_data_.pose.pose.position.x,
                                        last_relocalization_data_.pose.pose.position.y,
                                        last_relocalization_data_.pose.pose.position.z)
                         << " [m]\n";
        body_oss << " 姿态[RPY]: " << format_odom_rpy_deg(last_relocalization_data_)
                 << " [deg]\n";
        console_body_oss << " 姿态[RPY]: " << format_odom_rpy_deg(last_relocalization_data_)
                         << " [deg]\n";
    }

    body_oss << " ---------------------------------------------------------\n";
    console_body_oss << " ---------------------------------------------------------\n";

    const std::string header_line =
        ">>>>>>>>>>>>>>> 定位融合状态 - [ " + uav_ns_ + " ] <<<<<<<<<<<<<<<";
    const std::string plain_panel = header_line + "\n" + body_oss.str();
    const std::string console_panel =
        std::string(kAnsiLightCyan) + header_line + kAnsiReset + "\n" + console_body_oss.str();

    auto logger = SunrayLogger::instance().Get();
    if (!logger) {
        return;
    }

    spdlog::level::level_enum spdlog_level = spdlog::level::info;
    switch (severity) {
    case PanelLogSeverity::INFO:
        spdlog_level = spdlog::level::info;
        break;
    case PanelLogSeverity::WARN:
        spdlog_level = spdlog::level::warn;
        break;
    case PanelLogSeverity::ERROR:
        spdlog_level = spdlog::level::err;
        break;
    }

    auto log_to_sink = [&](const spdlog::sink_ptr& sink, const std::string& text) {
        if (!sink || !sink->should_log(spdlog_level)) {
            return;
        }
        const spdlog::string_view_t payload(text.data(), text.size());
        const spdlog::details::log_msg msg(logger->name(), spdlog_level, payload);
        sink->log(msg);
        sink->flush();
    };

    auto& sinks = logger->sinks();
    if (!sinks.empty()) {
        log_to_sink(sinks.front(), console_panel);
    }
    for (std::size_t i = 1; i < sinks.size(); ++i) {
        log_to_sink(sinks[i], plain_panel);
    }
}
