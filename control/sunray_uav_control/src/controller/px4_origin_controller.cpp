#include "controller/px4_origin_controller.hpp"
#include "string_uav_namespace_utils.hpp"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>  // 引入Yaml-cpp库，用于读取yaml文件
#include <sunray_msgs/UAVControllerState.h>
#include <px4_param_manager/px4_param_types.h>
#include <px4_param_manager/px4_param_decode.h>
// 构造函数，读取参数
PX4_OriginController::PX4_OriginController(ros::NodeHandle& nh)
    : nh_(nh), mavros_helper_(nh_), mavros_param_(nh_) {
    // 读取节点名
    std::string node_name = ros::this_node::getName();
    // 构造私有节点句柄，用于读取节点私有参数
    ros::NodeHandle private_nh_("~");
    if (!private_nh_.getParam("config_yamlfile_path", config_yamlfile_path_)) {
        // 读取失败，抛出异常
        throw std::runtime_error("missing param" + node_name + "/config_yamlfile_path");
    }
    if (config_yamlfile_path_.empty()) {
        // 如果路径参数为空，也抛出异常
        throw std::runtime_error("yaml_path connot be empty");
    }
    // 读取全局参数
    std::string uav_name;
    int uav_id;
    if (!nh_.getParam("/uav_name", uav_name)) {
        // 读取失败，抛出异常
        throw std::runtime_error("missing param /uav_name");
    }
    if (!nh_.getParam("/uav_id", uav_id)) {
        // 读取失败，抛出异常
        throw std::runtime_error("missing param /uav_id");
    }
    // 拼接uav_ns
    uav_ns_ = uav_name + std::to_string(uav_id);
    // 标准化
    uav_ns_ = sunray_common::normalize_uav_ns(uav_ns_);
}

// init()读取config.yaml文件中的相关配置

bool PX4_OriginController::init() {
    // 首先在构造函数中我们已经判断了config_yamlfile_path_非空
    YAML::Node root;  // 构造一个YAML文件的根节点
    // 由于读取的过程可能引发异常，因此使用try语法
    try {
        // 从指定的路径中读取yaml文件并解析为YAML::Node
        root = YAML::LoadFile(config_yamlfile_path_);
    } catch (const YAML::Exception& e) {  // 如果解析的过程中发生错误，捕捉到异常
        throw std::runtime_error("Failed to load yaml file '" + config_yamlfile_path_ + ":" +
                                 e.what());
    }
    // 顺利读取，取出字段basic_param的部分
    const YAML::Node basic_param = root["basic_param"];
    // 如果sources_list为空，或者不是键值对的形式，则抛出异常
    if (!basic_param || !basic_param.IsMap()) {
        throw std::runtime_error("the yaml file '" + config_yamlfile_path_ +
                                 "' is missing a valid sources_list map");
    }
    // 由于OriginalController需要的参数不多，我们直接拿字段读
    if (!basic_param["fuse_odom_type"]) {
        throw std::runtime_error("miss param 'fuse_odom_type'");
    } else {
        config_param_.fuse_odom_type = basic_param["fuse_odom_type"].as<int>();
    }
    if (!basic_param["fuse_odom_frequency"]) {
        throw std::runtime_error("miss param 'fuse_odom_frequency'");
    } else {
        config_param_.fuse_odom_frequency = basic_param["fuse_odom_frequency"].as<double>();
    }
    // 配置读取完毕，初始化mavros_helper
    MavrosHelper_ConfigList config_list(true);
    if (!mavros_helper_.init(config_list)) {
        throw std::runtime_error("mavros_helper init failed");
    }
    if (config_param_.fuse_odom_type == 1 ||
        config_param_.fuse_odom_type == 2) {  // 融合外部里程计到px4，设置对应参数,注册定时器
        // 检查外部里程计融合相关参数
        param_state_ = check_param();
        if (param_state_ != true) {
            throw std::runtime_error("EKF2 parameter check failed after reboot");
        }
        mavros_helper_.set_vision_fuse_type(config_param_.fuse_odom_type);
        // 创建定时器之前对定时器频率进行处理,限制范围在10.0 ~ 200 Hz
        config_param_.fuse_odom_frequency = std::max(10.0, config_param_.fuse_odom_frequency);
        config_param_.fuse_odom_frequency = std::min(200.0, config_param_.fuse_odom_frequency);
        pub_vision_pose_timer_ =
            nh_.createTimer(ros::Duration(1.0 / config_param_.fuse_odom_frequency),
                            &PX4_OriginController::pub_vision_fuse_timer_cb,
                            this);
    } else if (config_param_.fuse_odom_type != 0) {
        throw std::runtime_error("param 'fuse_odom_type' value is error");
    }
    // 初始化话题发布者
    controller_state_pub_ =
        nh_.advertise<sunray_msgs::UAVControllerState>(uav_ns_ + "/sunray/controller_state", 10);
    // 初始化发布px4_state数据的定时器
    pub_px4_state_timer = nh_.createTimer(ros::Duration(1.0 / pub_px4_state_freq_),
                                          &PX4_OriginController::pub_px4_state_timer_cb,
                                          this);
    return true;
}

void PX4_OriginController::pub_px4_state_timer_cb(const ros::TimerEvent&) {
    mavros_helper_.pub_px4_state();
}

void PX4_OriginController::pub_vision_fuse_timer_cb(const ros::TimerEvent&) {
    if (has_uav_odometry_) {
        mavros_helper_.pub_vision_pose(uav_odometry_);
    }
}

bool PX4_OriginController::check_param() {
    // 首先拉出来参数结构体，暂时只考虑这两个,先读取，如果参数不对再写入
    px4_param_decode::EKF2_EV_CTRL ev_ctrl_read;
    px4_param_decode::EKF2_HGT_REF hgt_ref_read;
    mavros_param_.read_param(&ev_ctrl_read);
    mavros_param_.read_param(&hgt_ref_read);
    // 这里由于一些原因，暂时先不关心velocity
    bool ev_state = ev_ctrl_read.enable_horizontal_position() &&
                    ev_ctrl_read.enable_vertical_position() && ev_ctrl_read.enable_yaw();
    bool hgt_state = hgt_ref_read.is_vision();
    if (ev_state == true && hgt_state == true) {
        return true;
    }
    if (ev_state != true) {
        px4_param_types::EKF2_EV_CTRL ev_ctrl_write;
        ev_ctrl_write.enable_Horizontalposition();
        ev_ctrl_write.enable_Verticalposition();
        ev_ctrl_write.enable_Yaw();
        mavros_param_.set_param(ev_ctrl_write);
    }
    if (hgt_state != true) {
        px4_param_types::EKF2_HGT_REF hgt_ref_write;
        hgt_ref_write.enable_vision();
        mavros_param_.set_param(hgt_ref_write);
    }
    if (ev_state != true || hgt_state != true) {
        // TODO:重启ekf2组件
    }
    // 再次检查
    mavros_param_.read_param(&ev_ctrl_read);
    mavros_param_.read_param(&hgt_ref_read);
    ev_state = ev_ctrl_read.enable_horizontal_position() &&
               ev_ctrl_read.enable_vertical_position() && ev_ctrl_read.enable_yaw();
    hgt_state = hgt_ref_read.is_vision();
    if (!(ev_state == true && hgt_state == true)) {
        return false;
    }
    // 切换模式为position模式
    bool set_mode_state = mavros_helper_.set_px4_mode(control_common::FlightMode::Posctl);
    if (!set_mode_state) {
        // 如果模式切换失败应该怎么办呢？在参数有效的情况下切换模式失败。。。
    }
}

void PX4_OriginController::set_current_odom(const control_common::UAVStateEstimate& odom) {
    uav_odometry_ = odom;
    has_uav_odometry_ = true;
}

bool PX4_OriginController::is_ready() {
    control_common::Mavros_State mavros_state = mavros_helper_.get_state();
    px4_mode_ = mavros_state.flight_mode;
    px4_land_ = mavros_state.landed_state;
    px4_arm_ = mavros_state.armed;
    if (px4_mode_ == control_common::FlightMode::Posctl &&
        px4_land_ == control_common::LandedState::OnGround && px4_arm_ == false &&
        has_uav_odometry_ == true) {
        return true;
    } else {
        return false;
    }
}
