#include "statemachine/sunray_fsm_loadparam.hpp"
#include "utils/orientation_utils.hpp"
#include <stdexcept>
#include <ros/ros.h>

// 从yaml文件构造的节点中，读取basic_param字段内容
void loadBasicParam(const YAML::Node& node, sunray_fsm::basic_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error(
            "the sunray control config basic_param is missing a valid basic_param map");
    }
    // 先判断存在，再读取值，再判断值是否允许
    // -----------------------控制器类型----------------------
    if (!node["controller_types"]) {
        throw std::runtime_error("the sunray control config miss param 'controller_types'");
    } else {
        param.controller_types = node["controller_types"].as<int>();
        if (param.controller_types != 0 && param.controller_types != 1) {
            throw std::runtime_error(
                "the sunray control config param 'controller_types' only can 0 or 1");
        }
    }
    // -----------------------控制器更新频率----------------------
    if (!node["controller_update_frequency"]) {
        throw std::runtime_error(
            "the sunray control config miss param 'controller_update_frequency'");
    } else {
        param.controller_update_frequency = node["controller_update_frequency"].as<double>();
        if (param.controller_update_frequency <= 10) {
            throw std::runtime_error(
                "the sunray control config param 'controller_update_frequency' must > 10");
        }
    }
    // -----------------------状态机检查频率----------------------
    if (!node["supervisor_update_frequency"]) {
        throw std::runtime_error(
            "the sunray control config miss param 'supervisor_update_frequency'");
    } else {
        param.supervisor_update_frequency = node["supervisor_update_frequency"].as<double>();
        if (param.supervisor_update_frequency <= 0) {
            throw std::runtime_error(
                "the sunray control config param 'supervisor_update_frequency' must > 0");
        }
    }
    // -----------------------里程计话题----------------------
    if (!node["odom_topic_name"]) {
        throw std::runtime_error("the sunray control config miss param 'odom_topic_name'");
    } else {
        param.odom_topic_name = node["odom_topic_name"].as<std::string>();
        if (param.odom_topic_name.empty()) {
            throw std::runtime_error(
                "the sunray control config param 'odom_topic_name' is empty");
        }
    }
    // -----------------------px4融合类型----------------------
    if (!node["fuse_odom_type"]) {
        throw std::runtime_error("the sunray control config miss param 'fuse_odom_type'");
    } else {
        param.fuse_odom_type = node["fuse_odom_type"].as<int>();
        if (param.fuse_odom_type != 0 && param.fuse_odom_type != 1 && param.fuse_odom_type != 2) {
            throw std::runtime_error(
                "the sunray control config param 'fuse_odom_type' only can 0, 1, 2");
        }
    }
    // ---------------------px4融合频率--------------------
    if (!node["fuse_odom_frequency"]) {
        throw std::runtime_error("the sunray control config miss param 'fuse_odom_frequency'");
    } else {
        param.fuse_odom_frequency = node["fuse_odom_frequency"].as<double>();
        if (param.fuse_odom_frequency <= 0) {
            throw std::runtime_error(
                "the sunray control config param 'fuse_odom_frequency' must > 0");
        }
    }
}
// 从yaml文件构造的节点中，读取msg_timeout_param字段的内容
void loadMsgTimeoutParam(const YAML::Node& node, sunray_fsm::msg_timeout_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error("the sunray control config msg_timeout_param is missing a "
                                 "valid msg_timeout_param map");
    }
    // 先判断存在，再读取值
    // -----------------------local_odometry话题的最大超时时间----------------------
    if (!node["local_odometry"]) {
        throw std::runtime_error("the sunray control config miss param 'local_odometry'");
    } else {
        param.local_odometry = node["local_odometry"].as<double>();
        if (param.local_odometry <= 0) {
            throw std::runtime_error(
                "the sunray control config param 'local_odometry' must > 0");
        }
    }
    // -----------------------mavros链路的最大超时时间----------------------
    if (!node["mavros_connect"]) {
        throw std::runtime_error("the sunray control config miss param 'mavros_connect'");
    } else {
        param.mavros_connect = node["mavros_connect"].as<double>();
        if (param.mavros_connect <= 0) {
            throw std::runtime_error(
                "the sunray control config param 'mavros_connect' must > 0");
        }
    }
    // -----------------------sunray地面站链路的最大超时时间----------------------
    if (!node["sunray_station"]) {
        throw std::runtime_error("the sunray control config miss param 'sunray_station'");
    } else {
        param.sunray_station = node["sunray_station"].as<double>();
        if (param.sunray_station <= 0) {
            throw std::runtime_error(
                "the sunray control config param 'sunray_station' must > 0");
        }
    }
}
// 从yaml文件构造的节点中，读取takeoff_land_param字段的内容
void loadTakeoffLandParam(const YAML::Node& node, sunray_fsm::takeoff_land_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error("the sunray control config takeoff_land_param is missing a "
                                 "valid takeoff_land_param map");
    }
    // 先判断存在，再读取值
    // -----------------------相对当前位置的起飞高度----------------------
    if (!node["takeoff_relative_height"]) {
        throw std::runtime_error(
            "the sunray control config miss param 'takeoff_relative_height'");
    } else {
        param.takeoff_relative_height = node["takeoff_relative_height"].as<double>();
        if (param.takeoff_relative_height <= 0) {
            throw std::runtime_error(
                "the sunray control config param 'takeoff_relative_height' must > 0");
        }
    }
    // -----------------------起飞过程中的最大速度----------------------
    if (!node["takeoff_max_velocity"]) {
        throw std::runtime_error(
            "the sunray control config miss param 'takeoff_max_velocity'");
    } else {
        param.takeoff_max_velocity = node["takeoff_max_velocity"].as<double>();
        if (param.takeoff_max_velocity <= 0) {
            throw std::runtime_error(
                "the sunray control config param 'takeoff_max_velocity' must > 0");
        }
    }
    // -----------------------指定降落模式----------------------
    if (!node["land_type"]) {
        throw std::runtime_error("the sunray control config miss param 'land_type'");
    } else {
        param.land_type = node["land_type"].as<int>();
        if (param.land_type != 0 && param.land_type != 1) {
            throw std::runtime_error("the sunray control config param 'land_type' only can 0, 1");
        }
    }
    // -----------------------降落过程中的最大速度----------------------
    if (!node["land_max_velocity"]) {
        throw std::runtime_error("the sunray control config miss param 'land_max_velocity'");
    } else {
        param.land_max_velocity = node["land_max_velocity"].as<double>();
        if (param.land_max_velocity <= 0) {
            throw std::runtime_error(
                "the sunray control config param 'land_max_velocity' must > 0");
        }
    }
    // -----------------------返航到点后是否自动降落----------------------
    if (!node["return_with_land"]) {
        throw std::runtime_error("the sunray control config miss param 'return_with_land'");
    } else {
        param.return_with_land = node["return_with_land"].as<bool>();
    }
}
// 从yaml文件构造的节点中，读取local_fence_param字段的的内容
void loadLocalFenceParam(const YAML::Node& node, sunray_fsm::local_fence_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error("the sunray control config local_fence_param is missing a "
                                 "valid local_fence_param map");
    }
    // 先判断存在，再读取值
    // -----------------------x方向----------------------
    if (!node["x_max"]) {
        throw std::runtime_error("the sunray control config miss param 'x_max'");
    } else {
        param.x_max = node["x_max"].as<double>();
    }
    if (!node["x_min"]) {
        throw std::runtime_error("the sunray control config miss param 'x_min'");
    } else {
        param.x_min = node["x_min"].as<double>();
    }
    if (param.x_max <= param.x_min) {
        throw std::runtime_error("the sunray control config param 'x_max' must > 'x_min'");
    }
    // -----------------------y方向----------------------
    if (!node["y_max"]) {
        throw std::runtime_error("the sunray control config miss param 'y_max'");
    } else {
        param.y_max = node["y_max"].as<double>();
    }
    if (!node["y_min"]) {
        throw std::runtime_error("the sunray control config miss param 'y_min'");
    } else {
        param.y_min = node["y_min"].as<double>();
    }
    if (param.y_max <= param.y_min) {
        throw std::runtime_error("the sunray control config param 'y_max' must > 'y_min'");
    }
    // -----------------------z方向----------------------
    if (!node["z_max"]) {
        throw std::runtime_error("the sunray control config miss param 'z_max'");
    } else {
        param.z_max = node["z_max"].as<double>();
    }
    if (!node["z_min"]) {
        throw std::runtime_error("the sunray control config miss param 'z_min'");
    } else {
        param.z_min = node["z_min"].as<double>();
    }
    if (param.z_max <= param.z_min) {
        throw std::runtime_error("the sunray control config param 'z_max' must > 'z_min'");
    }
}
// 从yaml文件构建的节点中，读取velocity_param字段的内容
void loadVelocityParam(const YAML::Node& node, sunray_fsm::velocity_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error(
            "the sunray control config velocity_param is missing a valid velocity_param map");
    }

    // 先判断存在，再读取值
    // 最大飞行速度字段
    YAML::Node mav_vel = node["max_velocity"];
    YAML::Node mav_vel_with_rc = node["max_velocity_with_rc"];
    // ----------------------------代码控制下的最大飞行速度---------------------
    if (!mav_vel["x_vel"]) {
        throw std::runtime_error("the sunray control config miss param 'x_vel'");
    } else {
        param.max_velocity.x() = mav_vel["x_vel"].as<double>();
        if (param.max_velocity.x() <= 0) {
            throw std::runtime_error("the sunray control config param 'x_vel' must > 0");
        }
    }
    if (!mav_vel["y_vel"]) {
        throw std::runtime_error("the sunray control config miss param 'y_vel'");
    } else {
        param.max_velocity.y() = mav_vel["y_vel"].as<double>();
        if (param.max_velocity.y() <= 0) {
            throw std::runtime_error("the sunray control config param 'y_vel' must > 0");
        }
    }
    if (!mav_vel["z_vel"]) {
        throw std::runtime_error("the sunray control config miss param 'z_vel'");
    } else {
        param.max_velocity.z() = mav_vel["z_vel"].as<double>();
        if (param.max_velocity.z() <= 0) {
            throw std::runtime_error("the sunray control config param 'z_vel' must > 0");
        }
    }
    // ----------------------------遥控器控制下的最大飞行速度---------------------
    if (!mav_vel_with_rc["x_vel"]) {
        throw std::runtime_error("the sunray control config miss param 'x_vel'");
    } else {
        param.max_velocity_with_rc.x() = mav_vel_with_rc["x_vel"].as<double>();
        if (param.max_velocity_with_rc.x() <= 0) {
            throw std::runtime_error("the sunray control config param 'x_vel' must > 0");
        }
    }
    if (!mav_vel_with_rc["y_vel"]) {
        throw std::runtime_error("the sunray control config miss param 'y_vel'");
    } else {
        param.max_velocity_with_rc.y() = mav_vel_with_rc["y_vel"].as<double>();
        if (param.max_velocity_with_rc.y() <= 0) {
            throw std::runtime_error("the sunray control config param 'y_vel' must > 0");
        }
    }
    if (!mav_vel_with_rc["z_vel"]) {
        throw std::runtime_error("the sunray control config miss param 'z_vel'");
    } else {
        param.max_velocity_with_rc.z() = mav_vel_with_rc["z_vel"].as<double>();
        if (param.max_velocity_with_rc.z() <= 0) {
            throw std::runtime_error("the sunray control config param 'z_vel' must > 0");
        }
    }
    if (!node["yaw_rate"]) {
        throw std::runtime_error("the sunray control config miss param 'yaw_rate'");
    } else {
        param.yaw_rate = deg2rad(node["yaw_rate"].as<double>());
        if (param.yaw_rate <= 0) {
            throw std::runtime_error("the sunray control config param 'yaw_rate' must > 0");
        }
    }
}
