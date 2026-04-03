#include "statemachine/sunray_fsm_loadparam.hpp"
#include <stdexcept>
#include <ros/ros.h>

// 从yaml文件构造的节点中，读取basic_param字段内容
void loadBasicParam(const YAML::Node& node, sunray_fsm::basic_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error(
            "the sunray_control_config.yaml basic_param is missing a valid basic_param map");
    }
    // 先判断存在，再读取值，再判断值是否允许
    // -----------------------无人机自重(带电池)----------------------
    if (!node["mass_kg"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'mass_kg'");
    } else {
        param.mass_kg = node["mass_kg"].as<double>();
        if (param.mass_kg <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml param 'mass_kg' must > 0");
        }
    }
    // -----------------------当地重力加速度(单z轴标量)----------------------
    if (!node["gravity"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'gravity'");
    } else {
        param.gravity = node["gravity"].as<double>();
        if (param.gravity <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml param 'gravity' must > 0");
        }
    }
    // -----------------------控制器类型----------------------
    if (!node["controller_types"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'controller_types'");
    } else {
        param.controller_types = node["controller_types"].as<int>();
        if (param.controller_types != 0 && param.controller_types != 1 &&
            param.controller_types != 2) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'controller_types' only can 0, 1，2");
        }
    }
    // -----------------------控制器更新频率----------------------
    if (!node["controller_update_frequency"]) {
        throw std::runtime_error(
            "the sunray_control_config.yaml miss param 'controller_update_frequency'");
    } else {
        param.controller_update_frequency = node["controller_update_frequency"].as<double>();
        if (param.controller_update_frequency <= 10) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'controller_update_frequency' must > 10");
        }
    }
    // -----------------------状态机检查频率----------------------
    if (!node["supervisor_update_frequency"]) {
        throw std::runtime_error(
            "the sunray_control_config.yaml miss param 'supervisor_update_frequency'");
    } else {
        param.supervisor_update_frequency = node["supervisor_update_frequency"].as<double>();
        if (param.supervisor_update_frequency <= 0) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'supervisor_update_frequency' must > 0");
        }
    }
    // -----------------------里程计话题----------------------
    if (!node["odom_topic_name"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'odom_topic_name'");
    } else {
        param.odom_topic_name = node["odom_topic_name"].as<std::string>();
        if (param.odom_topic_name.empty()) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'odom_topic_name' is empty");
        }
    }
    // -----------------------px4融合类型----------------------
    if (!node["fuse_odom_type"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'fuse_odom_type'");
    } else {
        param.fuse_odom_type = node["fuse_odom_type"].as<int>();
        if (param.fuse_odom_type != 0 && param.fuse_odom_type != 1 && param.fuse_odom_type != 2) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'fuse_odom_type' only can 0, 1, 2");
        }
    }
    // ---------------------px4融合频率--------------------
    if (!node["fuse_odom_frequency"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'fuse_odom_frequency'");
    } else {
        param.fuse_odom_frequency = node["fuse_odom_frequency"].as<double>();
        if (param.fuse_odom_frequency <= 0) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'fuse_odom_frequency' must > 0");
        }
    }
    // -----------------------控制器轨迹类型----------------------
    if (!node["trajectory_type"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'trajectory_type'");
    } else {
        param.trajectory_type = node["trajectory_type"].as<int>();
        if (param.trajectory_type != 0 && param.trajectory_type != 1) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'trajectory_type' only can 0, 1");
        }
    }
}
// 从yaml文件构造的节点中，读取protect_param字段的内容
void loadProtectParam(const YAML::Node& node, sunray_fsm::protect_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error(
            "the sunray_control_config.yaml protect_param is missing a valid protect_param map");
    }
    // 先判断存在，再读取值，再判断值是否允许
    // -----------------------飞行任务中的最低电压----------------------
    if (!node["low_voltage"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'low_voltage'");
    } else {
        param.low_voltage = node["low_voltage"].as<double>();
        if (param.low_voltage <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml param 'low_voltage' must > 0");
        }
    }
    // -----------------------达到最低电压时执行的操作----------------------
    if (!node["low_voltage_operate"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'low_voltage_operate'");
    } else {
        param.low_voltage_operate = node["low_voltage_operate"].as<int>();
        if (param.low_voltage_operate != 0 && param.low_voltage_operate != 1 &&
            param.low_voltage_operate != 2) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param low_voltage_operate only can 0, 1, 2");
        }
    }
    // -----------------------是否允许在无遥控器连接时启动无人机----------------------
    if (!node["control_with_no_rc"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'control_with_no_rc'");
    } else {
        param.control_with_no_rc = node["control_with_no_rc"].as<bool>();
    }
    // -----------------------当遥控器状态从连接变成丢失连接时，执行的操作----------------------
    if (!node["lost_with_rc"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'lost_with_rc'");
    } else {
        param.lost_with_rc = node["lost_with_rc"].as<int>();
        if (param.lost_with_rc != 0 && param.lost_with_rc != 1 && param.lost_with_rc != 2) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param lost_with_rc only can 0, 1, 2");
        }
    }
    // -----------------------是否允许代码解锁无人机----------------------
    if (!node["arm_with_code"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'arm_with_code'");
    } else {
        param.arm_with_code = node["arm_with_code"].as<bool>();
    }
    // -----------------------是否允许代码起飞无人机----------------------
    if (!node["takeoff_with_code"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'takeoff_with_code'");
    } else {
        param.takeoff_with_code = node["takeoff_with_code"].as<bool>();
    }
    // -----------------------是否进行倾倒检测----------------------
    if (!node["check_flip"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'check_flip'");
    } else {
        param.check_flip = node["check_flip"].as<bool>();
    }
    // -----------------------飞行任务中的最低电压----------------------
    if (!node["tilt_angle_max"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'tilt_angle_max'");
    } else {
        param.tilt_angle_max = node["tilt_angle_max"].as<double>();
        if (param.tilt_angle_max <= 0) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'tilt_angle_max' must > 0");
        }
    }
    // -----------------------当msg超时执行的操作---------------------
    if (!node["msg_timeout_operate"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'msg_timeout_operate'");
    } else {
        param.msg_timeout_operate = node["msg_timeout_operate"].as<int>();
        if (param.msg_timeout_operate != 0 && param.msg_timeout_operate != 1 &&
            param.msg_timeout_operate != 2) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param msg_timeout_operate only can 0, 1, 2");
        }
    }
}
// 从yaml文件构造的节点中，读取msg_timeout_param字段的内容
void loadMsgTimeoutParam(const YAML::Node& node, sunray_fsm::msg_timeout_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error("the sunray_control_config.yaml msg_timeout_param is missing a "
                                 "valid msg_timeout_param map");
    }
    // 先判断存在，再读取值
    // -----------------------local_odometry话题的最大超时时间----------------------
    if (!node["local_odometry"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'local_odometry'");
    } else {
        param.local_odometry = node["local_odometry"].as<double>();
        if (param.local_odometry <= 0) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'local_odometry' must > 0");
        }
    }
    // -----------------------mavros链路的最大超时时间----------------------
    if (!node["mavros_connect"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'mavros_connect'");
    } else {
        param.mavros_connect = node["mavros_connect"].as<double>();
        if (param.mavros_connect <= 0) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'mavros_connect' must > 0");
        }
    }
    // -----------------------sunray地面站链路的最大超时时间----------------------
    if (!node["sunray_station"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'sunray_station'");
    } else {
        param.sunray_station = node["sunray_station"].as<double>();
        if (param.sunray_station <= 0) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'sunray_station' must > 0");
        }
    }
}
// 从yaml文件构造的节点中，读取takeoff_land_param字段的内容
void loadTakeoffLandParam(const YAML::Node& node, sunray_fsm::takeoff_land_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error("the sunray_control_config.yaml takeoff_land_param is missing a "
                                 "valid takeoff_land_param map");
    }
    // 先判断存在，再读取值
    // -----------------------相对当前位置的起飞高度----------------------
    if (!node["takeoff_relative_height"]) {
        throw std::runtime_error(
            "the sunray_control_config.yaml miss param 'takeoff_relative_height'");
    } else {
        param.takeoff_relative_height = node["takeoff_relative_height"].as<double>();
        if (param.takeoff_relative_height <= 0) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'takeoff_relative_height' must > 0");
        }
    }
    // -----------------------起飞过程中的最大速度----------------------
    if (!node["takeoff_max_velocity"]) {
        throw std::runtime_error(
            "the sunray_control_config.yaml miss param 'takeoff_max_velocity'");
    } else {
        param.takeoff_max_velocity = node["takeoff_max_velocity"].as<double>();
        if (param.takeoff_max_velocity <= 0) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'takeoff_max_velocity' must > 0");
        }
    }
    // -----------------------指定降落模式----------------------
    if (!node["land_type"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'low_voltage_operate'");
    } else {
        param.land_type = node["land_type"].as<int>();
        if (param.land_type != 0 && param.land_type != 1) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param low_voltage_operate only can 0, 1");
        }
    }
    // -----------------------降落过程中的最大速度----------------------
    if (!node["land_max_velocity"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'land_max_velocity'");
    } else {
        param.land_max_velocity = node["land_max_velocity"].as<double>();
        if (param.land_max_velocity <= 0) {
            throw std::runtime_error(
                "the sunray_control_config.yaml param 'land_max_velocity' must > 0");
        }
    }
}
// 从yaml文件构造的节点中，读取mission_error_param字段的内容
void loadMissionErrorParam(const YAML::Node& node, sunray_fsm::mission_error_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error("the sunray_control_config.yaml mission_error_param is missing a "
                                 "valid mission_error_param map");
    }
    YAML::Node takeoff_error_node = node["takeoff"];
    YAML::Node move_error_node = node["move_point"];
    // 先判断存在，再读取值
    // -----------------------------------------起飞任务相关-------------------------------
    if (!takeoff_error_node["timeout_s"]) {
        throw std::runtime_error(
            "the sunray_control_config.yaml miss param 'mission_error_param : takeoff: timeout_s'");
    } else {
        param.takeoff_error_param.timeout_s = takeoff_error_node["timeout_s"].as<double>();
        if (param.takeoff_error_param.timeout_s <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml  param 'mission_error_param : "
                                     "takeoff: timeout_s' must > 0");
        }
    }
    if (!takeoff_error_node["judge_stabile_time_s"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'mission_error_param : "
                                 "takeoff: judge_stabile_time_s'");
    } else {
        param.takeoff_error_param.judge_stabile_time_s =
            takeoff_error_node["judge_stabile_time_s"].as<double>();
        if (param.takeoff_error_param.judge_stabile_time_s <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml  param 'mission_error_param : "
                                     "takeoff: judge_stabile_time_s' must > 0");
        }
    }
    if (!takeoff_error_node["pos_stabile_err_m"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'mission_error_param : "
                                 "takeoff: pos_stabile_err_m'");
    } else {
        param.takeoff_error_param.pos_stabile_err_m =
            takeoff_error_node["pos_stabile_err_m"].as<double>();
        if (param.takeoff_error_param.pos_stabile_err_m <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml  param 'mission_error_param : "
                                     "takeoff: pos_stabile_err_m' must > 0");
        }
    }
    if (!takeoff_error_node["vel_stabile_err_mps"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'mission_error_param : "
                                 "takeoff: vel_stabile_err_mps'");
    } else {
        param.takeoff_error_param.vel_stabile_err_mps =
            takeoff_error_node["vel_stabile_err_mps"].as<double>();
        if (param.takeoff_error_param.vel_stabile_err_mps <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml  param 'mission_error_param : "
                                     "takeoff: vel_stabile_err_mps' must > 0");
        }
    }
    // -----------------------------------------move_point相关-------------------------------
    if (!move_error_node["timeout_s"]) {
        throw std::runtime_error(
            "the sunray_control_config.yaml miss param 'mission_error_param : takeoff: timeout_s'");
    } else {
        param.move_point_error_param.timeout_s = move_error_node["timeout_s"].as<double>();
        if (param.move_point_error_param.timeout_s <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml  param 'mission_error_param : "
                                     "takeoff: timeout_s' must > 0");
        }
    }
    if (!move_error_node["judge_stabile_time_s"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'mission_error_param : "
                                 "takeoff: judge_stabile_time_s'");
    } else {
        param.move_point_error_param.judge_stabile_time_s =
            move_error_node["judge_stabile_time_s"].as<double>();
        if (param.move_point_error_param.judge_stabile_time_s <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml  param 'mission_error_param : "
                                     "takeoff: judge_stabile_time_s' must > 0");
        }
    }
    if (!move_error_node["pos_stabile_err_m"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'mission_error_param : "
                                 "takeoff: pos_stabile_err_m'");
    } else {
        param.move_point_error_param.pos_stabile_err_m =
            move_error_node["pos_stabile_err_m"].as<double>();
        if (param.move_point_error_param.pos_stabile_err_m <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml  param 'mission_error_param : "
                                     "takeoff: pos_stabile_err_m' must > 0");
        }
    }
    if (!move_error_node["vel_stabile_err_mps"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'mission_error_param : "
                                 "takeoff: vel_stabile_err_mps'");
    } else {
        param.move_point_error_param.vel_stabile_err_mps =
            move_error_node["vel_stabile_err_mps"].as<double>();
        if (param.move_point_error_param.vel_stabile_err_mps <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml  param 'mission_error_param : "
                                     "takeoff: vel_stabile_err_mps' must > 0");
        }
    }
}
// 从yaml文件构造的节点中，读取local_fence_param字段的的内容
void loadLocalFenceParam(const YAML::Node& node, sunray_fsm::local_fence_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error("the sunray_control_config.yaml local_fence_param is missing a "
                                 "valid local_fence_param map");
    }
    // 先判断存在，再读取值
    // -----------------------x方向----------------------
    if (!node["x_max"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'x_max'");
    } else {
        param.x_max = node["x_max"].as<double>();
    }
    if (!node["x_min"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'x_min'");
    } else {
        param.x_min = node["x_min"].as<double>();
    }
    if (param.x_max <= param.x_min) {
        throw std::runtime_error("the sunray_control_config.yaml param 'x_max' must > 'x_min'");
    }
    // -----------------------y方向----------------------
    if (!node["y_max"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'y_max'");
    } else {
        param.y_max = node["y_max"].as<double>();
    }
    if (!node["y_min"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'y_min'");
    } else {
        param.y_min = node["y_min"].as<double>();
    }
    if (param.y_max <= param.y_min) {
        throw std::runtime_error("the sunray_control_config.yaml param 'y_max' must > 'y_min'");
    }
    // -----------------------z方向----------------------
    if (!node["z_max"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'z_max'");
    } else {
        param.z_max = node["z_max"].as<double>();
    }
    if (!node["z_min"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'z_min'");
    } else {
        param.z_min = node["z_min"].as<double>();
    }
    if (param.z_max <= param.z_min) {
        throw std::runtime_error("the sunray_control_config.yaml param 'z_max' must > 'z_min'");
    }
}
// 从yaml文件构建的节点中，读取velocity_param字段的内容
void loadVelocityParam(const YAML::Node& node, sunray_fsm::velocity_param_t& param) {
    // 首先，如果传入的node为空，或者不是键值对的形式，则抛出异常
    if (!node || !node.IsMap()) {
        throw std::runtime_error(
            "the sunray_control_config.yaml velocity_param is missing a valid velocity_param map");
    }

    // 先判断存在，再读取值
    // 最大飞行速度字段
    YAML::Node mav_vel = node["max_velocity"];
    YAML::Node mav_vel_with_rc = node["max_velocity_with_rc"];
    // ----------------------------代码控制下的最大飞行速度---------------------
    if (!mav_vel["x_vel"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'x_vel'");
    } else {
        param.max_velocity.x() = mav_vel["x_vel"].as<double>();
        if (param.max_velocity.x() <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml param 'x_vel' must > 0");
        }
    }
    if (!mav_vel["y_vel"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'y_vel'");
    } else {
        param.max_velocity.y() = mav_vel["y_vel"].as<double>();
        if (param.max_velocity.y() <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml param 'y_vel' must > 0");
        }
    }
    if (!mav_vel["z_vel"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'z_vel'");
    } else {
        param.max_velocity.z() = mav_vel["z_vel"].as<double>();
        if (param.max_velocity.z() <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml param 'z_vel' must > 0");
        }
    }
    // ----------------------------遥控器控制下的最大飞行速度---------------------
    if (!mav_vel_with_rc["x_vel"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'x_vel'");
    } else {
        param.max_velocity_with_rc.x() = mav_vel_with_rc["x_vel"].as<double>();
        if (param.max_velocity_with_rc.x() <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml param 'x_vel' must > 0");
        }
    }
    if (!mav_vel_with_rc["y_vel"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'y_vel'");
    } else {
        param.max_velocity_with_rc.y() = mav_vel_with_rc["y_vel"].as<double>();
        if (param.max_velocity_with_rc.y() <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml param 'y_vel' must > 0");
        }
    }
    if (!mav_vel_with_rc["z_vel"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'z_vel'");
    } else {
        param.max_velocity_with_rc.z() = mav_vel_with_rc["z_vel"].as<double>();
        if (param.max_velocity_with_rc.z() <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml param 'z_vel' must > 0");
        }
    }
    if (!node["yaw_rate"]) {
        throw std::runtime_error("the sunray_control_config.yaml miss param 'yaw_rate'");
    } else {
        param.yaw_rate = node["yaw_rate"].as<double>();
        if (param.yaw_rate <= 0) {
            throw std::runtime_error("the sunray_control_config.yaml param 'yaw_rate' must > 0");
        }
    }
}
