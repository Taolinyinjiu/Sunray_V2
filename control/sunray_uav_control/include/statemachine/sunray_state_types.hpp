/**? * @file sunray_state_types.hpp
 * @brief
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-27
 * @version 0.1
 *
 */

#pragma once

#include <cstdint>
#include <string>
#include <Eigen/Dense>

namespace sunray_fsm {

// sunray_fsm的状态集合
enum class SunrayState {
    OFF = 0,  ///< 待机/未激活状态。
    INIT,     ///< 各组件初始化成功，允许在该状态使用遥控器进行控制
    TAKEOFF,  ///< 起飞过程状态。
    HOVER,    ///< 悬停状态（主稳态）。
    RETURN,   ///< 返航状态
    LAND,     ///< 降落过程状态。
    MOVE      ///< 运动状态
};
enum class SunrayEvent {
    TAKEOFF_REQUEST = 0,    ///< 请求起飞。
    TAKEOFF_COMPLETED,      ///< 起飞完成。
    LAND_REQUEST,           ///< 请求降落。
    LAND_COMPLETED,         ///< 降落完成。
    RETURN_REQUEST,         ///< 请求返航。
    RETURN_COMPLETED,       ///< 返航完成。
    KILL_REQUEST,           ///< 请求返航。
    KILL_COMPLETED,         ///< 返航完成。
    HOVER_REQUEST,          ///< 请求返航。
    HOVER_COMPLETED,        ///< 返航完成。
    POINT_REQUEST,          ///< 进入位置控制
    POINT_COMPLETED,        ///< 位置控制完成
    VELOCITY_REQUEST,       ///< 进入速度控制
    VELOCITY_COMPLETED,     ///< 速度控制完成
    TRAJECTORY_REQUEST,     ///< 进入轨迹控制
    TRAJECTORY_COMPLETED,   ///< 轨迹执行完成。
    POINT_WGS84_REQUEST,    ///< 进入 WGS98位置控制
    POINT_WGS84_COMPLETED,  ///< WGS98位置控制完成
};

}  // namespace sunray_fsm
