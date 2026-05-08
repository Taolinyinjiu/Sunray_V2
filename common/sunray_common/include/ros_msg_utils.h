/**
 * @file ros_msg_utils.h
 * @author YunDrone 
 * @brief 本文件用于简化uav_example中的头文件引入工作，只需要引入ros_msg_utils.h即可完成对sunray系列无人机的控制
 * @version 0.1
 * @date 2026-04-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

// common
#include <ros/ros.h>
#include <iostream>
#include <bitset>
#include <signal.h>
#include <math.h>
#include <map>
#include <signal.h>
#include <string>
#include <stdint.h>
#include "WGS84.h"
#include "string_uav_namespace_utils.hpp"

// Eigen
#include <Eigen/Eigen>

// ROS话题消息头文件

// sunray_msgs
#include <sunray_msgs/Formation.h>
#include <sunray_msgs/OdomStatus.h>
#include <sunray_msgs/OrcaCmd.h>
#include <sunray_msgs/OrcaSetup.h>
#include <sunray_msgs/PlanningWaypoint.h>
#include <sunray_msgs/Px4State.h>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVControlFSMState.h>
#include <sunray_msgs/UAVControllerState.h>
#include <sunray_msgs/UAVPlanningCMD.h>
#include <sunray_msgs/UAVPlanningState.h>
#include <sunray_msgs/UAVSwarmCMD.h>
#include <sunray_msgs/UAVSwarmState.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVControlFSMState.h>
#include <sunray_msgs/UGVSwarmCMD.h>
#include <sunray_msgs/UGVSwarmState.h>
#include <sunray_msgs/Vector2.h>

// std_msgs
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>

// sensor_msgs
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

// geometry_msgs
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

// mavros
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>

// nav_msgs
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// others
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>  //发布动态坐标关系
#include <tf2_ros/transform_listener.h>
#include <gazebo_msgs/ModelState.h>
