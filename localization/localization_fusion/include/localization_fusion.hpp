/**
 * @file localization_fusion.hpp
 * @brief 定位融合节点类声明。
 */

#pragma once

#include <deque>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sunray_msgs/OdomState.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <stdexcept>

#include <Eigen/Dense>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "agent_key_helper.hpp"
#include "localization_fusion_utils.hpp"

class LocalizationFusion {
  public:
    explicit LocalizationFusion(ros::NodeHandle& nh);
    ~LocalizationFusion() = default;

    bool Init();

  private:
    // 加载参数
    bool load_param();
    // 里程计回调函数，该回调函数会屏蔽坐标系，并在发布时显式的设置为surnay_local
    // 2. 我们直接在回调函数中发布surany_local的里程计，从而保证输入和输出同一频率，不会收到延迟之类的
    void odometry_callback(const nav_msgs::OdometryConstPtr& msg);
    // 重定位回调函数，本质上计算一个sunray_local与surnay_global的变换
    void relocalization_callback(const nav_msgs::OdometryConstPtr& msg);
    // 定时器回调函数，用于发布global_odom,odom_state以及tf
    void healthtimer_callback(const ros::TimerEvent& e);
    // 将输入的里程计，经过config中配置的外参矩阵，变换为sunray_local系发布的里程计
    nav_msgs::Odometry transform_source_odom_to_local(const nav_msgs::Odometry& msg) const;
    // 从local系里程计计算global系里程计
    bool build_global_odom_from_local(const nav_msgs::Odometry& local_msg, nav_msgs::Odometry& global_msg) const;
    // 从里程计中更新local到base_link的tf
    void update_local_to_base_tf_from_odom(const nav_msgs::Odometry& local_odom);

  private:
    // ros句柄  
    ros::NodeHandle nh_;
    // ros订阅者 [里程计]+[重定位]
    ros::Subscriber odometry_sub_;
    ros::Subscriber relocalization_sub_;
    // ros发布者 [local系]+[global系]+[模块状态]
    ros::Publisher local_odom_pub_;
    ros::Publisher global_odom_pub_;
    ros::Publisher odom_state_pub_;
    // ros定时器 [global+state+tf发布]
    ros::Timer health_timer_;
    // tf变换器 [静态TF] + [连续变换TF]
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 节点名字
    std::string node_name;
    // 智能体标识符
    std::string agent_key_;
    // yaml格式配置文件路径
    std::string config_yamlfile_path_;
    // 选择的外部定位源索引序号
    int selected_source_id_{-1};
    // 定位源配置
    SourceConfig selected_source_{};

    // 对外发布的定位融合状态快照
    sunray_msgs::OdomState odom_state;
    
    std::string global_odometry_topic_{"${agent_key}/sunray/localization/global_odom"};
    std::string local_odometry_topic_{"${agent_key}/sunray/localization/local_odom"};
    std::string odom_state_topic_{"${agent_key}/sunray/localization/odom_state"};

    ros::Time odometry_last_receive_time_;

    std::deque<double> hz_stamps_;
};
