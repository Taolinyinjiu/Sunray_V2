/**
 * @file localization_fusion.hpp
 * @brief
 * 设计意图：
 * 本模块负责统一不同定位源的输入语义，输出标准化的 local/global 里程计与 TF。
 *
 * 最新约定（2026-05-14）：
 * 1. odometry_topic 永远表示 local 主里程计输入
 * 2. relocalization_topic 表示 base_link 在 sunray_global 下的位姿输入，可为空
 * 3. local_odom 在 odometry_callback 中完成外参变换后立即发布，保持与输入同频
 * 4. global_odom / TF / OdomState 统一由 health_timer_ 周期发布
 * 5. localization_fusion 不再关心 world 系
 * 6. TF 树固定为 sunray_global -> {agent}/sunray_local -> {agent}/base_link
 */

#pragma once

#include <deque>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "localization_fusion_types.hpp"

class LocalizationFusion {
  public:
    explicit LocalizationFusion(ros::NodeHandle& nh);
    ~LocalizationFusion() = default;

    bool Init();
    void Spin();

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
    bool build_global_odom_from_local(const nav_msgs::Odometry& local_msg,
                                      nav_msgs::Odometry& global_msg) const;
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
    
    // 智能体标识符
    std::string agent_key_;
    // yaml格式配置文件路径
    std::string config_yamlfile_path_;
    // 选择的外部定位源索引序号
    int selected_source_id_{-1};
    // 定位源配置
    SourceConfig selected_source_{};
    
    bool has_selected_source_{false};
    double health_rate_hz_{10.0};
    bool use_receive_time_{false};
    bool tf_world_global_{false};
    bool tf_local_world_{false};

    std::string global_odometry_topic_{"${agent_key}/sunray/localization/global_odom"};
    std::string local_odometry_topic_{"${agent_key}/sunray/localization/local_odom"};
    std::string odom_state_topic_{"${agent_key}/sunray/localization/odom_state"};

    std::string world_frame_id_{"world"};
    std::string global_frame_id_{"sunray_global"};
    std::string local_frame_id_{"sunray_local"};
    std::string base_frame_id_{"base_link"};

    nav_msgs::Odometry latest_local_odom_;
    nav_msgs::Odometry latest_relocalization_odom_;
    nav_msgs::Odometry last_published_local_odom_;
    nav_msgs::Odometry last_published_global_odom_;

    ros::Time odometry_received_stamp_;
    ros::Time relocalization_received_stamp_;
    ros::Time odometry_last_receive_time_;
    ros::Time relocalization_last_receive_time_;

    bool has_local_odom_{false};
    bool has_relocalization_odom_{false};
    bool has_relocalization_input_{false};
    bool odometry_valid_{false};
    bool relocalization_valid_{false};
    bool global_local_tf_valid_{false};

    double odometry_update_hz{0.0};
    std::deque<double> hz_stamps_;

    geometry_msgs::TransformStamped global_to_local_tf_;
    geometry_msgs::TransformStamped local_to_base_tf_;
};
