/**
 * @file localization_fusion.h
 * @author your name (you@domain.com)
 * @brief
 * 设计意图：本模块负责对各种定位算法进行选择和检查，输出正确的全局定位话题和局部定位话题，且保证对px4无人机，无人机车，机器狗，DJI无人机通用
 * @version 0.1
 * @date 2026-03-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <ros/node_handle.h>

// #include <mutex>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "localization_fusion_types.hpp"

// 在clangd格式化的基础上做了一边手动对齐注释，提高可读性
// clang-format off 

class LocalizationFusion {
  public:
    explicit LocalizationFusion(ros::NodeHandle& nh);
    ~LocalizationFusion() = default;

    // 启动入口：读配置、校验模式、创建 pub/sub/timer
    bool Init();

    // 主循环（单线程 ros::spin）
    void Spin();

  private:
    // ---- 初始化阶段 ----
    bool load_param();  // 加载参数，校验参数

    // ---- 回调函数 ----
    /*
    回调函数的逻辑是这样定义的的
    我们为每个定位源配备了odometry话题和relocalization话题，当odometry话题得到数据时，我们认为这个话题传入的数据是sunray_local系下的
    也就是说对于odometry_callback我们会忽略里程计自带的坐标系，而提取数据强制赋予坐标系
    同时我们认为sunray_local和sunray_global在没有重定位的情况下，两者是重合的,因此对于只提供gloabl数据的动捕等定位源，直接将数据喂到odometry话题
    能够得到一样的效果

    relocalization_callback回调是为那些输入local系下的里程计，同时会有global的数据输入的定位源准备的，我们将这些分为两类，分别是aruco和lidar
    对于aruco，我们无法预知数据会在什么时候到来，但是我们知道重定位的数据一定会改变tf数据，这就导致tf必须使用动态的，
    我们将aruco的tf发送放在healthtimer_callback，这样做的好处是，得到一个连续的tf变换，同时带来一点可以接受的延时
    对于持续输入重定位的模式，比如mid360使用点云匹配先验地图，得到持续的global输入，此时我们将tf直接在relocalization_callback进行更新，因为他是持续发布的
    */
    void odometry_callback(const nav_msgs::OdometryConstPtr& msg);  // 里程计输入
    void relocalization_callback(const nav_msgs::OdometryConstPtr& msg);  // 重定位输入：local 在 global 下的位姿（global->local）
    void healthtimer_callback(const ros::TimerEvent &e);  // 周期检查 local/global 是否收到新数据,判断是否超时,更新local_odom_valid/global_odom_valid,发布 odom_state

    // ---- 发布相关----
    /*
    发布的逻辑是这样定义的
    1.
    对于local和global模式，他们的输入源只有一个单独的odometry,这个模式下sunray_local和sunray_global之间的tf关系为原点重合，姿态单位阵
    因此直接从odometry_callback中，将输入的里程计定义为sunray_local的，并通过默认的tf转换为sunray_global的
    2.
    对于aruco这样的，间断的重定位，其重定位数据从relocalization_callback重定位话题的回调得到，
    由于这样的重定位次数不定，频率不定，因此使用回调数据修改tf树，然后在定时器中发布动态的tf而不是在回调函数中发布
    3.
    对于fastlio2_relocalization这样的重定位模式，他实际上会有local的数据和global的数据，这里global的数据指的是重定位算法，在完成重定位后回输出当前的位姿在先验地图中的位姿，
    因此他是一个和local一样连续，高频的，我们直接在relocalization_callback中发布tf而非publish_global_odom_from_local或者healthtimer_callback
    因为这个global并不需要从local中推导
    4.
    由于我们使用了10Hz的定时器进行状态检查，因此将OdomStatus的发布也放置在healthtimer_callback中
    */
    void publish_global_odom_from_local(const nav_msgs::Odometry& msg);  // 当global由local推导时，在local的回调中使用这个函数发布global

    // ------TF 广播相关-----
    void broadcast_local_to_base_tf(const nav_msgs::Odometry& local_odom);  // 发布 sunray_local -> base_link

  private:
    // ---------------- ROS 相关参数 ----------------
    ros::NodeHandle nh_;
    // ROS 话题订阅者
    ros::Subscriber odometry_sub_;        // 里程计话题 订阅者
    ros::Subscriber relocalization_sub_;  // 重定位话题 订阅者
    // ROS话题发布者
    ros::Publisher local_odom_pub_;       // 局部定位话题 local_odom发布者
    ros::Publisher global_odom_pub_;      // 全局定位话题 global_odom 发布者
    ros::Publisher odom_state_pub_;       // 定位状态话题 odom_state发布者
    // ROS定时器
    ros::Timer health_timer_;  // 健康检查定时器：周期检查 local/global 是否超时，并发布 odom_state
    // TF 广播器
    tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // ---------------- Config 相关参数 ----------------
    std::string uav_ns_;                // 从全局参数中读取 uav_id+ uav_name 组成uav_namespace
    std::string config_yamlfile_path_;  // yaml 参数文件的路径
    int selected_source_id_{-1};        // launch文件中所指定的定位源编号
    SourceConfig selected_source_{};  // 结合launch文件中的source_id,从config文件中构建读取参数并缓存
    bool has_selected_source_{false};  // 状态标识符，表示是否读取到了配置参数
    double health_rate_hz_{10};

    // 输出topic约定
    std::string global_odometry_topic_{"${uav_ns}/sunray/localization/global_odom"};
    std::string local_odometry_topic_{"${uav_ns}/sunray/localization/local_odom"};
    std::string odom_status_topic_{"${uav_ns}/sunray/localization/odom_status"};

    // 输出 frame 约定：sunray_global -> sunray_local -> base_link
    std::string global_frame_id_{"sunray_global"};
    std::string local_frame_id_{"sunray_local"};
    std::string base_frame_id_{"base_link"};

    // 线程锁,在实现过程中注意到这个线程锁没有使用，先注释掉，如果后续测试过程中发现需要加入提高性能，再接入
    // mutable std::mutex mutex_;

    nav_msgs::Odometry last_odometry_data_;        // 最新的odometry数据
    nav_msgs::Odometry last_relocalization_data_;  // 最新的重定位数据

    bool has_odometry_data_{false};        // odometry回调函数是否接受到数据
    bool has_relocalization_data_{false};  // relocalization回调函数是否接受到数据

    bool relocalization_data_valid_{false};

    ros::Time last_odometry_rx_time_{ros::Time(0)};  // 最新的odometry数据接收时间

    bool odometry_data_timeout_{false};  // local系数据是否超时

    geometry_msgs::TransformStamped global_to_local_tf_;  // sunray_global -> sunray_local
    geometry_msgs::TransformStamped local_to_base_tf_;    // sunray_local -> base_link
};

// clang-format on