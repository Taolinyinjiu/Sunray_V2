#include "localization_fusion.hpp"
#include "string_uav_namespace_utils.hpp"
#include <stdexcept>
#include <sunray_msgs/OdomStatus.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

// 由于我们在launch文件中将传入的参数设置为了节点私有参数，因此需要在这里构造私有句柄
LocalizationFusion::LocalizationFusion(ros::NodeHandle &nh) {
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
    throw std::runtime_error("missing param" + node_name +
                             "/config_yamlfile_path");
  }
  if (!private_nh_.getParam("health_rate_hz", health_rate_hz_)) {
    // 读取失败，抛出异常
    throw std::runtime_error("missing param" + node_name + "/health_rate_hz");
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

bool LocalizationFusion::load_param() {
  // 读取参数
  selected_source_ = load_config_from_yaml(config_yamlfile_path_,
                                           selected_source_id_, uav_ns_);
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
  selected_source_.odometry_topic = sunray_common::replace_uav_ns(selected_source_.odometry_topic, uav_ns_);
	  // 检查，输入里程计话题不能为空
  if (selected_source_.odometry_topic.empty() ) {
    throw std::runtime_error("selected source config the odometry topic missing value");
  }
  // 如果模式要求有重定位的输入，就转换一下
	if(selected_source_.localization_mode != LocalizationMode::LOCAL && selected_source_.localization_mode != LocalizationMode::GLOBAL  ){
		selected_source_.relocalization_topic = sunray_common::replace_uav_ns(selected_source_.relocalization_topic, uav_ns_);
		if(selected_source_.relocalization_topic.empty())
		{
			throw std::runtime_error("selected source config the relocalizaiton topic missing value");
		}
	}

  // 输出话题
  global_odometry_topic_ =
      sunray_common::replace_uav_ns(global_odometry_topic_, uav_ns_);
  local_odometry_topic_ =
      sunray_common::replace_uav_ns(local_odometry_topic_, uav_ns_);
  odom_status_topic_ =
      sunray_common::replace_uav_ns(odom_status_topic_, uav_ns_);
  // 检查，输出话题不能为空
  if (global_odometry_topic_.empty() || local_odometry_topic_.empty() ||
      odom_status_topic_.empty()) {
    throw std::runtime_error("localization fusion config has empty topic");
  }

  // 注册订阅者
  odometry_sub_ = nh_.subscribe(selected_source_.odometry_topic, 50,
                                &LocalizationFusion::odometry_callback, this);
	if(selected_source_.localization_mode != LocalizationMode::LOCAL && selected_source_.localization_mode != LocalizationMode::GLOBAL  )
  {relocalization_sub_ =
      nh_.subscribe(selected_source_.relocalization_topic, 50,
                    &LocalizationFusion::relocalization_callback, this);
	}

  // 注册发布者
  local_odom_pub_ =
      nh_.advertise<nav_msgs::Odometry>(local_odometry_topic_, 10);
  global_odom_pub_ =
      nh_.advertise<nav_msgs::Odometry>(global_odometry_topic_, 10);
  odom_state_pub_ =
      nh_.advertise<sunray_msgs::OdomStatus>(odom_status_topic_, 10);

  // 注册定时器
  // 考虑到在构造函数读取参数时没有进行检查，这里构造之前检查一下，要求频率至少在1Hz
  health_rate_hz_ = std::max(1.0, health_rate_hz_);
  health_timer_ =
      nh_.createTimer(ros::Duration(1.0 / health_rate_hz_),
                      &LocalizationFusion::healthtimer_callback, this);

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

void LocalizationFusion::odometry_callback(
    const nav_msgs::OdometryConstPtr &msg) {
  // 将里程计转换为sunray_local系下的数据进行发送
  nav_msgs::Odometry temp_msg = *msg; // 首先解引用，拿到里程计的值
	last_odometry_data_ = temp_msg;
	has_odometry_data_ = true;
  // 改frame
  temp_msg.header.frame_id = local_frame_id_;
  temp_msg.child_frame_id = base_frame_id_;
  // 发布
  local_odom_pub_.publish(temp_msg);
  if (selected_source_.localization_mode != LocalizationMode::LOCAL_AND_GLOBAL) {
    publish_global_odom_from_local(temp_msg); // 根据tf构造输出
  }
  // 发布完了记录一下时间戳
  last_odometry_rx_time_ = temp_msg.header.stamp;
  last_odometry_data_ = temp_msg;
  // 同步发送TF
  broadcast_local_to_base_tf(temp_msg);
}

void LocalizationFusion::publish_global_odom_from_local(
    const nav_msgs::Odometry &msg) {
  nav_msgs::Odometry global_msg = msg;
  global_msg.header.frame_id = global_frame_id_;
  global_msg.child_frame_id = base_frame_id_;
  global_msg.header.stamp = msg.header.stamp;
  if (selected_source_.localization_mode !=
      LocalizationMode::LOCAL_WITH_ARUCO) {
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

void LocalizationFusion::relocalization_callback(const nav_msgs::OdometryConstPtr &msg){
	// 输出global系下的里程计，并根据差值重构tf
	nav_msgs::Odometry global_msg = *msg;
	// 首先要进行输入保护，由于tf是严格的计算过程，因此我们要先保证输入的数据是正常的
	// 1. 有界性：xyz不能是无限值
	// 2. 单位性：odometry使用四元数表示姿态，四元数需要是单位四元数
	// 提取位置与姿态
	Eigen::Vector3d position(global_msg.pose.pose.position.x,global_msg.pose.pose.position.y,global_msg.pose.pose.position.z);
	Eigen::Quaterniond quad(global_msg.pose.pose.orientation.w,global_msg.pose.pose.orientation.x,global_msg.pose.pose.orientation.y,global_msg.pose.pose.orientation.z);

	bool odometry_finite = std::isfinite(position.x()) 
													&& std::isfinite(position.y())
													&& std::isfinite(position.z()) ;
	if(!odometry_finite){
		relocalization_data_valid_ = false;
		return;
	}
	bool odometry_quad_safe = std::isfinite(quad.x()) 
													&& std::isfinite(quad.y()) 
													&& std::isfinite(quad.z()) 
													&& std::isfinite(quad.w()) 
													&& std::abs(quad.x()*quad.x()
														+ quad.y()*quad.y() + quad.z()*quad.z() + quad.w()*quad.w() - 1.0) < 1e-2;
	if(!odometry_quad_safe)
	{
		relocalization_data_valid_ = false;
		return;
	}
	// 本来我是想处理完如果输入有问题抛出异常的，但是考虑到飞行过程总如果出现问题抛出异常，会导致local数据中断发送，因此这里我选择的是置标志位，然后return
	relocalization_data_valid_ = true;
	last_relocalization_data_ = *msg;
	has_relocalization_data_ = true;
	if(selected_source_.localization_mode == LocalizationMode::LOCAL_AND_GLOBAL){
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
  const tf2::Transform T_global_local =
      T_global_base * T_local_base.inverse();

  global_to_local_tf_.header.stamp = global_msg.header.stamp;
  global_to_local_tf_.header.frame_id = global_frame_id_;
  global_to_local_tf_.child_frame_id = local_frame_id_;
  global_to_local_tf_.transform = tf2::toMsg(T_global_local);

  last_relocalization_data_ = global_msg;
  has_relocalization_data_ = true;

  // 动态广播 global -> local
	if(selected_source_.localization_mode == LocalizationMode::LOCAL_AND_GLOBAL){
		// 只有在lidar的高频里程计时，才选择在这里发布tf，因为aruco的频率不定，转移到healthtimer_callback更好
		tf_broadcaster_.sendTransform(global_to_local_tf_);
	}
}

void LocalizationFusion::healthtimer_callback(const ros::TimerEvent &e){
	// 本函数主要负责
	// 1. 检查Localization_Fusion各变量状态
	// 2. 当使用aruco进行重定位时，在这里持续发送动态tf树
	// 3，检查通信链路是否超时，只检查odometry,不检查relocalization
	// 4. 填充sunray_msgs::OdomStatus消息并发布

	if(selected_source_.localization_mode == LocalizationMode::LOCAL_WITH_ARUCO)
	{
		// 默认值为原点重合
		global_to_local_tf_.header.stamp = ros::Time::now();
		tf_broadcaster_.sendTransform(global_to_local_tf_);
	}
	// 对odometry通信链路的检查需要先接受到数据
	if(has_odometry_data_){
		// 在拥有数据的基础上，用当前时间和最新接收时间做差，如果大于配置结构体中的默认参数，则判断为超时
		if((ros::Time::now() - last_odometry_rx_time_).toSec() > selected_source_.timeout_s ){
			odometry_data_timeout_ = true;
		}else{
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
}

void LocalizationFusion::broadcast_local_to_base_tf(const nav_msgs::Odometry &local_odom){
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

void LocalizationFusion::Spin(){
	ros::spin();
}