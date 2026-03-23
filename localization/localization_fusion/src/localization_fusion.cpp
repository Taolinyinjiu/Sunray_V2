#include "localization_fusion.hpp"
#include "string_uav_namespace_utils.hpp"
#include <stdexcept>

// 由于我们在launch文件中将传入的参数设置为了节点私有参数，因此需要在这里构造私有句柄
LocalizationFusion::LocalizationFusion(ros::NodeHandle &nh) {
  // 缓存全局句柄
  nh_ = nh;
  // 读取节点名
  std::string node_name = ros::this_node::getName();
  // 构造私有节点句柄
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
	// 读取全局参数
	std::string uav_name;
	std::string uav_id;
	if(nh_.getParam("/uav_name",uav_name)){
		// 读取失败，抛出异常
    throw std::runtime_error("missing param /uav_name");
	}
	if(nh_.getParam("/uav_id",uav_id)){
		// 读取失败，抛出异常
    throw std::runtime_error("missing param /uav_id");
	}
	// 拼接uav_ns
	uav_ns_ = uav_name + uav_id;
	// 标准化
	uav_ns_ = sunray_common::normalize_uav_ns(uav_ns_);
}

bool LocalizationFusion::load_param() {
	// 读取参数
	selected_source_ = load_config_from_yaml(config_yamlfile_path_,selected_source_id_,uav_ns_);
	// 只要不是默认值，就说明被覆盖过值，字段名都经过校验，内容由localization_sources.yaml填充
	if(selected_source_.source_id != -1)
	{
		return true;
	}
	return false;
}

void LocalizationFusion::set_publisher(){
	// 首先将输出的topic进行转换
	global_odometry_topic_ = sunray_common::replace_uav_ns(global_odometry_topic_, uav_ns_);
	local_odometry_topic_ = sunray_common::replace_uav_ns(local_odometry_topic_, uav_ns_);
	odom_status_topic_ = sunray_common::replace_uav_ns(odom_status_topic_,uav_ns_);
	// 向全局注册
	local_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(local_odometry_topic_, 10);
  global_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(global_odometry_topic_, 10);
  odom_state_pub_ = nh_.advertise<sunray_msgs::OdomStatus>(odom_status_topic_, 10);
}

