/***
        @brief 先构造辅助类
        辅助类的目的是向开发者提供一个友好的,高宽容度的Sunray无人机控制接口,Helper主要与Surnay_FSM进行通信,获取Sunray_FSM的状态,无人机的状态,以及调用相关的运动接口
        */

#include <px4_bridge/px4_data_reader.h>
#include <px4_bridge/px4_param_manager.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <control_data_types/uav_state_estimate.hpp>
#include "sunray_statemachine/sunray_statemachine_datatypes.h"
#include <memory>
#include <string>
#include <utility>
#include <vector>
// 前项定义

class Sunray_Helper {
public:
  // 构造函数
  Sunray_Helper(ros::NodeHandle &nh_);
  // 析构函数
  ~Sunray_Helper() = default;
  // async 异步，非阻塞,使用话题的形式发布
  // block(spin) 同步，阻塞，使用服务的方式实现，没有服务接口的，使用话题回调
  // 状态检验实现
  // -------------------------控制接口--------------------------------
  // 触发起飞
  bool takeoff_async();
  bool takeoff_block();
  // 触发降落
  bool land_async();
  bool land_block();
	// 触发返航
  bool return_async();
  bool return_block();
  // 触发位置控制
  bool set_position_async(Eigen::Vector3d position_);
  bool set_position_block(Eigen::Vector3d position_);
  bool set_position_list_async(std::vector<Eigen::Vector3d> position_list_);
  bool set_position_list_bolck(std::vector<Eigen::Vector3d> position_list_);
  // 带有yaw角的位置控制
  bool set_position_async(Eigen::Vector3d position_, float yaw);
  bool set_position_block(Eigen::Vector3d position_, float yaw);
  bool set_position_list_async(
      std::vector<std::pair<Eigen::Vector3d, float>> point_list_);
  bool set_position_list_block(
      std::vector<std::pair<Eigen::Vector3d, float>> point_list_);
  // 触发速度控制
  bool set_linear_velocity_async(Eigen::Vector3d velocity_);
  bool set_angular_velocity_async(Eigen::Vector3d velocity_);
  bool set_position_velocity_async(Eigen::Vector3d position_, float velocity_);
  bool set_position_velocity_block(Eigen::Vector3d position_, float velocity_);
  bool set_position_velocity_list_async(
      std::vector<std::pair<Eigen::Vector3d, float>> point_list_);
  bool set_position_velocity_list_block(
      std::vector<std::pair<Eigen::Vector3d, float>> point_list_);
  // 触发姿态控制
  // 绝对yaw角控制
  bool set_yaw_async(float yaw_);
  bool set_yaw_bolck(float yaw_);
  // 相对当前时刻yaw角控制
  bool set_yaw_adjust_async(float adjust_yaw_);
  bool set_yaw_adjust_block(float adjust_yaw_);
  // 触发轨迹控制
  // TODO: 添加轨迹 数据类型
  bool set_trajectory_asycn();
  bool set_trajectory_block();

  // 复合控制模式，自适应控制频率?
  // 假设存在这样的使用场景，用户需要测试自己的控制模型
  // uav_state + setpoint -> 用户模型 -> 控制量(位置+速度+姿态+推力)
  // 此时，如果用户不想修改我们的控制器，或者说希望先使用我们稳定的控制器，悬停后切换到他们的控制输出量
  // 这里使用参数，用户选择是否屏蔽控制器的输出，如果屏蔽控制器的输出，则直通mavros接口
  // 请注意，当姿态+推力接口控制频率小于50Hz时会 ？(这里需要做一些措施)
  bool set_complex_control();

  // -------------------------查询接口--------------------------------
  // 无人机当前状态
  uav_control::UAVStateEstimate get_uav_odometry();
  Eigen::Vector3d get_uav_position();
  Eigen::Vector3d get_uav_velocity_linear();
  Eigen::Vector3d get_uav_velocity_angular();
  Eigen::Vector3d get_uav_attitude_rpy_rad();
  Eigen::Vector3d get_uav_attitude_rpy_deg();
  Eigen::Quaterniond get_uav_attitude_quat();
  // 得到设定的目标状态
  Eigen::Vector3d get_target_position();
  Eigen::Vector3d get_target_velocity_linear();
  Eigen::Vector3d get_target_velocity_angular();
  Eigen::Vector3d get_target_attitude_rpy_rad();
  Eigen::Vector3d get_target_attitude_rpy_deg();
  Eigen::Quaterniond get_target_attitude_quat();
  float get_target_thrust();
  // Sunray FSM状态
  // TODO:实现Sunray状态机 状态的数据类型,本质上是强类型枚举
  sunray_fsm::SunrayState get_statemachine_state();
  //

private:
  ros::NodeHandle nh_;
  ros::NodeHandle ctrl_nh_;
  std::string uav_ns_;

  std::unique_ptr<PX4_DataReader> px4_data_reader_;
  bool px4_reader_ready_{false};

  // 里程计消息缓存
  uav_control::UAVStateEstimate uav_odometry_;
  //  无人机目标状态缓存
  uav_control::UAVStateEstimate uav_target_;
  // 状态机状态缓存
  sunray_fsm::SunrayState fsm_state_;

  // 声明与Sunray_FSM相关的发布者
  // 触发模式相关
  ros::Publisher takeoff_pub_;
  ros::Publisher land_pub_;
  ros::Publisher return_pub_;
  // 控制接口相关
  ros::Publisher position_cmd_pub_;
  ros::Publisher velocity_cmd_pub_;
  ros::Publisher attitude_cmd_pub_;
  ros::Publisher trajectory_cmd_pub_;
  ros::Publisher complex_cmd_pub_;
  // 服务客户端(通过服务实现的控制，都是阻塞的方式)
  ros::ServiceClient takeoff_client_;
  ros::ServiceClient land_client_;
  ros::ServiceClient return_client_;
  ros::ServiceClient position_cmd_client_;
};
