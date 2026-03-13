#include "controller/base_controller/base_controller.hpp"

#include <algorithm>

namespace uav_control {

/**
 * @brief 切换控制器到起飞模式。
 *
 * @details
 * 仅允许在 `OFF` 状态下调用
 * 该函数会锁存地面参考高度，生成起飞目标点（当前位置 +`relative_takeoff_height_`），
 * 并重置起飞过程上下文后切换到 `TAKEOFF` 状态。
        @param relative_takeoff_height 相对起飞高度
        @param max_takeoff_velocity 起飞过程中的最大速度
 * @return true 切换成功；false 前置条件不满足。
 */
bool Base_Controller::set_takeoff_mode(double relative_takeoff_height,
                                       double max_takeoff_velocity) {
  // 如果控制器当前状态不为OFF状态，则返回false
  if (controller_state_ != ControllerState::OFF) {
    return false;
  }
  
	// 刷新home位置
	home_position_ = uav_current_state_.position;
	
	// 更新起飞最大速度
  takeoff_max_velocity_ = max_takeoff_velocity;

  // 进入起飞流程时刷新地面参考高度，供后续降落目标使用。
  ground_reference_z_ = uav_current_state_.position.z();
  ground_reference_initialized_ = true;

  // 以当前位置信息作为起飞参考，并叠加相对起飞高度。
  takeoff_expect_position_ = uav_current_state_.position;
  takeoff_expect_position_.z() += relative_takeoff_height;

  // 进入 TAKEOFF 前重置起飞过程上下文。
  takeoff_initialized_ = false;
  takeoff_holdstart_time_ = ros::Time(0);
  takeoff_holdkeep_time_ = ros::Time(0);
  controller_state_ = ControllerState::TAKEOFF;
  return true;
}

/**
 * @brief 切换控制器到降落模式。
 *
 * @details
 * 若当前已在 `LAND` 状态则视为幂等成功。
 * 否则要求当前状态不是 `OFF/UNDEFINED`
 * 降落目标点以当前位置为基准，若已锁存地面高度则使用
 * `min(current_z, ground_reference_z_)` 防止先上升后下降。
 *
 * @return true 切换成功；false 前置条件不满足。
 */
bool Base_Controller::set_land_mode() {
  // 检查当前是否已经处于LAND状态，是则返回true
  if (controller_state_ == ControllerState::LAND) {
    return true;
  }
  // 检查当前是否处于UNDEFINE或者OFF状态,以及里程计是否有效
  if (controller_state_ == ControllerState::OFF ||
      controller_state_ == ControllerState::UNDEFINED) {
    return false;
  }

  // 以当前位置作为降落参考；降落高度优先使用锁存的地面参考高度。
  land_expect_position_ = uav_current_state_.position;
  // 如果地面高度已经初始化(这个应当在起飞阶段保证)
  if (ground_reference_initialized_) {
    // 在当前位置的z轴数据和锁存的地面参考z轴高度进行对比,取小者
    land_expect_position_.z() =
        std::min(uav_current_state_.position.z(), ground_reference_z_);
  } else {
    land_expect_position_.z() = uav_current_state_.position.z();
  }
  // 如果上一阶段最后的结果是land_expect_position_.z() ==
  // uav_current_state_.position.z();
  // 说明当前可能存在负阶段降落,也就是降落点低于起飞点,这时候由于降落是位置的,因此需要使用另一套参考逻辑
  // 这里直接使用auto_land进行简化测试
  if (land_expect_position_.z() == uav_current_state_.position.z()) {
    land_type_ = 1;
  }
  // 请注意，这里我们没有回退到land_type =
  // 0的实现，因为我们认为，当触发了auto_land后，整个使用场景实际上并不能够稳定的使用基于起飞高度和五次项曲线的降落模式
  land_initialized_ = false;
  land_holdstart_time_ = ros::Time(0);
  land_holdkeep_time_ = ros::Time(0);
  controller_state_ = ControllerState::LAND;
  return true;
}

/**
 * @brief
 * EMERGENCY_LAND作为一个实验性的模式，暂时咩有太好的方式来实现
 * @details
 * `OFF` 和 `UNDEFINED` 状态下拒绝切换，其余状态直接进入
 * `EMERGENCY_LAND`。
 *
 * @return true 切换成功；false 当前状态不允许。
 */
bool Base_Controller::set_emergency_mode() {
  if (controller_state_ == ControllerState::OFF ||
      controller_state_ == ControllerState::UNDEFINED) {
    return false;
  }
  controller_state_ = ControllerState::EMERGENCY_LAND;
  return true;
}

/**
 * @brief 更新 PX4 解锁状态缓存。
 *
 * @param arm_state true 表示已解锁；false 表示未解锁。
 * @return true 始终返回成功。
 */
bool Base_Controller::set_px4_arm_state(bool arm_state) {
  px4_arm_state_ = arm_state;
  return true;
}

/**
 * @brief 更新当前状态估计（里程计）缓存。
 *
 * @param current_state 输入状态估计。
 * @return true 始终返回成功。
 *
 * @details
 * 首次收到有效状态时会自动锁存 `ground_reference_z_`，供后续降落逻辑使用。
 */
bool Base_Controller::set_current_odom(const UAVStateEstimate &current_state) {
  uav_current_state_ = current_state;
  if (!ground_reference_initialized_ && uav_current_state_.isValid()) {
    ground_reference_z_ = uav_current_state_.position.z();
    ground_reference_initialized_ = true;
  }
	controller_ready_ = true;
  return true;
}

/**
 * @brief 获取当前缓存的状态估计。
 *
 * @return 当前状态估计对象的常量引用。
 */
const UAVStateEstimate &Base_Controller::get_current_state() const {
  return uav_current_state_;
}

/**
 * @brief 更新 PX4 姿态四元数缓存。
 *
 * @param imu_msg 输入 IMU 消息（读取其 `orientation` 字段）。
 * @return true 始终返回成功。
 */
bool Base_Controller::set_px4_attitude(const sensor_msgs::Imu &imu_msg) {
  px4_attitude_.x() = imu_msg.orientation.x;
  px4_attitude_.y() = imu_msg.orientation.y;
  px4_attitude_.z() = imu_msg.orientation.z;
  px4_attitude_.w() = imu_msg.orientation.w;
  return true;
}

bool Base_Controller::set_px4_land_status(const bool land_status){
	if(land_status == false)
	{
		px4_land_status_ = false; // 置位
		// 清空时间参数
		land_holdstart_time_ = ros::Time(0);
		land_holdkeep_time_ = ros::Time(0);
		// 结束
		return true;
	}
	// 进入到这里说明设置的值为true 
	if(px4_land_status_ == false){
		px4_land_status_ = true; // 首先，置位
		land_holdstart_time_ = ros::Time::now(); // 设置检测落地接触时间为当前时间
	}
	return true;
}

/**
 * @brief 更新控制参考轨迹点。
 *
 * @param trajectory 输入轨迹点。
 * @return true 始终返回成功。
 *
 * @details
 * 当 `valid_mask` 为 `UNDEFINED` 时，会按非零字段推断有效通道，
 * 用于兼容未显式设置有效位的旧调用路径。
 */
bool Base_Controller::set_trajectory(const TrajectoryPoint &trajectory) {
  trajectory_ = trajectory;
  if (trajectory_.valid_mask ==
      static_cast<uint32_t>(TrajectoryPoint::ValidMask::UNDEFINED)) {
    trajectory_.infer_valid_mask_from_nonzero();
  }
  return true;
}

/**
 * @brief 获取控制器内部状态机当前状态。
 *
 * @return 当前 `ControllerState`。
 */
ControllerState Base_Controller::get_controller_state() const {
  return controller_state_;
}

/**
 * @brief 默认起飞完成判定。
 *
 * @return true 当前状态为 `HOVER`；否则 false。
 */
bool Base_Controller::is_takeoff_completed() const {
  if (controller_state_ == ControllerState::HOVER)
    return true;
  return false;
}

/**
 * @brief 默认降落完成判定。
 *
 * @return true 当前状态为 `OFF`；否则 false。
 */
bool Base_Controller::is_land_completed() const {
  if (controller_state_ == ControllerState::OFF)
    return true;
  return false;
}

/**
 * @brief 默认紧急降落完成判定。
 *
 * @return true 当前状态为 `OFF`；否则 false。
 */
bool Base_Controller::is_emergency_completed() const {
  if (controller_state_ == ControllerState::OFF)
    return true;
  return false;
}

/**
 * @brief 控制器健康检查（基础实现）。
 *
 * @return true 参数已加载且当前状态估计有效；否则 false。
 */
bool Base_Controller::is_ready() const {
  return controller_ready_ && uav_current_state_.isValid();
}

} // namespace uav_control
