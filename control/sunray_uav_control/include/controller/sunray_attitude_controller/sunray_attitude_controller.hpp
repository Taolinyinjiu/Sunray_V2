#pragma once

#include "controller/base_controller/base_controller.hpp"

namespace uav_control{

class Attitude_Controller : public Base_Controller{
	public:
	// 姿态控制器需要有自己的构造函数，用于加载参数
	bool load_param(ros::NodeHandle &nh);
	// 重写控制器周期更新入口函数
	ControllerOutput update(void) override;

	private:
	// 离开起飞状态时重置起飞上下文
	void reset_takeoff_context_if_needed();
	// 离开降落状态时重置降落上下文
	void reset_land_context_if_needed();
	/** @brief 处理 `UNDEFINED` 状态并返回对应控制输出。 */
  ControllerOutput handle_undefined_state();
  /** @brief 处理 `OFF` 状态并返回对应控制输出。 */
  ControllerOutput handle_off_state();
  /** @brief 处理 `TAKEOFF` 状态并返回对应控制输出。 */
  ControllerOutput handle_takeoff_state();
  /** @brief 处理 `HOVER` 状态并返回对应控制输出。 */
  ControllerOutput handle_hover_state();
  /** @brief 处理 `MOVE` 状态并返回对应控制输出。 */
  ControllerOutput handle_move_state();
  /** @brief 处理 `LAND` 状态并返回对应控制输出。 */
  ControllerOutput handle_land_state();
};

};