#include "sunray_statemachine/sunray_statemachine.h"
#include "controller/px4_position_controller/px4_position_controller.h"
#include <algorithm>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>

namespace {
template <typename CovarianceArray>
bool has_meaningful_covariance(const CovarianceArray &cov) {
  for (const double v : cov) {
    if (std::isfinite(v) && std::abs(v) > 1e-9) {
      return true;
    }
  }
  return false;
}
} // namespace

namespace sunray_fsm {

// 状态机构造函数，引用ros句柄，读取sunray_control_config.yaml中的参数
Sunray_StateMachine::Sunray_StateMachine(ros::NodeHandle &nh)
    : nh_(nh), uav_ns_(resolve_uav_namespace()),
      fsm_current_state_(SunrayState::OFF), fsm_param_config_{},
      external_odom_sub_(), latest_external_odom_{},
      latest_external_odom_msg_{}, odom_to_px4_vision_pose_pub_(),
      odom_to_px4_odometry_pub_(), odom_fuse_timer_(), px4_data_reader_(nh_),
      px4_param_manager_(nh_), px4_arming_client_(), px4_set_mode_client_(),
      px4_offboard_retry_state_{}, sunray_controller_(nullptr),
      controller_update_timer_(), arbiter_() {
  // 如果uav_ns命名空间为空，则在ros全局参数空间中查找参数/uav_name与/uav_id，然后拼接uav_ns命名空间
  if (uav_ns_.empty()) {
    std::string fallback_uav_name = "uav";
    int fallback_uav_id = 1;
    nh_.param("/uav_name", fallback_uav_name, fallback_uav_name);
    nh_.param("/uav_id", fallback_uav_id, fallback_uav_id);
    uav_ns_ = fallback_uav_name + std::to_string(fallback_uav_id);
  }
  // 构造config_ns命名空间，用于读取在uav_name+uav_id命名空间下的参数
  const std::string config_ns = uav_ns_.empty() ? "" : ("/" + uav_ns_);
  ros::NodeHandle cfg_nh = config_ns.empty() ? nh_ : ros::NodeHandle(config_ns);
  // 构造ctrl命名空间，用于发布控制参数
  const std::string ctrl_ns = "/" + uav_ns_ + "/sunray_control";
  ctrl_nh_ = ros::NodeHandle(ctrl_ns);

  // 1) 读取参数，顺序与sunray_control_config.yaml一致
  // -------------------基本参数-----------------------
  cfg_nh.param("uav_name", fsm_param_config_.uav_name,
               fsm_param_config_.uav_name);
  cfg_nh.param("uav_id", fsm_param_config_.uav_id, fsm_param_config_.uav_id);
  cfg_nh.param("mass_kg", fsm_param_config_.mass_kg, fsm_param_config_.mass_kg);
  cfg_nh.param("gravity", fsm_param_config_.gravity, fsm_param_config_.gravity);

  cfg_nh.param("controller_types", fsm_param_config_.controller_type,
               fsm_param_config_.controller_type);
  cfg_nh.param("controller_update_frequency",
               fsm_param_config_.controller_update_hz,
               fsm_param_config_.controller_update_hz);

  cfg_nh.param("odom_topic_name", fsm_param_config_.odom_topic_name,
               fsm_param_config_.odom_topic_name);
  cfg_nh.param("fuse_odom_to_px4", fsm_param_config_.fuse_odom_to_px4,
               fsm_param_config_.fuse_odom_to_px4);
  cfg_nh.param("fuse_odom_type", fsm_param_config_.fuse_odom_type,
               fsm_param_config_.fuse_odom_type);
  cfg_nh.param("fuse_odom_frequency", fsm_param_config_.fuse_odom_frequency_hz,
               fsm_param_config_.fuse_odom_frequency_hz);

  // -------------------保护参数-----------------------
  cfg_nh.param("low_voltage", fsm_param_config_.low_voltage_v,
               fsm_param_config_.low_voltage_v);
  cfg_nh.param("low_voltage_operate", fsm_param_config_.low_voltage_action,
               fsm_param_config_.low_voltage_action);
  cfg_nh.param("control_with_no_rc", fsm_param_config_.control_with_no_rc,
               fsm_param_config_.control_with_no_rc);
  cfg_nh.param("lost_with_rc", fsm_param_config_.lost_with_rc_action,
               fsm_param_config_.lost_with_rc_action);
  cfg_nh.param("arm_with_code", fsm_param_config_.arm_with_code,
               fsm_param_config_.arm_with_code);
  cfg_nh.param("takeoff_with_code", fsm_param_config_.takeoff_with_code,
               fsm_param_config_.takeoff_with_code);
  cfg_nh.param("check_flip", fsm_param_config_.check_flip,
               fsm_param_config_.check_flip);
  cfg_nh.param("tilt_angle_max", fsm_param_config_.tilt_angle_max_deg,
               fsm_param_config_.tilt_angle_max_deg);

  // -------------------起飞降落参数-----------------------
  cfg_nh.param("takeoff_relative_height", fsm_param_config_.takeoff_height_m,
               fsm_param_config_.takeoff_height_m);
  cfg_nh.param("takeoff_max_velocity", fsm_param_config_.takeoff_max_vel_mps,
               fsm_param_config_.takeoff_max_vel_mps);
  cfg_nh.param("land_type", fsm_param_config_.land_type,
               fsm_param_config_.land_type);
  cfg_nh.param("land_max_velocity", fsm_param_config_.land_max_vel_mps,
               fsm_param_config_.land_max_vel_mps);

  // -------------------电子围栏参数-----------------------
  cfg_nh.param("electronic_fence/x_max", fsm_param_config_.fence_x_max,
               fsm_param_config_.fence_x_max);
  cfg_nh.param("electronic_fence/x_min", fsm_param_config_.fence_x_min,
               fsm_param_config_.fence_x_min);
  cfg_nh.param("electronic_fence/y_max", fsm_param_config_.fence_y_max,
               fsm_param_config_.fence_y_max);
  cfg_nh.param("electronic_fence/y_min", fsm_param_config_.fence_y_min,
               fsm_param_config_.fence_y_min);
  cfg_nh.param("electronic_fence/z_max", fsm_param_config_.fence_z_max,
               fsm_param_config_.fence_z_max);
  cfg_nh.param("electronic_fence/z_min", fsm_param_config_.fence_z_min,
               fsm_param_config_.fence_z_min);

  // -------------------消息超时参数-----------------------
  cfg_nh.param("msg_timeout/odom", fsm_param_config_.timeout_odom_s,
               fsm_param_config_.timeout_odom_s);
  cfg_nh.param("msg_timeout/rc", fsm_param_config_.timeout_rc_s,
               fsm_param_config_.timeout_rc_s);
  cfg_nh.param("msg_timeout/control_heartbeat",
               fsm_param_config_.timeout_control_hb_s,
               fsm_param_config_.timeout_control_hb_s);
  cfg_nh.param("msg_timeout/imu", fsm_param_config_.timeout_imu_s,
               fsm_param_config_.timeout_imu_s);
  cfg_nh.param("msg_timeout/battery", fsm_param_config_.timeout_battery_s,
               fsm_param_config_.timeout_battery_s);
  // -------------------位置误差参数-----------------------
  cfg_nh.param("error_tolerance/pos_x",
               fsm_param_config_.error_tolerance_pos_x_m,
               fsm_param_config_.error_tolerance_pos_x_m);
  cfg_nh.param("error_tolerance/pos_y",
               fsm_param_config_.error_tolerance_pos_y_m,
               fsm_param_config_.error_tolerance_pos_y_m);
  cfg_nh.param("error_tolerance/pos_z",
               fsm_param_config_.error_tolerance_pos_z_m,
               fsm_param_config_.error_tolerance_pos_z_m);
  // -------------------飞行速度参数-----------------------
  cfg_nh.param("max_velocity/x_vel", fsm_param_config_.max_velocity_x_mps,
               fsm_param_config_.max_velocity_x_mps);
  cfg_nh.param("max_velocity/y_vel", fsm_param_config_.max_velocity_y_mps,
               fsm_param_config_.max_velocity_y_mps);
  cfg_nh.param("max_velocity/z_vel", fsm_param_config_.max_velocity_z_mps,
               fsm_param_config_.max_velocity_z_mps);
  cfg_nh.param("max_velocity_with_rc/x_vel",
               fsm_param_config_.max_velocity_with_rc_x_mps,
               fsm_param_config_.max_velocity_with_rc_x_mps);
  cfg_nh.param("max_velocity_with_rc/y_vel",
               fsm_param_config_.max_velocity_with_rc_y_mps,
               fsm_param_config_.max_velocity_with_rc_y_mps);
  cfg_nh.param("max_velocity_with_rc/z_vel",
               fsm_param_config_.max_velocity_with_rc_z_mps,
               fsm_param_config_.max_velocity_with_rc_z_mps);

  // 2) 初始化 MAVROS service client
  const std::string mavros_ns =
      uav_ns_.empty() ? "/mavros" : ("/" + uav_ns_ + "/mavros");
  px4_arming_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>(mavros_ns + "/cmd/arming");
  px4_set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>(mavros_ns + "/set_mode");
  odom_to_px4_vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      mavros_ns + "/vision_pose/pose", 20);
  odom_to_px4_odometry_pub_ =
      nh_.advertise<nav_msgs::Odometry>(mavros_ns + "/odometry/in", 20);
  external_odom_sub_ =
      nh_.subscribe(fsm_param_config_.odom_topic_name, 20,
                    &Sunray_StateMachine::external_odom_cb, this);
  odom_fuse_timer_ = nh_.createTimer(
      ros::Duration(1.0 / fsm_param_config_.fuse_odom_frequency_hz),
      &Sunray_StateMachine::odom_fuse_timer_cb, this);

  // 3) 初始化仲裁器
  if (!arbiter_.init(nh_)) {
    ROS_WARN("[SunrayFSM] arbiter init failed");
  }

  // 4) 注册控制器 + 启动周期 update 定时器
  (void)register_controller(fsm_param_config_.controller_type);

  const double hz = std::max(50.0, fsm_param_config_.controller_update_hz);
  controller_update_timer_ =
      nh_.createTimer(ros::Duration(1.0 / hz),
                      &Sunray_StateMachine::controller_update_timer_cb, this);
  
	// 5) 发布对应控制接口(先占位置)
  // 话题订阅
  takeoff_cmd_sub_ = ctrl_nh_.subscribe(
      "takeoff_cmd", 10, &Sunray_StateMachine::takeoff_cmd_cb, this);
  land_cmd_sub_ = ctrl_nh_.subscribe("land_cmd", 10,
                                     &Sunray_StateMachine::land_cmd_cb, this);
  return_cmd_sub_ = ctrl_nh_.subscribe(
      "return_cmd", 10, &Sunray_StateMachine::return_cmd_cb, this);
  position_cmd_sub_ = ctrl_nh_.subscribe(
      "position_cmd", 10, &Sunray_StateMachine::position_cmd_cb, this);
	// adve	
  velocity_cmd_sub_ = ctrl_nh_.subscribe(
      "velocity_cmd", 10, &Sunray_StateMachine::velocity_cmd_cb, this);

  attitude_cmd_sub_ = ctrl_nh_.subscribe(
      "attitude_cmd", 10, &Sunray_StateMachine::attitude_cmd_cb, this);

  trajectory_cmd_sub_ = ctrl_nh_.subscribe(
      "trajectory_cmd", 10, &Sunray_StateMachine::trajectory_cmd_cb, this);

  complex_cmd_sub_ = ctrl_nh_.subscribe(
      "complex_cmd", 10, &Sunray_StateMachine::complex_cmd_cb, this);

  // 服务
  takeoff_srv_ = ctrl_nh_.advertiseService(
      "takeoff_request", &Sunray_StateMachine::takeoff_srv_cb, this);
  land_srv_ = ctrl_nh_.advertiseService(
      "land_request", &Sunray_StateMachine::land_srv_cb, this);
  return_srv_ = ctrl_nh_.advertiseService(
      "return_request", &Sunray_StateMachine::return_srv_cb, this);
  position_srv_ = ctrl_nh_.advertiseService(
      "position_request", &Sunray_StateMachine::position_srv_cb, this);
  
	// 
	ROS_INFO("[SunrayFSM] init done, uav_ns='%s', state=OFF, odom='%s'",
           uav_ns_.c_str(), fsm_param_config_.odom_topic_name.c_str());
}

bool Sunray_StateMachine::register_controller(int controller_types) {
  // controller_type 约定（与 yaml 对齐）:
  // 0: PX4 position controller（已实现）
  // 1: px4_velocity_controller（预留）
  // 2: sunray_attitude_controller #2（预留）
  // 3: rl_raptor_controller #3（预留）
  std::shared_ptr<uav_control::Base_Controller> selected_controller;

  switch (controller_types) {
  case 0:
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  case 1:
    ROS_WARN("[SunrayFSM] controller_type=1 reserved, fallback to type=0 "
             "(PX4 position controller)");
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  case 2:
    ROS_WARN("[SunrayFSM] controller_type=2 reserved, fallback to type=0 "
             "(PX4 position controller)");
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  case 3:
    ROS_WARN("[SunrayFSM] controller_type=3 reserved, fallback to type=0 "
             "(PX4 position controller)");
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  default:
    ROS_WARN("[SunrayFSM] unknown controller_type=%d, fallback to type=0 "
             "(PX4 position controller)",
             controller_types);
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  }

  if (!selected_controller) {
    ROS_ERROR("[SunrayFSM] register_controller failed: create controller "
              "instance failed (type=%d)",
              controller_types);
    return false;
  }

  sunray_controller_ = selected_controller;
  ROS_INFO("[SunrayFSM] register global controller success, type=%d",
           controller_types);
  return true;
}

bool Sunray_StateMachine::handle_event(SunrayEvent event) {
  const SunrayState prev = fsm_current_state_;

  switch (fsm_current_state_) {
  case SunrayState::OFF:
    if (event == SunrayEvent::TAKEOFF_REQUEST && can_takeoff()) {
      return transition_to(SunrayState::TAKEOFF);
    }
    break;

  case SunrayState::TAKEOFF:
    if (event == SunrayEvent::TAKEOFF_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::HOVER:
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    if (event == SunrayEvent::ENTER_POSITION_CONTROL) {
      return transition_to(SunrayState::POSITION_CONTROL);
    }
    if (event == SunrayEvent::ENTER_VELOCITY_CONTROL) {
      return transition_to(SunrayState::VELOCITY_CONTROL);
    }
    if (event == SunrayEvent::ENTER_ATTITUDE_CONTROL) {
      return transition_to(SunrayState::ATTITUDE_CONTROL);
    }
    if (event == SunrayEvent::ENTER_COMPLEX_CONTROL) {
      return transition_to(SunrayState::COMPLEX_CONTROL);
    }
    if (event == SunrayEvent::ENTER_TRAJECTORY_CONTROL) {
      return transition_to(SunrayState::TRAJECTORY_CONTROL);
    }
    break;

  case SunrayState::RETURN:
    if (event == SunrayEvent::RETURN_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::LAND:
    if (event == SunrayEvent::LAND_COMPLETED) {
      return transition_to(SunrayState::OFF);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::EMERGENCY_LAND:
    if (event == SunrayEvent::EMERGENCY_COMPLETED) {
      return transition_to(SunrayState::OFF);
    }
    break;

  case SunrayState::POSITION_CONTROL:
    if (event == SunrayEvent::POSITION_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::VELOCITY_CONTROL:
    if (event == SunrayEvent::VELOCITY_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::ATTITUDE_CONTROL:
    if (event == SunrayEvent::ATTITUDE_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::COMPLEX_CONTROL:
    if (event == SunrayEvent::COMPLEX_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::TRAJECTORY_CONTROL:
    if (event == SunrayEvent::TRAJECTORY_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;
  }

  ROS_WARN("[SunrayFSM] ignore event %s at state %s", to_string(event),
           to_string(prev));
  return false;
}
/** -----------------状态机主要更新函数--------------------- */
void Sunray_StateMachine::update() {
  // 首先进行安全检查,此处主要是先构建逻辑框架，check_health() 总是返回true
  if (!check_health()) {
    // 如果安全检查不通过，并且状态机不在EMERGENCY_LAND状态
    if (fsm_current_state_ != SunrayState::EMERGENCY_LAND) {
      // 切换到EMERGENCY_LAND状态
      transition_to(SunrayState::EMERGENCY_LAND);
    }
  }
  // get_controller()函数将状态机内部保存的控制器指针返回给调用者，也就是这里的controller
  const std::shared_ptr<uav_control::Base_Controller> controller =
      get_controller();
  // 如果指针返回的是false 那么警告没有注册控制器
  if (!controller) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] no controller registered");
    return;
  }
  // 从px4_data_reader中得到px4飞控状态
  const px4_data_types::SystemState px4_state =
      px4_data_reader_.get_system_state();
  // 向控制器传递解锁状态
  (void)controller->set_px4_arm_state(px4_state.armed);
  // 得到当前时间戳
  const ros::Time now = ros::Time::now();
  // 里程计有效的三个条件
  // 1. 里程计有效
  // 2.时间戳非零
  // 3.当前时间戳减去里程计时间戳小于config中定义的超时限制
  const bool external_odom_fresh =
      latest_external_odom_.isValid() &&
      !latest_external_odom_.timestamp.isZero() &&
      (now - latest_external_odom_.timestamp).toSec() <=
          fsm_param_config_.timeout_odom_s;
  // 如果里程计无效
  if (!external_odom_fresh) {
    // 在里程计无效的时候，状态机的状态不是OFF或者EMERGENCY_LAND,说明无人机在空中
    if (fsm_current_state_ != SunrayState::OFF &&
        fsm_current_state_ != SunrayState::EMERGENCY_LAND) {
      // 切换到EMERGENCY_LAND状态，紧急降落
      (void)transition_to(SunrayState::EMERGENCY_LAND);
    }
    ROS_WARN_THROTTLE(
        1.0,
        "[SunrayFSM] external odom unavailable or timeout (timeout=%.3fs), "
        "switch to EMERGENCY",
        fsm_param_config_.timeout_odom_s);
  } else {
    // 里程计有效，向控制器注入里程计
    (void)controller->set_current_odom(latest_external_odom_);
  }
  // TODO：无论里程计是否有效，都需要注入px4的姿态
  // (void)controller->set_px4_attitude(const sensor_msgs::Imu &imu_msg);

  // 进入飞行相关状态后，持续确保 OFFBOARD + ARM
  if (requires_offboard() && !ensure_offboard_and_arm()) {
    ROS_WARN_THROTTLE(
        1.0, "[SunrayFSM] waiting for OFFBOARD/ARM before effective control");
  }

  // 根据当前状态机的状态，进行不同的操作
  switch (fsm_current_state_) {
  case SunrayState::TAKEOFF:
    // 起飞状态，触发控制器的起飞
    (void)controller->set_takeoff_mode(fsm_param_config_.takeoff_height_m,
                                       fsm_param_config_.takeoff_max_vel_mps);
    break;
  case SunrayState::LAND:
    // 降落状态，触发控制器的降落
    (void)controller->set_land_mode();
    break;
    // 紧急降落状态，触发控制器的紧急降落
  case SunrayState::EMERGENCY_LAND:
    (void)controller->set_emergency_mode();
    break;
  default:
    break;
  }
  // 更新控制器输出量
  const uav_control::ControllerOutput control_output = controller->update();
  // 有效输出定义为位置、速度、姿态、推力至少有一个是使能的
  const bool has_effective_output =
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::POSITION) ||
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::VELOCITY) ||
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::ATTITUDE) ||
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::THRUST);
  // 如果不存在有效输出
  if (!has_effective_output) {
    // 如果当前控制器的状态为OFF状态，那就没什么事儿，直接结束就行
    if (fsm_current_state_ == SunrayState::OFF) {
      return;
    }
    ROS_WARN_THROTTLE(
        1.0, "[SunrayFSM] controller produced empty output at state=%s",
        to_string(fsm_current_state_));
    return;
  }
  // 向仲裁器设置状态机的状态
  arbiter_.set_fsm_state(fsm_current_state_);
  // 向仲裁器设置状态机的状态
  arbiter_.set_uav_state(controller->get_current_state());
  // 如果当前处于状态机的紧急降落状态，设置为高优先级(255)
  if (fsm_current_state_ == SunrayState::EMERGENCY_LAND) {
    arbiter_.submit(
        uav_control::Sunray_Control_Arbiter::ControlSource::EMERGENCY,
        control_output, ros::Time::now(), 255U);
  } else {
    arbiter_.submit(
        uav_control::Sunray_Control_Arbiter::ControlSource::EXTERNAL,
        control_output, ros::Time::now(), 100U);
  }
  // 仲裁器进行仲裁并输出到mavros中
  (void)arbiter_.arbitrate_and_publish();
}

// 控制器更新回调函数
void Sunray_StateMachine::controller_update_timer_cb(const ros::TimerEvent &) {
  update();
}

// 向px4融合里程计的回调函数
void Sunray_StateMachine::odom_fuse_timer_cb(const ros::TimerEvent &) {
  publish_external_odom_to_px4();
}

// 发布外部里程计数据到px4
void Sunray_StateMachine::publish_external_odom_to_px4() {
  // 首先检查config中是否运行向px4进行里程计融合
  if (!fsm_param_config_.fuse_odom_to_px4) {
    return;
  }
  // 如果外部里程计无效，有两种可能
  // 1. 消息没有到达
  // 2. 消息坐标系有问题
  if (!latest_external_odom_.isValid()) {
    return;
  }

  // 得到当前时间戳
  const ros::Time now = ros::Time::now();
  // 如果里程计不附带时间戳，就使用当前时间戳
  const ros::Time stamp = latest_external_odom_.timestamp.isZero()
                              ? now
                              : latest_external_odom_.timestamp;
  const double age_s = (now - stamp).toSec();
  // 校验是否超时
  if (age_s < 0.0 || age_s > fsm_param_config_.timeout_odom_s) {
    ROS_WARN_THROTTLE(1.0,
                      "[SunrayFSM] skip odom->px4 publish: external odom "
                      "timeout, age=%.3f s timeout=%.3f s",
                      age_s, fsm_param_config_.timeout_odom_s);
    return;
  }
  // 根据参数选择通道对里程计进行融合 0:vision_pose 1:odometry
  if (fsm_param_config_.fuse_odom_type == 0) {
    // 使用vision_pose进行融合
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = latest_external_odom_msg_.header;
    if (pose_msg.header.stamp.isZero()) {
      pose_msg.header.stamp = now;
    }
    pose_msg.pose = latest_external_odom_msg_.pose.pose;
    odom_to_px4_vision_pose_pub_.publish(pose_msg);
  } else if (fsm_param_config_.fuse_odom_type == 1) {
    nav_msgs::Odometry odom_msg = latest_external_odom_msg_;
    if (odom_msg.header.stamp.isZero()) {
      odom_msg.header.stamp = now;
    }
    if (odom_msg.child_frame_id.empty()) {
      odom_msg.child_frame_id = "body";
    }
    odom_to_px4_odometry_pub_.publish(odom_msg);
  }
}

// 外部里程计回调
void Sunray_StateMachine::external_odom_cb(
    const nav_msgs::Odometry::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  // 使用UAVStateEstimate结构体快速解析
  uav_control::UAVStateEstimate odom_state(*msg);
  // 如果时间戳为零，则设置为当前时间戳
  if (odom_state.timestamp.isZero()) {
    odom_state.timestamp = ros::Time::now();
  }
  // 如果里程计无效，则结束
  if (!odom_state.isValid()) {
    ROS_WARN_THROTTLE(1.0,
                      "[SunrayFSM] ignore invalid external odom frame='%s'",
                      msg->header.frame_id.c_str());
    return;
  }
  //
  latest_external_odom_ = odom_state;
  latest_external_odom_msg_ = *msg;
  // 如果时间戳为零，则设置为当前时间戳
  if (latest_external_odom_msg_.header.stamp.isZero()) {
    latest_external_odom_msg_.header.stamp = odom_state.timestamp;
  }
}

// 解算命名空间
std::string Sunray_StateMachine::resolve_uav_namespace() const {
  std::string key;
  std::string ns;
  if (nh_.searchParam("uav_ns", key) && nh_.getParam(key, ns) && !ns.empty()) {
    if (!ns.empty() && ns.front() == '/') {
      return ns.substr(1);
    }
    return ns;
  }

  std::string name;
  int id = 0;
  bool ok_name = false;
  bool ok_id = false;
  if (nh_.searchParam("uav_name", key)) {
    ok_name = nh_.getParam(key, name) && !name.empty();
  }
  if (nh_.searchParam("uav_id", key)) {
    ok_id = nh_.getParam(key, id);
  }
  if (ok_name && ok_id) {
    return name + std::to_string(id);
  }
  return "";
}

bool Sunray_StateMachine::requires_offboard() const {
  switch (fsm_current_state_) {
  case SunrayState::TAKEOFF:
  case SunrayState::HOVER:
  case SunrayState::RETURN:
  case SunrayState::LAND:
  case SunrayState::EMERGENCY_LAND:
  case SunrayState::POSITION_CONTROL:
  case SunrayState::VELOCITY_CONTROL:
  case SunrayState::ATTITUDE_CONTROL:
  case SunrayState::COMPLEX_CONTROL:
  case SunrayState::TRAJECTORY_CONTROL:
    return true;
  case SunrayState::OFF:
  default:
    return false;
  }
}

bool Sunray_StateMachine::ensure_offboard_and_arm() {
  const px4_data_types::SystemState px4_state =
      px4_data_reader_.get_system_state();

  if (!px4_state.connected) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] PX4 is not connected");
    return false;
  }

  if (!px4_set_mode_client_.isValid() || !px4_arming_client_.isValid()) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] mavros service clients are not ready");
    return false;
  }

  const ros::Time now = ros::Time::now();

  if (px4_state.flight_mode != px4_data_types::FlightMode::kOffboard) {
    if (px4_offboard_retry_state_.last_set_mode_req_time.isZero() ||
        (now - px4_offboard_retry_state_.last_set_mode_req_time).toSec() >=
            px4_offboard_retry_state_.set_mode_retry_interval_s) {
      mavros_msgs::SetMode mode_cmd;
      mode_cmd.request.custom_mode = "OFFBOARD";
      if (px4_set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("[SunrayFSM] OFFBOARD mode request sent");
      } else {
        ROS_WARN_THROTTLE(1.0, "[SunrayFSM] OFFBOARD mode request failed");
      }
      px4_offboard_retry_state_.last_set_mode_req_time = now;
    }
    return false;
  }

  if (!px4_state.armed) {
    if (px4_offboard_retry_state_.last_arm_req_time.isZero() ||
        (now - px4_offboard_retry_state_.last_arm_req_time).toSec() >=
            px4_offboard_retry_state_.arm_retry_interval_s) {
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;
      if (px4_arming_client_.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("[SunrayFSM] ARM request sent");
      } else {
        ROS_WARN_THROTTLE(1.0, "[SunrayFSM] ARM request failed");
      }
      px4_offboard_retry_state_.last_arm_req_time = now;
    }
    return false;
  }

  return true;
}
// 返回当前状态机的状态
SunrayState Sunray_StateMachine::get_current_state() const {
  return fsm_current_state_;
}
// 将当前状态机的状态转换为字符串
const char *Sunray_StateMachine::to_string(SunrayState state) {
  switch (state) {
  case SunrayState::OFF:
    return "OFF";
  case SunrayState::TAKEOFF:
    return "TAKEOFF";
  case SunrayState::HOVER:
    return "HOVER";
  case SunrayState::RETURN:
    return "RETURN";
  case SunrayState::LAND:
    return "LAND";
  case SunrayState::EMERGENCY_LAND:
    return "EMERGENCY_LAND";
  case SunrayState::POSITION_CONTROL:
    return "POSITION_CONTROL";
  case SunrayState::VELOCITY_CONTROL:
    return "VELOCITY_CONTROL";
  case SunrayState::ATTITUDE_CONTROL:
    return "ATTITUDE_CONTROL";
  case SunrayState::COMPLEX_CONTROL:
    return "COMPLEX_CONTROL";
  case SunrayState::TRAJECTORY_CONTROL:
    return "TRAJECTORY_CONTROL";
  default:
    return "UNKNOWN_STATE";
  }
}
// 将当前状态机的事件转换为字符串
const char *Sunray_StateMachine::to_string(SunrayEvent event) {
  switch (event) {
  case SunrayEvent::TAKEOFF_REQUEST:
    return "TAKEOFF_REQUEST";
  case SunrayEvent::TAKEOFF_COMPLETED:
    return "TAKEOFF_COMPLETED";
  case SunrayEvent::LAND_REQUEST:
    return "LAND_REQUEST";
  case SunrayEvent::LAND_COMPLETED:
    return "LAND_COMPLETED";
  case SunrayEvent::EMERGENCY_REQUEST:
    return "EMERGENCY_REQUEST";
  case SunrayEvent::EMERGENCY_COMPLETED:
    return "EMERGENCY_COMPLETED";
  case SunrayEvent::RETURN_REQUEST:
    return "RETURN_REQUEST";
  case SunrayEvent::RETURN_COMPLETED:
    return "RETURN_COMPLETED";
  case SunrayEvent::WATCHDOG_ERROR:
    return "WATCHDOG_ERROR";
  case SunrayEvent::ENTER_POSITION_CONTROL:
    return "ENTER_POSITION_CONTROL";
  case SunrayEvent::ENTER_VELOCITY_CONTROL:
    return "ENTER_VELOCITY_CONTROL";
  case SunrayEvent::ENTER_ATTITUDE_CONTROL:
    return "ENTER_ATTITUDE_CONTROL";
  case SunrayEvent::ENTER_COMPLEX_CONTROL:
    return "ENTER_COMPLEX_CONTROL";
  case SunrayEvent::ENTER_TRAJECTORY_CONTROL:
    return "ENTER_TRAJECTORY_CONTROL";
  case SunrayEvent::POSITION_COMPLETED:
    return "POSITION_COMPLETED";
  case SunrayEvent::VELOCITY_COMPLETED:
    return "VELOCITY_COMPLETED";
  case SunrayEvent::ATTITUDE_COMPLETED:
    return "ATTITUDE_COMPLETED";
  case SunrayEvent::COMPLEX_COMPLETED:
    return "COMPLEX_COMPLETED";
  case SunrayEvent::TRAJECTORY_COMPLETED:
    return "TRAJECTORY_COMPLETED";
  default:
    return "UNKNOWN_EVENT";
  }
}

// 在起飞前/解锁前的安全检查
bool Sunray_StateMachine::check_health_preflight() {
  if (!sunray_controller_) {
    ROS_WARN(
        "[SunrayFSM] preflight check failed: controller is not registered");
    return false;
  }

  if (!validate_offstage_odometry_source()) {
    ROS_WARN("[SunrayFSM] preflight check failed: odometry source is invalid "
             "in OFF stage");
    return false;
  }

  return true;
}
// 安全检查
bool Sunray_StateMachine::check_health() {
  // 当前为占位检查：
  // 1) 起飞前由 check_health_preflight() 执行完整检查
  // 2) 飞行中后续可扩展里程计稳定性、控制器输出饱和等检查。
  if (fsm_current_state_ == SunrayState::OFF) {
    return true;
  }
  // 返回控制器是否已注册
  return sunray_controller_ != nullptr;
}

// 起飞前里程计检查，检查里程计存在，有效，未超时
bool Sunray_StateMachine::validate_offstage_odometry_source() const {
  if (!latest_external_odom_.isValid() ||
      latest_external_odom_.timestamp.isZero()) {
    ROS_WARN_THROTTLE(
        1.0, "[SunrayFSM] preflight blocked: no valid external odom injected");
    return false;
  }

  const double age_s =
      (ros::Time::now() - latest_external_odom_.timestamp).toSec();
  if (age_s < 0.0 || age_s > fsm_param_config_.timeout_odom_s) {
    ROS_WARN_THROTTLE(
        1.0,
        "[SunrayFSM] preflight blocked: external odom timeout age=%.3f s "
        "timeout=%.3f s",
        age_s, fsm_param_config_.timeout_odom_s);
    return false;
  }
  return true;
}

// 检查是否允许起飞
bool Sunray_StateMachine::can_takeoff() const {
  return validate_offstage_odometry_source() &&
         static_cast<bool>(sunray_controller_);
}

// 里程计状态转移底层函数
bool Sunray_StateMachine::transition_to(SunrayState next_state) {
  if (fsm_current_state_ == next_state) {
    return true;
  }

  if (fsm_current_state_ == SunrayState::OFF &&
      next_state == SunrayState::TAKEOFF && !check_health_preflight()) {
    ROS_WARN("[SunrayFSM] transition blocked: OFF -> TAKEOFF preflight check "
             "failed");
    return false;
  }

  ROS_INFO("[SunrayFSM] transition: %s -> %s", to_string(fsm_current_state_),
           to_string(next_state));
  fsm_current_state_ = next_state;
  return true;
}
// 获取当前控制器指针
std::shared_ptr<uav_control::Base_Controller>
Sunray_StateMachine::get_controller() const {
  return sunray_controller_;
}

} // namespace sunray_fsm
