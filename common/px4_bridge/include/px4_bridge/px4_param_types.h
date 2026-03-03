/**
 * @file px4_param_types.h
 * @brief PX4 参数类型定义（EKF2 与控制器参数语义封装）。
 *
 * 本文件定义参数位掩码、单选值与浮点参数封装结构体，
 * 统一参数配置调用风格并降低“魔法数字”使用。
 */

#pragma once

#include <cstdint>
#include <string>

namespace px4_param_types {

/**
 * @brief PX4 参数类型命名空间。
 */

/**
 * @brief EKF2 参数类型快速使用说明。
 *
 * 使用流程：
 * 1. 选择对应参数结构体并通过语义化接口赋值；
 * 2. 位掩码类参数使用 enable/disable 组合；
 * 3. 单值类参数使用 set_xxx 或 set_ms；
 * 4. 通过 toInt()/toFloat() 取出最终写入值。
 *
 * EV 模块示例：
 * @code
 * px4_param_types::EKF2_EV_CTRL ev_ctrl;
 * ev_ctrl.enable_Horizontalposition();
 * ev_ctrl.enable_Yaw();
 * int ev_ctrl_raw = ev_ctrl.toInt();
 *
 * px4_param_types::EKF2_HGT_REF hgt_ref;
 * hgt_ref.set_Gnss();
 * int hgt_ref_raw = hgt_ref.toInt();
 *
 * px4_param_types::EKF2_EV_DELAY ev_delay;
 * ev_delay.set_ms(10.0f);
 * float ev_delay_raw = ev_delay.toFloat();
 * @endcode
 *
 * MAG 模块示例：
 * @code
 * px4_param_types::EKF2_MAG_TYPE mag_type;
 * mag_type.set_Yaw();
 * int mag_type_raw = mag_type.toInt();
 *
 * px4_param_types::EKF2_MAG_CHECK mag_check;
 * mag_check.enable_Strength();
 * mag_check.enable_Inclination();
 * int mag_check_raw = mag_check.toInt();
 * @endcode
 *
 * GPS 模块示例：
 * @code
 * px4_param_types::EKF2_GPS_CTRL gps_ctrl;
 * gps_ctrl.enable_Lon_Lat();
 * gps_ctrl.enable_Velocity3D();
 * int gps_ctrl_raw = gps_ctrl.toInt();
 *
 * px4_param_types::EKF2_GPS_CHECK gps_check;
 * gps_check.enable_Min_sat_count();
 * gps_check.enable_Max_PDOP();
 * int gps_check_raw = gps_check.toInt();
 *
 * px4_param_types::EKF2_GPS_DELAY gps_delay;
 * gps_delay.set_ms(110.0f);
 * float gps_delay_raw = gps_delay.toFloat();
 * @endcode
 */

/**
 * @brief 通用 float 参数基类。
 *
 * 目的：
 * - 让“单值 float 参数”也能用结构体 + 方法的方式表达；
 * - 复用 set/get/toFloat 接口，避免业务层散落裸 float。
 */
struct FloatParamBase {
  float value = 0.0f;

  void set(float v) { value = v; }
  float get() const { return value; }
  float toFloat() const { return value; }
};

// ==================== EV Module ====================

/**
 * @brief EKF2_HGT_REF 单项选择值封装（EV 模块，TYPE 参数）。
 *
 * 目的：
 * - 避免业务代码直接写“魔法数字”；
 * - 提供可读的 set/is 接口；
 * - 需要写入参数时可通过 toInt() 得到整型值。
 *
 * 说明：
 * - EKF2_HGT_REF 只能在候选项中选择一个，不支持按位叠加。
 *
 * 用法示例：
 * @code
 * px4_param_types::EKF2_HGT_REF param;
 * param.set_Gnss();
 * int v = param.toInt();  // 可直接用于 EKF2_HGT_REF
 * @endcode
 */
struct EKF2_HGT_REF {
  enum Value : int {
    kBaro = 0,  // 气压计
    kGnss = 1,  // GPS
    kRange = 2, // 测距仪
    kVision = 3 // 外部里程计
  };

  Value value = kBaro;

  void set_Baro() { value = kBaro; }
  void set_Gnss() { value = kGnss; }
  void set_Range() { value = kRange; }
  void set_Vision() { value = kVision; }

  bool is_Baro() const { return value == kBaro; }
  bool is_Gnss() const { return value == kGnss; }
  bool is_Range() const { return value == kRange; }
  bool is_Vision() const { return value == kVision; }

  void setRaw(int raw) {
    switch (raw) {
    case kBaro:
      value = kBaro;
      break;
    case kGnss:
      value = kGnss;
      break;
    case kRange:
      value = kRange;
      break;
    case kVision:
      value = kVision;
      break;
    default:
      value = kBaro;
      break;
    }
  }

  int toInt() const { return static_cast<int>(value); }
};

/**
 * @brief EKF2_EV_CTRL 位掩码封装（EV 模块，CTRL 参数）。
 *
 * 目的：
 * - 避免业务代码直接写“魔法数字”；
 * - 提供可读的 enable/disable 接口；
 * - 需要写入参数时可通过 toInt() 得到整型值。
 *
 * 用法示例：
 * @code
 * px4_param_types::EKF2_EV_CTRL param;
 * param.enable_Horizontalposition();
 * param.enable_Yaw();
 * int v = param.toInt();  // 可直接用于 EKF2_EV_CTRL
 * @endcode
 */
struct EKF2_EV_CTRL {
  // 位定义（按 PX4 EKF2_EV_CTRL 的常见约定）
  static constexpr uint32_t kHorizontalPosition = 1u << 0;
  static constexpr uint32_t kVerticalPosition = 1u << 1;
  static constexpr uint32_t kVelocity = 1u << 2;
  static constexpr uint32_t kYaw = 1u << 3;

  uint32_t mask = 0u;

  void clear() { mask = 0u; }

  void enable_Horizontalposition() { mask |= kHorizontalPosition; }
  void disable_Horizontalposition() { mask &= ~kHorizontalPosition; }
  bool has_Horizontalposition() const {
    return (mask & kHorizontalPosition) != 0u;
  }

  void enable_Verticalposition() { mask |= kVerticalPosition; }
  void disable_Verticalposition() { mask &= ~kVerticalPosition; }
  bool has_Verticalposition() const { return (mask & kVerticalPosition) != 0u; }

  void enable_Velocity() { mask |= kVelocity; }
  void disable_Velocity() { mask &= ~kVelocity; }
  bool has_Velocity() const { return (mask & kVelocity) != 0u; }

  void enable_Yaw() { mask |= kYaw; }
  void disable_Yaw() { mask &= ~kYaw; }
  bool has_Yaw() const { return (mask & kYaw) != 0u; }

  int toInt() const { return static_cast<int>(mask); }
};

/**
 * @brief EKF2_EV_DELAY 参数封装（EV 模块，float 参数，单位：毫秒）。
 *
 * 目的：
 * - 让单值 float 参数也能通过结构体表达语义；
 * - 通过 toFloat() 输出最终写入值。
 *
 * 用法示例：
 * @code
 * px4_param_types::EKF2_EV_DELAY param;
 * param.set_ms(10.0f);
 * float v = param.toFloat();  // 可直接用于 EKF2_EV_DELAY
 * @endcode
 */
struct EKF2_EV_DELAY : public FloatParamBase {
  void set_ms(float ms) { set(ms); }
  float milliseconds() const { return get(); }

  static EKF2_EV_DELAY fromFloat(float ms) {
    EKF2_EV_DELAY out;
    out.set_ms(ms);
    return out;
  }
};

// ==================== MAG Module ====================

/**
 * @brief EKF2_MAG_TYPE 单项选择值封装（MAG 模块，TYPE 参数）。
 *
 * 目的：
 * - 避免业务代码直接写“魔法数字”；
 * - 提供可读的 set/is 接口；
 * - 需要写入参数时可通过 toInt() 得到整型值。
 *
 * 用法示例：
 * @code
 * px4_param_types::EKF2_MAG_TYPE param;
 * param.set_Yaw();
 * int v = param.toInt();  // 可直接用于 EKF2_MAG_TYPE
 * @endcode
 */
struct EKF2_MAG_TYPE {
  enum Value : int {
    kAuto = 0,    // PX4 自动决定如何融合 MAG 数据
    kMag_Yaw = 1, // 仅使用 MAG 数据融合到 Yaw 角
    kNone = 5     // 不使用 MAG Yaw 融合
  };

  Value value = kAuto;

  void set_Auto() { value = kAuto; }
  void set_Yaw() { value = kMag_Yaw; }
  void set_None() { value = kNone; }

  bool is_Auto() const { return value == kAuto; }
  bool is_Yaw() const { return value == kMag_Yaw; }
  bool is_None() const { return value == kNone; }

  void setRaw(int raw) {
    switch (raw) {
    case kAuto:
      value = kAuto;
      break;
    case kMag_Yaw:
      value = kMag_Yaw;
      break;
    case kNone:
      value = kNone;
      break;
    default:
      value = kAuto;
      break;
    }
  }

  int toInt() const { return static_cast<int>(value); }
};

/**
 * @brief EKF2_MAG_CHECK 位掩码封装（MAG 模块）。
 *
 * 目的：
 * - 通过位掩码控制磁力计一致性检查项；
 * - 提供可读的 enable/disable/has 接口。
 */
struct EKF2_MAG_CHECK {
  // 位定义（按 EKF2_MAG_CHECK 的常见约定）
  static constexpr uint32_t kStrength = 1u << 0;
  static constexpr uint32_t kInclination = 1u << 1;
  static constexpr uint32_t kWait2WMM = 1u << 2;

  uint32_t mask = 0u;

  void clear() { mask = 0u; }

  void enable_Strength() { mask |= kStrength; }
  void disable_Strength() { mask &= ~kStrength; }
  bool has_Strength() const { return (mask & kStrength) != 0u; }

  void enable_Inclination() { mask |= kInclination; }
  void disable_Inclination() { mask &= ~kInclination; }
  bool has_Inclination() const { return (mask & kInclination) != 0u; }

  void enable_Wait2WMM() { mask |= kWait2WMM; }
  void disable_Wait2WMM() { mask &= ~kWait2WMM; }
  bool has_Wait2WMM() const { return (mask & kWait2WMM) != 0u; }

  // 将位合并输出为 int 类型，用于设置参数过程
  int toInt() const { return static_cast<int>(mask); }
};

// ==================== GPS Module ====================

/**
 * @brief EKF2_GPS_CTRL 位掩码封装（GPS 模块，CTRL 参数）。
 *
 * 目的：
 * - 控制 GPS 融合维度开关；
 * - 提供可读的 enable/disable/has 接口。
 */
struct EKF2_GPS_CTRL {
  // 位定义（按 EKF2_GPS_CTRL 的常见约定）
  static constexpr uint32_t kLon_Lat = 1u << 0;
  static constexpr uint32_t kAltitude = 1u << 1;
  static constexpr uint32_t kVelocity3D = 1u << 2;
  static constexpr uint32_t kDual_AntennaHeading = 1u << 3;

  uint32_t mask = 0u;

  void clear() { mask = 0u; }

  void enable_Lon_Lat() { mask |= kLon_Lat; }
  void disable_Lon_Lat() { mask &= ~kLon_Lat; }
  bool has_Lon_Lat() const { return (mask & kLon_Lat) != 0u; }

  void enable_Altitude() { mask |= kAltitude; }
  void disable_Altitude() { mask &= ~kAltitude; }
  bool has_Altitude() const { return (mask & kAltitude) != 0u; }

  void enable_Velocity3D() { mask |= kVelocity3D; }
  void disable_Velocity3D() { mask &= ~kVelocity3D; }
  bool has_Velocity3D() const { return (mask & kVelocity3D) != 0u; }

  void enable_Dual_AntennaHeading() { mask |= kDual_AntennaHeading; }
  void disable_Dual_AntennaHeading() { mask &= ~kDual_AntennaHeading; }
  bool has_Dual_AntennaHeading() const {
    return (mask & kDual_AntennaHeading) != 0u;
  }

  // 将位合并输出为 int 类型，用于设置参数过程
  int toInt() const { return static_cast<int>(mask); }
};

/**
 * @brief EKF2_GPS_CHECK 位掩码封装（GPS 模块）。
 *
 * 目的：
 * - 通过位掩码控制 GPS 质量检查项；
 * - 提供可读的 enable/disable/has 接口。
 */
struct EKF2_GPS_CHECK {
  // 位定义（按 EKF2_GPS_CHECK 的常见约定）
  static constexpr uint32_t kMin_sat_count = 1u << 0;
  static constexpr uint32_t kMax_PDOP = 1u << 1;
  static constexpr uint32_t kMax_HorizontalPosition_Error = 1u << 2;
  static constexpr uint32_t kMax_VerticalPosition_Error = 1u << 3;
  static constexpr uint32_t kMax_Speed_Error = 1u << 4;
  static constexpr uint32_t kMax_HorizontalPosition_Rate = 1u << 5;
  static constexpr uint32_t kMax_VerticalPosition_Rate = 1u << 6;
  static constexpr uint32_t kMax_HorizontalSpeed = 1u << 7;
  static constexpr uint32_t kMax_VerticalVelocity_Discrepancy = 1u << 8;

  uint32_t mask = 0u;

  void clear() { mask = 0u; }

  void enable_Min_sat_count() { mask |= kMin_sat_count; }
  void disable_Min_sat_count() { mask &= ~kMin_sat_count; }
  bool has_Min_sat_count() const { return (mask & kMin_sat_count) != 0u; }

  void enable_Max_PDOP() { mask |= kMax_PDOP; }
  void disable_Max_PDOP() { mask &= ~kMax_PDOP; }
  bool has_Max_PDOP() const { return (mask & kMax_PDOP) != 0u; }

  void enable_Max_HorizontalPosition_Error() {
    mask |= kMax_HorizontalPosition_Error;
  }
  void disable_Max_HorizontalPosition_Error() {
    mask &= ~kMax_HorizontalPosition_Error;
  }
  bool has_Max_HorizontalPosition_Error() const {
    return (mask & kMax_HorizontalPosition_Error) != 0u;
  }

  void enable_Max_VerticalPosition_Error() {
    mask |= kMax_VerticalPosition_Error;
  }
  void disable_Max_VerticalPosition_Error() {
    mask &= ~kMax_VerticalPosition_Error;
  }
  bool has_Max_VerticalPosition_Error() const {
    return (mask & kMax_VerticalPosition_Error) != 0u;
  }

  void enable_Max_Speed_Error() { mask |= kMax_Speed_Error; }
  void disable_Max_Speed_Error() { mask &= ~kMax_Speed_Error; }
  bool has_Max_Speed_Error() const { return (mask & kMax_Speed_Error) != 0u; }

  void enable_Max_HorizontalPosition_Rate() {
    mask |= kMax_HorizontalPosition_Rate;
  }
  void disable_Max_HorizontalPosition_Rate() {
    mask &= ~kMax_HorizontalPosition_Rate;
  }
  bool has_Max_HorizontalPosition_Rate() const {
    return (mask & kMax_HorizontalPosition_Rate) != 0u;
  }

  void enable_Max_VerticalPosition_Rate() {
    mask |= kMax_VerticalPosition_Rate;
  }
  void disable_Max_VerticalPosition_Rate() {
    mask &= ~kMax_VerticalPosition_Rate;
  }
  bool has_Max_VerticalPosition_Rate() const {
    return (mask & kMax_VerticalPosition_Rate) != 0u;
  }

  void enable_Max_HorizontalSpeed() { mask |= kMax_HorizontalSpeed; }
  void disable_Max_HorizontalSpeed() { mask &= ~kMax_HorizontalSpeed; }
  bool has_Max_HorizontalSpeed() const {
    return (mask & kMax_HorizontalSpeed) != 0u;
  }

  void enable_Max_VerticalVelocity_Discrepancy() {
    mask |= kMax_VerticalVelocity_Discrepancy;
  }
  void disable_Max_VerticalVelocity_Discrepancy() {
    mask &= ~kMax_VerticalVelocity_Discrepancy;
  }
  bool has_Max_VerticalVelocity_Discrepancy() const {
    return (mask & kMax_VerticalVelocity_Discrepancy) != 0u;
  }

  // 将位合并输出为 int 类型，用于设置参数过程
  int toInt() const { return static_cast<int>(mask); }
};

/**
 * @brief EKF2_GPS_DELAY 参数封装（GPS 模块，float 参数，单位：毫秒）。
 *
 * 目的：
 * - 让单值 float 参数也能通过结构体表达语义；
 * - 通过 toFloat() 输出最终写入值。
 *
 * 用法示例：
 * @code
 * px4_param_types::EKF2_GPS_DELAY param;
 * param.set_ms(110.0f);
 * float v = param.toFloat();  // 可直接用于 EKF2_GPS_DELAY
 * @endcode
 */
struct EKF2_GPS_DELAY : public FloatParamBase {
  void set_ms(float ms) { set(ms); }
  float milliseconds() const { return get(); }

  static EKF2_GPS_DELAY fromFloat(float ms) {
    EKF2_GPS_DELAY out;
    out.set_ms(ms);
    return out;
  }
};

// ==================== PX4 Controller Module ====================

/**
 * @brief PX4 控制器单个增益参数封装（float）。
 *
 * 目的：
 * - 统一 P/I/D 单个增益值的表达；
 * - 提供 should_update 标记，便于参数管理器按需下发。
 */
struct RATE_GAIN {
  float value = 0.0f;
  bool should_update = false;

  void clear_update() { should_update = false; }
  void set_gain(float v) {
    value = v;
    should_update = true;
  }
  float toFloat() const { return value; }
};

/**
 * @brief 单轴角速度 PID 参数封装（ROLL 或 PITCH 或 YAW）。
 *
 * 目的：
 * - 将单轴的 P/I/D 三个增益聚合在一起；
 * - 提供语义化 set 接口，保持调用风格一致。
 */
struct RATE_AXIS_PID {
  RATE_GAIN p;
  RATE_GAIN i;
  RATE_GAIN d;

  void set_P(float v) { p.set_gain(v); }
  void set_I(float v) { i.set_gain(v); }
  void set_D(float v) { d.set_gain(v); }
};

/**
 * @brief PX4 原生角速度环 PID 参数封装（ROLL/PITCH/YAW）。
 *
 * 目的：
 * - 用统一结构表达 3 轴 * 3 项（P/I/D）参数；
 * - 提供轴级别语义化接口，避免业务层直接散写浮点数。
 *
 * 用法示例：
 * @code
 * px4_param_types::PX4_RATE_PID param;
 * param.set_Roll_P(0.12f);
 * param.set_Roll_I(0.08f);
 * param.set_Roll_D(0.002f);
 * param.set_Pitch_P(0.12f);
 * param.set_Yaw_D(0.001f);
 * @endcode
 */
struct PX4_RATE_PID {
  RATE_AXIS_PID roll;
  RATE_AXIS_PID pitch;
  RATE_AXIS_PID yaw;

  void set_Roll_P(float v) { roll.set_P(v); }
  void set_Roll_I(float v) { roll.set_I(v); }
  void set_Roll_D(float v) { roll.set_D(v); }

  void set_Pitch_P(float v) { pitch.set_P(v); }
  void set_Pitch_I(float v) { pitch.set_I(v); }
  void set_Pitch_D(float v) { pitch.set_D(v); }

  void set_Yaw_P(float v) { yaw.set_P(v); }
  void set_Yaw_I(float v) { yaw.set_I(v); }
  void set_Yaw_D(float v) { yaw.set_D(v); }
};

/**
 * @brief 单轴速度环加速度 PID 参数封装（XY 合并轴或 Z 轴）。
 *
 * 目的：
 * - 适配 PX4 速度环参数命名（*_VEL_*_ACC）；
 * - 提供与参数名一致的语义化接口。
 */
struct VELOCITY_ACC_PID_AXIS {
  RATE_GAIN p_acc;
  RATE_GAIN i_acc;
  RATE_GAIN d_acc;

  void set_P_ACC(float v) { p_acc.set_gain(v); }
  void set_I_ACC(float v) { i_acc.set_gain(v); }
  void set_D_ACC(float v) { d_acc.set_gain(v); }
};

/**
 * @brief PX4 速度环参数封装（XY 合并，Z 独立）。
 *
 * 说明：
 * - 常见映射：
 *   XY: MPC_XY_VEL_P_ACC / MPC_XY_VEL_I_ACC / MPC_XY_VEL_D_ACC
 *   Z : MPC_Z_VEL_P_ACC  / MPC_Z_VEL_I_ACC  / MPC_Z_VEL_D_ACC
 *
 * 用法示例：
 * @code
 * px4_param_types::PX4_VELOCITY_PID param;
 * param.set_XY_P_ACC(1.2f);
 * param.set_XY_I_ACC(0.4f);
 * param.set_Z_P_ACC(2.0f);
 * @endcode
 */
struct PX4_VELOCITY_PID {
  VELOCITY_ACC_PID_AXIS xy;
  VELOCITY_ACC_PID_AXIS z;

  void set_XY_P_ACC(float v) { xy.set_P_ACC(v); }
  void set_XY_I_ACC(float v) { xy.set_I_ACC(v); }
  void set_XY_D_ACC(float v) { xy.set_D_ACC(v); }

  void set_Z_P_ACC(float v) { z.set_P_ACC(v); }
  void set_Z_I_ACC(float v) { z.set_I_ACC(v); }
  void set_Z_D_ACC(float v) { z.set_D_ACC(v); }
};

/**
 * @brief PX4 位置环参数封装（仅 P，XY 合并，Z 独立）。
 *
 * 说明：
 * - 常见映射：
 *   XY: MPC_XY_P
 *   Z : MPC_Z_P
 *
 * 用法示例：
 * @code
 * px4_param_types::PX4_POSITION_P param;
 * param.set_XY_P(0.95f);
 * param.set_Z_P(1.20f);
 * @endcode
 */
struct PX4_POSITION_P {
  RATE_GAIN xy_p;
  RATE_GAIN z_p;

  void set_XY_P(float v) { xy_p.set_gain(v); }
  void set_Z_P(float v) { z_p.set_gain(v); }
};

} // namespace px4_param_types
