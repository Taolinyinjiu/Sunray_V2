#include "utils/quintic_curve.hpp"

#include <algorithm>
#include <cmath>
#include "Eigen/src/Core/Matrix.h"

namespace uav_control {

Curve_Output get_quintic_curve(const Eigen::Vector3d& start_point,
                               const Eigen::Vector3d& end_point,
                               double start_time, double keep_time,
                               double current_time) {
  Curve_Output temp_output;
  temp_output.position = start_point;
  temp_output.velocity.setZero();
  temp_output.acceleration.setZero();
  // 如果运动持续时间小于等于零，或者当前时间小于零，说明传入参数有问题，拒绝生成
  if (keep_time <= 0.0 || keep_time < 0) {
    temp_output.curve_status = false;
    temp_output.position = start_point;
    return temp_output;
  }
  // 使用当前时间减去起始时间，再除以持续时间，计算进度
  const double progress_raw = (current_time - start_time) / keep_time;
  const double progress = std::max(0.0, std::min(1.0, progress_raw));

  const double p2 = progress * progress;
  const double p3 = p2 * progress;
  const double p4 = p3 * progress;
  const double p5 = p4 * progress;

  // Quintic smoothstep with zero velocity/acceleration at start/end.
  const double blend = 6.0 * p5 - 15.0 * p4 + 10.0 * p3;
  const double d_blend = 30.0 * p4 - 60.0 * p3 + 30.0 * p2;
  const double dd_blend = 120.0 * p3 - 180.0 * p2 + 60.0 * progress;

  const Eigen::Vector3d delta = end_point - start_point;
  temp_output.position = start_point + blend * delta;

  const double inv_keep_time = 1.0 / keep_time;
  const double inv_keep_time2 = inv_keep_time * inv_keep_time;
  temp_output.velocity = d_blend * inv_keep_time * delta;
  temp_output.acceleration = dd_blend * inv_keep_time2 * delta;
  temp_output.curve_status = true;
  return temp_output;
}

void Quintic_Curve::set_start_position(Eigen::Vector3d point_position) {
  start_position_ = point_position;
}

void Quintic_Curve::set_end_position(Eigen::Vector3d point_position) {
  end_position_ = point_position;
}

void Quintic_Curve::set_keep_time(double keep_time) { keep_time_ = keep_time; }

bool Quintic_Curve::set_start_time(ros::Time start_time) {
  if (start_time_ != 0) {
    return false;
  } else {
    start_time_ = start_time.toSec();
		log_start_time_ = start_time;
  }
  return true;
}

void Quintic_Curve::clear_time() {
  keep_time_ = 0.0;
  start_time_ = 0.0;
	log_start_time_ = ros::Time(0);
}

void Quintic_Curve::clear_position() {
  start_position_ = Eigen::Vector3d::Zero();
  end_position_ = Eigen::Vector3d::Zero();
}

void Quintic_Curve::clear_all() {
  clear_time();
  clear_position();
}

ros::Time Quintic_Curve::get_start_time() {
	return log_start_time_;
}

double Quintic_Curve::get_keep_time(){
	return keep_time_;
}

Eigen::Vector3d Quintic_Curve::get_start_position(){
	return start_position_;
}
Eigen::Vector3d Quintic_Curve::get_end_position(){
	return end_position_;
}
Eigen::Vector3d Quintic_Curve::get_position() { return position_; }

Eigen::Vector3d Quintic_Curve::get_velocity() { return velocity_; }

Eigen::Vector3d Quintic_Curve::get_acceleration() { return acceleration_; }

bool Quintic_Curve::generate_by_current_time(ros::Time current_time) {
  // 首先检查时间戳
  if (current_time.isZero()) {
    return false;
  } else if (current_time.toSec() < start_time_) {
    // 当前时间戳小于起点时间戳，输出起点位置
    position_ = start_position_;
    velocity_.setZero();
    acceleration_.setZero();
    return false;
  } else if (current_time.toSec() > (start_time_ + keep_time_)) {
    // 当前时间戳大于起始时间+持续时间,输出终点位置
    position_ = end_position_;
    velocity_.setZero();
    acceleration_.setZero();
    return false;
  }
  // 时间戳正确，开始归一化,使用当前时间减去起始时间，再除以持续时间，计算进度
  const double progress = (current_time.toSec() - start_time_) / keep_time_;
  // 根据参数计算
  const double p2 = progress * progress;
  const double p3 = p2 * progress;
  const double p4 = p3 * progress;
  const double p5 = p4 * progress;

  // 基于起点和终点速度与加速度均为零得到的参数，计算五此项曲线
  const double blend = 6.0 * p5 - 15.0 * p4 + 10.0 * p3;
  const double d_blend = 30.0 * p4 - 60.0 * p3 + 30.0 * p2;
  const double dd_blend = 120.0 * p3 - 180.0 * p2 + 60.0 * progress;

  const Eigen::Vector3d delta = end_position_ - start_position_;
  position_ = start_position_ + blend * delta;

  const double inv_keep_time = 1.0 / keep_time_;
  const double inv_keep_time2 = inv_keep_time * inv_keep_time;
  velocity_ = d_blend * inv_keep_time * delta;
  acceleration_ = dd_blend * inv_keep_time2 * delta;
  return true;
}

bool Quintic_Curve::generate_land_curve(ros::Time current_time) {
  if (current_time.isZero() || keep_time_ <= 0.0 || start_time_ <= 0.0) {
    position_ = start_position_;
    velocity_.setZero();
    acceleration_.setZero();
    return false;
  }

  const double current_time_s = current_time.toSec();
  if (current_time_s < start_time_) {
    position_ = start_position_;
    velocity_.setZero();
    acceleration_.setZero();
    return false;
  }
  if (current_time_s > (start_time_ + keep_time_)) {
    position_ = end_position_;
    velocity_.setZero();
    acceleration_.setZero();
    return false;
  }

  const double tau =
      std::max(0.0, std::min(1.0, (current_time_s - start_time_) / keep_time_));
  // gamma=0.750668... 对应速度峰值约在 tau=0.35
  constexpr double kGamma = 0.750668845430905;
  const Eigen::Vector3d delta = end_position_ - start_position_;

  if (tau <= 1e-9) {
    position_ = start_position_;
    velocity_.setZero();
    acceleration_.setZero();
    return true;
  }

  const double u = std::pow(tau, kGamma);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;

  const double blend = 6.0 * u5 - 15.0 * u4 + 10.0 * u3;
  const double d_blend = 30.0 * u4 - 60.0 * u3 + 30.0 * u2;
  const double dd_blend = 120.0 * u3 - 180.0 * u2 + 60.0 * u;

  const double inv_keep_time = 1.0 / keep_time_;
  const double inv_keep_time2 = inv_keep_time * inv_keep_time;
  const double du_dt = kGamma * std::pow(tau, kGamma - 1.0) * inv_keep_time;
  const double d2u_dt2 =
      kGamma * (kGamma - 1.0) * std::pow(tau, kGamma - 2.0) * inv_keep_time2;

  position_ = start_position_ + blend * delta;
  velocity_ = d_blend * du_dt * delta;
  acceleration_ = (dd_blend * du_dt * du_dt + d_blend * d2u_dt2) * delta;
  return true;
}

}  // namespace uav_control
