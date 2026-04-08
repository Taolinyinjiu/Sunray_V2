#include "controller/core_algorithm/linear_attitude_control.hpp"

// 构造函数，从使用Linear_AttitudeControl_Param_t构造函数从传入的结构体中获取值

void Linear_AttitudeControl::load_param(Linear_AttitudeControl_Param_t& param) {
    // 从入口参数中加载参数
    param_.gravity = param.gravity;
    param_.output_type = param.output_type;
    param_.accurate_thrust_model = param.accurate_thrust_model;
    param_.hover_percentage = param.hover_percentage;
    param_.pos_gain = param.pos_gain;
    param_.vel_gain = param.vel_gain;
    resetThrustMapping();  // 重置推力映射
}

// 对控制器进行如下修改
// 1. pos修改为optional项，当没有输入位置时，使用纯速度控制 -> rc_control
Linear_AttitudeControl_Output_t Linear_AttitudeControl::calculateControl(
    const controller_data_types::TargetTrajectoryPoint_t des_state,
    const control_common::UAVStateEstimate& current_odom,
    const control_common::Mavros_IMU& imu) {
    Linear_AttitudeControl_Output_t output;
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
    Eigen::Vector3d Kp, Kv;
    Kp << param_.pos_gain.Kx, param_.pos_gain.Ky, param_.pos_gain.Kz;
    Kv << param_.vel_gain.Kx, param_.vel_gain.Ky, param_.vel_gain.Kz;
    des_acc = des_state.acceleration +
              Kv.asDiagonal() * (des_state.velocity - current_odom.velocity) +
              Kp.asDiagonal() * (des_state.position - current_odom.position);
    std::cout << "----------------------" << std::endl;
    std::cout << " directly des_acc:" << des_acc.z() << std::endl;
    des_acc += Eigen::Vector3d(0, 0, param_.gravity);
    std::cout << " gravity des_acc:" << des_acc.z() << std::endl;
    output.thrust = computeDesiredCollectiveThrustSignal(des_acc);  // 从加速度映射推力
    std::cout << " acc to thrust :" << output.thrust << std::endl;
    double roll, pitch;
    double yaw_imu = fromQuaternion2yaw(imu.orientation);
    double yaw_odom = fromQuaternion2yaw(current_odom.orientation);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gravity;
    pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gravity;
    Eigen::Quaterniond q = Eigen::AngleAxisd(des_state.yaw.value(), Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    output.orientation = imu.orientation * current_odom.orientation.inverse() * q;
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), output.thrust));
    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }
    return output;
}

double clamp_symmetric(double value, double limit_abs) {
    const double limit = std::abs(limit_abs);
    return std::max(-limit, std::min(limit, value));
}

bool is_finite_quaternion(const Eigen::Quaterniond& q) {
    return std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) &&
           std::isfinite(q.z());
}

Eigen::Quaterniond normalized_or_identity(const Eigen::Quaterniond& q) {
    if (!is_finite_quaternion(q)) {
        return Eigen::Quaterniond::Identity();
    }
    Eigen::Quaterniond normalized = q;
    if (normalized.norm() > 1e-9) {
        normalized.normalize();
        return normalized;
    }
    return Eigen::Quaterniond::Identity();
}

double yaw_from_quaternion(const Eigen::Quaterniond& q) {
    const Eigen::Quaterniond normalized = normalized_or_identity(q);
    const double siny_cosp =
        2.0 * (normalized.w() * normalized.z() + normalized.x() * normalized.y());
    const double cosy_cosp =
        1.0 - 2.0 * (normalized.y() * normalized.y() + normalized.z() * normalized.z());
    return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Quaterniond quaternion_from_force_yaw(const Eigen::Vector3d& force_world,
                                             double yaw,
                                             const Eigen::Quaterniond& fallback_attitude) {
    if (!force_world.allFinite() || force_world.norm() < 1e-6) {
        return normalized_or_identity(fallback_attitude);
    }

    Eigen::Vector3d b3_des = force_world.normalized();
    Eigen::Vector3d b1_ref(std::cos(yaw), std::sin(yaw), 0.0);
    if (b1_ref.norm() < 1e-6) {
        b1_ref = Eigen::Vector3d::UnitX();
    }

    Eigen::Vector3d b2_des = b3_des.cross(b1_ref);
    if (b2_des.norm() < 1e-6) {
        const Eigen::Matrix3d fallback_rotation =
            normalized_or_identity(fallback_attitude).toRotationMatrix();
        b2_des = b3_des.cross(fallback_rotation.col(0));
        if (b2_des.norm() < 1e-6) {
            b2_des = b3_des.cross(Eigen::Vector3d::UnitX());
        }
        if (b2_des.norm() < 1e-6) {
            b2_des = b3_des.cross(Eigen::Vector3d::UnitY());
        }
    }
    b2_des.normalize();

    Eigen::Vector3d b1_des = b2_des.cross(b3_des);
    if (b1_des.norm() < 1e-6) {
        return normalized_or_identity(fallback_attitude);
    }
    b1_des.normalize();

    Eigen::Matrix3d desired_rotation;
    desired_rotation.col(0) = b1_des;
    desired_rotation.col(1) = b2_des;
    desired_rotation.col(2) = b3_des;
    return normalized_or_identity(Eigen::Quaterniond(desired_rotation));
}

Linear_AttitudeControl_Output_t Linear_AttitudeControl::calculateControl(
    const controller_data_types::TargetTrajectoryPoint_t des_state,
    const control_common::UAVStateEstimate& current_odom,
    const control_common::Mavros_IMU& imu,
    bool enable_integral) {
    Linear_AttitudeControl_Output_t output;
    // 计算误差
    Eigen::Vector3d pos_error = des_state.position - current_odom.position;
    Eigen::Vector3d vel_error = des_state.velocity - current_odom.velocity;
    // 对误差进行限幅
    for (uint8_t i = 0; i < 3; i++) {
        if (std::abs(pos_error[i]) > test_control_param_.max_pos_error) {
            pos_error[i] = clamp_symmetric(pos_error[i], test_control_param_.max_pos_error);
        }
        if (std::abs(vel_error[i]) > test_control_param_.max_vel_error) {
            vel_error[i] = clamp_symmetric(vel_error[i], test_control_param_.max_vel_error);
        }
    }
    // 检查是否允许使用积分项
    for (uint8_t i = 0; i < 2; i++) {
        if (enable_integral &&
            std::abs(pos_error[i]) < test_control_param_.position_integral_start_error_xy_m) {
            test_control_param_.int_e_v_[i] += pos_error[i] / test_control_param_.controller_hz;
            test_control_param_.int_e_v_[i] =
                clamp_symmetric(test_control_param_.int_e_v_[i], test_control_param_.int_max[i]);
        } else {
            test_control_param_.int_e_v_[i] = 0.0;
        }
    }
    if (enable_integral &&
        std::abs(pos_error[2]) < test_control_param_.position_integral_start_error_z_m) {
        test_control_param_.int_e_v_[2] += pos_error[2] / test_control_param_.controller_hz;
        test_control_param_.int_e_v_[2] =
            clamp_symmetric(test_control_param_.int_e_v_[2], test_control_param_.int_max[2]);
    } else {
        test_control_param_.int_e_v_[2] = 0;
    }
    // 当输入期望状态有速度时，积分项清零
    if (std::abs(des_state.velocity.x()) > 1e-9 || std::abs(des_state.velocity.y()) > 1e-9 ||
        std::abs(des_state.velocity.z()) > 1e-9) {
        test_control_param_.int_e_v_[0] = 0.0;
        test_control_param_.int_e_v_[1] = 0.0;
    }
    const Eigen::Vector3d des_acc =
        des_state.acceleration + test_control_param_.Kp.cwiseProduct(pos_error) +
        test_control_param_.Kv.cwiseProduct(vel_error) +
        test_control_param_.Kvi.cwiseProduct(test_control_param_.int_e_v_);

    Eigen::Vector3d f_des = des_acc * test_control_param_.mass_kg;
    f_des.z() += test_control_param_.mass_kg * test_control_param_.gravity_mps2;

    const double nominal_weight = test_control_param_.mass_kg * test_control_param_.gravity_mps2;
    if (f_des.z() < 0.5 * nominal_weight && std::abs(f_des.z()) > 1e-6) {
        f_des = f_des / f_des.z() * (0.5 * nominal_weight);
    } else if (f_des.z() > 2.0 * nominal_weight && std::abs(f_des.z()) > 1e-6) {
        f_des = f_des / f_des.z() * (2.0 * nominal_weight);
    }

    const double tilt_tan = std::tan(test_control_param_.tilt_angle_max_rad);
    const double max_horizontal_force = std::abs(f_des.z()) * tilt_tan;
    const double horizontal_force_norm = f_des.head<2>().norm();
    if (horizontal_force_norm > max_horizontal_force && horizontal_force_norm > 1e-6) {
        const double scale = max_horizontal_force / horizontal_force_norm;
        f_des.x() *= scale;
        f_des.y() *= scale;
    }
    const double yaw = des_state.yaw.value();
    const Eigen::Quaterniond desired_attitude =
        quaternion_from_force_yaw(f_des, yaw, normalized_or_identity(current_odom.orientation));
    const Eigen::Matrix3d desired_rotation = desired_attitude.toRotationMatrix();
    const double u1 = std::max(0.0, f_des.dot(desired_rotation.col(2)));
    const double full_thrust = nominal_weight / std::max(1e-6, test_control_param_.hover_percent);
    output.orientation = desired_attitude;
    output.thrust =
        std::max(test_control_param_.min_command_thrust, std::min(1.0, u1 / full_thrust));
    return output;
}

/*
  compute throttle percentage
*/
double
Linear_AttitudeControl::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d& des_acc) {
    double z_acc = des_acc(2);

    // 1. 如果计算出的加速度连重力的一半都不到，说明控制器想急剧下降或已经撞地
    // 我们可以引入一个阈值（假设 param_.gravity 是 9.8）
    if (z_acc < 0.5 * 9.8) {
        // 此时我们可以给一个更小的比例，或者直接压低输出
        // 比如：让推力加速减小
        z_acc *= 0.5;
    }

    double throttle_percentage = z_acc / thr2acc_;

    // 2. 限制最小值（PX4上锁通常需要推力低于 0.1-0.15）
    // 如果你保底给 0.05，PX4 一定会上锁
    if (throttle_percentage < 0.05)
        throttle_percentage = 0.05;
    if (throttle_percentage > 1.0)
        throttle_percentage = 1.0;

    return throttle_percentage;
}

bool Linear_AttitudeControl::estimateThrustModel(const Eigen::Vector3d& est_a) {
    // 更新时间戳
    ros::Time t_now = ros::Time::now();
    // timed_thrust_是一个队列容器,.size()返回队列中元素的个数,如果队列中有元素则进入循环
    while (timed_thrust_.size() >= 1) {
        // 选择35~45ms之间的推力数据
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed = (t_now - t_t.first).toSec();
        // 45ms以上的数据太旧了,弹出队列继续选择
        if (time_passed > 0.045) {
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            continue;
        }
        // 小于35ms的数据太新了,直接return
        if (time_passed < 0.035)  // 35ms
        {
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();

        /***********************************/
        /* Model: est_a(2) = thr1acc_ * thr */
        /***********************************/
        double gamma = 1 / (rho2_ + thr * P_ * thr);
        double K = gamma * P_ * thr;
        thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
        P_ = (1 - K * thr) * P_ / rho2_;
        // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
        // fflush(stdout);

        // debug_msg_.thr2acc = thr2acc_;
        return true;
    }
    return false;
}

void Linear_AttitudeControl::resetThrustMapping(void) {
    thr2acc_ = param_.gravity / param_.hover_percentage;
    P_ = 1e6;
}

// 从四元数提取yaw角
double Linear_AttitudeControl::fromQuaternion2yaw(Eigen::Quaterniond q) {
    double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                       q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}
