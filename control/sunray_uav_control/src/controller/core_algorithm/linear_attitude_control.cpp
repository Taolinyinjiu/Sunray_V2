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

// 析构函数编译器自动生成
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
