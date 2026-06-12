#include "imu_process.h"

void ImuProcess::ProcessIMU(const ImuData& input_imu_data, ImuData& output_imu_data, double& dt) {

    static bool first_frame_flag = true;

    Eigen::Vector3d cur_imu_gyr = input_imu_data.cur_imu_gyr;
    Eigen::Vector3d cur_imu_acc = input_imu_data.cur_imu_acc;
    double cur_imu_timeStamp = input_imu_data.timeStamp;

    if (first_frame_flag) {

        last_imu_gyr_ = cur_imu_gyr;
        last_imu_acc_ = cur_imu_acc;
        last_imu_timeStamp_ = cur_imu_timeStamp;
        first_frame_flag = false;
        return;
    }

    dt = cur_imu_timeStamp - last_imu_timeStamp_;

    // 中值积分
    output_imu_data.timeStamp = cur_imu_timeStamp;
    output_imu_data.cur_imu_gyr = 0.5 * (cur_imu_gyr + last_imu_gyr_);
    output_imu_data.cur_imu_acc = 0.5 * (cur_imu_acc + last_imu_acc_);
    output_imu_data.cur_imu_acc = output_imu_data.cur_imu_acc * G_m_s2 / mean_imu_acc_.norm();  //通过重力数值对加速度进行调整

    last_imu_gyr_ = cur_imu_gyr;
    last_imu_acc_ = cur_imu_acc;
    last_imu_timeStamp_ = cur_imu_timeStamp;
}

bool ImuProcess::ImuInit(const ImuData& input_imu_data) {

    static int iter_num = 0;

    Eigen::Vector3d cur_imu_gyr = input_imu_data.cur_imu_gyr;
    Eigen::Vector3d cur_imu_acc = input_imu_data.cur_imu_acc;
    double cur_imu_timeStamp = input_imu_data.timeStamp;

    ++iter_num;

    mean_imu_acc_ += (cur_imu_acc - mean_imu_acc_) / iter_num;
    mean_imu_gyr_ += (cur_imu_gyr - mean_imu_gyr_) / iter_num;

    // 累计100帧数据后，完成IMU初始化
    if (iter_num > 99) {

        gravity_ = -mean_imu_acc_ / mean_imu_acc_.norm() * G_m_s2;
        return true;

    } else {

        return false;
    }
}
