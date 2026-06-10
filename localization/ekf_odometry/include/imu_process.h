#ifndef IMU_PROCESS_H
#define IMU_PROCESS_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>

struct ImuData {

    double timeStamp = 0.0;
    Eigen::Vector3d cur_imu_gyr = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d cur_imu_acc = Eigen::Vector3d(0, 0, 0);
};

class ImuProcess {

  public:
    ImuProcess(){};

    ~ImuProcess(){};

    Eigen::Vector3d GetInitGyrBias() {
        return mean_imu_gyr_;
    };

    Eigen::Vector3d GetInitGravity() {
        return gravity_;
    };

    void ProcessIMU(const ImuData& input_imu_data, ImuData& output_imu_data, double& dt);


    bool ImuInit(const ImuData& input_imu_data);

  private:
    const double G_m_s2 = 9.81;              //重力加速度常数
    Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, 0);       //重力
    Eigen::Vector3d mean_imu_acc_ = Eigen::Vector3d(0, 0, 0);  //加速度均值
    Eigen::Vector3d mean_imu_gyr_ = Eigen::Vector3d(0, 0, 0);  //角速度均值

    double last_imu_timeStamp_ = 0.0;
    Eigen::Vector3d last_imu_gyr_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d last_imu_acc_ = Eigen::Vector3d(0, 0, 0);
};

#endif  // IMU_PROCESS_H
