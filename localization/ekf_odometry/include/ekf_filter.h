#ifndef EKF_FILTER_H
#define EKF_FILTER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.h>
#include "imu_process.h"

//状态量
struct EkfState {

    double timestamp = 0.0;

    Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
    Sophus::SO3 rot = Sophus::SO3(Eigen::Matrix3d::Identity());
    Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d bg = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d ba = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d grav = Eigen::Vector3d(0, 0, 0);
};

class EkfFilter {

  public:
    EkfFilter();

    ~EkfFilter(){};

    EkfState GetEkfState() {
        return x_;
    }

    void Update(const Eigen::Matrix4d& reg_pose);

    void SetInitGyrBias(const Eigen::Vector3d& init_gyr_bias) {
        x_.bg = init_gyr_bias;
    }

    void SetInitGravity(const Eigen::Vector3d& init_gravity) {
        x_.grav = init_gravity;
    }

    void SetInitPose(const Eigen::Matrix4d& reg_pose) {
        x_.pos = reg_pose.block<3, 1>(0, 3);
        x_.rot = Sophus::SO3(reg_pose.block<3, 3>(0, 0));
    }

    void Predict(const ImuData& imu_data, double dt);

  private:
    EkfState x_;

    //协方差矩阵
    Eigen::Matrix<double, 18, 18> P_ = Eigen::Matrix<double, 18, 18>::Zero();

    //状态转移噪声协方差矩阵
    Eigen::Matrix<double, 12, 12> Q_ = Eigen::Matrix<double, 12, 12>::Zero();

    //观测噪声
    double R_ = 0.001;


    Eigen::Matrix<double, 18, 12> Df_Dw(const EkfState& x_in, const ImuData& imu_data);

    Eigen::Matrix<double, 18, 18> Df_Dx(const EkfState& x_in, const ImuData& imu_data);

    EkfState BoxPlus(const EkfState& x_in, const Eigen::Matrix<double, 18, 1>& f_);

    Eigen::Matrix<double, 18, 1> Get_F(const EkfState& x_in, const ImuData& imu_data);
};

#endif
