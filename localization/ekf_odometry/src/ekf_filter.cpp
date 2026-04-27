
#include "ekf_filter.h"
#include <ctime>

EkfFilter::EkfFilter() {

    // 配置协方差
    R_ = 0.001;  //观测噪声

    Eigen::Vector3d gyr_cov = Eigen::Vector3d(0.1, 0.1, 0.1);                //角速度协方差
    Eigen::Vector3d acc_cov = Eigen::Vector3d(0.1, 0.1, 0.1);                //加速度协方差
    Eigen::Vector3d bias_gyr_cov = Eigen::Vector3d(0.0001, 0.0001, 0.0001);  //角速度bias的协方差
    Eigen::Vector3d bias_acc_cov = Eigen::Vector3d(0.0001, 0.0001, 0.0001);  //加速度bias的协方差

    Q_.block<3, 3>(0, 0).diagonal() = gyr_cov;
    Q_.block<3, 3>(3, 3).diagonal() = acc_cov;
    Q_.block<3, 3>(6, 6).diagonal() = bias_gyr_cov;
    Q_.block<3, 3>(9, 9).diagonal() = bias_acc_cov;

    Eigen::Matrix<double, 18, 18> init_P = Eigen::Matrix<double, 18, 18>::Identity();

    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.0001;
    init_P(12, 12) = init_P(13, 13) = init_P(14, 14) = 0.001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.00001;

    P_ = init_P;
}

//广义加法
EkfState EkfFilter::BoxPlus(const EkfState& x_in, const Eigen::Matrix<double, 18, 1>& f_) {

    // TODO:是否符合运动公式
    EkfState x_out;
    x_out.pos = x_in.pos + f_.block<3, 1>(0, 0);
    x_out.rot = x_in.rot * Sophus::SO3::exp(f_.block<3, 1>(3, 0));
    x_out.vel = x_in.vel + f_.block<3, 1>(6, 0);
    x_out.bg = x_in.bg + f_.block<3, 1>(9, 0);
    x_out.ba = x_in.ba + f_.block<3, 1>(12, 0);
    x_out.grav = x_in.grav + f_.block<3, 1>(15, 0);

    return x_out;
}

//对应公式(2) 中的f
Eigen::Matrix<double, 18, 1> EkfFilter::Get_F(const EkfState& x_in, const ImuData& imu_data) {

    Eigen::Matrix<double, 18, 1> res = Eigen::Matrix<double, 18, 1>::Zero();

    Eigen::Vector3d omega = imu_data.cur_imu_gyr - x_in.bg;
    Eigen::Vector3d a_inertial = x_in.rot.matrix() * (imu_data.cur_imu_acc - x_in.ba);

    res.block<3, 1>(0, 0) = x_in.vel;
    res.block<3, 1>(3, 0) = omega;
    res.block<3, 1>(6, 0) = a_inertial + x_in.grav;  // TODO:确定这里对重力是加法吗

    return res;
}

// 预测
void EkfFilter::Predict(const ImuData& imu_data, double dt) {

    Eigen::Matrix<double, 18, 1> f_ = Get_F(x_, imu_data);

    Eigen::Matrix<double, 18, 18> f_x_ = Df_Dx(x_, imu_data);
    Eigen::Matrix<double, 18, 12> f_w_ = Df_Dw(x_, imu_data);

    x_ = BoxPlus(x_, f_ * dt);

    x_.timestamp = imu_data.timeStamp;

    f_x_ = Eigen::Matrix<double, 18, 18>::Identity() + f_x_ * dt;
    P_ = (f_x_)*P_ * (f_x_).transpose() + (dt * f_w_) * Q_ * (dt * f_w_).transpose();
}

// 更新
void EkfFilter::Update(const Eigen::Matrix4d& reg_pose) {

    Eigen::Matrix<double, 18, 18> P = P_;

    Sophus::SO3 reg_so3 = Sophus::SO3(reg_pose.block<3, 3>(0, 0));

    // H矩阵6*18
    Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    H.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

    //观测噪声
    Eigen::Matrix<double, 6, 6> V = Eigen::Matrix<double, 6, 6>::Identity() * R_;

    //卡尔曼增益
    Eigen::Matrix<double, 18, 6> K = P * H.transpose() * (H * P * H.transpose() + V).inverse();

    //误差状态
    Eigen::Matrix<double, 6, 1> innov = Eigen::Matrix<double, 6, 1>::Zero();
    innov.head<3>() << reg_pose(0, 3) - x_.pos(0), reg_pose(1, 3) - x_.pos(1), reg_pose(2, 3) - x_.pos(2);
    innov.tail<3>() = (x_.rot.inverse() * reg_so3).log();

    //更新误差
    Eigen::Matrix<double, 18, 1> updated_err = K * innov;

    //更新协方差
    P_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * P_;

    //更新状态量
    x_.pos += updated_err.block<3, 1>(0, 0);                            //更新位置
    x_.rot = x_.rot * Sophus::SO3::exp(updated_err.block<3, 1>(3, 0));  //更新旋转
    x_.vel += updated_err.block<3, 1>(6, 0);                            //更新速度
    x_.bg += updated_err.block<3, 1>(9, 0);                             //更新bg
    x_.ba += updated_err.block<3, 1>(12, 0);                            //更新ba
    x_.grav += updated_err.block<3, 1>(15, 0);                          //更新gravity
}

//对应公式(7)的Fx  注意该矩阵没乘dt，没加单位阵
Eigen::Matrix<double, 18, 18> EkfFilter::Df_Dx(const EkfState& x_in, const ImuData& imu_data) {

    Eigen::Matrix<double, 18, 18> cov = Eigen::Matrix<double, 18, 18>::Zero();
    cov.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();  //对应公式(7)第2行第3列   I

    Eigen::Vector3d acc = imu_data.cur_imu_acc - x_in.ba;                //测量加速度 = a_m - bias
    cov.block<3, 3>(6, 3) = -x_in.rot.matrix() * Sophus::SO3::hat(acc);  //对应公式(7)第3行第1列
    cov.block<3, 3>(6, 12) = -x_in.rot.matrix();                         //对应公式(7)第3行第5列
    cov.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity();                //对应公式(7)第3行第6列   I
    cov.block<3, 3>(3, 9) = -Eigen::Matrix3d::Identity();                //对应公式(7)第1行第4列 (简化为-I)
    return cov;
}

//对应公式(7)的Fw  注意该矩阵没乘dt
Eigen::Matrix<double, 18, 12> EkfFilter::Df_Dw(const EkfState& x_in, const ImuData& imu_data) {

    Eigen::Matrix<double, 18, 12> cov = Eigen::Matrix<double, 18, 12>::Zero();
    cov.block<3, 3>(6, 3) = -x_in.rot.matrix();            //对应公式(7)第3行第2列  -R
    cov.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();  //对应公式(7)第1行第1列  -A(w dt)简化为-I
    cov.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();   //对应公式(7)第4行第3列  I
    cov.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();  //对应公式(7)第5行第4列  I
    return cov;
}
