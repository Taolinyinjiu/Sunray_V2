#ifndef SUNRAY_MAVROS_SIM_UGV_DYNAMICS_H
#define SUNRAY_MAVROS_SIM_UGV_DYNAMICS_H

#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>

namespace sunray_mavros_sim
{
struct UgvDynamicParams
{
    enum DriveType
    {
        DIFFERENTIAL = 0,
        MECANUM = 1
    };

    DriveType drive_type = DIFFERENTIAL;
    double max_linear_x = 1.5;
    double max_linear_y = 1.0;
    double max_angular_z = 2.0;
    double linear_acc_limit = 3.0;
    double angular_acc_limit = 6.0;
    double cmd_timeout = 0.2;
    double odom_covariance_xy = 1.0e-3;
    double odom_covariance_yaw = 2.0e-3;
};

struct UgvState
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_body = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double yaw_rate = 0.0;
    double yaw_acc = 0.0;
};

class UgvDynamics
{
public:
    explicit UgvDynamics(const UgvDynamicParams& params);

    void reset(double x, double y, double yaw);
    void update(const geometry_msgs::Twist& cmd, double dt);

    const UgvState& state() const { return state_; }
    const UgvDynamicParams& params() const { return params_; }

private:
    static double clamp(double value, double lower, double upper);
    static double wrapAngle(double angle);
    static double limitToward(double current, double target, double max_delta);

    UgvDynamicParams params_;
    UgvState state_;
};
}  // namespace sunray_mavros_sim

#endif
