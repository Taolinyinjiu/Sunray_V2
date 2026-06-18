#ifndef UAV_SIMULATOR_IMU_MODEL_H
#define UAV_SIMULATOR_IMU_MODEL_H

#include <boost/array.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <random>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace sunray_imu_sim
{
struct ImuRosDefaults
{
    std::string preset;
    Eigen::Vector3d orientation_covariance = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_covariance = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acceleration_covariance = Eigen::Vector3d::Zero();
};

struct ImuModelConfig
{
    std::string preset = "custom";
    bool enable = true;
    int random_seed = 1;

    Eigen::Vector3d orientation_covariance = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_covariance = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acceleration_covariance = Eigen::Vector3d::Zero();

    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_noise_density = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_noise_density = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_markov_std = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_markov_tau = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_rw_density = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_markov_std = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_markov_tau = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias_rw_density = Eigen::Vector3d::Zero();
    Eigen::Matrix3d gyro_scale_install = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d accel_scale_install = Eigen::Matrix3d::Zero();
    Eigen::Vector3d accel_quadratic = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_lever_arm = Eigen::Vector3d::Zero();
    double gyro_accel_time_async = 0.0;

    std::string debug_summary;
};

struct ImuTruth
{
    ros::Time stamp;
    std::string frame_id;
    double sim_time = 0.0;
    double dt = 0.0;
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_specific_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_angular_acceleration = Eigen::Vector3d::Zero();
};

struct ImuMeasurement
{
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
};

class ImuModel
{
public:
    ImuModel();

    void configure(const ImuModelConfig& config);
    sensor_msgs::Imu generateMessage(const ImuTruth& truth);

    const ImuModelConfig& config() const { return config_; }
    std::string summary() const;

private:
    struct TruthSample
    {
        double time = 0.0;
        Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    };

    ImuMeasurement generateMeasurement(const ImuTruth& truth);
    void pushTruthSample(const double time,
                         const Eigen::Vector3d& gyro,
                         const Eigen::Vector3d& accel);
    Eigen::Vector3d sampleTruthWithDelay(const double sim_time,
                                         const Eigen::Vector3d& fallback,
                                         const bool use_gyro,
                                         const double delay) const;
    void updateBiasState(Eigen::Vector3d& state,
                         const Eigen::Vector3d& markov_std,
                         const Eigen::Vector3d& markov_tau,
                         const Eigen::Vector3d& rw_density,
                         const double dt);
    Eigen::Vector3d drawWhiteNoise(const Eigen::Vector3d& density, const double dt);

    ImuModelConfig config_;
    std::deque<TruthSample> history_;
    Eigen::Vector3d gyro_markov_state_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_markov_state_ = Eigen::Vector3d::Zero();
    mutable bool delay_clip_warned_ = false;
    std::mt19937 rng_;
    std::normal_distribution<double> gaussian_;
};

ImuModelConfig loadImuModelConfig(const ros::NodeHandle& nh,
                                  double gravity,
                                  const ImuRosDefaults& defaults = ImuRosDefaults());
ImuModelConfig makeBuiltinImuConfig(const std::string& preset,
                                    double gravity,
                                    const ImuRosDefaults& defaults = ImuRosDefaults());
std::vector<std::string> listBuiltinImuPresets();
void fillDiagonalCovariance3(boost::array<double, 9>& covariance, const Eigen::Vector3d& diagonal);
Eigen::Matrix3d rotationBodyToWorld(const Eigen::Quaterniond& orientation);
}  // namespace sunray_imu_sim

#endif
