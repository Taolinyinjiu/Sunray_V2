#include "imu_model.h"

#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <limits>
#include <sstream>

namespace sunray_imu_sim
{
namespace
{
constexpr double kSecondsPerHour = 3600.0;
constexpr double kSqrtSecondsPerHour = 60.0;
constexpr double kArcsecToRad = M_PI / (180.0 * 3600.0);
constexpr double kPpmToScale = 1e-6;
constexpr double kMicroScale = 1e-6;
constexpr std::size_t kMaxImuHistorySize = 4096;
constexpr double kRandomWalkTauThreshold = 1.0e8;

struct ImuUserParams
{
    std::string preset = "custom";
    bool enable = true;
    int random_seed = 1;

    Eigen::Vector3d orientation_covariance = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_covariance = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acceleration_covariance = Eigen::Vector3d::Zero();

    Eigen::Vector3d gyro_bias_degph = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias_ug = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_arw_deg_sqrt_hour = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_vrw_ug_sqrt_hz = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_corr_bias_degph = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_corr_tau_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_corr_bias_ug = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_corr_tau_s = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_scale_factor_ppm = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_scale_factor_ppm = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_quadratic_ug_g2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_lever_arm_cm = Eigen::Vector3d::Zero();
    std::vector<double> gyro_installation_error_arcsec = std::vector<double>(6, 0.0);
    std::vector<double> accel_installation_error_arcsec = std::vector<double>(6, 0.0);
    double gyro_accel_time_async_ms = 0.0;
};

bool xmlRpcToDouble(const XmlRpc::XmlRpcValue& value, double& output)
{
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        output = static_cast<double>(value);
        return true;
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        output = static_cast<int>(value);
        return true;
    }
    return false;
}

std::string normalizePresetName(const std::string& name)
{
    std::string normalized = name;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), ::tolower);
    return normalized;
}

std::vector<double> parseNumericString(const std::string& input)
{
    std::string normalized = input;
    for (std::size_t i = 0; i < normalized.size(); ++i)
    {
        char& ch = normalized[i];
        if (ch == '[' || ch == ']' || ch == ',')
        {
            ch = ' ';
        }
    }

    std::vector<double> values;
    std::stringstream ss(normalized);
    double value = 0.0;
    while (ss >> value)
    {
        values.push_back(value);
    }
    return values;
}

std::vector<double> loadNumericParam(const ros::NodeHandle& nh, const std::string& name)
{
    XmlRpc::XmlRpcValue param;
    if (!nh.getParam(name, param))
    {
        return std::vector<double>();
    }

    std::vector<double> values;
    if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        values.reserve(static_cast<std::size_t>(param.size()));
        for (int i = 0; i < param.size(); ++i)
        {
            double value = 0.0;
            if (!xmlRpcToDouble(param[i], value))
            {
                ROS_WARN_STREAM("Parameter '" << name << "' contains a non-numeric entry, ignoring it.");
                continue;
            }
            values.push_back(value);
        }
        return values;
    }

    if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
        return parseNumericString(static_cast<std::string>(param));
    }

    double scalar = 0.0;
    if (xmlRpcToDouble(param, scalar))
    {
        values.push_back(scalar);
        return values;
    }

    ROS_WARN_STREAM("Unsupported parameter format for '" << name << "', keeping the current value.");
    return std::vector<double>();
}

Eigen::Vector3d expandTriadOrDefault(const std::vector<double>& values,
                                     const Eigen::Vector3d& fallback,
                                     const std::string& name)
{
    if (values.empty())
    {
        return fallback;
    }
    if (values.size() == 1)
    {
        return Eigen::Vector3d::Constant(values[0]);
    }
    if (values.size() == 2)
    {
        return Eigen::Vector3d(values[0], values[0], values[1]);
    }
    if (values.size() == 3)
    {
        return Eigen::Vector3d(values[0], values[1], values[2]);
    }

    ROS_WARN_STREAM("Parameter '" << name << "' expects 1, 2, or 3 values, received "
                    << values.size() << ". Keeping the current value.");
    return fallback;
}

std::vector<double> expandSixOrDefault(const std::vector<double>& values,
                                       const std::vector<double>& fallback,
                                       const std::string& name)
{
    if (values.empty())
    {
        return fallback;
    }
    if (values.size() == 1)
    {
        return std::vector<double>(6, values[0]);
    }
    if (values.size() == 3)
    {
        std::vector<double> expanded(6, 0.0);
        expanded[0] = values[0];
        expanded[1] = values[1];
        expanded[2] = values[2];
        return expanded;
    }
    if (values.size() == 6)
    {
        return values;
    }

    ROS_WARN_STREAM("Parameter '" << name << "' expects 1, 3, or 6 values, received "
                    << values.size() << ". Keeping the current value.");
    return fallback;
}

double degPerHourToRadPerSecond(const double value_degph)
{
    return value_degph * M_PI / 180.0 / kSecondsPerHour;
}

double degPerSqrtHourToRadPerSecondPerSqrtHz(const double value_degpsh)
{
    return value_degpsh * M_PI / 180.0 / kSqrtSecondsPerHour;
}

double degPerHourPerSqrtHourToRadPerSecondPerSqrtSecond(const double value_degphpsh)
{
    return value_degphpsh * M_PI / 180.0 / (kSecondsPerHour * kSqrtSecondsPerHour);
}

double microGToMetersPerSecondSquared(const double value_ug, const double gravity)
{
    return value_ug * kMicroScale * gravity;
}

double microGPerSqrtHzToMetersPerSecondSquaredPerSqrtHz(const double value_ugpshz, const double gravity)
{
    return value_ugpshz * kMicroScale * gravity;
}

double microGPerSqrtHourToMetersPerSecondSquaredPerSqrtSecond(const double value_ugpsh, const double gravity)
{
    return value_ugpsh * kMicroScale * gravity / kSqrtSecondsPerHour;
}

double accelQuadraticUgPerG2ToSI(const double value_ugpg2, const double gravity)
{
    if (gravity <= 0.0)
    {
        return 0.0;
    }
    return value_ugpg2 * kMicroScale / gravity;
}

Eigen::Matrix3d buildGyroScaleInstallMatrix(const Eigen::Vector3d& diag_ppm,
                                            const std::vector<double>& off_diag_arcsec)
{
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
    matrix.diagonal() = diag_ppm * kPpmToScale;

    if (off_diag_arcsec.size() != 6)
    {
        return matrix;
    }

    matrix(1, 0) = off_diag_arcsec[0] * kArcsecToRad;
    matrix(2, 0) = off_diag_arcsec[1] * kArcsecToRad;
    matrix(2, 1) = off_diag_arcsec[2] * kArcsecToRad;
    matrix(0, 1) = off_diag_arcsec[3] * kArcsecToRad;
    matrix(0, 2) = off_diag_arcsec[4] * kArcsecToRad;
    matrix(1, 2) = off_diag_arcsec[5] * kArcsecToRad;
    return matrix;
}

Eigen::Matrix3d buildAccelScaleInstallMatrix(const Eigen::Vector3d& diag_ppm,
                                             const std::vector<double>& off_diag_arcsec)
{
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
    matrix.diagonal() = diag_ppm * kPpmToScale;

    if (off_diag_arcsec.size() != 6)
    {
        return matrix;
    }

    matrix(1, 0) = off_diag_arcsec[0] * kArcsecToRad;
    matrix(2, 0) = off_diag_arcsec[1] * kArcsecToRad;
    matrix(2, 1) = off_diag_arcsec[2] * kArcsecToRad;
    matrix(0, 1) = off_diag_arcsec[3] * kArcsecToRad;
    matrix(0, 2) = off_diag_arcsec[4] * kArcsecToRad;
    matrix(1, 2) = off_diag_arcsec[5] * kArcsecToRad;
    return matrix;
}

Eigen::Vector3d sampleNormalVector(std::mt19937& rng, std::normal_distribution<double>& distribution)
{
    return Eigen::Vector3d(distribution(rng), distribution(rng), distribution(rng));
}

ImuUserParams makePreset(const std::string& preset_name,
                         const Eigen::Vector3d& gyro_bias_degph,
                         const Eigen::Vector3d& accel_bias_ug,
                         const Eigen::Vector3d& gyro_arw_deg_sqrt_hour,
                         const Eigen::Vector3d& accel_vrw_ug_sqrt_hz)
{
    ImuUserParams preset;
    preset.preset = preset_name;
    preset.gyro_bias_degph = gyro_bias_degph;
    preset.accel_bias_ug = accel_bias_ug;
    preset.gyro_arw_deg_sqrt_hour = gyro_arw_deg_sqrt_hour;
    preset.accel_vrw_ug_sqrt_hz = accel_vrw_ug_sqrt_hz;
    return preset;
}

bool tryApplyBuiltinPreset(const std::string& name, ImuUserParams& params)
{
    const std::string normalized = normalizePresetName(name);
    if (normalized.empty() || normalized == "custom")
    {
        params.preset = normalized.empty() ? "custom" : normalized;
        return false;
    }

    ImuUserParams preset;
    if (normalized == "ideal")
    {
        // Idealized zero-error IMU for regression/debug usage.
        preset = makePreset("ideal",
                            Eigen::Vector3d::Zero(),
                            Eigen::Vector3d::Zero(),
                            Eigen::Vector3d::Zero(),
                            Eigen::Vector3d::Zero());
    }
    else if (normalized == "vn100" || normalized == "vectornav_vn100")
    {
        // VectorNav VN-100 product specs.
        preset = makePreset("vn100",
                            Eigen::Vector3d::Constant(5.0),
                            Eigen::Vector3d::Constant(40.0),
                            Eigen::Vector3d::Constant(0.21),
                            Eigen::Vector3d::Constant(140.0));
    }
    else if (normalized == "mti680g" || normalized == "xsens_mti680g")
    {
        // Movella/Xsens MTi-680G product specs.
        preset = makePreset("mti680g",
                            Eigen::Vector3d::Constant(8.0),
                            Eigen::Vector3d(10.0, 10.0, 15.0),
                            Eigen::Vector3d::Constant(0.42),
                            Eigen::Vector3d::Constant(60.0));
    }
    else if (normalized == "adis16470")
    {
        // Analog Devices ADIS16470 datasheet.
        preset = makePreset("adis16470",
                            Eigen::Vector3d::Constant(8.0),
                            Eigen::Vector3d::Constant(13.0),
                            Eigen::Vector3d::Constant(0.34),
                            Eigen::Vector3d::Constant(100.0));
    }
    else if (normalized == "gq7" || normalized == "3dm_gq7" || normalized == "microstrain_3dm_gq7")
    {
        // MicroStrain 3DM-GQ7 product specs.
        preset = makePreset("gq7",
                            Eigen::Vector3d::Constant(1.5),
                            Eigen::Vector3d::Constant(5.0),
                            Eigen::Vector3d::Constant(0.15),
                            Eigen::Vector3d::Constant(20.0));
    }
    else
    {
        return false;
    }

    preset.enable = params.enable;
    preset.random_seed = params.random_seed;
    preset.orientation_covariance = params.orientation_covariance;
    preset.angular_velocity_covariance = params.angular_velocity_covariance;
    preset.linear_acceleration_covariance = params.linear_acceleration_covariance;
    params = preset;
    return true;
}

std::string vectorToString(const Eigen::Vector3d& value)
{
    std::ostringstream ss;
    ss << value.transpose();
    return ss.str();
}

ImuModelConfig buildImuModelConfigFromUserParams(const ImuUserParams& params, const double gravity)
{
    ImuModelConfig config;
    config.preset = params.preset.empty() ? "custom" : params.preset;
    config.enable = params.enable;
    config.random_seed = params.random_seed;
    config.orientation_covariance = params.orientation_covariance;
    config.angular_velocity_covariance = params.angular_velocity_covariance;
    config.linear_acceleration_covariance = params.linear_acceleration_covariance;
    config.gyro_accel_time_async = params.gyro_accel_time_async_ms / 1000.0;

    for (int i = 0; i < 3; ++i)
    {
        config.gyro_bias(i) = degPerHourToRadPerSecond(params.gyro_bias_degph(i));
        config.accel_bias(i) = microGToMetersPerSecondSquared(params.accel_bias_ug(i), gravity);
        config.gyro_noise_density(i) = degPerSqrtHourToRadPerSecondPerSqrtHz(params.gyro_arw_deg_sqrt_hour(i));
        config.accel_noise_density(i) =
            microGPerSqrtHzToMetersPerSecondSquaredPerSqrtHz(params.accel_vrw_ug_sqrt_hz(i), gravity);
        config.accel_quadratic(i) = accelQuadraticUgPerG2ToSI(params.accel_quadratic_ug_g2(i), gravity);
        config.accel_lever_arm(i) = params.accel_lever_arm_cm(i) / 100.0;

        if (params.gyro_corr_tau_s(i) > 0.0 && params.gyro_corr_tau_s(i) < kRandomWalkTauThreshold)
        {
            config.gyro_markov_std(i) = degPerHourToRadPerSecond(params.gyro_corr_bias_degph(i));
            config.gyro_markov_tau(i) = params.gyro_corr_tau_s(i);
        }
        else if (params.gyro_corr_tau_s(i) >= kRandomWalkTauThreshold)
        {
            config.gyro_bias_rw_density(i) =
                degPerHourPerSqrtHourToRadPerSecondPerSqrtSecond(params.gyro_corr_bias_degph(i));
        }

        if (params.accel_corr_tau_s(i) > 0.0 && params.accel_corr_tau_s(i) < kRandomWalkTauThreshold)
        {
            config.accel_markov_std(i) = microGToMetersPerSecondSquared(params.accel_corr_bias_ug(i), gravity);
            config.accel_markov_tau(i) = params.accel_corr_tau_s(i);
        }
        else if (params.accel_corr_tau_s(i) >= kRandomWalkTauThreshold)
        {
            config.accel_bias_rw_density(i) =
                microGPerSqrtHourToMetersPerSecondSquaredPerSqrtSecond(params.accel_corr_bias_ug(i), gravity);
        }
    }

    config.gyro_scale_install =
        buildGyroScaleInstallMatrix(params.gyro_scale_factor_ppm, params.gyro_installation_error_arcsec);
    config.accel_scale_install =
        buildAccelScaleInstallMatrix(params.accel_scale_factor_ppm, params.accel_installation_error_arcsec);

    std::ostringstream summary;
    summary << "preset=" << config.preset
            << ", enabled=" << (config.enable ? "true" : "false")
            << ", gyro_bias_degph=[" << vectorToString(params.gyro_bias_degph) << "]"
            << ", accel_bias_ug=[" << vectorToString(params.accel_bias_ug) << "]"
            << ", gyro_arw_deg_sqrt_hour=[" << vectorToString(params.gyro_arw_deg_sqrt_hour) << "]"
            << ", accel_vrw_ug_sqrt_hz=[" << vectorToString(params.accel_vrw_ug_sqrt_hz) << "]";
    config.debug_summary = summary.str();
    return config;
}

std::string joinPresetNames()
{
    const std::vector<std::string> presets = listBuiltinImuPresets();
    std::ostringstream ss;
    for (std::size_t i = 0; i < presets.size(); ++i)
    {
        if (i > 0)
        {
            ss << ", ";
        }
        ss << presets[i];
    }
    return ss.str();
}

}  // namespace

ImuModel::ImuModel()
    : rng_(1), gaussian_(0.0, 1.0)
{
}

void ImuModel::configure(const ImuModelConfig& config)
{
    config_ = config;
    history_.clear();
    gyro_markov_state_.setZero();
    accel_markov_state_.setZero();
    delay_clip_warned_ = false;
    rng_.seed(static_cast<std::mt19937::result_type>(static_cast<std::uint32_t>(config.random_seed)));
}

sensor_msgs::Imu ImuModel::generateMessage(const ImuTruth& truth)
{
    sensor_msgs::Imu msg;
    msg.header.stamp = truth.stamp;
    msg.header.frame_id = truth.frame_id;
    msg.orientation.w = truth.orientation.w();
    msg.orientation.x = truth.orientation.x();
    msg.orientation.y = truth.orientation.y();
    msg.orientation.z = truth.orientation.z();

    const ImuMeasurement measurement = generateMeasurement(truth);
    msg.angular_velocity.x = measurement.gyro.x();
    msg.angular_velocity.y = measurement.gyro.y();
    msg.angular_velocity.z = measurement.gyro.z();
    msg.linear_acceleration.x = measurement.accel.x();
    msg.linear_acceleration.y = measurement.accel.y();
    msg.linear_acceleration.z = measurement.accel.z();

    fillDiagonalCovariance3(msg.orientation_covariance, config_.orientation_covariance);
    fillDiagonalCovariance3(msg.angular_velocity_covariance, config_.angular_velocity_covariance);
    fillDiagonalCovariance3(msg.linear_acceleration_covariance, config_.linear_acceleration_covariance);
    return msg;
}

std::string ImuModel::summary() const
{
    return config_.debug_summary;
}

ImuMeasurement ImuModel::generateMeasurement(const ImuTruth& truth)
{
    ImuMeasurement measurement;

    const Eigen::Vector3d lever_arm_accel =
        truth.body_specific_force +
        truth.body_angular_acceleration.cross(config_.accel_lever_arm) +
        truth.angular_velocity.cross(truth.angular_velocity.cross(config_.accel_lever_arm));

    pushTruthSample(truth.sim_time, truth.angular_velocity, lever_arm_accel);

    measurement.gyro = sampleTruthWithDelay(truth.sim_time,
                                            truth.angular_velocity,
                                            true,
                                            config_.gyro_accel_time_async);
    measurement.accel = sampleTruthWithDelay(truth.sim_time,
                                             lever_arm_accel,
                                             false,
                                             -config_.gyro_accel_time_async);

    if (!config_.enable)
    {
        return measurement;
    }

    updateBiasState(gyro_markov_state_,
                    config_.gyro_markov_std,
                    config_.gyro_markov_tau,
                    config_.gyro_bias_rw_density,
                    truth.dt);
    updateBiasState(accel_markov_state_,
                    config_.accel_markov_std,
                    config_.accel_markov_tau,
                    config_.accel_bias_rw_density,
                    truth.dt);

    const Eigen::Vector3d gyro_noise = drawWhiteNoise(config_.gyro_noise_density, truth.dt);
    const Eigen::Vector3d accel_noise = drawWhiteNoise(config_.accel_noise_density, truth.dt);

    measurement.gyro =
        (Eigen::Matrix3d::Identity() + config_.gyro_scale_install) * measurement.gyro +
        config_.gyro_bias + gyro_markov_state_ + gyro_noise;

    const Eigen::Vector3d accel_quadratic_term =
        (config_.accel_quadratic.array() * measurement.accel.array().square()).matrix();
    measurement.accel =
        (Eigen::Matrix3d::Identity() + config_.accel_scale_install) * measurement.accel +
        accel_quadratic_term + config_.accel_bias + accel_markov_state_ + accel_noise;

    return measurement;
}

void ImuModel::pushTruthSample(const double time,
                               const Eigen::Vector3d& gyro,
                               const Eigen::Vector3d& accel)
{
    TruthSample sample;
    sample.time = time;
    sample.gyro = gyro;
    sample.accel = accel;
    history_.push_back(sample);
    while (history_.size() > kMaxImuHistorySize)
    {
        history_.pop_front();
    }
}

Eigen::Vector3d ImuModel::sampleTruthWithDelay(const double sim_time,
                                               const Eigen::Vector3d& fallback,
                                               const bool use_gyro,
                                               const double delay) const
{
    if (delay <= 0.0 || history_.empty())
    {
        return fallback;
    }

    const double target_time = sim_time - delay;
    if (target_time <= history_.front().time)
    {
        if (!delay_clip_warned_)
        {
            ROS_WARN_STREAM("IMU time asynchrony exceeds cached history at startup; clamping delayed samples.");
            delay_clip_warned_ = true;
        }
        return use_gyro ? history_.front().gyro : history_.front().accel;
    }

    for (std::size_t i = 1; i < history_.size(); ++i)
    {
        if (target_time <= history_[i].time)
        {
            const TruthSample& prev = history_[i - 1];
            const TruthSample& next = history_[i];
            const double span = next.time - prev.time;
            if (span <= 1.0e-9)
            {
                return use_gyro ? prev.gyro : prev.accel;
            }

            const double alpha = std::max(0.0, std::min(1.0, (target_time - prev.time) / span));
            const Eigen::Vector3d& prev_value = use_gyro ? prev.gyro : prev.accel;
            const Eigen::Vector3d& next_value = use_gyro ? next.gyro : next.accel;
            return (1.0 - alpha) * prev_value + alpha * next_value;
        }
    }

    return use_gyro ? history_.back().gyro : history_.back().accel;
}

void ImuModel::updateBiasState(Eigen::Vector3d& state,
                               const Eigen::Vector3d& markov_std,
                               const Eigen::Vector3d& markov_tau,
                               const Eigen::Vector3d& rw_density,
                               const double dt)
{
    if (dt <= 0.0)
    {
        return;
    }

    for (int i = 0; i < 3; ++i)
    {
        if (markov_tau(i) > 0.0)
        {
            const double phi = std::exp(-dt / markov_tau(i));
            const double sigma = markov_std(i) * std::sqrt(std::max(0.0, 1.0 - phi * phi));
            state(i) = phi * state(i) + sigma * gaussian_(rng_);
        }
        else if (rw_density(i) > 0.0)
        {
            state(i) += rw_density(i) * std::sqrt(dt) * gaussian_(rng_);
        }
        else
        {
            state(i) = 0.0;
        }
    }
}

Eigen::Vector3d ImuModel::drawWhiteNoise(const Eigen::Vector3d& density, const double dt)
{
    if (dt <= 0.0)
    {
        return Eigen::Vector3d::Zero();
    }

    const double inv_sqrt_dt = 1.0 / std::sqrt(dt);
    return (density.array() * inv_sqrt_dt * sampleNormalVector(rng_, gaussian_).array()).matrix();
}

ImuModelConfig loadImuModelConfig(const ros::NodeHandle& nh,
                                  const double gravity,
                                  const ImuRosDefaults& defaults)
{
    ImuUserParams params;
    params.preset = defaults.preset.empty() ? "custom" : normalizePresetName(defaults.preset);
    params.orientation_covariance = defaults.orientation_covariance;
    params.angular_velocity_covariance = defaults.angular_velocity_covariance;
    params.linear_acceleration_covariance = defaults.linear_acceleration_covariance;

    nh.param<bool>("imu/enable", params.enable, params.enable);
    nh.param<int>("imu/random_seed", params.random_seed, params.random_seed);

    std::string preset_name = params.preset;
    nh.param<std::string>("imu/preset", preset_name, preset_name);
    preset_name = normalizePresetName(preset_name);
    if (!preset_name.empty() && preset_name != "custom" && !tryApplyBuiltinPreset(preset_name, params))
    {
        ROS_WARN_STREAM("Unknown IMU preset '" << preset_name << "'. Available presets: " << joinPresetNames());
    }
    else if (!preset_name.empty())
    {
        params.preset = preset_name;
    }

    params.orientation_covariance =
        expandTriadOrDefault(loadNumericParam(nh, "imu/orientation_covariance"),
                             params.orientation_covariance,
                             "imu/orientation_covariance");
    params.angular_velocity_covariance =
        expandTriadOrDefault(loadNumericParam(nh, "imu/angular_velocity_covariance"),
                             params.angular_velocity_covariance,
                             "imu/angular_velocity_covariance");
    params.linear_acceleration_covariance =
        expandTriadOrDefault(loadNumericParam(nh, "imu/linear_acceleration_covariance"),
                             params.linear_acceleration_covariance,
                             "imu/linear_acceleration_covariance");

    params.gyro_bias_degph =
        expandTriadOrDefault(loadNumericParam(nh, "imu/gyro_bias_degph"),
                             params.gyro_bias_degph,
                             "imu/gyro_bias_degph");
    params.accel_bias_ug =
        expandTriadOrDefault(loadNumericParam(nh, "imu/accel_bias_ug"),
                             params.accel_bias_ug,
                             "imu/accel_bias_ug");
    params.gyro_arw_deg_sqrt_hour =
        expandTriadOrDefault(loadNumericParam(nh, "imu/gyro_arw_deg_sqrt_hour"),
                             params.gyro_arw_deg_sqrt_hour,
                             "imu/gyro_arw_deg_sqrt_hour");
    params.accel_vrw_ug_sqrt_hz =
        expandTriadOrDefault(loadNumericParam(nh, "imu/accel_vrw_ug_sqrt_hz"),
                             params.accel_vrw_ug_sqrt_hz,
                             "imu/accel_vrw_ug_sqrt_hz");
    params.gyro_corr_bias_degph =
        expandTriadOrDefault(loadNumericParam(nh, "imu/gyro_corr_bias_degph"),
                             params.gyro_corr_bias_degph,
                             "imu/gyro_corr_bias_degph");
    params.accel_corr_bias_ug =
        expandTriadOrDefault(loadNumericParam(nh, "imu/accel_corr_bias_ug"),
                             params.accel_corr_bias_ug,
                             "imu/accel_corr_bias_ug");
    params.gyro_corr_tau_s =
        expandTriadOrDefault(loadNumericParam(nh, "imu/gyro_corr_tau_s"),
                             params.gyro_corr_tau_s,
                             "imu/gyro_corr_tau_s");
    params.accel_corr_tau_s =
        expandTriadOrDefault(loadNumericParam(nh, "imu/accel_corr_tau_s"),
                             params.accel_corr_tau_s,
                             "imu/accel_corr_tau_s");
    params.gyro_scale_factor_ppm =
        expandTriadOrDefault(loadNumericParam(nh, "imu/gyro_scale_factor_ppm"),
                             params.gyro_scale_factor_ppm,
                             "imu/gyro_scale_factor_ppm");
    params.accel_scale_factor_ppm =
        expandTriadOrDefault(loadNumericParam(nh, "imu/accel_scale_factor_ppm"),
                             params.accel_scale_factor_ppm,
                             "imu/accel_scale_factor_ppm");
    params.accel_quadratic_ug_g2 =
        expandTriadOrDefault(loadNumericParam(nh, "imu/accel_quadratic_ug_g2"),
                             params.accel_quadratic_ug_g2,
                             "imu/accel_quadratic_ug_g2");
    params.accel_lever_arm_cm =
        expandTriadOrDefault(loadNumericParam(nh, "imu/accel_lever_arm_cm"),
                             params.accel_lever_arm_cm,
                             "imu/accel_lever_arm_cm");
    params.gyro_installation_error_arcsec =
        expandSixOrDefault(loadNumericParam(nh, "imu/gyro_installation_error_arcsec"),
                           params.gyro_installation_error_arcsec,
                           "imu/gyro_installation_error_arcsec");
    params.accel_installation_error_arcsec =
        expandSixOrDefault(loadNumericParam(nh, "imu/accel_installation_error_arcsec"),
                           params.accel_installation_error_arcsec,
                           "imu/accel_installation_error_arcsec");
    nh.param<double>("imu/gyro_accel_time_async_ms",
                     params.gyro_accel_time_async_ms,
                     params.gyro_accel_time_async_ms);

    return buildImuModelConfigFromUserParams(params, gravity);
}

ImuModelConfig makeBuiltinImuConfig(const std::string& preset,
                                    const double gravity,
                                    const ImuRosDefaults& defaults)
{
    ImuUserParams params;
    params.preset = defaults.preset.empty() ? normalizePresetName(preset) : normalizePresetName(defaults.preset);
    params.orientation_covariance = defaults.orientation_covariance;
    params.angular_velocity_covariance = defaults.angular_velocity_covariance;
    params.linear_acceleration_covariance = defaults.linear_acceleration_covariance;

    const std::string requested = normalizePresetName(preset);
    if (!requested.empty() && requested != "custom" && !tryApplyBuiltinPreset(requested, params))
    {
        throw std::invalid_argument("unknown IMU preset: " + preset);
    }
    if (!requested.empty())
    {
        params.preset = requested;
    }

    return buildImuModelConfigFromUserParams(params, gravity);
}

std::vector<std::string> listBuiltinImuPresets()
{
    std::vector<std::string> presets;
    presets.push_back("ideal");
    presets.push_back("vn100");
    presets.push_back("mti680g");
    presets.push_back("adis16470");
    presets.push_back("gq7");
    return presets;
}

void fillDiagonalCovariance3(boost::array<double, 9>& covariance, const Eigen::Vector3d& diagonal)
{
    covariance.assign(0.0);
    covariance[0] = diagonal.x();
    covariance[4] = diagonal.y();
    covariance[8] = diagonal.z();
}

Eigen::Matrix3d rotationBodyToWorld(const Eigen::Quaterniond& orientation)
{
    return orientation.normalized().toRotationMatrix();
}
}  // namespace sunray_imu_sim
