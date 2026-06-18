#include "ugv_dynamics.h"

#include <algorithm>
#include <cmath>

namespace sunray_mavros_sim
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

UgvDynamics::UgvDynamics(const UgvDynamicParams& params)
    : params_(params)
{
}

void UgvDynamics::reset(const double x, const double y, const double yaw)
{
    state_ = UgvState();
    state_.position.x() = x;
    state_.position.y() = y;
    state_.yaw = wrapAngle(yaw);
}

void UgvDynamics::update(const geometry_msgs::Twist& cmd, const double dt)
{
    if (dt <= 0.0)
    {
        state_.acceleration_body.setZero();
        state_.yaw_acc = 0.0;
        return;
    }

    double target_vx = clamp(cmd.linear.x, -params_.max_linear_x, params_.max_linear_x);
    double target_vy = clamp(cmd.linear.y, -params_.max_linear_y, params_.max_linear_y);
    double target_wz = clamp(cmd.angular.z, -params_.max_angular_z, params_.max_angular_z);

    if (params_.drive_type == UgvDynamicParams::DIFFERENTIAL)
    {
        target_vy = 0.0;
    }

    const double linear_delta_limit = std::max(0.0, params_.linear_acc_limit) * dt;
    const double angular_delta_limit = std::max(0.0, params_.angular_acc_limit) * dt;

    const double prev_vx = state_.velocity_body.x();
    const double prev_vy = state_.velocity_body.y();
    const double prev_wz = state_.yaw_rate;

    state_.velocity_body.x() = limitToward(prev_vx, target_vx, linear_delta_limit);
    state_.velocity_body.y() = limitToward(prev_vy, target_vy, linear_delta_limit);
    state_.velocity_body.z() = 0.0;
    state_.yaw_rate = limitToward(prev_wz, target_wz, angular_delta_limit);

    state_.acceleration_body.x() = (state_.velocity_body.x() - prev_vx) / dt;
    state_.acceleration_body.y() = (state_.velocity_body.y() - prev_vy) / dt;
    state_.acceleration_body.z() = 0.0;
    state_.yaw_acc = (state_.yaw_rate - prev_wz) / dt;

    const double cos_yaw = std::cos(state_.yaw);
    const double sin_yaw = std::sin(state_.yaw);
    const double vx_world = cos_yaw * state_.velocity_body.x() - sin_yaw * state_.velocity_body.y();
    const double vy_world = sin_yaw * state_.velocity_body.x() + cos_yaw * state_.velocity_body.y();

    state_.position.x() += vx_world * dt;
    state_.position.y() += vy_world * dt;
    state_.yaw = wrapAngle(state_.yaw + state_.yaw_rate * dt);
}

double UgvDynamics::clamp(const double value, const double lower, const double upper)
{
    return std::max(lower, std::min(value, upper));
}

double UgvDynamics::wrapAngle(const double angle)
{
    double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
    if (wrapped < 0.0)
    {
        wrapped += 2.0 * kPi;
    }
    return wrapped - kPi;
}

double UgvDynamics::limitToward(const double current, const double target, const double max_delta)
{
    const double delta = target - current;
    if (delta > max_delta)
    {
        return current + max_delta;
    }
    if (delta < -max_delta)
    {
        return current - max_delta;
    }
    return target;
}
}  // namespace sunray_mavros_sim
