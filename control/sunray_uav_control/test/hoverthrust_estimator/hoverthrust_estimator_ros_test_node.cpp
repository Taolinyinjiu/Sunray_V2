#include <cmath>
#include <algorithm>
#include <deque>
#include <memory>
#include <random>
#include <string>

#include <Eigen/Geometry>
#include <ros/ros.h>

#include "controller/core_algorithm/hoverthrust_estimator.hpp"

namespace {

double deg2rad(const double deg) {
    return deg * M_PI / 180.0;
}

class HoverThrustEstimatorRosTestNode {
  public:
    explicit HoverThrustEstimatorRosTestNode(const ros::NodeHandle& nh) : nh_(nh), pnh_("~") {
        pnh_.param<std::string>("estimator_type", estimator_type_, "rls");
        pnh_.param("loop_hz", loop_hz_, 100.0);
        pnh_.param("test_duration_s", test_duration_s_, 20.0);
        pnh_.param("gravity", gravity_, 9.81);
        pnh_.param("true_hover_thrust", true_hover_thrust_, 0.45);
        pnh_.param("init_hover_thrust", init_hover_thrust_, 0.30);
        pnh_.param("max_tilt_deg", max_tilt_deg_, 15.0);
        pnh_.param("acc_noise_std", acc_noise_std_, 0.08);
        pnh_.param("acc_cmd_amp", acc_cmd_amp_, 0.6);
        pnh_.param("acc_cmd_freq_hz", acc_cmd_freq_hz_, 0.35);
        pnh_.param("sim_delay_s", sim_delay_s_, 0.04);
        pnh_.param("pass_abs_error", pass_abs_error_, 0.05);

        rng_.seed(42);
        noise_dist_ = std::normal_distribution<double>(0.0, acc_noise_std_);

        estimator_ = createEstimator(estimator_type_);
        if (!estimator_) {
            ROS_ERROR("[hoverthrust_estimator_test] Unsupported estimator_type: %s",
                      estimator_type_.c_str());
            exit_code_ = 2;
            ros::requestShutdown();
            return;
        }

        thrust_estimator::Param_t param;
        param.hover_thrust = init_hover_thrust_;
        param.gravity = gravity_;
        estimator_->load_param(param);

        thr2acc_true_ = gravity_ / std::max(true_hover_thrust_, 1e-6);
        start_time_ = ros::Time::now();
        timer_ = nh_.createTimer(ros::Duration(1.0 / loop_hz_),
                                 &HoverThrustEstimatorRosTestNode::onTimer,
                                 this);

        ROS_INFO("[hoverthrust_estimator_test] Start. type=%s true_hover=%.4f init_hover=%.4f",
                 estimator_type_.c_str(),
                 true_hover_thrust_,
                 init_hover_thrust_);
    }

    int exitCode() const {
        return exit_code_;
    }

  private:
    std::unique_ptr<thrust_estimator::HoverThrustEstimator> createEstimator(
        const std::string& estimator_type) {
        if (estimator_type == "rls") {
            return std::unique_ptr<thrust_estimator::HoverThrustEstimator>(
                new thrust_estimator::RLS_HoverThrustEstimator());
        }
        if (estimator_type == "ekf") {
            return std::unique_ptr<thrust_estimator::HoverThrustEstimator>(
                new thrust_estimator::EKF_HoverThrustEstimator());
        }
        return nullptr;
    }

    void onTimer(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        const double t = (now - start_time_).toSec();
        if (t <= 0.0) {
            return;
        }

        const double tilt = deg2rad(max_tilt_deg_) * std::sin(2.0 * M_PI * 0.2 * t);
        const Eigen::Quaterniond attitude(Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()));
        const Eigen::Vector3d zb = attitude.toRotationMatrix().col(2);
        const double tilt_cos = std::clamp(zb.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);

        const double acc_cmd = acc_cmd_amp_ * std::sin(2.0 * M_PI * acc_cmd_freq_hz_ * t);
        const double thrust_cmd = std::clamp((acc_cmd + gravity_) /
                                                 (thr2acc_true_ * std::max(tilt_cos, 1e-3)),
                                             0.05,
                                             0.80);
        command_history_.push_back(CommandSample{now, attitude, thrust_cmd});
        while (!command_history_.empty() &&
               (now - command_history_.front().stamp).toSec() > sim_delay_s_ + 0.1) {
            command_history_.pop_front();
        }

        CommandSample response_sample{now, attitude, thrust_cmd};
        if (estimator_type_ == "ekf") {
            for (auto it = command_history_.rbegin(); it != command_history_.rend(); ++it) {
                const double age = (now - it->stamp).toSec();
                if (age >= sim_delay_s_) {
                    response_sample = *it;
                    break;
                }
            }
        }

        const Eigen::Vector3d response_zb = response_sample.attitude.toRotationMatrix().col(2);
        const double response_tilt_cos =
            std::clamp(response_zb.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
        const double measured_acc_z = thr2acc_true_ * response_sample.thrust_cmd *
                                          response_tilt_cos -
                                      gravity_ + noise_dist_(rng_);

        thrust_estimator::Input_t input;
        input.stamp = now;
        input.attitude = attitude;
        input.velocity_w = Eigen::Vector3d::Zero();
        input.acceleration_w = Eigen::Vector3d(0.0, 0.0, measured_acc_z);
        input.thrust_cmd = thrust_cmd;

        estimator_->update(input);
        ++sample_count_;

        if (sample_count_ % 100 == 0) {
            ROS_INFO("[hoverthrust_estimator_test] t=%.2f est=%.4f true=%.4f err=%.4f",
                     t,
                     estimator_->get_hover_thrust(),
                     true_hover_thrust_,
                     std::abs(estimator_->get_hover_thrust() - true_hover_thrust_));
        }

        if (t >= test_duration_s_) {
            timer_.stop();
            finalizeAndShutdown();
        }
    }

    void finalizeAndShutdown() {
        const double estimated_hover = estimator_->get_hover_thrust();
        const double abs_error = std::abs(estimated_hover - true_hover_thrust_);
        const bool pass = abs_error <= pass_abs_error_;

        if (pass) {
            ROS_INFO("[hoverthrust_estimator_test] PASS type=%s est=%.4f true=%.4f abs_error=%.4f (<= %.4f)",
                     estimator_type_.c_str(),
                     estimated_hover,
                     true_hover_thrust_,
                     abs_error,
                     pass_abs_error_);
            exit_code_ = 0;
        } else {
            ROS_ERROR("[hoverthrust_estimator_test] FAIL type=%s est=%.4f true=%.4f abs_error=%.4f (> %.4f)",
                      estimator_type_.c_str(),
                      estimated_hover,
                      true_hover_thrust_,
                      abs_error,
                      pass_abs_error_);
            exit_code_ = 1;
        }

        ros::requestShutdown();
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    std::unique_ptr<thrust_estimator::HoverThrustEstimator> estimator_;

    struct CommandSample {
        ros::Time stamp{ros::Time(0)};
        Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
        double thrust_cmd{0.0};
    };

    std::string estimator_type_{"rls"};
    double loop_hz_{100.0};
    double test_duration_s_{20.0};
    double gravity_{9.81};
    double true_hover_thrust_{0.45};
    double init_hover_thrust_{0.30};
    double max_tilt_deg_{15.0};
    double acc_noise_std_{0.08};
    double acc_cmd_amp_{0.6};
    double acc_cmd_freq_hz_{0.35};
    double pass_abs_error_{0.05};
    double sim_delay_s_{0.04};

    double thr2acc_true_{0.0};
    int sample_count_{0};
    int exit_code_{0};
    ros::Time start_time_{ros::Time(0)};
    std::deque<CommandSample> command_history_;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "hoverthrust_estimator_ros_test_node");
    ros::NodeHandle nh;
    HoverThrustEstimatorRosTestNode node(nh);
    ros::spin();
    return node.exitCode();
}
