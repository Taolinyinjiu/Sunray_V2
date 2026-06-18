#include <gtest/gtest.h>

#include <limits>

#include "odom_filter/odom_kalman_filter.hpp"

namespace {

control_common::UAVStateEstimate make_odom(const double t,
                                           const Eigen::Vector3d& position,
                                           const Eigen::Vector3d& velocity) {
    control_common::UAVStateEstimate odom;
    odom.timestamp = ros::Time(t);
    odom.position = position;
    odom.velocity = velocity;
    odom.orientation = Eigen::Quaterniond::Identity();
    odom.bodyrate = Eigen::Vector3d::Zero();
    return odom;
}

control_common::OdomKalmanFilterParam default_params() {
    control_common::OdomKalmanFilterParam params;
    params.process_noise_acc = 0.5;
    params.meas_noise_pos = 0.05;
    params.meas_noise_vel = 0.10;
    return params;
}

TEST(OdomKalmanFilterTest, FirstFrameInitializesPositionAndVelocity) {
    control_common::OdomKalmanFilter filter;
    filter.init(default_params());

    const auto output =
        filter.update(make_odom(1.0,
                                Eigen::Vector3d(1.0, 2.0, 3.0),
                                Eigen::Vector3d(0.2, -0.1, 0.3)),
                      0.0);

    EXPECT_TRUE(filter.is_initialized());
    EXPECT_TRUE(filter.last_update_info().reset_this_update);
    EXPECT_TRUE(filter.last_update_info().velocity_valid);
    EXPECT_NEAR((output.position - Eigen::Vector3d(1.0, 2.0, 3.0)).norm(), 0.0, 1e-12);
    EXPECT_NEAR((output.velocity - Eigen::Vector3d(0.2, -0.1, 0.3)).norm(), 0.0, 1e-12);
}

TEST(OdomKalmanFilterTest, FirstFrameInvalidVelocityInitializesVelocityToZero) {
    auto raw = make_odom(1.0,
                         Eigen::Vector3d(1.0, 2.0, 3.0),
                         Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));

    control_common::OdomKalmanFilter filter;
    filter.init(default_params());
    const auto output = filter.update(raw, 0.0);

    EXPECT_TRUE(filter.is_initialized());
    EXPECT_FALSE(filter.last_update_info().velocity_valid);
    EXPECT_NEAR((output.position - raw.position).norm(), 0.0, 1e-12);
    EXPECT_NEAR(output.velocity.norm(), 0.0, 1e-12);
}

TEST(OdomKalmanFilterTest, InvalidFirstPositionDoesNotInitializeToFakeZero) {
    auto raw = make_odom(1.0,
                         Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
                         Eigen::Vector3d(1.0, 0.0, 0.0));

    control_common::OdomKalmanFilter filter;
    filter.init(default_params());
    const auto output = filter.update(raw, 0.0);

    EXPECT_FALSE(filter.is_initialized());
    EXPECT_TRUE(filter.last_update_info().reset_this_update);
    EXPECT_TRUE(std::isnan(output.position.x()));
}

TEST(OdomKalmanFilterTest, SinglePositionJumpUsesMahalanobisGateAndVelocityOnlyUpdate) {
    control_common::OdomKalmanFilter filter;
    filter.init(default_params());

    filter.update(make_odom(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()), 0.0);
    const auto output =
        filter.update(make_odom(1.01,
                                Eigen::Vector3d(5.0, 0.0, 0.0),
                                Eigen::Vector3d(0.2, 0.0, 0.0)),
                      0.01);

    const auto& info = filter.last_update_info();
    EXPECT_TRUE(info.position_jump);
    EXPECT_TRUE(info.velocity_valid);
    EXPECT_GT(info.position_mahalanobis_distance, info.position_mahalanobis_gate);
    EXPECT_EQ(info.consecutive_position_jump_count, 1);
    EXPECT_LT(output.position.x(), 1.0);
    EXPECT_GT(output.velocity.x(), 0.0);
}

TEST(OdomKalmanFilterTest, ConsecutivePositionJumpResetsToNewMeasurement) {
    control_common::OdomKalmanFilter filter;
    filter.init(default_params());

    filter.update(make_odom(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()), 0.0);
    for (int i = 1; i < 5; ++i) {
        filter.update(make_odom(1.0 + 0.01 * i,
                                Eigen::Vector3d(5.0 + 0.1 * i, 0.0, 0.0),
                                Eigen::Vector3d::Zero()),
                      0.01);
        EXPECT_TRUE(filter.last_update_info().position_jump);
        EXPECT_FALSE(filter.last_update_info().reset_this_update);
    }

    const auto output =
        filter.update(make_odom(1.05,
                                Eigen::Vector3d(5.5, 0.0, 0.0),
                                Eigen::Vector3d::Zero()),
                      0.01);

    EXPECT_TRUE(filter.last_update_info().reset_this_update);
    EXPECT_EQ(filter.last_update_info().consecutive_position_jump_count, 0);
    EXPECT_NEAR(output.position.x(), 5.5, 1e-12);
}

TEST(OdomKalmanFilterTest, InvalidDtReinitializesFromCurrentMeasurement) {
    control_common::OdomKalmanFilter filter;
    filter.init(default_params());

    filter.update(make_odom(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()), 0.0);
    const auto output =
        filter.update(make_odom(2.0,
                                Eigen::Vector3d(2.0, 0.0, 0.0),
                                Eigen::Vector3d(0.1, 0.0, 0.0)),
                      1.0);

    EXPECT_TRUE(filter.last_update_info().reset_this_update);
    EXPECT_NEAR(output.position.x(), 2.0, 1e-12);
    EXPECT_NEAR(output.velocity.x(), 0.1, 1e-12);
}

TEST(OdomKalmanFilterTest, JumpWithInvalidVelocityKeepsPredictedState) {
    control_common::OdomKalmanFilter filter;
    filter.init(default_params());

    filter.update(make_odom(1.0,
                            Eigen::Vector3d::Zero(),
                            Eigen::Vector3d(0.1, 0.0, 0.0)),
                  0.0);
    const auto output =
        filter.update(make_odom(1.01,
                                Eigen::Vector3d(5.0, 0.0, 0.0),
                                Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)),
                      0.01);

    EXPECT_TRUE(filter.last_update_info().position_jump);
    EXPECT_FALSE(filter.last_update_info().velocity_valid);
    EXPECT_NEAR(output.position.x(), 0.001, 1e-6);
    EXPECT_NEAR(output.velocity.x(), 0.1, 1e-6);
}

TEST(OdomKalmanFilterTest, OrientationAndBodyrateArePassedThrough) {
    control_common::OdomKalmanFilter filter;
    filter.init(default_params());

    auto odom = make_odom(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    odom.orientation = Eigen::Quaterniond(2.0, 0.0, 0.0, 0.0);
    odom.bodyrate = Eigen::Vector3d(0.1, 0.2, 0.3);
    const auto output = filter.update(odom, 0.0);

    EXPECT_NEAR(output.orientation.w(), 2.0, 1e-12);
    EXPECT_NEAR((output.bodyrate - odom.bodyrate).norm(), 0.0, 1e-12);
}

TEST(OdomKalmanFilterTest, ResetMakesNextFrameBehaveLikeFirstUpdate) {
    control_common::OdomKalmanFilter filter;
    filter.init(default_params());

    filter.update(make_odom(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()), 0.0);
    filter.reset();
    const auto output =
        filter.update(make_odom(2.0,
                                Eigen::Vector3d(3.0, 0.0, 0.0),
                                Eigen::Vector3d(0.4, 0.0, 0.0)),
                      0.01);

    EXPECT_TRUE(filter.last_update_info().reset_this_update);
    EXPECT_NEAR(output.position.x(), 3.0, 1e-12);
    EXPECT_NEAR(output.velocity.x(), 0.4, 1e-12);
}

TEST(OdomKalmanFilterTest, LongRunKeepsFiniteSymmetricCovariance) {
    control_common::OdomKalmanFilter filter;
    filter.init(default_params());

    filter.update(make_odom(1.0,
                            Eigen::Vector3d::Zero(),
                            Eigen::Vector3d(0.1, 0.0, 0.0)),
                  0.0);
    for (int i = 1; i <= 2000; ++i) {
        const double t = 1.0 + 0.01 * i;
        const Eigen::Vector3d pos(0.001 * i, 0.0, 0.0);
        filter.update(make_odom(t, pos, Eigen::Vector3d(0.1, 0.0, 0.0)), 0.01);
    }

    EXPECT_TRUE(filter.state().allFinite());
    EXPECT_TRUE(filter.covariance().allFinite());
    EXPECT_LT((filter.covariance() - filter.covariance().transpose()).norm(), 1e-9);
    EXPECT_GT(filter.covariance().diagonal().minCoeff(), 0.0);
}

}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
