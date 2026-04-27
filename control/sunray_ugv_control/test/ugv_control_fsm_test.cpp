#include <gtest/gtest.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sunray_msgs/OdomStatus.h>
#include <sunray_msgs/UGVControlCMD.h>
#include <sunray_msgs/UGVControlFSMState.h>

#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "sunray_ugv_control/ugv_control_utils.h"
#include "sunray_ugv_control/ugv_controller.h"
#define private public
#include "sunray_ugv_control/ugv_control_fsm.h"
#undef private

namespace sunray_ugv_control {
namespace {

constexpr double kWaitStepSeconds = 0.01;

bool wait_until(const std::function<bool()>& predicate, const double timeout_seconds) {
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout_seconds);
    while (ros::ok() && ros::WallTime::now() < deadline) {
        ros::spinOnce();
        if (predicate()) {
            return true;
        }
        ros::WallDuration(kWaitStepSeconds).sleep();
    }
    return predicate();
}

nav_msgs::Odometry make_odom(const double x, const double y = 0.0) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.w = 1.0;
    return odom;
}

sunray_msgs::OdomStatus make_odom_status(const bool valid) {
    sunray_msgs::OdomStatus status;
    status.header.stamp = ros::Time::now();
    status.has_odometry = valid;
    status.odom_timeout = !valid;
    return status;
}

void publish_for(const ros::Publisher& publisher,
                 const nav_msgs::Odometry& message,
                 const double duration_seconds = 0.15) {
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(duration_seconds);
    while (ros::ok() && ros::WallTime::now() < deadline) {
        publisher.publish(message);
        ros::spinOnce();
        ros::WallDuration(kWaitStepSeconds).sleep();
    }
}

void publish_for(const ros::Publisher& publisher,
                 const sunray_msgs::OdomStatus& message,
                 const double duration_seconds = 0.15) {
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(duration_seconds);
    while (ros::ok() && ros::WallTime::now() < deadline) {
        publisher.publish(message);
        ros::spinOnce();
        ros::WallDuration(kWaitStepSeconds).sleep();
    }
}

void publish_for(const ros::Publisher& publisher,
                 const sunray_msgs::UGVControlCMD& message,
                 const double duration_seconds = 0.15) {
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(duration_seconds);
    while (ros::ok() && ros::WallTime::now() < deadline) {
        publisher.publish(message);
        ros::spinOnce();
        ros::WallDuration(kWaitStepSeconds).sleep();
    }
}

bool is_zero_twist(const geometry_msgs::Twist& twist) {
    constexpr double eps = 1e-9;
    return std::fabs(twist.linear.x) < eps && std::fabs(twist.linear.y) < eps &&
           std::fabs(twist.linear.z) < eps && std::fabs(twist.angular.x) < eps &&
           std::fabs(twist.angular.y) < eps && std::fabs(twist.angular.z) < eps;
}

class CmdVelRecorder {
  public:
    void callback(const geometry_msgs::Twist::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        messages_.push_back(*msg);
    }

    size_t count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return messages_.size();
    }

    bool has_message_after(const size_t start_index,
                           const std::function<bool(const geometry_msgs::Twist&)>& predicate) const {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t index = start_index; index < messages_.size(); ++index) {
            if (predicate(messages_[index])) {
                return true;
            }
        }
        return false;
    }

  private:
    mutable std::mutex mutex_;
    std::vector<geometry_msgs::Twist> messages_;
};

class FSMStateRecorder {
  public:
    void callback(const sunray_msgs::UGVControlFSMState::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = *msg;
        received_ = true;
    }

    bool latest(sunray_msgs::UGVControlFSMState& output) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!received_) {
            return false;
        }
        output = latest_;
        return true;
    }

  private:
    mutable std::mutex mutex_;
    sunray_msgs::UGVControlFSMState latest_;
    bool received_{false};
};

class UGVControlFSMTest : public testing::Test {
  protected:
    void SetUp() override {
        ros::NodeHandle private_nh("~");
        private_nh.setParam("ugv_name", "ugvtest");
        private_nh.setParam("ugv_id", next_ugv_id_++);
    }

    static std::string current_namespace() {
        ros::NodeHandle private_nh("~");
        std::string ugv_name;
        int ugv_id = 0;
        private_nh.getParam("ugv_name", ugv_name);
        private_nh.getParam("ugv_id", ugv_id);
        return "/" + ugv_name + std::to_string(ugv_id);
    }

  private:
    static int next_ugv_id_;
};

int UGVControlFSMTest::next_ugv_id_ = 41;

TEST_F(UGVControlFSMTest, UnsupportedMoveCommandPublishesZeroAfterPreviousNonZeroCommand) {
    ros::NodeHandle nh;
    UGVControlFSM fsm(nh);
    fsm.init();

    const std::string ns = current_namespace();
    CmdVelRecorder recorder;
    ros::Subscriber cmd_vel_sub =
        nh.subscribe(ns + "/cmd_vel", 20, &CmdVelRecorder::callback, &recorder);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(
        ns + "/sunray/localization/local_odom", 10);
    ros::Publisher cmd_pub = nh.advertise<sunray_msgs::UGVControlCMD>(
        ns + "/sunray/ugv_control/control_cmd", 10);

    ASSERT_TRUE(wait_until([&] { return odom_pub.getNumSubscribers() > 0; }, 2.0));
    ASSERT_TRUE(wait_until([&] { return cmd_pub.getNumSubscribers() > 0; }, 2.0));
    ASSERT_TRUE(wait_until([&] { return cmd_vel_sub.getNumPublishers() > 0; }, 2.0));

    publish_for(odom_pub, make_odom(0.0));

    sunray_msgs::UGVControlCMD body_cmd;
    body_cmd.header.stamp = ros::Time::now();
    body_cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_VELOCITY_BODY;
    body_cmd.desired_linear.x = 0.5;
    publish_for(cmd_pub, body_cmd);

    ASSERT_TRUE(wait_until(
        [&] {
            return recorder.has_message_after(0, [](const geometry_msgs::Twist& twist) {
                return twist.linear.x > 0.4;
            });
        },
        2.0));

    const size_t after_nonzero = recorder.count();
    sunray_msgs::UGVControlCMD wgs84_cmd;
    wgs84_cmd.header.stamp = ros::Time::now();
    wgs84_cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_WGS84;
    publish_for(cmd_pub, wgs84_cmd);

    EXPECT_TRUE(wait_until(
        [&] {
            return recorder.has_message_after(after_nonzero, [](const geometry_msgs::Twist& twist) {
                return is_zero_twist(twist);
            });
        },
        1.0));
}

TEST_F(UGVControlFSMTest, CurrentPoseHomeWaitsForValidOdometryStatus) {
    ros::NodeHandle nh;
    UGVControlFSM fsm(nh);
    fsm.init();

    const std::string ns = current_namespace();
    FSMStateRecorder state_recorder;
    ros::Subscriber state_sub = nh.subscribe(
        ns + "/sunray/ugv_control/ugv_control_fsm_state",
        10,
        &FSMStateRecorder::callback,
        &state_recorder);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(
        ns + "/sunray/localization/local_odom", 10);
    ros::Publisher status_pub = nh.advertise<sunray_msgs::OdomStatus>(
        ns + "/sunray/localization/odom_status", 10);

    ASSERT_TRUE(wait_until([&] { return odom_pub.getNumSubscribers() > 0; }, 2.0));
    ASSERT_TRUE(wait_until([&] { return status_pub.getNumSubscribers() > 0; }, 2.0));
    ASSERT_TRUE(wait_until([&] { return state_sub.getNumPublishers() > 0; }, 2.0));

    publish_for(odom_pub, make_odom(10.0));
    fsm.process();

    sunray_msgs::UGVControlFSMState state_msg;
    ASSERT_TRUE(wait_until([&] { return state_recorder.latest(state_msg); }, 2.0));
    EXPECT_NEAR(state_msg.home_point.x, 0.0, 1e-9);

    publish_for(status_pub, make_odom_status(false));
    publish_for(odom_pub, make_odom(11.0));
    fsm.process();

    ASSERT_TRUE(wait_until([&] { return state_recorder.latest(state_msg); }, 2.0));
    EXPECT_NEAR(state_msg.home_point.x, 0.0, 1e-9);

    publish_for(status_pub, make_odom_status(true));
    publish_for(odom_pub, make_odom(2.0));
    fsm.process();

    ASSERT_TRUE(wait_until(
        [&] {
            return state_recorder.latest(state_msg) && std::fabs(state_msg.home_point.x - 2.0) < 1e-9;
        },
        2.0));
}

TEST_F(UGVControlFSMTest, MovePointAutomaticallySwitchesToHoldWhenTargetReached) {
    ros::NodeHandle nh;
    UGVControlFSM fsm(nh);
    fsm.init();

    const std::string ns = current_namespace();
    FSMStateRecorder state_recorder;
    ros::Subscriber state_sub = nh.subscribe(
        ns + "/sunray/ugv_control/ugv_control_fsm_state",
        10,
        &FSMStateRecorder::callback,
        &state_recorder);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(
        ns + "/sunray/localization/local_odom", 10);
    ros::Publisher status_pub = nh.advertise<sunray_msgs::OdomStatus>(
        ns + "/sunray/localization/odom_status", 10);
    ros::Publisher cmd_pub = nh.advertise<sunray_msgs::UGVControlCMD>(
        ns + "/sunray/ugv_control/control_cmd", 10);

    ASSERT_TRUE(wait_until([&] { return odom_pub.getNumSubscribers() > 0; }, 2.0));
    ASSERT_TRUE(wait_until([&] { return status_pub.getNumSubscribers() > 0; }, 2.0));
    ASSERT_TRUE(wait_until([&] { return cmd_pub.getNumSubscribers() > 0; }, 2.0));
    ASSERT_TRUE(wait_until([&] { return state_sub.getNumPublishers() > 0; }, 2.0));

    publish_for(status_pub, make_odom_status(true));
    publish_for(odom_pub, make_odom(0.0));

    sunray_msgs::UGVControlCMD point_cmd;
    point_cmd.control_cmd = sunray_msgs::UGVControlCMD::MOVE_POINT;
    point_cmd.desired_pos.x = 1.0;
    point_cmd.desired_pos.y = 0.0;
    point_cmd.desired_yaw = 0.0;
    publish_for(cmd_pub, point_cmd);

    sunray_msgs::UGVControlFSMState state_msg;
    ASSERT_TRUE(wait_until(
        [&] {
            fsm.process();
            return state_recorder.latest(state_msg) &&
                   state_msg.fsm_state == sunray_msgs::UGVControlFSMState::FSM_MOVE &&
                   state_msg.active_control_cmd.control_cmd == sunray_msgs::UGVControlCMD::MOVE_POINT;
        },
        0.8));

    publish_for(odom_pub, make_odom(1.0));

    EXPECT_TRUE(wait_until(
        [&] {
            fsm.process();
            return state_recorder.latest(state_msg) &&
                   state_msg.fsm_state == sunray_msgs::UGVControlFSMState::FSM_HOLD &&
                   state_msg.active_control_cmd.control_cmd == sunray_msgs::UGVControlCMD::HOLD;
        },
        0.8));
}

TEST_F(UGVControlFSMTest, MovePointTimerTransitionsToHoldWhenReachedBeforeCommandTimeout) {
    ros::NodeHandle nh;
    UGVControlFSM fsm(nh);
    fsm.init();

    sunray_msgs::OdomStatus::Ptr valid_status(new sunray_msgs::OdomStatus(make_odom_status(true)));
    fsm.odom_status_callback(valid_status);

    nav_msgs::Odometry::Ptr start_odom(new nav_msgs::Odometry(make_odom(0.0)));
    fsm.odom_callback(start_odom);

    sunray_msgs::UGVControlCMD::Ptr point_cmd(new sunray_msgs::UGVControlCMD);
    point_cmd->header.stamp = ros::Time::now();
    point_cmd->control_cmd = sunray_msgs::UGVControlCMD::MOVE_POINT;
    point_cmd->desired_pos.x = 1.0;
    point_cmd->desired_pos.y = 0.0;
    point_cmd->desired_yaw = 0.0;
    fsm.control_cmd_callback(point_cmd);

    ASSERT_EQ(fsm.state_, UGVControlFSM::State::MOVE);

    fsm.control_timer_callback(ros::TimerEvent{});
    ASSERT_EQ(fsm.state_, UGVControlFSM::State::MOVE);

    nav_msgs::Odometry::Ptr reached_odom(new nav_msgs::Odometry(make_odom(1.0)));
    fsm.odom_callback(reached_odom);
    fsm.control_timer_callback(ros::TimerEvent{});

    EXPECT_EQ(fsm.state_, UGVControlFSM::State::HOLD);
    EXPECT_EQ(fsm.active_cmd_.control_cmd, sunray_msgs::UGVControlCMD::HOLD);
}

TEST_F(UGVControlFSMTest, MovePointSingleShotDoesNotExpireBeforeTargetReached) {
    ros::NodeHandle nh;
    UGVControlFSM fsm(nh);
    fsm.init();
    fsm.config_.timeout.wait_poscmd_time = 0.01;

    sunray_msgs::OdomStatus::Ptr valid_status(new sunray_msgs::OdomStatus(make_odom_status(true)));
    fsm.odom_status_callback(valid_status);

    nav_msgs::Odometry::Ptr start_odom(new nav_msgs::Odometry(make_odom(0.0)));
    fsm.odom_callback(start_odom);

    sunray_msgs::UGVControlCMD::Ptr point_cmd(new sunray_msgs::UGVControlCMD);
    point_cmd->header.stamp = ros::Time::now() - ros::Duration(1.0);
    point_cmd->control_cmd = sunray_msgs::UGVControlCMD::MOVE_POINT;
    point_cmd->desired_pos.x = 1.0;
    point_cmd->desired_pos.y = 0.0;
    point_cmd->desired_yaw = 0.0;
    fsm.control_cmd_callback(point_cmd);

    fsm.process();

    EXPECT_EQ(fsm.state_, UGVControlFSM::State::MOVE);
    EXPECT_EQ(fsm.active_cmd_.control_cmd, sunray_msgs::UGVControlCMD::MOVE_POINT);
}

TEST_F(UGVControlFSMTest, VelocityCommandStillExpiresWhenNotRefreshed) {
    ros::NodeHandle nh;
    UGVControlFSM fsm(nh);
    fsm.init();
    fsm.config_.timeout.wait_velcmd_time = 0.01;

    sunray_msgs::OdomStatus::Ptr valid_status(new sunray_msgs::OdomStatus(make_odom_status(true)));
    fsm.odom_status_callback(valid_status);

    nav_msgs::Odometry::Ptr start_odom(new nav_msgs::Odometry(make_odom(0.0)));
    fsm.odom_callback(start_odom);

    sunray_msgs::UGVControlCMD::Ptr velocity_cmd(new sunray_msgs::UGVControlCMD);
    velocity_cmd->header.stamp = ros::Time::now() - ros::Duration(1.0);
    velocity_cmd->control_cmd = sunray_msgs::UGVControlCMD::MOVE_VELOCITY;
    velocity_cmd->desired_vel.x = 0.5;
    velocity_cmd->desired_yaw = 0.0;
    fsm.control_cmd_callback(velocity_cmd);

    fsm.process();

    EXPECT_EQ(fsm.state_, UGVControlFSM::State::HOLD);
    EXPECT_EQ(fsm.active_cmd_.control_cmd, sunray_msgs::UGVControlCMD::HOLD);
}

}  // namespace
}  // namespace sunray_ugv_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "ugv_control_fsm_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
