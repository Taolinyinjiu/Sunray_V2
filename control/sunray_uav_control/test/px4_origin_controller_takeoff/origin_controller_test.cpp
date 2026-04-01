#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "controller/px4_origin_controller.hpp"  // 按实际路径调整
#include "control_data_types/uav_state_estimate.hpp"
class OriginControllerTakeoffTest {
  public:
    OriginControllerTakeoffTest(ros::NodeHandle& nh) : nh_(nh), has_odom_(false), controller_(nh_) {
        ros::NodeHandle private_nh("~");
        odom_sub_ = nh_.subscribe(
            "/uav1/sunray/localization/local_odom", 10, &OriginControllerTakeoffTest::odomCb, this);

        private_nh.param("relative_takeoff_height", relative_takeoff_height_, 0.6);
        private_nh.param("max_takeoff_velocity", max_takeoff_velocity_, 0.6);
        private_nh.param("loop_hz", loop_hz_, 30.0);
        private_nh.param("takeoff_land_cycles", takeoff_land_cycles_, 1);
        takeoff_land_cycles_ = std::max(1, takeoff_land_cycles_);
        ROS_INFO("[origin_controller_test] configured takeoff_land_cycles: %d", takeoff_land_cycles_);

        if (!controller_.init()) {
            ROS_ERROR("[origin_controller_test] controller init failed.");
            ros::shutdown();
            return;
        }

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / loop_hz_), &OriginControllerTakeoffTest::timerCb, this);
    }

  private:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        has_odom_ = true;
        control_common::UAVStateEstimate odom(*msg);

        controller_.set_current_odom(odom);  // 若接口不同，请替换
    }

    void timerCb(const ros::TimerEvent&) {
        if (!has_odom_) {
            ROS_WARN_THROTTLE(1.0, "[origin_controller_test] waiting odom...");
            return;
        } else {
            ROS_WARN_THROTTLE(1.0, "[origin_controller_test] recive odom...");
        }
        bool controller_state = controller_.is_ready();
        if (controller_state) {
            ROS_WARN_THROTTLE(1.0, "controller is ok.");
        } else {
            ROS_WARN_THROTTLE(1.0, "controller is error.");
        }

        if (completed_cycles_ >= takeoff_land_cycles_) {
            ROS_INFO_THROTTLE(2.0,
                              "[origin_controller_test] all cycles completed: %d/%d",
                              completed_cycles_,
                              takeoff_land_cycles_);
            return;
        }

        if (in_land_phase_) {
            const bool land_state = controller_.land(0, 0.2);
            if (land_state) {
                ++completed_cycles_;
                in_land_phase_ = false;
                takeoff_success_time_ = ros::Time(0);
                ROS_INFO("[origin_controller_test] cycle %d/%d land complete.",
                         completed_cycles_,
                         takeoff_land_cycles_);
            } else {
                ROS_WARN_THROTTLE(1.0, "land error.");
            }
            return;
        }

        const bool takeoff_done = controller_.takeoff(relative_takeoff_height_, max_takeoff_velocity_);
        if (takeoff_done) {
            if (takeoff_success_time_ == ros::Time(0)) {
                takeoff_success_time_ = ros::Time::now();
                ROS_INFO("[origin_controller_test] cycle %d/%d takeoff complete.",
                         completed_cycles_ + 1,
                         takeoff_land_cycles_);
            }
            // 保持当前策略：起飞完成后等待5秒再进入降落
            if ((ros::Time::now() - takeoff_success_time_).toSec() > 5.0) {
                in_land_phase_ = true;
                ROS_INFO("[origin_controller_test] cycle %d/%d start landing.",
                         completed_cycles_ + 1,
                         takeoff_land_cycles_);
            }
        } else {
            ROS_WARN_THROTTLE(1.0, "error.");
        }
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;

    ros::Time takeoff_success_time_{ros::Time(0)};

    PX4_OriginController controller_;
    bool has_odom_;
    bool in_land_phase_{false};
    int takeoff_land_cycles_{1};
    int completed_cycles_{0};
    double relative_takeoff_height_{1.5};
    double max_takeoff_velocity_{0.6};
    double loop_hz_{30.0};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "origin_controller_takeoff_test");
    ros::NodeHandle nh;
    OriginControllerTakeoffTest node(nh);
    ros::spin();
    return 0;
}
