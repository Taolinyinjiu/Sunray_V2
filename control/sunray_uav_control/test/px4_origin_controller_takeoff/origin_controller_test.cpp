#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "controller/px4_origin_controller.hpp"  // 按实际路径调整
#include "control_data_types/uav_state_estimate.hpp"
class OriginControllerTakeoffTest {
  public:
    OriginControllerTakeoffTest(ros::NodeHandle& nh) : nh_(nh), has_odom_(false), controller_(nh_) {
        odom_sub_ = nh_.subscribe(
            "/uav1/sunray/localization/local_odom", 10, &OriginControllerTakeoffTest::odomCb, this);

        nh_.param("relative_takeoff_height", relative_takeoff_height_, 0.6);
        nh_.param("max_takeoff_velocity", max_takeoff_velocity_, 0.6);
        nh_.param("loop_hz", loop_hz_, 30.0);

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
        if (has_land_ == false) {
            const bool done = controller_.takeoff(relative_takeoff_height_, max_takeoff_velocity_);
            if (done) {
                ROS_INFO("[origin_controller_test] takeoff complete.");
                if (takeoff_success_time == ros::Time(0))
                    takeoff_success_time = ros::Time::now();
                // 可选：继续悬停测试
                // controller_.hover();
            } else {
                ROS_WARN_THROTTLE(1.0, "error.");
            }
        }
        if (takeoff_success_time != ros::Time(0)) {
            if ((ros::Time::now() - takeoff_success_time).toSec() > 5) {
                has_land_ = true;
                const bool land_state = controller_.land(0, 0.2);
                if (land_state) {
                    ROS_INFO("[origin_controller_test] land complete.");
                } else {
                    ROS_WARN_THROTTLE(1.0, "land error.");
                }
            }
        }
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;

    ros::Time takeoff_success_time{ros::Time(0)};

    PX4_OriginController controller_;
    bool has_odom_;
    bool has_land_;
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
