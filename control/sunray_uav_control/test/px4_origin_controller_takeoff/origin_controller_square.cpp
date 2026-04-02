#include <algorithm>
#include <array>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

#include "controller/px4_origin_controller.hpp"
#include "control_data_types/uav_state_estimate.hpp"
#include "control_data_types/controller_desired_types.hpp"

class OriginControllerSquareTest {
  public:
    OriginControllerSquareTest(ros::NodeHandle& nh) : nh_(nh), has_odom_(false), controller_(nh_) {
        ros::NodeHandle private_nh("~");
        odom_sub_ = nh_.subscribe(
            "/uav1/sunray/localization/local_odom", 10, &OriginControllerSquareTest::odomCb, this);

        private_nh.param("relative_takeoff_height", relative_takeoff_height_, 0.6);
        private_nh.param("max_takeoff_velocity", max_takeoff_velocity_, 0.6);
        private_nh.param("loop_hz", loop_hz_, 30.0);
        private_nh.param("square_side_length", square_side_length_, 2.0);
        private_nh.param("point_hold_time_s", point_hold_time_s_, 3.0);
        private_nh.param<std::string>(
            "move_frame", move_frame_, std::string("local"));  // local/body

        if (!controller_.init()) {
            ROS_ERROR("[origin_controller_square] controller init failed.");
            ros::shutdown();
            return;
        }

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / loop_hz_), &OriginControllerSquareTest::timerCb, this);
    }

  private:
    enum class Phase { Takeoff = 0, SquareMove, Done };

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        has_odom_ = true;
        control_common::UAVStateEstimate odom(*msg);
        current_odom_ = odom;
        controller_.set_current_odom(odom);
    }

    void buildLocalVerticesFromCenter(const Eigen::Vector3d& center, double yaw_rad) {
        const double h = square_side_length_ * 0.5;
        vertices_local_[0] = center + Eigen::Vector3d(h, h, 0.0);
        vertices_local_[1] = center + Eigen::Vector3d(h, -h, 0.0);
        vertices_local_[2] = center + Eigen::Vector3d(-h, -h, 0.0);
        vertices_local_[3] = center + Eigen::Vector3d(-h, h, 0.0);
        vertex_yaw_ = yaw_rad;
    }

    // 以起飞时 body 原点定义四个“绝对”body角点
    void buildBodyAbsoluteVertices() {
        const double h = square_side_length_ * 0.5;
        vertices_body_abs_[0] = Eigen::Vector3d(h, h, 0.0);
        vertices_body_abs_[1] = Eigen::Vector3d(h, -h, 0.0);
        vertices_body_abs_[2] = Eigen::Vector3d(-h, -h, 0.0);
        vertices_body_abs_[3] = Eigen::Vector3d(-h, h, 0.0);
    }

    // 使用“起飞时 body 坐标系”表达当前点，得到相对该固定body系的坐标
    Eigen::Vector3d currentPosInTakeoffBodyFrame() const {
        const Eigen::Vector3d d_w = current_odom_.position - square_center_w_;
        const double c = std::cos(square_yaw0_);
        const double s = std::sin(square_yaw0_);

        Eigen::Vector3d p_b0;
        // Rz(yaw0)^T * d_w
        p_b0.x() = c * d_w.x() + s * d_w.y();
        p_b0.y() = -s * d_w.x() + c * d_w.y();
        p_b0.z() = d_w.z();
        return p_b0;
    }

    void timerCb(const ros::TimerEvent&) {
        if (!has_odom_) {
            ROS_WARN_THROTTLE(1.0, "[origin_controller_square] waiting odom...");
            return;
        }

        if (!controller_.is_ready()) {
            ROS_WARN_THROTTLE(1.0, "[origin_controller_square] controller not ready.");
        }

        if (phase_ == Phase::Takeoff) {
            const bool takeoff_done =
                controller_.takeoff(relative_takeoff_height_, max_takeoff_velocity_);
            if (takeoff_done) {
                // 起飞完成时作为 square 参考
                square_center_w_ = current_odom_.position;
                square_yaw0_ = mavrosYaw();

                buildLocalVerticesFromCenter(square_center_w_, square_yaw0_);
                buildBodyAbsoluteVertices();

                current_vertex_idx_ = 0;
                hold_start_time_ = ros::Time(0);
                phase_ = Phase::SquareMove;

                ROS_INFO("[origin_controller_square] takeoff complete, start square, frame=%s",
                         move_frame_.c_str());
            }
            return;
        }

        if (phase_ == Phase::SquareMove) {
            controller_data_types::TargetPoint_t target;
            bool arrived = false;

            if (move_frame_ == "body") {
                // 关键：传给 move_point_body 的是“相对当前点偏移”
                // delta_b = p_target_abs_b0 - p_current_b0
                const Eigen::Vector3d p_cur_b0 = currentPosInTakeoffBodyFrame();
                Eigen::Vector3d delta_b = vertices_body_abs_[current_vertex_idx_] - p_cur_b0;
                delta_b.z() = 0.0;  // 方形平面运动

                target.position = delta_b;
                target.yaw = 0.0;  // 相对yaw增量=0，保持朝向
                arrived = controller_.move_point_body(target);
            } else {
                target.position = vertices_local_[current_vertex_idx_];
                target.yaw = vertex_yaw_;
                arrived = controller_.move_point(target);
            }

            if (arrived) {
                if (hold_start_time_ == ros::Time(0)) {
                    hold_start_time_ = ros::Time::now();
                    ROS_INFO("[origin_controller_square] arrived vertex %d, hold start.",
                             current_vertex_idx_);
                }

                if ((ros::Time::now() - hold_start_time_).toSec() >= point_hold_time_s_) {
                    ++current_vertex_idx_;
                    hold_start_time_ = ros::Time(0);

                    if (current_vertex_idx_ >= static_cast<int>(vertices_local_.size())) {
                        phase_ = Phase::Done;
                        ROS_INFO("[origin_controller_square] square finished.");
                    } else {
                        ROS_INFO("[origin_controller_square] switch to vertex %d.",
                                 current_vertex_idx_);
                    }
                }
            } else {
                hold_start_time_ = ros::Time(0);
            }
            return;
        }

        if (phase_ == Phase::Done) {
            ROS_INFO_THROTTLE(2.0, "[origin_controller_square] done.");
            // 可选：controller_.hover();
            return;
        }
    }

    double mavrosYaw() const {
        // 若你已有统一接口可直接替换
        // 这里只是复用 controller 内部风格，测试节点可不直接读helper
        // 简化：使用当前里程计姿态转换也可以，这里暂用0，建议改成你的真实yaw来源
        return 0.0;
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;

    PX4_OriginController controller_;
    bool has_odom_{false};
    control_common::UAVStateEstimate current_odom_;

    // params
    double relative_takeoff_height_{0.6};
    double max_takeoff_velocity_{0.6};
    double square_side_length_{2.0};
    double point_hold_time_s_{3.0};
    double loop_hz_{30.0};
    std::string move_frame_{"local"};  // local/body

    // state
    Phase phase_{Phase::Takeoff};
    int current_vertex_idx_{0};
    ros::Time hold_start_time_{ros::Time(0)};

    // local模式
    std::array<Eigen::Vector3d, 4> vertices_local_{};
    double vertex_yaw_{0.0};

    // body模式（以起飞时body系定义角点）
    std::array<Eigen::Vector3d, 4> vertices_body_abs_{};
    Eigen::Vector3d square_center_w_{Eigen::Vector3d::Zero()};
    double square_yaw0_{0.0};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "origin_controller_square_test");
    ros::NodeHandle nh;
    OriginControllerSquareTest node(nh);
    ros::spin();
    return 0;
}
