#include <string>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "utils/curve/quintic_curve.hpp"

namespace {

geometry_msgs::Vector3 to_ros_vector3(const Eigen::Vector3d &vec) {
  geometry_msgs::Vector3 out;
  out.x = vec.x();
  out.y = vec.y();
  out.z = vec.z();
  return out;
}

} // namespace

class QuinticCurveTestNode {
public:
  QuinticCurveTestNode() : nh_(), pnh_("~") {
    double start_x = 1.0;
    double start_y = 1.0;
    double start_z = 2.0;
    double end_x = 1.0;
    double end_y = 1.0;
    double end_z = 0.0;
    pnh_.param("start_x", start_x, start_x);
    pnh_.param("start_y", start_y, start_y);
    pnh_.param("start_z", start_z, start_z);
    pnh_.param("end_x", end_x, end_x);
    pnh_.param("end_y", end_y, end_y);
    pnh_.param("end_z", end_z, end_z);
    start_point_ = Eigen::Vector3d(start_x, start_y, start_z);
    end_point_ = Eigen::Vector3d(end_x, end_y, end_z);

    pnh_.param("keep_time", keep_time_, 8.0);
    if (keep_time_ <= 0.0) {
      ROS_WARN("[QuinticCurveTest] keep_time <= 0, force to 5.0");
      keep_time_ = 5.0;
    }

    pnh_.param("publish_hz", publish_hz_, 50.0);
    if (publish_hz_ <= 0.0) {
      ROS_WARN("[QuinticCurveTest] publish_hz <= 0, force to 50.0");
      publish_hz_ = 50.0;
    }

    pnh_.param("loop", loop_, false);
    pnh_.param("publish_initial_sample", publish_initial_sample_, true);
    pnh_.param("use_generate_test", use_generate_test_, true);
    pnh_.param("frame_id", frame_id_, std::string("map"));
    pnh_.param("topic_prefix", topic_prefix_, std::string("quintic_curve_test"));
    pnh_.param("start_delay_s", start_delay_s_, 0.0);
    if (start_delay_s_ < 0.0) {
      start_delay_s_ = 0.0;
    }

    position_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
        topic_prefix_ + "/position", 10);
    velocity_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        topic_prefix_ + "/velocity", 10);
    acceleration_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        topic_prefix_ + "/acceleration", 10);
    curve_status_pub_ =
        nh_.advertise<std_msgs::Bool>(topic_prefix_ + "/curve_status", 10);

    start_time_ros_ = ros::Time::now() + ros::Duration(start_delay_s_);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_hz_),
                             &QuinticCurveTestNode::timer_cb, this);

    if (publish_initial_sample_) {
      publish_sample(ros::Time::now(), start_time_ros_);
    }

    ROS_INFO(
        "[QuinticCurveTest] start=[%.3f %.3f %.3f], end=[%.3f %.3f %.3f], "
        "keep_time=%.3fs, publish_hz=%.1f, loop=%s",
        start_point_.x(), start_point_.y(), start_point_.z(), end_point_.x(),
        end_point_.y(), end_point_.z(), keep_time_, publish_hz_,
        loop_ ? "true" : "false");
    ROS_INFO(
        "[QuinticCurveTest] publishing to '%s/{position,velocity,acceleration,"
        "curve_status}'",
        topic_prefix_.c_str());
  }

private:
  void publish_sample(const ros::Time &stamp, const ros::Time &eval_time) {
    (void)use_generate_test_;
    const auto curve_state = uav_control::curve::evaluate_quintic_curve(
        start_point_, Eigen::Vector3d::Zero(), end_point_, Eigen::Vector3d::Zero(),
        start_time_ros_.toSec(), keep_time_, eval_time.toSec());

    const bool curve_status = curve_state.valid;
    const Eigen::Vector3d position = curve_state.position;
    const Eigen::Vector3d velocity = curve_state.velocity;
    const Eigen::Vector3d acceleration = curve_state.acceleration;

    geometry_msgs::PointStamped pos_msg;
    pos_msg.header.stamp = stamp;
    pos_msg.header.frame_id = frame_id_;
    pos_msg.point.x = position.x();
    pos_msg.point.y = position.y();
    pos_msg.point.z = position.z();

    geometry_msgs::Vector3Stamped vel_msg;
    vel_msg.header = pos_msg.header;
    vel_msg.vector = to_ros_vector3(velocity);

    geometry_msgs::Vector3Stamped acc_msg;
    acc_msg.header = pos_msg.header;
    acc_msg.vector = to_ros_vector3(acceleration);

    std_msgs::Bool status_msg;
    status_msg.data = curve_status;

    position_pub_.publish(pos_msg);
    velocity_pub_.publish(vel_msg);
    acceleration_pub_.publish(acc_msg);
    curve_status_pub_.publish(status_msg);

    ROS_INFO_THROTTLE(
        1.0,
        "[QuinticCurveTest] t=%.2fs pos=[%.3f %.3f %.3f] vel=[%.3f %.3f %.3f] "
        "acc=[%.3f %.3f %.3f]",
        (eval_time - start_time_ros_).toSec(), position.x(), position.y(),
        position.z(), velocity.x(), velocity.y(), velocity.z(),
        acceleration.x(), acceleration.y(), acceleration.z());
  }

  void timer_cb(const ros::TimerEvent &) {
    const ros::Time now = ros::Time::now();

    if (loop_ && (now - start_time_ros_).toSec() > keep_time_) {
      start_time_ros_ = now;
    }

    publish_sample(now, now);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  Eigen::Vector3d start_point_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_point_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  double keep_time_ = 5.0;
  double publish_hz_ = 50.0;
  double start_delay_s_ = 0.0;
  bool loop_ = false;
  bool publish_initial_sample_ = true;
  bool use_generate_test_ = true;

  std::string frame_id_ = "map";
  std::string topic_prefix_ = "quintic_curve_test";
  ros::Time start_time_ros_;

  ros::Publisher position_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher acceleration_pub_;
  ros::Publisher curve_status_pub_;
  ros::Timer timer_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "quintic_curve_test_node");
  QuinticCurveTestNode node;
  ros::spin();
  return 0;
}
