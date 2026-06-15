#include <ros/ros.h>

#include <planner_msgs/EgoPositionCommand.h>
#include <sunray_msgs/UAVControlCMD.h>

#include <stdexcept>
#include <string>

namespace {

class Ego2Sunray {
public:
  Ego2Sunray() : private_nh_("~") {
    std::string input_topic = "/uav1/sunray/planning/ego_planner/position_cmd";
    std::string output_topic = "/uav1/sunray/uav_control/control_cmd";
    int queue_size = 50;
    bool use_yaw_rate = false;

    private_nh_.param<std::string>("input_topic", input_topic, input_topic);
    private_nh_.param<std::string>("output_topic", output_topic, output_topic);
    private_nh_.param("queue_size", queue_size, queue_size);
    private_nh_.param("use_yaw_rate", use_yaw_rate, use_yaw_rate);

    input_topic_ = input_topic;
    output_topic_ = output_topic;
    use_yaw_rate_ = use_yaw_rate;

    control_cmd_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>(output_topic_, queue_size);
    position_cmd_sub_ = nh_.subscribe(input_topic_, queue_size, &Ego2Sunray::positionCmdCallback, this);

    ROS_INFO_STREAM("Ego2Sunray subscribe: " << input_topic_);
    ROS_INFO_STREAM("Ego2Sunray publish:   " << output_topic_);
  }

private:
  void positionCmdCallback(const planner_msgs::EgoPositionCommand::ConstPtr& msg) {
    sunray_msgs::UAVControlCMD control_cmd;
    control_cmd.header = msg->header;
    control_cmd.cmd_source = sunray_msgs::UAVControlCMD::PLANNING;
    control_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY;

    control_cmd.desired_pos.x = msg->position.x;
    control_cmd.desired_pos.y = msg->position.y;
    control_cmd.desired_pos.z = msg->position.z;

    control_cmd.desired_vel = msg->velocity;
    control_cmd.desired_acc = msg->acceleration;

    control_cmd.yaw_mode = use_yaw_rate_
                               ? sunray_msgs::UAVControlCMD::SET_YAWRATE
                               : sunray_msgs::UAVControlCMD::SET_YAW;
    control_cmd.desired_yaw = static_cast<float>(msg->yaw);
    control_cmd.desired_yaw_rate = static_cast<float>(msg->yaw_dot);

    control_cmd_pub_.publish(control_cmd);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber position_cmd_sub_;
  ros::Publisher control_cmd_pub_;
  std::string input_topic_;
  std::string output_topic_;
  bool use_yaw_rate_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "Ego2Sunray_node");

  try {
    Ego2Sunray bridge;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("Ego2Sunray init failed: " << e.what());
    return 1;
  }

  return 0;
}
