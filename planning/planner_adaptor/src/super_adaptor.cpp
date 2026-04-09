#include <ros/ros.h>

#include <quadrotor_msgs/PositionCommand.h>
#include <sunray_msgs/UAVControlCMD.h>

class PositionCommandConverter {
  public:
    PositionCommandConverter(ros::NodeHandle& nh) {
        // ====== 硬编码话题名（你自己改这里）======
        sub_topic_ = "/position_cmd_in";
        pub_topic_ = "/uav_control_cmd_out";

        pub_ = nh.advertise<sunray_msgs::UAVControlCMD>(pub_topic_, 10);
        sub_ = nh.subscribe(sub_topic_, 10, &PositionCommandConverter::callback, this);

        ROS_INFO_STREAM("[pcmd_converter] Sub: " << sub_topic_ << " -> Pub: " << pub_topic_);
    }

  private:
    void callback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        sunray_msgs::UAVControlCMD out;

        // header
        out.header = msg->header;

        // 命令来源：代码控制
        out.cmd_source = sunray_msgs::UAVControlCMD::CONTROL_CMD;

        // 控制命令：轨迹跟踪（最贴合 PositionCommand）
        out.control_cmd = sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY;

        // 位置映射: Point -> Vector3
        out.desired_pos.x = msg->position.x;
        out.desired_pos.y = msg->position.y;
        out.desired_pos.z = msg->position.z;

        // 速度 / 加速度 / jerk
        out.desired_vel = msg->velocity;
        out.desired_acc = msg->acceleration;
        out.desired_jerk = msg->jerk;

        // yaw / yaw_rate
        out.desired_yaw = static_cast<float>(msg->yaw);
        out.desired_yaw_rate = static_cast<float>(msg->yaw_dot);

        // yaw mode:
        // 如果有明显 yaw_rate 则用 SET_YAWRATE，否则 SET_YAW
        if (std::fabs(msg->yaw_dot) > 1e-6)
            out.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAWRATE;
        else
            out.yaw_mode = sunray_msgs::UAVControlCMD::SET_YAW;

        // WGS84 在 PositionCommand 中没有，置零
        out.wgs84_pos.latitude = 0.0;
        out.wgs84_pos.longitude = 0.0;
        out.wgs84_pos.altitude = 0.0;

        // mode flag: 默认非固定高度
        out.fixed_height = false;

        pub_.publish(out);
    }

  private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string sub_topic_;
    std::string pub_topic_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "position_command_to_uav_control_cmd_node");
    ros::NodeHandle nh;

    PositionCommandConverter converter(nh);

    ros::spin();
    return 0;
}
