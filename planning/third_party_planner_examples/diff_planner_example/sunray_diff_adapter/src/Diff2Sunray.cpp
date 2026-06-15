#include <ros/ros.h>

#include <planner_msgs/DiffPositionCommand.h>
#include <sunray_msgs/UAVControlCMD.h>

#include <string>

ros::Publisher g_control_cmd_pub;
bool g_use_yaw_rate = false;
bool g_ready_only = true;

void diffPositionCallback(const planner_msgs::DiffPositionCommand::ConstPtr& msg) {
    // Diff planner 会通过 trajectory_flag 给出轨迹状态。
    // 默认只转发 READY 状态，避免空命令或异常命令直接进入控制器。
    if (g_ready_only &&
        msg->trajectory_flag != planner_msgs::DiffPositionCommand::TRAJECTORY_STATUS_READY) {
        return;
    }

    sunray_msgs::UAVControlCMD control_cmd;

    // 保留原始时间戳和坐标系，便于控制器和调试工具对齐。
    control_cmd.header = msg->header;

    // 命令来源明确标记为规划模块，控制模式使用统一的轨迹跟踪接口。
    control_cmd.cmd_source = sunray_msgs::UAVControlCMD::PLANNING;
    control_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY;

    // 位置、速度、加速度、jerk 均直接映射。
    control_cmd.desired_pos.x = msg->position.x;
    control_cmd.desired_pos.y = msg->position.y;
    control_cmd.desired_pos.z = msg->position.z;
    control_cmd.desired_vel = msg->velocity;
    control_cmd.desired_acc = msg->acceleration;
    control_cmd.desired_jerk = msg->jerk;

    // 两种偏航控制数据都填好，最终由 yaw_mode 决定控制器采用哪一组。
    control_cmd.yaw_mode = g_use_yaw_rate ? sunray_msgs::UAVControlCMD::SET_YAWRATE
                                          : sunray_msgs::UAVControlCMD::SET_YAW;
    control_cmd.desired_yaw = static_cast<float>(msg->yaw);
    control_cmd.desired_yaw_rate = static_cast<float>(msg->yaw_dot);

    g_control_cmd_pub.publish(control_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Diff2Sunray_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string input_topic = "/uav1/pos_cmd";
    std::string output_topic = "/uav1/sunray/uav_control/control_cmd";
    int queue_size = 50;

    private_nh.param<std::string>("input_topic", input_topic, input_topic);
    private_nh.param<std::string>("output_topic", output_topic, output_topic);
    private_nh.param("queue_size", queue_size, queue_size);
    private_nh.param("use_yaw_rate", g_use_yaw_rate, false);
    private_nh.param("ready_only", g_ready_only, true);

    g_control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(output_topic, queue_size);
    ros::Subscriber diff_cmd_sub = nh.subscribe(input_topic, queue_size, diffPositionCallback);

    // 保留订阅对象，避免离开作用域后析构导致回调失效。
    (void)diff_cmd_sub;

    ROS_INFO_STREAM("Diff2Sunray subscribe: " << input_topic);
    ROS_INFO_STREAM("Diff2Sunray publish:   " << output_topic);
    ROS_INFO_STREAM("Diff2Sunray use_yaw_rate: " << (g_use_yaw_rate ? "true" : "false"));
    ROS_INFO_STREAM("Diff2Sunray ready_only:   " << (g_ready_only ? "true" : "false"));

    ros::spin();
    return 0;
}
