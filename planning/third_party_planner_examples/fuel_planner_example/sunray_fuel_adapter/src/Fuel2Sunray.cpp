#include <ros/ros.h>

#include <planner_msgs/FuelPositionCommand.h>
#include <sunray_msgs/UAVControlCMD.h>

#include <string>

ros::Publisher g_control_cmd_pub;
bool g_use_yaw_rate = false;
bool g_ready_only = true;

void fuelPositionCallback(const planner_msgs::FuelPositionCommand::ConstPtr& msg) {
    // FUEL 会通过 trajectory_flag 标识当前轨迹命令是否合法、是否已准备好。
    // 这里默认只放行 READY 状态，避免空轨迹或异常轨迹直接进入控制器。
    if (g_ready_only &&
        msg->trajectory_flag != planner_msgs::FuelPositionCommand::TRAJECTORY_STATUS_READY) {
        return;
    }

    sunray_msgs::UAVControlCMD control_cmd;

    // 直接复用 FUEL 的消息头，保留时间戳和坐标系信息。
    control_cmd.header = msg->header;

    // 明确告诉下游控制器：该命令来自规划模块，控制模式为轨迹跟踪。
    control_cmd.cmd_source = sunray_msgs::UAVControlCMD::PLANNING;
    control_cmd.control_cmd = sunray_msgs::UAVControlCMD::MOVE_TRAJECTORY;

    // 位置、速度、加速度是一一对应的直接映射。
    control_cmd.desired_pos.x = msg->position.x;
    control_cmd.desired_pos.y = msg->position.y;
    control_cmd.desired_pos.z = msg->position.z;
    control_cmd.desired_vel = msg->velocity;
    control_cmd.desired_acc = msg->acceleration;

    // Sunray 控制器支持两种偏航控制方式：
    // 1. SET_YAW：直接给期望 yaw
    // 2. SET_YAWRATE：直接给期望 yaw_rate
    // 这里两组数据都填充，最终实际采用哪一组由 yaw_mode 决定。
    control_cmd.yaw_mode = g_use_yaw_rate ? sunray_msgs::UAVControlCMD::SET_YAWRATE
                                          : sunray_msgs::UAVControlCMD::SET_YAW;
    control_cmd.desired_yaw = static_cast<float>(msg->yaw);
    control_cmd.desired_yaw_rate = static_cast<float>(msg->yaw_dot);

    g_control_cmd_pub.publish(control_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Fuel2Sunray_node");

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
    ros::Subscriber fuel_cmd_sub = nh.subscribe(input_topic, queue_size, fuelPositionCallback);

    // 保留订阅对象，避免离开作用域后自动析构导致回调失效。
    (void)fuel_cmd_sub;

    ROS_INFO_STREAM("Fuel2Sunray subscribe: " << input_topic);
    ROS_INFO_STREAM("Fuel2Sunray publish:   " << output_topic);
    ROS_INFO_STREAM("Fuel2Sunray use_yaw_rate: " << (g_use_yaw_rate ? "true" : "false"));
    ROS_INFO_STREAM("Fuel2Sunray ready_only:   " << (g_ready_only ? "true" : "false"));

    ros::spin();
    return 0;
}
