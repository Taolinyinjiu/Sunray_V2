#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// 全局发布者
ros::Publisher goal_pub;

/**
 * 回调函数：接收 Rviz 的 2D Nav Goal 消息
 * Rviz 默认发布话题为 /move_base_simple/goal
 */
void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("Received goal from Rviz: [x: %.2f, y: %.2f]",
             msg->pose.position.x, msg->pose.position.y);

    // 创建一个新的消息并转发
    // 这里可以直接转发 msg，也可以根据需要修改 frame_id
    geometry_msgs::PoseStamped forwarded_goal = *msg;
    forwarded_goal.pose.position.z = 1;
    // 发布到硬编码的话题 "/goal"
    goal_pub.publish(forwarded_goal);

    ROS_INFO("Forwarded goal to /goal");
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "catch_rviz_point_node");
    ros::NodeHandle nh;

    // 创建发布者：硬编码话题名为 "/goal"，队列长度为 10
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 10);

    // 创建订阅者：订阅 Rviz 默认的 2D Nav Goal 话题
    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 10, rvizGoalCallback);

    ROS_INFO("Catch Rviz Point Node Started. Waiting for 2D Nav Goal...");

    // 循环等待回调
    ros::spin();

    return 0;
}
