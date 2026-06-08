#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher g_pub;

void odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    g_pub.publish(pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_odom_to_vision_pose_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string local_odom_topic;
    std::string vision_pose_topic;
    pnh.param<std::string>("local_odom_topic", local_odom_topic,
                           "/uav1/sunray/localization/local_odom");
    pnh.param<std::string>("vision_pose_topic", vision_pose_topic,
                           "/uav1/mavros/vision_pose/pose");

    g_pub = nh.advertise<geometry_msgs::PoseStamped>(vision_pose_topic, 10);
    ros::Subscriber sub = nh.subscribe(local_odom_topic, 10, odomCb);
    ROS_INFO("local_odom_to_vision_pose: %s -> %s",
             local_odom_topic.c_str(), vision_pose_topic.c_str());
    ros::spin();
    return 0;
}
