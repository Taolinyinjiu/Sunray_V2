#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>
#include <signal.h>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> SyncPolicy;

ros::Publisher mocap_odom_pub;
ros::Publisher mocap_odom_relative_pub;

bool first_frame_received = false;
tf2::Transform T_first_inv;

// 中断信号
void MySigintHandler(int sig) {

    ros::shutdown();
}

void SyncCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg, const geometry_msgs::TwistStamped::ConstPtr& twist_msg) {


    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = pose_msg->header.stamp;
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "base_link";

    // Pose
    odom_msg.pose.pose.position.x = pose_msg->pose.position.x;
    odom_msg.pose.pose.position.y = pose_msg->pose.position.y;
    odom_msg.pose.pose.position.z = pose_msg->pose.position.z;
    odom_msg.pose.pose.orientation.x = pose_msg->pose.orientation.x;
    odom_msg.pose.pose.orientation.y = pose_msg->pose.orientation.y;
    odom_msg.pose.pose.orientation.z = pose_msg->pose.orientation.z;
    odom_msg.pose.pose.orientation.w = pose_msg->pose.orientation.w;

    // Twist
    odom_msg.twist.twist.linear.x = twist_msg->twist.linear.x;
    odom_msg.twist.twist.linear.y = twist_msg->twist.linear.y;
    odom_msg.twist.twist.linear.z = twist_msg->twist.linear.z;
    odom_msg.twist.twist.angular.x = twist_msg->twist.angular.x;
    odom_msg.twist.twist.angular.y = twist_msg->twist.angular.y;
    odom_msg.twist.twist.angular.z = twist_msg->twist.angular.z;

    mocap_odom_pub.publish(odom_msg);

    // --- Build current transform from incoming pose ---
    tf2::Vector3 t_curr(
        pose_msg->pose.position.x,
        pose_msg->pose.position.y,
        pose_msg->pose.position.z);
    tf2::Quaternion q_curr(
        pose_msg->pose.orientation.x,
        pose_msg->pose.orientation.y,
        pose_msg->pose.orientation.z,
        pose_msg->pose.orientation.w);
    tf2::Transform T_curr(q_curr, t_curr);

    // Record first-frame inverse on first call
    if (!first_frame_received) {
        T_first_inv = T_curr.inverse();
        first_frame_received = true;
    }

    // --- Relative odometry (first frame = identity) ---
    tf2::Transform T_rel = T_first_inv * T_curr;

    tf2::Vector3    t_rel = T_rel.getOrigin();
    tf2::Quaternion q_rel = T_rel.getRotation();

    // Rotate world-frame linear velocity into the initial frame
    tf2::Vector3 v_world(twist_msg->twist.linear.x,
                         twist_msg->twist.linear.y,
                         twist_msg->twist.linear.z);
    tf2::Vector3 v_rel = T_first_inv.getBasis() * v_world;

    nav_msgs::Odometry odom_rel_msg;
    odom_rel_msg.header.stamp    = pose_msg->header.stamp;
    odom_rel_msg.header.frame_id = "world";
    odom_rel_msg.child_frame_id  = "base_link";

    odom_rel_msg.pose.pose.position.x    = t_rel.x();
    odom_rel_msg.pose.pose.position.y    = t_rel.y();
    odom_rel_msg.pose.pose.position.z    = t_rel.z();
    odom_rel_msg.pose.pose.orientation.x = q_rel.x();
    odom_rel_msg.pose.pose.orientation.y = q_rel.y();
    odom_rel_msg.pose.pose.orientation.z = q_rel.z();
    odom_rel_msg.pose.pose.orientation.w = q_rel.w();

    odom_rel_msg.twist.twist.linear.x  = v_rel.x();
    odom_rel_msg.twist.twist.linear.y  = v_rel.y();
    odom_rel_msg.twist.twist.linear.z  = v_rel.z();
    odom_rel_msg.twist.twist.angular.x = twist_msg->twist.angular.x;
    odom_rel_msg.twist.twist.angular.y = twist_msg->twist.angular.y;
    odom_rel_msg.twist.twist.angular.z = twist_msg->twist.angular.z;

    mocap_odom_relative_pub.publish(odom_rel_msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "mocap_odom_node");

    ros::NodeHandle nh;

    // 中断信号注册
    signal(SIGINT, MySigintHandler);

    int uav_id = 0;
    nh.param<int>("uav_id", uav_id, 1);

    std::string odom_pub_topic = "/uav" + std::to_string(uav_id) + "/sunray/mocap_odometry";
    mocap_odom_pub = nh.advertise<nav_msgs::Odometry>(odom_pub_topic, 10);

    std::string odom_rel_pub_topic = "/uav" + std::to_string(uav_id) + "/sunray/mocap_odometry_relative";
    mocap_odom_relative_pub = nh.advertise<nav_msgs::Odometry>(odom_rel_pub_topic, 10);

    std::string pose_sub_topic = "/vrpn_client_node_1/uav" + std::to_string(uav_id) + "/pose";
    std::string twist_sub_topic = "/vrpn_client_node_1/uav" + std::to_string(uav_id) + "/twist";
    message_filters::Subscriber<geometry_msgs::PoseStamped> mocap_pose_sub(nh, pose_sub_topic, 20);
    message_filters::Subscriber<geometry_msgs::TwistStamped> mocap_twist_sub(nh, twist_sub_topic, 20);

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(20), mocap_pose_sub, mocap_twist_sub);
    sync.registerCallback(boost::bind(&SyncCallback, _1, _2));

    ROS_INFO("Subscribe pose topic: %s", pose_sub_topic.c_str());
    ROS_INFO("Subscribe twist topic: %s", twist_sub_topic.c_str());
    ROS_INFO("Publish odometry topic: %s", odom_pub_topic.c_str());
    ROS_INFO("Publish relative odometry topic: %s", odom_rel_pub_topic.c_str());

    ros::spin();
    return 0;
}
