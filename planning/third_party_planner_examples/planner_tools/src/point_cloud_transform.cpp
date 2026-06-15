#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>

class PointCloudTransformer {
public:
    PointCloudTransformer(ros::NodeHandle& nh) : nh_(nh) {
        nh.param<std::string>("input_point_topic",input_topic_, "pointcloud");
        nh.param<std::string>("output_point_topic",output_topic_, "pointcloud_world");
        nh.param<std::string>("odom_topic",odom_topic_, "odom");
        nh.param<std::string>("frame_id",frame_id_, "world");
        nh.param<std::string>("child_frame_id",child_frame_id_, "base_link");

        ROS_INFO_STREAM("Input topic: "<<input_topic_);
        ROS_INFO_STREAM("Output topic: "<<output_topic_);
        ROS_INFO_STREAM("Odom topic: "<<odom_topic_);
        ROS_INFO_STREAM("Frame id: "<<frame_id_);
        ROS_INFO_STREAM("Child frame id: "<<child_frame_id_);

        odom_sub_ = nh_.subscribe(odom_topic_, 1, &PointCloudTransformer::odomCallback, this);
        // 订阅输入点云；收到后使用最近一次里程计提供的位姿做坐标变换。
        sub_ = nh_.subscribe(input_topic_, 1, &PointCloudTransformer::cloudCallback, this);

        // 发布转换到目标坐标系后的点云。
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
        // 分别保存输入点云和变换后的点云。
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);

        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
        pcl_ros::transformPointCloud(*temp_cloud, *cloud_transformed, transform);

        pcl::toROSMsg(*cloud_transformed, cloud_publish);
        cloud_publish.header = input->header;
        cloud_publish.header.frame_id = frame_id_;

        pub_.publish(cloud_publish);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        // 根据里程计更新当前使用的刚体变换，供点云回调直接复用。
        transform.stamp_ = odom_msg->header.stamp;
        transform.frame_id_ = frame_id_;
        transform.child_frame_id_ = child_frame_id_;
        transform.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z));
        transform.setRotation(tf::Quaternion(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w));
    }

    void run() {
        ros::Rate rate(10.0);
        ros::Duration(1.0).sleep();
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        while (nh_.ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Subscriber odom_sub_;
    tf::StampedTransform transform;

    std::string input_topic_;
    std::string output_topic_;
    std::string odom_topic_;
    std::string frame_id_;
    std::string child_frame_id_;

    sensor_msgs::PointCloud2 cloud_publish;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_transform");
    ros::NodeHandle nh("~");
    PointCloudTransformer transformer(nh);
    transformer.run();
    return 0;
}