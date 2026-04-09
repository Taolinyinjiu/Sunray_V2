#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>

class PointCloudTransformer {
public:
    PointCloudTransformer() : tf_listener_(tf_buffer_) {
        ros::NodeHandle nh;
        sub_ = nh.subscribe("input_cloud", 1, &PointCloudTransformer::callback, this);
        pub_ = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 1);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg_in) {
        sensor_msgs::PointCloud2 msg_out;
        try {
            // 1. 获取变换矩阵
            // 注意：ROS 1 中通常使用 ros::Time(0) 表示最新的变换
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                "base_link",          // 目标坐标系
                msg_in->header.frame_id, // 源坐标系
                msg_in->header.stamp,   // 使用数据的时间戳
                ros::Duration(0.1)     // 等待时间
            );

            // 2. 执行转换
            // 这里调用的是 tf2::doTransform
            tf2::doTransform(*msg_in, msg_out, transform);

            // 3. 发布
            pub_.publish(msg_out);

        } catch (tf2::TransformException &ex) {
            ROS_WARN("转换失败: %s", ex.what());
        }
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pc_transformer");
    PointCloudTransformer pct;
    ros::spin();
    return 0;
}
