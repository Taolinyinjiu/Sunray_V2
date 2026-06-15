#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <csignal>
#include <condition_variable>
#include <cmath>
#include "imu_process.h"
#include "ekf_filter.h"

typedef std::pair<double, Eigen::Matrix4d> OdomDequeType;
typedef std::pair<double, ImuData> ImuDequeType;

std::deque<ImuDequeType> imu_buffer;
std::deque<OdomDequeType> odom_buffer;

bool ros_exit_flag = false;

void SigHandle(int sig) {

    ros_exit_flag = true;
    ROS_WARN("Catch sig %d", sig);
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {

    double time = msg->header.stamp.toSec();
    ImuData imu_data;
    imu_data.timeStamp = time;
    imu_data.cur_imu_gyr = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    imu_data.cur_imu_acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    imu_buffer.push_back(ImuDequeType(time, imu_data));
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    double time = msg->header.stamp.toSec();
    Eigen::Matrix4d odom = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    odom.block<3, 3>(0, 0) = q.toRotationMatrix();
    odom.block<3, 1>(0, 3) = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odom_buffer.push_back(OdomDequeType(time, odom));
}

void PublishOdometry(ros::Publisher& odom_pub, const EkfState& state) {

    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = ros::Time(state.timestamp);
    //odom_msg.header.stamp = ros::Time::now();


    // Set position
    odom_msg.pose.pose.position.x = state.pos.x();
    odom_msg.pose.pose.position.y = state.pos.y();
    odom_msg.pose.pose.position.z = state.pos.z();

    // Set orientation
    Eigen::Quaterniond q(state.rot.matrix());
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();

    // Set velocity
    odom_msg.twist.twist.linear.x = state.vel.x();
    odom_msg.twist.twist.linear.y = state.vel.y();
    odom_msg.twist.twist.linear.z = state.vel.z();

    odom_pub.publish(odom_msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "ekf_odometry_node");

    ros::NodeHandle nh("~");

    std::string imu_topic, odom_topic;

    nh.param<std::string>("imu_topic", imu_topic, "/livox/imu");
    nh.param<std::string>("odom_topic", odom_topic, "/Odometry");

    // Setup subscriber
    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 100, ImuCallback);
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, OdomCallback);

    // Setup publisher
    ros::Publisher ekf_odom_pub = nh.advertise<nav_msgs::Odometry>("/ekf_odometry", 100);

    ROS_INFO("EKF Odometry Node Starting...");
    ROS_INFO("Subscribing to IMU topic: %s", imu_topic.c_str());
    ROS_INFO("Subscribing to Odometry topic: %s", odom_topic.c_str());
    ROS_INFO("Publishing to: /ekf_odometry");

    signal(SIGINT, SigHandle);

    ros::Rate rate(300);
    const double max_odom_delay = 0.15;

    // 初始化IMU重力，陀螺仪偏置
    std::shared_ptr<ImuProcess> imu_process = std::make_shared<ImuProcess>();

    bool imu_init_flag = false;

    while (ros::ok()) {

        if (ros_exit_flag || imu_init_flag)
            break;

        ros::spinOnce();

        while (!imu_buffer.empty()) {

            auto cur_imu = imu_buffer.front();
            imu_buffer.pop_front();

            if (imu_process->ImuInit(cur_imu.second)) {

                imu_init_flag = true;
                ROS_INFO("Imu Initialized!");
                break;
            }
        }

        rate.sleep();
    }

    // 初始化EKF滤波器
    std::shared_ptr<EkfFilter> ekf_filter = std::make_shared<EkfFilter>();

    ekf_filter->SetInitGyrBias(imu_process->GetInitGyrBias());
    ekf_filter->SetInitGravity(imu_process->GetInitGravity());

    bool ekf_init_flag = false;
    while (ros::ok()) {

        if (ros_exit_flag || ekf_init_flag)
            break;

        ros::spinOnce();

        while (!imu_buffer.empty() && !odom_buffer.empty()) {

            auto cur_imu = imu_buffer.front();
            auto cur_odom = odom_buffer.front();

            double dt = cur_imu.first - cur_odom.first;

            if (dt < 0) {

                imu_buffer.pop_front();

            } else {

                if (dt > 0.1) {

                    odom_buffer.pop_front();

                } else {

                    ekf_filter->SetInitPose(cur_odom.second);  //设置初始值

                    odom_buffer.pop_front();

                    // IMU预积分的第一帧
                    ImuData imu_data;
                    double dt = 0;
                    imu_process->ProcessIMU(cur_imu.second, imu_data, dt);
                    imu_buffer.pop_front();

                    ekf_init_flag = true;
                    ROS_INFO("EKF Initialized!");
                    break;
                }
            }
        }

        rate.sleep();
    }

    while (ros::ok()) {

        if (ros_exit_flag)
            break;

        ros::spinOnce();

        // EKF预测-更新循环：IMU驱动发布，odom只在时间到达时校正
        if (!imu_buffer.empty()) {

            auto cur_imu = imu_buffer.front();

            while (!odom_buffer.empty() && odom_buffer.front().first <= cur_imu.first) {

                double odom_delay = cur_imu.first - odom_buffer.front().first;

                if (odom_delay > max_odom_delay) {

                    ROS_WARN_THROTTLE(1.0, "Drop stale odom, delay: %.3f s", odom_delay);
                    odom_buffer.pop_front();
                    continue;
                }

                ekf_filter->Update(odom_buffer.front().second);
                odom_buffer.pop_front();
            }

            ImuData imu_data;
            double dt = 0.0;
            imu_process->ProcessIMU(cur_imu.second, imu_data, dt);
            imu_buffer.pop_front();

            if (dt > 0.0 && std::isfinite(dt)) {

                ekf_filter->Predict(imu_data, dt);
                PublishOdometry(ekf_odom_pub, ekf_filter->GetEkfState());

            } else {

                ROS_WARN_THROTTLE(1.0, "Skip IMU frame with invalid dt: %.9f", dt);
            }
        }

        rate.sleep();
    }

    return 0;
}
