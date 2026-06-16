#pragma once

#include <cstdint>
#include <vector>

#include <nav_msgs/Odometry.h>

#include <sunray_logger/binary_writer.hpp>
#include <sunray_logger/time_utils.hpp>
#include <sunray_logger/ulog_field.hpp>

namespace sunray_logger {

struct OdomRecord {
    uint64_t timestamp = 0;
    float position[3] = {0.0f, 0.0f, 0.0f};
    float velocity[3] = {0.0f, 0.0f, 0.0f};
    float orientation_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    float angular_velocity[3] = {0.0f, 0.0f, 0.0f};

    static const char* formatName() {
        return "sunray_odom";
    }

    static std::vector<UlogField> fields() {
        return {
            {"uint64_t", "timestamp"},
            {"float[3]", "position"},
            {"float[3]", "velocity"},
            {"float[4]", "orientation_q"},
            {"float[3]", "angular_velocity"},
        };
    }

    void serialize(BinaryWriter& writer) const {
        writer.write(timestamp);
        writer.writeArray(position, 3);
        writer.writeArray(velocity, 3);
        writer.writeArray(orientation_q, 4);
        writer.writeArray(angular_velocity, 3);
    }
};

inline OdomRecord makeOdomRecord(const nav_msgs::Odometry& msg) {
    OdomRecord record;
    record.timestamp = rosTimeToUs(msg.header.stamp);
    record.position[0] = static_cast<float>(msg.pose.pose.position.x);
    record.position[1] = static_cast<float>(msg.pose.pose.position.y);
    record.position[2] = static_cast<float>(msg.pose.pose.position.z);
    record.velocity[0] = static_cast<float>(msg.twist.twist.linear.x);
    record.velocity[1] = static_cast<float>(msg.twist.twist.linear.y);
    record.velocity[2] = static_cast<float>(msg.twist.twist.linear.z);
    record.orientation_q[0] = static_cast<float>(msg.pose.pose.orientation.w);
    record.orientation_q[1] = static_cast<float>(msg.pose.pose.orientation.x);
    record.orientation_q[2] = static_cast<float>(msg.pose.pose.orientation.y);
    record.orientation_q[3] = static_cast<float>(msg.pose.pose.orientation.z);
    record.angular_velocity[0] = static_cast<float>(msg.twist.twist.angular.x);
    record.angular_velocity[1] = static_cast<float>(msg.twist.twist.angular.y);
    record.angular_velocity[2] = static_cast<float>(msg.twist.twist.angular.z);
    return record;
}

}  // namespace sunray_logger
