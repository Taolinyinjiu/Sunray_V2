#pragma once

#include <cstdint>
#include <vector>

#include <sensor_msgs/Imu.h>

#include <sunray_logger/binary_writer.hpp>
#include <sunray_logger/time_utils.hpp>
#include <sunray_logger/ulog_field.hpp>

namespace sunray_logger {

struct ImuRecord {
    uint64_t timestamp = 0;
    float orientation_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    float angular_velocity[3] = {0.0f, 0.0f, 0.0f};
    float linear_acceleration[3] = {0.0f, 0.0f, 0.0f};

    static const char* formatName() {
        return "sunray_imu";
    }

    static std::vector<UlogField> fields() {
        return {
            {"uint64_t", "timestamp"},
            {"float[4]", "orientation_q"},
            {"float[3]", "angular_velocity"},
            {"float[3]", "linear_acceleration"},
        };
    }

    void serialize(BinaryWriter& writer) const {
        writer.write(timestamp);
        writer.writeArray(orientation_q, 4);
        writer.writeArray(angular_velocity, 3);
        writer.writeArray(linear_acceleration, 3);
    }
};

inline ImuRecord makeImuRecord(const sensor_msgs::Imu& msg) {
    ImuRecord record;
    record.timestamp = rosTimeToUs(msg.header.stamp);
    record.orientation_q[0] = static_cast<float>(msg.orientation.w);
    record.orientation_q[1] = static_cast<float>(msg.orientation.x);
    record.orientation_q[2] = static_cast<float>(msg.orientation.y);
    record.orientation_q[3] = static_cast<float>(msg.orientation.z);
    record.angular_velocity[0] = static_cast<float>(msg.angular_velocity.x);
    record.angular_velocity[1] = static_cast<float>(msg.angular_velocity.y);
    record.angular_velocity[2] = static_cast<float>(msg.angular_velocity.z);
    record.linear_acceleration[0] = static_cast<float>(msg.linear_acceleration.x);
    record.linear_acceleration[1] = static_cast<float>(msg.linear_acceleration.y);
    record.linear_acceleration[2] = static_cast<float>(msg.linear_acceleration.z);
    return record;
}

}  // namespace sunray_logger
