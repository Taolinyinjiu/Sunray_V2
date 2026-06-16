#pragma once

#include <cstdint>
#include <vector>

#include <geometry_msgs/TwistStamped.h>

#include <sunray_logger/binary_writer.hpp>
#include <sunray_logger/time_utils.hpp>
#include <sunray_logger/ulog_field.hpp>

namespace sunray_logger {

struct TwistRecord {
    uint64_t timestamp = 0;
    float velocity[3] = {0.0f, 0.0f, 0.0f};
    float angular_velocity[3] = {0.0f, 0.0f, 0.0f};

    static const char* formatName() {
        return "sunray_twist";
    }

    static std::vector<UlogField> fields() {
        return {
            {"uint64_t", "timestamp"},
            {"float[3]", "velocity"},
            {"float[3]", "angular_velocity"},
        };
    }

    void serialize(BinaryWriter& writer) const {
        writer.write(timestamp);
        writer.writeArray(velocity, 3);
        writer.writeArray(angular_velocity, 3);
    }
};

inline TwistRecord makeTwistRecord(const geometry_msgs::TwistStamped& msg) {
    TwistRecord record;
    record.timestamp = rosTimeToUs(msg.header.stamp);
    record.velocity[0] = static_cast<float>(msg.twist.linear.x);
    record.velocity[1] = static_cast<float>(msg.twist.linear.y);
    record.velocity[2] = static_cast<float>(msg.twist.linear.z);
    record.angular_velocity[0] = static_cast<float>(msg.twist.angular.x);
    record.angular_velocity[1] = static_cast<float>(msg.twist.angular.y);
    record.angular_velocity[2] = static_cast<float>(msg.twist.angular.z);
    return record;
}

}  // namespace sunray_logger
