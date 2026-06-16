#pragma once

#include <cstdint>
#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include <sunray_logger/binary_writer.hpp>
#include <sunray_logger/time_utils.hpp>
#include <sunray_logger/ulog_field.hpp>

namespace sunray_logger {

struct PoseRecord {
    uint64_t timestamp = 0;
    float position[3] = {0.0f, 0.0f, 0.0f};
    float orientation_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z

    static const char* formatName() {
        return "sunray_pose";
    }

    static std::vector<UlogField> fields() {
        return {
            {"uint64_t", "timestamp"},
            {"float[3]", "position"},
            {"float[4]", "orientation_q"},
        };
    }

    void serialize(BinaryWriter& writer) const {
        writer.write(timestamp);
        writer.writeArray(position, 3);
        writer.writeArray(orientation_q, 4);
    }
};

inline PoseRecord makePoseRecord(const geometry_msgs::PoseStamped& msg) {
    PoseRecord record;
    record.timestamp = rosTimeToUs(msg.header.stamp);
    record.position[0] = static_cast<float>(msg.pose.position.x);
    record.position[1] = static_cast<float>(msg.pose.position.y);
    record.position[2] = static_cast<float>(msg.pose.position.z);
    record.orientation_q[0] = static_cast<float>(msg.pose.orientation.w);
    record.orientation_q[1] = static_cast<float>(msg.pose.orientation.x);
    record.orientation_q[2] = static_cast<float>(msg.pose.orientation.y);
    record.orientation_q[3] = static_cast<float>(msg.pose.orientation.z);
    return record;
}

}  // namespace sunray_logger
