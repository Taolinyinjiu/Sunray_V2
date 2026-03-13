#pragma once

#include <cstdint>

namespace uav_control {
namespace complex_cmd {

// Match ComplexCmd.msg "mode" field.
enum class Mode : uint8_t {
  POSITION_TARGET = 0,
  ATTITUDE_TARGET = 1,
  BOTH = 2,
};

namespace position_target {
// Coordinate frames (match mavros_msgs/PositionTarget).
static constexpr uint8_t FRAME_LOCAL_NED = 1;
static constexpr uint8_t FRAME_LOCAL_OFFSET_NED = 7;
static constexpr uint8_t FRAME_BODY_NED = 8;
static constexpr uint8_t FRAME_BODY_OFFSET_NED = 9;

// Type mask bits (match mavros_msgs/PositionTarget).
static constexpr uint16_t IGNORE_PX = 1;       // position.x
static constexpr uint16_t IGNORE_PY = 2;       // position.y
static constexpr uint16_t IGNORE_PZ = 4;       // position.z
static constexpr uint16_t IGNORE_VX = 8;       // velocity.x
static constexpr uint16_t IGNORE_VY = 16;      // velocity.y
static constexpr uint16_t IGNORE_VZ = 32;      // velocity.z
static constexpr uint16_t IGNORE_AFX = 64;     // accel.x or force.x
static constexpr uint16_t IGNORE_AFY = 128;    // accel.y or force.y
static constexpr uint16_t IGNORE_AFZ = 256;    // accel.z or force.z
static constexpr uint16_t FORCE = 512;         // use accel as force
static constexpr uint16_t IGNORE_YAW = 1024;   // yaw
static constexpr uint16_t IGNORE_YAW_RATE = 2048; // yaw_rate
}  // namespace position_target

namespace attitude_target {
// Type mask bits (match mavros_msgs/AttitudeTarget).
static constexpr uint8_t IGNORE_ROLL_RATE = 1;   // body_rate.x
static constexpr uint8_t IGNORE_PITCH_RATE = 2;  // body_rate.y
static constexpr uint8_t IGNORE_YAW_RATE = 4;    // body_rate.z
static constexpr uint8_t IGNORE_THRUST = 64;     // thrust
static constexpr uint8_t IGNORE_ATTITUDE = 128;  // orientation
}  // namespace attitude_target

}  // namespace complex_cmd
}  // namespace uav_control
