/**
 * @file px4_param_types.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-03-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>

namespace px4_param_types {

struct EKF2_HGT_REF {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_HGT_REF";
    enum mask : int {
        Baro = 0,   // 气压计
        Gnss = 1,   // GPS
        Range = 2,  // 测距仪
        Vision = 3  // 外部里程计
    };
    mask value = Baro;
    void enable_baro() {
        value = Baro;
    }
    void enable_gnss() {
        value = Gnss;
    }
    void enable_range() {
        value = Range;
    }
    void enable_vision() {
        value = Vision;
    }
    raw_type encode() const {
        return static_cast<raw_type>(value);
    }
};

struct EKF2_EV_CTRL {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_EV_CTRL";
    uint32_t value = 0u;
    void enable_Horizontalposition() {
        value |= 1u << 0;
    }
    void enable_Verticalposition() {
        value |= 1u << 1;
    }
    void enable_Velocity() {
        value |= 1u << 2;
    }
    void enable_Yaw() {
        value |= 1u << 3;
    }
    raw_type encode() const {
        return static_cast<raw_type>(value);
    }
};

struct EKF2_EV_DELAY {
    using raw_type = double;
    static constexpr const char* param_name = "EKF2_EV_DELAY";
    double value = 0.0f;
    void set_delay_ms(double ms) {
        value = ms;
    }
    raw_type encode() const {
        return value;
    }
};

struct EKF2_MAG_TYPE {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_MAG_TYPE";
    enum mask : int {
        Auto = 0,     // PX4 自动决定如何融合 MAG 数据
        Mag_Yaw = 1,  // 仅使用 MAG 数据融合到 Yaw 角
        None = 5      // 不使用 MAG Yaw 融合
    };
    mask value = Auto;
    void set_Auto() {
        value = Auto;
    }
    void set_Yaw() {
        value = Mag_Yaw;
    }
    void set_None() {
        value = None;
    }
    raw_type encode() const {
        return static_cast<raw_type>(value);
    }
};

struct EKF2_MAG_CHECK {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_MAG_CHECK";
    uint32_t value = 0u;
    void enable_Strength() {
        value |= 1u << 0;
    }
    void enable_Inclination() {
        value |= 1u << 1;
    }
    void enable_Wait2WMM() {
        value |= 1u << 2;
    }
    raw_type encode() const {
        return static_cast<raw_type>(value);
    }
};

struct EKF2_GPS_CTRL {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_GPS_CTRL";
    uint32_t value = 0u;
    void enable_Lon_Lat() {
        value |= 1u << 0;
    }
    void enable_Altitude() {
        value |= 1u << 1;
    }
    void enable_Velocity3D() {
        value |= 1u << 2;
    }
    void enable_Dual_AntennaHeading() {
        value |= 1u << 3;
    }
    raw_type encode() const {
        return static_cast<raw_type>(value);
    }
};

struct EKF2_GPS_CHECK {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_GPS_CHECK";
    uint32_t value = 0u;
    void enable_Min_sat_count() {
        value |= 1u << 0;
    }
    void enable_Max_PDOP() {
        value |= 1u << 1;
    }
    void enable_Max_HorizontalPosition_Error() {
        value |= 1u << 2;
    }
    void enable_Max_VerticalPosition_Error() {
        value |= 1u << 3;
    }
    void enable_Max_Speed_Error() {
        value |= 1u << 4;
    }
    void enable_Max_HorizontalPosition_Rate() {
        value |= 1u << 5;
    }
    void enable_Max_VerticalPosition_Rate() {
        value |= 1u << 6;
    }
    void enable_Max_HorizontalSpeed() {
        value |= 1u << 7;
    }
    void enable_Max_VerticalVelocity_Discrepancy() {
        value |= 1u << 8;
    }
    raw_type encode() const {
        return static_cast<raw_type>(value);
    }
};

struct EKF2_GPS_DELAY {
    using raw_type = double;
    static constexpr const char* param_name = "EKF2_GPS_DELAY";
    double value = 0.0f;
    void set_delay_ms(double ms) {
        value = ms;
    }
    raw_type encode() const {
        return value;
    }
};

}  // namespace px4_param_types
