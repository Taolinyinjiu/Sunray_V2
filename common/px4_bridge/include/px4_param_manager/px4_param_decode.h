/**
 * @file px4_param_decode.h
 * @author your name (you@domain.com)
 * @brief
 * 当通过mavros的服务，拿到px4的参数时，得到的并不是完整的语义信息，而是int或者double类型的变量(本质是float，但是mavros传递的是double)
 * 因此我们需要一个解码库，来让我们理解，这个返回的变量想要说明什么信息，原本这部分内容是和px4_param_types耦合在一起的，但是可读性太低了，因此拆出来了
 * 写解码库是一件很头疼的事情,因为我们无法预知用户或者同事会以什么方式来进行解码,又或者说他们想要得到一个怎样的结果,于是我们只能尽量的满足
 * 字符串在程序中并不能够比较好的作为判断的手段,字符串适合作为终端展示的结果,又或者是日志展示的结果
 * 因此我们选择在结构体中提供诸如 is_* ,
 * enable_*的函数,用于判断参数所想要表达的内容
 * 对于单值参数,也就是像EKF2_HGT_REF,几个选项之间只有一个会生效的,我们使用is_*函数来进行判断,几个函数之间只会有一个为true
 * 对于多值参数,也就是像EKF2_EV_CTRL这种,几个参数可以同时生效的,我们使用enable_*函数进行判断,几个函数可能会都返回true
 * @version 0.1
 * @date 2026-03-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>

// 解码命名空间，使用和px4_param_types相同的结构体命名
namespace px4_param_decode {
struct EKF2_HGT_REF {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_HGT_REF";

    enum value_type : raw_type { Unknown = -1, Baro = 0, Gnss = 1, Range = 2, Vision = 3 };

    raw_type raw_value = Unknown;
    value_type value = Unknown;

    void decode(raw_type raw) noexcept {
        raw_value = raw;
        switch (raw) {
        case Baro:
            value = Baro;
            break;
        case Gnss:
            value = Gnss;
            break;
        case Range:
            value = Range;
            break;
        case Vision:
            value = Vision;
            break;
        default:
            value = Unknown;
            break;
        }
    }

    bool is_baro() const noexcept {
        return value == Baro;
    }
    bool is_gnss() const noexcept {
        return value == Gnss;
    }
    bool is_range() const noexcept {
        return value == Range;
    }
    bool is_vision() const noexcept {
        return value == Vision;
    }
};

struct EKF2_EV_CTRL {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_EV_CTRL";

    enum value_type : uint32_t {
        Horizontalposition = 1u << 0,
        Verticalposition = 1u << 1,
        Velocity = 1u << 2,
        Yaw = 1u << 3
    };

    uint32_t value = 0u;

    void decode(raw_type raw) noexcept {
        value = static_cast<uint32_t>(raw);
    }

    bool enable_horizontal_position() const noexcept {
        return (value & Horizontalposition) != 0u;
    }
    bool enable_vertical_position() const noexcept {
        return (value & Verticalposition) != 0u;
    }
    bool enable_velocity() const noexcept {
        return (value & Velocity) != 0u;
    }
    bool enable_yaw() const noexcept {
        return (value & Yaw) != 0u;
    }
};

struct EKF2_EV_DELAY {
    using raw_type = double;
    static constexpr const char* param_name = "EKF2_EV_DELAY";

    raw_type value = 0.0;

    void decode(raw_type raw) noexcept {
        value = raw;
    }
};

struct EKF2_MAG_TYPE {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_MAG_TYPE";

    enum value_type : raw_type { Unknown = -1, Auto = 0, Mag_Yaw = 1, None = 5 };

    raw_type raw_value = Unknown;
    value_type value = Unknown;

    void decode(raw_type raw) noexcept {
        raw_value = raw;
        switch (raw) {
        case Auto:
            value = Auto;
            break;
        case Mag_Yaw:
            value = Mag_Yaw;
            break;
        case None:
            value = None;
            break;
        default:
            value = Unknown;
            break;
        }
    }

    bool is_auto() const noexcept {
        return value == Auto;
    }
    bool is_yaw() const noexcept {
        return value == Mag_Yaw;
    }
    bool is_none() const noexcept {
        return value == None;
    }
};

struct EKF2_MAG_CHECK {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_MAG_CHECK";

    enum value_type : uint32_t { Strength = 1u << 0, Inclination = 1u << 1, Wait2WMM = 1u << 2 };

    uint32_t value = 0u;

    void decode(raw_type raw) noexcept {
        value = static_cast<uint32_t>(raw);
    }

    bool enable_strength() const noexcept {
        return (value & Strength) != 0u;
    }
    bool enable_inclination() const noexcept {
        return (value & Inclination) != 0u;
    }
    bool enable_wait2wmm() const noexcept {
        return (value & Wait2WMM) != 0u;
    }
};

struct EKF2_GPS_CTRL {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_GPS_CTRL";

    enum value_type : uint32_t {
        Lon_Lat = 1u << 0,
        Altitude = 1u << 1,
        Velocity3D = 1u << 2,
        Dual_AntennaHeading = 1u << 3
    };

    uint32_t value = 0u;

    void decode(raw_type raw) noexcept {
        value = static_cast<uint32_t>(raw);
    }

    bool enable_lon_lat() const noexcept {
        return (value & Lon_Lat) != 0u;
    }
    bool enable_altitude() const noexcept {
        return (value & Altitude) != 0u;
    }
    bool enable_velocity3d() const noexcept {
        return (value & Velocity3D) != 0u;
    }
    bool enable_dual_antenna_heading() const noexcept {
        return (value & Dual_AntennaHeading) != 0u;
    }
};

struct EKF2_GPS_CHECK {
    using raw_type = int;
    static constexpr const char* param_name = "EKF2_GPS_CHECK";

    enum value_type : uint32_t {
        Min_sat_count = 1u << 0,
        Max_PDOP = 1u << 1,
        Max_HorizontalPosition_Error = 1u << 2,
        Max_VerticalPosition_Error = 1u << 3,
        Max_Speed_Error = 1u << 4,
        Max_HorizontalPosition_Rate = 1u << 5,
        Max_VerticalPosition_Rate = 1u << 6,
        Max_HorizontalSpeed = 1u << 7,
        Max_VerticalVelocity_Discrepancy = 1u << 8
    };

    uint32_t value = 0u;

    void decode(raw_type raw) noexcept {
        value = static_cast<uint32_t>(raw);
    }

    bool enable_min_sat_count() const noexcept {
        return (value & Min_sat_count) != 0u;
    }
    bool enable_max_pdop() const noexcept {
        return (value & Max_PDOP) != 0u;
    }
    bool enable_max_horizontal_position_error() const noexcept {
        return (value & Max_HorizontalPosition_Error) != 0u;
    }
    bool enable_max_vertical_position_error() const noexcept {
        return (value & Max_VerticalPosition_Error) != 0u;
    }
    bool enable_max_speed_error() const noexcept {
        return (value & Max_Speed_Error) != 0u;
    }
    bool enable_max_horizontal_position_rate() const noexcept {
        return (value & Max_HorizontalPosition_Rate) != 0u;
    }
    bool enable_max_vertical_position_rate() const noexcept {
        return (value & Max_VerticalPosition_Rate) != 0u;
    }
    bool enable_max_horizontal_speed() const noexcept {
        return (value & Max_HorizontalSpeed) != 0u;
    }
    bool enable_max_vertical_velocity_discrepancy() const noexcept {
        return (value & Max_VerticalVelocity_Discrepancy) != 0u;
    }
};

struct EKF2_GPS_DELAY {
    using raw_type = double;
    static constexpr const char* param_name = "EKF2_GPS_DELAY";

    raw_type value = 0.0;

    void decode(raw_type raw) noexcept {
        value = raw;
    }
};

}  // namespace px4_param_decode
