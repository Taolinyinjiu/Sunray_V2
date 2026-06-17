#pragma once

#include <cstdint>

namespace control_common {

enum class TakeoffFailureReason : uint8_t {
    None = 0,
    LowHeightUnrecoverable = 1,
    CurveUnrecoverable = 2,
    ControllerNotReady = 3,
    SafetyAbort = 4,
};

inline const char* to_cstr(TakeoffFailureReason reason) {
    switch (reason) {
    case TakeoffFailureReason::None:
        return "none";
    case TakeoffFailureReason::LowHeightUnrecoverable:
        return "low_height_unrecoverable";
    case TakeoffFailureReason::CurveUnrecoverable:
        return "curve_unrecoverable";
    case TakeoffFailureReason::ControllerNotReady:
        return "controller_not_ready";
    case TakeoffFailureReason::SafetyAbort:
        return "safety_abort";
    }
    return "unknown";
}

}  // namespace control_common
