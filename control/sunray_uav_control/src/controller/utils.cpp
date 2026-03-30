#include "controller/utils.hpp"

std::string flightmode_to_string(control_common::FlightMode flight) {
    switch (flight) {
    case control_common::FlightMode::Undefined:
        return "UNDEFINED";
    case control_common::FlightMode::Manual:
        return "MANUAL";
    case control_common::FlightMode::Acro:
        return "ACRO";
    case control_common::FlightMode::Altctl:
        return "ALTCTL";
    case control_common::FlightMode::Posctl:
        return "POSCTL";
    case control_common::FlightMode::Offboard:
        return "OFFBOARD";
    case control_common::FlightMode::Stabilized:
        return "STABILIZED";
    case control_common::FlightMode::Rattitude:
        return "RATTITUDE";
    case control_common::FlightMode::AutoMission:
        return "AUTO.MISSION";
    case control_common::FlightMode::AutoLoiter:
        return "AUTO.LOITER";
    case control_common::FlightMode::AutoRtl:
        return "AUTO.RTL";
    case control_common::FlightMode::AutoLand:
        return "AUTO.LAND";
    case control_common::FlightMode::AutoRtgs:
        return "AUTO.RTGS";
    case control_common::FlightMode::AutoReady:
        return "AUTO.READY";
    case control_common::FlightMode::AutoTakeoff:
        return "AUTO.TAKEOFF";
    default:
        return "UNKNOWN";
    }
}

std::string landed_to_string(control_common::LandedState land_state) {
    switch (land_state) {
    case control_common::LandedState::Undefined:
        return "UNDEFINED";
    case control_common::LandedState::OnGround:
        return "ON_GROUND";
    case control_common::LandedState::InAir:
        return "IN_AIR";
    case control_common::LandedState::Takeoff:
        return "TAKEOFF";
    case control_common::LandedState::Landing:
        return "LANDING";
    }
    return "UNKNOWN";
}
