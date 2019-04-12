#ifndef AUTOPILOT_TYPES_H
#define AUTOPILOT_TYPES_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class AutopilotType
{
    AUTOPILOT_TYPE_GENERIC=0, /* Generic autopilot, full support for everything | */
    AUTOPILOT_TYPE_ARDUPILOTMEGA=1, /* ArduPilotMega / ArduCopter, http://diydrones.com | */
    AUTOPILOT_TYPE_INVALID=2, /* No valid autopilot, e.g. a GCS or other MAVLink component | */
    AUTOPILOT_TYPE_PX4=3, /* PX4 Autopilot - http://pixhawk.ethz.ch/px4/ | */
    AUTOPILOT_TYPE_DJI=4 /* DJI Autopilot | */
};


inline std::string AutopilotTypeToString(const AutopilotType &autopilottype) {
    switch (autopilottype) {
    case AutopilotType::AUTOPILOT_TYPE_GENERIC:
        return "AUTOPILOT_TYPE_GENERIC";
    case AutopilotType::AUTOPILOT_TYPE_ARDUPILOTMEGA:
        return "AUTOPILOT_TYPE_ARDUPILOTMEGA";
    case AutopilotType::AUTOPILOT_TYPE_INVALID:
        return "AUTOPILOT_TYPE_INVALID";
    case AutopilotType::AUTOPILOT_TYPE_PX4:
        return "AUTOPILOT_TYPE_PX4";
    case AutopilotType::AUTOPILOT_TYPE_DJI:
        return "AUTOPILOT_TYPE_DJI";
    default:
        throw std::runtime_error("Unknown autopilot type seen");
    }
}

inline AutopilotType AutopilotTypeFromString(const std::string &str) {
    if(str == "AUTOPILOT_TYPE_GENERIC")
        return AutopilotType::AUTOPILOT_TYPE_GENERIC;
    if(str == "AUTOPILOT_TYPE_ARDUPILOTMEGA")
        return AutopilotType::AUTOPILOT_TYPE_ARDUPILOTMEGA;
    if(str == "AUTOPILOT_TYPE_INVALID")
        return AutopilotType::AUTOPILOT_TYPE_INVALID;
    if(str == "AUTOPILOT_TYPE_PX4")
        return AutopilotType::AUTOPILOT_TYPE_PX4;
    if(str == "AUTOPILOT_TYPE_DJI")
        return AutopilotType::AUTOPILOT_TYPE_DJI;
    throw std::runtime_error("Unknown autopilot type seen");
}

}

#endif // AUTOPILOT_TYPES_H
