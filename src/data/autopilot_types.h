#ifndef AUTOPILOT_TYPES_H
#define AUTOPILOT_TYPES_H

#include "mavlink.h"

#include <string>
#include <stdexcept>

namespace Data
{



inline std::string AutopilotTypeToString(const  MAV_AUTOPILOT &autopilottype) {
    switch (autopilottype) {
    case  MAV_AUTOPILOT:: MAV_AUTOPILOT_GENERIC:
        return " MAV_AUTOPILOT_GENERIC";
    case  MAV_AUTOPILOT:: MAV_AUTOPILOT_ARDUPILOTMEGA:
        return " MAV_AUTOPILOT_ARDUPILOTMEGA";
    case MAV_AUTOPILOT:: MAV_AUTOPILOT_INVALID:
        return " MAV_AUTOPILOT_INVALID";
    case MAV_AUTOPILOT:: MAV_AUTOPILOT_PX4:
        return " MAV_AUTOPILOT_PX4";
    default:
        throw std::runtime_error("Unknown autopilot type seen (to string): " + std::to_string(autopilottype));
    }
}

inline MAV_AUTOPILOT AutopilotTypeFromString(const std::string &str) {
    if(str == " MAV_AUTOPILOT_GENERIC")
        return MAV_AUTOPILOT:: MAV_AUTOPILOT_GENERIC;
    if(str == " MAV_AUTOPILOT_ARDUPILOTMEGA")
        return MAV_AUTOPILOT:: MAV_AUTOPILOT_ARDUPILOTMEGA;
    if(str == " MAV_AUTOPILOT_INVALID")
        return MAV_AUTOPILOT:: MAV_AUTOPILOT_INVALID;
    if(str == " MAV_AUTOPILOT_PX4")
        return MAV_AUTOPILOT:: MAV_AUTOPILOT_PX4;
    throw std::runtime_error("Unknown autopilot type seen (from string): " + str);
}

}

#endif // AUTOPILOT_TYPES_H
