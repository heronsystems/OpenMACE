#ifndef MAV_TYPE_H
#define MAV_TYPE_H

#include <string>
#include <stdexcept>
#include <iostream>

#include "mavlink.h"

namespace Data
{

inline bool isSystemTypeRotary(const MAV_TYPE &type)
{
    bool rtn = false;
    switch (type) {
    case MAV_TYPE::MAV_TYPE_HELICOPTER:
    case MAV_TYPE::MAV_TYPE_TRICOPTER:
    case MAV_TYPE::MAV_TYPE_QUADROTOR:
    case MAV_TYPE::MAV_TYPE_HEXAROTOR:
    case MAV_TYPE::MAV_TYPE_OCTOROTOR:
        rtn = true;
        break;
    default:
        rtn = false;
        break;
    }
    return rtn;
}

inline std::string MAVTypeToString(const MAV_TYPE &systemType) {
    switch (systemType) {
    case MAV_TYPE::MAV_TYPE_GENERIC:
        return "GENERIC";
    case MAV_TYPE::MAV_TYPE_HELICOPTER:
        return "HELICOPTER";
    case MAV_TYPE::MAV_TYPE_GCS:
        return "GCS";
    case MAV_TYPE::MAV_TYPE_GROUND_ROVER:
        return "GROUND_ROVER";
    case MAV_TYPE::MAV_TYPE_SURFACE_BOAT:
        return "SURFACE_BOAT";
    case MAV_TYPE::MAV_TYPE_TRICOPTER:
        return "TRICOPTER";
    case MAV_TYPE::MAV_TYPE_QUADROTOR:
        return "QUADROTOR";
    case MAV_TYPE::MAV_TYPE_HEXAROTOR:
        return "HEXAROTOR";
    case MAV_TYPE::MAV_TYPE_OCTOROTOR:
        return "OCTOROTOR";
    case MAV_TYPE::MAV_TYPE_ONBOARD_CONTROLLER:
        return "ONBOARD_CONTROLLER";
    case MAV_TYPE::MAV_TYPE_FIXED_WING:
        return "FIXED_WING";
    default:
        std::cout << "Unsupported MAV type (to string): " + std::to_string(systemType) << std::endl;
//        throw std::runtime_error("Unknown MAV type seen (to string): " + std::to_string(systemType));
        return "";
    }
}

inline MAV_TYPE SystemTypeFromString(const std::string &str) {
    if(str == "GENERIC")
        return MAV_TYPE::MAV_TYPE_GENERIC;
    if(str == "HELICOPTER")
        return MAV_TYPE::MAV_TYPE_HELICOPTER;
    if(str == "GCS")
        return MAV_TYPE::MAV_TYPE_GCS;
    if(str == "GROUND_ROVER")
        return MAV_TYPE::MAV_TYPE_GROUND_ROVER;
    if(str == "SURFACE_BOAT")
        return MAV_TYPE::MAV_TYPE_SURFACE_BOAT;
    if(str == "TRICOPTER")
        return MAV_TYPE::MAV_TYPE_TRICOPTER;
    if(str == "QUADROTOR")
        return MAV_TYPE::MAV_TYPE_QUADROTOR;
    if(str == "HEXAROTOR")
        return MAV_TYPE::MAV_TYPE_HEXAROTOR;
    if(str == "OCTOROTOR")
        return MAV_TYPE::MAV_TYPE_OCTOROTOR;
    if(str == "ONBOARD_CONTROLLER")
        return MAV_TYPE::MAV_TYPE_ONBOARD_CONTROLLER;
    if(str == "FIXED_WING")
        return MAV_TYPE::MAV_TYPE_FIXED_WING;

    std::cout << "Unsupported MAV type (from string): " + str << std::endl;
    return MAV_TYPE::MAV_TYPE_ENUM_END;
}

}

#endif // MAV_TYPE_H
