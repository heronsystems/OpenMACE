#ifndef SYSTEM_TYPE_H
#define SYSTEM_TYPE_H

#include <string>
#include <stdexcept>

namespace Data
{
enum class SystemType
{
    SYSTEM_TYPE_GENERIC=0, /* Generic micro air vehicle. | */
    SYSTEM_TYPE_HELICOPTER=1, /* Normal helicopter with tail rotor. | */
    SYSTEM_TYPE_GCS=2, /* Operator control unit / ground control station | */
    SYSTEM_TYPE_REPEATER=3, /* Operator control unit / ground control station | */
    SYSTEM_TYPE_GROUND_ROVER=4, /* Ground rover | */
    SYSTEM_TYPE_SURFACE_BOAT=5, /* Surface vessel, boat, ship | */
    SYSTEM_TYPE_TRICOPTER=6, /* Tricopter | */
    SYSTEM_TYPE_QUADROTOR=7, /* Quadrotor | */
    SYSTEM_TYPE_HEXAROTOR=8, /* Hexarotor | */
    SYSTEM_TYPE_OCTOROTOR=9, /* Octorotor | */
    SYSTEM_TYPE_ONBOARD_CONTROLLER=10, /* Onboard companion controller | */
    SYSTEM_TYPE_FIXED_WING=11 /* Onboard companion controller | */
};

inline bool isSystemTypeRotary(const SystemType &type)
{
    bool rtn = false;
    switch (type) {
    case SystemType::SYSTEM_TYPE_HELICOPTER:
    case SystemType::SYSTEM_TYPE_TRICOPTER:
    case SystemType::SYSTEM_TYPE_QUADROTOR:
    case SystemType::SYSTEM_TYPE_HEXAROTOR:
    case SystemType::SYSTEM_TYPE_OCTOROTOR:
        rtn = true;
        break;
    default:
        rtn = false;
        break;
    }
    return rtn;
}

inline std::string SystemTypeToString(const SystemType &systemType) {
    switch (systemType) {
    case SystemType::SYSTEM_TYPE_GENERIC:
        return "GENERIC";
    case SystemType::SYSTEM_TYPE_HELICOPTER:
        return "HELICOPTER";
    case SystemType::SYSTEM_TYPE_GCS:
        return "GCS";
    case SystemType::SYSTEM_TYPE_REPEATER:
        return "REPEATER";
    case SystemType::SYSTEM_TYPE_GROUND_ROVER:
        return "GROUND_ROVER";
    case SystemType::SYSTEM_TYPE_SURFACE_BOAT:
        return "SURFACE_BOAT";
    case SystemType::SYSTEM_TYPE_TRICOPTER:
        return "TRICOPTER";
    case SystemType::SYSTEM_TYPE_QUADROTOR:
        return "QUADROTOR";
    case SystemType::SYSTEM_TYPE_HEXAROTOR:
        return "HEXAROTOR";
    case SystemType::SYSTEM_TYPE_OCTOROTOR:
        return "OCTOROTOR";
    case SystemType::SYSTEM_TYPE_ONBOARD_CONTROLLER:
        return "ONBOARD_CONTROLLER";
    case SystemType::SYSTEM_TYPE_FIXED_WING:
        return "FIXED_WING";
    default:
        throw std::runtime_error("Unknown autopilot type seen");
    }
}

inline SystemType SystemTypeFromString(const std::string &str) {
    if(str == "GENERIC")
        return SystemType::SYSTEM_TYPE_GENERIC;
    if(str == "HELICOPTER")
        return SystemType::SYSTEM_TYPE_HELICOPTER;
    if(str == "GCS")
        return SystemType::SYSTEM_TYPE_GCS;
    if(str == "REPEATER")
        return SystemType::SYSTEM_TYPE_REPEATER;
    if(str == "GROUND_ROVER")
        return SystemType::SYSTEM_TYPE_GROUND_ROVER;
    if(str == "SURFACE_BOAT")
        return SystemType::SYSTEM_TYPE_SURFACE_BOAT;
    if(str == "TRICOPTER")
        return SystemType::SYSTEM_TYPE_TRICOPTER;
    if(str == "QUADROTOR")
        return SystemType::SYSTEM_TYPE_QUADROTOR;
    if(str == "HEXAROTOR")
        return SystemType::SYSTEM_TYPE_HEXAROTOR;
    if(str == "OCTOROTOR")
        return SystemType::SYSTEM_TYPE_OCTOROTOR;
    if(str == "ONBOARD_CONTROLLER")
        return SystemType::SYSTEM_TYPE_ONBOARD_CONTROLLER;
    if(str == "FIXED_WING")
        return SystemType::SYSTEM_TYPE_FIXED_WING;
    throw std::runtime_error("Unknown autopilot type seen");
}

}

#endif // SYSTEM_TYPE_H
