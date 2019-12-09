#ifndef VEHICLE_TYPES_H
#define VEHICLE_TYPES_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class VehicleTypes
{
    COPTER,
    PLANE,
    UNKNOWN
};

inline std::string VehicleTypesToString(const VehicleTypes &vehicleType) {
    switch (vehicleType) {
    case VehicleTypes::COPTER:
        return "COPTER";
    case VehicleTypes::PLANE:
        return "PLANE";
    case VehicleTypes::UNKNOWN:
        return "UNKNOWN";
    default:
        throw std::runtime_error("Unknown operating mode seen in VehicleTypesToString method.");
    }
}

inline VehicleTypes VehicleTypesFromString(const std::string &str) {
    if(str == "COPTER")
        return VehicleTypes::COPTER;
    if(str == "PLANE")
        return VehicleTypes::PLANE;
    if(str == "UNKNOWN")
        return VehicleTypes::UNKNOWN;
    throw std::runtime_error("Unknown operating mode seen in VehicleTypesFromString method");
}

}

#endif // VEHICLE_TYPES_H
