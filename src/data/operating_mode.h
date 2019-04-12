#ifndef OPERATING_MODE_H
#define OPERATING_MODE_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class VehicleOperatingMode{
    STANDARD,
    COMPANION
};

inline std::string OperatingModeToString(const VehicleOperatingMode &operatingMode) {
    switch (operatingMode) {
    case VehicleOperatingMode::STANDARD:
        return "STANDARD";
    case VehicleOperatingMode::COMPANION:
        return "COMPANION";
    default:
        throw std::runtime_error("Unknown operating mode seen");
    }
}

inline VehicleOperatingMode OperatingModeFromString(const std::string &str) {
    if(str == "STANDARD")
        return VehicleOperatingMode::STANDARD;
    if(str == "COMPANION")
        return VehicleOperatingMode::COMPANION;

    throw std::runtime_error("Unknown operating mode seen");
}

}

#endif // OPERATING_MODE_H
