#ifndef VEHICLE_COMMAND_TYPES_H
#define VEHICLE_COMMAND_TYPES_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class VehicleCommandTypes{
    ACTION,
    GUIDED,
    MISSION
};

inline std::string CommandTypeToString(const VehicleCommandTypes &cmdType) {
    switch (cmdType) {
    case VehicleCommandTypes::ACTION:
        return "ACTION";
    case VehicleCommandTypes::GUIDED:
        return "GUIDED";
    case VehicleCommandTypes::MISSION:
        return "MISSION";
    default:
        throw std::runtime_error("Unknown command type seen in CommandTypeToString method");
    }
}

inline VehicleCommandTypes CommandTypeFromString(const std::string &str) {
    if(str == "ACTION")
        return VehicleCommandTypes::ACTION;
    if(str == "GUIDED")
        return VehicleCommandTypes::GUIDED;
    if(str == "MISSION")
        return VehicleCommandTypes::MISSION;

    throw std::runtime_error("Unknown command type seen in CommandTypeToString method");
}

}
#endif // VEHICLE_COMMAND_TYPES_H
