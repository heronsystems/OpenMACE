#ifndef COMMAND_TYPES_H
#define COMMAND_TYPES_H

#include <string>
#include <stdexcept>

namespace DataVehicleCommands
{


enum class CommandTypes{
    ACTION,
    GUIDED,
    MISSION
};

inline std::string CommandTypeToString(const CommandTypes &cmdType) {
    switch (cmdType) {
    case CommandTypes::ACTION:
        return "ACTION";
    case CommandTypes::GUIDED:
        return "GUIDED";
    case CommandTypes::MISSION:
        return "MISSION";
    default:
        throw std::runtime_error("Unknown command type seen");
    }
}

inline CommandTypes CommandTypeFromString(const std::string &str) {
    if(str == "ACTION")
        return CommandTypes::ACTION;
    if(str == "GUIDED")
        return CommandTypes::GUIDED;
    if(str == "MISSION")
        return CommandTypes::MISSION;

    throw std::runtime_error("Unknown command type seen");
}

}

#endif // COMMAND_TYPES_H
