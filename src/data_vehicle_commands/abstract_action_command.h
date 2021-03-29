#ifndef ABSTRACT_ACTION_COMMAND_H
#define ABSTRACT_ACTION_COMMAND_H

#include "command_types.h"

namespace DataVehicleCommands {

enum class ActionCommandTypes{
    ARM,
    CHANGE_MODE,
    LAND,
    RTL,
    TAKEOFF
};

class AbstractActionCommand
{
public:

    virtual CommandTypes getCommandType() const = 0;

    virtual ActionCommandTypes getActionItemType() const = 0;

    virtual std::string getDescription() const = 0;

    inline std::string CommandActionTypeToString(const ActionCommandTypes &cmdType) {
        switch (cmdType) {
        case ActionCommandTypes::ARM:
            return "ARM";
        case ActionCommandTypes::CHANGE_MODE:
            return "CHANGE_MODE";
        case ActionCommandTypes::LAND:
            return "LAND";
        case ActionCommandTypes::RTL:
            return "RTL";
        case ActionCommandTypes::TAKEOFF:
            return "TAKEOFF";
        default:
            throw std::runtime_error("Unknown action command type seen");
        }
    }

    inline ActionCommandTypes CommandActionTypeFromString(const std::string &str) {
        if(str == "ARM")
            return ActionCommandTypes::ARM;
        if(str == "CHANGE_MODE")
            return ActionCommandTypes::CHANGE_MODE;
        if(str == "LAND")
            return ActionCommandTypes::LAND;
        if(str == "RTL")
            return ActionCommandTypes::RTL;
        if(str == "TAKEOFF")
            return ActionCommandTypes::TAKEOFF;
        throw std::runtime_error("Unknown action command type seen");
    }

};

} //end of namespace DataVehicleCommands

#endif // ABSTRACT_ACTION_COMMAND_H
