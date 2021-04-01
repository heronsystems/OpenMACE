#include "command_vehicle_mode.h"

namespace DataVehicleCommands {

CommandTypes CommandVehicleMode::getCommandType() const
{
    return CommandTypes::ACTION;
}

ActionCommandTypes CommandVehicleMode::getActionItemType() const
{
    return ActionCommandTypes::CHANGE_MODE;
}

std::string CommandVehicleMode::getDescription() const
{
    return "This will change the mode of the aircraft";
}

}





