#include "command_vehicle_arm.h"

namespace DataVehicleCommands {

CommandTypes CommandVehicleArm::getCommandType() const
{
    return CommandTypes::ACTION;
}

ActionCommandTypes CommandVehicleArm::getActionItemType() const
{
    return ActionCommandTypes::ARM;
}

std::string CommandVehicleArm::getDescription() const
{
    return "This will command the aircraft to arm/disarm.";
}

}


