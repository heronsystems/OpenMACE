#include "command_vehicle_rtl.h"

namespace DataVehicleCommands {

CommandTypes CommandVehicleRTL::getCommandType() const
{
    return CommandTypes::ACTION;
}

ActionCommandTypes CommandVehicleRTL::getActionItemType() const
{
    return ActionCommandTypes::RTL;
}

std::string CommandVehicleRTL::getDescription() const
{
    return "This will command the aircraft to RTL.";
}

}
