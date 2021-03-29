#ifndef COMMAND_VEHICLE_RTL_H
#define COMMAND_VEHICLE_RTL_H

#include "data_vehicle_commands/abstract_action_command.h"

namespace DataVehicleCommands {

class CommandVehicleRTL : public AbstractActionCommand
{
public:
    virtual CommandTypes getCommandType() const;

    virtual ActionCommandTypes getActionItemType() const;

    virtual std::string getDescription() const;

};

} //end of namespace DataVehicleCommands
#endif // COMMAND_VEHICLE_RTL_H
