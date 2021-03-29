#ifndef COMMAND_VEHICLE_MODE_H
#define COMMAND_VEHICLE_MODE_H

#include <string>

#include "data_vehicle_commands/abstract_action_command.h"

namespace DataVehicleCommands {

class CommandVehicleMode : public AbstractActionCommand
{
public:
    virtual CommandTypes getCommandType() const;

    virtual ActionCommandTypes getActionItemType() const;

    virtual std::string getDescription() const;

public:
    void setRequestMode(const std::string &mode)
    {
        m_CommandVehicleMode = mode;
    }

    std::string getRequestMode(){
        return m_CommandVehicleMode;
    }

private:
    std::string m_CommandVehicleMode;

};

}

#endif // COMMAND_VEHICLE_MODE_H
