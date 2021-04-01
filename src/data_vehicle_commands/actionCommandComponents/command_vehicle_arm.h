#ifndef COMMAND_VEHICLE_ARM_H
#define COMMAND_VEHICLE_ARM_H

#include "data_vehicle_commands/abstract_action_command.h"

namespace DataVehicleCommands {

class CommandVehicleArm : public AbstractActionCommand
{
public:
    virtual CommandTypes getCommandType() const;

    virtual ActionCommandTypes getActionItemType() const;

    virtual std::string getDescription() const;

    void setVehicleArm(const bool &arm){
        m_VehicleArm = arm;
    }

    bool getVehicleArm()
    {
        m_VehicleArm;
    }

private:
    bool m_VehicleArm;
};

} //end of namespace DataVehicleCommands
#endif // COMMAND_VEHICLE_ARM_H
