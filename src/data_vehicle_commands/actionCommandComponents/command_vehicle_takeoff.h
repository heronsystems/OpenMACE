#ifndef COMMAND_VEHICLE_TAKEOFF_H
#define COMMAND_VEHICLE_TAKEOFF_H

#include <string>
#include "data_vehicle_commands/abstract_action_command.h"

#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

namespace DataVehicleCommands {

template<class T>
class CommandVehicleTakeoff : public AbstractActionCommand
{
public:
    virtual CommandTypes getCommandType() const;

    virtual ActionCommandTypes getActionItemType() const;

    virtual std::string getDescription() const;

public:
    CommandVehicleTakeoff();

    void setLocation(const T &location);
    T getLocation();

    void setPitch(const double &pitch){
        m_pitch = pitch;
    }
    double getPitch(){
        return m_pitch;
    }

    void setYaw(const double &yaw){
        m_yawAngle = yaw;
    }

    double getYaw(){
        return m_yawAngle;
    }

private:
    T m_Location;
    double m_pitch;
    double m_yawAngle;
};

}

#endif // COMMAND_VEHICLE_TAKEOFF_H
