#ifndef MODULE_VEHICLE_MAVLINK_CONTROLLER_COMMAND_SET_SURFACE_DEFLECTION_H
#define MODULE_VEHICLE_MAVLINK_CONTROLLER_COMMAND_SET_SURFACE_DEFLECTION_H

#include <mavlink.h>

#include "generic_long_command.h"

#include "data_generic_command_item/do_items/action_set_surface_deflection.h"

namespace MAVLINKUXVControllers {

class Controller_CommandSurfaceDeflection : public Controller_GenericLongCommand<command_item::Action_SetSurfaceDeflection, MAV_CMD::SET_SURFACE_DEFLECTION_NORMALIZED>
{
public:
    Controller_CommandSurfaceDeflection(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::Action_SetSurfaceDeflection, MAV_CMD::SET_SURFACE_DEFLECTION_NORMALIZED>(cb, queue, linkChan)
    {

    }

    virtual ~Controller_CommandSurfaceDeflection() = default;

protected:

    virtual void FillCommand(const command_item::Action_SetSurfaceDeflection &commandItem, mavlink_command_long_t &cmd) const
    {               
        cmd.param2 = static_cast<float>(commandItem._surfaceDeflection._elevator);
        cmd.param3 = static_cast<float>(commandItem._surfaceDeflection._aileron);
        cmd.param4 = static_cast<float>(commandItem._surfaceDeflection._rudder);
        cmd.param5 = static_cast<float>(commandItem._surfaceDeflection._throttle);
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::Action_SetSurfaceDeflection &data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};

} //end of namespace MAVLINKVehicleControllers

#endif // MODULE_VEHICLE_MAVLINK_CONTROLLER_COMMAND_SET_SURFACE_DEFLECTION_H
