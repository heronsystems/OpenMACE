#ifndef MODULE_VEHICLE_MAVLINK_COMMAND_ARM_H
#define MODULE_VEHICLE_MAVLINK_COMMAND_ARM_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKUXVControllers{


class CommandSurfaceDeflection : public Controller_GenericLongCommand<command_item::Action_SetSurfaceDeflection, MAV_CMD_AI_DEFLECTION>
{
public:
    CommandSurfaceDeflection(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::Action_SetSurfaceDeflection, MAV_CMD_AI_DEFLECTION>(cb, queue, linkChan)
    {

    }

    virtual ~CommandSurfaceDeflection() = default;

protected:

    virtual void FillCommand(const command_item::Action_SetSurfaceDeflection &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = static_cast<uint8_t>(commandItem.getTargetSystem());
        cmd.param1 = commandItem._surfaceDeflection.elevator;
        cmd.param2 = commandItem._surfaceDeflection.aileron;
        cmd.param3 = commandItem._surfaceDeflection.rudder;
        cmd.param4 = commandItem._surfaceDeflection.throttle;
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::Action_SetSurfaceDeflection &data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};

} //end of namespace MAVLINKVehicleControllers


#endif // MODULE_VEHICLE_MAVLINK_COMMAND_ARM_H
