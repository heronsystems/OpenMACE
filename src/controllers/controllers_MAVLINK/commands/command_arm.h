#ifndef MAVLINK_CONTROLLER_COMMAND_ARM_H
#define MAVLINK_CONTROLLER_COMMAND_ARM_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKUXVControllers{


class CommandARM : public Controller_GenericLongCommand<command_item::ActionArm, MAV_CMD_COMPONENT_ARM_DISARM>
{
public:
    CommandARM(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::ActionArm, MAV_CMD_COMPONENT_ARM_DISARM>(cb, queue, linkChan)
    {

    }

    virtual ~CommandARM() = default;

protected:

    virtual void FillCommand(const command_item::ActionArm &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = static_cast<uint8_t>(commandItem.getTargetSystem());
        cmd.param1 = commandItem.getRequestArm();
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::ActionArm &data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};

} //end of namespace MAVLINKVehicleControllers


#endif // MAVLINK_CONTROLLER_COMMAND_ARM_H
