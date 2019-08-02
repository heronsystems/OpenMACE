#ifndef MODULE_VEHICLE_MAVLINK_COMMAND_RTL_H
#define MODULE_VEHICLE_MAVLINK_COMMAND_RTL_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKVehicleControllers {


class CommandRTL : public Controller_GenericLongCommand<command_item::SpatialRTL, MAV_CMD_NAV_RETURN_TO_LAUNCH>
{
public:
    CommandRTL(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::SpatialRTL, MAV_CMD_NAV_RETURN_TO_LAUNCH>(cb, queue, linkChan)
    {

    }

    virtual ~CommandRTL() = default;

protected:

    virtual void FillCommand(const command_item::SpatialRTL &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
        UNUSED(commandItem);
        UNUSED(cmd);
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialRTL &data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};


}

#endif // MODULE_VEHICLE_MAVLINK_COMMAND_RTL_H
