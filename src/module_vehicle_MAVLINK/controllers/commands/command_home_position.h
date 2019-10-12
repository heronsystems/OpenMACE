#ifndef MODULE_VEHICLE_MAVLINK_COMMAND_HOME_POSITION_H
#define MODULE_VEHICLE_MAVLINK_COMMAND_HOME_POSITION_H

#include "generic_long_command.h"
#include "generic_int_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKUXVControllers{


class Command_HomePositionSet : public Controller_GenericLongCommand<command_item::SpatialHome, MAV_CMD_DO_SET_HOME>
{
public:
    Command_HomePositionSet(const Controllers::IMessageNotifier<mavlink_message_t, int> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::SpatialHome, MAV_CMD_DO_SET_HOME>(cb, queue, linkChan)
    {

    }

    virtual ~Command_HomePositionSet() = default;

protected:

    void FillCommand(const command_item::SpatialHome &commandItem, mavlink_command_long_t &cmd) const override
    {
        UNUSED(commandItem); UNUSED(cmd);
    }

    void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialHome &data) const override
    {
        UNUSED(message); UNUSED(data);
    }
};

class Command_HomePositionGet : public Controller_GenericLongCommand<command_item::SpatialHome, MAV_CMD_GET_HOME_POSITION>
{
public:
    Command_HomePositionGet(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::SpatialHome, MAV_CMD_GET_HOME_POSITION>(cb, queue, linkChan)
    {

    }

    protected:

    void FillCommand(const command_item::SpatialHome &commandItem, mavlink_command_long_t &cmd) const override
    {
        UNUSED(commandItem); UNUSED(cmd);
    }

    void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialHome &data) const override
    {
        UNUSED(message); UNUSED(data);
    }
};

} //end of namespace MAVLINKVehicleControllers


#endif // MODULE_VEHICLE_MAVLINK_COMMAND_HOME_POSITION_H
