#ifndef COMMAND_MSG_INTERVAL_H
#define COMMAND_MSG_INTERVAL_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKVehicleControllers{
struct STRUCT_CMDMSGINTERVAL
{
    uint8_t targetSytem;
    int32_t interval_us;
    uint16_t message_id;

};

class CommandMSGInterval : public Controller_GenericLongCommand<STRUCT_CMDMSGINTERVAL, MAV_CMD_SET_MESSAGE_INTERVAL>
{
public:
    CommandMSGInterval(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<STRUCT_CMDMSGINTERVAL, MAV_CMD_SET_MESSAGE_INTERVAL>(cb, queue, linkChan)
    {

    }

    virtual ~CommandMSGInterval() = default;

protected:

    virtual void FillCommand(const STRUCT_CMDMSGINTERVAL &command, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = command.targetSytem;
        cmd.param1 = command.message_id;
        cmd.param2 = command.interval_us;

    }

    virtual void BuildCommand(const mavlink_command_long_t &message, STRUCT_CMDMSGINTERVAL &data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};

} //end of namespace MAVLINKVehicleControllers

#endif // COMMAND_MSG_INTERVAL_H
