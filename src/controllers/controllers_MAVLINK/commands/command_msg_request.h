#ifndef COMMAND_MSG_REQUEST_H
#define COMMAND_MSG_REQUEST_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKUXVControllers{

class CommandMSGRequest : public Controller_GenericLongCommand<command_item::ActionMessageRequest, MAV_CMD_REQUEST_MESSAGE>
{
public:
    CommandMSGRequest(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::ActionMessageRequest, MAV_CMD_REQUEST_MESSAGE>(cb, queue, linkChan)
    {

    }

    ~CommandMSGRequest() override = default;

protected:

    void FillCommand(const command_item::ActionMessageRequest &command, mavlink_command_long_t &cmd) const override
    {
        cmd.param1 = command.getMessageID();
    }

    void BuildCommand(const mavlink_command_long_t &message, command_item::ActionMessageRequest &data) const override
    {
        UNUSED(message);
        UNUSED(data);
    }
};

} //end of namespace MAVLINKVehicleControllers

#endif // COMMAND_MSG_REQUEST_H
