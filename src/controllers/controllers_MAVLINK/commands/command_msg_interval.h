#ifndef COMMAND_MSG_INTERVAL_H
#define COMMAND_MSG_INTERVAL_H

#include <unordered_map>
#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKUXVControllers{

class CommandMSGInterval : public Controller_GenericLongCommand<command_item::ActionMessageInterval, MAV_CMD_SET_MESSAGE_INTERVAL>
{
public:
    CommandMSGInterval(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::ActionMessageInterval, MAV_CMD_SET_MESSAGE_INTERVAL>(cb, queue, linkChan, "MSGInterval")
    {

    }

    virtual ~CommandMSGInterval() = default;

    void addIntervalRequest(const command_item::ActionMessageInterval &request);

    void removeIntervalRequest(const unsigned int &messageID);

    bool removeCurrentAndTransmitNext();

    bool transmitNextRequest();

    unsigned int getCurrentRequestID();

    size_t getNumRequests() const
    {
        return m_MapIntervalRequest.size();
    }


protected:

    virtual void FillCommand(const command_item::ActionMessageInterval &command, mavlink_command_long_t &cmd) const
    {
        cmd.param1 = command.getMessageID();
        cmd.param2 = command.getMessageInterval();
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::ActionMessageInterval &data) const
    {
        UNUSED(message);
        UNUSED(data);
    }

    virtual bool Finish_Receive(const mavlink_command_ack_t &msg, const MavlinkEntityKey &sender, uint8_t & ack, MavlinkEntityKey &queueObj)
    {
        if(msg.command != MAV_CMD_SET_MESSAGE_INTERVAL)
        {
            return false;
        }
        queueObj = sender;
        ack = msg.result;
        return true;
    }

private:
    unsigned int m_CurrentRequestID = 0;

    std::map<unsigned int, command_item::ActionMessageInterval> m_MapIntervalRequest;

};

} //end of namespace MAVLINKVehicleControllers

#endif // COMMAND_MSG_INTERVAL_H
