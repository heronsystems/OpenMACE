#ifndef COMMAND_MISSION_ITEM_H
#define COMMAND_MISSION_ITEM_H

#include "generic_short_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace ExternalLink {


class CommandMissionItem : public Controller_GenericShortCommand<command_item::ActionMissionCommand, (uint8_t)MAV_CMD::MAV_CMD_USER_1>
{
public:

    CommandMissionItem(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan);

protected:

    virtual void FillCommand(const command_item::ActionMissionCommand &commandItem, mavlink_command_short_t &cmd) const;

    virtual void BuildCommand(const mavlink_command_short_t &message, command_item::ActionMissionCommand &data) const;
};


}

#endif // COMMAND_MISSION_ITEM_H
