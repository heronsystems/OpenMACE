#ifndef COMMAND_MISSION_ITEM_H
#define COMMAND_MISSION_ITEM_H

#include "generic_short_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace Controllers {


template <typename MESSAGETYPE>
class CommandMissionItem : public Controller_GenericShortCommand<MESSAGETYPE, command_item::ActionMissionCommand, (uint8_t)command_item::COMMANDITEM::CI_ACT_MISSIONCOMMAND>
{
public:
    CommandMissionItem(const IMessageNotifier<MESSAGETYPE> *cb, MessageModuleTransmissionQueue<MESSAGETYPE> *queue, int linkChan) :
        Controller_GenericShortCommand<MESSAGETYPE, command_item::ActionMissionCommand, (uint8_t)command_item::COMMANDITEM::CI_ACT_MISSIONCOMMAND>(cb, queue, linkChan)
    {

    }

    protected:

    virtual void FillCommand(const command_item::ActionMissionCommand &commandItem, mace_command_short_t &cmd) const
    {
        cmd.param = (uint8_t)commandItem.getMissionCommandAction();
    }

    virtual void BuildCommand(const mace_command_short_t &message, command_item::ActionMissionCommand &data) const
    {
        data->setMissionCommandType((Data::MissionCommandAction)message.param);
    }
};


}

#endif // COMMAND_MISSION_ITEM_H
