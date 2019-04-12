#include "command_mission_item.h"

namespace ExternalLink {

    CommandMissionItem::CommandMissionItem(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericShortCommand<CommandItem::ActionMissionCommand, (uint8_t)CommandItem::COMMANDITEM::CI_ACT_MISSIONCOMMAND>(cb, queue, linkChan)
    {

    }

    void CommandMissionItem::FillCommand(const CommandItem::ActionMissionCommand &commandItem, mace_command_short_t &cmd) const
    {
        cmd.param = (uint8_t)commandItem.getMissionCommandAction();
    }

    void CommandMissionItem::BuildCommand(const mace_command_short_t &message, CommandItem::ActionMissionCommand &data) const
    {
        data.setMissionCommandType((Data::MissionCommandAction)message.param);
    }

}
