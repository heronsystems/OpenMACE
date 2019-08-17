#include "command_mission_item.h"

namespace ExternalLink {

    CommandMissionItem::CommandMissionItem(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericShortCommand<command_item::ActionMissionCommand, static_cast<uint8_t>(command_item::COMMANDTYPE::CI_ACT_MISSIONITEM)>(cb, queue, linkChan)
    {

    }

    void CommandMissionItem::FillCommand(const command_item::ActionMissionCommand &commandItem, mace_command_short_t &cmd) const
    {
        cmd.param = (uint8_t)commandItem.getMissionCommandAction();
    }

    void CommandMissionItem::BuildCommand(const mace_command_short_t &message, command_item::ActionMissionCommand &data) const
    {
        data.setMissionCommandType((Data::MissionCommandAction)message.param);
    }

}
