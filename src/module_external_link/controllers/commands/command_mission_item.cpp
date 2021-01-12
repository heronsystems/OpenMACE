#include "command_mission_item.h"

namespace ExternalLink {

    CommandMissionItem::CommandMissionItem(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericShortCommand<command_item::ActionMissionCommand, static_cast<uint8_t>(MAV_CMD::MAV_CMD_USER_1)>(cb, queue, linkChan)
    {

    }

    void CommandMissionItem::FillCommand(const command_item::ActionMissionCommand &commandItem, mavlink_command_short_t &cmd) const
    {
        cmd.param1 = (uint8_t)commandItem.getMissionCommandAction();
    }

    void CommandMissionItem::BuildCommand(const mavlink_command_short_t &message, command_item::ActionMissionCommand &data) const
    {
        data.setMissionCommandType((Data::MissionCommandAction)message.param1);
    }

}
