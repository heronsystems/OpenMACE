#include "command_arm.h"

namespace ExternalLink {

    CommandARM::CommandARM(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::ActionArm, static_cast<uint8_t>(MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM)>(cb, queue, linkChan)
    {

    }

    void CommandARM::FillCommand(const command_item::ActionArm &commandItem, mavlink_command_long_t &cmd) const
    {
        commandItem.populateCommandItem(cmd);
    }

    void CommandARM::BuildCommand(const mavlink_command_long_t &message, command_item::ActionArm &data) const
    {
        data.fromCommandItem(message);
    }

}
