#include "command_arm.h"

namespace ExternalLink {

    CommandARM::CommandARM(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericShortCommand<command_item::ActionArm, static_cast<uint8_t>(command_item::COMMANDTYPE::CI_ACT_ARM)>(cb, queue, linkChan)
    {

    }

    void CommandARM::FillCommand(const command_item::ActionArm &commandItem, mace_command_short_t &cmd) const
    {
        commandItem.populateCommandItem(cmd);
    }

    void CommandARM::BuildCommand(const mace_command_short_t &message, command_item::ActionArm &data) const
    {
        data.fromCommandItem(message);
    }

}
