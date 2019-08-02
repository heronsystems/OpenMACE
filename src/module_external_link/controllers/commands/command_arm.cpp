#include "command_arm.h"

namespace ExternalLink {

    CommandARM::CommandARM(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericShortCommand<command_item::ActionArm, (uint8_t)command_item::COMMANDTYPE::CI_ACT_ARM>(cb, queue, linkChan)
    {

    }

    void CommandARM::FillCommand(const command_item::ActionArm &commandItem, mace_command_short_t &cmd) const
    {
        cmd.param = commandItem.getRequestArm();
    }

    void CommandARM::BuildCommand(const mace_command_short_t &message, command_item::ActionArm &data) const
    {
        data.setVehicleArm(message.param);
    }

}
