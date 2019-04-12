#include "controller_goto.h"

namespace ExternalLink {

Controller_GoTo::Controller_GoTo(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
    BasicExternalLinkController_ModuleKeyed<CommandItem::CommandGoTo>(cb, queue, linkChan),
    ActionSend_CommandGoTo_Broadcast(this, ModuleToSysIDCompIDConverter<mace_command_goto_t>(mace_msg_command_goto_encode_chan)),
    ActionSend_CommandGoTo_TargedWithResponse(this, ModuleToSysIDCompIDConverter<mace_command_goto_t>(mace_msg_command_goto_encode_chan)),
    ActionSend_CommandGoTo_ReceiveRespond(this, mace_msg_command_goto_decode, ModuleToSysIDCompIDConverter<mace_command_goto_ack_t>(mace_msg_command_goto_ack_encode_chan)),
    ActionFinish_CommandGoTo(this, mace_msg_command_goto_ack_decode)
{

}

void Controller_GoTo::FillCommand(const CommandItem::CommandGoTo &commandItem, mace_command_goto_t &cmd) const
{
    commandItem.getSpatialCommand()->setGoToCommand(cmd);
}

void Controller_GoTo::BuildCommand(const mace_command_goto_t &message, CommandItem::CommandGoTo &data) const
{
    data.setTargetSystem(-1);

    data.updateFromGoToCommand(message);
}

}
