#include "command_execute_spatial_action.h"

namespace ExternalLink {

Controller_GoTo::Controller_GoTo(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
    BasicExternalLinkController_ModuleKeyed<command_item::Action_ExecuteSpatialItem>(cb, queue, linkChan),
    ActionSend_CommandGoTo_Broadcast(this, ModuleToSysIDCompIDConverter<mavlink_execute_spatial_action_t>(mavlink_msg_execute_spatial_action_encode_chan)),
    ActionSend_CommandGoTo_TargedWithResponse(this, ModuleToSysIDCompIDConverter<mavlink_execute_spatial_action_t>(mavlink_msg_execute_spatial_action_encode_chan)),
    ActionSend_CommandGoTo_ReceiveRespond(this, mavlink_msg_execute_spatial_action_decode, ModuleToSysIDCompIDConverter<mavlink_execute_spatial_action_ack_t>(mavlink_msg_execute_spatial_action_ack_encode_chan)),
    ActionFinish_CommandGoTo(this, mavlink_msg_execute_spatial_action_ack_decode)
{

}

void Controller_GoTo::FillCommand(const command_item::Action_ExecuteSpatialItem &commandItem, mavlink_execute_spatial_action_t &cmd) const
{
    UNUSED(commandItem);
    UNUSED(cmd);
}

void Controller_GoTo::BuildCommand(const mavlink_execute_spatial_action_t &message, command_item::Action_ExecuteSpatialItem &data) const
{
    UNUSED(message);
    UNUSED(data);
//    data.setTargetSystem(-1);

//    data.updateFromGoToCommand(message);
}

}
