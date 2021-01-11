#ifndef COMMAND_ARM_H
#define COMMAND_ARM_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace ExternalLink {


class CommandARM : public Controller_GenericLongCommand<command_item::ActionArm, static_cast<uint8_t>(MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM)>
{
public:

    CommandARM(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan);

protected:

    virtual void FillCommand(const command_item::ActionArm &commandItem, mavlink_command_long_t &cmd) const;

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::ActionArm &data) const;

};


}


#endif // COMMAND_ARM_H
