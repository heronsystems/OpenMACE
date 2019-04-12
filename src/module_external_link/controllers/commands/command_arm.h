#ifndef COMMAND_ARM_H
#define COMMAND_ARM_H

#include "generic_short_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace ExternalLink {


class CommandARM : public Controller_GenericShortCommand<CommandItem::ActionArm, (uint8_t)CommandItem::COMMANDITEM::CI_ACT_ARM>
{
public:

    CommandARM(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan);

protected:

    virtual void FillCommand(const CommandItem::ActionArm &commandItem, mace_command_short_t &cmd) const;

    virtual void BuildCommand(const mace_command_short_t &message, CommandItem::ActionArm &data) const;

};


}


#endif // COMMAND_ARM_H
