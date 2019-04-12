#ifndef COMMAND_ARM_H
#define COMMAND_ARM_H

#include "generic_short_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace Controllers {


template <typename MESSAGETYPE>
class CommandARM : public Controller_GenericShortCommand<MESSAGETYPE, CommandItem::ActionArm, (uint8_t)CommandItem::COMMANDITEM::CI_ACT_ARM>
{
public:
    CommandARM(const IMessageNotifier<MESSAGETYPE> *cb, MessageModuleTransmissionQueue<MESSAGETYPE> *queue, int linkChan) :
        Controller_GenericShortCommand<MESSAGETYPE, CommandItem::ActionArm, (uint8_t)CommandItem::COMMANDITEM::CI_ACT_ARM>(cb, queue, linkChan)
    {

    }

    protected:

    virtual void FillCommand(const CommandItem::ActionArm &commandItem, mace_command_short_t &cmd) const
    {
        cmd.param = commandItem.getRequestArm();
    }

    virtual void BuildCommand(const mace_command_short_t &message, CommandItem::ActionArm &data) const
    {
        data->setVehicleArm(message.param);
    }
};


}


#endif // COMMAND_ARM_H
