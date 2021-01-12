#ifndef CONTROLLER_COMMAND_SYSTEM_MODE_H
#define CONTROLLER_COMMAND_SYSTEM_MODE_H

#include <mavlink.h>

#include "generic_short_command.h"

#include "data_generic_command_item/do_items/action_change_mode.h"

namespace ExternalLink {

class ControllerCommand_SystemMode : public Controller_GenericShortCommand<command_item::ActionChangeMode, MAV_CMD::MAV_CMD_DO_SET_MODE>
{
public:

    ControllerCommand_SystemMode(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan);

protected:

    virtual void FillCommand(const command_item::ActionChangeMode &commandItem, mavlink_command_short_t &cmd) const;

    virtual void BuildCommand(const mavlink_command_short_t &message, command_item::ActionChangeMode &data) const;
};

} //end of namespace ExternalLink

#endif // CONTROLLER_COMMAND_SYSTEM_MODE_H
