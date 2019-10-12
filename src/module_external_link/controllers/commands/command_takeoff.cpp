#include "command_takeoff.h"

namespace ExternalLink {

    CommandTakeoff::CommandTakeoff(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::SpatialTakeoff, static_cast<uint8_t>(command_item::COMMANDTYPE::CI_NAV_TAKEOFF)>(cb, queue, linkChan)
    {

    }

    void CommandTakeoff::FillCommand(const command_item::SpatialTakeoff &commandItem, mace_command_long_t &cmd) const
    {
        commandItem.populateCommandItem(cmd);
    }

    void CommandTakeoff::BuildCommand(const mace_command_long_t &message, SpatialTakeoff &data) const
    {
        data.fromCommandItem(message);
    }


}
