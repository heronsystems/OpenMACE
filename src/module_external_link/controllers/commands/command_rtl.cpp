#include "command_rtl.h"

namespace ExternalLink {


    CommandRTL::CommandRTL(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericShortCommand<command_item::SpatialRTL, static_cast<uint8_t>(MAV_CMD::MAV_CMD_NAV_RETURN_TO_LAUNCH)>(cb, queue, linkChan)
    {

    }

    void CommandRTL::FillCommand(const command_item::SpatialRTL &commandItem, mavlink_command_short_t &cmd) const
    {
        UNUSED(commandItem);
        UNUSED(cmd);
    }

    void CommandRTL::BuildCommand(const mavlink_command_short_t &message, command_item::SpatialRTL &data) const
    {
        UNUSED(message);
        UNUSED(data);
    }

}
