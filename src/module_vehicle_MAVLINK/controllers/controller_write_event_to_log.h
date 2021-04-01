#ifndef CONTROLLER_WRITE_EVENT_TO_LOG_H
#define CONTROLLER_WRITE_EVENT_TO_LOG_H

#ifdef WITH_HERON_MAVLINK_SUPPORT

#include <mavlink.h>

#include "common/common.h"

#include "controllers/actions/action_broadcast.h"

#include "module_vehicle_MAVLINK/mavlink_entity_key.h"
#include "module_vehicle_MAVLINK/controllers/common.h"

#include "data_generic_command_item/mace/ai_items/action_event_tag.h"

namespace MAVLINKUXVControllers {

struct LogDetails
{
    LOGGING_EVENT_TAGS type;
    std::string details;
};

using WriteToLogBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<command_item::Action_EventTag>,
    command_item::Action_EventTag,
    mavlink_write_event_to_log_t
>;

class Controller_WriteEventToLog : public BasicMavlinkController_ModuleKeyed<command_item::Action_EventTag>,
        public WriteToLogBroadcast
{

protected:

    void Construct_Broadcast(const command_item::Action_EventTag &obj, const MavlinkEntityKey &sender, mavlink_write_event_to_log_t &mavObj) override
    {
        mavObj.event_type = obj.get_EventTag();

        std::string writeString = "";
        //check if the string is too long, if so truncate.
        if(obj.get_LoggingText().length() > 50)
            writeString = obj.get_LoggingText().substr(0, 49);
        else
            writeString = obj.get_LoggingText();

        char *target;
        target = static_cast<char*>(malloc(writeString.length()));

        if (target == nullptr) /* check that nothing special happened to prevent tragedy  */
            return;

        strcpy(target, writeString.c_str());
        strcpy(mavObj.text, target);
        free(target);
    }


public:
    Controller_WriteEventToLog(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<command_item::Action_EventTag>(cb, queue, linkChan),
        WriteToLogBroadcast(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_write_event_to_log_t>(mavlink_msg_write_event_to_log_encode_chan))
    {

    }

private:


};

} //end of namespace MAVLINKVehicleControllers


#endif //WITH_HERON_MAVLINK_SUPPORT

#endif // CONTROLLER_WRITE_EVENT_TO_LOG_H
