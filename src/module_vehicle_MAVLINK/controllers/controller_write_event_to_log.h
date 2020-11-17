#ifndef CONTROLLER_WRITE_EVENT_TO_LOG_H
#define CONTROLLER_WRITE_EVENT_TO_LOG_H

#ifdef WITH_AI_SUPPORT

#include <mavlink.h>

#include "common/common.h"

#include "controllers/actions/action_broadcast.h"

#include "module_vehicle_MAVLINK/mavlink_entity_key.h"
#include "module_vehicle_MAVLINK/controllers/common.h"

namespace MAVLINKUXVControllers {

struct LogDetails
{
    AI_EVENT_TYPE type;
    std::string details;
};

using WriteToLogBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<LogDetails>,
    LogDetails,
    mavlink_set_attitude_target_t
>;

class Controller_WriteEventToLog : public BasicMavlinkController_ModuleKeyed<LogDetails>,
        public WriteToLogBroadcast
{

protected:

    void Construct_Broadcast(const LogDetails &obj, const MavlinkEntityKey &sender, mavlink_write_event_to_log_t &mavObj) override
    {
        mavObj.event_type = obj.type;
        std::string writeString = "";
        //check if the string is too long, if so truncate.
        if(obj.details.length() > 50)
            writeString = obj.details.substr(0, 49);
        else
            writeString = obj.details;

        char *target;
        target = static_cast<char*>(malloc(writeString.length()));

        if (target == nullptr) /* check that nothing special happened to prevent tragedy  */
            return false;

        strcpy(target, writeString.c_str());
        strcpy(mavObj.text, target);
        free(target);
    }


public:
    Controller_WriteEventToLog(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<LogDetails>(cb, queue, linkChan),
        WriteToLogBroadcast(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_write_event_to_log_t>(mavlink_msg_write_event_to_log_encode_chan))
    {

    }

private:


};

} //end of namespace MAVLINKVehicleControllers


#endif //WITH_AI_SUPPORT

#endif // CONTROLLER_WRITE_EVENT_TO_LOG_H
