#ifndef MAVLINK_CONTROLLER_COMMAND_WRITE_EVENT_TO_LOGS_H
#define MAVLINK_CONTROLLER_COMMAND_WRITE_EVENT_TO_LOGS_H

#include <iostream>

#include "mavlink.h"

#include "common/common.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_unsolicited_receive.h"

#include "controllers/controllers_MAVLINK/common.h"

#include "data_generic_command_item/mace/ai_items/action_event_tag.h"

namespace MAVLINKUXVControllers {

namespace ModuleController {

using ActionSend_EventTagBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::Action_EventTag>,
    command_item::Action_EventTag,
    mavlink_write_event_to_log_t
>;

using ActionReceive_EventTagBroadcast = Controllers::ActionUnsolicitedReceive<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::Action_EventTag>,
    MaceCore::ModuleCharacteristic,
    command_item::Action_EventTag,
    mavlink_write_event_to_log_t,
    MAVLINK_MSG_ID_WRITE_EVENT_TO_LOG
>;

class ControllerCommand_WriteEventToLog : public BasicExternalLinkController_ModuleKeyed<command_item::Action_EventTag>,
        public ActionSend_EventTagBroadcast,
        public ActionReceive_EventTagBroadcast
{

protected:

    void Construct_Broadcast(const command_item::Action_EventTag &obj, const MaceCore::ModuleCharacteristic &sender, mavlink_write_event_to_log_t &mavObj) override
    {
        UNUSED(sender);
        mavObj.event_type = obj.get_EventTag();
        mavObj.target_system = obj.getTargetSystem();

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

    bool Construct_FinalObject(const mavlink_write_event_to_log_t &msg, const MaceCore::ModuleCharacteristic &sender, MaceCore::ModuleCharacteristic &key, command_item::Action_EventTag &data) override
    {
        key = sender;
        command_item::Action_EventTag tag(static_cast<LOGGING_EVENT_TAGS>(msg.event_type), msg.text);
        tag.setTargetSystem(msg.target_system);
        tag.setOriginatingSystem(sender.MaceInstance);
        data = tag;
        return true;
    }


public:
    ControllerCommand_WriteEventToLog(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        BasicExternalLinkController_ModuleKeyed<command_item::Action_EventTag>(cb, queue, linkChan, "WriteEventToLog", false),
        ActionSend_EventTagBroadcast(this, ModuleToSysIDCompIDConverter<mavlink_write_event_to_log_t>(mavlink_msg_write_event_to_log_encode_chan)),
        ActionReceive_EventTagBroadcast(this, mavlink_msg_write_event_to_log_decode)
    {

    }

};

} //end of namespace ModuleController

} //end of namespace MAVLINKVehicleControllers

#endif // MAVLINK_CONTROLLER_COMMAND_WRITE_EVENT_TO_LOGS_H
