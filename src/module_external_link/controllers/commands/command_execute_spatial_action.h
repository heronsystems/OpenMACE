#ifndef CONTROLLER_GOTO_H
#define CONTROLLER_GOTO_H

#include <iostream>
#include <mavlink.h>

#include "controllers/generic_controller.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"

#include "data/command_ack_type.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"


#include "module_external_link/controllers/common.h"

namespace ExternalLink {

using ActionSend_CommandGoTo_Broadcast = Controllers::ActionBroadcast<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::Action_ExecuteSpatialItem>,
    command_item::Action_ExecuteSpatialItem,
    mavlink_execute_spatial_action_t
>;

using ActionSend_CommandGoTo_TargedWithResponse = Controllers::ActionSend<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::Action_ExecuteSpatialItem>,
    MaceCore::ModuleCharacteristic,
    command_item::Action_ExecuteSpatialItem,
    mavlink_execute_spatial_action_t,
    MAVLINK_MSG_ID_EXECUTE_SPATIAL_ACTION_ACK
>;

using ActionSend_CommandGoTo_ReceiveRespond = Controllers::ActionFinalReceiveRespond<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::Action_ExecuteSpatialItem>,
    MaceCore::ModuleCharacteristic,
    MaceCore::ModuleCharacteristic,
    command_item::Action_ExecuteSpatialItem,
    mavlink_execute_spatial_action_t,
    mavlink_execute_spatial_action_ack_t,
    MAVLINK_MSG_ID_EXECUTE_SPATIAL_ACTION
>;

using ActionFinish_CommandGoTo = Controllers::ActionFinish<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::Action_ExecuteSpatialItem>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mavlink_execute_spatial_action_ack_t,
    MAVLINK_MSG_ID_EXECUTE_SPATIAL_ACTION_ACK
>;


class Controller_GoTo : public BasicExternalLinkController_ModuleKeyed<command_item::Action_ExecuteSpatialItem>,
        public ActionSend_CommandGoTo_Broadcast,
        public ActionSend_CommandGoTo_TargedWithResponse,
        public ActionSend_CommandGoTo_ReceiveRespond,
        public ActionFinish_CommandGoTo
{

public:
    Controller_GoTo(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic>* cb, TransmitQueue * queue, int linkChan);

private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    void FillCommand(const command_item::Action_ExecuteSpatialItem &commandItem, mavlink_execute_spatial_action_t &cmd) const;

    void BuildCommand(const mavlink_execute_spatial_action_t &message, command_item::Action_ExecuteSpatialItem &data) const;


protected:

    virtual void Construct_Broadcast(const command_item::Action_ExecuteSpatialItem &data, const MaceCore::ModuleCharacteristic &sender, mavlink_execute_spatial_action_t &cmd)
    {
        UNUSED(sender);

        std::cout << "!!!WARNING!!!: Broadcasting a command. Commands should be targeted" << std::endl;

        cmd = initializeSpatialAction();
        cmd.action = (uint16_t)data.getSpatialAction()->getCommandType();
        cmd.target_system = 0;
        cmd.target_component = 0;

        FillCommand(data, cmd);
    }

    virtual bool Construct_Send(const command_item::Action_ExecuteSpatialItem &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_execute_spatial_action_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        queueObj = target;

        cmd = initializeSpatialAction();
        cmd.action = static_cast<uint16_t>(data.getSpatialAction()->getCommandType());
        cmd.target_system = static_cast<uint8_t>(target.MaceInstance);
        cmd.target_component = static_cast<uint8_t>(target.ModuleID);

        if(m_CommandRequestedFrom.find(target) != m_CommandRequestedFrom.cend())
        {
            printf("Command already issued, Ignoring\n");
            return false;
        }
        m_CommandRequestedFrom.insert({target, sender});

        FillCommand(data, cmd);

        return true;
    }


    virtual bool Construct_FinalObjectAndResponse(const mavlink_execute_spatial_action_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_execute_spatial_action_ack_t &ack, MaceCore::ModuleCharacteristic &key, command_item::Action_ExecuteSpatialItem &data, MaceCore::ModuleCharacteristic &moduleFor, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        moduleFor.MaceInstance = msg.target_system;
        moduleFor.ModuleID = msg.target_component;

        queueObj = moduleFor;

        key = moduleFor;

        this->BuildCommand(msg,data);

        //ack.command = data.getSpatialCommand()->getCommandType();
        ack.result = MAV_CMD_ACK::MAV_CMD_ACK_OK;

        //sending acknowledgement of the goTo command
        return true;
    }


    virtual bool Finish_Receive(const mavlink_execute_spatial_action_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        if(m_CommandRequestedFrom.find(sender) != m_CommandRequestedFrom.cend())
        {
            UNUSED(msg);
            queueObj = sender;
            ack = msg.result;
            m_CommandRequestedFrom.erase(sender);
            return true;
        }
        else
        {
            return false;
        }
    }

public:

    mavlink_execute_spatial_action_t initializeSpatialAction()
    {
        mavlink_execute_spatial_action_t cmdSpatialAction;
        cmdSpatialAction.action = 0;
        cmdSpatialAction.frame = 0;
        cmdSpatialAction.param1 = 0.0;
        cmdSpatialAction.param2 = 0.0;
        cmdSpatialAction.param3 = 0.0;
        cmdSpatialAction.param4 = 0.0;
        cmdSpatialAction.param5 = 0.0;
        cmdSpatialAction.param6 = 0.0;
        cmdSpatialAction.param7 = 0.0;
        cmdSpatialAction.target_system = 0;
        cmdSpatialAction.target_component = 0;
        return cmdSpatialAction;
    }

};

} //end of namespace ExternalLink

#endif // CONTROLLER_GOTO_H
