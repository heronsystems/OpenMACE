#ifndef CONTROLLER_GOTO_H
#define CONTROLLER_GOTO_H

#include <iostream>

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
    mace_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::CommandGoTo>,
    command_item::CommandGoTo,
    mace_command_goto_t
>;

using ActionSend_CommandGoTo_TargedWithResponse = Controllers::ActionSend<
    mace_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::CommandGoTo>,
    MaceCore::ModuleCharacteristic,
    command_item::CommandGoTo,
    mace_command_goto_t,
    MACE_MSG_ID_COMMAND_GOTO_ACK
>;

using ActionSend_CommandGoTo_ReceiveRespond = Controllers::ActionFinalReceiveRespond<
    mace_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::CommandGoTo>,
    MaceCore::ModuleCharacteristic,
    MaceCore::ModuleCharacteristic,
    command_item::CommandGoTo,
    mace_command_goto_t,
    mace_command_goto_ack_t,
    MACE_MSG_ID_COMMAND_GOTO
>;

using ActionFinish_CommandGoTo = Controllers::ActionFinish<
    mace_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::CommandGoTo>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mace_command_goto_ack_t,
    MACE_MSG_ID_COMMAND_GOTO_ACK
>;


class Controller_GoTo : public BasicExternalLinkController_ModuleKeyed<command_item::CommandGoTo>,
        public ActionSend_CommandGoTo_Broadcast,
        public ActionSend_CommandGoTo_TargedWithResponse,
        public ActionSend_CommandGoTo_ReceiveRespond,
        public ActionFinish_CommandGoTo
{

public:
    Controller_GoTo(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic>* cb, TransmitQueue * queue, int linkChan);

private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    void FillCommand(const command_item::CommandGoTo &commandItem, mace_command_goto_t &cmd) const;

    void BuildCommand(const mace_command_goto_t &message, command_item::CommandGoTo &data) const;


protected:

    virtual void Construct_Broadcast(const command_item::CommandGoTo &data, const MaceCore::ModuleCharacteristic &sender, mace_command_goto_t &cmd)
    {
        std::cout << "!!!WARNING!!!: Broadcasting a command. Commands should be targeted" << std::endl;

        cmd = initializeCommandGoTo();
        cmd.action = (uint16_t)data.getSpatialCommand()->getCommandType();
        cmd.target_system = 0;
        cmd.target_component = 0;

        FillCommand(data, cmd);
    }

    virtual bool Construct_Send(const command_item::CommandGoTo &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_command_goto_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        queueObj = target;

        cmd = initializeCommandGoTo();
        cmd.action = (uint16_t)data.getSpatialCommand()->getCommandType();
        cmd.target_system = target.MaceInstance;
        cmd.target_component = target.ModuleID;

        if(m_CommandRequestedFrom.find(target) != m_CommandRequestedFrom.cend())
        {
            printf("Command already issued, Ignoring\n");
            return false;
        }
        m_CommandRequestedFrom.insert({target, sender});

        FillCommand(data, cmd);

        return true;
    }


    virtual bool Construct_FinalObjectAndResponse(const mace_command_goto_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_command_goto_ack_t &ack, MaceCore::ModuleCharacteristic &key, command_item::CommandGoTo &data, MaceCore::ModuleCharacteristic &moduleFor, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        moduleFor.MaceInstance = msg.target_system;
        moduleFor.ModuleID = msg.target_component;

        queueObj = moduleFor;

        key = moduleFor;

        this->BuildCommand(msg,data);

        //ack.command = data.getSpatialCommand()->getCommandType();
        ack.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;

        //sending acknowledgement of the goTo command
        return true;
    }


    virtual bool Finish_Receive(const mace_command_goto_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
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

    mace_command_goto_t initializeCommandGoTo()
    {
        mace_command_goto_t cmdGoTo;
        cmdGoTo.action = 0;
        cmdGoTo.frame = 0;
        cmdGoTo.param1 = 0.0;
        cmdGoTo.param2 = 0.0;
        cmdGoTo.param3 = 0.0;
        cmdGoTo.param4 = 0.0;
        cmdGoTo.param5 = 0.0;
        cmdGoTo.param6 = 0.0;
        cmdGoTo.param7 = 0.0;
        cmdGoTo.target_system = 0;
        cmdGoTo.target_component = 0;
        return cmdGoTo;
    }

};

} //end of namespace ExternalLink

#endif // CONTROLLER_GOTO_H
