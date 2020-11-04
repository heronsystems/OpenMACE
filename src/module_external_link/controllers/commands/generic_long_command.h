#ifndef BASE_LONG_COMMAND_H
#define BASE_LONG_COMMAND_H

#include "controllers/generic_controller.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"

#include <iostream>
#include "data/command_ack_type.h"

#include "module_external_link/controllers/common.h"

namespace ExternalLink {


template <typename T>
using ActionSend_CommandLong_Broadcast = Controllers::ActionBroadcast<
    mace_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<T>,
    T,
    mace_command_long_t
>;

template <typename T>
using ActionSend_Command_TargedWithResponse = Controllers::ActionSend<
    mace_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<T>,
    MaceCore::ModuleCharacteristic,
    T,
    mace_command_long_t,
    MACE_MSG_ID_COMMAND_ACK
>;

template <typename T>
using ActionSend_Command_ReceiveRespond = Controllers::ActionFinalReceiveRespond<
    mace_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<T>,
    MaceCore::ModuleCharacteristic,
    MaceCore::ModuleCharacteristic,
    T,
    mace_command_long_t,
    mace_command_ack_t,
    MACE_MSG_ID_COMMAND_LONG
>;

template<typename T>
using ActionFinish_Command = Controllers::ActionFinish<
    mace_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<T>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mace_command_ack_t,
    MACE_MSG_ID_COMMAND_ACK
>;





template <typename COMMANDDATASTRUCTURE, const int COMMANDTYPE>
class Controller_GenericLongCommand : public BasicExternalLinkController_ModuleKeyed<COMMANDDATASTRUCTURE>,
        public ActionSend_CommandLong_Broadcast<COMMANDDATASTRUCTURE>,
        public ActionSend_Command_TargedWithResponse<COMMANDDATASTRUCTURE>,
        public ActionSend_Command_ReceiveRespond<COMMANDDATASTRUCTURE>,
        public ActionFinish_Command<COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mace_command_long_t &) const = 0;

    virtual void BuildCommand(const mace_command_long_t &, COMMANDDATASTRUCTURE &) const= 0;


protected:

    virtual void Construct_Broadcast(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, mace_command_long_t &cmd)
    {
        UNUSED(sender);

        std::cout << "!!!WARNING!!!: Broadcasting a command. Commands should be targeted" << std::endl;

        cmd = initializeCommandLong();
        cmd.command = COMMANDTYPE;
        cmd.target_system = 0;
        cmd.target_component = 0;

        FillCommand(data, cmd);
    }

    virtual bool Construct_Send(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_command_long_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        queueObj = target;

        cmd = initializeCommandLong();
        cmd.command = COMMANDTYPE;
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


    virtual bool Construct_FinalObjectAndResponse(const mace_command_long_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_command_ack_t &ack, MaceCore::ModuleCharacteristic &key, COMMANDDATASTRUCTURE &data, MaceCore::ModuleCharacteristic &moduleFor, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        moduleFor.MaceInstance = msg.target_system;
        moduleFor.ModuleID = msg.target_component;

        queueObj = moduleFor;

        if(msg.command != COMMANDTYPE)
        {
            return false;
        }

        key = moduleFor;
//        this-> template BuildCommand(msg, data);
        this->BuildCommand(msg, data);

        ack.command = COMMANDTYPE;
        ack.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;

        return true;
    }


    virtual bool Finish_Receive(const mace_command_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
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

    Controller_GenericLongCommand(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        BasicExternalLinkController_ModuleKeyed<COMMANDDATASTRUCTURE>(cb, queue, linkChan),
        ActionSend_CommandLong_Broadcast<COMMANDDATASTRUCTURE>(this, ModuleToSysIDCompIDConverter<mace_command_long_t>(mace_msg_command_long_encode_chan)),
        ActionSend_Command_TargedWithResponse<COMMANDDATASTRUCTURE>(this, ModuleToSysIDCompIDConverter<mace_command_long_t>(mace_msg_command_long_encode_chan)),
        ActionSend_Command_ReceiveRespond<COMMANDDATASTRUCTURE>(this, mace_msg_command_long_decode, ModuleToSysIDCompIDConverter<mace_command_ack_t>(mace_msg_command_ack_encode_chan)),
        ActionFinish_Command<COMMANDDATASTRUCTURE>(this, mace_msg_command_ack_decode)
    {

    }


    mace_command_long_t initializeCommandLong()
    {
        mace_command_long_t cmdLong;
        cmdLong.command = 0;
        cmdLong.confirmation = 0;
        cmdLong.param1 = 0.0;
        cmdLong.param2 = 0.0;
        cmdLong.param3 = 0.0;
        cmdLong.param4 = 0.0;
        cmdLong.param5 = 0.0;
        cmdLong.param6 = 0.0;
        cmdLong.param7 = 0.0;
        cmdLong.target_system = 0;
        cmdLong.target_component = 0;
        return cmdLong;
    }

};

}

#endif // BASE_LONG_COMMAND_H
