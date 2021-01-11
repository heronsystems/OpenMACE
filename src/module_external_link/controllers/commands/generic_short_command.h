#ifndef BASE_SHORT_COMMAND_H
#define BASE_SHORT_COMMAND_H

#include <mavlink.h>

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
using ActionSend_CommandShort_Broadcast = Controllers::ActionBroadcast<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<T>,
    T,
    mavlink_command_short_t
>;


template <typename T>
using ActionSend_CommandShort_TargedWithResponse = Controllers::ActionSend<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<T>,
    MaceCore::ModuleCharacteristic,
    T,
    mavlink_command_short_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;

template <typename T>
using ActionSend_CommandShort_ReceiveRespond = Controllers::ActionFinalReceiveRespond<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<T>,
    MaceCore::ModuleCharacteristic,
    MaceCore::ModuleCharacteristic,
    T,
    mavlink_command_short_t,
    mavlink_command_ack_t,
    MAVLINK_MSG_ID_COMMAND_SHORT
>;

template<typename T>
using ActionFinish_CommandShort = Controllers::ActionFinish<
    mavlink_message_t, MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<T>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mavlink_command_ack_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;





template <typename COMMANDDATASTRUCTURE, const int COMMANDTYPE>
class Controller_GenericShortCommand : public BasicExternalLinkController_ModuleKeyed<COMMANDDATASTRUCTURE>,
        public ActionSend_CommandShort_Broadcast<COMMANDDATASTRUCTURE>,
        public ActionSend_CommandShort_TargedWithResponse<COMMANDDATASTRUCTURE>,
        public ActionSend_CommandShort_ReceiveRespond<COMMANDDATASTRUCTURE>,
        public ActionFinish_CommandShort<COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mavlink_command_short_t &) const = 0;

    virtual void BuildCommand(const mavlink_command_short_t &, COMMANDDATASTRUCTURE &) const= 0;


protected:

    virtual void Construct_Broadcast(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, mavlink_command_short_t &cmd)
    {
        UNUSED(sender);

        std::cout << "!!!WARNING!!!: Broadcasting a command. Commands should be targeted" << std::endl;

//        MaceCore::ModuleCharacteristic target = this->GetKeyFromSecondaryID(data.getTargetSystem());

        cmd = initializeCommandShort();
        cmd.command = COMMANDTYPE;
        cmd.target_system = 0;
        cmd.target_component = 0;

        FillCommand(data, cmd);
    }

    virtual bool Construct_Send(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_command_short_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        queueObj = this->GetKeyFromSecondaryID(data.getTargetSystem());

        cmd = initializeCommandShort();
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


    virtual bool Construct_FinalObjectAndResponse(const mavlink_command_short_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_command_ack_t &ack, MaceCore::ModuleCharacteristic &key, COMMANDDATASTRUCTURE &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        vehicleObj.MaceInstance = msg.target_system;
        vehicleObj.ModuleID = msg.target_component;

        queueObj = vehicleObj;

        if(msg.command != COMMANDTYPE)
        {
            return false;
        }

        key = vehicleObj;
//        this-> template BuildCommand(msg, data);
        this->BuildCommand(msg, data);

        ack.command = COMMANDTYPE;
        ack.result = (uint8_t)MAV_CMD_ACK::MAV_CMD_ACK_OK;

        return true;
    }


    virtual bool Finish_Receive(const mavlink_command_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
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

    Controller_GenericShortCommand(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        BasicExternalLinkController_ModuleKeyed<COMMANDDATASTRUCTURE>(cb, queue, linkChan),
        ActionSend_CommandShort_Broadcast<COMMANDDATASTRUCTURE>(this, ModuleToSysIDCompIDConverter<mavlink_command_short_t>(mavlink_msg_command_short_encode_chan)),
        ActionSend_CommandShort_TargedWithResponse<COMMANDDATASTRUCTURE>(this, ModuleToSysIDCompIDConverter<mavlink_command_short_t>(mavlink_msg_command_short_encode_chan)),
        ActionSend_CommandShort_ReceiveRespond<COMMANDDATASTRUCTURE>(this, mavlink_msg_command_short_decode, ModuleToSysIDCompIDConverter<mavlink_command_ack_t>(mavlink_msg_command_ack_encode_chan)),
        ActionFinish_CommandShort<COMMANDDATASTRUCTURE>(this, mavlink_msg_command_ack_decode)
    {

    }


    mavlink_command_short_t initializeCommandShort()
    {
        mavlink_command_short_t cmdShort;
        cmdShort.command = 0;
        cmdShort.confirmation = 0;
        cmdShort.param1 = 0.0;
        return cmdShort;
    }

};

}

#endif // BASE_SHORT_COMMAND_H
