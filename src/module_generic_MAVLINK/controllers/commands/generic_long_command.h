#ifndef BASE_LONG_COMMAND_H
#define BASE_LONG_COMMAND_H

#include "../generic_controller.h"
#include "../generic_controller_queue_data_with_module.h"


#include "../actions/action_broadcast.h"
#include "../actions/action_send.h"
#include "../actions/action_final_receive_respond.h"
#include "../actions/action_finish.h"

namespace Controllers {


template <typename MESSAGETYPE, typename T>
using ActionSend_CommandLong_Broadcast = ActionBroadcast<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, T>,
    T,
    mace_command_long_t
>;

template <typename MESSAGETYPE, typename T>
using ActionSend_Command_TargedWithResponse = ActionSend<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, T>,
    MaceCore::ModuleCharacteristic,
    T,
    mace_command_long_t,
    MACE_MSG_ID_COMMAND_ACK
>;

template <typename MESSAGETYPE, typename T>
using ActionSend_Command_ReceiveRespond = ActionFinalReceiveRespond<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, T>,
    MaceCore::ModuleCharacteristic,
    T,
    mace_command_long_t,
    mace_command_ack_t,
    MACE_MSG_ID_COMMAND_LONG
>;

template<typename MESSAGETYPE, typename T>
using ActionFinish_Command = ActionFinish<
    MESSAGETYPE,
    GenericControllerQueueDataWithModule<MESSAGETYPE, T>,
    MaceCore::ModuleCharacteristic,
    mace_command_ack_t,
    MACE_MSG_ID_COMMAND_ACK
>;





template <typename MESSAGETYPE, typename COMMANDDATASTRUCTURE, const int COMMANDTYPE>
class Controller_GenericLongCommand : public GenericControllerQueueDataWithModule<MESSAGETYPE, COMMANDDATASTRUCTURE>,
        public ActionSend_CommandLong_Broadcast<MESSAGETYPE, COMMANDDATASTRUCTURE>,
        public ActionSend_Command_TargedWithResponse<MESSAGETYPE, COMMANDDATASTRUCTURE>,
        public ActionSend_Command_ReceiveRespond<MESSAGETYPE, COMMANDDATASTRUCTURE>,
        public ActionFinish_Command<MESSAGETYPE, COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mace_command_long_t &) const = 0;

    virtual void BuildCommand(const mace_command_long_t &, COMMANDDATASTRUCTURE &) const= 0;


protected:

    virtual void Construct_Broadcast(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, mace_command_long_t &cmd)
    {
        std::cout << "!!!WARNING!!!: Broadcasting a command. Commands should be targeted" << std::endl;

        cmd = initializeCommandLong();
        cmd.command = COMMANDTYPE;
        cmd.target_system = data.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        FillCommand(data, cmd);
    }

    virtual void Construct_Send(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_command_long_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        queueObj.ID = data.getTargetSystem();
        queueObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        cmd = initializeCommandLong();
        cmd.command = COMMANDTYPE;
        cmd.target_system = data.getTargetSystem();
        cmd.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        FillCommand(data, cmd);
    }


    virtual bool Construct_FinalObjectAndResponse(const mace_command_long_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_command_ack_t &ack, MaceCore::ModuleCharacteristic &key, COMMANDDATASTRUCTURE &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        vehicleObj.ID = msg.target_system;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        queueObj = vehicleObj;

        if(msg.command != COMMANDTYPE)
        {
            return false;
        }

        key = vehicleObj;
        this-> template BuildCommand(msg, data);

        ack.command = COMMANDTYPE;
        ack.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;

        return true;
    }


    virtual bool Finish_Receive(const mace_command_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        return true;
    }

public:

    Controller_GenericLongCommand(const IMessageNotifier<MESSAGETYPE> *cb, MessageModuleTransmissionQueue<MESSAGETYPE> *queue, int linkChan) :
        GenericControllerQueueDataWithModule<MESSAGETYPE, COMMANDDATASTRUCTURE>(cb, queue, linkChan),
        ActionSend_CommandLong_Broadcast<MESSAGETYPE, COMMANDDATASTRUCTURE>(this, mace_msg_command_long_encode_chan),
        ActionSend_Command_TargedWithResponse<MESSAGETYPE, COMMANDDATASTRUCTURE>(this, mace_msg_command_long_encode_chan),
        ActionSend_Command_ReceiveRespond<MESSAGETYPE, COMMANDDATASTRUCTURE>(this, mace_msg_command_long_decode, mace_msg_command_ack_encode_chan),
        ActionFinish_Command<MESSAGETYPE, COMMANDDATASTRUCTURE>(this, mace_msg_command_ack_decode)
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
