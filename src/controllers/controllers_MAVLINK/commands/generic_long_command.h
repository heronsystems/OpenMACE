#ifndef MAVLINK_CONTROLLER_BASE_LONG_COMMAND_H
#define MAVLINK_CONTROLLER_BASE_LONG_COMMAND_H

#include "mavlink.h"
#include "common/common.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"

#include "controllers/controllers_MAVLINK/common.h"

namespace MAVLINKUXVControllers {

inline mavlink_command_long_t initializeCommandLong()
{
    mavlink_command_long_t cmdLong;
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

//!
//! THE FOLLOWING CONTROLLERS ARE USED TO COMMUNICATE WITH AN EXPLICIT VEHICLE
//!
template <typename T>
using ActionVehicleSend_LongCommand_TargetedWithResponse = Controllers::ActionSend<
mavlink_message_t,
MavlinkEntityKey,
BasicMavlinkController_ModuleKeyed<T>,
MavlinkEntityKey,
T,
mavlink_command_long_t,
MAVLINK_MSG_ID_COMMAND_ACK
>;

template<typename T>
using ActionVehicleFinish_LongCommand = Controllers::ActionFinish<
mavlink_message_t,
MavlinkEntityKey,
BasicMavlinkController_ModuleKeyed<T>,
MavlinkEntityKey,
uint8_t,
mavlink_command_ack_t,
MAVLINK_MSG_ID_COMMAND_ACK
>;

template <typename COMMANDDATASTRUCTURE, const int COMMANDTYPE>
class Controller_GenericLongCommand : public BasicMavlinkController_ModuleKeyed<COMMANDDATASTRUCTURE>,
        public ActionVehicleSend_LongCommand_TargetedWithResponse<COMMANDDATASTRUCTURE>,
        public ActionVehicleFinish_LongCommand<COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_FromModuleRequest;

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mavlink_command_long_t &) const = 0;

    virtual void BuildCommand(const mavlink_command_long_t &, COMMANDDATASTRUCTURE &) const= 0;

    //!
    //! THE FOLLOWING CONTROLLERS ARE USED TO COMMUNICATE WITH AN EXPLICIT VEHICLE
    //!

protected:

    virtual bool Construct_Send(const COMMANDDATASTRUCTURE &data, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_command_long_t &cmd, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);
        queueObj = this->GetKeyFromSecondaryID(data.getTargetSystem());

        cmd = initializeCommandLong();
        cmd.command = COMMANDTYPE;
        cmd.target_system = data.getTargetSystem();
        cmd.target_component = 0;

        FillCommand(data, cmd);

        return true;
    }


    virtual bool Finish_Receive(const mavlink_command_ack_t &msg, const MavlinkEntityKey &sender, uint8_t & ack, MavlinkEntityKey &queueObj)
    {
        if(msg.command != COMMANDTYPE)
        {
            return false;
        }
        queueObj = sender;
        ack = msg.result;
        return true;
    }

public:

    Controller_GenericLongCommand(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan, std::string controllerName) :
        BasicMavlinkController_ModuleKeyed<COMMANDDATASTRUCTURE>(cb, queue, linkChan, controllerName),
        ActionVehicleSend_LongCommand_TargetedWithResponse<COMMANDDATASTRUCTURE>(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_command_long_t>(mavlink_msg_command_long_encode_chan)),
        ActionVehicleFinish_LongCommand<COMMANDDATASTRUCTURE>(this, mavlink_msg_command_ack_decode)
    {

    }

    virtual ~Controller_GenericLongCommand() = default;

};

//!
//! THE FOLLOWING CONTROLLERS ARE USED TO COMMUNICATE BETWEEN MACE MODULES
//!
template <typename T>
using ActionModuleSend_CommandLong_Broadcast = Controllers::ActionBroadcast<
mavlink_message_t, MaceCore::ModuleCharacteristic,
BasicExternalLinkController_ModuleKeyed<T>,
T,
mavlink_command_long_t
>;

template <typename T>
using ActionModuleSend_Command_TargedWithResponse = Controllers::ActionSend<
mavlink_message_t,
MaceCore::ModuleCharacteristic,
BasicExternalLinkController_ModuleKeyed<T>,
MaceCore::ModuleCharacteristic,
T,
mavlink_command_long_t,
MAVLINK_MSG_ID_COMMAND_ACK
>;

template <typename T>
using ActionModuleSend_Command_ReceiveRespond = Controllers::ActionFinalReceiveRespond<
mavlink_message_t, MaceCore::ModuleCharacteristic,
BasicExternalLinkController_ModuleKeyed<T>,
MaceCore::ModuleCharacteristic,
MaceCore::ModuleCharacteristic,
T,
mavlink_command_long_t,
mavlink_command_ack_t,
MAVLINK_MSG_ID_COMMAND_LONG
>;

template<typename T>
using ActionModuleFinish_Command = Controllers::ActionFinish<
mavlink_message_t, MaceCore::ModuleCharacteristic,
BasicExternalLinkController_ModuleKeyed<T>,
MaceCore::ModuleCharacteristic,
uint8_t,
mavlink_command_ack_t,
MAVLINK_MSG_ID_COMMAND_ACK
>;

template <typename COMMANDDATASTRUCTURE, const int COMMANDTYPE>
class ModuleController_GenericLongCommand : public BasicExternalLinkController_ModuleKeyed<COMMANDDATASTRUCTURE>,
        public ActionModuleSend_CommandLong_Broadcast<COMMANDDATASTRUCTURE>,
        public ActionModuleSend_Command_TargedWithResponse<COMMANDDATASTRUCTURE>,
        public ActionModuleSend_Command_ReceiveRespond<COMMANDDATASTRUCTURE>,
        public ActionModuleFinish_Command<COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_FromModuleRequest;

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mavlink_command_long_t &) const = 0;

    virtual void BuildCommand(const mavlink_command_long_t &, COMMANDDATASTRUCTURE &) const= 0;

    //!
    //! THE FOLLOWING CONTROLLERS ARE USED TO COMMUNICATE BETWEEN MACE MODULES
    //!
protected:

    virtual void Construct_Broadcast(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, mavlink_command_long_t &cmd)
    {
        UNUSED(sender);

        cmd = initializeCommandLong();
        cmd.command = COMMANDTYPE;
        cmd.target_system = 0;
        cmd.target_component = 0;

        FillCommand(data, cmd);
    }

    virtual bool Construct_Send(const COMMANDDATASTRUCTURE &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_command_long_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        queueObj = target;

        cmd = initializeCommandLong();
        cmd.command = COMMANDTYPE;
        cmd.target_system = target.MaceInstance;
        cmd.target_component = target.ModuleID;

        if(m_FromModuleRequest.find(target) != m_FromModuleRequest.cend())
        {
            printf("Generic long command already issued, Ignoring\n");
            return false;
        }
        std::cout << "Insert sender: " << sender.MaceInstance << " / " << sender.ModuleID << std::endl;
        std::cout << "Insert target: " << target.MaceInstance << " / " << target.ModuleID << std::endl;
        m_FromModuleRequest.insert({target, sender});

        FillCommand(data, cmd);

        return true;
    }


    virtual bool Construct_FinalObjectAndResponse(const mavlink_command_long_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_command_ack_t &ack, MaceCore::ModuleCharacteristic &dataKey, COMMANDDATASTRUCTURE &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);

        if(msg.command != COMMANDTYPE)
        {
            return false;
        }

        vehicleObj = this->GetKeyFromSecondaryID(msg.target_system);
        queueObj = vehicleObj;
        dataKey = vehicleObj;
        this->BuildCommand(msg, data);
        ack.command = COMMANDTYPE;
        ack.result = MAV_CMD_ACK::MAV_CMD_ACK_OK;
        return true;
    }


    virtual bool Finish_Receive(const mavlink_command_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        if(msg.command == COMMANDTYPE && m_FromModuleRequest.find(sender) != m_FromModuleRequest.cend())
        {
            UNUSED(msg);
            queueObj = sender;
            ack = msg.result;
            m_FromModuleRequest.erase(sender);
            return true;
        }
        else
        {
            return false;
        }
    }

public:

    ModuleController_GenericLongCommand(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan, std::string controllerName) :
        BasicExternalLinkController_ModuleKeyed<COMMANDDATASTRUCTURE>(cb, queue, linkChan, controllerName, false),
        ActionModuleSend_CommandLong_Broadcast<COMMANDDATASTRUCTURE>(this, ModuleToSysIDCompIDConverter<mavlink_command_long_t>(mavlink_msg_command_long_encode_chan)),
        ActionModuleSend_Command_TargedWithResponse<COMMANDDATASTRUCTURE>(this, ModuleToSysIDCompIDConverter<mavlink_command_long_t>(mavlink_msg_command_long_encode_chan)),
        ActionModuleSend_Command_ReceiveRespond<COMMANDDATASTRUCTURE>(this, mavlink_msg_command_long_decode, ModuleToSysIDCompIDConverter<mavlink_command_ack_t>(mavlink_msg_command_ack_encode_chan)),
        ActionModuleFinish_Command<COMMANDDATASTRUCTURE>(this, mavlink_msg_command_ack_decode)
    {

    }

    virtual ~ModuleController_GenericLongCommand() = default;


};

} //end of namepsace ModuleController






#endif // MAVLINK_CONTROLLER_BASE_LONG_COMMAND_H
