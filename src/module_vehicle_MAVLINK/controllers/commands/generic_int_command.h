#ifndef GENERIC_INT_COMMAND_H
#define GENERIC_INT_COMMAND_H

#include "common/common.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"

#include "mavlink.h"

#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

namespace MAVLINKVehicleControllers {

template <typename T>
using ActionSend_IntCommand_TargedWithResponse = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<T>,
    MavlinkEntityKey,
    T,
    mavlink_command_int_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;

template<typename T>
using ActionFinish_IntCommand = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<T>,
    MavlinkEntityKey,
    uint8_t,
    mavlink_command_ack_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;


template <typename COMMANDDATASTRUCTURE, const int COMMANDTYPE>
class Controller_GenericIntCommand : public BasicMavlinkController_ModuleKeyed<COMMANDDATASTRUCTURE>,
        public ActionSend_IntCommand_TargedWithResponse<COMMANDDATASTRUCTURE>,
        public ActionFinish_IntCommand<COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MavlinkEntityKey, MavlinkEntityKey> m_CommandRequestedFrom;

public:

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mavlink_command_int_t &) const = 0;

    virtual void BuildCommand(const mavlink_command_int_t &, COMMANDDATASTRUCTURE &) const= 0;

protected:

    virtual bool Construct_Send(const COMMANDDATASTRUCTURE &data, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_command_int_t &cmd, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);
        queueObj = this->GetKeyFromSecondaryID(data.getTargetSystem());

        cmd = initializeCommandInt();
        cmd.command = COMMANDTYPE;
        cmd.target_system = data.getTargetSystem();
        cmd.target_component = 0;

        FillCommand(data, cmd);

        return true;
    }


    virtual bool Finish_Receive(const mavlink_command_ack_t &msg, const MavlinkEntityKey &sender, uint8_t & ack, MavlinkEntityKey &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = msg.result;
        return true;
    }

public:

    Controller_GenericIntCommand(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<COMMANDDATASTRUCTURE>(cb, queue, linkChan),
        ActionSend_IntCommand_TargedWithResponse<COMMANDDATASTRUCTURE>(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_command_int_t>(mavlink_msg_command_int_encode_chan)),
        ActionFinish_IntCommand<COMMANDDATASTRUCTURE>(this, mavlink_msg_command_ack_decode)
    {

    }


    mavlink_command_int_t initializeCommandInt()
    {
        mavlink_command_int_t cmdInt;
        cmdInt.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        cmdInt.command = 0;
        cmdInt.current = 1;
        cmdInt.autocontinue = 1;
        cmdInt.param1 = 0.0;
        cmdInt.param2 = 0.0;
        cmdInt.param3 = 0.0;
        cmdInt.param4 = 0.0;
        cmdInt.x = 0;
        cmdInt.y = 0;
        cmdInt.z = 0;
        cmdInt.target_system = 0;
        cmdInt.target_component = 0;
        return cmdInt;
    }

};

}

#endif // GENERIC_INT_COMMAND_H
