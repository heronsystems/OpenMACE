#ifndef MODULE_VEHICLE_MAVLINK_BASE_LONG_COMMAND_H
#define MODULE_VEHICLE_MAVLINK_BASE_LONG_COMMAND_H

#include "common/common.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"

#include "mavlink.h"
#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

#include "module_vehicle_MAVLINK/controllers/common.h"

namespace MAVLINKUXVControllers {

template <typename T>
using ActionSend_LongCommand_TargedWithResponse = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<T>,
    MavlinkEntityKey,
    T,
    mavlink_command_long_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;

template<typename T>
using ActionFinish_LongCommand = Controllers::ActionFinish<
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
        public ActionSend_LongCommand_TargedWithResponse<COMMANDDATASTRUCTURE>,
        public ActionFinish_LongCommand<COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MavlinkEntityKey, MavlinkEntityKey> m_CommandRequestedFrom;

protected:

    virtual void FillCommand(const COMMANDDATASTRUCTURE &, mavlink_command_long_t &) const = 0;

    virtual void BuildCommand(const mavlink_command_long_t &, COMMANDDATASTRUCTURE &) const= 0;

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
        UNUSED(msg);
        queueObj = sender;
        ack = msg.result;
        return true;
    }

public:

    Controller_GenericLongCommand(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<COMMANDDATASTRUCTURE>(cb, queue, linkChan),
        ActionSend_LongCommand_TargedWithResponse<COMMANDDATASTRUCTURE>(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_command_long_t>(mavlink_msg_command_long_encode_chan)),
        ActionFinish_LongCommand<COMMANDDATASTRUCTURE>(this, mavlink_msg_command_ack_decode)
    {

    }

    virtual ~Controller_GenericLongCommand() = default;


    mavlink_command_long_t initializeCommandLong()
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

};

}

#endif // MODULE_VEHICLE_MAVLINK_BASE_LONG_COMMAND_H
