#ifndef MAVLINK_CONTROLLER_TEST_PROCEDURAL_H
#define MAVLINK_CONTROLLER_TEST_PROCEDURAL_H

#include <iostream>

#include "mavlink.h"

#include "common/common.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_unsolicited_receive.h"

#include "controllers/controllers_MAVLINK/common.h"

#include "data_generic_command_item/mace/ai_items/action_procedural_command.h"

namespace MAVLINKUXVControllers {

namespace ModuleController {

using ActionSend_TestProceduralBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::Action_ProceduralCommand>,
    command_item::Action_ProceduralCommand,
    mavlink_ai_execute_procedural_t
>;

//Receive a broadcasted test procedural event position, accept and finish (no response)
using ActionReceive_TestProceduralBroadcast = Controllers::ActionUnsolicitedReceive<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    BasicExternalLinkController_ModuleKeyed<command_item::Action_ProceduralCommand>,
    MaceCore::ModuleCharacteristic,
    command_item::Action_ProceduralCommand,
    mavlink_ai_execute_procedural_t,
    MAVLINK_MSG_ID_AI_EXECUTE_PROCEDURAL
>;

class Command_TestProcedural : public BasicExternalLinkController_ModuleKeyed<command_item::Action_ProceduralCommand>,
        public ActionSend_TestProceduralBroadcast,
        public ActionReceive_TestProceduralBroadcast
{

protected:

    void Construct_Broadcast(const command_item::Action_ProceduralCommand &cmd, const MaceCore::ModuleCharacteristic &sender, mavlink_ai_execute_procedural_t &procedural) override
    {
        UNUSED(sender);
        cmd.populateMACECOMMS_ExecuteProcedural(procedural);
    }

    bool Construct_FinalObject(const mavlink_ai_execute_procedural_t &msg, const MaceCore::ModuleCharacteristic &sender, MaceCore::ModuleCharacteristic &key, command_item::Action_ProceduralCommand &data) override
    {
        UNUSED(sender);
        UNUSED(key);
        std::cout<<"A new test procedural command had been received."<<std::endl;
        data.fromMACECOMMS_ExecuteProcedural(msg);
        return true;
    }

public:
    Command_TestProcedural(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        BasicExternalLinkController_ModuleKeyed<command_item::Action_ProceduralCommand>(cb, queue, linkChan, "TestProcedural", false),
        ActionSend_TestProceduralBroadcast(this, ModuleToSysIDCompIDConverter<mavlink_ai_execute_procedural_t>(mavlink_msg_ai_execute_procedural_encode_chan)),
        ActionReceive_TestProceduralBroadcast(this, mavlink_msg_ai_execute_procedural_decode)
    {

    }

};

} //end of namespace ModuleController

} //end of namespace MAVLINKVehicleControllers

#endif // MAVLINK_CONTROLLER_TEST_PROCEDURAL_H
