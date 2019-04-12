#ifndef CONTROLLER_SYSTEM_MODE_H
#define CONTROLLER_SYSTEM_MODE_H

#include "controllers/generic_controller.h"

#include "data_generic_command_item/do_items/action_change_mode.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"

#include "module_external_link/controllers/common.h"

namespace ExternalLink {

using CONTROLLER_SYSTEMMODE_TYPE = Controllers::GenericController<
    mace_message_t, MaceCore::ModuleCharacteristic,
    TransmitQueueWithKeys<MaceCore::ModuleCharacteristic, ObjectMaceMsgIDTuple<MaceCore::ModuleCharacteristic>>,
    uint8_t,
    Controllers::DataItem<MaceCore::ModuleCharacteristic, CommandItem::ActionChangeMode>
>;


using SystemModeSend = Controllers::ActionSend<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_SYSTEMMODE_TYPE,
    MaceCore::ModuleCharacteristic,
    CommandItem::ActionChangeMode,
    mace_command_system_mode_t,
    MACE_MSG_ID_SYSTEM_MODE_ACK
>;

using SystemMode_FinalReceiveRespond = Controllers::ActionFinalReceiveRespond<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_SYSTEMMODE_TYPE,
    MaceCore::ModuleCharacteristic,
    MaceCore::ModuleCharacteristic,
    CommandItem::ActionChangeMode,
    mace_command_system_mode_t,
    mace_system_mode_ack_t,
    MACE_MSG_ID_COMMAND_SYSTEM_MODE
>;

using SystemModeFinish = Controllers::ActionFinish<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_SYSTEMMODE_TYPE,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mace_system_mode_ack_t,
    MACE_MSG_ID_SYSTEM_MODE_ACK
>;


class ControllerSystemMode : public CONTROLLER_SYSTEMMODE_TYPE,

        public SystemModeSend,
        public SystemMode_FinalReceiveRespond,
        public SystemModeFinish
{


private:

    std::unordered_map<MaceCore::ModuleCharacteristic, bool> m_ActiveTransmits;


protected:


    virtual bool Construct_Send(const CommandItem::ActionChangeMode &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_command_system_mode_t &cmd, MaceCore::ModuleCharacteristic &queueObj);

    virtual bool Construct_FinalObjectAndResponse(const mace_command_system_mode_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_system_mode_ack_t &ack, MaceCore::ModuleCharacteristic &dataKey, CommandItem::ActionChangeMode &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj);

    virtual bool Finish_Receive(const mace_system_mode_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj);

public:

    ControllerSystemMode(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic>* cb, TransmitQueue * queue, int linkChan);

};

}

#endif // CONTROLLER_SYSTEM_MODE_H
