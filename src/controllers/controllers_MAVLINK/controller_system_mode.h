#ifndef MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H
#define MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H
#include "mavlink.h"
#include "common/common.h"
#include "data_generic_command_item/do_items/action_change_mode.h"
#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"
#include "controllers/controllers_MAVLINK/common.h"

namespace MAVLINKUXVControllers {

namespace VehicleController {

struct VehicleMode_Struct
{
    uint8_t targetID;
    uint8_t vehicleMode;
};

using SystemModeSend = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<VehicleMode_Struct>,
    MavlinkEntityKey,
    VehicleMode_Struct,
    mavlink_set_mode_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;
using SystemModeFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<VehicleMode_Struct>,
    MavlinkEntityKey,
    uint8_t,
    mavlink_command_ack_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;

class ControllerSystemMode : public BasicMavlinkController_ModuleKeyed<VehicleMode_Struct>,
        public SystemModeSend,
        public SystemModeFinish
{
private:
    std::unordered_map<int, int> m_CommandRequestedFrom;
protected:
    virtual bool Construct_Send(const VehicleMode_Struct &commandMode, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_set_mode_t &mode, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        queueObj = target;
        mode.target_system = commandMode.targetID;
        mode.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        mode.custom_mode = commandMode.vehicleMode;
        return true;
    }

    virtual bool Finish_Receive(const mavlink_command_ack_t &msg, const MavlinkEntityKey &sender, uint8_t & ack, MavlinkEntityKey &queueObj)
    {
        if(msg.command != MAVLINK_MSG_ID_SET_MODE)
        {
            return false;
        }

        queueObj = sender;
        ack = msg.result;
        return true;
    }

public:
    ControllerSystemMode(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<VehicleMode_Struct>(cb, queue, linkChan, "SystemMode"),
        SystemModeSend(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_set_mode_t>(mavlink_msg_set_mode_encode_chan)),
        SystemModeFinish(this, mavlink_msg_command_ack_decode)
    {
    }
    virtual ~ControllerSystemMode() = default;
};
} //end of namepsace VehicleController
namespace ModuleController {

using CONTROLLER_SYSTEMMODE_TYPE = Controllers::GenericController<
    mavlink_message_t, 
    MaceCore::ModuleCharacteristic,
    TransmitQueueWithKeys<MaceCore::ModuleCharacteristic, ObjectMaceMsgIDTuple<MaceCore::ModuleCharacteristic>>,
    uint8_t,
    Controllers::DataItem<MaceCore::ModuleCharacteristic, command_item::ActionChangeMode>
>;
using ActionModule_SystemModeSend = Controllers::ActionSend<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    CONTROLLER_SYSTEMMODE_TYPE,
    MaceCore::ModuleCharacteristic,
    command_item::ActionChangeMode,
    mavlink_mace_set_vehicle_mode_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;
using ActionModule_SystemMode_FinalReceiveRespond = Controllers::ActionFinalReceiveRespond<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    CONTROLLER_SYSTEMMODE_TYPE,
    MaceCore::ModuleCharacteristic,
    MaceCore::ModuleCharacteristic,
    command_item::ActionChangeMode,
    mavlink_mace_set_vehicle_mode_t,
    mavlink_command_ack_t,
    MAVLINK_MSG_ID_MACE_SET_VEHICLE_MODE
>;
using ActionModule_SystemModeFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MaceCore::ModuleCharacteristic,
    CONTROLLER_SYSTEMMODE_TYPE,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mavlink_command_ack_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;
class ControllerSystemMode : public CONTROLLER_SYSTEMMODE_TYPE,
        public ActionModule_SystemModeSend,
        public ActionModule_SystemMode_FinalReceiveRespond,
        public ActionModule_SystemModeFinish
{
private:
    std::unordered_map<MaceCore::ModuleCharacteristic, bool> m_ActiveTransmits;
protected:
    virtual bool Construct_Send(const command_item::ActionChangeMode &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_mace_set_vehicle_mode_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        queueObj = target;
        if(m_ActiveTransmits.find(target) != m_ActiveTransmits.cend() && m_ActiveTransmits.at(target) == true)
        {
            std::cout << "System Mode is already being changed for this target, ignoring" << std::endl;
            return false;
        }
        m_ActiveTransmits.insert({target, true});
        for(size_t i = 0 ; i < sizeof(cmd.mode)/sizeof(*cmd.mode) ; i++)
        {
            cmd.mode[i] = 0;
        }
        cmd.target_system = commandItem.getTargetSystem();
        strcpy(cmd.mode, commandItem.getRequestMode().c_str());
        return true;
    }
    virtual bool Construct_FinalObjectAndResponse(const mavlink_mace_set_vehicle_mode_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_command_ack_t &ack, MaceCore::ModuleCharacteristic &dataKey, command_item::ActionChangeMode &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        vehicleObj = this->GetKeyFromSecondaryID(msg.target_system);
        queueObj = vehicleObj;
        dataKey = vehicleObj;
        data.setTargetSystem(msg.target_system);
        data.setRequestMode(std::string(msg.mode));
        ack.result = MAV_CMD_ACK::MAV_CMD_ACK_OK;
        ack.command = MAVLINK_MSG_ID_MACE_SET_VEHICLE_MODE;

	std::cout << "Received command system mode: " << std::string(msg.mode) << std::endl;
        return true;
    }
    virtual bool Finish_Receive(const mavlink_command_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        if(msg.command != MAVLINK_MSG_ID_MACE_SET_VEHICLE_MODE)
        {
            return false;
        }

        queueObj = sender;
        ack = msg.result;
        m_ActiveTransmits.erase(sender);
        return true;
    }
public:
    ControllerSystemMode(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue * queue, int linkChan) :
        CONTROLLER_SYSTEMMODE_TYPE(cb, queue, linkChan, "SystemMode", false),
        ActionModule_SystemModeSend(this, ModuleToSysIDCompIDConverter<mavlink_mace_set_vehicle_mode_t>(mavlink_msg_mace_set_vehicle_mode_encode_chan)),
        ActionModule_SystemMode_FinalReceiveRespond(this, mavlink_msg_mace_set_vehicle_mode_decode, ModuleToSysIDCompIDConverter<mavlink_command_ack_t>(mavlink_msg_command_ack_encode_chan)),
        ActionModule_SystemModeFinish(this, mavlink_msg_command_ack_decode)
    {
    }
};
} //end of namespace ModuleController
} //end of namespace MAVLINKVehicleControllers
#endif // MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H
