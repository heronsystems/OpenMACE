#ifndef CONTROLLER_GUIDED_TARGET_ITEM_GLOBAL_H
#define CONTROLLER_GUIDED_TARGET_ITEM_GLOBAL_H

#include "common/common.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "mavlink.h"

#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

#include "module_vehicle_MAVLINK/controllers/common.h"

#include "data_generic_command_item/target_items/dynamic_target.h"

namespace MAVLINKVehicleControllers {

using namespace mace::pose;

struct TargetControllerStruct_Global
{
    uint8_t targetID;
    command_target::DynamicTarget target;
};

using GuidedTGTGlobalSend = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<TargetControllerStruct_Global>,
    MavlinkEntityKey,
    TargetControllerStruct_Global,
    mavlink_set_position_target_global_int_t,
    MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT
>;

using GuidedTGTGlobalFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<TargetControllerStruct_Global>,
    MavlinkEntityKey,
    uint8_t,
    mavlink_position_target_global_int_t,
    MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT
>;

class ControllerGuidedTargetItem_Global : public BasicMavlinkController_ModuleKeyed<TargetControllerStruct_Global>,
        public GuidedTGTGlobalSend,
        public GuidedTGTGlobalFinish
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual bool Construct_Send(const TargetControllerStruct_Global &commandItem, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_set_position_target_global_int_t &targetItem, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);
        queueObj = this->GetKeyFromSecondaryID(commandItem.targetID);

        targetItem = initializeMAVLINKTargetItem();
        targetItem.target_system = commandItem.targetID;
        targetItem.target_component = static_cast<uint8_t>(MaceCore::ModuleClasses::VEHICLE_COMMS);

        FillTargetItem(commandItem,targetItem);

        return true;
    }


    virtual bool Finish_Receive(const mavlink_position_target_global_int_t &msg, const MavlinkEntityKey &sender, uint8_t& ack, MavlinkEntityKey &queueObj)
    {
        std::cout<<"We have seen a position target global structure come in."<<std::endl;
        UNUSED(msg);
        queueObj = sender;
        ack = 0;
        return true;
    }

protected:
    void FillTargetItem(const TargetControllerStruct_Global &targetItem, mavlink_set_position_target_global_int_t &mavlinkItem);

    mavlink_set_position_target_global_int_t initializeMAVLINKTargetItem()
    {
        mavlink_set_position_target_global_int_t targetItem;
        targetItem.afx = 0.0;
        targetItem.afy = 0.0;
        targetItem.afz = 0.0;
        targetItem.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        targetItem.target_component =0;
        targetItem.target_system = 0;
        targetItem.time_boot_ms = 0;
        targetItem.type_mask = 65535; //by default we want to ignore all of the values
        targetItem.vx = 0.0;
        targetItem.vy = 0.0;
        targetItem.vz = 0.0;
        targetItem.lat_int = 0;
        targetItem.lon_int = 0;
        targetItem.yaw_rate = 0.0;
        targetItem.alt = 0.0;

        return targetItem;
    }

public:
    ControllerGuidedTargetItem_Global(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<TargetControllerStruct_Global>(cb, queue, linkChan),
        GuidedTGTGlobalSend(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_set_position_target_global_int_t>(mavlink_msg_set_position_target_global_int_encode_chan)),
        GuidedTGTGlobalFinish(this, mavlink_msg_position_target_global_int_decode)
    {

    }

};

} //end of namespace MAVLINKVehicleControllers


#endif // CONTROLLER_GUIDED_TARGET_ITEM_GLOBAL_H
