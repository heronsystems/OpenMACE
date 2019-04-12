#ifndef CONTROLLER_GUIDED_TARGET_ITEM_GLOBAL_H
#define CONTROLLER_GUIDED_TARGET_ITEM_GLOBAL_H

#include "common/common.h"

#include "data_generic_command_item/target_items/dynamic_target_list.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "base/pose/geodetic_position_3D.h"

#include "mavlink.h"

#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

#include "module_vehicle_MAVLINK/controllers/common.h"


namespace MAVLINKVehicleControllers {

using namespace mace::pose;

struct TargetControllerStructGlobal
{
    uint8_t targetID;
    TargetItem::GeodeticDynamicTarget target;
};

template <typename T>
using GuidedTGTGlobalSend = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<T>,
    MaceCore::ModuleCharacteristic,
    T,
    mavlink_set_position_target_global_int_t,
    MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT
>;

template <typename T>
using GuidedTGTGlobalFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<T>,
    MaceCore::ModuleCharacteristic,
    uint8_t,
    mavlink_position_target_global_int_t,
    MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT
>;

template <typename TARGETITEM>
class ControllerGuidedTargetItem_Global : public BasicMavlinkController_ModuleKeyed<TARGETITEM>,
        public GuidedTGTGlobalSend<TARGETITEM>,
        public GuidedTGTGlobalFinish<TARGETITEM>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

protected:

    virtual bool Construct_Send(const TARGETITEM &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_set_position_target_global_int_t &targetItem, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);
        queueObj = commandItem.targetID;

        targetItem = initializeMAVLINKTargetItem();
        targetItem.target_system = commandItem.targetID;
        targetItem.target_component = (int)MaceCore::ModuleClasses::VEHICLE_COMMS;

        FillTargetItem(commandItem,targetItem);

        return true;
    }


    virtual bool Finish_Receive(const mavlink_position_target_global_int_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t& ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        std::cout<<"We has finished recieving the message."<<std::endl;
        UNUSED(msg);
        queueObj = sender;
        ack = 0;
        return true;
    }

protected:
    void FillTargetItem(const TargetControllerStructGlobal &targetItem, mavlink_set_position_target_global_int_t &mavlinkItem);

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
        BasicMavlinkController_ModuleKeyed<TARGETITEM>(cb, queue, linkChan),
        GuidedTGTGlobalSend<TARGETITEM>(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_set_position_target_global_int_t>(mavlink_msg_set_position_target_global_int_encode_chan)),
        GuidedTGTGlobalFinish<TARGETITEM>(this, mavlink_msg_position_target_global_int_decode)
    {

    }

};

} //end of namespace MAVLINKVehicleControllers


#endif // CONTROLLER_GUIDED_TARGET_ITEM_GLOBAL_H
